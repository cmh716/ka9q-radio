// monitor's receiver summary (see monitor-summary.h)
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "monitor-summary.h"

#define BILLION_NS 1000000000LL

static struct sum_frontend *find(struct rx_summary *s,char const *description){
  char const *d = description != NULL ? description : "";
  struct sum_frontend *spare = NULL;
  for(int k = 0; k < SUM_FRONTENDS; k++){
    struct sum_frontend *f = &s->fe[k];
    if(f->used && strcmp(f->description,d) == 0)
      return f;
    if(!f->used && spare == NULL)
      spare = f;
  }
  if(spare == NULL)
    return NULL; // more front ends than kept: the rest aren't summarised
  memset(spare,0,sizeof *spare);
  spare->used = true;
  snprintf(spare->description,sizeof spare->description,"%s",d);
  return spare;
}

void summary_reading(struct rx_summary *s,struct sum_reading const *r){
  if(r->clocktime <= 0)
    return; // no time: can't place it
  struct sum_frontend * const f = find(s,r->description);
  if(f == NULL || r->clocktime <= f->now.clocktime)
    return; // nothing new (every channel's status repeats the front end's)
  if(f->first == 0)
    f->first = r->clocktime;
  f->now = *r;
  f->now.description = f->description;
  int64_t const id = r->clocktime / (SUM_BUCKET_S * BILLION_NS);
  int const i = (int)(id % SUM_BUCKETS);
  if(f->bucket[i].id != id){
    f->bucket[i].id = id;
    f->bucket[i].peak = r->if_dbfs;
    f->bucket[i].peak_time = r->clocktime;
    f->bucket[i].over_start = r->overranges;
  } else if(r->if_dbfs > f->bucket[i].peak){
    f->bucket[i].peak = r->if_dbfs;
    f->bucket[i].peak_time = r->clocktime;
  }
}

void summary_activity(struct rx_summary *s,int slot,uint32_t ssrc,bool active,int64_t now){
  if(slot < 0 || slot >= SUM_CHANNELS)
    return;
  struct sum_channel * const c = &s->chan[slot];
  if(c->ssrc != ssrc || c->updated == 0 || now < c->updated){
    memset(c,0,sizeof *c);
    c->ssrc = ssrc;
    c->updated = now;
    return;
  }
  double const dt = (double)(now - c->updated) / BILLION_NS;
  c->updated = now;
  if(!active || dt > SUM_BUCKET_S)
    return; // (a gap that long: not known to have been audio throughout)
  int64_t const id = now / (SUM_BUCKET_S * BILLION_NS);
  int const i = (int)(id % SUM_BUCKETS);
  if(c->bucket[i].id != id){
    c->bucket[i].id = id;
    c->bucket[i].seconds = 0;
  }
  c->bucket[i].seconds += (float)dt;
}

double summary_seconds(struct rx_summary const *s,int slot,int64_t now){
  if(slot < 0 || slot >= SUM_CHANNELS)
    return 0;
  struct sum_channel const * const c = &s->chan[slot];
  int64_t const now_id = now / (SUM_BUCKET_S * BILLION_NS);
  double sum = 0;
  for(int i = 0; i < SUM_BUCKETS; i++)
    if(c->bucket[i].id != 0 && c->bucket[i].id > now_id - SUM_BUCKETS && c->bucket[i].id <= now_id)
      sum += c->bucket[i].seconds;
  return sum;
}

bool summary_view(struct rx_summary const *s,int k,struct sum_view *v){
  if(k < 0 || k >= SUM_FRONTENDS || !s->fe[k].used)
    return false;
  struct sum_frontend const * const f = &s->fe[k];
  memset(v,0,sizeof *v);
  v->now = &f->now;
  v->peak = -INFINITY;
  int64_t const now_id = f->now.clocktime / (SUM_BUCKET_S * BILLION_NS);
  int64_t oldest = 0;
  uint64_t oldest_overs = f->now.overranges;
  for(int i = 0; i < SUM_BUCKETS; i++){
    int64_t const id = f->bucket[i].id;
    if(id == 0 || id <= now_id - SUM_BUCKETS || id > now_id)
      continue; // unused, or older than ten minutes
    if(f->bucket[i].peak > v->peak){
      v->peak = f->bucket[i].peak;
      v->peak_time = f->bucket[i].peak_time;
    }
    if(oldest == 0 || id < oldest){
      oldest = id;
      oldest_overs = f->bucket[i].over_start;
    }
  }
  // Overloads since the oldest bucket's start (a restart of radiod resets
  // its count: then all it has counted since)
  v->recent_overs = f->now.overranges >= oldest_overs ? f->now.overranges - oldest_overs : f->now.overranges;
  v->whole_window = f->now.clocktime - f->first >= (int64_t)SUM_WINDOW_S * BILLION_NS;
  if(f->now.overranges > 0 && f->now.samprate > 0)
    v->last_over = f->now.clocktime - (int64_t)((double)f->now.samp_since_over / f->now.samprate * BILLION_NS);
  return true;
}

static double Sort_seconds[SUM_CHANNELS]; // (the display thread's alone)
static int by_seconds(void const *a,void const *b){
  double const x = Sort_seconds[*(int const *)a], y = Sort_seconds[*(int const *)b];
  return x > y ? -1 : x < y ? +1 : *(int const *)a - *(int const *)b;
}
int summary_busiest(struct rx_summary const *s,int *slots,int n,int64_t now){
  int all[SUM_CHANNELS];
  int count = 0;
  for(int i = 0; i < SUM_CHANNELS; i++){
    if(s->chan[i].updated == 0)
      continue;
    Sort_seconds[i] = summary_seconds(s,i,now);
    if(Sort_seconds[i] >= 0.5) // anything to speak of
      all[count++] = i;
  }
  qsort(all,(size_t)count,sizeof all[0],by_seconds);
  if(count > n)
    count = n;
  memcpy(slots,all,(size_t)count * sizeof all[0]);
  return count;
}

void summary_ago(int64_t ns,char *out,size_t size){
  if(ns < 0)
    ns = 0;
  int64_t const s = ns / BILLION_NS;
  if(s < 60)
    snprintf(out,size,"%llds",(long long)s);
  else if(s < 3600)
    snprintf(out,size,"%lldm",(long long)(s / 60));
  else if(s < 86400)
    snprintf(out,size,"%lldh",(long long)(s / 3600));
  else
    snprintf(out,size,"%lldd",(long long)(s / 86400));
}

void summary_frontend_line(struct sum_view const *v,int64_t wall_now,char *out,size_t size){
  struct sum_reading const * const r = v->now;
  char gain[64] = "";
  if(r->lna_gain != 0 || r->mixer_gain != 0 || r->if_gain != 0)
    snprintf(gain,sizeof gain," | gain LNA %d mix %d IF %d",r->lna_gain,r->mixer_gain,r->if_gain);
  else if(isfinite(r->rf_gain))
    snprintf(gain,sizeof gain," | gain %.0f dB",r->rf_gain);
  if(r->agc)
    strncat(gain," AGC",sizeof gain - strlen(gain) - 1);
  char peak[48] = "";
  if(isfinite(v->peak)){
    char ago[16];
    summary_ago(r->clocktime - v->peak_time,ago,sizeof ago);
    snprintf(peak,sizeof peak," (peak %.1f, %s ago)",v->peak,ago);
  }
  char over[96];
  if(r->overranges == 0)
    snprintf(over,sizeof over," | A/D over 0");
  else {
    char ago[16] = "?";
    if(v->last_over != 0)
      summary_ago(r->clocktime - v->last_over,ago,sizeof ago);
    snprintf(over,sizeof over," | A/D over %llu, %s%llu, last %s ago",(unsigned long long)r->overranges,
	     v->whole_window ? "10m: " : "here: ",(unsigned long long)v->recent_overs,ago);
  }
  char age[16];
  summary_ago(wall_now - r->clocktime,age,sizeof age);
  snprintf(out,size,"%s%s%.4f MHz | IF %.1f dBFS%s%s%s | %s ago",
	   r->description[0] ? r->description : "",r->description[0] ? " " : "",
	   r->frequency * 1e-6,r->if_dbfs,peak,gain,over,age);
}
