// monitor's receiver summary (see monitor-summary.h)
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "monitor-summary.h"

#define BILLION_NS 1000000000LL

static int64_t bucket_of(int64_t t){
  return t / (SUM_BUCKET_S * BILLION_NS);
}
// A bucket id still within the ten minutes to now
static bool in_window(int64_t id,int64_t now){
  int64_t const now_id = bucket_of(now);
  return id != 0 && id > now_id - SUM_BUCKETS && id <= now_id;
}

static struct sum_frontend *find(struct rx_summary *s,char const *key){
  struct sum_frontend *spare = NULL;
  for(int k = 0; k < SUM_FRONTENDS; k++){
    struct sum_frontend *f = &s->fe[k];
    if(f->used && strcmp(f->now.key,key) == 0)
      return f;
    if(!f->used && spare == NULL)
      spare = f;
  }
  if(spare == NULL)
    return NULL; // more front ends than kept: the rest aren't summarised
  memset(spare,0,sizeof *spare);
  spare->used = true;
  snprintf(spare->now.key,sizeof spare->now.key,"%s",key);
  return spare;
}

void summary_reading(struct rx_summary *s,struct sum_reading const *r){
  if(r->clocktime <= 0)
    return; // no time: can't place it
  pthread_mutex_lock(&s->lock);
  struct sum_frontend * const f = find(s,r->key);
  if(f == NULL || r->clocktime <= f->now.clocktime){
    pthread_mutex_unlock(&s->lock);
    return; // nothing new (every channel's status repeats the front end's)
  }
  if(f->first == 0)
    f->first = r->clocktime;
  // Overloads since the last reading: its increase, or (radiod restarted,
  // its count starting again) all it has counted since
  uint64_t overs = !f->have_prev ? 0
    : r->overranges >= f->now.overranges ? r->overranges - f->now.overranges : r->overranges;
  bool const counted_up = f->have_prev ? overs > 0 : r->overranges > 0;
  f->now = *r;
  f->have_prev = true;
  if(counted_up && r->samprate > 0)
    f->last_over = r->clocktime - (int64_t)((double)r->samp_since_over / r->samprate * BILLION_NS);
  // After a gap in status, the overloads it counts may all be older than
  // the window: if even the last of them was, none belong in it
  if(overs > 0 && f->last_over != 0 && r->clocktime - f->last_over >= (int64_t)SUM_WINDOW_S * BILLION_NS)
    overs = 0;
  int64_t const id = bucket_of(r->clocktime);
  int const i = (int)(id % SUM_BUCKETS);
  if(f->bucket[i].id != id){
    f->bucket[i].id = id;
    f->bucket[i].peak = r->if_dbfs;
    f->bucket[i].peak_time = r->clocktime;
    f->bucket[i].overs = 0;
  } else if(r->if_dbfs > f->bucket[i].peak){
    f->bucket[i].peak = r->if_dbfs;
    f->bucket[i].peak_time = r->clocktime;
  }
  f->bucket[i].overs += overs;
  pthread_mutex_unlock(&s->lock);
}

void summary_audio(struct rx_summary *s,int slot,uint32_t ssrc,double seconds,int64_t now){
  if(slot < 0 || slot >= SUM_CHANNELS || !(seconds > 0))
    return;
  pthread_mutex_lock(&s->lock);
  struct sum_channel * const c = &s->chan[slot];
  if(c->ssrc != ssrc){
    pthread_mutex_unlock(&s->lock);
    return; // not the stream the slot was last made for: not counted
  }
  int64_t const id = bucket_of(now);
  int const i = (int)(id % SUM_BUCKETS);
  if(c->bucket[i].id != id){
    c->bucket[i].id = id;
    c->bucket[i].seconds = 0;
  }
  c->bucket[i].seconds += (float)seconds;
  pthread_mutex_unlock(&s->lock);
}

void summary_channel_reset(struct rx_summary *s,int slot,uint32_t ssrc){
  if(slot < 0 || slot >= SUM_CHANNELS)
    return;
  pthread_mutex_lock(&s->lock);
  memset(&s->chan[slot],0,sizeof s->chan[slot]);
  s->chan[slot].ssrc = ssrc;
  pthread_mutex_unlock(&s->lock);
}

void summary_channel_label(struct rx_summary *s,int slot,uint32_t ssrc,double freq,char const *label){
  if(slot < 0 || slot >= SUM_CHANNELS)
    return;
  pthread_mutex_lock(&s->lock);
  struct sum_channel * const c = &s->chan[slot];
  if(c->ssrc == ssrc){
    c->freq = freq;
    snprintf(c->label,sizeof c->label,"%s",label != NULL ? label : "");
  }
  pthread_mutex_unlock(&s->lock);
}

bool summary_view(struct rx_summary *s,int k,int64_t now,struct sum_view *v){
  if(k < 0 || k >= SUM_FRONTENDS)
    return false;
  pthread_mutex_lock(&s->lock);
  struct sum_frontend const * const f = &s->fe[k];
  if(!f->used){
    pthread_mutex_unlock(&s->lock);
    return false;
  }
  memset(v,0,sizeof *v);
  v->now = f->now;
  v->peak = -INFINITY;
  for(int i = 0; i < SUM_BUCKETS; i++){
    if(!in_window(f->bucket[i].id,now))
      continue; // unused, or older than ten minutes (by monitor's clock: they age even if status stops)
    if(f->bucket[i].peak > v->peak){
      v->peak = f->bucket[i].peak;
      v->peak_time = f->bucket[i].peak_time;
    }
    v->recent_overs += f->bucket[i].overs;
  }
  v->whole_window = now - f->first >= (int64_t)SUM_WINDOW_S * BILLION_NS;
  v->last_over = f->last_over;
  v->recent_over = f->last_over != 0 && now - f->last_over < (int64_t)SUM_WINDOW_S * BILLION_NS;
  pthread_mutex_unlock(&s->lock);
  return true;
}

static double Sort_seconds[SUM_CHANNELS]; // (under the lock)
static int by_seconds(void const *a,void const *b){
  double const x = Sort_seconds[*(int const *)a], y = Sort_seconds[*(int const *)b];
  return x > y ? -1 : x < y ? +1 : *(int const *)a - *(int const *)b;
}
int summary_busiest(struct rx_summary *s,struct sum_busy *busy,int n,int64_t now){
  static int all[SUM_CHANNELS]; // (under the lock)
  pthread_mutex_lock(&s->lock);
  int count = 0;
  for(int i = 0; i < SUM_CHANNELS; i++){
    double sum = 0;
    for(int b = 0; b < SUM_BUCKETS; b++)
      if(in_window(s->chan[i].bucket[b].id,now))
	sum += s->chan[i].bucket[b].seconds;
    Sort_seconds[i] = sum;
    if(sum >= 0.5) // anything to speak of
      all[count++] = i;
  }
  qsort(all,(size_t)count,sizeof all[0],by_seconds);
  if(count > n)
    count = n;
  for(int i = 0; i < count; i++){
    struct sum_channel const * const c = &s->chan[all[i]];
    busy[i].slot = all[i];
    busy[i].seconds = Sort_seconds[all[i]];
    busy[i].freq = c->freq;
    snprintf(busy[i].label,sizeof busy[i].label,"%s",c->label);
  }
  pthread_mutex_unlock(&s->lock);
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

void summary_frontend_line(struct sum_view const *v,int64_t now,char *out,size_t size){
  struct sum_reading const * const r = &v->now;
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
    summary_ago(now - v->peak_time,ago,sizeof ago);
    snprintf(peak,sizeof peak," (peak %.1f, %s ago)",v->peak,ago);
  }
  char over[96];
  if(r->overranges == 0 && v->recent_overs == 0)
    snprintf(over,sizeof over," | A/D over 0");
  else {
    char ago[16] = "?";
    if(v->last_over != 0)
      summary_ago(now - v->last_over,ago,sizeof ago);
    snprintf(over,sizeof over," | A/D over %llu, %s%llu, last %s ago",(unsigned long long)r->overranges,
	     v->whole_window ? "10m: " : "here: ",(unsigned long long)v->recent_overs,ago);
  }
  char age[16];
  summary_ago(now - r->clocktime,age,sizeof age);
  snprintf(out,size,"%s%s%.4f MHz | IF %.1f dBFS%s%s%s | %s ago",
	   r->description[0] ? r->description : "",r->description[0] ? " " : "",
	   r->frequency * 1e-6,r->if_dbfs,peak,gain,over,age);
}
