// P25 trunking: a control channel's grants steer a second downconverter and
// voice decoder (Phase 1 FDMA or Phase 2 TDMA) on the same channel thread,
// so one radiod channel plays whichever permitted call is up
// Copyright 2026 ka9q-radio contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#define _GNU_SOURCE 1
#include <assert.h>
#include <complex.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "misc.h"
#include "radio.h"
#include "nco.h"
#include "dv.h"
#include "dv_follow.h"

extern int Overlap; // fast convolution overlap factor (radio.c)

#define FOLLOW_GRANTS 16
#define FOLLOW_FRESH (DV_IQ_SAMPRATE_RADIOD * 3 / 2)   // grants or updates this recent are live calls
#define FOLLOW_HANG (DV_IQ_SAMPRATE_RADIOD * 2)        // no voice this long: the call is over
#define FOLLOW_LISTEN (DV_IQ_SAMPRATE_RADIOD * 3)      // give a new channel this long to produce voice

struct follow_grant {
  bool valid;
  uint64_t freq;
  int slot;                  // TDMA slot, -1 for FDMA
  uint32_t group, source;
  bool unit;
  uint64_t seen;             // sample count of the last grant or update
};

struct dv_follow {
  chan_t *chan;
  dv_pcm_callback pcm;
  void *pcm_user;
  struct follow_grant grants[FOLLOW_GRANTS];
  uint64_t now;              // samples processed

  // Second downconverter, always run so it stays in step with the main one
  struct filter_out out;
  bool have_out;
  int bin_shift;
  uint64_t phase_adjust;
  uint64_t fine_phase;

  // The call being followed
  dv_decoder *voice;
  struct follow_grant cur;
  uint64_t started, last_voice, last_count;
  uint64_t skip_freq;        // the call we last left: don't rejoin it while it's re-announced
  int skip_slot;
  uint32_t skip_group;
  uint64_t calls;            // calls followed
};

// Position in the talkgroup list (lower is better), or -1 if not wanted
static int priority(struct dv_follow const *f, struct follow_grant const *g){
  chan_t const *chan = f->chan;
  if(chan->p25.ntalkgroups == 0)
    return 0; // everything, first come first served
  if(g->unit)
    return -1;
  for(int i = 0; i < chan->p25.ntalkgroups; i++)
    if(chan->p25.talkgroups[i] == g->group)
      return i;
  return -1;
}

struct dv_follow *dv_follow_create(chan_t *chan, dv_pcm_callback pcm, void *pcm_user){
  struct dv_follow *f = calloc(1,sizeof *f);
  if(f == NULL)
    return NULL;
  f->chan = chan;
  f->pcm = pcm;
  f->pcm_user = pcm_user;
  f->bin_shift = -1000000; // force the phase bookkeeping on the first block
  int const blocksize = (int)lrint(DV_IQ_SAMPRATE_RADIOD * Blocktime);
  if(create_filter_output(&f->out,&chan->frontend->in,blocksize,COMPLEX) != 0){
    free(f);
    return NULL;
  }
  f->have_out = true;
  // Same response as the control channel's
  int const samprate = DV_IQ_SAMPRATE_RADIOD;
  float const nyquist = samprate / 2.0f;
  float const lower = max(chan->filter.min_IF, -nyquist);
  float const upper = min(chan->filter.max_IF, nyquist);
  set_filter(&f->out,lower/samprate,upper/samprate,chan->filter.kaiser_beta);
  return f;
}

static void release(struct dv_follow *f, char const *why){
  if(f->voice == NULL)
    return;
  if(Verbose > 0){
    char const *name = f->chan->name;
    uint32_t const group = f->cur.group;
    double const mhz = f->cur.freq * 1e-6;
    dv_stats st;
    dv_get_stats(f->voice,&st);
    fprintf(stderr,"%s P25 leaving TG %" PRIu32 " on %.4f MHz (%s; %" PRIu64 " voice frames, %" PRIu64 " muted)\n",
	    name,group,mhz,why,st.voice_decoded,st.voice_muted);
  }
  dv_destroy(f->voice);
  f->voice = NULL;
  if(strcmp(why,"preempted") != 0){ // a preempted call may be rejoined later
    f->skip_freq = f->cur.freq;
    f->skip_slot = f->cur.slot;
    f->skip_group = f->cur.group;
  }
}

void dv_follow_destroy(struct dv_follow *f){
  if(f == NULL)
    return;
  release(f,"stopped");
  if(f->have_out)
    delete_filter_output(&f->out);
  free(f);
}

void dv_follow_event(struct dv_follow *f, dv_event const *e){
  if(f == NULL || (e->type != DV_EVENT_GRANT && e->type != DV_EVENT_UPDATE))
    return;
  if(e->freq_hz == 0 || e->encrypted)
    return; // band plan unknown yet, or nothing to hear
  struct follow_grant g = {
    .valid = true, .freq = e->freq_hz, .slot = e->slot,
    .group = e->group, .source = e->source, .unit = e->unit, .seen = f->now,
  };
  if(priority(f,&g) < 0)
    return;
  // A new grant of the call we left is a new call
  if(e->type == DV_EVENT_GRANT && g.freq == f->skip_freq && g.slot == f->skip_slot && g.group == f->skip_group)
    f->skip_freq = 0;
  // One entry per voice channel and slot
  int slot = -1, oldest = 0;
  for(int i = 0; i < FOLLOW_GRANTS; i++){
    struct follow_grant const *o = &f->grants[i];
    if(o->valid && o->freq == g.freq && o->slot == g.slot){
      slot = i;
      break;
    }
    if(!o->valid || o->seen < f->grants[oldest].seen)
      oldest = i;
  }
  f->grants[slot >= 0 ? slot : oldest] = g;
  // A talker change on the call we follow
  if(f->voice != NULL && g.freq == f->cur.freq && g.slot == f->cur.slot && g.group == f->cur.group)
    f->cur.seen = f->now;
}

// The best live grant, or NULL
static struct follow_grant const *best_grant(struct dv_follow const *f){
  struct follow_grant const *best = NULL;
  int best_pri = 0;
  for(int i = 0; i < FOLLOW_GRANTS; i++){
    struct follow_grant const *g = &f->grants[i];
    if(!g->valid || f->now - g->seen > FOLLOW_FRESH)
      continue;
    if(g->freq == f->skip_freq && g->slot == f->skip_slot && g->group == f->skip_group)
      continue;
    int const p = priority(f,g);
    if(p < 0)
      continue;
    if(best == NULL || p < best_pri || (p == best_pri && g->seen > best->seen)){
      best = g;
      best_pri = p;
    }
  }
  return best;
}

static void join(struct dv_follow *f, struct follow_grant const *g, dv_stats const *cc){
  chan_t const *chan = f->chan;
  dv_config config = {
    .p25_nac = cc->last_nac,
    .p25_wacn = cc->wacn,
    .p25_sysid = cc->sysid,
    .p25_slot = g->slot < 0 ? 0 : g->slot,
  };
  dv_mode mode;
  if(g->slot >= 0){
    if(cc->wacn == 0)
      return; // can't descramble until the control channel names the network
    mode = DV_P25P2;
  } else
    mode = chan->p25.modulation == P25_CQPSK ? DV_P25_CQPSK : DV_P25_C4FM;
  f->voice = dv_create(mode,&config,f->pcm,f->pcm_user);
  if(f->voice == NULL)
    return;
  f->cur = *g;
  f->started = f->last_voice = f->now;
  f->last_count = 0;
  f->calls++;
  if(Verbose > 0){
    char where[24] = "";
    if(g->slot >= 0)
      snprintf(where,sizeof where," slot %d",g->slot);
    fprintf(stderr,"%s P25 following %s %" PRIu32 " on %.4f MHz%s\n",chan->name,
	    g->unit ? "unit" : "TG",g->group,g->freq * 1e-6,where);
  }
}

// The second downconverter: downconvert()'s tuning, without sweeping or filter2
static float complex const *tune_block(struct dv_follow *f, double freq, int *count){
  pthread_mutex_lock(&Frontend.status_mutex);
  double const offset = freq - Frontend.frequency;
  int shift = 0;
  double remainder = 0;
  int const r = compute_tuning(Frontend.in.ilen + Frontend.in.impulse_length - 1,
			       Frontend.samprate,&shift,&remainder,offset);
  pthread_mutex_unlock(&Frontend.status_mutex);
  if(r != 0)
    shift = 0; // out of the front end's range: keep step, discard the block
  execute_filter_output(&f->out,shift);
  float complex * const output = f->out.output.c;
  int const olen = f->out.olen;
  if(r != 0 || output == NULL)
    return NULL;
  double const samptime = Blocktime / olen;
  uint64_t const increment = set_nco(-(remainder * samptime));
  if(shift != f->bin_shift){
    int m = shift % Overlap;
    if(m < 0)
      m += Overlap;
    f->phase_adjust = (uint64_t)(((__uint128_t)m << 64) / Overlap);
    m = (f->bin_shift - shift) % (2*Overlap);
    if(m < 0)
      m += 2*Overlap;
    f->fine_phase += (uint64_t)(((__uint128_t)m << 64) / (2*Overlap));
    f->bin_shift = shift;
  }
  f->fine_phase += f->phase_adjust;
  f->fine_phase = mixdown(output,olen,f->fine_phase,increment);
  *count = olen;
  return output;
}

void dv_follow_block(struct dv_follow *f, dv_stats const *cc, int count){
  f->now += (uint64_t)count;
  double const freq = f->voice != NULL ? (double)f->cur.freq : f->chan->tune.freq;
  int n = 0;
  float complex const *iq = tune_block(f,freq,&n);

  if(f->voice != NULL){
    if(iq != NULL)
      dv_process(f->voice,(float const *)iq,(size_t)n);
    dv_stats st;
    dv_get_stats(f->voice,&st);
    uint64_t const heard = st.voice_decoded + st.voice_muted;
    if(heard != f->last_count){
      f->last_count = heard;
      f->last_voice = f->now;
    }
    struct follow_grant const *b = best_grant(f);
    if(st.m17_encrypted && st.voice_decoded == 0)
      release(f,"encrypted");
    else if(f->last_voice == f->started && f->now - f->started > FOLLOW_LISTEN)
      release(f,"no voice");
    else if(f->last_voice != f->started && f->now - f->last_voice > FOLLOW_HANG)
      release(f,"call ended");
    else if(b != NULL && priority(f,b) < priority(f,&f->cur)
	    && !(b->freq == f->cur.freq && b->slot == f->cur.slot))
      release(f,"preempted");
  }
  if(f->voice == NULL){
    struct follow_grant const *g = best_grant(f);
    if(g != NULL)
      join(f,g,cc);
  }
  if(f->skip_freq != 0){
    // Forget the skip once its call is gone from the control channel
    bool live = false;
    for(int i = 0; i < FOLLOW_GRANTS; i++){
      struct follow_grant const *g = &f->grants[i];
      if(g->valid && g->freq == f->skip_freq && g->slot == f->skip_slot && g->group == f->skip_group
	 && f->now - g->seen <= FOLLOW_FRESH)
	live = true;
    }
    if(!live)
      f->skip_freq = 0;
  }
}

bool dv_follow_source(struct dv_follow const *f, char *buf, size_t len){
  if(f == NULL || f->voice == NULL)
    return false;
  // Talkgroup and voice channel: the talker is in the recordings and the -v log
  snprintf(buf,len,"TG%" PRIu32 " %.4f",f->cur.group,f->cur.freq * 1e-6);
  return true;
}

void dv_follow_print(struct dv_follow const *f){
  if(f == NULL || f->voice == NULL)
    return;
  dv_stats st;
  dv_get_stats(f->voice,&st);
  fprintf(stderr,"%s P25 voice %.4f MHz TG %" PRIu32 " src %s syncs=%" PRIu64 " frames=%" PRIu64
	  " voice=%" PRIu64 " muted=%" PRIu64 "%s errors=%.1f%% offset=%+.1f Hz\n",
	  f->chan->name,f->cur.freq * 1e-6,f->cur.group,st.src_call[0] ? st.src_call : "-",
	  st.syncs,st.frames,st.voice_decoded,st.voice_muted,st.m17_encrypted ? " encrypted" : "",
	  100.0f * st.error_rate,st.freq_offset);
}

bool dv_follow_call(struct dv_follow const *f, char *dest, size_t dlen, char *src, size_t slen){
  if(f == NULL || f->voice == NULL)
    return false;
  dv_stats st;
  dv_get_stats(f->voice,&st);
  snprintf(dest,dlen,"%" PRIu32,f->cur.group);
  if(st.src_call[0] != '\0')
    strlcpy(src,st.src_call,slen);
  else if(f->cur.source != 0)
    snprintf(src,slen,"%" PRIu32,f->cur.source);
  else
    src[0] = '\0';
  return true;
}
