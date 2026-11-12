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
#define FOLLOW_HANG (DV_IQ_SAMPRATE_RADIOD * 2)        // no voice this long (and no end seen): the call is over
#define FOLLOW_LISTEN (DV_IQ_SAMPRATE_RADIOD * 3)      // give a new channel this long to produce voice
#define FOLLOW_REGRANT (DV_IQ_SAMPRATE_RADIOD * 3 / 10) // voice this long after a grant of our channel: it was ours
#define FOLLOW_LINGER (DV_IQ_SAMPRATE_RADIOD)          // after an end with such a grant unplaced: wait for voice this long

struct follow_grant {
  bool valid;
  uint64_t freq;
  int slot;                  // TDMA slot, -1 for FDMA
  uint32_t group, source;
  bool unit;
  uint64_t seen;             // sample count of the last grant or update
  uint64_t granted;          // sample count of the last grant (not update): a new transmission
  uint32_t transmission;     // the control channel's number for its explicit grant, 0 if unseen
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
  uint32_t pending;          // a later grant number for our own channel and talkgroup, not yet placed
  uint64_t pending_at;       // when it came
  bool lingering;            // stayed past an end to see whether that grant brings voice
  uint64_t voice_serial;     // the voice decoder's call_serial when we joined
  uint64_t skip_freq;        // the call we last left: don't rejoin it while it's re-announced
  int skip_slot;
  uint32_t skip_group;
  uint32_t skip_source;
  uint32_t skip_transmission;
  uint32_t last_group;       // talkgroup of the call last followed: the conversation
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
  f->last_group = f->cur.group;
  if(strcmp(why,"preempted") != 0){ // a preempted call may be rejoined later
    f->skip_freq = f->cur.freq;
    f->skip_slot = f->cur.slot;
    f->skip_group = f->cur.group;
    f->skip_source = f->cur.source;
    f->skip_transmission = f->cur.transmission;
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
    .group = e->group, .source = e->source, .unit = e->unit, .seen = f->now, .granted = f->now,
    .transmission = e->transmission,
  };
  if(priority(f,&g) < 0)
    return;
  // A new grant of the call we left (even to the same talker on the same
  // channel), or another talker on it, is a new transmission
  if(g.freq == f->skip_freq && g.slot == f->skip_slot && g.group == f->skip_group
     && (e->type == DV_EVENT_GRANT || g.transmission > f->skip_transmission
	 || (g.source != 0 && g.source != f->skip_source)))
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
  // An update keeps the time of the grant it repeats
  if(slot >= 0 && e->type == DV_EVENT_UPDATE && f->grants[slot].group == g.group)
    g.granted = f->grants[slot].granted;
  f->grants[slot >= 0 ? slot : oldest] = g;
  // The call we follow: still live, and perhaps a new talker (the voice
  // channel's own source ID takes precedence when it has one)
  if(f->voice != NULL && g.freq == f->cur.freq && g.slot == f->cur.slot && g.group == f->cur.group){
    f->cur.seen = f->now;
    if(g.source != 0)
      f->cur.source = g.source;
    // A later explicit grant of our own channel is either this transmission
    // re-announced, or the next one (a quick re-key on the same channel).
    // Which, the voice tells: if it carries on, it was this one (see below)
    if(g.transmission > f->cur.transmission && g.transmission > f->pending){
      f->pending = g.transmission;
      f->pending_at = f->now;
    }
  }
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
    // Equal priority: stay with the conversation, then the newest
    bool const conv = g->group == f->last_group, best_conv = best != NULL && best->group == f->last_group;
    if(best == NULL || p < best_pri || (p == best_pri && (conv != best_conv ? conv : g->seen > best->seen))){
      best = g;
      best_pri = p;
    }
  }
  return best;
}

// A later transmission of the talkgroup we follow, on another channel (the
// reply, under transmission trunking): the newest, or NULL. Ordered by the
// control channel's grant numbers where both have one, else by grant time
static struct follow_grant const *newer_transmission(struct dv_follow const *f){
  struct follow_grant const *newest = NULL;
  for(int i = 0; i < FOLLOW_GRANTS; i++){
    struct follow_grant const *g = &f->grants[i];
    if(!g->valid || f->now - g->seen > FOLLOW_FRESH || g->group != f->cur.group || g->unit != f->cur.unit)
      continue;
    if(g->freq == f->cur.freq && g->slot == f->cur.slot)
      continue;
    bool const newer = (g->transmission != 0 && f->cur.transmission != 0)
      ? g->transmission > f->cur.transmission
      : g->transmission != 0 ? g->granted >= f->started  // an explicit grant from our join on
      : g->granted > f->started;
    if(!newer)
      continue;
    if(newest == NULL || g->transmission > newest->transmission || g->granted > newest->granted)
      newest = g;
  }
  return newest;
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
  f->pending = 0;
  f->lingering = false;
  {
    dv_stats st;
    dv_get_stats(f->voice,&st);
    f->voice_serial = st.call_serial;
  }
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
      f->lingering = false;
    }
    // Voice well after a grant of our channel: that grant re-announced this
    // transmission, so it's the number we finish with. (If the transmission
    // ends first, the grant was the next one, and stays followable)
    if(f->pending != 0 && f->last_voice > f->pending_at + FOLLOW_REGRANT){
      f->cur.transmission = f->pending;
      f->pending = 0;
    }
    struct follow_grant const *b = best_grant(f);
    struct follow_grant const *r = NULL;
    if(st.m17_encrypted && st.voice_decoded == 0)
      release(f,"encrypted");
    else if(st.call_serial != f->voice_serial){
      if(f->pending != 0){
	// Our channel was granted again during this transmission and the
	// voice didn't settle whether that was this transmission re-announced
	// or the next one. The channel will say: stay up to FOLLOW_LINGER.
	// Voice now is the next transmission (a new call: the decoder's call
	// serial has moved on); silence means it was a re-announcement, and
	// we leave with its number so its updates can't bring us back.
	// A re-key whose voice starts later than that is missed: the price of
	// not having a signal that tells the two apart
	f->cur.transmission = f->pending;
	f->pending = 0;
	f->voice_serial = st.call_serial;
	f->started = f->last_voice = f->now;
	f->lingering = true;
      } else
	release(f,"transmission ended"); // MAC_END_PTT, TDU: don't wait out the hang time
    }
    else if(f->lingering && f->last_voice == f->started && f->now - f->started > FOLLOW_LINGER)
      release(f,"transmission ended");
    else if((r = newer_transmission(f)) != NULL
	    && !(b != NULL && priority(f,b) < priority(f,r) && b->group != f->cur.group)){
      // Transmission trunking: the reply has its own channel. Go straight
      // to it (unless a higher priority call is waiting, which the
      // preemption below then takes)
      struct follow_grant const reply = *r;
      release(f,"next transmission");
      join(f,&reply,cc);
    }
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

bool dv_follow_call(struct dv_follow const *f, char *dest, size_t dlen, char *src, size_t slen,
		    double *freq, int *slot, uint64_t *generation, uint64_t *serial){
  if(f == NULL || f->voice == NULL)
    return false;
  *freq = (double)f->cur.freq;
  *slot = f->cur.slot;
  dv_stats st;
  dv_get_stats(f->voice,&st);
  *generation = f->calls;
  *serial = st.call_serial;
  snprintf(dest,dlen,"%" PRIu32,f->cur.group);
  if(st.src_call[0] != '\0')
    strlcpy(src,st.src_call,slen);
  else if(f->cur.source != 0)
    snprintf(src,slen,"%" PRIu32,f->cur.source);
  else
    src[0] = '\0';
  return true;
}
