// Digital voice demodulator for radiod: P25 Phase 1, DMR, M17, YSF, D-STAR, NXDN,
// or whichever of them is on the air ("digital")
// The decoder library (contrib/digital-voice) takes 48 kHz complex IQ and
// emits 8 kHz speech in bursts (P25: 9 frames per LDU; DMR: 3 per burst;
// M17: 2 per frame; YSF: 5 per frame; D-STAR: 1 per frame; NXDN: 4 per frame);
// a small queue paces that out at real time
#include <assert.h>
#include <complex.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "misc.h"
#include "radio.h"
#include "sched.h"
#include "dv.h"
#include "dv_follow.h"

#define DV_AUDIO_QUEUE_DEPTH 45 // 900 ms

// Speech level control: vocoder output level follows the talker (13 dB
// between talkers is common) and differs between vocoders. Track the mean
// power of frames with speech in them (vocoder pauses are far quieter than
// the gate), quickly when it rises, more slowly when it falls, and bring it
// to a P25-like level; 'agc = no' turns it off. The preset 'gain' applies on
// top. On decoded P25-level, YSF, M17, D-STAR and NXDN speech this lands
// within 3 dB of the target
#define DV_AGC_TARGET_DB (-33.0f)  // dBFS, the middle of live P25 traffic
#define DV_AGC_GATE_DB (-70.0f)    // quieter frames are pauses: leave the level alone
#define DV_AGC_ATTACK 0.25f        // per 20 ms frame, level rising (~80 ms)
#define DV_AGC_RELEASE 0.05f       // per frame, level falling (~400 ms)
#define DV_AGC_MIN_DB (-12.0f)
#define DV_AGC_MAX_DB (18.0f)
#define DV_AGC_CEILING 0.9f        // peak limit, fraction of full scale

// Analog FM alongside "digital" (analog = yes): discriminator, low pass and
// 6:1 decimation to 8 kHz, a 300 Hz high pass (subaudible tones), the
// channel's de-emphasis, and a squelch on the channel's SNR. Its audio is
// delayed DV_FM_DELAY frames so that a digital transmission is recognized
// (any receiver's sync) before any of it can be heard as FM
#define DV_FM_TAPS 63
#define DV_FM_DECIM 6               // 48 kHz -> 8 kHz
#define DV_FM_DEVIATION 5000.0f     // Hz at full scale
#define DV_FM_DELAY 12              // frames (240 ms)
#define DV_FM_SLACK 16              // more frames room: a long blocktime makes several per block
#define DV_FM_CLOSE_TIME 0.06f      // seconds below the close threshold to close
#define DV_DIGITAL_HOLD (DV_IQ_SAMPRATE_RADIOD / 2) // FM stays muted 500 ms after digital activity

// Carrier gate: the decoders run only while there is something to decode
#define DV_GATE_OPEN 1.0f            // SNR (power ratio in the channel filter) to start them: 0 dB
#define DV_GATE_CLOSE 0.5f           // below this (-3 dB) ...
#define DV_GATE_HANG (DV_IQ_SAMPRATE_RADIOD)          // ... for 1 s, with no sync word, stops them
#define DV_GATE_PREROLL (DV_IQ_SAMPRATE_RADIOD / 5)   // 200 ms of IQ replayed into them on opening
// Mixed channels: a carrier with no digital sync this long is FM; the digital
// receivers stop until it drops, except for a look every DV_FM_RECHECK
#define DV_FM_LOCK (DV_IQ_SAMPRATE_RADIOD / 2)
#define DV_FM_RECHECK (DV_IQ_SAMPRATE_RADIOD * 5)

struct dv_gate {
  bool enabled;
  bool open;
  uint64_t opened;                   // sample count when it opened
  uint64_t quiet_since;              // when the SNR went (and stayed) below DV_GATE_CLOSE
  float complex ring[DV_GATE_PREROLL];
  size_t ring_pos, ring_fill;
};

// Remember the latest DV_GATE_PREROLL samples
static void gate_remember(struct dv_gate *g,float complex const *iq,int count){
  for(int i = 0; i < count; i++){
    g->ring[g->ring_pos] = iq[i];
    g->ring_pos = (g->ring_pos + 1) % DV_GATE_PREROLL;
  }
  g->ring_fill = g->ring_fill + (size_t)count > DV_GATE_PREROLL ? DV_GATE_PREROLL : g->ring_fill + (size_t)count;
}

// Feed the remembered samples, oldest first, to a decoder that was asleep
static void gate_replay(struct dv_gate *g,dv_decoder *decoder){
  size_t start = (g->ring_pos + DV_GATE_PREROLL - g->ring_fill) % DV_GATE_PREROLL;
  size_t left = g->ring_fill;
  while(left > 0){
    size_t const n = start + left > DV_GATE_PREROLL ? DV_GATE_PREROLL - start : left;
    dv_process(decoder,(float const *)&g->ring[start],n);
    left -= n;
    start = (start + n) % DV_GATE_PREROLL;
  }
  g->ring_fill = 0;
}

struct dv_fm {
  float complex prev;
  float taps[DV_FM_TAPS];
  float hist[DV_FM_TAPS];
  int pos, phase;
  float hp_x1, hp_x2, hp_y1, hp_y2;  // 2nd order high pass state
  float hp_b0, hp_b1, hp_b2, hp_a1, hp_a2;
  float deemph;
  bool open;
  int below;                          // frames below the close threshold
  float frame[DV_PCM_SAMPLES];        // 8 kHz audio being collected
  int frame_len;
  float delay[DV_FM_DELAY + DV_FM_SLACK][DV_PCM_SAMPLES]; // delay line of finished frames
  bool delay_open[DV_FM_DELAY + DV_FM_SLACK];
  int delay_count;
};

static void dv_fm_init(struct dv_fm *fm){
  memset(fm,0,sizeof *fm);
  // Low pass: windowed sinc, 3.4 kHz at 48 kHz
  float sum = 0;
  for(int i = 0; i < DV_FM_TAPS; i++){
    float const x = i - (DV_FM_TAPS - 1) / 2.0f;
    float const fc = 3400.0f / DV_IQ_SAMPRATE_RADIOD;
    float const sinc = x == 0 ? 2 * fc : sinf(2 * M_PIf * fc * x) / (M_PIf * x);
    float const w = 0.42f - 0.5f * cosf(2 * M_PIf * i / (DV_FM_TAPS - 1)) + 0.08f * cosf(4 * M_PIf * i / (DV_FM_TAPS - 1));
    fm->taps[i] = sinc * w;
    sum += fm->taps[i];
  }
  for(int i = 0; i < DV_FM_TAPS; i++)
    fm->taps[i] /= sum;
  // High pass: 2nd order Butterworth at 300 Hz, 8 kHz (RBJ cookbook)
  float const w0 = 2 * M_PIf * 300.0f / DV_AUDIO_SAMPRATE;
  float const alpha = sinf(w0) / (2 * 0.70710678f);
  float const a0 = 1 + alpha;
  fm->hp_b0 = (1 + cosf(w0)) / 2 / a0;
  fm->hp_b1 = -(1 + cosf(w0)) / a0;
  fm->hp_b2 = (1 + cosf(w0)) / 2 / a0;
  fm->hp_a1 = -2 * cosf(w0) / a0;
  fm->hp_a2 = (1 - alpha) / a0;
}

// One block of 48 kHz baseband through the FM chain; finished 8 kHz frames
// go into the delay line, tagged with the squelch state
static void dv_fm_block(struct dv_fm *fm,chan_t const *chan,float complex const *in,int n){
  // Squelch on the channel's SNR (as fm.c's simple estimator)
  float const noise = chan->sig.n0 * fabsf(chan->filter.max_IF - chan->filter.min_IF);
  float const snr = noise <= 0 ? 0 : chan->sig.bb_power / noise - 1;
  float const open_at = (isfinite(chan->squelch.open) && chan->squelch.open > 0) ? chan->squelch.open : 6.3f;
  float const close_at = (isfinite(chan->squelch.close) && chan->squelch.close > 0) ? chan->squelch.close : 4.0f;
  if(snr >= open_at){
    fm->open = true;
    fm->below = 0;
  } else if(fm->open && snr < close_at
	    && ++fm->below > (int)ceilf(DV_FM_CLOSE_TIME / (float)Blocktime) + chan->squelch.tail){
    // (the tail, as in fm.c, is in blocks)
    fm->open = false;
  }
  for(int i = 0; i < n; i++){
    float const f = cargf(in[i] * conjf(fm->prev)) * (DV_IQ_SAMPRATE_RADIOD / (2 * M_PIf)) / DV_FM_DEVIATION;
    fm->prev = in[i];
    fm->hist[fm->pos] = f;
    fm->pos = (fm->pos + 1) % DV_FM_TAPS;
    if(++fm->phase < DV_FM_DECIM)
      continue;
    fm->phase = 0;
    float y = 0;
    for(int k = 0; k < DV_FM_TAPS; k++)
      y += fm->taps[k] * fm->hist[(fm->pos + k) % DV_FM_TAPS];
    // High pass
    float const h = fm->hp_b0 * y + fm->hp_b1 * fm->hp_x1 + fm->hp_b2 * fm->hp_x2 - fm->hp_a1 * fm->hp_y1 - fm->hp_a2 * fm->hp_y2;
    fm->hp_x2 = fm->hp_x1; fm->hp_x1 = y;
    fm->hp_y2 = fm->hp_y1; fm->hp_y1 = h;
    // De-emphasis (deemph-tc, deemph-gain), or flat
    float out = h;
    if(chan->fm.rate != 1){
      fm->deemph += chan->fm.rate * (h - fm->deemph);
      out = fm->deemph * chan->fm.gain;
    }
    fm->frame[fm->frame_len++] = out;
    if(fm->frame_len == DV_PCM_SAMPLES){
      fm->frame_len = 0;
      if(fm->delay_count < DV_FM_DELAY + DV_FM_SLACK){
        memcpy(fm->delay[fm->delay_count],fm->frame,sizeof fm->frame);
        fm->delay_open[fm->delay_count] = fm->open;
        fm->delay_count++;
      }
    }
  }
}

// Take the oldest frame once the delay line is full; false if none is due
static bool dv_fm_take(struct dv_fm *fm,float out[DV_PCM_SAMPLES],bool *open){
  if(fm->delay_count <= DV_FM_DELAY)
    return false;
  memcpy(out,fm->delay[0],sizeof fm->delay[0]);
  *open = fm->delay_open[0];
  memmove(fm->delay[0],fm->delay[1],(size_t)(fm->delay_count - 1) * sizeof fm->delay[0]);
  memmove(fm->delay_open,fm->delay_open + 1,(size_t)(fm->delay_count - 1) * sizeof fm->delay_open[0]);
  fm->delay_count--;
  return true;
}

struct dv_agc {
  float level_db;  // tracked speech level
  float gain;      // current linear gain, smoothed across frames
};

// Apply level control to one frame of floats (full scale +/-1) in place
static void dv_agc_frame(struct dv_agc *agc,float *audio,int n){
  float power = 0;
  for(int i = 0; i < n; i++)
    power += audio[i] * audio[i];
  float const db = 10.0f * log10f(power / n + 1e-20f);
  if(db > DV_AGC_GATE_DB){
    // Track mean power (what loudness follows), not mean dB
    float const level = powf(10.0f,agc->level_db / 10.0f);
    float const p = power / n;
    float const k = p > level ? DV_AGC_ATTACK : DV_AGC_RELEASE;
    agc->level_db = 10.0f * log10f(level + k * (p - level) + 1e-20f);
  }
  float gain_db = DV_AGC_TARGET_DB - agc->level_db;
  gain_db = gain_db < DV_AGC_MIN_DB ? DV_AGC_MIN_DB : gain_db > DV_AGC_MAX_DB ? DV_AGC_MAX_DB : gain_db;
  float const target = powf(10.0f,gain_db / 20.0f);
  // Ramp across the frame to avoid steps
  float const step = (target - agc->gain) / n;
  float peak = 0;
  for(int i = 0; i < n; i++){
    agc->gain += step;
    audio[i] *= agc->gain;
    if(fabsf(audio[i]) > peak)
      peak = fabsf(audio[i]);
  }
  if(peak > DV_AGC_CEILING){
    float const scale = DV_AGC_CEILING / peak;
    for(int i = 0; i < n; i++)
      audio[i] *= scale;
  }
}

// A call: what a recorder files it under. id changes with each call; 0 = none
struct dv_call {
  char dest[sizeof ((chan_t *)0)->dv.call_dest];
  char src[sizeof ((chan_t *)0)->dv.call_src];
  char mode[sizeof ((chan_t *)0)->dv.call_mode];
  uint32_t id;
  double freq;
  int slot;
  uint64_t serial;     // decoder's call-ending counter
  uint64_t generation; // followed decoder instance, which may reuse serials
};

// Decoded speech waiting for its 20 ms slot in the output, each frame
// labelled with the call it belongs to, so the call's start and end are
// published exactly where its audio starts and ends
struct dv_audio_queue {
  int16_t samples[DV_AUDIO_QUEUE_DEPTH][DV_PCM_SAMPLES];
  struct dv_call call[DV_AUDIO_QUEUE_DEPTH];
  unsigned int head;
  unsigned int count;
  uint64_t dropped;
  struct dv_call *current;
  chan_t const *chan;
  dv_decoder const *decoder;
  dv_mode mode;
  struct dv_follow const *follow;
  int absent; // samples without IDs, independent of the number of PCM frames
};

// P25 control channel events (site, band plan, voice grants), logged with -v
// and passed to the grant follower, if any
struct dv_events {
  struct channel const *chan;
  struct dv_follow *follow;
};
static void log_event(void *user_data,dv_event const *event){
  struct dv_events const *ev = user_data;
  if(Verbose > 0 && event->text[0] != '\0')
    fprintf(stderr,"%s P25 %s\n",ev->chan->name,event->text);
  dv_follow_event(ev->follow,event);
}

// The current call's destination, source and mode, as recorders want them
static void call_ids(dv_stats const *st,dv_mode mode,struct dv_follow const *follow,chan_t const *chan,struct dv_call *call){
  memset(call,0,sizeof *call);
  call->freq = chan->tune.freq;
  call->slot = -1;
  call->serial = st->call_serial;
  if(follow != NULL){
    if(dv_follow_call(follow,call->dest,sizeof call->dest,call->src,sizeof call->src,
		      &call->freq,&call->slot,&call->generation,&call->serial))
      strlcpy(call->mode,"P25",sizeof call->mode);
    return;
  }
  int const m = mode == DV_AUTO ? st->active_mode : (int)mode;
  if(m < 0)
    return; // DV_AUTO, nothing playing
  if(m == DV_P25P2 && st->active_slot == 0)
    return; // no call on the slot we play (DMR reports its call until it ends)
  if(m == DV_P25P2)
    call->slot = st->active_slot - 1;
  if(st->dst_call[0] == '\0' && st->src_call[0] == '\0')
    return;
  // DMR shows "TG 44772": recorders file talkgroups as bare numbers, as for P25
  char const *dest = st->dst_call;
  if(m == DV_DMR && strncmp(dest,"TG ",3) == 0)
    dest += 3;
  strlcpy(call->dest,dest,sizeof call->dest);
  strlcpy(call->src,st->src_call,sizeof call->src);
  strlcpy(call->mode,dv_mode_name(m),sizeof call->mode);
}

// Send status on the output stream now, so a recorder learns of a new call
// before (or as) its audio arrives
static void push_status(chan_t *chan){
  pthread_mutex_lock(&chan->status.lock);
  send_radio_status((struct sockaddr *)&chan->status.dest_socket,chan->frontend,chan);
  chan->status.output_timer = chan->status.output_interval;
  pthread_mutex_unlock(&chan->status.lock);
}

// Follow the decoders' view of the call. A call starts when IDs appear, or
// when the destination, the talker or the mode changes; it ends when the IDs
// go. A talker or destination that fills in later is the same call
static uint32_t Call_seq; // ids across all channels, so they never repeat on one
#define DV_CALL_GONE (DV_IQ_SAMPRATE_RADIOD / 2) // samples (0.5 s) without IDs that end a call
// samples: how much signal this observation covers (0 inside a block: a PCM frame)
static void track_call(chan_t const *chan,struct dv_call *cur,struct dv_call const *seen,struct dv_audio_queue *queue,int samples){
  // A terminator is definitive, even when the next key-up has identical
  // IDs, or both calls occur within a single input block. Only uncertain
  // loss of IDs gets the fade grace period below.
  if(cur->serial != seen->serial || cur->generation != seen->generation){
    if(cur->id != 0 && Verbose > 1)
      fprintf(stderr,"%s call %u ended\n",chan->name,cur->id);
    memset(cur,0,sizeof *cur);
    queue->absent = 0;
  }
  bool const has_ids = seen->dest[0] != '\0' || seen->src[0] != '\0';
  if(!has_ids){
    // Gone for a moment (a fade, a slot glitch) is still the same call
    if(cur->id != 0 && samples > 0 && (queue->absent += samples) >= DV_CALL_GONE){
      if(Verbose > 1)
	fprintf(stderr,"%s call %u ended\n",chan->name,cur->id);
      memset(cur,0,sizeof *cur); // ended
    }
    if(cur->id == 0)
      *cur = *seen; // retain the boundary token for unidentified audio
    return;
  }
  queue->absent = 0;
  bool const differs = cur->id == 0
    || (cur->dest[0] != '\0' && seen->dest[0] != '\0' && strcmp(cur->dest,seen->dest) != 0)
    || (cur->src[0] != '\0' && seen->src[0] != '\0' && strcmp(cur->src,seen->src) != 0)
    || strcmp(cur->mode,seen->mode) != 0 || cur->freq != seen->freq || cur->slot != seen->slot;
  if(differs){
    bool const fresh = cur->id == 0;
    uint32_t id = __atomic_add_fetch(&Call_seq,1,__ATOMIC_RELAXED);
    if(id == 0)
      id = __atomic_add_fetch(&Call_seq,1,__ATOMIC_RELAXED); // 0 means none
    if(Verbose > 1)
      fprintf(stderr,"%s call %u: %s %s to %s%s\n",chan->name,id,seen->mode,
	      seen->src[0] ? seen->src : "?",seen->dest[0] ? seen->dest : "?",
	      cur->id != 0 ? " (replacing the last)" : "");
    *cur = *seen;
    cur->id = id;
    if(fresh){
      // Speech decoded before the IDs arrived, still queued, is this call's start
      for(unsigned int i = 0; i < queue->count; i++){
	struct dv_call *c = &queue->call[(queue->head + i) % DV_AUDIO_QUEUE_DEPTH];
	if(c->id == 0 && c->serial == cur->serial && c->generation == cur->generation)
	  *c = *cur;
      }
    }
  } else {
    // Same call: take any ID that has filled in
    if(cur->dest[0] == '\0')
      strlcpy(cur->dest,seen->dest,sizeof cur->dest);
    if(cur->src[0] == '\0')
      strlcpy(cur->src,seen->src,sizeof cur->src);
  }
}

// Make a call (or none) the channel's published call: flush the audio of the
// previous one onto the network first, then send status
static void publish_call(chan_t *chan,struct dv_call const *call){
  if(call->id == chan->dv.call_id && strcmp(call->dest,chan->dv.call_dest) == 0
     && strcmp(call->src,chan->dv.call_src) == 0)
    return;
  if(call->id != chan->dv.call_id)
    send_output(chan,NULL,0,false); // flush: the old call's last packet goes first
  strlcpy(chan->dv.call_dest,call->dest,sizeof chan->dv.call_dest);
  strlcpy(chan->dv.call_src,call->src,sizeof chan->dv.call_src);
  strlcpy(chan->dv.call_mode,call->mode,sizeof chan->dv.call_mode);
  chan->dv.call_id = call->id;
  chan->dv.call_freq = call->freq;
  chan->dv.call_slot = call->slot;
  // Where in the stream this applies: the old call's audio is flushed, so the
  // next packet starts here. A recorder splits at this timestamp, however
  // status and data happen to interleave on their two sockets
  chan->dv.call_timestamp = chan->output.rtp.timestamp;
  push_status(chan);
}

static void queue_pcm(void *user_data, int16_t const pcm[DV_PCM_SAMPLES]){
  struct dv_audio_queue * const queue = user_data;
  // Query synchronously: after dv_process() returns, the decoder may have
  // reached a terminator or a later call. Existing queued frames keep their
  // own labels when this frame changes the current call.
  dv_stats st;
  dv_get_stats(queue->decoder,&st);
  struct dv_call seen;
  call_ids(&st,queue->mode,queue->follow,queue->chan,&seen);
  track_call(queue->chan,queue->current,&seen,queue,0);
  if(queue->count == DV_AUDIO_QUEUE_DEPTH){
    queue->dropped++;
    return;
  }
  unsigned int const tail = (queue->head + queue->count) % DV_AUDIO_QUEUE_DEPTH;
  memcpy(queue->samples[tail],pcm,sizeof queue->samples[tail]);
  if(queue->current != NULL)
    queue->call[tail] = *queue->current;
  else
    memset(&queue->call[tail],0,sizeof queue->call[tail]);
  queue->count++;
}

int demod_dv(void *arg){
  chan_t * const chan = arg;
  assert(chan != NULL);
  if(chan == NULL)
    return -1;

  _Static_assert(DV_IQ_SAMPRATE_RADIOD == DV_IQ_SAMPRATE, "digital voice IQ rate mismatch");
  _Static_assert(DV_AUDIO_SAMPRATE == DV_PCM_SAMPRATE, "digital voice audio rate mismatch");
  bool const is_dmr = chan->demod_type == DMR_DEMOD;
  bool const is_m17 = chan->demod_type == M17_DEMOD;
  bool const is_ysf = chan->demod_type == YSF_DEMOD;
  bool const is_dstar = chan->demod_type == DSTAR_DEMOD;
  bool const is_nxdn = chan->demod_type == NXDN_DEMOD;
  bool const is_auto = chan->demod_type == DIGITAL_DEMOD;
  bool const has_callsign = is_m17 || is_ysf || is_dstar || is_nxdn;
  char const * const label = is_dmr ? "DMR" : is_m17 ? "M17" : is_ysf ? "YSF" : is_dstar ? "D-STAR"
    : is_nxdn ? (chan->nxdn.nxdn96 ? "NXDN96" : "NXDN48") : is_auto ? "DIGITAL" : "P25";

  // Like WFM, the output rate is fixed by the mode, whatever was set before
  chan->output.samprate = DV_AUDIO_SAMPRATE;
  chan->output.channels = 1;
  {
    int const pt = pt_from_info(chan->output.samprate,chan->output.channels,chan->output.encoding);
    if(pt >= 0)
      chan->output.rtp.type = (uint8_t)pt;
  }
  int const blocksize = (int)lrint(DV_IQ_SAMPRATE_RADIOD * Blocktime);
  if(create_filter_output(&chan->filter.out,&chan->frontend->in,blocksize,COMPLEX) != 0){
    fprintf(stderr,"%s: couldn't create %s channel filter\n",chan->name,label);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }
  set_channel_filter(chan);

  struct dv_audio_queue audio_queue = {0};
  struct dv_call decoding = {0};     // the call the decoders are producing
  audio_queue.current = &decoding;
  audio_queue.chan = chan;
  int idle_frames = 0;               // output frames with nothing queued
  dv_config const config = {
    .p25_nac = chan->p25.nac,
    .p25_wacn = chan->p25.wacn,
    .p25_sysid = chan->p25.sysid,
    .p25_slot = chan->p25.slot,
    .dmr_slot = chan->dmr.slot,
    .m17_can_filter = chan->m17.can >= 0,
    .m17_can = chan->m17.can,
  };
  dv_mode const mode = is_auto ? DV_AUTO : is_dmr ? DV_DMR : is_m17 ? DV_M17 : is_ysf ? DV_YSF : is_dstar ? DV_DSTAR
    : is_nxdn ? (chan->nxdn.nxdn96 ? DV_NXDN96 : DV_NXDN48)
    : chan->p25.modulation == P25_CQPSK ? DV_P25_CQPSK
    : chan->p25.modulation == P25_PHASE2 ? DV_P25P2 : DV_P25_C4FM;
  bool const is_p25p2 = mode == DV_P25P2;
  // A following channel plays only the calls it follows: its control channel
  // decoder's own speech (should the control channel move and this frequency
  // carry voice) would bypass the talkgroup list
  bool const following = chan->p25.follow && (mode == DV_P25_C4FM || mode == DV_P25_CQPSK);
  dv_decoder * const decoder = dv_create(mode,&config,following ? NULL : queue_pcm,&audio_queue);
  if(decoder == NULL){
    fprintf(stderr,"%s: couldn't create %s decoder\n",chan->name,label);
    chan->demod_type = INVALID_DEMOD;
    return -1;
  }
  audio_queue.decoder = decoder;
  audio_queue.mode = mode;
  // A P25 Phase 1 control channel can follow its voice grants
  struct dv_events events = { .chan = chan };
  if(following){
    events.follow = dv_follow_create(chan,queue_pcm,&audio_queue);
    if(events.follow == NULL)
      fprintf(stderr,"%s: couldn't create the P25 voice channel follower\n",chan->name);
  }
  dv_set_event_callback(decoder,log_event,&events);
  audio_queue.follow = events.follow;

  // Emit one 20 ms audio frame per 20 ms of IQ, in integer arithmetic
  uint64_t output_clock = 0;
  uint64_t const audio_frame_clock = (uint64_t)DV_IQ_SAMPRATE_RADIOD * DV_PCM_SAMPLES;
  uint64_t reported_drops = 0;
  int telemetry_samples = 0;
  struct dv_agc agc = { .level_db = DV_AGC_TARGET_DB, .gain = 1.0f };
  bool const analog = is_auto && chan->dv.analog;
  static struct dv_fm fm_zero; // large: not on the stack
  struct dv_fm *fm = NULL;
  if(analog){
    fm = malloc(sizeof *fm);
    if(fm == NULL)
      fm = &fm_zero;
    dv_fm_init(fm);
  }
  // The carrier gate; never on a channel following a control channel, which must run
  static struct dv_gate gate_zero;
  struct dv_gate *gate = calloc(1,sizeof *gate);
  if(gate == NULL)
    gate = &gate_zero;
  gate->enabled = chan->dv.gate && events.follow == NULL;
  gate->open = !gate->enabled;
  uint64_t gate_syncs = 0;      // decoder sync count when last seen moving
  bool fm_locked = false;       // mixed: this carrier is FM, the digital receivers rest
  uint64_t fm_locked_at = 0;
  uint64_t digital_seen = 0;    // sample count of the last digital activity
  uint64_t sample_count = 0;
  uint64_t last_syncs = 0;
  bool fm_playing = false;
  chan->tp1 = 0; // detected sync words
  chan->tp2 = 0; // decoded frames (P25) or bursts (DMR)
  chan->p25.have_nac = false;
  chan->dv.source[0] = '\0';
  chan->dv.call_dest[0] = chan->dv.call_src[0] = chan->dv.call_mode[0] = '\0';
  chan->dv.call_id = 0;
  chan->dv.call_freq = chan->tune.freq;
  chan->dv.call_slot = -1;
  bool restart_needed = false;
  realtime(chan->prio);

  while(!restart_needed){
    restart_needed = handle_commands(chan);
    int r;
    if(restart_needed || (r = downconvert(chan)) == -1)
      break; // restart or terminate
    else if(r == 1)
      continue; // channel inactive; poll for commands

    int const count = chan->sampcount;
    sample_count += (uint64_t)count;
    bool run = true;
    if(gate->enabled){
      float const noise = chan->sig.n0 * fabsf(chan->filter.max_IF - chan->filter.min_IF);
      float const snr = noise <= 0 ? 0 : chan->sig.bb_power / noise - 1;
      if(!gate->open){
	if(snr >= DV_GATE_OPEN){
	  gate->open = true;
	  gate->opened = sample_count;
	  gate->quiet_since = 0;
	  fm_locked = false;
	  gate_replay(gate,decoder); // the start of the transmission
	  if(Verbose > 1)
	    fprintf(stderr,"%s %s gate open, SNR %.1f dB\n",chan->name,label,power2dB(snr));
	}
      } else if(snr >= DV_GATE_CLOSE){
	gate->quiet_since = 0;
      } else if(gate->quiet_since == 0){
	gate->quiet_since = sample_count;
      } else if(sample_count - gate->quiet_since > DV_GATE_HANG && audio_queue.count == 0){
	gate->open = false; // carrier gone and the decoders have nothing more
	fm_locked = false;
	dv_signal_lost(decoder); // the next transmission starts clean
	if(Verbose > 1)
	  fprintf(stderr,"%s %s gate closed after %.1f s\n",chan->name,label,
		  (double)(sample_count - gate->opened) / DV_IQ_SAMPRATE_RADIOD);
      }
      if(!gate->open)
	gate_remember(gate,chan->baseband,count);
      run = gate->open;
    }
    if(run && fm_locked && sample_count - fm_locked_at > DV_FM_RECHECK){
      fm_locked = false; // a look for digital that came up under the FM
      gate->opened = sample_count;
    }
    if(run && !fm_locked){
      // float complex is layout-compatible with interleaved I/Q floats
      dv_process(decoder,(float const *)chan->baseband,(size_t)count);
    }
    {
      dv_stats st;
      dv_get_stats(decoder,&st);
      if(events.follow != NULL)
	dv_follow_block(events.follow,&st,count);
      if(st.syncs != gate_syncs){
	gate_syncs = st.syncs;
	gate->quiet_since = 0; // still decoding: hold the gate open
      }
      // Labels for what the decoders produce next; published as it plays
      struct dv_call seen;
      call_ids(&st,mode,events.follow,chan,&seen);
      track_call(chan,&decoding,&seen,&audio_queue,count);
    }
    if(analog){
      dv_fm_block(fm,chan,chan->baseband,count);
      // Any receiver in a call, or any sync word, means digital: keep FM quiet
      dv_stats st;
      dv_get_stats(decoder,&st);
      if(st.active_mode >= 0 || st.syncs != last_syncs || audio_queue.count != 0)
	digital_seen = sample_count;
      last_syncs = st.syncs;
      // FM squelch open, and no digital since the carrier came up: it's FM
      if(gate->enabled && gate->open && !fm_locked && fm->open
	 && sample_count - gate->opened > DV_FM_LOCK && digital_seen < gate->opened){
	fm_locked = true;
	fm_locked_at = sample_count;
	if(Verbose > 1)
	  fprintf(stderr,"%s %s carrier is FM: digital receivers resting\n",chan->name,label);
      }
    }

    telemetry_samples += count;
    if(telemetry_samples >= DV_IQ_SAMPRATE_RADIOD){
      telemetry_samples -= DV_IQ_SAMPRATE_RADIOD;
      dv_stats stats;
      dv_get_stats(decoder,&stats);
      chan->tp1 = (double)stats.syncs;
      chan->tp2 = (double)stats.frames;
      chan->sig.foffset = stats.freq_offset;
      if(has_callsign)
	strlcpy(chan->dv.source,stats.src_call,sizeof chan->dv.source);
      if(is_p25p2){
	if(stats.active_slot == 0)
	  chan->dv.source[0] = '\0';
	else
	  snprintf(chan->dv.source,sizeof chan->dv.source,"TG%s%.6s%s%.8s",
		   stats.dst_call[0] ? " " : "",stats.dst_call,
		   stats.src_call[0] ? " " : "",stats.src_call);
      }
      if(is_dmr){
	if(stats.active_slot == 0)
	  chan->dv.source[0] = '\0';
	else
	  snprintf(chan->dv.source,sizeof chan->dv.source,"TS%d%s%s",stats.active_slot,
		   stats.src_call[0] ? " " : "",stats.src_call);
      }
      if(is_auto){
	// Mode, then whatever identifies the transmission
	if(stats.active_mode < 0)
	  strlcpy(chan->dv.source,fm_playing ? "FM" : "",sizeof chan->dv.source);
	else if(stats.active_mode == DV_P25_C4FM && stats.frames != 0)
	  snprintf(chan->dv.source,sizeof chan->dv.source,"P25 N%03X",stats.last_nac);
	else if(stats.active_mode == DV_DMR && !stats.src_call[0] && stats.active_slot != 0)
	  snprintf(chan->dv.source,sizeof chan->dv.source,"DMR TS%d",stats.active_slot);
	else
	  snprintf(chan->dv.source,sizeof chan->dv.source,"%s%s%s",dv_mode_name(stats.active_mode),
		   stats.src_call[0] ? " " : "",stats.src_call);
      }
      if(events.follow != NULL){
	if(!dv_follow_source(events.follow,chan->dv.source,sizeof chan->dv.source))
	  chan->dv.source[0] = '\0';
      } else if(mode == DV_P25_C4FM || mode == DV_P25_CQPSK){
	// Conventional P25: the call's talkgroup and talker, from the link control
	if(stats.dst_call[0] == '\0')
	  chan->dv.source[0] = '\0';
	else
	  snprintf(chan->dv.source,sizeof chan->dv.source,"TG%.6s%s%.8s",stats.dst_call,
		   stats.src_call[0] ? " " : "",stats.src_call);
      }
      if(!is_dmr && !has_callsign && !is_auto && stats.frames != 0){
	chan->p25.last_nac = stats.last_nac;
	chan->p25.have_nac = true;
      }
      if(Verbose > 0){
	if(is_auto)
	  fprintf(stderr,"%s DIGITAL samples=%" PRIu64 " syncs=%" PRIu64 " frames=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64 " active=%s offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,
		  chan->dv.source[0] ? chan->dv.source : "-",stats.freq_offset);
	else if(is_ysf || is_dstar || is_nxdn)
	  fprintf(stderr,"%s %s samples=%" PRIu64 " syncs=%" PRIu64 " frames=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64 " src=%s dst=%s offset=%+.1f Hz\n",
		  chan->name,label,stats.samples,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,
		  stats.src_call[0] ? stats.src_call : "-",stats.dst_call[0] ? stats.dst_call : "-",
		  stats.freq_offset);
	else if(is_m17)
	  fprintf(stderr,"%s M17 samples=%" PRIu64 " syncs=%" PRIu64 " frames=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64 " src=%s dst=%s can=%d%s"
		  " errors=%.1f%% offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,
		  stats.src_call[0] ? stats.src_call : "-",stats.dst_call[0] ? stats.dst_call : "-",
		  stats.m17_can,stats.m17_encrypted ? " encrypted" : "",
		  100.0f * stats.error_rate,stats.freq_offset);
	else if(is_p25p2)
	  fprintf(stderr,"%s P25P2 samples=%" PRIu64 " syncs=%" PRIu64 " bursts=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64 " slot=%d tg=%s src=%s%s errors=%.1f%% offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,config.p25_slot,
		  stats.dst_call[0] ? stats.dst_call : "-",stats.src_call[0] ? stats.src_call : "-",
		  stats.m17_encrypted ? " encrypted" : "",
		  100.0f * stats.error_rate,stats.freq_offset);
	else if(is_dmr)
	  fprintf(stderr,"%s DMR samples=%" PRIu64 " symbols=%" PRIu64
		  " syncs=%" PRIu64 " bursts=%" PRIu64
		  " voice=%" PRIu64 " muted=%" PRIu64
		  " slot=%d cc=%d src=%s dst=%s%s errors=%.1f%% offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.symbols,stats.syncs,stats.frames,
		  stats.voice_decoded,stats.voice_muted,
		  stats.active_slot,stats.dmr_cc,
		  stats.src_call[0] ? stats.src_call : "-",stats.dst_call[0] ? stats.dst_call : "-",
		  stats.m17_encrypted ? " encrypted" : "",
		  100.0f * stats.error_rate,stats.freq_offset);
	else
	  fprintf(stderr,"%s P25 samples=%" PRIu64 " symbols=%" PRIu64
		  " syncs=%" PRIu64 " frames=%" PRIu64
		  " LDU1=%" PRIu64 " LDU2=%" PRIu64
		  " voice=%" PRIu64 " encrypted=%" PRIu64
		  " NAC=%03x BCH=%" PRIu32 " TSBK=%" PRIu64 "/%" PRIu64 " offset=%+.1f Hz\n",
		  chan->name,stats.samples,stats.symbols,stats.syncs,stats.frames,
		  stats.ldu1,stats.ldu2,stats.voice_decoded,stats.voice_muted,
		  stats.last_nac,stats.last_bch_errors,
		  stats.tsbks,stats.tsbks + stats.tsbk_errors,stats.freq_offset);
	dv_follow_print(events.follow);
      }
    }
    if(audio_queue.dropped != reported_drops){
      fprintf(stderr,"%s: %s audio queue overflowed; dropped %" PRIu64 " frames\n",
	      chan->name,label,audio_queue.dropped - reported_drops);
      reported_drops = audio_queue.dropped;
    }

    output_clock += (uint64_t)count * DV_AUDIO_SAMPRATE;
    while(output_clock >= audio_frame_clock){
      output_clock -= audio_frame_clock;
      float audio[DV_PCM_SAMPLES];
      if(analog){
	// FM's delay line advances in step with the output clock
	bool fm_open = false;
	bool const have_fm = dv_fm_take(fm,audio,&fm_open);
	bool const digital = digital_seen != 0 && sample_count - digital_seen < DV_DIGITAL_HOLD;
	fm_playing = false;
	if(audio_queue.count == 0){
	  if(have_fm && fm_open && !digital){
	    fm_playing = true;
	    idle_frames = 0;
	    if(chan->dv.source[0] == '\0')
	      strlcpy(chan->dv.source,"FM",sizeof chan->dv.source);
	    if(strcmp(chan->dv.call_mode,"FM") != 0){
	      // An FM transmission is a call of its own (no IDs)
	      struct dv_call fmcall = { .mode = "FM", .freq = chan->tune.freq, .slot = -1 };
	      fmcall.id = __atomic_add_fetch(&Call_seq,1,__ATOMIC_RELAXED);
	      publish_call(chan,&fmcall);
	    }
	    goto play;
	  }
	}
      }
      if(audio_queue.count == 0){
	// Nothing to play. A call whose decoders have finished, with its last
	// frame out 100 ms ago, has ended
	if(++idle_frames >= 5 && chan->dv.call_id != 0
	   && (decoding.id == 0 || decoding.id != chan->dv.call_id)){
	  struct dv_call const none = { .freq = chan->tune.freq, .slot = -1 };
	  publish_call(chan,&none);
	}
	chan->output.power = 0;
	send_output(chan,NULL,DV_PCM_SAMPLES,true); // keep RTP timestamps moving
	continue;
      }
      idle_frames = 0;
      publish_call(chan,&audio_queue.call[audio_queue.head]); // a new call's first frame
      {
	int16_t const * const pcm = audio_queue.samples[audio_queue.head];
	for(int i = 0; i < DV_PCM_SAMPLES; i++)
	  audio[i] = pcm[i] / 32768.0f;
	audio_queue.head = (audio_queue.head + 1) % DV_AUDIO_QUEUE_DEPTH;
	audio_queue.count--;
      }
    play:;
      if(chan->linear.agc)
	dv_agc_frame(&agc,audio,DV_PCM_SAMPLES);
      float power = 0;
      for(int i = 0; i < DV_PCM_SAMPLES; i++){
	audio[i] *= chan->output.gain;
	power += audio[i] * audio[i];
      }
      chan->output.power = power / DV_PCM_SAMPLES;
      if(send_output(chan,audio,DV_PCM_SAMPLES,false) < 0){
	restart_needed = true;
	break;
      }
    }
  }
  response(chan); // in case one is pending as we're restarting
  if(fm != NULL && fm != &fm_zero)
    free(fm);
  dv_destroy(decoder);
  dv_follow_destroy(events.follow);
  if(gate != &gate_zero)
    free(gate);
  return chan->demod_type == INVALID_DEMOD ? -1 : 0;
}
