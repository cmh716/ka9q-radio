// tone-survey: what radiod's channels actually carry, transmission by
// transmission: the CTCSS tone or DCS code of each analog one, or the mode
// and identifiers of each digital one; summed up per frequency.
//
//   tone-survey [-o results.csv] [-v] group ...
//
// For a survey: channels on the frequencies in question, without tone
// squelch ("mixed": analog FM and every digital mode), sent to a group of
// their own (frequency-survey.py --survey-config writes them). Each
// transmission (radiod sends nothing while a channel is squelched: a burst
// of audio) is judged as it ends: the tone found in most of it (monitor's
// detector: of the 55 tones, one with over half their energy, every
// 0.24 s), else the DCS code found most (radiod's decoder), else none; or,
// for a digital call, its mode and IDs (P25 NAC, talkgroup; DMR, NXDN...).
// A change of call (radiod's call ID, at the RTP timestamp status gives for
// it: FM into digital, one call into the next) ends a transmission too. Under half a second isn't judged; with no tone,
// code or call, under 1.5 s is a scrap (the FM a mixed channel plays after
// a digital call, a burst of noise) and isn't counted. The results, per frequency: transmissions,
// first and last heard, the commonest, and all with their counts.
// Rewritten every minute and at exit (SIGINT, SIGTERM).
// SPDX-License-Identifier: GPL-3.0-or-later
#define _GNU_SOURCE 1
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <opus/opus.h>
#include <samplerate.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sysexits.h>
#include <time.h>
#include <unistd.h>
#include "misc.h"
#include "multicast.h"
#include "radio.h"
#include "rtp.h"
#include "status.h"
#include "iir.h"
#include "dcs.h"

#define MAX_GROUPS 16
#define MAX_SESSIONS 1024 // frequency-survey.py --survey-config writes at most 1000
#define MAX_INFO 1024
#define SAMPRATE 8000
#define N_TONES 55

int Verbose;
char const *App_path;

static char const *Results_path = "tone-survey.csv";
static float const Tone_period = 0.24f;  // as monitor
static double const Min_seconds = 0.5;   // shorter: not judged
static double const Min_none = 1.5;      // shorter with no tone, code or call: a scrap (a mixed
                                         // channel's FM after a digital call, a noise burst), not counted
static double const Quiet_gap = 1.0;     // s without packets: the transmission is over
static double const Rtp_gap = 0.3;
// Audio is held this long before it's judged, so a call's status, should it
// come after the call's first packets, still marks where the call begins
static double const Delay = 0.5;
#define DELAY_SAMPLES 8000  // at SAMPRATE: twice Delay, and more
#define DELAY_CHUNKS 128

static float const PL_tones[N_TONES] = {
     67.0,  69.3,  71.9,  74.4,  77.0,  79.7,  82.5,  85.4,  88.5,  91.5,
     94.8,  97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3,
    131.8, 136.5, 141.3, 146.2, 150.0, 151.4, 156.7, 159.8, 162.2, 165.5,
    167.9, 171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6,
    199.5, 203.5, 206.5, 210.7, 213.8, 218.1, 221.3, 225.7, 229.1, 233.6,
    237.1, 241.8, 245.5, 250.3, 254.1
};

static double monotonic(void){
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC,&ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static int64_t wall_s(void){
  return (int64_t)time(NULL);
}

// --- what status says of each stream ---
struct stream_info {
  bool inuse;
  struct sockaddr_storage sender;
  uint32_t ssrc;
  double freq;
  int pt;
  enum encoding encoding;
  int samprate,channels;
  // The calls radiod announces (mixed: "FM" while analog plays, the mode and
  // IDs while digital), each with the RTP timestamp its audio starts at:
  // queued until the audio gets there, however status and data interleave
  bool have_last;
  uint32_t last_id;            // the last announced
  bool have_applied;
  uint32_t applied_ts;         // the last boundary reached: an announcement before it is stale
  char last_label[96];
  struct call_change { uint32_t timestamp,id; char label[96]; double received; bool untimed; } changes[16];
  int nchanges;                // (untimed: an older radiod, no call timestamps: from the audio that came after it)
  double seen;
};
static struct stream_info Info[MAX_INFO];

static struct stream_info *info(struct sockaddr_storage const *sender,uint32_t ssrc,bool create){
  struct stream_info *oldest = NULL;
  for(int i = 0; i < MAX_INFO; i++){
    struct stream_info *ip = &Info[i];
    if(ip->inuse && ip->ssrc == ssrc && address_match(&ip->sender,sender))
      return ip;
    if(oldest == NULL || !ip->inuse || (oldest->inuse && ip->seen < oldest->seen))
      oldest = ip;
  }
  if(!create)
    return NULL;
  memset(oldest,0,sizeof *oldest);
  oldest->inuse = true;
  oldest->sender = *sender;
  oldest->ssrc = ssrc;
  oldest->pt = -1;
  return oldest;
}

// --- per frequency: what was heard ---
#define MAX_FREQS 1024
#define MAX_KINDS 16
struct freqstat {
  double freq;
  int transmissions;
  int64_t first,last;
  int nkinds;
  struct { char what[64]; int count; } kinds[MAX_KINDS];
  int other;            // the transmissions of kinds past MAX_KINDS
};
static struct freqstat Freqs[MAX_FREQS];
static int Nfreqs;

static void tally(double freq,char const *what){
  if(!(freq > 0))
    return;
  struct freqstat *f = NULL;
  for(int i = 0; i < Nfreqs && f == NULL; i++)
    if(fabs(Freqs[i].freq - freq) < 1)
      f = &Freqs[i];
  if(f == NULL){
    if(Nfreqs >= MAX_FREQS)
      return;
    f = &Freqs[Nfreqs++];
    memset(f,0,sizeof *f);
    f->freq = freq;
  }
  int64_t const now = wall_s();
  f->transmissions++;
  if(f->first == 0)
    f->first = now;
  f->last = now;
  for(int k = 0; k < f->nkinds; k++){
    if(strcmp(f->kinds[k].what,what) == 0){
      f->kinds[k].count++;
      return;
    }
  }
  if(f->nkinds < MAX_KINDS){
    strlcpy(f->kinds[f->nkinds].what,what,sizeof f->kinds[0].what);
    f->kinds[f->nkinds++].count = 1;
  } else
    f->other++;
}

static char const *utc(int64_t t,char *out,size_t size){
  struct tm tm;
  time_t const tt = (time_t)t;
  gmtime_r(&tt,&tm);
  strftime(out,size,"%Y-%m-%d %H:%M:%SZ",&tm);
  return out;
}
static int by_freq(void const *a,void const *b){
  double const x = ((struct freqstat const *)a)->freq, y = ((struct freqstat const *)b)->freq;
  return x < y ? -1 : x > y;
}
static int by_count(void const *a,void const *b){
  int const x = ((int const *)a)[1], y = ((int const *)b)[1];
  return y - x;
}
static void write_results(void){
  char tmp[4096];
  snprintf(tmp,sizeof tmp,"%s.XXXXXX",Results_path);
  int const fd = mkstemp(tmp);
  FILE *fp = fd >= 0 ? fdopen(fd,"w") : NULL;
  if(fp == NULL){
    if(fd >= 0){
      close(fd);
      unlink(tmp);
    }
    fprintf(stderr,"%s: %s\n",Results_path,strerror(errno));
    return;
  }
  qsort(Freqs,(size_t)Nfreqs,sizeof Freqs[0],by_freq);
  fprintf(fp,"frequency_mhz,transmissions,first,last,commonest,heard\n");
  for(int i = 0; i < Nfreqs; i++){
    struct freqstat const *f = &Freqs[i];
    int order[MAX_KINDS][2];
    for(int k = 0; k < f->nkinds; k++){
      order[k][0] = k;
      order[k][1] = f->kinds[k].count;
    }
    qsort(order,(size_t)f->nkinds,sizeof order[0],by_count);
    char heard[1024] = "";
    for(int k = 0; k < f->nkinds; k++){
      char item[96];
      snprintf(item,sizeof item,"%s%s x%d",k ? "; " : "",f->kinds[order[k][0]].what,order[k][1]);
      strlcat(heard,item,sizeof heard);
    }
    if(f->other > 0){
      char item[64];
      snprintf(item,sizeof item,"%sother x%d",f->nkinds ? "; " : "",f->other);
      strlcat(heard,item,sizeof heard);
    }
    char a[32],b[32];
    fprintf(fp,"%.5f,%d,%s,%s,\"%s\",\"%s\"\n",f->freq * 1e-6,f->transmissions,utc(f->first,a,sizeof a),utc(f->last,b,sizeof b),
	    f->nkinds ? f->kinds[order[0][0]].what : "",heard);
  }
  bool ok = !ferror(fp);
  ok = fclose(fp) == 0 && ok;
  if(!ok || rename(tmp,Results_path) != 0){
    fprintf(stderr,"%s: can't write: %s\n",Results_path,strerror(errno));
    unlink(tmp);
  }
}

// --- a stream's transmissions ---
struct session {
  bool inuse;
  struct sockaddr_storage sender;
  uint32_t ssrc;
  int pt;
  enum encoding encoding;
  int samprate,channels;
  OpusDecoder *opus;
  SRC_STATE *src;
  double last_packet;
  bool rtp_init;
  uint16_t next_seq;
  uint32_t next_ts;
  // the transmission going on
  bool on;
  long samples;
  struct goertzel tone[N_TONES];
  int tone_fill;
  int periods;
  int tone_votes[N_TONES];
  struct dcs_decoder dcs;
  int dcs_votes;          // periods with a DCS code (the commonest kept below)
  int dcs_code[8];        // codes seen, and their periods
  bool dcs_inv[8];
  int dcs_n[8];
  int ndcs;
  // The call its audio is part of now (radiod's call ID; 0 none), and its
  // label: the mode and IDs of a digital one, "" for analog. A change of
  // call ends a transmission (FM into digital, one call into the next)
  uint32_t call_id;
  char digital[96];
  // The delay line: decoded audio, each packet's with its RTP timestamp and
  // arrival, and the ends of transmissions found in the stream, in order
  float held[DELAY_SAMPLES];
  uint64_t held_in,held_out;
  struct { uint32_t timestamp; double arrived; int n; bool end; } chunks[DELAY_CHUNKS];
  int chunk_head,nchunks;
};
static struct session Sessions[MAX_SESSIONS];

static void tx_begin(struct session *sp){
  sp->on = true;
  sp->samples = 0;
  sp->tone_fill = 0;
  sp->periods = 0;
  memset(sp->tone_votes,0,sizeof sp->tone_votes);
  for(int j = 0; j < N_TONES; j++)
    init_goertzel(&sp->tone[j],PL_tones[j] / (float)SAMPRATE);
  init_dcs_decoder(&sp->dcs,(float)SAMPRATE);
  sp->dcs_votes = 0;
  sp->ndcs = 0;
}

// The transmission is over: judged, and tallied for its frequency
static void tx_end(struct session *sp){
  if(!sp->on)
    return;
  sp->on = false;
  if((double)sp->samples / SAMPRATE < Min_seconds)
    return;
  struct stream_info const *ip = info(&sp->sender,sp->ssrc,false);
  double const freq = ip != NULL ? ip->freq : 0;
  char what[96];
  if(sp->digital[0] != '\0')
    strlcpy(what,sp->digital,sizeof what);
  else {
    int best = -1,dbest = -1;
    for(int j = 0; j < N_TONES; j++)
      if(sp->tone_votes[j] > 0 && (best < 0 || sp->tone_votes[j] > sp->tone_votes[best]))
	best = j;
    for(int k = 0; k < sp->ndcs; k++)
      if(dbest < 0 || sp->dcs_n[k] > sp->dcs_n[dbest])
	dbest = k;
    int const tv = best >= 0 ? sp->tone_votes[best] : 0;
    int const dv = dbest >= 0 ? sp->dcs_n[dbest] : 0;
    // most of the transmission: over half its looks
    if(tv * 2 > sp->periods && tv >= dv)
      snprintf(what,sizeof what,"%.1f PL",PL_tones[best]);
    else if(dv * 2 > sp->periods)
      snprintf(what,sizeof what,"%c%03d",sp->dcs_inv[dbest] ? 'I' : 'D',sp->dcs_code[dbest]);
    else {
      if((double)sp->samples / SAMPRATE < Min_none)
	return;
      strlcpy(what,"none",sizeof what);
    }
  }
  if(Verbose)
    printf("%.4f MHz  %.1f s  %s\n",freq * 1e-6,(double)sp->samples / SAMPRATE,what);
  fflush(stdout);
  tally(freq,what);
}

static void audio(struct session *sp,float const *x,int n){
  if(!sp->on)
    tx_begin(sp);
  sp->samples += n;
  for(int i = 0; i < n; i++){
    for(int j = 0; j < N_TONES; j++)
      update_goertzel(&sp->tone[j],x[i]);
    process_dcs_sample(&sp->dcs,x[i]);
    if(++sp->tone_fill >= (int)(Tone_period * SAMPRATE)){
      sp->tone_fill = 0;
      sp->periods++;
      int best = -1;
      float strongest = 0,total = 0;
      for(int j = 0; j < N_TONES; j++){
	float const e = cnrmf(output_goertzel(&sp->tone[j]));
	total += e;
	reset_goertzel(&sp->tone[j]);
	if(e > strongest){
	  strongest = e;
	  best = j;
	}
      }
      if(best >= 0 && 2 * strongest > total)
	sp->tone_votes[best]++; // (monitor's rule: over half the tones' energy)
      if(sp->dcs.detected_code != 0){
	int k;
	for(k = 0; k < sp->ndcs; k++)
	  if(sp->dcs_code[k] == sp->dcs.detected_code && sp->dcs_inv[k] == sp->dcs.detected_inverted)
	    break;
	if(k == sp->ndcs && sp->ndcs < 8){
	  sp->dcs_code[k] = sp->dcs.detected_code;
	  sp->dcs_inv[k] = sp->dcs.detected_inverted;
	  sp->dcs_n[k] = 0;
	  sp->ndcs++;
	}
	if(k < sp->ndcs)
	  sp->dcs_n[k]++;
      }
    }
  }
}

static void release_due(struct session *sp,bool all);
static void end_session(struct session *sp){
  release_due(sp,true);
  tx_end(sp);
  if(sp->opus != NULL)
    opus_decoder_destroy(sp->opus);
  if(sp->src != NULL)
    src_delete(sp->src);
  memset(sp,0,sizeof *sp);
}

static struct session *session(struct sockaddr_storage const *sender,uint32_t ssrc){
  struct session *free_one = NULL;
  for(int i = 0; i < MAX_SESSIONS; i++){
    struct session *sp = &Sessions[i];
    if(sp->inuse && sp->ssrc == ssrc && address_match(&sp->sender,sender))
      return sp;
    if(!sp->inuse && free_one == NULL)
      free_one = sp;
  }
  if(free_one == NULL){
    // the longest unheard gives way, if a minute idle
    struct session *oldest = NULL;
    for(int i = 0; i < MAX_SESSIONS; i++)
      if(!Sessions[i].on && (oldest == NULL || Sessions[i].last_packet < oldest->last_packet))
	oldest = &Sessions[i];
    if(oldest == NULL || monotonic() - oldest->last_packet < 60){
      static double warned;
      double const now = monotonic();
      if(now - warned > 60){
	fprintf(stderr,"%s: %d streams already, none idle: stream %u not surveyed\n",App_path,MAX_SESSIONS,ssrc);
	warned = now;
      }
      return NULL;
    }
    end_session(oldest);
    free_one = oldest;
  }
  memset(free_one,0,sizeof *free_one);
  free_one->inuse = true;
  free_one->sender = *sender;
  free_one->ssrc = ssrc;
  free_one->pt = -1;
  free_one->last_packet = monotonic();
  return free_one;
}

// A call change reaching the audio: another call ends the transmission;
// the same one's IDs, come late, label it
static void call_change(struct session *sp,uint32_t id,char const *label){
  if(id != sp->call_id)
    tx_end(sp);
  sp->call_id = id;
  strlcpy(sp->digital,label,sizeof sp->digital);
}

static void status(uint8_t const *buffer,int length,struct sockaddr_storage const *sender){
  if(length < 2 || buffer[0] != STATUS)
    return;
  uint32_t const ssrc = get_ssrc(buffer + 1,length - 1);
  struct stream_info *ip = info(sender,ssrc,true);
  static chan_t chan;
  static struct frontend frontend;
  memset(&chan,0,sizeof chan);
  decode_radio_status(&frontend,&chan,buffer + 1,length - 1);
  ip->seen = monotonic();
  if(chan.tune.freq > 0)
    ip->freq = chan.tune.freq;
  if(chan.output.encoding != NO_ENCODING && chan.output.samprate > 0){
    ip->pt = chan.output.rtp.type & 0x7f;
    ip->encoding = chan.output.encoding;
    ip->samprate = chan.output.samprate;
    ip->channels = chan.output.channels > 0 ? chan.output.channels : 1;
  }
  // The call: a digital one's mode and IDs (vocoded audio isn't for the
  // CTCSS filters); analog (FM) or none, ""
  char label[96] = "";
  if(chan.dv.call_mode[0] != '\0' && strcmp(chan.dv.call_mode,"FM") != 0){
    strlcpy(label,chan.dv.call_mode,sizeof label);
    if(strcmp(chan.dv.call_mode,"P25") == 0 && chan.p25.have_nac){
      char n[16];
      snprintf(n,sizeof n," NAC %03X",chan.p25.last_nac);
      strlcat(label,n,sizeof label);
    }
    if(chan.dv.call_dest[0] != '\0'){
      strlcat(label," TG ",sizeof label);
      strlcat(label,chan.dv.call_dest,sizeof label);
    }
  }
  uint32_t const id = chan.dv.call_id;
  if(ip->have_last && id == ip->last_id && strcmp(label,ip->last_label) == 0)
    return; // no change
  ip->have_last = true;
  ip->last_id = id;
  strlcpy(ip->last_label,label,sizeof ip->last_label);
  bool const untimed = !chan.dv.call_timestamp_valid;
  uint32_t const ts = chan.dv.call_timestamp;
  if(!untimed && ip->have_applied && (int32_t)(ts - ip->applied_ts) < 0)
    return; // older than a boundary already reached: delayed, stale
  if(ip->nchanges == (int)(sizeof ip->changes / sizeof ip->changes[0])){
    for(int i = 0; i < MAX_SESSIONS; i++) // full: the earliest applies now
      if(Sessions[i].inuse && Sessions[i].ssrc == ssrc && address_match(&Sessions[i].sender,sender))
	call_change(&Sessions[i],ip->changes[0].id,ip->changes[0].label);
    if(!ip->changes[0].untimed){
      ip->have_applied = true;
      ip->applied_ts = ip->changes[0].timestamp;
    }
    memmove(&ip->changes[0],&ip->changes[1],(size_t)(ip->nchanges - 1) * sizeof ip->changes[0]);
    ip->nchanges--;
  }
  // In stream order, however they came: after any at the same timestamp
  // (the same call's later IDs)
  int at = ip->nchanges;
  while(!untimed && at > 0 && !ip->changes[at - 1].untimed && (int32_t)(ts - ip->changes[at - 1].timestamp) < 0)
    at--;
  memmove(&ip->changes[at + 1],&ip->changes[at],(size_t)(ip->nchanges - at) * sizeof ip->changes[0]);
  ip->nchanges++;
  struct call_change *c = &ip->changes[at];
  c->timestamp = ts;
  c->untimed = untimed;
  c->id = id;
  strlcpy(c->label,label,sizeof c->label);
  c->received = ip->seen;
}

// The calls due by this packet: each new call a transmission of its own
static void calls_due(struct session *sp,struct stream_info *ip,uint32_t timestamp,double arrived){
  if(ip == NULL)
    return;
  int k = 0;
  // Its audio reached; or audio 5 s after it still not there (the stream's
  // clock restarted, say): it applies anyway
  // (untimed: the audio came after it)
  while(k < ip->nchanges){
    struct call_change const *c = &ip->changes[k];
    if(c->untimed ? arrived < c->received : ((int32_t)(timestamp - c->timestamp) < 0 && arrived - c->received <= 5.0))
      break;
    call_change(sp,c->id,c->label);
    if(!c->untimed){
      ip->have_applied = true;
      ip->applied_ts = c->timestamp;
    }
    k++;
  }
  if(k > 0){
    memmove(&ip->changes[0],&ip->changes[k],(size_t)(ip->nchanges - k) * sizeof ip->changes[0]);
    ip->nchanges -= k;
  }
}

// Into the delay line: audio (n > 0) from the packet at timestamp, or the end
// of a transmission (end)
static void release(struct session *sp);
static void hold(struct session *sp,uint32_t timestamp,float const *x,int n,bool end){
  if(n > DELAY_SAMPLES)
    n = DELAY_SAMPLES;
  while(sp->nchunks == DELAY_CHUNKS || sp->held_in - sp->held_out + (uint64_t)n > DELAY_SAMPLES)
    release(sp); // full: the oldest goes now
  int const k = (sp->chunk_head + sp->nchunks++) % DELAY_CHUNKS;
  sp->chunks[k].timestamp = timestamp;
  sp->chunks[k].arrived = monotonic();
  sp->chunks[k].n = n;
  sp->chunks[k].end = end;
  for(int i = 0; i < n; i++)
    sp->held[(sp->held_in + (uint64_t)i) % DELAY_SAMPLES] = x[i];
  sp->held_in += (uint64_t)n;
}
// The oldest out of the delay line: the calls due by its timestamp first
static void release(struct session *sp){
  if(sp->nchunks == 0)
    return;
  int const k = sp->chunk_head;
  sp->chunk_head = (sp->chunk_head + 1) % DELAY_CHUNKS;
  sp->nchunks--;
  if(sp->chunks[k].end){
    tx_end(sp);
    return;
  }
  calls_due(sp,info(&sp->sender,sp->ssrc,false),sp->chunks[k].timestamp,sp->chunks[k].arrived);
  float x[DELAY_SAMPLES];
  int const n = sp->chunks[k].n;
  for(int i = 0; i < n; i++)
    x[i] = sp->held[(sp->held_out + (uint64_t)i) % DELAY_SAMPLES];
  sp->held_out += (uint64_t)n;
  audio(sp,x,n);
}
// What has been held long enough (all of it: at the end)
static void release_due(struct session *sp,bool all){
  double const now = monotonic();
  while(sp->nchunks > 0 && (all || now - sp->chunks[sp->chunk_head].arrived >= Delay))
    release(sp);
}

static void expect(struct session *sp,struct rtp_header const *rtp,uint32_t ts_units){
  sp->next_seq = (uint16_t)(rtp->seq + 1);
  sp->next_ts = rtp->timestamp + ts_units;
  sp->rtp_init = true;
}

static void data(uint8_t *buffer,ssize_t size,struct sockaddr_storage const *sender){
  if(size <= RTP_MIN_SIZE || (buffer[0] >> 6) != 2)
    return;
  ssize_t header = RTP_MIN_SIZE + 4 * (buffer[0] & 0x0f);
  if(buffer[0] & 0x10){
    if(header + 4 > size)
      return;
    header += 4 + 4 * (ssize_t)(buffer[header + 2] << 8 | buffer[header + 3]);
  }
  if(header >= size)
    return;
  ssize_t payload = size - header;
  if(buffer[0] & 0x20){
    int const pad = buffer[size - 1];
    if(pad == 0 || pad > payload)
      return;
    payload -= pad;
  }
  if(payload <= 0)
    return;
  struct rtp_header rtp;
  uint8_t const *dp = ntoh_rtp(&rtp,buffer);
  struct session *sp = session(sender,rtp.ssrc);
  if(sp == NULL)
    return;
  struct stream_info *ip = info(sender,rtp.ssrc,false);
  if(ip != NULL && ip->pt >= 0){
    sp->pt = ip->pt;
    sp->encoding = ip->encoding;
    sp->samprate = ip->samprate;
    sp->channels = ip->channels;
  }
  double const now = monotonic();
  if(now - sp->last_packet > Quiet_gap){
    hold(sp,0,NULL,0,true);
    sp->rtp_init = false; // the next transmission's numbering is its own
  }
  sp->last_packet = now;
  enum encoding encoding = sp->pt == rtp.type ? sp->encoding : encoding_from_pt(rtp.type);
  int samprate = sp->pt == rtp.type ? sp->samprate : samprate_from_pt(rtp.type);
  int channels = sp->pt == rtp.type ? sp->channels : channels_from_pt(rtp.type);
  if(encoding == NO_ENCODING && rtp.type == Opus_pt){
    encoding = OPUS;
    samprate = 48000;
    channels = 1;
  }
  bool const opus = encoding == OPUS || encoding == OPUS_VOIP;
  int const clock = opus ? 48000 : samprate;
  // A jump in the stream of more than Rtp_gap: a new transmission; late or
  // repeated packets dropped
  if(sp->rtp_init && clock > 0){
    int16_t const seq_step = (int16_t)(rtp.seq - sp->next_seq);
    int32_t const ts_step = (int32_t)(rtp.timestamp - sp->next_ts);
    if(seq_step < 0 || ts_step < 0)
      return;
    if((double)ts_step / clock > Rtp_gap)
      hold(sp,0,NULL,0,true);
  }
  float out[2880 * 2];
  if(opus){
    if(sp->opus == NULL){
      int error;
      sp->opus = opus_decoder_create(SAMPRATE,1,&error);
      if(sp->opus == NULL)
	return;
    }
    int const n = opus_decode_float(sp->opus,dp,(opus_int32)payload,out,2880,0);
    if(n > 0){
      hold(sp,rtp.timestamp,out,n,false);
      expect(sp,&rtp,(uint32_t)n * (48000 / SAMPRATE));
    } else
      expect(sp,&rtp,0);
    return;
  }
  if(samprate <= 0 || channels <= 0)
    return;
  int bytes;
  switch(encoding){
  case S16BE: case S16LE: bytes = 2; break;
  case F32LE: bytes = 4; break;
  default: return;
  }
  int const frames = (int)(payload / (bytes * channels));
  float in[4096];
  int const nin = frames < 4096 ? frames : 4096;
  for(int i = 0; i < nin; i++){
    uint8_t const *p = dp + (size_t)i * bytes * channels;
    if(encoding == S16BE)
      in[i] = (int16_t)(p[0] << 8 | p[1]) * 0x1p-15f;
    else if(encoding == S16LE)
      in[i] = (int16_t)(p[1] << 8 | p[0]) * 0x1p-15f;
    else {
      float f;
      memcpy(&f,p,4);
      in[i] = f;
    }
  }
  expect(sp,&rtp,(uint32_t)frames);
  if(samprate == SAMPRATE){
    hold(sp,rtp.timestamp,in,nin,false);
    return;
  }
  if(sp->src == NULL){
    int error;
    sp->src = src_new(SRC_SINC_FASTEST,1,&error);
    if(sp->src == NULL)
      return;
  }
  SRC_DATA d = {.data_in = in,.input_frames = nin,.data_out = out,.output_frames = (long)(sizeof out / sizeof out[0]),
		.src_ratio = (double)SAMPRATE / samprate,.end_of_input = 0};
  if(src_process(sp->src,&d) == 0 && d.output_frames_gen > 0)
    hold(sp,rtp.timestamp,out,(int)d.output_frames_gen,false);
}

static volatile sig_atomic_t Stop;
static void stop(int s){ (void)s; Stop = 1; }

static void usage(void){
  fprintf(stderr,"Usage: %s [-o results.csv] [-v] group ...\n",App_path);
}

int main(int argc,char *argv[]){
  App_path = argv[0];
  int c;
  while((c = getopt(argc,argv,"o:v")) != -1){
    switch(c){
    case 'o': Results_path = optarg; break;
    case 'v': Verbose++; break;
    default: usage(); exit(EX_USAGE);
    }
  }
  if(optind >= argc){
    usage();
    exit(EX_USAGE);
  }
  int fds[2 * MAX_GROUPS];
  bool is_status[2 * MAX_GROUPS];
  int nfds = 0, maxfd = 0;
  for(int i = optind; i < argc && nfds + 2 <= 2 * MAX_GROUPS; i++){
    for(int s = 0; s < 2; s++){
      struct sockaddr_storage sock;
      char iface[1024] = {0};
      if(resolve_mcast(argv[i],&sock,s ? DEFAULT_STAT_PORT : DEFAULT_RTP_PORT,iface,sizeof iface,0) != 0){
	fprintf(stderr,"%s: can't resolve\n",argv[i]);
	break;
      }
      int const fd = listen_mcast(NULL,&sock,iface);
      if(fd < 0)
	continue;
      fds[nfds] = fd;
      is_status[nfds++] = s;
      if(fd > maxfd)
	maxfd = fd;
    }
  }
  if(nfds == 0)
    exit(EX_UNAVAILABLE);
  signal(SIGINT,stop);
  signal(SIGTERM,stop);
  fprintf(stderr,"%s: watching %d group%s; results in %s\n",App_path,nfds / 2,nfds / 2 == 1 ? "" : "s",Results_path);
  uint8_t buffer[PKTSIZE];
  double last_write = monotonic();
  while(!Stop){
    fd_set set;
    FD_ZERO(&set);
    for(int i = 0; i < nfds; i++)
      FD_SET(fds[i],&set);
    struct timeval tv = {.tv_sec = 0,.tv_usec = 50000}; // the delay line drained this often
    int const r = select(maxfd + 1,&set,NULL,NULL,&tv);
    if(r < 0 && errno != EINTR)
      break;
    // Status before data: a call's announcement ahead of its audio, where both wait
    for(int j = 0; r > 0 && j < 2 * nfds; j++){
      int const i = j % nfds;
      if(!FD_ISSET(fds[i],&set) || is_status[i] != (j < nfds))
	continue;
      struct sockaddr_storage sender;
      socklen_t len = sizeof sender;
      ssize_t const size = recvfrom(fds[i],buffer,sizeof buffer,0,(struct sockaddr *)&sender,&len);
      if(size <= 0)
	continue;
      if(is_status[i])
	status(buffer,(int)size,&sender);
      else
	data(buffer,size,&sender);
    }
    double const now = monotonic();
    for(int k = 0; k < MAX_SESSIONS; k++){
      struct session *sp = &Sessions[k];
      if(!sp->inuse)
	continue;
      release_due(sp,false);
      if(sp->on && sp->nchunks == 0 && now - sp->last_packet > Quiet_gap){
	tx_end(sp); // its channel squelched: the transmission is over
	sp->rtp_init = false;
      }
    }
    if(now - last_write > 60){
      write_results();
      last_write = now;
    }
  }
  for(int k = 0; k < MAX_SESSIONS; k++)
    if(Sessions[k].inuse){
      release_due(&Sessions[k],true);
      tx_end(&Sessions[k]);
    }
  write_results();
  return 0;
}
