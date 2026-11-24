// waterfall: a web spectrum and waterfall of a radiod front end's whole
// coverage, zoomable, with its channels marked.
//
//   waterfall [-p port] [-H page.html] [-r repeaters.d] [-v] radiod-status-group
//
// A small HTTP server, no libraries: each browser that opens the page gets a
// spectrum channel of its own in radiod (one-byte bins, SPECT2) for the view
// it asks for, polled about five times a second and sent to it as
// Server-Sent Events; it's closed when the browser goes. radiod's channels
// (polled every few seconds) are sent for markers, named from repeaters.d.
//
//   GET /                         the page
//   GET /info                     the front end: {"center":Hz,"samprate":Hz,"description":...}
//   GET /channels                 [{"ssrc","freq","mode","active","call","name"}...]
//   GET /stream?center=&span=&bins=   text/event-stream, a spectrum each event:
//                                 {"start":Hz,"rbw":Hz,"n":bins,"base":dB,"step":dB,"bins":base64}
//                                 bins low to high frequency, dB = base + byte * step
//
// Copyright 2026 ka9q-radio contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#define _GNU_SOURCE 1
#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sysexits.h>
#include <unistd.h>

#include "misc.h"
#include "status.h"
#include "multicast.h"
#include "radio.h"
#include "database.h"
#include "timing.h"

int Verbose;
char const *App_path;

static int Port = 8090;
static char const *Page = "/usr/local/share/ka9q-radio/waterfall.html";
static char const *Repeaters = "/var/lib/ka9q-radio/repeaters.d";
static char const *Target;            // radiod's status group
static struct sockaddr_storage Ctl_dest;
static char Iface[1024];
static int Wf_ctl_fd = -1, Wf_status_fd = -1;

#define MAX_CLIENTS 8
#define MAX_BINS 8192
#define MAX_CHANS 512
#define FRAME_MS 200                  // a spectrum every 200 ms
#define MAX_CONNECTIONS 32            // HTTP connections at once
#define IO_TIMEOUT_S 5                // to read a request, or write anything
#define GUARD_HZ 25e3                 // views kept this far inside the front end's coverage (or 2% of it, if less)

// The front end, from any status
static pthread_mutex_t Fe_lock = PTHREAD_MUTEX_INITIALIZER;
static double Fe_center, Fe_samprate;
static double Fe_min_if, Fe_max_if;   // its coverage about its centre (0, 0: not yet known)
static double Fe_blocktime;           // radiod's block, s (0: not yet known)
static bool Fe_isreal;
static char Fe_description[128];
static _Atomic int Connections;

// The range a view may take, Hz: the front end's coverage as it is tuned
// now, less a guard, so that no spectrum channel's passband ever falls
// outside it (radiod would retune the hardware for it, moving every other
// channel). false until the front end is known
static bool coverage(double *lo,double *hi){
  pthread_mutex_lock(&Fe_lock);
  double min_if = Fe_min_if, max_if = Fe_max_if;
  if(!(max_if > min_if) && Fe_samprate > 0 && !Fe_isreal){
    min_if = -0.47 * Fe_samprate; // (a front end that doesn't give its edges)
    max_if = +0.47 * Fe_samprate;
  }
  double const guard = fmin(GUARD_HZ,0.02 * (max_if - min_if));
  bool const ok = Fe_samprate > 0 && max_if > min_if; // (its LO may be 0 Hz)
  *lo = Fe_center + min_if + guard;
  *hi = Fe_center + max_if - guard;
  pthread_mutex_unlock(&Fe_lock);
  return ok;
}
// A lifetime for a spectrum channel, in radiod's blocks
static int blocks(double seconds){
  pthread_mutex_lock(&Fe_lock);
  double const bt = Fe_blocktime > 0 ? Fe_blocktime : 0.02;
  pthread_mutex_unlock(&Fe_lock);
  int const n = (int)lrint(seconds / bt);
  return n < 1 ? 1 : n;
}

// A browser's view: its spectrum channel, and the latest spectrum radiod sent it
struct client {
  bool used;
  uint32_t ssrc;
  pthread_mutex_t lock;
  pthread_cond_t cond;
  bool fresh;
  uint32_t tag;                       // the command the latest answers
  double freq,rbw;
  int n;
  float base,step;
  uint8_t bins[MAX_BINS];
};
static struct client Clients[MAX_CLIENTS];
static pthread_mutex_t Clients_lock = PTHREAD_MUTEX_INITIALIZER;

// radiod's channels, for markers
struct chaninfo {
  uint32_t ssrc;
  double freq;
  char mode[16];
  bool active;
  char call[48];
  char name[48];
  int64_t seen;
};
static struct chaninfo Chans[MAX_CHANS];
// Others' spectrum channels heard (another waterfall's viewers, powers,
// ka9q-web): their SSRCs, avoided for ours, for ten minutes
#define MAX_SPECTRA 256
static struct { uint32_t ssrc; int64_t seen; } Spectra[MAX_SPECTRA];
static int Nchans;
static pthread_mutex_t Chans_lock = PTHREAD_MUTEX_INITIALIZER;

// --- status from radiod ---

// The one-byte bins, which decode_radio_status() leaves alone
static int byte_bins(uint8_t const *buffer,int length,uint8_t *out,int max){
  uint8_t const *cp = buffer;
  while(cp - buffer < length){
    enum status_type const type = (enum status_type)*cp++;
    if(type == EOL || cp - buffer >= length)
      break;
    unsigned optlen = *cp++;
    if(optlen & 0x80){
      int const lol = optlen & 0x7f;
      optlen = 0;
      for(int i = 0; i < lol && cp - buffer < length; i++)
	optlen = (optlen << 8) | *cp++;
    }
    if(cp + optlen > buffer + length)
      break;
    if(type == BIN_BYTE_DATA){
      int const n = (int)optlen < max ? (int)optlen : max;
      memcpy(out,cp,(size_t)n);
      return n;
    }
    cp += optlen;
  }
  return 0;
}

static void *status_thread(void *arg){
  (void)arg;
  pthread_setname("wf status");
  static uint8_t buffer[PKTSIZE];
  static struct frontend fe;
  static chan_t chan;
  while(true){
    ssize_t const length = recv(Wf_status_fd,buffer,sizeof buffer,0);
    if(length < 2 || (enum pkt_type)buffer[0] != STATUS)
      continue;
    memset(&fe,0,sizeof fe);
    memset(&chan,0,sizeof chan);
    chan.spectrum.base = NAN;
    chan.spectrum.step = NAN;
    chan.output.power = NAN;
    decode_radio_status(&fe,&chan,buffer + 1,(int)length - 1);
    if(fe.samprate > 0){ // (an LO of 0 Hz is valid: an HF front end's)
      pthread_mutex_lock(&Fe_lock);
      Fe_center = fe.frequency;
      Fe_samprate = fe.samprate;
      Fe_isreal = fe.isreal;
      if(fe.max_IF > fe.min_IF){
	Fe_min_if = fe.min_IF;
	Fe_max_if = fe.max_IF;
      }
      if(fe.L > 0)
	Fe_blocktime = fe.L / fe.samprate;
      strlcpy(Fe_description,fe.description,sizeof Fe_description);
      pthread_mutex_unlock(&Fe_lock);
    }
    uint32_t const ssrc = chan.output.rtp.ssrc;
    if(ssrc == 0)
      continue; // the front end alone
    // A spectrum for one of ours? Found and given to it under the clients'
    // lock, so its slot can't go to another viewer in between
    bool ours = false;
    static uint8_t bins[MAX_BINS];
    int const n = byte_bins(buffer + 1,(int)length - 1,bins,MAX_BINS);
    pthread_mutex_lock(&Clients_lock);
    for(int i = 0; i < MAX_CLIENTS && !ours; i++){
      struct client * const c = &Clients[i];
      if(!c->used || c->ssrc != ssrc)
	continue;
      ours = true;
      if(n > 0 && isfinite(chan.spectrum.base) && isfinite(chan.spectrum.step)){
	pthread_mutex_lock(&c->lock);
	c->tag = get_tag(buffer + 1,(int)length - 1);
	c->freq = chan.tune.freq;
	c->rbw = chan.spectrum.rbw;
	c->n = n;
	c->base = chan.spectrum.base;
	c->step = chan.spectrum.step;
	memcpy(c->bins,bins,(size_t)n);
	c->fresh = true;
	pthread_cond_broadcast(&c->cond);
	pthread_mutex_unlock(&c->lock);
      }
    }
    bool const spectrum = chan.demod_type == SPECT_DEMOD || chan.demod_type == SPECT2_DEMOD;
    if(!ours && spectrum){
      // Another's spectrum channel: its SSRC taken, recorded before the
      // clients' lock goes (in the allocator's order: the clients', then the
      // channels'), so no viewer made meanwhile can take it
      int64_t const now = gps_time_ns();
      pthread_mutex_lock(&Chans_lock);
      int k = 0;
      for(int i = 0; i < MAX_SPECTRA; i++){
	if(Spectra[i].ssrc == ssrc){
	  k = i;
	  break;
	}
	if(Spectra[i].seen < Spectra[k].seen)
	  k = i; // else the oldest (or unused) entry
      }
      Spectra[k].ssrc = ssrc;
      Spectra[k].seen = now;
      pthread_mutex_unlock(&Chans_lock);
    }
    pthread_mutex_unlock(&Clients_lock);
    if(ours || spectrum)
      continue; // (others' spectrum channels aren't channels to mark)
    if(demod_name_from_type(chan.demod_type) == NULL)
      continue; // no demodulator (a channel timing out says so last): nothing to mark
    if(!(chan.tune.freq > 0))
      continue;
    // A channel: remember it for the markers
    pthread_mutex_lock(&Chans_lock);
    int k = -1;
    for(int i = 0; i < Nchans; i++)
      if(Chans[i].ssrc == ssrc)
	k = i;
    if(k < 0 && Nchans < MAX_CHANS)
      k = Nchans++;
    if(k >= 0){
      struct chaninfo *ci = &Chans[k];
      ci->ssrc = ssrc;
      ci->freq = chan.tune.freq;
      char const *mode = demod_name_from_type(chan.demod_type);
      strlcpy(ci->mode,mode != NULL ? mode : "?",sizeof ci->mode);
      ci->active = isfinite(chan.output.power) || chan.dv.call_dest[0] != '\0';
      snprintf(ci->call,sizeof ci->call,"%s%s%s",chan.dv.call_dest,chan.dv.call_src[0] ? " " : "",chan.dv.call_src);
      repeater_t const *r = search_database((int)lrint(chan.tune.freq),0);
      strlcpy(ci->name,r != NULL && r->callsign != NULL ? r->callsign : "",sizeof ci->name);
      ci->seen = gps_time_ns();
    }
    pthread_mutex_unlock(&Chans_lock);
  }
  return NULL;
}

static void send_command(uint8_t *buffer,uint8_t *bp){
  encode_eol(&bp);
  socklen_t const slen = Ctl_dest.ss_family == AF_INET ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6);
  if(sendto(Wf_ctl_fd,buffer,(size_t)(bp - buffer),0,(struct sockaddr *)&Ctl_dest,slen) < 0 && Verbose)
    perror("command send");
}

// Every few seconds: every channel's status (radiod staggers the answers)
static void *channel_poll_thread(void *arg){
  (void)arg;
  pthread_setname("wf poll");
  while(true){
    // The front end's own status (SSRC 0), heard even with no channel running
    uint8_t buffer[64],*bp = buffer;
    *bp++ = CMD;
    encode_int(&bp,OUTPUT_SSRC,0);
    encode_int(&bp,COMMAND_TAG,(uint32_t)random());
    send_command(buffer,bp);
    bp = buffer;
    *bp++ = CMD;
    encode_int(&bp,OUTPUT_SSRC,0xffffffff);
    encode_int(&bp,COMMAND_TAG,(uint32_t)random());
    send_command(buffer,bp);
    sleep(3);
  }
  return NULL;
}

// --- HTTP ---

static bool write_all(int fd,char const *buf,size_t len){
  while(len > 0){
    ssize_t const r = send(fd,buf,len,MSG_NOSIGNAL);
    if(r <= 0)
      return false;
    buf += r;
    len -= (size_t)r;
  }
  return true;
}
static bool write_str(int fd,char const *s){
  return write_all(fd,s,strlen(s));
}
static void reply(int fd,char const *status,char const *type,char const *body,size_t len){
  char head[256];
  snprintf(head,sizeof head,"HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n",
	   status,type,len);
  if(write_str(fd,head))
    write_all(fd,body,len);
}
// JSON string contents, escaped
static void json_escape(char *out,size_t size,char const *in){
  size_t k = 0;
  for(; *in != '\0' && k + 7 < size; in++){
    unsigned char const ch = (unsigned char)*in;
    if(ch == '"' || ch == '\\'){
      out[k++] = '\\';
      out[k++] = (char)ch;
    } else if(ch < 0x20)
      k += (size_t)snprintf(out + k,size - k,"\\u%04x",ch);
    else
      out[k++] = (char)ch;
  }
  out[k] = '\0';
}
// A query parameter's value as a number: the default if absent, NAN if
// it isn't wholly a finite number
static double query_number(char const *query,char const *name,double dflt){
  if(query == NULL)
    return dflt;
  size_t const len = strlen(name);
  for(char const *p = query; p != NULL && *p != '\0'; p = strchr(p,'&') ? strchr(p,'&') + 1 : NULL){
    if(strncmp(p,name,len) == 0 && p[len] == '='){
      char *end = NULL;
      double const x = strtod(p + len + 1,&end);
      if(end == p + len + 1 || (*end != '\0' && *end != '&') || !isfinite(x))
	return NAN;
      return x;
    }
  }
  return dflt;
}
static void base64(char *out,uint8_t const *in,int n){
  static char const tab[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int k = 0;
  for(int i = 0; i < n; i += 3){
    uint32_t const v = (uint32_t)in[i] << 16 | (i + 1 < n ? (uint32_t)in[i + 1] << 8 : 0) | (i + 2 < n ? in[i + 2] : 0);
    out[k++] = tab[(v >> 18) & 63];
    out[k++] = tab[(v >> 12) & 63];
    out[k++] = i + 1 < n ? tab[(v >> 6) & 63] : '=';
    out[k++] = i + 2 < n ? tab[v & 63] : '=';
  }
  out[k] = '\0';
}

static void serve_page(int fd){
  FILE *fp = fopen(Page,"r");
  if(fp == NULL){
    char const msg[] = "waterfall: the page isn't installed\n";
    reply(fd,"404 Not Found","text/plain",msg,sizeof msg - 1);
    return;
  }
  size_t const size = 1 << 20;
  char *body = malloc(size); // its own: another request's mustn't change it as it goes
  if(body == NULL){
    fclose(fp);
    return;
  }
  size_t const len = fread(body,1,size,fp);
  fclose(fp);
  reply(fd,"200 OK","text/html; charset=utf-8",body,len);
  free(body);
}

static void serve_info(int fd){
  char body[512],desc[256];
  double lo = 0,hi = 0;
  bool const known = coverage(&lo,&hi);
  pthread_mutex_lock(&Fe_lock);
  json_escape(desc,sizeof desc,Fe_description);
  snprintf(body,sizeof body,"{\"center\":%.0f,\"samprate\":%.0f,\"real\":%s,\"lo\":%.0f,\"hi\":%.0f,\"description\":\"%s\"}",
	   Fe_center,Fe_samprate,Fe_isreal ? "true" : "false",known ? lo : 0,known ? hi : 0,desc);
  pthread_mutex_unlock(&Fe_lock);
  reply(fd,"200 OK","application/json",body,strlen(body));
}

static void serve_channels(int fd){
  enum { SIZE = MAX_CHANS * 256 };
  char *body = malloc(SIZE); // its own (as the page's)
  if(body == NULL)
    return;
  size_t k = 0;
  body[k++] = '[';
  int64_t const now = gps_time_ns();
  pthread_mutex_lock(&Chans_lock);
  bool first = true;
  for(int i = 0; i < Nchans; i++){
    struct chaninfo const *ci = &Chans[i];
    if(now - ci->seen > 30 * (int64_t)BILLION)
      continue; // gone
    char call[128],name[128];
    json_escape(call,sizeof call,ci->call);
    json_escape(name,sizeof name,ci->name);
    k += (size_t)snprintf(body + k,SIZE - k,"%s{\"ssrc\":%u,\"freq\":%.0f,\"mode\":\"%s\",\"active\":%s,\"call\":\"%s\",\"name\":\"%s\"}",
			  first ? "" : ",",ci->ssrc,ci->freq,ci->mode,ci->active ? "true" : "false",call,name);
    first = false;
    if(k > SIZE - 512)
      break;
  }
  pthread_mutex_unlock(&Chans_lock);
  body[k++] = ']';
  reply(fd,"200 OK","application/json",body,k);
  free(body);
}

// A browser's view, as Server-Sent Events, until it goes
static void serve_stream(int fd,char const *query){
  double center = query_number(query,"center",NAN);
  double span = query_number(query,"span",NAN);
  double const want_bins = query_number(query,"bins",2048);
  if(!isfinite(center) || !isfinite(span) || !isfinite(want_bins) || span < 1e3 || want_bins < 64 || want_bins > MAX_BINS){
    char const msg[] = "center and span (Hz, span at least 1 kHz), bins (64-8192) wanted\n";
    reply(fd,"400 Bad Request","text/plain",msg,sizeof msg - 1);
    return;
  }
  // Within the front end's coverage as tuned now: never a view that would
  // have radiod move the hardware (and every other channel)
  double lo,hi;
  if(!coverage(&lo,&hi)){
    char const msg[] = "radiod's front end not heard from yet\n";
    reply(fd,"503 Service Unavailable","text/plain",msg,sizeof msg - 1);
    return;
  }
  if(span > hi - lo)
    span = hi - lo;
  if(center < lo + span / 2)
    center = lo + span / 2;
  if(center > hi - span / 2)
    center = hi - span / 2;
  int const bins = (int)want_bins & ~1; // even: radiod's one-byte bins run from -n/2 for an even count
  double const rbw = span / bins;
  // A spectrum channel of its own
  struct client *c = NULL;
  pthread_mutex_lock(&Clients_lock);
  for(int i = 0; i < MAX_CLIENTS && c == NULL; i++){
    if(!Clients[i].used){
      c = &Clients[i];
      c->used = true;
      // Its SSRC: random in the upper half of the range, where radiod's own
      // channels aren't (theirs are their frequencies in kHz), and none seen
      // among radiod's channels or our other viewers. (radiod would take a
      // command to a channel it has as for that channel: another waterfall's
      // viewer colliding is a chance of about one in 2^31 per pair)
      uint32_t ssrc;
      bool clash;
      do {
	ssrc = 0x80000000u | (uint32_t)random() << 1 | (uint32_t)(random() & 1);
	clash = false;
	for(int j = 0; j < MAX_CLIENTS; j++)
	  clash = clash || (Clients[j].used && Clients[j].ssrc == ssrc);
	int64_t const now = gps_time_ns();
	pthread_mutex_lock(&Chans_lock);
	for(int j = 0; j < Nchans; j++)
	  clash = clash || Chans[j].ssrc == ssrc;
	for(int j = 0; j < MAX_SPECTRA; j++)
	  clash = clash || (Spectra[j].ssrc == ssrc && now - Spectra[j].seen < 600 * (int64_t)BILLION);
	pthread_mutex_unlock(&Chans_lock);
      } while(clash || ssrc == 0xffffffffu);
      c->ssrc = ssrc;
      pthread_mutex_lock(&c->lock); // (lock order: the clients', then a client's)
      c->fresh = false;
      pthread_mutex_unlock(&c->lock);
    }
  }
  pthread_mutex_unlock(&Clients_lock);
  if(c == NULL){
    char const msg[] = "too many viewers\n";
    reply(fd,"503 Service Unavailable","text/plain",msg,sizeof msg - 1);
    return;
  }
  if(Verbose)
    fprintf(stderr,"viewer ssrc %u: %.4f MHz +/- %.4f MHz, %d bins of %.0f Hz\n",c->ssrc,center * 1e-6,span * 0.5e-6,bins,rbw);
  bool ok = write_str(fd,"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache\r\nConnection: keep-alive\r\n\r\n");
  static __thread uint8_t ordered[MAX_BINS];
  static __thread char b64[MAX_BINS * 4 / 3 + 8];
  static __thread char event[MAX_BINS * 4 / 3 + 512];
  while(ok){
    int64_t const started = gps_time_ns();
    uint8_t buffer[256],*bp = buffer;
    *bp++ = CMD;
    uint32_t const tag = (uint32_t)random();
    encode_int(&bp,OUTPUT_SSRC,c->ssrc);
    encode_int(&bp,COMMAND_TAG,tag);
    encode_int(&bp,DEMOD_TYPE,SPECT2_DEMOD);
    encode_int(&bp,LIFETIME,blocks(2)); // 2 s without a poll (a viewer gone): radiod closes it
    encode_double(&bp,RADIO_FREQUENCY,center);
    encode_int(&bp,BIN_COUNT,bins);
    encode_float(&bp,RESOLUTION_BW,(float)rbw);
    send_command(buffer,bp);
    // Its answer, up to a second
    struct timespec until;
    clock_gettime(CLOCK_REALTIME,&until);
    until.tv_sec += 1;
    pthread_mutex_lock(&c->lock);
    while(!(c->fresh && c->tag == tag)){
      if(pthread_cond_timedwait(&c->cond,&c->lock,&until) != 0)
	break;
    }
    bool const have = c->fresh && c->tag == tag;
    int n = 0;
    double freq = 0,got_rbw = 0;
    float base = 0,step = 0;
    if(have){
      c->fresh = false;
      n = c->n;
      freq = c->freq;
      got_rbw = c->rbw;
      base = c->base;
      step = c->step;
      // radiod sends one-byte bins low to high already (encode_byte_data)
      memcpy(ordered,c->bins,(size_t)n);
    }
    pthread_mutex_unlock(&c->lock);
    if(have && n > 0 && got_rbw > 0){
      base64(b64,ordered,n);
      double const start = freq - got_rbw * (n / 2);
      int const len = snprintf(event,sizeof event,"data: {\"start\":%.1f,\"rbw\":%.3f,\"n\":%d,\"base\":%.2f,\"step\":%.4f,\"bins\":\"%s\"}\n\n",
			       start,got_rbw,n,base,step,b64);
      ok = write_all(fd,event,(size_t)len);
    } else
      ok = write_str(fd,": waiting\n\n"); // (a comment: keeps the connection, finds a closed one)
    int64_t const left = FRAME_MS * 1000000LL - (gps_time_ns() - started);
    if(left > 0)
      usleep((useconds_t)(left / 1000));
  }
  // Gone: its spectrum channel ends with its lifetime, unpolled. (No
  // command to end it: radiod makes a channel for a command to an SSRC it
  // doesn't have, which one sent after it had already ended would do)
  if(Verbose)
    fprintf(stderr,"viewer ssrc %u gone\n",c->ssrc);
  pthread_mutex_lock(&Clients_lock);
  c->used = false;
  pthread_mutex_unlock(&Clients_lock);
}

static void *http_thread(void *arg){
  int const fd = (int)(intptr_t)arg;
  pthread_setname("wf http");
  // Deadlines: a request must come within them, and a write that can't
  // go (a stalled viewer) fails, freeing its thread and viewer slot
  struct timeval const tv = { .tv_sec = IO_TIMEOUT_S };
  setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof tv);
  setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&tv,sizeof tv);
  char req[4096];
  size_t len = 0;
  int64_t const deadline = gps_time_ns() + IO_TIMEOUT_S * (int64_t)BILLION; // the whole request, however it trickles
  while(len < sizeof req - 1 && gps_time_ns() < deadline){
    ssize_t const r = recv(fd,req + len,sizeof req - 1 - len,0);
    if(r <= 0)
      break;
    len += (size_t)r;
    req[len] = '\0';
    if(strstr(req,"\r\n\r\n") != NULL)
      break;
  }
  req[len] = '\0';
  char method[8] = "",path[1024] = "";
  if(strstr(req,"\r\n\r\n") == NULL){
    // Incomplete (timed out, closed, or too long): not routed
  } else if(sscanf(req,"%7s %1023s",method,path) == 2 && strcmp(method,"GET") == 0){
    char *query = strchr(path,'?');
    if(query != NULL)
      *query++ = '\0';
    if(strcmp(path,"/") == 0 || strcmp(path,"/index.html") == 0)
      serve_page(fd);
    else if(strcmp(path,"/info") == 0)
      serve_info(fd);
    else if(strcmp(path,"/channels") == 0)
      serve_channels(fd);
    else if(strcmp(path,"/stream") == 0)
      serve_stream(fd,query);
    else {
      char const msg[] = "not found\n";
      reply(fd,"404 Not Found","text/plain",msg,sizeof msg - 1);
    }
  }
  close(fd);
  Connections--;
  return NULL;
}

static void usage(void){
  fprintf(stderr,"Usage: %s [-p port] [-H page.html] [-r repeaters.d] [-v] radiod-status-group\n",App_path);
}

int main(int argc,char *argv[]){
  App_path = argv[0];
  int ch;
  while((ch = getopt(argc,argv,"p:H:r:v")) != -1){
    switch(ch){
    case 'p': Port = atoi(optarg); break;
    case 'H': Page = optarg; break;
    case 'r': Repeaters = optarg; break;
    case 'v': Verbose++; break;
    default: usage(); exit(EX_USAGE);
    }
  }
  if(optind >= argc){
    usage();
    exit(EX_USAGE);
  }
  Target = argv[optind];
  signal(SIGPIPE,SIG_IGN);
  srandom((unsigned)gps_time_ns());
  for(int i = 0; i < MAX_CLIENTS; i++){
    pthread_mutex_init(&Clients[i].lock,NULL);
    pthread_cond_init(&Clients[i].cond,NULL);
  }
  load_databases(Repeaters,0,0); // names for the markers (none if absent)

  resolve_mcast(Target,&Ctl_dest,DEFAULT_STAT_PORT,Iface,sizeof Iface,0);
  Wf_status_fd = listen_mcast(NULL,&Ctl_dest,Iface[0] ? Iface : NULL);
  Wf_ctl_fd = output_mcast(&Ctl_dest,Iface[0] ? Iface : NULL,1,0);
  if(Wf_status_fd < 0 || Wf_ctl_fd < 0){
    fprintf(stderr,"%s: can't reach radiod's status group %s\n",App_path,Target);
    exit(EX_NOHOST);
  }
  pthread_t t;
  pthread_create(&t,NULL,status_thread,NULL);
  pthread_create(&t,NULL,channel_poll_thread,NULL);

  int const lfd = socket(AF_INET6,SOCK_STREAM,0);
  int const yes = 1,no = 0;
  setsockopt(lfd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes);
  setsockopt(lfd,IPPROTO_IPV6,IPV6_V6ONLY,&no,sizeof no); // IPv4 too
  struct sockaddr_in6 addr = { .sin6_family = AF_INET6, .sin6_port = htons((uint16_t)Port), .sin6_addr = in6addr_any };
  if(lfd < 0 || bind(lfd,(struct sockaddr *)&addr,sizeof addr) != 0 || listen(lfd,16) != 0){
    fprintf(stderr,"%s: can't listen on port %d: %s\n",App_path,Port,strerror(errno));
    exit(EX_OSERR);
  }
  fprintf(stderr,"%s: radiod %s, http://<this host>:%d/\n",App_path,Target,Port);
  while(true){
    int const fd = accept(lfd,NULL,NULL);
    if(fd < 0)
      continue;
    if(Connections >= MAX_CONNECTIONS){
      close(fd); // bounded before any thread is made
      continue;
    }
    Connections++;
    pthread_t h;
    if(pthread_create(&h,NULL,http_thread,(void *)(intptr_t)fd) == 0)
      pthread_detach(h);
    else {
      close(fd);
      Connections--;
    }
  }
}
