// activity: find transmissions anywhere in radiod's front end coverage, on
// frequencies configured or not, and log them.
//
//   activity [-c config ...] [-l log.csv] [-s summary.csv] [-u coverage.csv] [-t dB] [-m s]
//            [-w watch.csv ... -a audit-group [-A max] [-r results.csv [-N n]]] [-v] radiod-status-group
//
// A spectrum channel of its own in radiod covers the whole front end
// (6.25 kHz bins), nominally four times a second, each the average of as
// many FFTs as radiod holds (nominally ~80 ms): sampled, not continuous. A
// transmission of half a second or more (the shortest logged) is sampled
// while radiod answers promptly (a reply can take up to a second to give up
// on); shorter ones may fall between looks. A TDMA burst (DMR, NXDN), on for
// about half a look, reads ~3 dB below its carrier; a weak one may stay
// under the threshold. Each bin has a noise floor of
// its own (falling fast, rising over half an hour: a strong signal's skirt
// isn't activity, and a carrier that never stops is logged once, then taken
// into the floor). Bins 10 dB above it, side by side, make a signal; a
// signal at the same frequency continues a transmission, 2 s without it
// ends it; under half a second is noise.
//
// Each transmission is logged: when, frequency (and the nearest 6.25 kHz
// channel), width, length, how far above the floor, and whether it's on a
// frequency known from the configuration (-c: radiod's config files or
// directory; every frequency in them, trunked systems' channel plans too,
// named by section) or radiod's channels. The summary (rewritten every
// minute, and rebuilt from the log at a restart) gives each channel's
// transmissions, time, first and last heard.
//
// The on-demand audit (-w, -a): frequencies to audit, from CSV lists (-w a
// file, or a folder: every .csv in it) (a
// RadioReference download, the catalog's master.csv: their RX MHz /
// Frequency Output column; a trunked system's site download, each site's
// Frequencies). A transmission heard within 5 kHz of one (the
// nearest) gets a channel in radiod for as long as it's busy: mode
// "mixed" (analog FM with no tone squelch, and every digital mode), sent to
// the audit group, where tone-survey measures its tone or digital IDs.
// The channel is asked for again every 2 s while it's heard (each ask has
// radiod send its status, which tone-survey reads: a digital transmission
// with no voice, encrypted or data, has no audio to send it), and lapses
// 30 s after (radiod's lifetime): no configuration, and only as many
// channels as are busy, at most -A (16) at once. The start of a
// transmission (~half a second, as it's seen and the channel made) is
// missed: CTCSS and DCS go on all through one; digital IDs are repeated.
// With -r (tone-survey's results, read every minute), a frequency with -N
// (10) transmissions judged is done: its channel goes, for those that
// still need one (the busiest would otherwise keep the channels).
//
// Copyright 2026 ka9q-radio contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#define _GNU_SOURCE 1
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sysexits.h>
#include <time.h>
#include <unistd.h>

#include "misc.h"
#include "status.h"
#include "multicast.h"
#include "radio.h"
#include "timing.h"

int Verbose;
char const *App_path;

static char const *Log_path = "activity.csv";
static char const *Summary_path = "activity-summary.csv";
static char const *Coverage_path;    // (default: the log's name with -coverage)
static double Threshold = 10;      // dB above a bin's floor
static double Min_seconds = 0.5;   // shorter: noise
static double const Hang = 2.0;    // s without a signal: its transmission is over
static double const Raster = 6250; // channels, Hz
static double const Rbw = 6250;    // bins, Hz
static double const Poll_s = 0.25;
static double const Rise_tau = 1800; // s: a floor's rise (a carrier that never stops is taken in)
static double const Known_hz = 5000; // a transmission this near a known frequency is on it
static double Shadow_hz = 150e3;      // a signal this near one heard at the same time,
static double Shadow_db = 20;         // and this much weaker, is its skirt or splatter
static double const Shadow_max = 15;  // s a transmission goes on through shadows alone

static struct sockaddr_storage Ctl_dest;
// The on-demand audit
#define MAX_WATCH 8192
static struct watch {
  double freq;
  int64_t asked;        // ns, last asked for its channel (0: none)
  int64_t opened;       // ns, when its turn began
  int64_t refused;      // ns, last heard while every channel was taken
  int64_t rest_until;   // ns, its turn over: others first until then
  bool done;            // tone-survey has enough of it (-r): not audited
} Watch[MAX_WATCH];     // sorted by frequency
static int Nwatch;
static bool Audit;
static char const *Audit_results;      // tone-survey's results (-r): frequencies with enough aren't audited
static int Audit_enough = 10;          // (-N) transmissions judged
static struct sockaddr_storage Audit_dest;
static int Audit_max = 16;
// The audit's channels' SSRCs: from the frequency, 125 Hz steps (to 8 GHz)
#define AUDIT_SSRC_BASE 0x60000000u
#define AUDIT_SSRC_END  (AUDIT_SSRC_BASE + 0x04000000u)
static double const Audit_life = 30;   // s a channel outlives the last time it was asked for
static double const Audit_every = 2;   // s between asks while it's heard (each draws the channel's status:
                                       // a digital transmission with no voice, encrypted or data, sends none else)
#ifndef AUDIT_TURN
#define AUDIT_TURN 120
#define AUDIT_REST 300
#endif
static double const Audit_turn = AUDIT_TURN; // s a frequency keeps its channel while others wait
static double const Audit_rest = AUDIT_REST; // s it then waits before another turn
static double Audit_blocktime = 0.02;        // radiod's block, s (lifetimes count blocks)
static char Iface[1024];
static int Act_ctl_fd = -1, Act_status_fd = -1;
static uint32_t My_ssrc;

#define MAX_BINS 16384
#define MAX_KNOWN 4096

// --- the front end, from status ---
static pthread_mutex_t Lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t Fresh_cond = PTHREAD_COND_INITIALIZER;
static double Fe_center, Fe_samprate, Fe_min_if, Fe_max_if, Fe_blocktime;
static bool Fe_isreal;
// The latest spectrum
static bool Fresh;
static uint32_t Fresh_tag;
static double F_freq,F_rbw;
static int F_n;
static float F_base,F_step;
static uint8_t F_bins[MAX_BINS];

// --- known frequencies: the configuration's, and radiod's channels ---
struct known { double freq; char name[64]; bool from_radiod; int64_t seen; };
static struct known Known[MAX_KNOWN];
static int Nknown;

static void add_known(double f,char const *name,bool from_radiod){
  if(!(f > 0))
    return;
  for(int i = 0; i < Nknown; i++){
    if(fabs(Known[i].freq - f) < 1){
      if(from_radiod)
	Known[i].seen = gps_time_ns();
      return; // (the configuration's name kept)
    }
  }
  if(Nknown >= MAX_KNOWN)
    return;
  Known[Nknown].freq = f;
  strlcpy(Known[Nknown].name,name,sizeof Known[Nknown].name);
  Known[Nknown].from_radiod = from_radiod;
  Known[Nknown].seen = gps_time_ns();
  Nknown++;
}
// The known frequency nearest f, within Known_hz: its name, or NULL
static char const *known_at(double f){
  char const *best = NULL;
  double bd = Known_hz;
  pthread_mutex_lock(&Lock);
  for(int i = 0; i < Nknown; i++){
    double const d = fabs(Known[i].freq - f);
    if(d <= bd){
      bd = d;
      best = Known[i].name;
    }
  }
  pthread_mutex_unlock(&Lock);
  return best;
}

// Every frequency in a radiod config file: freq/freq0-9 lists, and trunked
// systems' channel plans and control channels ("5:451m9375"), named by section
static void read_config_file(char const *path){
  FILE *fp = fopen(path,"r");
  if(fp == NULL){
    fprintf(stderr,"%s: %s\n",path,strerror(errno));
    return;
  }
  char line[4096],section[64] = "";
  int added = 0;
  while(fgets(line,sizeof line,fp) != NULL){
    char *p = line;
    while(isspace((unsigned char)*p))
      p++;
    if(*p == '#' || *p == ';' || *p == '\0')
      continue;
    if(*p == '['){
      char *e = strchr(p,']');
      if(e != NULL){
	*e = '\0';
	strlcpy(section,p + 1,sizeof section);
      }
      continue;
    }
    char *eq = strchr(p,'=');
    if(eq == NULL)
      continue;
    *eq = '\0';
    char key[64];
    strlcpy(key,p,sizeof key);
    for(char *k = key + strlen(key); k > key && isspace((unsigned char)k[-1]); )
      *--k = '\0';
    bool const list = (strncmp(key,"freq",4) == 0 && (key[4] == '\0' || isdigit((unsigned char)key[4])))
      || strcmp(key,"nxdn-channels") == 0 || strcmp(key,"ltr-repeaters") == 0
      || strcmp(key,"p25-control-channels") == 0 || strcmp(key,"control-channels") == 0;
    if(!list)
      continue;
    char *v = eq + 1;
    char *hash = strpbrk(v,"#;");
    if(hash != NULL)
      *hash = '\0';
    for(char *tok = strtok(v," \t\r\n\",'"); tok != NULL; tok = strtok(NULL," \t\r\n\",'")){
      char *colon = strchr(tok,':'); // a channel number first
      double const f = parse_frequency(colon != NULL ? colon + 1 : tok,true);
      if(f > 1e6){
	add_known(f,section,false);
	added++;
      }
    }
  }
  fclose(fp);
  if(Verbose)
    fprintf(stderr,"%s: %d frequencies\n",path,added);
}
static int conf_only(struct dirent const *d){
  size_t const n = strlen(d->d_name);
  return n > 5 && strcmp(d->d_name + n - 5,".conf") == 0;
}
static void read_config(char const *path){
  struct dirent **list = NULL;
  int const n = scandir(path,&list,conf_only,alphasort);
  if(n < 0){
    read_config_file(path); // a file, not a directory
    return;
  }
  for(int i = 0; i < n; i++){
    char full[4096];
    snprintf(full,sizeof full,"%s/%s",path,list[i]->d_name);
    read_config_file(full);
    free(list[i]);
  }
  free(list);
}

// --- status from radiod ---
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
  pthread_setname("act status");
  static uint8_t buffer[PKTSIZE];
  static uint8_t bins[MAX_BINS];
  static struct frontend fe;
  static chan_t chan;
  while(true){
    ssize_t const length = recv(Act_status_fd,buffer,sizeof buffer,0);
    if(length < 2 || (enum pkt_type)buffer[0] != STATUS)
      continue;
    memset(&fe,0,sizeof fe);
    memset(&chan,0,sizeof chan);
    chan.spectrum.base = chan.spectrum.step = NAN;
    decode_radio_status(&fe,&chan,buffer + 1,(int)length - 1);
    uint32_t const ssrc = chan.output.rtp.ssrc;
    pthread_mutex_lock(&Lock);
    if(fe.samprate > 0){
      Fe_center = fe.frequency;
      Fe_samprate = fe.samprate;
      Fe_isreal = fe.isreal;
      if(fe.max_IF > fe.min_IF){
	Fe_min_if = fe.min_IF;
	Fe_max_if = fe.max_IF;
      }
      if(fe.L > 0)
	Fe_blocktime = fe.L / fe.samprate;
    }
    if(ssrc == My_ssrc && ssrc != 0){
      int const n = byte_bins(buffer + 1,(int)length - 1,bins,MAX_BINS);
      if(n > 0 && isfinite(chan.spectrum.base) && isfinite(chan.spectrum.step)){
	Fresh_tag = get_tag(buffer + 1,(int)length - 1);
	F_freq = chan.tune.freq;
	F_rbw = chan.spectrum.rbw;
	F_n = n;
	F_base = chan.spectrum.base;
	F_step = chan.spectrum.step;
	memcpy(F_bins,bins,(size_t)n);
	Fresh = true;
	pthread_cond_broadcast(&Fresh_cond);
      }
    } else if(ssrc != 0 && chan.demod_type != SPECT_DEMOD && chan.demod_type != SPECT2_DEMOD
	      && !(ssrc >= AUDIT_SSRC_BASE && ssrc < AUDIT_SSRC_END) // (an audit's: not configured)
	      && demod_name_from_type(chan.demod_type) != NULL){
      // A channel of radiod's, and a followed call's voice channel
      char name[64];
      snprintf(name,sizeof name,"ssrc %u",ssrc);
      add_known(chan.tune.freq,name,true);
      if(chan.dv.call_freq > 0 && chan.dv.call_freq != chan.tune.freq)
	add_known(chan.dv.call_freq,name,true);
    }
    pthread_mutex_unlock(&Lock);
  }
  return NULL;
}

static void send_command(uint8_t *buffer,uint8_t *bp){
  encode_eol(&bp);
  socklen_t const slen = Ctl_dest.ss_family == AF_INET ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6);
  if(sendto(Act_ctl_fd,buffer,(size_t)(bp - buffer),0,(struct sockaddr *)&Ctl_dest,slen) < 0 && Verbose)
    perror("command send");
}

static void *poll_thread(void *arg){
  (void)arg;
  pthread_setname("act poll");
  while(true){
    // Two polls: the front end alone (SSRC 0), then every channel
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
    sleep(10);
  }
  return NULL;
}

// The wall clock, ns since the Unix epoch (gps_time_ns() isn't: GPS time),
// for every time logged
static int64_t wall_ns(void){
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME,&ts);
  return (int64_t)ts.tv_sec * BILLION + ts.tv_nsec;
}

// --- the on-demand audit ---
// One CSV line's fields (quoted fields, doubled quotes): how many
static int csv_fields(char *line,char **field,int max){
  int n = 0;
  char *p = line;
  while(n < max){
    char *out = p;
    field[n++] = p;
    if(*p == '"'){
      char *in = p + 1;
      while(*in != '\0'){
	if(*in == '"' && in[1] == '"'){
	  *out++ = '"';
	  in += 2;
	} else if(*in == '"'){
	  in++;
	  break;
	} else
	  *out++ = *in++;
      }
      while(*in != '\0' && *in != ',')
	in++;
      bool const more = *in == ',';
      *out = '\0';
      if(!more)
	break;
      p = in + 1;
    } else {
      char *comma = strchr(p,',');
      if(comma == NULL){
	p[strcspn(p,"\r\n")] = '\0';
	break;
      }
      *comma = '\0';
      p = comma + 1;
    }
  }
  return n;
}
static int by_watch(void const *a,void const *b){
  double const x = ((struct watch const *)a)->freq,y = ((struct watch const *)b)->freq;
  return x < y ? -1 : x > y;
}
// A list of frequencies to audit: a CSV with an RX MHz, Frequency Output,
// Frequency or RX Frequency column (MHz)
static void read_watch(char const *path){
  FILE *fp = fopen(path,"r");
  if(fp == NULL){
    fprintf(stderr,"%s: %s\n",path,strerror(errno));
    exit(EX_NOINPUT);
  }
  static char line[65536];
  char *field[256];
  int col = -1;
  bool sites = false;
  if(fgets(line,sizeof line,fp) != NULL){
    char *h = line;
    if((unsigned char)h[0] == 0xef && (unsigned char)h[1] == 0xbb && (unsigned char)h[2] == 0xbf)
      h += 3; // (a byte order mark)
    int const n = csv_fields(h,field,256);
    static char const *const names[] = {"RX MHz","Frequency Output","Frequency","RX Frequency","Output Freq","Frequencies"};
    for(int k = 0; k < 6 && col < 0; k++)
      for(int i = 0; i < n; i++)
	if(strcmp(field[i],names[k]) == 0){
	  col = i;
	  sites = k == 5; // a trunked system's site download: its frequencies in this column and those after
	  break;
	}
  }
  if(col < 0){
    fprintf(stderr,"%s: no RX MHz, Frequency Output, Frequency, RX Frequency, Output Freq or Frequencies column\n",path);
    exit(EX_DATAERR);
  }
  int added = 0;
  bool full = false;
  while(!full && fgets(line,sizeof line,fp) != NULL){
    int const n = csv_fields(line,field,256);
    // One frequency a row; a site download's row, each of its site's
    // ("452.0125c": a control channel, the letter ignored)
    for(int c = col; c < (sites ? n : (col < n ? col + 1 : col)); c++){
      char *end;
      double const mhz = strtod(field[c],&end);
      if(end == field[c] || !(mhz > 0))
	continue;
      double const f = mhz * 1e6;
      bool dup = false;
      for(int i = 0; i < Nwatch && !dup; i++)
	dup = fabs(Watch[i].freq - f) < 100;
      if(dup)
	continue;
      if(Nwatch >= MAX_WATCH){
	fprintf(stderr,"%s: more than %d frequencies to audit: the rest left out\n",path,MAX_WATCH);
	full = true;
	break;
      }
      Watch[Nwatch++] = (struct watch){ .freq = f };
      added++;
    }
  }
  fclose(fp);
  qsort(Watch,(size_t)Nwatch,sizeof Watch[0],by_watch);
  fprintf(stderr,"%s: %d frequencies to audit\n",path,added);
}
static int csv_only(struct dirent const *d){
  size_t const len = strlen(d->d_name);
  return d->d_name[0] != '.' && len > 4 && strcasecmp(d->d_name + len - 4,".csv") == 0;
}
static void send_command(uint8_t *buffer,uint8_t *bp);
static uint32_t audit_ssrc(double f){
  return AUDIT_SSRC_BASE + (uint32_t)lrint(f / 125);
}
// A channel the audit holds, closed at once (a lifetime of one block)
static void audit_close(struct watch *w){
  uint8_t buffer[64],*bp = buffer;
  *bp++ = CMD;
  encode_int(&bp,OUTPUT_SSRC,audit_ssrc(w->freq));
  encode_int(&bp,COMMAND_TAG,(uint32_t)random());
  encode_int(&bp,LIFETIME,1);
  send_command(buffer,bp);
  w->asked = 0;
}
// Channels the audit holds now
static int audit_busy(int64_t now){
  int busy = 0;
  for(int k = 0; k < Nwatch; k++)
    if(Watch[k].asked != 0 && (double)(now - Watch[k].asked) / BILLION < Audit_life)
      busy++;
  return busy;
}
// Frequencies heard without a channel lately, for want of one (within secs)
static int audit_refused(int64_t now,double secs){
  int n = 0;
  for(int k = 0; k < Nwatch; k++)
    if(Watch[k].refused != 0 && !Watch[k].done && (double)(now - Watch[k].refused) / BILLION < secs && now >= Watch[k].rest_until)
      n++;
  return n;
}
static bool audit_waiting(int64_t now){
  return audit_refused(now,10) > 0;
}
// A transmission at f, heard now: the nearest frequency to audit within
// Known_hz gets its channel (asked for again every Audit_every s)
static void audit(double f,int64_t now){
  if(!Audit || Nwatch == 0)
    return;
  int lo = 0,hi = Nwatch;
  while(lo < hi){
    int const mid = (lo + hi) / 2;
    if(Watch[mid].freq < f)
      lo = mid + 1;
    else
      hi = mid;
  }
  struct watch *w = NULL;
  for(int k = lo - 1; k <= lo; k++)
    if(k >= 0 && k < Nwatch && fabs(Watch[k].freq - f) <= Known_hz && (w == NULL || fabs(Watch[k].freq - f) < fabs(w->freq - f)))
      w = &Watch[k];
  if(w == NULL)
    return;
  if(w->done)
    return; // (its channel closed as it was found done)
  if(now < w->rest_until)
    return; // its turn was had: others first
  bool const live = w->asked != 0 && (double)(now - w->asked) / BILLION < Audit_life;
  if(live && (double)(now - w->opened) / BILLION >= Audit_turn){
    if(audit_waiting(now)){
      // Its turn is over, and others are waiting: its channel goes now, and it rests
      audit_close(w);
      w->rest_until = now + (int64_t)(Audit_rest * BILLION);
      if(Verbose)
	fprintf(stderr,"audit: %.5f MHz: its turn over, its channel given to others\n",w->freq * 1e-6);
      return;
    }
    w->opened = now; // nobody waiting: another turn
  }
  if(live && (double)(now - w->asked) / BILLION < Audit_every)
    return;
  if(!live){
    if(audit_busy(now) >= Audit_max){
      w->refused = now; // (counted in the minute's line)
      return;
    }
    w->opened = now;
    if(Verbose)
      fprintf(stderr,"audit: %.5f MHz heard: a channel\n",w->freq * 1e-6);
  }
  w->asked = now;
  uint8_t buffer[256],*bp = buffer;
  *bp++ = CMD;
  // Its own SSRC, from the frequency (125 Hz steps), clear of radiod's
  // (kHz) and the spectrum channels' (the top bit)
  encode_int(&bp,OUTPUT_SSRC,audit_ssrc(w->freq));
  encode_int(&bp,COMMAND_TAG,(uint32_t)random());
  encode_string(&bp,PRESET,"mixed",5);
  encode_double(&bp,RADIO_FREQUENCY,w->freq);
  encode_socket(&bp,OUTPUT_DATA_DEST_SOCKET,&Audit_dest);
  encode_int(&bp,LIFETIME,(int)lrint(Audit_life / Audit_blocktime));
  send_command(buffer,bp);
}

// tone-survey's results (frequency_mhz, transmissions, ...): a frequency
// to audit with Audit_enough or more judged is done. Read every minute
static void audit_read_results(void){
  if(Audit_results == NULL)
    return;
  FILE *fp = fopen(Audit_results,"r");
  if(fp == NULL)
    return; // (not written yet)
  char line[4096];
  int done = 0;
  int64_t const now = wall_ns();
  while(fgets(line,sizeof line,fp) != NULL){
    char *end;
    double const mhz = strtod(line,&end);
    if(end == line || *end != ',' || !(mhz > 0))
      continue; // (the header)
    long const n = strtol(end + 1,NULL,10);
    double const f = mhz * 1e6;
    int lo = 0,hi = Nwatch;
    while(lo < hi){
      int const mid = (lo + hi) / 2;
      if(Watch[mid].freq < f - 100)
	lo = mid + 1;
      else
	hi = mid;
    }
    for(int k = lo; k < Nwatch && Watch[k].freq <= f + 100; k++)
      if(n >= Audit_enough && !Watch[k].done){
	Watch[k].done = true;
	done++;
	if(Watch[k].asked != 0 && (double)(now - Watch[k].asked) / BILLION < Audit_life){
	  // (a lease lapsed already: no command, which would make the channel anew)
	  audit_close(&Watch[k]); // its channel goes now, for others
	  if(Verbose)
	    fprintf(stderr,"audit: %.5f MHz: enough results, its channel given to others\n",Watch[k].freq * 1e-6);
	}
      }
  }
  fclose(fp);
  if(Verbose && done > 0){
    int all = 0;
    for(int k = 0; k < Nwatch; k++)
      all += Watch[k].done;
    fprintf(stderr,"audit: %d more frequencies have %d results; %d of %d done\n",done,Audit_enough,all,Nwatch);
  }
}

// --- transmissions ---
struct tx {
  bool open;
  double freq;          // power-weighted centre, Hz
  double wsum;          // (its weight)
  double width;         // the widest seen, Hz
  int64_t start,last;   // ns
  float peak_over;      // dB above the floor
  float peak_dbfs;
  int64_t matched;      // the frame a clean detection last continued it (now)
  float level;          // its level (dBFS) when last seen clear
  int64_t clean;        // when last seen clear (not only through a shadow)
};
#define MAX_TX 512
static struct tx Txs[MAX_TX];

// The summary: per 6.25 kHz channel
struct chanstat { double chan; int count; double seconds; int64_t first,last; float peak; char known[64]; };
#define MAX_STATS 8192
static struct chanstat Stats[MAX_STATS];
static int Nstats;

static struct chanstat *stat_for(double chan){
  for(int i = 0; i < Nstats; i++)
    if(fabs(Stats[i].chan - chan) < 1)
      return &Stats[i];
  if(Nstats >= MAX_STATS)
    return NULL;
  struct chanstat *s = &Stats[Nstats++];
  memset(s,0,sizeof *s);
  s->chan = chan;
  s->peak = -INFINITY;
  return s;
}
static void tally(double chan,double seconds,int64_t start,int64_t end,float peak,char const *known){
  struct chanstat *s = stat_for(chan);
  if(s == NULL)
    return;
  s->count++;
  s->seconds += seconds;
  if(s->first == 0 || start < s->first)
    s->first = start;
  if(end > s->last)
    s->last = end;
  if(peak > s->peak)
    s->peak = peak;
  if(known != NULL && known[0] != '\0')
    strlcpy(s->known,known,sizeof s->known);
}

// A time as written: UTC, marked Z ("2026-11-24 13:29:20Z"), so a span
// across a daylight saving change is still its length
static char const *when(int64_t ns,char *out,size_t size){
  time_t const t = (time_t)(ns / BILLION); // (wall_ns())
  struct tm tm;
  gmtime_r(&t,&tm);
  strftime(out,size,"%Y-%m-%d %H:%M:%SZ",&tm);
  return out;
}
// A time as read: UTC if marked Z, else local (rows written before),
// ns; the text after it in *rest. false if it isn't one
static bool parse_when(char const *text,int64_t *ns,char const **rest){
  struct tm tm = {0};
  char const *e = strptime(text,"%Y-%m-%d %H:%M:%S",&tm);
  if(e == NULL)
    return false;
  time_t t;
  if(*e == 'Z'){
    t = timegm(&tm);
    e++;
  } else {
    tm.tm_isdst = -1;
    t = mktime(&tm);
  }
  *ns = (int64_t)t * BILLION;
  if(rest != NULL)
    *rest = e;
  return true;
}
// A CSV field's text, quoted, quotes in it doubled
static void csv_quote(char *out,size_t size,char const *in){
  size_t k = 0;
  if(size < 3){
    if(size > 0)
      out[0] = '\0';
    return;
  }
  out[k++] = '"';
  for(; *in != '\0' && k + 3 < size; in++){
    if(*in == '"')
      out[k++] = '"';
    out[k++] = *in;
  }
  out[k++] = '"';
  out[k] = '\0';
}

static char const *mode_guess(double width){
  // From the width alone, in 6.25 kHz bins: a guess
  if(width <= 6250)
    return "narrow (NXDN48, dPMR, 6.25 kHz)";
  if(width <= 12500)
    return "12.5 kHz (NFM, DMR, P25, NXDN96)";
  if(width <= 25000)
    return "wide (25 kHz FM)";
  return "very wide";
}

static void close_tx(struct tx *t){
  t->open = false;
  double const seconds = (double)(t->last - t->start) / BILLION + Poll_s;
  if(seconds < Min_seconds)
    return;
  double const chan = Raster * round(t->freq / Raster);
  char const *known = known_at(t->freq);
  char a[32],b[32],kq[160];
  csv_quote(kq,sizeof kq,known != NULL ? known : "");
  // Logged, or not counted at all: the summary is rebuilt from the log at a
  // restart, so it mustn't hold what the log doesn't
  FILE *fp = fopen(Log_path,"a+");
  bool ok = fp != NULL;
  if(ok){
    // A row left incomplete by a failed write: ended here, so this one
    // starts a line of its own (the rebuild skips the incomplete one)
    // (a seek between that read and any write, as C requires)
    bool const unended = fseek(fp,-1,SEEK_END) == 0 && fgetc(fp) != '\n';
    if(fseek(fp,0,SEEK_END) != 0){
      fclose(fp);
      ok = false;
    } else if(unended)
      fputc('\n',fp);
  }
  if(ok){
    if(ftell(fp) == 0)
      fprintf(fp,"start,end,frequency_mhz,channel_mhz,width_khz,seconds,db_over_floor,dbfs,known,mode_guess\n");
    fprintf(fp,"%s,%s,%.4f,%.5f,%.2f,%.1f,%.1f,%.1f,%s,\"%s\"\n",when(t->start,a,sizeof a),when(t->last,b,sizeof b),
	    t->freq * 1e-6,chan * 1e-6,t->width * 1e-3,seconds,t->peak_over,t->peak_dbfs,kq,mode_guess(t->width));
    ok = !ferror(fp);
    ok = fclose(fp) == 0 && ok;
  }
  if(!ok){
    fprintf(stderr,"%s: can't write the log: %s (a %.4f MHz transmission not recorded)\n",Log_path,strerror(errno),t->freq * 1e-6);
    return;
  }
  if(Verbose)
    printf("%s  %.4f MHz  %4.1f s  +%.0f dB  %s\n",when(t->start,a,sizeof a),t->freq * 1e-6,seconds,t->peak_over,
	   known != NULL ? known : "UNKNOWN");
  tally(chan,seconds,t->start,t->last,t->peak_over,known);
}

static int by_chan(void const *a,void const *b){
  double const x = ((struct chanstat const *)a)->chan, y = ((struct chanstat const *)b)->chan;
  return x < y ? -1 : x > y;
}
static void write_summary(void){
  // A temporary file of a name no other has (not "summary.tmp", which the
  // log could be), renamed over the summary once complete
  char tmp[4096];
  snprintf(tmp,sizeof tmp,"%s.XXXXXX",Summary_path);
  int const fd = mkstemp(tmp);
  FILE *fp = fd >= 0 ? fdopen(fd,"w") : NULL;
  if(fp == NULL){
    if(fd >= 0){
      close(fd);
      unlink(tmp);
    }
    fprintf(stderr,"%s: can't write the summary: %s\n",Summary_path,strerror(errno));
    return;
  }
  qsort(Stats,(size_t)Nstats,sizeof Stats[0],by_chan);
  fprintf(fp,"channel_mhz,transmissions,seconds,first,last,peak_db_over_floor,known\n");
  for(int i = 0; i < Nstats; i++){
    struct chanstat const *s = &Stats[i];
    char a[32],b[32],kq[160];
    csv_quote(kq,sizeof kq,s->known);
    fprintf(fp,"%.5f,%d,%.0f,%s,%s,%.0f,%s\n",s->chan * 1e-6,s->count,s->seconds,when(s->first,a,sizeof a),
	    when(s->last,b,sizeof b),s->peak,kq);
  }
  // The old summary is replaced only by a complete new one
  bool ok = !ferror(fp);
  ok = fclose(fp) == 0 && ok;
  if(!ok || rename(tmp,Summary_path) != 0){
    fprintf(stderr,"%s: can't write the summary: %s\n",Summary_path,strerror(errno));
    unlink(tmp);
  }
}
// The summary rebuilt from the log, at a restart
static void reload_log(void){
  FILE *fp = fopen(Log_path,"r");
  if(fp == NULL)
    return;
  char line[1024];
  int n = 0,lines = 0;
  while(fgets(line,sizeof line,fp) != NULL){
    lines++;
    // Only a complete row: its newline, and its last field (the mode
    // guess, quoted) closed. A write that failed partway left less
    size_t const len = strlen(line);
    if(len < 3 || line[len - 1] != '\n' || line[len - 2] != '"')
      continue;
    int64_t a,b;
    double f,chan,width,seconds,over,dbfs;
    char known[128] = "";
    int consumed = 0;
    if(!parse_when(line,&a,NULL))
      continue; // the header
    char const *p = strchr(line,',');
    if(p == NULL || !parse_when(p + 1,&b,NULL))
      continue;
    p = strchr(p + 1,',');
    if(p == NULL || sscanf(p + 1,"%lf,%lf,%lf,%lf,%lf,%lf,%n",&f,&chan,&width,&seconds,&over,&dbfs,&consumed) < 6)
      continue;
    char const *k = p + 1 + consumed;
    if(*k != '"')
      continue;
    {
      // a quoted field, quotes in it doubled; then the mode guess's, to the end
      size_t j = 0;
      bool closed = false;
      for(k++; *k != '\0'; k++){
	if(*k == '"'){
	  if(k[1] == '"')
	    k++;
	  else {
	    closed = true;
	    break;
	  }
	}
	if(j + 1 < sizeof known)
	  known[j++] = *k;
      }
      known[j] = '\0';
      if(!closed || strncmp(k,"\",\"",3) != 0)
	continue; // not a complete row
      // the mode guess: its whole field, closed, then the line's end
      char const *q = strchr(k + 3,'"');
      if(q == NULL || q[1] != '\n')
	continue;
    }
    tally(chan * 1e6,seconds,a,b,(float)over,known);
    n++;
  }
  fclose(fp);
  int const skipped = lines - n - 1; // (less the header)
  if(Verbose || skipped > 0)
    fprintf(stderr,"%s: %d transmissions already logged%s\n",Log_path,n,skipped > 0 ? ", incomplete rows skipped" : "");
}

// --- coverage: when activity was watching, and what ---
// A row per stretch of continuous monitoring: start, end, and the span
// watched. A stretch ends with a gap in spectra (Cover_gap), a change of
// span (the front end retuned), or Cover_chunk (rows are written as they
// close: a crash loses at most that), and when activity stops. What the log
// can't say (how long a quiet frequency was watched) this does
static double const Cover_gap = 5;     // s without a spectrum: an outage
static double const Cover_chunk = 600; // s, the longest row
static struct { bool open; int64_t start,last; double lo,hi; } Cover;

static void cover_close(void){
  if(!Cover.open)
    return;
  Cover.open = false;
  char a[32],b[32];
  FILE *fp = fopen(Coverage_path,"a");
  bool ok = fp != NULL;
  if(ok){
    if(ftell(fp) == 0)
      fprintf(fp,"start,end,lo_mhz,hi_mhz\n");
    fprintf(fp,"%s,%s,%.5f,%.5f\n",when(Cover.start,a,sizeof a),when(Cover.last,b,sizeof b),Cover.lo * 1e-6,Cover.hi * 1e-6);
    ok = !ferror(fp);
    ok = fclose(fp) == 0 && ok;
  }
  if(!ok)
    fprintf(stderr,"%s: can't write the coverage: %s\n",Coverage_path,strerror(errno));
}
// A spectrum came, covering lo-hi, at now (none: cover_close() after a gap)
static void cover_frame(double lo,double hi,int64_t now){
  if(Cover.open && (fabs(lo - Cover.lo) > Rbw || fabs(hi - Cover.hi) > Rbw
		    || (double)(now - Cover.last) / BILLION > Cover_gap
		    || (double)(now - Cover.start) / BILLION >= Cover_chunk))
    cover_close();
  if(!Cover.open){
    Cover.open = true;
    Cover.start = now;
    Cover.lo = lo;
    Cover.hi = hi;
  }
  Cover.last = now;
}
static void cover_idle(int64_t now){
  if(Cover.open && (double)(now - Cover.last) / BILLION > Cover_gap)
    cover_close(); // (its end: the last spectrum, not now)
}

// --- one spectrum ---
static float Floor[MAX_BINS];
static bool Floor_ready;
static double Floor_start;
static int Floor_n;

static int cmp_float(void const *a,void const *b){
  float const x = *(float const *)a, y = *(float const *)b;
  return x < y ? -1 : x > y;
}

// Transmissions not heard for Hang are over: before a frame's signals are
// matched (one coming back later is another transmission), and with no
// usable frame (radiod not answering: an outage ends them too)
static void expire(int64_t now){
  for(int k = 0; k < MAX_TX; k++)
    if(Txs[k].open && (double)(now - Txs[k].last) / BILLION > Hang)
      close_tx(&Txs[k]);
}

struct sig { double f,w,width; float over,top; bool shadow; };
static struct sig const *Sort_sig;
static int by_strength(void const *a,void const *b){
  float const x = Sort_sig[*(int const *)a].top, y = Sort_sig[*(int const *)b].top;
  return x > y ? -1 : x < y ? +1 : *(int const *)a - *(int const *)b;
}

static void process(double start,double rbw,int n,float base,float step,uint8_t const *bins,int64_t now){
  static float db[MAX_BINS],sorted[MAX_BINS];
  expire(now);
  // Byte 0 is the bottom of the scale: a quiet bin (or one with no power at
  // all, outside the front end's passband; this asks only within it, so
  // those are at the very edge at most): a valid, quiet reading either way
  for(int i = 0; i < n; i++){
    db[i] = base + bins[i] * step;
    sorted[i] = db[i];
  }
  int const valid = n;
  if(!Floor_ready || n != Floor_n || fabs(start - Floor_start) > rbw / 2){
    // A new grid (the front end retuned): transmissions going on are logged
    // as heard so far, not lost
    for(int i = 0; i < MAX_TX; i++)
      if(Txs[i].open)
	close_tx(&Txs[i]);
    // A floor to start from: the median, everywhere (signals then stand out from it)
    qsort(sorted,(size_t)valid,sizeof sorted[0],cmp_float);
    float const median = sorted[valid / 2];
    for(int i = 0; i < n; i++)
      Floor[i] = median;
    Floor_ready = true;
    Floor_n = n;
    Floor_start = start;
  }
  float const rise = (float)(Poll_s / Rise_tau);
  if(Verbose > 1){
    float most = -INFINITY;
    int at = 0;
    for(int k = 0; k < n; k++)
      if(isfinite(db[k]) && db[k] - Floor[k] > most){
	most = db[k] - Floor[k];
	at = k;
      }
    fprintf(stderr,"spectrum: %d bins of %.0f Hz from %.4f MHz, %d with power; most above its floor %.1f dB at %.4f MHz\n",
	    n,rbw,start * 1e-6,valid,most,(start + at * rbw) * 1e-6);
  }
  // Signals: runs of bins above their floors (a one-bin gap bridged)
  static struct sig sig[MAX_BINS / 2];
  int nsig = 0;
  int i = 0;
  while(i < n && nsig < MAX_BINS / 2){
    if(!(isfinite(db[i]) && db[i] - Floor[i] > Threshold)){
      i++;
      continue;
    }
    int lo = i,hi = i;
    while(hi + 1 < n && ((isfinite(db[hi + 1]) && db[hi + 1] - Floor[hi + 1] > Threshold)
			 || (hi + 2 < n && isfinite(db[hi + 2]) && db[hi + 2] - Floor[hi + 2] > Threshold)))
      hi++;
    double w = 0,wf = 0;
    float over = 0,top = -INFINITY;
    for(int k = lo; k <= hi; k++){
      if(!isfinite(db[k]))
	continue;
      double const p = pow(10,(db[k] - Floor[k]) / 10) - 1; // power above the floor
      if(p > 0){
	w += p;
	wf += p * (start + k * rbw);
      }
      if(db[k] - Floor[k] > over)
	over = db[k] - Floor[k];
      if(db[k] > top)
	top = db[k];
    }
    sig[nsig++] = (struct sig){ .f = w > 0 ? wf / w : start + (lo + hi) / 2.0 * rbw, .w = w,
				.width = (hi - lo + 1) * rbw, .over = over, .top = top };
    i = hi + 1;
  }
  // Shadows: a signal heard with one Shadow_db stronger (in dBFS, not over
  // its floor: the strong one's skirt raises the floor near it) within
  // Shadow_hz is its splatter or skirt (a transmitter in the house, a
  // repeater down the road), not a transmission of its own
  // Strongest first, and only signals kept can shadow others: the radius
  // runs from a real signal, not on from one shadow to the next
  static int order[MAX_BINS / 2];
  for(int a = 0; a < nsig; a++)
    order[a] = a;
  Sort_sig = sig;
  qsort(order,(size_t)nsig,sizeof order[0],by_strength);
  int shadowed = 0;
  for(int x = 0; x < nsig; x++){
    struct sig *s1 = &sig[order[x]];
    for(int y = 0; y < x && !s1->shadow; y++){
      struct sig const *s2 = &sig[order[y]];
      if(!s2->shadow && s2->top - s1->top >= Shadow_db && fabs(s2->f - s1->f) <= Shadow_hz)
	s1->shadow = true;
    }
    shadowed += s1->shadow;
  }
  if(Verbose > 1 && shadowed > 0)
    fprintf(stderr,"%d of %d signals taken for a stronger one's skirt\n",shadowed,nsig);
  // Clean detections first: each continues or begins its transmission
  for(int a = 0; a < nsig; a++){
    if(sig[a].shadow)
      continue;
    double const f = sig[a].f,w = sig[a].w,width = sig[a].width;
    float const over = sig[a].over,top = sig[a].top;
    // Continue the nearest transmission at that frequency not already
    // continued this frame, or begin one
    struct tx *t = NULL,*spare = NULL;
    double nearest = fmax(1.5 * rbw,Known_hz);
    for(int k = 0; k < MAX_TX; k++){
      if(Txs[k].open && Txs[k].matched != now && fabs(Txs[k].freq - f) <= nearest){
	nearest = fabs(Txs[k].freq - f);
	t = &Txs[k];
      } else if(!Txs[k].open && spare == NULL)
	spare = &Txs[k];
    }
    if(t == NULL && spare != NULL){
      t = spare;
      memset(t,0,sizeof *t);
      t->open = true;
      t->start = now;
      t->peak_over = -INFINITY;
      t->peak_dbfs = -INFINITY;
    }
    if(t != NULL){
      t->matched = now;
      t->clean = now;
      t->level = top;
      t->freq = (t->freq * t->wsum + f * w) / (t->wsum + w > 0 ? t->wsum + w : 1);
      if(t->wsum + w <= 0)
	t->freq = f;
      t->wsum += w;
      if(width > t->width)
	t->width = width;
      t->last = now;
      if(over > t->peak_over)
	t->peak_over = over;
      if(top > t->peak_dbfs)
	t->peak_dbfs = top;
      audit(t->freq,now);
    }
  }
  // Then shadows: one begins nothing, but keeps a transmission already
  // tracked (from frames where it stood clear) going through it, not split,
  // if it can only be that one: a single open transmission that no clean
  // detection continued this frame, within a bin of its centre and 6 dB of
  // its level when last clear. None, or more than one: nothing. (A skirt
  // where a transmission was, as it ended, can't be told from it for
  // certain; matching both its place and its level makes that unlikely)
  for(int a = 0; a < nsig; a++){
    if(!sig[a].shadow)
      continue;
    struct tx *only = NULL;
    int candidates = 0;
    for(int k = 0; k < MAX_TX; k++)
      if(Txs[k].open && Txs[k].matched != now && fabs(Txs[k].freq - sig[a].f) <= rbw
	 && fabsf(Txs[k].level - sig[a].top) <= 6
	 && (double)(now - Txs[k].clean) / BILLION <= Shadow_max){ // (and not for long: a steady skirt mustn't keep it open)
	only = &Txs[k];
	candidates++;
      }
    if(candidates == 1){
      only->last = now;
      only->matched = now; // (one shadow each)
    }
  }
  // The floors: down at once to a quieter reading, up slowly
  for(int k = 0; k < n; k++){
    if(!isfinite(db[k]))
      continue;
    if(db[k] < Floor[k])
      Floor[k] += 0.3f * (db[k] - Floor[k]);
    else
      Floor[k] += rise * (db[k] - Floor[k]);
  }
}

static volatile sig_atomic_t Stop;
static void on_stop(int sig){
  (void)sig;
  Stop = 1;
}

// Two existing files that are one (hard links)
static bool same_inode(char const *a,char const *b){
  struct stat x,y;
  return stat(a,&x) == 0 && stat(b,&y) == 0 && x.st_dev == y.st_dev && x.st_ino == y.st_ino;
}

// A path as its real path if it exists (a symlink followed to its target),
// else as its directory's real path and its name
static void canonical(char const *path,char *out,size_t size){
  char dir[4096],real[4096];
  if(realpath(path,real) != NULL){
    strlcpy(out,real,size);
    return;
  }
  struct stat st;
  if(lstat(path,&st) == 0 && S_ISLNK(st.st_mode)){
    // A symlink to nothing yet: where it would lead can't be compared
    fprintf(stderr,"%s: %s is a symlink to a file that doesn't exist: give the file itself\n",App_path,path);
    exit(EX_USAGE);
  }
  char const *slash = strrchr(path,'/');
  char const *name = slash != NULL ? slash + 1 : path;
  if(slash == NULL)
    strlcpy(dir,".",sizeof dir);
  else if(slash == path)
    strlcpy(dir,"/",sizeof dir);
  else
    snprintf(dir,sizeof dir,"%.*s",(int)(slash - path),path);
  if(realpath(dir,real) == NULL)
    strlcpy(real,dir,sizeof real);
  snprintf(out,size,"%s/%s",real,name);
}

// An option's number, wholly, finite and in range, or the program stops
static double number(char const *text,char const *opt,double lo,double hi){
  char *end = NULL;
  double const x = strtod(text,&end);
  if(end == text || *end != '\0' || !isfinite(x) || x < lo || x > hi){
    fprintf(stderr,"%s %s: a number from %g to %g wanted\n",opt,text,lo,hi);
    exit(EX_USAGE);
  }
  return x;
}

static void usage(void){
  fprintf(stderr,"Usage: %s [-c config ...] [-l log.csv] [-s summary.csv] [-u coverage.csv] [-t dB] [-m seconds] [-S kHz] [-D dB]"
	  " [-w watch.csv ... -a audit-group [-A max] [-r results.csv [-N enough]]] [-v] radiod-status-group\n",App_path);
}

int main(int argc,char *argv[]){
  App_path = argv[0];
  int ch;
  char const *configs[32];
  int nconfigs = 0;
  char const *watches[32];
  int nwatches = 0;
  char const *audit_group = NULL;
  while((ch = getopt(argc,argv,"c:l:s:u:t:m:S:D:w:a:A:r:N:v")) != -1){
    switch(ch){
    case 'c': if(nconfigs < 32) configs[nconfigs++] = optarg; break;
    case 'l': Log_path = optarg; break;
    case 's': Summary_path = optarg; break;
    case 'u': Coverage_path = optarg; break;
    case 't': Threshold = number(optarg,"-t",0.1,100); break;
    case 'm': Min_seconds = number(optarg,"-m",0,3600); break;
    case 'S': Shadow_hz = number(optarg,"-S",0,10000) * 1e3; break; // 0: the rule off
    case 'D': Shadow_db = number(optarg,"-D",1,100); break;
    case 'w': if(nwatches < 32) watches[nwatches++] = optarg; break;
    case 'a': audit_group = optarg; break;
    case 'A': Audit_max = (int)number(optarg,"-A",1,256); break;
    case 'r': Audit_results = optarg; break;
    case 'N': Audit_enough = (int)number(optarg,"-N",1,1000000); break;
    case 'v': Verbose++; break;
    default: usage(); exit(EX_USAGE);
    }
  }
  if(optind >= argc || !(Threshold > 0)){
    usage();
    exit(EX_USAGE);
  }
  if(Coverage_path == NULL){
    // The log's name with -coverage (activity.csv: activity-coverage.csv),
    // as frequency-survey.py looks for it
    static char path[4096];
    size_t len = strlen(Log_path);
    if(len >= 4 && strcmp(Log_path + len - 4,".csv") == 0)
      len -= 4;
    snprintf(path,sizeof path,"%.*s-coverage.csv",(int)len,Log_path);
    Coverage_path = path;
  }
  {
    // The log, the summary and the coverage must be different files (the
    // summary's rewrite would replace the others; appends would mix).
    // Compared by real path (or directory and name, not made yet)
    char a[8192],b[8192],c[8192];
    canonical(Log_path,a,sizeof a);
    canonical(Summary_path,b,sizeof b);
    canonical(Coverage_path,c,sizeof c);
    if(strcmp(a,b) == 0 || strcmp(a,c) == 0 || strcmp(b,c) == 0
       || same_inode(Log_path,Summary_path) || same_inode(Log_path,Coverage_path) || same_inode(Summary_path,Coverage_path)){
      fprintf(stderr,"%s: the log, the summary and the coverage must be different files\n",App_path);
      exit(EX_USAGE);
    }
  }
  for(int i = 0; i < nconfigs; i++)
    read_config(configs[i]);
  if((nwatches > 0) != (audit_group != NULL)){
    fprintf(stderr,"%s: -w (what to audit) and -a (the audit's group) go together\n",App_path);
    exit(EX_USAGE);
  }
  for(int i = 0; i < nwatches; i++){
    // A folder: every .csv in it
    struct stat st;
    if(stat(watches[i],&st) == 0 && S_ISDIR(st.st_mode)){
      struct dirent **list;
      int const n = scandir(watches[i],&list,csv_only,alphasort);
      if(n < 0){
	fprintf(stderr,"%s: %s\n",watches[i],strerror(errno));
	exit(EX_NOINPUT);
      }
      for(int k = 0; k < n; k++){
	char path[4096];
	snprintf(path,sizeof path,"%s/%s",watches[i],list[k]->d_name);
	read_watch(path);
	free(list[k]);
      }
      free(list);
    } else
      read_watch(watches[i]);
  }
  if(audit_group != NULL){
    char iface[1024] = {0};
    if(name_mcast(audit_group,&Audit_dest,DEFAULT_RTP_PORT,iface,sizeof iface) != 0){
      fprintf(stderr,"%s: can't resolve %s\n",App_path,audit_group);
      exit(EX_NOHOST);
    }
    Audit = true;
    fprintf(stderr,"%s: auditing %d frequencies: channels to %s, at most %d at once\n",App_path,Nwatch,audit_group,Audit_max);
  }
  reload_log();
  {
    // The log made now, with its header, so a run that hears nothing still
    // leaves one (frequency-survey.py reads it with the coverage)
    FILE *fp = fopen(Log_path,"a");
    if(fp == NULL){
      fprintf(stderr,"%s: %s\n",Log_path,strerror(errno));
      exit(EX_CANTCREAT);
    }
    if(ftell(fp) == 0)
      fprintf(fp,"start,end,frequency_mhz,channel_mhz,width_khz,seconds,db_over_floor,dbfs,known,mode_guess\n");
    fclose(fp);
  }
  srandom((unsigned)gps_time_ns());
  My_ssrc = 0x80000000u | (uint32_t)random() << 1 | (uint32_t)(random() & 1); // clear of radiod's own (see waterfall.c)

  resolve_mcast(argv[optind],&Ctl_dest,DEFAULT_STAT_PORT,Iface,sizeof Iface,0);
  Act_status_fd = listen_mcast(NULL,&Ctl_dest,Iface[0] ? Iface : NULL);
  Act_ctl_fd = output_mcast(&Ctl_dest,Iface[0] ? Iface : NULL,1,0);
  if(Act_status_fd < 0 || Act_ctl_fd < 0){
    fprintf(stderr,"%s: can't reach radiod's status group %s\n",App_path,argv[optind]);
    exit(EX_NOHOST);
  }
  pthread_t t;
  pthread_create(&t,NULL,status_thread,NULL);
  pthread_create(&t,NULL,poll_thread,NULL);
  fprintf(stderr,"%s: radiod %s, %d known frequencies; log %s, summary %s, coverage %s\n",App_path,argv[optind],Nknown,
	  Log_path,Summary_path,Coverage_path);

  signal(SIGINT,on_stop);
  signal(SIGTERM,on_stop);
  int64_t last_summary = 0;
  while(!Stop){
    int64_t const started = gps_time_ns();
    // The whole coverage, as tuned now (radiod won't retune for a spectrum channel)
    pthread_mutex_lock(&Lock);
    double min_if = Fe_min_if,max_if = Fe_max_if;
    if(!(max_if > min_if) && Fe_samprate > 0 && !Fe_isreal){
      min_if = -0.47 * Fe_samprate;
      max_if = 0.47 * Fe_samprate;
    }
    double const center = Fe_center + (min_if + max_if) / 2;
    double const blocktime = Fe_blocktime > 0 ? Fe_blocktime : 0.02;
    Audit_blocktime = blocktime;
    bool const known = Fe_samprate > 0 && max_if > min_if;
    pthread_mutex_unlock(&Lock);
    if(!known){
      expire(wall_ns());
      cover_idle(wall_ns());
      sleep(1);
      continue;
    }
    int bins = (int)((max_if - min_if) / Rbw) & ~1;
    if(bins > MAX_BINS)
      bins = MAX_BINS;
    uint8_t buffer[256],*bp = buffer;
    *bp++ = CMD;
    uint32_t const tag = (uint32_t)random();
    encode_int(&bp,OUTPUT_SSRC,My_ssrc);
    encode_int(&bp,COMMAND_TAG,tag);
    encode_int(&bp,DEMOD_TYPE,SPECT2_DEMOD);
    encode_int(&bp,LIFETIME,(int)lrint(5 / blocktime));
    encode_double(&bp,RADIO_FREQUENCY,center);
    encode_int(&bp,BIN_COUNT,bins);
    encode_float(&bp,RESOLUTION_BW,(float)Rbw);
    encode_int(&bp,SPECTRUM_AVG,1000); // as many as radiod holds (~80 ms): a TDMA burst isn't missed
    send_command(buffer,bp);
    struct timespec until;
    clock_gettime(CLOCK_REALTIME,&until);
    until.tv_sec += 1;
    pthread_mutex_lock(&Lock);
    while(!(Fresh && Fresh_tag == tag))
      if(pthread_cond_timedwait(&Fresh_cond,&Lock,&until) != 0)
	break;
    bool const have = Fresh && Fresh_tag == tag;
    static uint8_t bins_copy[MAX_BINS];
    int n = 0;
    double freq = 0,rbw = 0;
    float base = 0,step = 0;
    if(have){
      Fresh = false;
      n = F_n;
      freq = F_freq;
      rbw = F_rbw;
      base = F_base;
      step = F_step;
      memcpy(bins_copy,F_bins,(size_t)n);
    }
    pthread_mutex_unlock(&Lock);
    int64_t const now = wall_ns();
    if(have && rbw > 0){
      double const lo = freq - rbw * (n / 2); // (radiod's one-byte bins run low to high from -n/2)
      process(lo,rbw,n,base,step,bins_copy,now);
      cover_frame(lo,lo + rbw * n,now);
    } else {
      expire(now); // no spectrum: time still passes
      cover_idle(now);
    }
    if(now - last_summary > 60 * (int64_t)BILLION){
      audit_read_results();
      if(Audit && Verbose){
	int const waiting = audit_refused(now,60);
	fprintf(stderr,"audit: %d of %d channels in use%s",audit_busy(now),Audit_max,waiting ? "" : "\n");
	if(waiting)
	  fprintf(stderr,"; %d more frequencies heard in the last minute waited for one\n",waiting);
      }
      write_summary();
      last_summary = now;
    }
    fflush(stdout);
    int64_t const left = (int64_t)(Poll_s * BILLION) - (gps_time_ns() - started);
    if(left > 0)
      usleep((useconds_t)(left / 1000));
  }
  // Stopped: transmissions going on are logged as they are, and the summary written
  for(int k = 0; k < MAX_TX; k++)
    if(Txs[k].open)
      close_tx(&Txs[k]);
  write_summary();
  cover_close();
  return 0;
}
