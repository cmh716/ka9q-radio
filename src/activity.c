// activity: find transmissions anywhere in radiod's front end coverage, on
// frequencies configured or not, and log them.
//
//   activity [-c config ...] [-l log.csv] [-s summary.csv] [-t dB] [-m s] [-v] radiod-status-group
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

static char const *when(int64_t ns,char *out,size_t size){
  time_t const t = (time_t)(ns / BILLION); // (wall_ns())
  struct tm tm;
  localtime_r(&t,&tm);
  strftime(out,size,"%Y-%m-%d %H:%M:%S",&tm);
  return out;
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
    struct tm a = {0},b = {0};
    double f,chan,width,seconds,over,dbfs;
    char known[128] = "";
    int consumed = 0;
    if(strptime(line,"%Y-%m-%d %H:%M:%S",&a) == NULL)
      continue; // the header
    char const *p = strchr(line,',');
    if(p == NULL || strptime(p + 1,"%Y-%m-%d %H:%M:%S",&b) == NULL)
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
    a.tm_isdst = b.tm_isdst = -1;
    tally(chan * 1e6,seconds,(int64_t)mktime(&a) * BILLION,(int64_t)mktime(&b) * BILLION,(float)over,known);
    n++;
  }
  fclose(fp);
  int const skipped = lines - n - 1; // (less the header)
  if(Verbose || skipped > 0)
    fprintf(stderr,"%s: %d transmissions already logged%s\n",Log_path,n,skipped > 0 ? ", incomplete rows skipped" : "");
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
  fprintf(stderr,"Usage: %s [-c config ...] [-l log.csv] [-s summary.csv] [-t dB] [-m seconds] [-S kHz] [-D dB] [-v] radiod-status-group\n",App_path);
}

int main(int argc,char *argv[]){
  App_path = argv[0];
  int ch;
  char const *configs[32];
  int nconfigs = 0;
  while((ch = getopt(argc,argv,"c:l:s:t:m:S:D:v")) != -1){
    switch(ch){
    case 'c': if(nconfigs < 32) configs[nconfigs++] = optarg; break;
    case 'l': Log_path = optarg; break;
    case 's': Summary_path = optarg; break;
    case 't': Threshold = number(optarg,"-t",0.1,100); break;
    case 'm': Min_seconds = number(optarg,"-m",0,3600); break;
    case 'S': Shadow_hz = number(optarg,"-S",0,10000) * 1e3; break; // 0: the rule off
    case 'D': Shadow_db = number(optarg,"-D",1,100); break;
    case 'v': Verbose++; break;
    default: usage(); exit(EX_USAGE);
    }
  }
  if(optind >= argc || !(Threshold > 0)){
    usage();
    exit(EX_USAGE);
  }
  {
    // The log and the summary must be different files: the summary's
    // rewrite would replace the log. Compared as their directories' real
    // paths and their names, whether or not they exist yet
    char a[8192],b[8192];
    canonical(Log_path,a,sizeof a);
    canonical(Summary_path,b,sizeof b);
    if(strcmp(a,b) == 0){
      fprintf(stderr,"%s: the log and the summary must be different files\n",App_path);
      exit(EX_USAGE);
    }
  }
  for(int i = 0; i < nconfigs; i++)
    read_config(configs[i]);
  reload_log();
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
  fprintf(stderr,"%s: radiod %s, %d known frequencies; log %s, summary %s\n",App_path,argv[optind],Nknown,Log_path,Summary_path);

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
    bool const known = Fe_samprate > 0 && max_if > min_if;
    pthread_mutex_unlock(&Lock);
    if(!known){
      expire(wall_ns());
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
    if(have && rbw > 0)
      process(freq - rbw * (n / 2),rbw,n,base,step,bins_copy,now); // (radiod's one-byte bins run low to high from -n/2)
    else
      expire(now); // no spectrum: time still passes
    if(now - last_summary > 60 * (int64_t)BILLION){
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
  return 0;
}
