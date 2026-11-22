// tones: watch radiod's audio streams for fire/EMS paging tones (two-tone
// sequential, long tone), log each page, record it with the dispatch that
// follows, and run a command for the departments one wants told of.
//
//   tones [options] group ...
//   eg: tones -t departments.csv -n Alden -c ~/page-alert.sh fire-dispatch.local
//
// Each page: a line on stdout and in a CSV log; a WAV recording (8 kHz),
// from just before its first tone until the channel has been quiet for a
// while, named after the department ("20261122T120314_Alden_453.500.wav");
// and, once the recording is complete, the command, if the page is a
// department in the notify list, with the page in its environment:
// PAGE_DEPARTMENTS, PAGE_TONES, PAGE_TIME, PAGE_FREQ, PAGE_FILE.
// Pages that match no department are logged and recorded too.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <locale.h>
#include <math.h>
#include <netinet/in.h>
#include <opus/opus.h>
#include <samplerate.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sysexits.h>
#include <time.h>
#include <unistd.h>

#include "misc.h"
#include "multicast.h"
#include "radio.h"
#include "rtp.h"
#include "status.h"
#include "tone-detect.h"

#define MAX_GROUPS 16
#define MAX_SESSIONS 64
#define MAX_DEPARTMENTS 256
#define PREROLL_SECONDS 8   // audio kept, to start a recording before its first tone
#define MAX_RECORDING_DEPARTMENTS 8

int Verbose;
char const *App_path;

static char const *Pages_dir = "pages";
static char const *Log_path;           // default Pages_dir/pages.csv
static char const *Command;
static char const *Notify;             // comma separated department names, or "*"
static double Quiet_seconds = 5;       // a recording ends after this long without audio
static double Max_seconds = 120;       // or at this length
static double Tolerance = 0.015;
static struct td_department Departments[MAX_DEPARTMENTS];
static int N_departments;

struct session {
  bool inuse;
  struct sockaddr_storage sender;
  uint32_t ssrc;
  double freq;                         // Hz, from status (0 until known)
  // what its payload types are, from status
  int pt;
  enum encoding encoding;
  int samprate,channels;
  OpusDecoder *opus;
  SRC_STATE *src;                      // PCM to 8 kHz
  struct tone_detector det;
  float ring[PREROLL_SECONDS * TD_SAMPRATE];
  uint64_t written;                    // samples into the ring (and the detector) in all
  // a page being recorded
  FILE *wav;
  char wav_path[512];
  uint64_t wav_samples;
  double rec_started;                  // monotonic seconds
  double last_audio;                   // monotonic seconds, the last time it wasn't quiet
  char departments[MAX_RECORDING_DEPARTMENTS][64];
  int n_departments;
  char tones[256];
  char page_time[32];
};
static struct session Sessions[MAX_SESSIONS];
static void where(struct session const *sp,char *out,size_t size,bool file);

static double monotonic(void){
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC,&ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Departments: "name,tone A,tone B" (or tab separated, as tables are
// pasted); tone B empty or 0 for a long tone; lines without numbers (a
// header) skipped
static int read_departments(char const *path){
  FILE *fp = fopen(path,"r");
  if(fp == NULL){
    fprintf(stderr,"%s: %s\n",path,strerror(errno));
    return -1;
  }
  char line[512];
  while(fgets(line,sizeof line,fp) != NULL && N_departments < MAX_DEPARTMENTS){
    line[strcspn(line,"\r\n")] = '\0';
    char *fields[3] = {NULL,NULL,NULL};
    int nf = 0;
    char *save = NULL;
    for(char *f = strtok_r(line,",\t",&save); f != NULL && nf < 3; f = strtok_r(NULL,",\t",&save))
      fields[nf++] = f;
    if(nf < 2)
      continue;
    char *end;
    double const a = strtod(fields[1],&end);
    if(end == fields[1] || !(a > 0))
      continue; // a header
    double b = 0;
    if(nf == 3){
      b = strtod(fields[2],&end);
      if(end == fields[2])
	b = 0;
    }
    struct td_department *d = &Departments[N_departments++];
    // trim the name
    char *name = fields[0];
    while(*name == ' ')
      name++;
    snprintf(d->name,sizeof d->name,"%s",name);
    for(size_t n = strlen(d->name); n > 0 && d->name[n - 1] == ' '; n--)
      d->name[n - 1] = '\0';
    d->a_hz = a;
    d->b_hz = b > 0 ? b : 0;
  }
  fclose(fp);
  return N_departments;
}

static bool notify_wanted(char const *name){
  if(Command == NULL || Notify == NULL)
    return false;
  if(strcmp(Notify,"*") == 0)
    return true;
  char list[1024];
  snprintf(list,sizeof list,"%s",Notify);
  char *save = NULL;
  for(char *n = strtok_r(list,",",&save); n != NULL; n = strtok_r(NULL,",",&save)){
    while(*n == ' ')
      n++;
    if(strcasecmp(n,name) == 0)
      return true;
  }
  return false;
}

// A file-name-safe form of a name
static void safe_name(char *out,size_t size,char const *in){
  size_t o = 0;
  for(; *in != '\0' && o + 1 < size; in++)
    out[o++] = (isalnum((unsigned char)*in) || *in == '-' || *in == '.') ? *in : '_';
  out[o] = '\0';
}

static void wav_header(FILE *fp,uint32_t samples){
  uint32_t const data = samples * 2, rate = TD_SAMPRATE, byterate = TD_SAMPRATE * 2;
  uint32_t const riff = 36 + data, fmt = 16;
  uint16_t const pcm = 1, channels = 1, align = 2, bits = 16;
  fseek(fp,0,SEEK_SET);
  fwrite("RIFF",1,4,fp); fwrite(&riff,4,1,fp); fwrite("WAVE",1,4,fp);
  fwrite("fmt ",1,4,fp); fwrite(&fmt,4,1,fp); fwrite(&pcm,2,1,fp); fwrite(&channels,2,1,fp);
  fwrite(&rate,4,1,fp); fwrite(&byterate,4,1,fp); fwrite(&align,2,1,fp); fwrite(&bits,2,1,fp);
  fwrite("data",1,4,fp); fwrite(&data,4,1,fp);
  fseek(fp,0,SEEK_END);
}

static void wav_write(struct session *sp,float const *x,int n){
  for(int i = 0; i < n; i++){
    float v = x[i] * 32767;
    v = v > 32767 ? 32767 : v < -32768 ? -32768 : v;
    int16_t const s = (int16_t)lrintf(v);
    fwrite(&s,2,1,sp->wav);
  }
  sp->wav_samples += n;
}

// The recording is complete: close it, and tell (the command) if wanted
static void finish_recording(struct session *sp){
  if(sp->wav == NULL)
    return;
  wav_header(sp->wav,(uint32_t)sp->wav_samples);
  fclose(sp->wav);
  sp->wav = NULL;
  printf("%s  recorded %s (%.0f s)\n",sp->page_time,sp->wav_path,(double)sp->wav_samples / TD_SAMPRATE);
  fflush(stdout);
  bool tell = false;
  char departments[512] = "";
  for(int i = 0; i < sp->n_departments; i++){
    tell = tell || notify_wanted(sp->departments[i]);
    if(i > 0)
      strlcat(departments,", ",sizeof departments);
    strlcat(departments,sp->departments[i],sizeof departments);
  }
  if(!tell)
    return;
  pid_t const pid = fork();
  if(pid == 0){
    char freq[32];
    where(sp,freq,sizeof freq,false);
    setenv("PAGE_DEPARTMENTS",departments,1);
    setenv("PAGE_TONES",sp->tones,1);
    setenv("PAGE_TIME",sp->page_time,1);
    setenv("PAGE_FREQ",freq,1);
    setenv("PAGE_FILE",sp->wav_path,1);
    execl("/bin/sh","sh","-c",Command,(char *)NULL);
    _exit(127);
  }
  if(pid < 0)
    fprintf(stderr,"fork: %s\n",strerror(errno));
}

// Where a stream is, for logs and file names: its frequency, or its SSRC
// until status has said
static void where(struct session const *sp,char *out,size_t size,bool file){
  if(sp->freq > 0)
    snprintf(out,size,file ? "%.3f" : "%.4f",sp->freq * 1e-6);
  else
    snprintf(out,size,"ssrc%u",sp->ssrc);
}

// A page: log it, and record it (or add it to the recording going on)
static void page(struct session *sp,struct td_event const *ev){
  int const m = td_match(Departments,N_departments,ev,Tolerance);
  char const *name = m >= 0 ? Departments[m].name : "unknown";
  // When it began: its first tone, so long ago in the stream
  time_t const now = time(NULL);
  time_t const began = now - (time_t)((sp->written - ev->start) / TD_SAMPRATE);
  struct tm tm;
  localtime_r(&began,&tm);
  char when[32],stamp[32];
  strftime(when,sizeof when,"%Y-%m-%d %H:%M:%S",&tm);
  strftime(stamp,sizeof stamp,"%Y%m%dT%H%M%S",&tm);
  char tones[64];
  if(ev->type == TD_TWO_TONE)
    snprintf(tones,sizeof tones,"%.1f/%.1f Hz %.1f/%.1f s",ev->a_hz,ev->b_hz,ev->a_seconds,ev->b_seconds);
  else
    snprintf(tones,sizeof tones,"%.1f Hz %.1f s (long tone)",ev->a_hz,ev->a_seconds);
  char at[32];
  where(sp,at,sizeof at,false);
  printf("%s  %s  %s  %s\n",when,at,name,tones);
  fflush(stdout);
  // The CSV log
  char log[600];
  if(Log_path != NULL)
    snprintf(log,sizeof log,"%s",Log_path);
  else
    snprintf(log,sizeof log,"%s/pages.csv",Pages_dir);
  FILE *lf = fopen(log,"a");
  if(lf != NULL){
    if(ftell(lf) == 0)
      fprintf(lf,"time,frequency_mhz,department,type,tone_a_hz,tone_b_hz,a_seconds,b_seconds\n");
    fprintf(lf,"%s,%s,\"%s\",%s,%.1f,%.1f,%.1f,%.1f\n",when,at,name,
	    ev->type == TD_TWO_TONE ? "two-tone" : "long-tone",ev->a_hz,ev->b_hz,ev->a_seconds,ev->b_seconds);
    fclose(lf);
  }
  // The recording
  if(sp->wav == NULL){
    char safe[80];
    safe_name(safe,sizeof safe,name);
    char at_file[32];
    where(sp,at_file,sizeof at_file,true);
    snprintf(sp->wav_path,sizeof sp->wav_path,"%s/%s_%s_%s.wav",Pages_dir,stamp,safe,at_file);
    sp->wav = fopen(sp->wav_path,"w+");
    if(sp->wav == NULL){
      fprintf(stderr,"%s: %s\n",sp->wav_path,strerror(errno));
      return;
    }
    wav_header(sp->wav,0);
    sp->wav_samples = 0;
    sp->rec_started = sp->last_audio = monotonic();
    sp->n_departments = 0;
    sp->tones[0] = '\0';
    snprintf(sp->page_time,sizeof sp->page_time,"%s",when);
    // From half a second before its first tone, as far as the ring reaches
    uint64_t const ring = sizeof sp->ring / sizeof sp->ring[0];
    uint64_t from = ev->start > TD_SAMPRATE / 2 ? ev->start - TD_SAMPRATE / 2 : 0;
    if(sp->written - from > ring)
      from = sp->written - ring;
    for(uint64_t i = from; i < sp->written; i++)
      wav_write(sp,&sp->ring[i % ring],1);
  }
  if(sp->n_departments < MAX_RECORDING_DEPARTMENTS)
    snprintf(sp->departments[sp->n_departments++],sizeof sp->departments[0],"%s",name);
  if(sp->tones[0] != '\0')
    strlcat(sp->tones,"; ",sizeof sp->tones);
  strlcat(sp->tones,tones,sizeof sp->tones);
}

// 8 kHz audio of a session: to the ring, the detector and any recording
static void audio(struct session *sp,float const *x,int n){
  uint64_t const ring = sizeof sp->ring / sizeof sp->ring[0];
  double energy = 0;
  for(int i = 0; i < n; i++){
    sp->ring[(sp->written + i) % ring] = x[i];
    energy += (double)x[i] * x[i];
  }
  struct td_event ev[4];
  int const pages = td_process(&sp->det,x,n,ev,4);
  sp->written += n;
  if(sp->wav != NULL){
    wav_write(sp,x,n);
    if(n > 0 && energy / n > 1e-6) // not silence
      sp->last_audio = monotonic();
  }
  for(int i = 0; i < pages; i++)
    page(sp,&ev[i]);
}

static struct session *session(struct sockaddr_storage const *sender,uint32_t ssrc,bool create){
  struct session *free_one = NULL;
  for(int i = 0; i < MAX_SESSIONS; i++){
    struct session *sp = &Sessions[i];
    if(sp->inuse && sp->ssrc == ssrc && address_match(&sp->sender,sender))
      return sp;
    if(!sp->inuse && free_one == NULL)
      free_one = sp;
  }
  if(!create || free_one == NULL)
    return NULL;
  memset(free_one,0,sizeof *free_one);
  free_one->inuse = true;
  free_one->sender = *sender;
  free_one->ssrc = ssrc;
  free_one->pt = -1;
  if(!td_init(&free_one->det)){
    free_one->inuse = false;
    return NULL;
  }
  return free_one;
}

// Status: a stream's frequency, and what its payload type carries
static void status(uint8_t const *buffer,int length,struct sockaddr_storage const *sender){
  if(length < 2 || buffer[0] != STATUS)
    return;
  uint32_t const ssrc = get_ssrc(buffer + 1,length - 1);
  struct session *sp = session(sender,ssrc,true);
  if(sp == NULL)
    return;
  static chan_t chan;
  static struct frontend frontend;
  memset(&chan,0,sizeof chan);
  decode_radio_status(&frontend,&chan,buffer + 1,length - 1);
  if(chan.tune.freq > 0 && sp->freq == 0 && Verbose)
    fprintf(stderr,"stream %u: %.4f MHz, %s %d Hz\n",ssrc,chan.tune.freq * 1e-6,
	    encoding_string(chan.output.encoding),chan.output.samprate);
  if(chan.tune.freq > 0)
    sp->freq = chan.tune.freq;
  if(chan.output.encoding != NO_ENCODING && chan.output.samprate > 0){
    sp->pt = chan.output.rtp.type & 0x7f;
    sp->encoding = chan.output.encoding;
    sp->samprate = chan.output.samprate;
    sp->channels = chan.output.channels > 0 ? chan.output.channels : 1;
  }
}

static void data(uint8_t *buffer,ssize_t size,struct sockaddr_storage const *sender){
  if(size <= RTP_MIN_SIZE)
    return;
  struct rtp_header rtp;
  uint8_t const *dp = ntoh_rtp(&rtp,buffer);
  size -= dp - buffer;
  if(rtp.pad)
    size -= dp[size - 1];
  if(size <= 0)
    return;
  struct session *sp = session(sender,rtp.ssrc,true);
  if(sp == NULL)
    return;
  // What it is: from status if it came, else the payload type's usual meaning
  enum encoding encoding = sp->pt == rtp.type ? sp->encoding : encoding_from_pt(rtp.type);
  int samprate = sp->pt == rtp.type ? sp->samprate : samprate_from_pt(rtp.type);
  int channels = sp->pt == rtp.type ? sp->channels : channels_from_pt(rtp.type);
  if(encoding == NO_ENCODING && rtp.type == Opus_pt){
    encoding = OPUS;
    samprate = 48000;
    channels = 1;
  }
  float out[2880 * 2];
  if(encoding == OPUS){
    if(sp->opus == NULL){
      int error;
      sp->opus = opus_decoder_create(TD_SAMPRATE,1,&error); // straight to 8 kHz mono
      if(sp->opus == NULL)
	return;
    }
    int const n = opus_decode_float(sp->opus,dp,(opus_int32)size,out,2880,0);
    if(n > 0)
      audio(sp,out,n);
    return;
  }
  if(samprate <= 0 || channels <= 0)
    return;
  // PCM: mono (the first channel), to 8 kHz
  int bytes;
  switch(encoding){
  case S16BE: case S16LE: bytes = 2; break;
  case F32LE: bytes = 4; break;
  default: return;
  }
  int const frames = (int)(size / (bytes * channels));
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
  if(samprate == TD_SAMPRATE){
    audio(sp,in,nin);
    return;
  }
  if(sp->src == NULL){
    int error;
    sp->src = src_new(SRC_SINC_FASTEST,1,&error);
    if(sp->src == NULL)
      return;
  }
  SRC_DATA d = {.data_in = in,.input_frames = nin,.data_out = out,.output_frames = (long)(sizeof out / sizeof out[0]),
		.src_ratio = (double)TD_SAMPRATE / samprate,.end_of_input = 0};
  if(src_process(sp->src,&d) == 0 && d.output_frames_gen > 0)
    audio(sp,out,(int)d.output_frames_gen);
}

static volatile sig_atomic_t Stop;
static void stop(int s){ (void)s; Stop = 1; }

static struct option const Options[] = {
  {"departments",required_argument,NULL,'t'},
  {"pages",required_argument,NULL,'d'},
  {"log",required_argument,NULL,'l'},
  {"command",required_argument,NULL,'c'},
  {"notify",required_argument,NULL,'n'},
  {"quiet",required_argument,NULL,'q'},
  {"max",required_argument,NULL,'m'},
  {"tolerance",required_argument,NULL,'T'},
  {"verbose",no_argument,NULL,'v'},
  {"version",no_argument,NULL,'V'},
  {NULL,0,NULL,0},
};

static void usage(void){
  fprintf(stderr,"Usage: %s [-t departments.csv] [-d pages-dir] [-l log.csv] [-c command] [-n dept,dept|*]"
	  " [-q quiet-s] [-m max-s] [-T tolerance] [-v] group ...\n",App_path);
}

int main(int argc,char *argv[]){
  App_path = argv[0];
  setlocale(LC_ALL,"");
  int c;
  while((c = getopt_long(argc,argv,"t:d:l:c:n:q:m:T:vV",Options,NULL)) != -1){
    switch(c){
    case 't':
      if(read_departments(optarg) < 0)
	exit(EX_NOINPUT);
      break;
    case 'd': Pages_dir = optarg; break;
    case 'l': Log_path = optarg; break;
    case 'c': Command = optarg; break;
    case 'n': Notify = optarg; break;
    case 'q': Quiet_seconds = strtod(optarg,NULL); break;
    case 'm': Max_seconds = strtod(optarg,NULL); break;
    case 'T': Tolerance = strtod(optarg,NULL); break;
    case 'v': Verbose++; break;
    case 'V': VERSION(); exit(EX_OK);
    default: usage(); exit(EX_USAGE);
    }
  }
  if(!(isfinite(Quiet_seconds) && Quiet_seconds > 0 && Quiet_seconds <= 600)
     || !(isfinite(Max_seconds) && Max_seconds >= 5 && Max_seconds <= 3600)
     || !(isfinite(Tolerance) && Tolerance > 0 && Tolerance < 0.1)){
    fprintf(stderr,"quiet 0-600 s, max 5-3600 s, tolerance 0-0.1\n");
    exit(EX_USAGE);
  }
  if(optind >= argc){
    usage();
    exit(EX_USAGE);
  }
  if(mkdir(Pages_dir,0755) != 0 && errno != EEXIST){
    fprintf(stderr,"%s: %s\n",Pages_dir,strerror(errno));
    exit(EX_CANTCREAT);
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
      if(fd < 0){
	fprintf(stderr,"%s: can't listen\n",argv[i]);
	continue;
      }
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
  signal(SIGCHLD,SIG_IGN); // commands' exits reaped
  fprintf(stderr,"tones: %d department%s, watching %d group%s; pages in %s\n",N_departments,N_departments == 1 ? "" : "s",
	  nfds / 2,nfds / 2 == 1 ? "" : "s",Pages_dir);
  uint8_t buffer[PKTSIZE];
  while(!Stop){
    fd_set set;
    FD_ZERO(&set);
    for(int i = 0; i < nfds; i++)
      FD_SET(fds[i],&set);
    struct timeval tv = {.tv_sec = 0,.tv_usec = 250000};
    int const r = select(maxfd + 1,&set,NULL,NULL,&tv);
    if(r < 0 && errno != EINTR)
      break;
    for(int i = 0; r > 0 && i < nfds; i++){
      if(!FD_ISSET(fds[i],&set))
	continue;
      struct sockaddr_storage sender;
      socklen_t len = sizeof sender;
      ssize_t const size = recvfrom(fds[i],buffer,sizeof buffer,0,(struct sockaddr *)&sender,&len);
      if(size <= 0)
	continue;
      if(Verbose > 1){
	static long counts[2 * MAX_GROUPS];
	if(counts[i]++ == 0)
	  fprintf(stderr,"first %s packet on socket %d: %zd bytes, type %u\n",is_status[i] ? "status" : "data",i,size,buffer[0]);
      }
      if(is_status[i])
	status(buffer,(int)size,&sender);
      else
	data(buffer,size,&sender);
    }
    // Recordings whose channel has gone quiet (radiod sends nothing while
    // squelched), or that are long enough
    double const now = monotonic();
    for(int k = 0; k < MAX_SESSIONS; k++){
      struct session *sp = &Sessions[k];
      if(sp->inuse && sp->wav != NULL && (now - sp->last_audio > Quiet_seconds || now - sp->rec_started > Max_seconds))
	finish_recording(sp);
    }
  }
  for(int k = 0; k < MAX_SESSIONS; k++)
    if(Sessions[k].inuse)
      finish_recording(&Sessions[k]);
  return 0;
}
