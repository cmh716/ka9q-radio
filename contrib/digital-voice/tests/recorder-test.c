// pcmrecord --by-talkgroup end to end: runs pcmrecord, sends it radiod-like
// RTP audio and status over local multicast, and checks every 20 ms frame
// of every file is the right call's (each call is a tone of its own).
// Scenarios: a packet reordered across a call change, a status arriving
// after its call's first audio, a preemption with the audio sent in a
// burst, and the same talker keying up again 0.3 s after a call ends.
//
//   recorder-test path/to/pcmrecord work-directory
//
// SPDX-License-Identifier: GPL-3.0-or-later
#define _GNU_SOURCE 1
#include <arpa/inet.h>
#include <dirent.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rtp.h"
#include "status.h"
#include "multicast.h"

#define GROUP "239.77.1.3"
static int Fd, Pt;
static struct sockaddr_in Data, Stat;
static uint32_t const Ssrc = 851912;

static void status(char const *dest, char const *src, unsigned id, uint32_t ts){
  uint8_t buf[512], *bp = buf;
  *bp++ = STATUS;
  encode_int32(&bp,OUTPUT_SSRC,Ssrc); encode_int(&bp,RTP_PT,Pt); encode_int(&bp,OUTPUT_SAMPRATE,8000);
  encode_int(&bp,OUTPUT_CHANNELS,1); encode_int(&bp,OUTPUT_ENCODING,S16BE);
  encode_double(&bp,RADIO_FREQUENCY,851912500.0); encode_int(&bp,DEMOD_TYPE,6);
  encode_string(&bp,CALL_DEST,dest,strlen(dest)); encode_string(&bp,CALL_SOURCE,src,strlen(src));
  encode_string(&bp,CALL_MODE,"P25",3); encode_int32(&bp,CALL_ID,id);
  encode_double(&bp,CALL_FREQUENCY,851912500.0); encode_int(&bp,CALL_SLOT,-1);
  encode_int32(&bp,CALL_TIMESTAMP,ts); encode_eol(&bp);
  sendto(Fd,buf,(size_t)(bp - buf),0,(struct sockaddr *)&Stat,sizeof Stat);
}

// Packet at stream position k (RTP timestamp 160 k), 20 ms of a tone. As
// from radiod, sequence numbers run on while timestamps jump over silence
static uint16_t Seq;
static void pkt_seq(int k, double hz, int pace, uint16_t seq){
  uint8_t p[1500];
  struct rtp_header h = {.version = RTP_VERS, .type = Pt, .ssrc = Ssrc, .seq = seq, .timestamp = (uint32_t)(160 * k)};
  int16_t *d = hton_rtp(p,&h);
  for(int i = 0; i < 160; i++)
    d[i] = (int16_t)htons((uint16_t)(int16_t)(8000 * sin(2 * M_PI * hz * (160 * k + i) / 8000.0)));
  sendto(Fd,p,(size_t)((uint8_t *)(d + 160) - p),0,(struct sockaddr *)&Data,sizeof Data);
  if(pace)
    usleep(20000);
}
static void pkt(int k, double hz, int pace){
  pkt_seq(k,hz,pace,Seq++);
}

static void send_scenarios(void){
  status("","",0,0);
  usleep(300000);
  // TG 100 (440 Hz), packets 0-49; its last arrives after TG 200's first,
  // with TG 200's status between them
  status("100","1",1,0);
  for(int k = 0; k < 49; k++) pkt(k,440,1);
  pkt_seq(50,880,0,(uint16_t)(Seq + 1)); // B's first, early
  status("200","2",2,160 * 50);
  pkt_seq(49,440,0,Seq);                 // A's last, late
  Seq += 2;
  for(int k = 51; k < 100; k++) pkt(k,880,1);       // TG 200: 50-99
  // TG 300 (660 Hz): its first 0.1 s arrives before its status
  for(int k = 100; k < 105; k++) pkt(k,660,1);
  status("300","3",3,160 * 100);
  for(int k = 105; k < 150; k++) pkt(k,660,1);      // TG 300: 100-149
  status("","",0,160 * 150);
  usleep(400000);
  // TG 400 (550 Hz) preempted by TG 500 (770 Hz), all sent in a burst
  status("400","4",4,160 * 170);
  for(int k = 170; k < 220; k++) pkt(k,550,0);
  status("500","5",5,160 * 220);
  for(int k = 220; k < 270; k++) pkt(k,770,0);
  status("","",0,160 * 270);
  usleep(300000);
  // TG 500 again, same talker, 0.3 s after its end: a new call
  status("500","5",6,160 * 290);
  for(int k = 290; k < 315; k++) pkt(k,770,1);      // 0.5 s
  status("","",0,160 * 315);
  usleep(1500000);
}

// The tone of a 160-sample frame by Goertzel at the candidates; 0 if silent
static int tone(int16_t const *x){
  static int const hz[] = {440,550,660,770,880};
  double best = 0;
  int which = 0;
  for(int t = 0; t < 5; t++){
    double const w = 2 * M_PI * hz[t] / 8000.0, c = 2 * cos(w);
    double s1 = 0, s2 = 0;
    for(int i = 0; i < 160; i++){
      double const s0 = x[i] + c * s1 - s2;
      s2 = s1; s1 = s0;
    }
    double const p = s1 * s1 + s2 * s2 - c * s1 * s2;
    if(p > best){ best = p; which = hz[t]; }
  }
  return best > 1e9 ? which : 0;
}

static int Failures;
static void check(char const *path, char const *tg, int want_hz, int want_frames){
  FILE *f = fopen(path,"rb");
  if(f == NULL){ printf("FAIL: can't open %s\n",path); Failures++; return; }
  static uint8_t buf[1 << 20];
  size_t const n = fread(buf,1,sizeof buf,f);
  fclose(f);
  uint8_t const *data = memmem(buf,n,"data",4);
  if(data == NULL){ printf("FAIL: %s: no data chunk\n",path); Failures++; return; }
  int16_t const *x = (int16_t const *)(data + 8);
  int const frames = (int)((n - (size_t)(data + 8 - buf)) / 320);
  int wrong = 0;
  for(int i = 0; i < frames; i++)
    if(tone(x + 160 * i) != want_hz)
      wrong++;
  bool const ok = frames == want_frames && wrong == 0;
  printf("%s TG %s: %d frames (want %d), %d not %d Hz\n",ok ? "ok  " : "FAIL",tg,frames,want_frames,wrong,want_hz);
  if(!ok)
    Failures++;
}

int main(int argc, char **argv){
  if(argc != 3){
    fprintf(stderr,"usage: %s pcmrecord workdir\n",argv[0]);
    return 2;
  }
  char rec[4096], log[4096];
  snprintf(rec,sizeof rec,"%s/rec",argv[2]);
  snprintf(log,sizeof log,"%s/calls.csv",argv[2]);
  mkdir(argv[2],0777);
  mkdir(rec,0777);
  unlink(log);
  pid_t const pid = fork();
  if(pid == 0){
    execl(argv[1],argv[1],"--by-talkgroup","--call-log",log,"-d",rec,GROUP,(char *)NULL);
    _exit(127);
  }
  sleep(1);
  Pt = pt_from_info(8000,1,S16BE);
  Fd = socket(AF_INET,SOCK_DGRAM,0);
  int loop = 1, ttl = 0;
  setsockopt(Fd,IPPROTO_IP,IP_MULTICAST_LOOP,&loop,sizeof loop);
  setsockopt(Fd,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof ttl);
  Data.sin_family = Stat.sin_family = AF_INET;
  inet_pton(AF_INET,GROUP,&Data.sin_addr);
  Stat.sin_addr = Data.sin_addr;
  Data.sin_port = htons(5004);
  Stat.sin_port = htons(5006);
  send_scenarios();
  kill(pid,SIGTERM);
  waitpid(pid,NULL,0);

  // Each call log line: start,seconds,talkgroup,source,mode,frequency,slot,ssrc,file
  static int const want_hz[] = {440,880,660,550,770,770};
  static int const want_frames[] = {50,50,50,50,50,25};
  static char const *const want_tg[] = {"100","200","300","400","500","500"};
  FILE *f = fopen(log,"r");
  char line[8192];
  int calls = 0;
  if(f != NULL && fgets(line,sizeof line,f) != NULL){
    while(fgets(line,sizeof line,f) != NULL && calls < 6){
      char *fields[9] = {0};
      char *p = line;
      for(int i = 0; i < 9 && p != NULL; i++)
	fields[i] = strsep(&p,",");
      if(fields[8] == NULL)
	continue;
      fields[8][strcspn(fields[8],"\n")] = '\0';
      if(strcmp(fields[2],want_tg[calls]) != 0){
	printf("FAIL: call %d filed under %s, want %s\n",calls + 1,fields[2],want_tg[calls]);
	Failures++;
      }
      check(fields[8],fields[2],want_hz[calls],want_frames[calls]);
      calls++;
    }
  }
  if(f != NULL)
    fclose(f);
  if(calls != 6){
    printf("FAIL: %d calls recorded, want 6\n",calls);
    Failures++;
  }
  printf(Failures ? "recorder tests FAILED\n" : "Recorder call boundary tests passed\n");
  return Failures ? 1 : 0;
}
