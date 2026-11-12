// Call tracking and PCM queue regression tests. Include the demodulator so
// these exercise the production queue; unused radio paths are discarded by
// --gc-sections. The fake decoder changes IDs at the instant it emits PCM.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dv.c"

int Verbose;
static dv_stats Frame_stats;
static uint64_t Follow_generation;

void dv_get_stats(dv_decoder const *decoder,dv_stats *stats){
  (void)decoder;
  *stats = Frame_stats;
}

char const *dv_mode_name(int mode){
  return mode == DV_DMR ? "DMR" : "P25";
}

bool dv_follow_call(struct dv_follow const *f,char *dest,size_t dlen,char *src,size_t slen,
                    double *freq,int *slot,uint64_t *generation,uint64_t *serial){
  (void)f;
  snprintf(dest,dlen,"%s",Frame_stats.dst_call);
  snprintf(src,slen,"%s",Frame_stats.src_call);
  *freq = 852000000;
  *slot = 0;
  *generation = Follow_generation;
  *serial = Frame_stats.call_serial;
  return true;
}

static void check(bool ok,char const *what){
  if(!ok){
    fprintf(stderr,"FAIL: %s\n",what);
    exit(EXIT_FAILURE);
  }
}

static void frame(struct dv_audio_queue *q,char const *dest,char const *src,uint64_t serial,int sample){
  memset(&Frame_stats,0,sizeof Frame_stats);
  snprintf(Frame_stats.dst_call,sizeof Frame_stats.dst_call,"%s",dest);
  snprintf(Frame_stats.src_call,sizeof Frame_stats.src_call,"%s",src);
  Frame_stats.call_serial = serial;
  int16_t pcm[DV_PCM_SAMPLES] = {sample};
  queue_pcm(q,pcm);
}

int main(void){
  chan_t chan = {0};
  chan.tune.freq = 851000000;
  struct dv_call current = {0};
  struct dv_audio_queue q = {.current=&current,.chan=&chan,.mode=DV_P25_C4FM};

  // A and B can be emitted in one dv_process block. A's queued tail must
  // remain A, while B's very first PCM frame must already carry B's IDs.
  frame(&q,"100","1",0,11);
  uint32_t const a = q.call[0].id;
  frame(&q,"200","2",0,22);
  check(a != 0 && q.call[0].id == a && strcmp(q.call[0].dest,"100") == 0,"queued A preserved");
  check(q.call[1].id != a && strcmp(q.call[1].dest,"200") == 0 && q.samples[1][0] == 22,"first B frame labelled B");

  // An explicit end followed immediately by the same IDs is a new call;
  // no empty-ID observation or 500 ms wait is required.
  uint32_t const b = q.call[1].id;
  frame(&q,"200","2",1,33);
  check(q.call[2].id != b && q.call[2].id != 0,"same-talker immediate re-key");
  check(q.call[1].id == b,"previous call remains queued under its own ID");

  // Definitive end during a short gap, versus uncertain ID loss in a fade.
  struct dv_call none = {.freq=chan.tune.freq,.slot=-1,.serial=2};
  track_call(&chan,&current,&none,&q,true);
  check(current.id == 0,"terminator ends call immediately");
  frame(&q,"200","2",2,44);
  uint32_t const rekey = current.id;
  none.serial = 2;
  for(int i=0;i<10;i++)
    track_call(&chan,&current,&none,&q,true);
  check(current.id == rekey,"200 ms fade preserves call");
  frame(&q,"200","2",2,55);
  check(current.id == rekey,"IDs return after fade without splitting call");

  // Late identification backfills only unknown frames of this transmission.
  frame(&q,"","",3,66);
  unsigned const unknown_old = q.count - 1;
  frame(&q,"","",4,77);
  unsigned const unknown_new = q.count - 1;
  frame(&q,"300","3",4,88);
  check(q.call[unknown_old].id == 0,"late IDs do not relabel an ended transmission");
  check(q.call[unknown_new].id == current.id && strcmp(q.call[unknown_new].dest,"300") == 0,"late IDs label this transmission's queued start");

  // A new followed decoder starts its serial counter over. Even if it joins
  // the same talkgroup/frequency immediately, its PCM belongs to a new call.
  q.follow = (struct dv_follow const *)&chan; // opaque, never dereferenced
  Follow_generation = 1;
  frame(&q,"400","4",0,99);
  uint32_t const followed = current.id;
  unsigned const followed_tail = q.count - 1;
  Follow_generation = 2;
  frame(&q,"400","4",0,100);
  check(current.id != followed && q.call[followed_tail].id == followed,"followed decoder replacement preserves queued call identity");
  check(q.call[q.count - 1].freq == 852000000 && q.call[q.count - 1].slot == 0,"followed frame retains voice frequency and slot");

  puts("Call queue tests passed");
  return EXIT_SUCCESS;
}
