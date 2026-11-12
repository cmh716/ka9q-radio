// Grant follower regression tests: dv_follow.c itself, with the tuner
// stubbed out and a scripted voice decoder, driven the way a transmission
// trunked system drives it (each push-to-talk granted its own channel).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dv_follow.c"

int Verbose;
struct frontend Frontend;
double Blocktime = 0.02;
int Overlap = 5;

// --- stand-ins for the filter bank and tuning (no signal processing here)
static float complex Block[960];
int create_filter_output(struct filter_out *out, struct filter_in *in, int olen, enum filtertype t){
  (void)in; (void)t; out->olen = olen; out->output.c = Block; return 0;
}
int set_filter(struct filter_out *out, float lo, float hi, float beta){ (void)out; (void)lo; (void)hi; (void)beta; return 0; }
int execute_filter_output(struct filter_out *out, int shift){ (void)out; (void)shift; return 0; }
int delete_filter_output(struct filter_out *out){ (void)out; return 0; }
int compute_tuning(int N, double samprate, int *shift, double *remainder, double freq){
  (void)N; (void)samprate; (void)freq; *shift = 0; *remainder = 0; return 0;
}
uint64_t mixdown(float complex *b, int n, uint64_t phase, uint64_t inc){ (void)b; (void)n; (void)inc; return phase; }

// --- a scripted voice decoder: the test sets what it reports
struct dv_decoder { int unused; };
static struct dv_decoder Voice;
static dv_stats Voice_stats;
static int Created;
dv_decoder *dv_create(dv_mode mode, dv_config const *c, dv_pcm_callback cb, void *u){
  (void)mode; (void)c; (void)cb; (void)u;
  memset(&Voice_stats,0,sizeof Voice_stats);
  Created++;
  return &Voice;
}
void dv_destroy(dv_decoder *d){ (void)d; }
void dv_process(dv_decoder *d, float const *iq, size_t n){ (void)d; (void)iq; (void)n; }
void dv_get_stats(dv_decoder const *d, dv_stats *s){ (void)d; *s = Voice_stats; }

static int Failures;
static void check(bool ok, char const *what){
  printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if(!ok)
    Failures++;
}

static struct dv_follow *F;
static chan_t Chan;
static dv_stats Cc = { .wacn = 0x458AD, .sysid = 0x229, .last_nac = 0x469 };

static void grant(dv_event_type type, uint64_t hz, uint32_t tg, uint32_t src){
  dv_event e = { .type = type, .freq_hz = hz, .slot = 0, .group = tg, .source = src };
  dv_follow_event(F,&e);
}
// n 20 ms blocks; with voice, the decoder plays a frame each block
static void run(int n, bool voice){
  for(int i = 0; i < n; i++){
    if(voice && F->voice != NULL)
      Voice_stats.voice_decoded++;
    dv_follow_block(F,&Cc,960);
  }
}
static uint64_t following(void){ return F->voice != NULL ? F->cur.freq : 0; }

int main(void){
  Chan.p25.follow = true;
  Chan.filter.min_IF = -6000;
  Chan.filter.max_IF = 6000;
  Chan.tune.freq = 851462500;
  static struct frontend fe;
  Chan.frontend = &fe;
  F = dv_follow_create(&Chan,NULL,NULL);

  // A conversation on TG 10003: each reply on a new channel, the next grant
  // arriving 0.1 s after the previous transmission's last voice
  grant(DV_EVENT_GRANT,851287500,10003,3451001);
  run(2,false);
  check(following() == 851287500,"joins the first grant");
  run(50,true);
  grant(DV_EVENT_GRANT,851912500,10003,9989123);   // the reply, elsewhere, no END_PTT seen
  run(1,false);
  check(following() == 851912500,"moves to the reply's channel at once (was: 2 s hang)");

  // The voice channel signals the end of the transmission (MAC_END_PTT)
  run(25,true);
  Voice_stats.call_serial++;
  run(1,false);
  check(F->voice == NULL || following() != 851912500,"leaves when the transmission ends");

  // The next talker re-uses the channel just left, announced by an update
  grant(DV_EVENT_UPDATE,851912500,10003,3421116);
  run(1,false);
  check(following() == 851912500,"rejoins the same channel for a new talker");

  // A re-announcement of the transmission just left (same talker) is not rejoined
  Voice_stats.call_serial++;
  run(1,false);
  grant(DV_EVENT_UPDATE,851912500,10003,3421116);
  run(1,false);
  check(F->voice == NULL,"doesn't rejoin a transmission it has finished");

  // Without any end signal or new grant, the 2 s hang still applies
  grant(DV_EVENT_GRANT,852362500,10003,3451001);
  run(2,false);
  run(10,true);
  check(following() == 852362500,"follows a new grant");
  run(60,false);
  check(following() == 852362500,"holds through 1.2 s of silence");
  run(50,false);
  check(F->voice == NULL,"leaves after the 2 s hang");

  dv_follow_destroy(F);
  printf(Failures ? "follower tests FAILED\n" : "Follower tests passed\n");
  return Failures ? 1 : 0;
}
