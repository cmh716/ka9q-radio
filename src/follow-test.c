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

// transmission: the control channel's number for an explicit grant (0: none seen)
static void grant(dv_event_type type, uint64_t hz, uint32_t tg, uint32_t src, uint32_t transmission){
  dv_event e = { .type = type, .freq_hz = hz, .slot = 0, .group = tg, .source = src, .transmission = transmission };
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
  grant(DV_EVENT_GRANT,851287500,10003,3451001,1);
  run(2,false);
  check(following() == 851287500,"joins the first grant");
  run(50,true);
  grant(DV_EVENT_GRANT,851912500,10003,9989123,2);   // the reply, elsewhere, no END_PTT seen
  run(1,false);
  check(following() == 851912500,"moves to the reply's channel at once (was: 2 s hang)");

  // The voice channel signals the end of the transmission (MAC_END_PTT)
  run(25,true);
  Voice_stats.call_serial++;
  run(1,false);
  check(F->voice == NULL || following() != 851912500,"leaves when the transmission ends");

  // The next talker re-uses the channel just left, announced by an update
  grant(DV_EVENT_UPDATE,851912500,10003,3421116,2);
  run(1,false);
  check(following() == 851912500,"rejoins the same channel for a new talker");

  // A re-announcement of the transmission just left (same talker) is not rejoined
  Voice_stats.call_serial++;
  run(1,false);
  grant(DV_EVENT_UPDATE,851912500,10003,3421116,2);
  run(1,false);
  check(F->voice == NULL,"doesn't rejoin a transmission it has finished");

  // Without any end signal or new grant, the 2 s hang still applies
  grant(DV_EVENT_GRANT,852362500,10003,3451001,3);
  run(2,false);
  run(10,true);
  check(following() == 852362500,"follows a new grant");
  run(60,false);
  check(following() == 852362500,"holds through 1.2 s of silence");
  run(50,false);
  check(F->voice == NULL,"leaves after the 2 s hang");

  // Codex: the same talker keys up again on the channel just left. The
  // control channel repeats the explicit grant, which it reports as an
  // update because it knows that channel and talkgroup; the grant number is new
  grant(DV_EVENT_GRANT,852812500,10003,9989123,4);
  run(2,false);
  run(10,true);
  check(following() == 852812500,"follows TG 10003 on 852.8125");
  Voice_stats.call_serial++;                      // MAC_END_PTT
  run(1,false);
  grant(DV_EVENT_UPDATE,852812500,10003,9989123,5); // same talker, explicit grant again
  run(1,false);
  check(following() == 852812500,"rejoins the same talker re-granted on the same channel");

  // Codex: with no talkgroup list, an unrelated grant arriving in the same
  // block mustn't hide the conversation's reply
  run(10,true);
  grant(DV_EVENT_GRANT,853037500,10999,1234567,6);  // unrelated
  grant(DV_EVENT_GRANT,851287500,10003,3451001,7);  // the reply
  run(1,false);
  check(following() == 851287500,"moves to the reply despite an unrelated grant");

  // Codex: a reply granted in the very block after joining (the same
  // sample count as the join) is still a later transmission
  run(10,true);
  Voice_stats.call_serial++;
  run(1,false);
  grant(DV_EVENT_GRANT,852362500,10003,3421116,8);
  run(1,false);                                   // joins 852.3625 here
  check(following() == 852362500,"joins the next transmission");
  grant(DV_EVENT_GRANT,851912500,10003,9989123,9); // decoded in the next block
  run(1,false);
  check(following() == 851912500,"follows a reply granted right after joining");

  // The re-grant cases, each on a fresh follower: our own channel granted
  // again during the transmission we follow (10), as a re-announcement or
  // as the next transmission (a quick re-key on the same channel)
  struct regrant { char const *what; int before_end; int gap; bool rekey; };
  static struct regrant const cases[] = {
    {"re-announced, voice 0.8 s on after it: not rejoined",40,0,false},
    {"re-announced, voice 0.2 s on after it (Codex): not rejoined",10,0,false},
    {"re-key granted 0.4 s before the end, voice 0.3 s after (Codex): followed",20,15,true},
    {"re-key granted just before the end, voice 0.3 s after: followed",2,15,true},
    {"re-key granted 0.4 s before the end, voice 1.2 s after: missed (the documented limit)",20,60,true},
  };
  for(size_t k = 0; k < sizeof cases / sizeof cases[0]; k++){
    struct regrant const *c = &cases[k];
    dv_follow_destroy(F);
    memset(&Voice_stats,0,sizeof Voice_stats);
    F = dv_follow_create(&Chan,NULL,NULL);
    grant(DV_EVENT_GRANT,853037500,10003,3421241,10);
    run(2,false);
    run(25,true);
    bool ok = following() == 853037500;
    grant(DV_EVENT_UPDATE,853037500,10003,3421241,11); // our channel, granted again
    run(c->before_end,true);
    Voice_stats.call_serial++;                         // the end of 10
    run(1,false);
    grant(DV_EVENT_UPDATE,853037500,10003,3421241,11);
    run(c->gap,false);
    bool const listening = F->voice != NULL;
    uint64_t const heard = Voice_stats.voice_decoded;
    if(c->rekey){
      run(25,true);                                    // 11 talks, if anyone listens
      grant(DV_EVENT_UPDATE,853037500,10003,3421241,11);
      run(1,false);
    } else {
      run(60,false);                                   // past the listening window
      grant(DV_EVENT_UPDATE,853037500,10003,3421241,11);
      run(5,false);
    }
    bool const followed = Voice_stats.voice_decoded > heard && listening;
    bool const limit = c->gap > 50;
    ok = ok && (c->rekey ? followed != limit : F->voice == NULL);
    check(ok,c->what);
  }

  dv_follow_destroy(F);
  printf(Failures ? "follower tests FAILED\n" : "Follower tests passed\n");
  return Failures ? 1 : 0;
}
