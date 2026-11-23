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
static int Tunes;
double set_freq(chan_t *chan, double f){ chan->tune.freq = f; Tunes++; return f; }

// --- scripted voice decoders: the test sets what each reports
struct dv_decoder { dv_stats st; dv_config config; dv_mode mode; };
static struct dv_decoder Decoders[256];
static int Created;
dv_decoder *dv_create(dv_mode mode, dv_config const *c, dv_pcm_callback cb, void *u){
  (void)cb; (void)u;
  struct dv_decoder *d = &Decoders[Created++ % 256];
  memset(d,0,sizeof *d);
  d->config = *c;
  d->mode = mode;
  return d;
}
void dv_destroy(dv_decoder *d){ (void)d; }
void dv_process(dv_decoder *d, float const *iq, size_t n){ (void)d; (void)iq; (void)n; }
void dv_get_stats(dv_decoder const *d, dv_stats *s){ *s = d->st; }

static int Failures;
static void check(bool ok, char const *what){
  printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if(!ok)
    Failures++;
}

static struct dv_follow *F;
static chan_t Chan;
// A lane's decoder's stats (a stand-in when it has none)
static dv_stats *lane_stats(int lane){
  static dv_stats none;
  return F != NULL && F->lanes[lane].voice != NULL ? &F->lanes[lane].voice->st : &none;
}
#define Voice_stats (*lane_stats(0))
static dv_stats Cc = { .wacn = 0x458AD, .sysid = 0x229, .last_nac = 0x469 };

// transmission: the control channel's number for an explicit grant (0: none seen)
static void grant(dv_event_type type, uint64_t hz, uint32_t tg, uint32_t src, uint32_t transmission){
  dv_event e = { .type = type, .freq_hz = hz, .slot = 0, .group = tg, .source = src, .transmission = transmission };
  dv_follow_event(F,&e);
}
// n 20 ms blocks; with voice, the decoder plays a frame each block
// n 20 ms blocks; with voice, every followed call plays a frame each block
static void run(int n, bool voice){
  for(int i = 0; i < n; i++){
    for(int l = 0; voice && l < F->nlanes; l++)
      if(F->lanes[l].voice != NULL)
	F->lanes[l].voice->st.voice_decoded++;
    dv_follow_block(F,&Cc,960);
  }
}
static uint64_t following(void){ return F->lanes[0].voice != NULL ? F->lanes[0].cur.freq : 0; }
// The talkgroup lane l follows, 0 if none
static uint32_t lane_tg(int l){ return F->lanes[l].voice != NULL ? F->lanes[l].cur.group : 0; }
// The lane following talkgroup tg, -1 if none; and how many lanes are free
static int lane_of(uint32_t tg){
  for(int l = 0; l < F->nlanes; l++)
    if(lane_tg(l) == tg)
      return l;
  return -1;
}
static int free_lanes(void){
  int n = 0;
  for(int l = 0; l < F->nlanes; l++)
    n += F->lanes[l].voice == NULL;
  return n;
}

int main(void){
  Chan.p25.follow = true;
  Chan.filter.min_IF = -6000;
  Chan.filter.max_IF = 6000;
  Chan.tune.freq = 851462500;
  static struct frontend fe;
  Chan.frontend = &fe;
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);

  // Between calls the status text says whether the control channel is heard
  char text[20];
  run(5,false);
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"no CC") == 0,"idle with no control messages yet: \"no CC\"");
  Cc.tsbks = 40;
  run(5,false);
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"CC N469") == 0,"idle with control messages decoding: \"CC N469\"");
  Cc.tsbk_errors = 500; // only errors since
  run(95,false); // 4 + 95 blocks of 20 ms since the last good one
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"CC N469") == 0,"still \"CC N469\" 2 s after the last good message");
  run(3,false);
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"no CC") == 0,"then \"no CC\"");
  Cc.tsbks = 41;
  run(1,false);
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"CC N469") == 0,"and \"CC N469\" again with the next good one");

  // A conversation on TG 10003: each reply on a new channel, the next grant
  // arriving 0.1 s after the previous transmission's last voice
  grant(DV_EVENT_GRANT,851287500,10003,3451001,1);
  run(2,false);
  check(following() == 851287500,"joins the first grant");
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"TG10003 851.2875") == 0,"a call shows its talkgroup and channel");
  run(50,true);
  grant(DV_EVENT_GRANT,851912500,10003,9989123,2);   // the reply, elsewhere, no END_PTT seen
  run(1,false);
  check(following() == 851912500,"moves to the reply's channel at once (was: 2 s hang)");

  // The voice channel signals the end of the transmission (MAC_END_PTT)
  run(25,true);
  Voice_stats.call_serial++;
  run(1,false);
  check(F->lanes[0].voice == NULL || following() != 851912500,"leaves when the transmission ends");

  // The next talker re-uses the channel just left, announced by an update
  grant(DV_EVENT_UPDATE,851912500,10003,3421116,2);
  run(1,false);
  check(following() == 851912500,"rejoins the same channel for a new talker");

  // A re-announcement of the transmission just left (same talker) is not rejoined
  Voice_stats.call_serial++;
  run(1,false);
  grant(DV_EVENT_UPDATE,851912500,10003,3421116,2);
  run(1,false);
  check(F->lanes[0].voice == NULL,"doesn't rejoin a transmission it has finished");

  // Without any end signal or new grant, the 2 s hang still applies
  grant(DV_EVENT_GRANT,852362500,10003,3451001,3);
  run(2,false);
  run(10,true);
  check(following() == 852362500,"follows a new grant");
  run(60,false);
  check(following() == 852362500,"holds through 1.2 s of silence");
  run(50,false);
  check(F->lanes[0].voice == NULL,"leaves after the 2 s hang");

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
  // sample count as the join) is still a later transmission. (The
  // unrelated call is gone by then: its grant isn't repeated for 1.6 s)
  run(80,true);
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
    F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
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
    bool const listening = F->lanes[0].voice != NULL;
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
    ok = ok && (c->rekey ? followed != limit : F->lanes[0].voice == NULL);
    check(ok,c->what);
  }

  // Fable: an encrypted call is left even when some frames were played
  dv_follow_destroy(F);
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  grant(DV_EVENT_UPDATE,852362500,10003,3421116,40); // learned from updates: no service options
  run(2,false);
  run(10,true);
  Voice_stats.m17_encrypted = true;                  // its LDU2 says encrypted
  run(1,false);
  check(F->lanes[0].voice == NULL,"leaves an encrypted call even after frames were played");

  // Fable: until the control channel names the network (WACN), a Phase 2
  // grant can't be followed, and mustn't keep a Phase 1 call from being
  dv_follow_destroy(F);
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  uint32_t const wacn = Cc.wacn;
  Cc.wacn = 0;
  grant(DV_EVENT_GRANT,851912500,10003,3421116,50);  // TDMA (slot 0)
  dv_event fdma = { .type = DV_EVENT_GRANT, .freq_hz = 852812500, .slot = -1, .group = 10029, .source = 9989125, .transmission = 51 };
  dv_follow_event(F,&fdma);
  run(2,false);
  check(following() == 852812500,"follows a Phase 1 call while a Phase 2 grant waits for the WACN");
  Cc.wacn = wacn;

  dv_follow_destroy(F);

  int const created_before = Created;
  // Codex: a transmission left for its reply stays left when the reply ends
  // too, while the control channel still announces both
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  grant(DV_EVENT_GRANT,852362500,10003,3421116,60);
  run(50,true);
  grant(DV_EVENT_GRANT,851912500,10003,9989123,61);  // the reply
  run(2,true);
  check(following() == 851912500,"moves to the reply");
  for(int i = 0; i < 12; i++){ // 3 s: the reply's voice stops after 0.5 s; both still announced
    grant(DV_EVENT_UPDATE,852362500,10003,3421116,0);
    grant(DV_EVENT_UPDATE,851912500,10003,9989123,0);
    run(12,i < 2);
  }
  check(following() == 0 && Created == created_before + 2,"doesn't go back to the transmission it left for the reply");
  grant(DV_EVENT_GRANT,852362500,10003,3421116,62);
  run(2,true);
  check(following() == 852362500,"a new grant of that channel is followed");
  dv_follow_destroy(F);

  // The control channel goes: search the site's other control channels.
  // Listed in the config: 851.2875 and 853.0375 (and 860.0, outside the
  // front end's coverage); the site broadcasts 852.3625 as a secondary
  Frontend.frequency = 852000000;
  Frontend.min_IF = -2000000;
  Frontend.max_IF = 2000000;
  Chan.tune.freq = 851462500;
  Chan.p25.ncontrol = 3;
  Chan.p25.control[0] = 851287500;
  Chan.p25.control[1] = 860000000;
  Chan.p25.control[2] = 853037500;
  Cc.tsbks = Cc.tsbk_errors = 0;
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  Tunes = 0;
  Cc.tsbks = 1;
  run(10,false);
  { dv_event e = { .type = DV_EVENT_CONTROL, .freq_hz = 852362500, .slot = -1 }; dv_follow_event(F,&e); }
  run(240,false); // 4.8 s since the last control message
  check(Tunes == 0 && Chan.tune.freq == 851462500,"a control channel silent under 5 s is left alone");
  run(20,false);
  check(Tunes == 1 && Chan.tune.freq == 852362500,"silent for 5 s: tries the next known, the secondary the site broadcast");
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"CC search") == 0,"and says \"CC search\"");
  run(101,false);
  check(Tunes == 2 && Chan.tune.freq == 851287500,"nothing there in 2 s: the first listed");
  run(101,false);
  check(Tunes == 3 && Chan.tune.freq == 853037500,"then the next listed that the front end covers");
  run(101,false);
  check(Tunes == 4 && Chan.tune.freq == 851462500,"then back where it was first heard");
  run(101,false);
  check(Tunes == 5 && Chan.tune.freq == 852362500,"and round again");
  run(30,false);
  Cc.tsbks = 2; // control messages here
  run(1,false);
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"CC N469") == 0,"a control message ends the search");
  run(200,false);
  check(Tunes == 5 && Chan.tune.freq == 852362500,"and it stays there, through 4 s of silence too");
  // Calls go on being followed during a search
  run(60,false);
  check(Tunes == 6,"searching again after 5 s of silence");
  grant(DV_EVENT_GRANT,851912500,10003,3451001,90);
  run(2,false);
  check(following() == 851912500,"a grant is still followed while searching");
  dv_follow_destroy(F);

  // Nowhere else to look: stay put
  Chan.tune.freq = 851462500;
  Chan.p25.ncontrol = 0;
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  Tunes = 0;
  run(1000,false);
  check(Tunes == 0 && dv_follow_source(F,0,text,sizeof text) && strcmp(text,"no CC") == 0,"with no other control channel known it stays put: \"no CC\"");
  dv_follow_destroy(F);

  // An NXDN Type-C control channel: the same follower, an NXDN voice decoder
  Chan.demod_type = NXDN_DEMOD;
  Chan.tune.freq = 153492500;
  Cc.wacn = 0;
  Cc.last_nac = 31; // the RAN
  Cc.tsbks = 100;
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  run(2,false);
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"CC R31") == 0,"NXDN: idle shows the RAN, \"CC R31\"");
  { dv_event e = { .type = DV_EVENT_GRANT, .freq_hz = 152877500, .slot = -1, .group = 59, .source = 1234, .transmission = 1 }; dv_follow_event(F,&e); }
  run(2,false);
  check(following() == 152877500,"NXDN: follows a grant without waiting for a WACN");
  // A site that assigns channel numbers: the channel plan gives the frequency
  Chan.nxdn.nchannels = 1;
  Chan.nxdn.channels[0].number = 11;
  Chan.nxdn.channels[0].freq = 464712500;
  dv_follow_destroy(F);
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  { dv_event e = { .type = DV_EVENT_GRANT, .channel = 8, .slot = -1, .group = 60, .source = 1234, .transmission = 2 }; dv_follow_event(F,&e); }
  run(2,false);
  check(following() == 0,"NXDN: a channel number that isn't in the plan isn't followed");
  { dv_event e = { .type = DV_EVENT_GRANT, .channel = 11, .slot = -1, .group = 61, .source = 1234, .transmission = 3 }; dv_follow_event(F,&e); }
  run(2,false);
  check(following() == 464712500,"NXDN: a channel number in the plan is followed to its frequency");
  dv_follow_destroy(F);

  // An LTR home repeater: its words say which repeater each call is on
  Chan.demod_type = LTR_DEMOD;
  Chan.tune.freq = 463950000;
  Chan.nxdn.nchannels = 2;
  Chan.nxdn.channels[0].number = 1;
  Chan.nxdn.channels[0].freq = 463950000;
  Chan.nxdn.channels[1].number = 5;
  Chan.nxdn.channels[1].freq = 461825000;
  Cc.last_nac = 1; // the repeater's number
  Cc.tsbks = 200;
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  run(2,false);
  run(400,false); // an idle repeater sends a word only every ten seconds
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"LTR R1") == 0,"LTR: idle shows the repeater, \"LTR R1\"");
  { dv_event e = { .type = DV_EVENT_GRANT, .channel = 5, .slot = -1, .group = 1099, .transmission = 1 }; dv_follow_event(F,&e); }
  run(2,false);
  check(following() == 461825000,"LTR: follows a group to the repeater its call is on");
  dv_follow_destroy(F);
  Chan.nxdn.nchannels = 0;
  Chan.demod_type = P25_DEMOD;
  // --- voice-channels: several calls at once, one control channel
  static chan_t Lane2, Lane3;
  strcpy(Chan.name,"p25 851462");
  strcpy(Lane2.name,"p25 851462-2");
  strcpy(Lane3.name,"p25 851462-3");
  chan_t *const lanes[] = { &Lane2, &Lane3 };
  Chan.tune.freq = 851462500;
  Chan.p25.ncontrol = 0;
  Cc.wacn = 0x458AD;
  Cc.last_nac = 0x469;
  Cc.tsbks = 300;
  F = dv_follow_create(&Chan,3,lanes,NULL,NULL);
  check(F != NULL && dv_follow_lanes(F) == 3,"three lanes");
  run(2,false);
  check(dv_follow_source(F,0,text,sizeof text) && strcmp(text,"CC N469") == 0
	&& !dv_follow_source(F,1,text,sizeof text),"idle: the first lane shows the control channel, the others nothing");
  grant(DV_EVENT_GRANT,851287500,101,1001,1);
  grant(DV_EVENT_GRANT,851912500,102,1002,2);
  run(1,false);
  check(lane_of(101) >= 0 && lane_of(102) >= 0 && free_lanes() == 1,"two calls: two lanes, the third free");
  int const l102 = lane_of(102);
  check(l102 >= 0 && dv_follow_source(F,l102,text,sizeof text) && strcmp(text,"TG102 851.9125") == 0,"a lane shows its call");
  {
    char dest[24],src[24];
    double freq; int slot; uint64_t gen,serial;
    check(l102 >= 0 && dv_follow_call(F,l102,dest,sizeof dest,src,sizeof src,&freq,&slot,&gen,&serial)
	  && strcmp(dest,"102") == 0 && strcmp(src,"1002") == 0 && freq == 851912500,"and its call for recorders");
  }
  int const l101 = lane_of(101);
  grant(DV_EVENT_GRANT,852362500,103,1003,3);
  grant(DV_EVENT_GRANT,852812500,104,1004,4);
  run(1,false);
  int const l103 = lane_of(103);
  check(free_lanes() == 0 && lane_of(101) == l101 && lane_of(102) == l102 && (l103 >= 0 || lane_of(104) >= 0),
	"two more calls: the free lane takes one, the others keep theirs");
  int const waiting = l103 >= 0 ? 104 : 103;
  check(lane_of(waiting) < 0,"the other waits: no list, no priority, nothing preempted");
  // A reply of TG 102 elsewhere: its lane moves; no other lane takes it
  run(10,true);
  grant(DV_EVENT_GRANT,853037500,102,1005,5);
  run(1,false);
  check(lane_of(102) == l102 && F->lanes[l102].cur.freq == 853037500 && lane_of(101) == l101,
	"a talkgroup's reply goes to the lane following it");
  // A lane's call ends: the waiting call takes it
  F->lanes[l101].voice->st.call_serial++;
  for(int i = 0; i < 10; i++){ // the waiting call still announced
    grant(DV_EVENT_UPDATE,waiting == 103 ? 852362500 : 852812500,waiting,1000 + waiting % 100,0);
    run(1,true);
  }
  check(lane_of(waiting) == l101,"a lane freed takes the call waiting");
  dv_follow_destroy(F);

  // Phase 2: two talkgroups on one frequency, a slot each
  F = dv_follow_create(&Chan,3,lanes,NULL,NULL);
  { dv_event e = { .type = DV_EVENT_GRANT, .freq_hz = 851912500, .slot = 0, .group = 201, .source = 2001, .transmission = 1 }; dv_follow_event(F,&e); }
  { dv_event e = { .type = DV_EVENT_GRANT, .freq_hz = 851912500, .slot = 1, .group = 202, .source = 2002, .transmission = 2 }; dv_follow_event(F,&e); }
  run(1,false);
  {
    int const a = lane_of(201), b = lane_of(202);
    check(a >= 0 && b >= 0 && F->lanes[a].cur.slot == 0 && F->lanes[b].cur.slot == 1
	  && F->lanes[a].voice->config.p25_slot == 0 && F->lanes[b].voice->config.p25_slot == 1,
	  "Phase 2: both slots of a channel, a lane each");
  }
  dv_follow_destroy(F);

  // Priority: a talkgroup list, two lanes. A better call takes the lane of the worst
  Chan.p25.ntalkgroups = 4;
  Chan.p25.talkgroups[0] = 300;
  Chan.p25.talkgroups[1] = 301;
  Chan.p25.talkgroups[2] = 302;
  Chan.p25.talkgroups[3] = 303;
  F = dv_follow_create(&Chan,2,lanes,NULL,NULL);
  grant(DV_EVENT_GRANT,851287500,302,3002,1);
  grant(DV_EVENT_GRANT,851912500,303,3003,2);
  run(1,false);
  run(10,true);
  check(lane_tg(0) == 302 && lane_tg(1) == 303,"two lanes, TGs 302 and 303");
  grant(DV_EVENT_GRANT,852362500,999,9999,3); // not in the list
  run(1,true);
  check(lane_tg(0) == 302 && lane_tg(1) == 303,"a talkgroup not in the list takes nothing");
  grant(DV_EVENT_GRANT,852812500,300,3000,4);
  run(1,true);
  check(lane_tg(0) == 302 && lane_tg(1) == 300,"the first in the list preempts the last (303), not 302");
  grant(DV_EVENT_GRANT,853037500,301,3001,5);
  run(1,true);
  check(lane_tg(0) == 301 && lane_tg(1) == 300,"then 301 preempts 302, now the worst");
  grant(DV_EVENT_UPDATE,851287500,302,3002,0);
  run(1,true);
  check(lane_tg(0) == 301 && lane_tg(1) == 300,"and 302, worse than both, waits");
  // 300's call ends: 302 (still announced) is back
  F->lanes[1].voice->st.call_serial++;
  for(int i = 0; i < 5; i++){
    grant(DV_EVENT_UPDATE,851287500,302,3002,0);
    run(1,true);
  }
  check(lane_tg(1) == 302 && lane_tg(0) == 301,"a call preempted is followed again when a lane frees");
  dv_follow_destroy(F);

  // Codex: a reply mustn't take a channel and slot another lane is still on
  Chan.p25.ntalkgroups = 0;
  F = dv_follow_create(&Chan,2,lanes,NULL,NULL);
  grant(DV_EVENT_GRANT,851287500,501,5001,1);
  run(1,false);
  grant(DV_EVENT_GRANT,851912500,502,5002,2);
  run(1,false);
  run(10,true);
  {
    int const a = lane_of(501), b = lane_of(502);
    grant(DV_EVENT_GRANT,851912500,501,5003,3); // 501's reply, on the bearer 502's lane is on
    run(1,true);
    check(a >= 0 && b >= 0 && F->lanes[a].cur.freq == 851287500 && F->lanes[b].cur.freq == 851912500 && lane_of(502) == b,
	  "a reply on a bearer another lane is on: not taken while that lane is there");
  }
  dv_follow_destroy(F);

  // Codex: a better call waiting doesn't hold up a reply when a free lane can take it
  Chan.p25.ntalkgroups = 3;
  Chan.p25.talkgroups[0] = 601;
  Chan.p25.talkgroups[1] = 602;
  Chan.p25.talkgroups[2] = 603;
  F = dv_follow_create(&Chan,2,lanes,NULL,NULL);
  grant(DV_EVENT_GRANT,851287500,602,6002,1);
  run(1,false);
  run(10,true);
  {
    int const a = lane_of(602);
    grant(DV_EVENT_GRANT,851912500,602,6003,2); // 602's reply
    grant(DV_EVENT_GRANT,852362500,601,6001,3); // a better call
    run(1,true);
    check(a >= 0 && lane_of(602) == a && F->lanes[a].cur.freq == 851912500 && lane_of(601) >= 0,
	  "a reply and a better call in one block: the reply followed, the free lane takes the better call");
  }
  dv_follow_destroy(F);

  // Codex: one lane, a re-grant pending at the end of the transmission: it
  // lingers, and isn't preempted in that block (as before lanes)
  Chan.p25.talkgroups[0] = 300;
  Chan.p25.talkgroups[1] = 303;
  Chan.p25.ntalkgroups = 2;
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  grant(DV_EVENT_GRANT,851287500,303,3003,1);
  run(1,false);
  run(10,true);
  grant(DV_EVENT_UPDATE,851287500,303,3003,2); // its bearer granted again: pending
  run(2,true);
  grant(DV_EVENT_GRANT,851912500,300,3000,3);  // a better call, as the transmission ends
  Voice_stats.call_serial++;
  run(1,false);
  check(lane_tg(0) == 303 && F->lanes[0].lingering,"one lane: the end with a re-grant pending lingers, not preempted in that block");
  run(1,false);
  check(lane_tg(0) == 300,"and the better call takes it in the next");
  dv_follow_destroy(F);

  // One lane, the list: the old rule, a better call preempts
  Chan.p25.ntalkgroups = 4;
  Chan.p25.talkgroups[0] = 300;
  Chan.p25.talkgroups[1] = 301;
  Chan.p25.talkgroups[2] = 302;
  Chan.p25.talkgroups[3] = 303;
  F = dv_follow_create(&Chan,1,NULL,NULL,NULL);
  grant(DV_EVENT_GRANT,851287500,303,3003,1);
  run(1,false);
  run(5,true);
  grant(DV_EVENT_GRANT,851912500,300,3000,2);
  run(1,true);
  check(lane_tg(0) == 300,"one lane: a better call preempts, as before");
  dv_follow_destroy(F);
  Chan.p25.ntalkgroups = 0;

  printf(Failures ? "follower tests FAILED\n" : "Follower tests passed\n");
  return Failures ? 1 : 0;
}
