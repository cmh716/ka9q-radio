// Exercise DV_AUTO's real collection/playout path with deterministic decoder
// transitions within one IQ block. DSP is real; frame decoding is scripted.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../dv_auto.cpp"
#include <cstdlib>

namespace {
class scripted_decoder : public dv_decoder {
public:
  scripted_decoder(bool emit,dv_pcm_callback cb,void *user) : emit_(emit),cb_(cb),user_(user) {}
  void process(cfloat const *,size_t) override { run(); }
  void process_leveled(cfloat const *,size_t) override { run(); }
  void signal_lost() override { st_.call_serial++; }
  void get_stats(dv_stats *st) const override { *st = st_; }
private:
  void run() {
    if(!emit_)
      return;
    int16_t pcm[DV_PCM_SAMPLES]{};
    for(int i=0;i<3;i++) {
      std::snprintf(st_.dst_call,sizeof st_.dst_call,"%d",i == 0 ? 100 : 200);
      std::snprintf(st_.src_call,sizeof st_.src_call,"%d",i == 0 ? 1 : 2);
      st_.call_serial = i == 2 ? 1 : 0;
      st_.voice_decoded++;
      st_.frames++;
      pcm[0] = i;
      cb_(user_,pcm);
    }
    // A terminator later in the very same process call.
    st_.src_call[0] = st_.dst_call[0] = '\0';
    st_.call_serial = 2;
  }
  bool emit_;
  dv_pcm_callback cb_;
  void *user_;
  dv_stats st_{};
};

dv_decoder *playing;
int received;
bool throw_from_callback;
void check(bool ok) {
  if(!ok) {
    std::fprintf(stderr,"FAIL: DV_AUTO lost a frame's call identity\n");
    std::exit(EXIT_FAILURE);
  }
}
void receive(void *,int16_t const pcm[DV_PCM_SAMPLES]) {
  dv_stats st;
  playing->get_stats(&st);
  check(pcm[0] == received && st.active_mode == DV_P25_C4FM);
  check(std::strcmp(st.dst_call,received == 0 ? "100" : "200") == 0);
  check(st.call_serial == (received == 2 ? 1u : 0u));
  received++;
  if(throw_from_callback)
    throw 42;
}
}

// Only P25 emits frames; the other factories provide silent competitors.
dv_decoder *dv_create_p25(bool,uint16_t,bool,dv_pcm_callback cb,void *u) { return new scripted_decoder(true,cb,u); }
dv_decoder *dv_create_dmr(int,dv_pcm_callback cb,void *u) { return new scripted_decoder(false,cb,u); }
dv_decoder *dv_create_ysf(dv_pcm_callback cb,void *u) { return new scripted_decoder(false,cb,u); }
dv_decoder *dv_create_dstar(dv_pcm_callback cb,void *u) { return new scripted_decoder(false,cb,u); }
dv_decoder *dv_create_nxdn(bool,dv_pcm_callback cb,void *u) { return new scripted_decoder(false,cb,u); }
dv_decoder *dv_create_m17(int,dv_pcm_callback cb,void *u) { return new scripted_decoder(false,cb,u); }

int main() {
  dv_config config{};
  std::unique_ptr<dv_decoder> decoder(dv_create_auto(config,receive,nullptr));
  playing = decoder.get();
  cfloat iq[960]{};
  decoder->process(iq,960);
  check(received == 3);
  dv_stats st;
  decoder->get_stats(&st);
  check(st.call_serial == 2 && st.dst_call[0] == '\0');
  // An exception must not leave get_stats pointing at a queued frame.
  decoder.reset(dv_create_auto(config,receive,nullptr));
  playing = decoder.get();
  received = 0;
  throw_from_callback = true;
  bool caught = false;
  try { decoder->process(iq,960); }
  catch(int) { caught = true; }
  decoder->get_stats(&st);
  check(caught && st.call_serial == 2 && st.dst_call[0] == '\0');
  std::puts("Automatic mode call snapshot tests passed");
}
