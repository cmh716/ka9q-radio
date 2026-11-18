// Synthetic mixed-mode IQ through the production FM chain and monitor detectors.
// Build with function sections and --gc-sections to discard device I/O paths.
#include "dv.c"
#include "monitor-data.c"

double Blocktime = 0.02;
float const Tone_period = 0.24f;

static void check_output_notch(void){
  // Exercise the shared filter at ordinary FM and mixed FM audio rates.
  int const rates[] = {8000,24000,48000};
  for(unsigned r = 0; r < sizeof rates/sizeof rates[0]; r++){
    int const rate = rates[r];
    for(int tone = 0; tone < N_tones; tone++){
      struct audio_notch notch = {0};
      double complex pl = 0, voice = 0;
      for(int i = 0; i < 3*rate; i++){
        double t = (double)i/rate;
        float x = 0.1f*sinf(2*M_PI*PL_tones[tone]*t) + 0.2f*sinf(2*M_PI*1000*t);
        if(i == 0)
          assert(apply_audio_notch(&notch,x,0,rate) == x);
        float y = apply_audio_notch(&notch,x,PL_tones[tone],rate);
        assert(isfinite(y));
        if(i >= 2*rate){
          pl += y*cexp(-I*2*M_PI*PL_tones[tone]*t);
          voice += y*cexp(-I*2*M_PI*1000*t);
        }
      }
      double const pl_gain = 2*cabs(pl)/rate/0.1;
      double const voice_gain = 2*cabs(voice)/rate/0.2;
      if(pl_gain > 0.04 || voice_gain < 0.95 || voice_gain > 1.05){
        fprintf(stderr,"notch %.1f Hz at %d: tone gain %.4f, voice gain %.4f\n",
                PL_tones[tone],rate,pl_gain,voice_gain);
        exit(1);
      }
      assert(apply_audio_notch(&notch,0.25f,0,rate) == 0.25f);
    }
  }
  // Verify that the actual mixed FM path applies the optional notch.
  struct dv_fm fm;
  dv_fm_init(&fm);
  chan_t chan = {0};
  chan.fm.rate = 1;
  chan.fm.notch_freq = 123;
  double phase = 0;
  double complex pl = 0, voice = 0;
  int samples = 0;
  for(int block = 0; block < 150; block++){
    float complex iq[960];
    for(int i = 0; i < 960; i++){
      double t = (block*960+i)/48000.0;
      phase += 2*M_PI*(500*sin(2*M_PI*123*t)+1000*sin(2*M_PI*1000*t))/48000;
      iq[i] = cexpf(I*phase);
    }
    dv_fm_block(&fm,&chan,iq,960);
    float pcm[DV_PCM_SAMPLES];
    bool open;
    while(dv_fm_take(&fm,pcm,&open)){
      for(int i = 0; i < DV_PCM_SAMPLES; i++,samples++){
        if(samples < 16000) continue;
        double t = samples/8000.0;
        pl += pcm[i]*cexp(-I*2*M_PI*123*t);
        voice += pcm[i]*cexp(-I*2*M_PI*1000*t);
      }
    }
  }
  assert(cabs(voice) > 1 && cabs(pl)/cabs(voice) < 0.02);
  puts("Optional notch: 55 tones at 8/24/48 kHz, voice preservation, bypass and mixed FM passed");
}

int main(void){
  check_output_notch();
  static sess_t sp;
  static chan_t chan;
  chan.fm.rate = 1;
  chan.sig.bb_power = 100;
  chan.sig.n0 = 0.001;
  chan.filter.min_IF = -8000;
  chan.filter.max_IF = 8000;
  sp.samprate = DV_AUDIO_SAMPRATE;
  sp.channels = 1;
  // Reproduce the status received by monitor: no dv.analog flag is sent.
  struct frontend frontend = {0};
  uint8_t status[128], *bp = status;
  encode_byte(&bp,DEMOD_TYPE,DIGITAL_DEMOD);
  encode_string(&bp,CALL_MODE,"FM",2);
  encode_string(&bp,SOURCE_CALLSIGN,"FM",2);
  encode_eol(&bp);
  assert(decode_radio_status(&frontend,&sp.chan,status,bp-status) >= 0);
  assert(!sp.chan.dv.analog);
  assert(session_has_analog_audio(&sp));
  // Detection must work with the default setting: notching disabled.
  assert(!sp.notch_enable);
  for(int tone = 0; tone < N_tones; tone++){
    struct dv_fm fm;
    dv_fm_init(&fm);
    init_pl(&sp);
    double phase = 0;
    for(int block = 0; block < 80; block++){
      float complex iq[960];
      for(int i = 0; i < 960; i++){
        double t = (block*960+i)/48000.0;
        double hz = 500*sin(2*M_PI*PL_tones[tone]*t) + 1200*sin(2*M_PI*1000*t);
        phase += 2*M_PI*hz/48000;
        iq[i] = cexpf(I*phase);
      }
      dv_fm_block(&fm,&chan,iq,960);
      float pcm[DV_PCM_SAMPLES];
      bool open;
      while(dv_fm_take(&fm,pcm,&open)){
        sp.bounce = pcm;
        sp.frame_size = DV_PCM_SAMPLES;
        run_pl(&sp);
      }
    }
    if(sp.current_tone != PL_tones[tone]){
      fprintf(stderr,"tone %.1f detected as %.1f\n",PL_tones[tone],sp.current_tone);
      return 1;
    }
  }
  init_pl(&sp);
  float silence[DV_PCM_SAMPLES] = {0};
  sp.bounce = silence;
  for(int block = 0; block < 30; block++)
    run_pl(&sp);
  assert(sp.current_tone == 0 && sp.current_dcs == 0);
  strcpy(sp.chan.dv.call_mode,"DMR");
  strcpy(sp.chan.dv.source,"FM"); // stale source must not override a digital call
  assert(!session_has_analog_audio(&sp));
  init_pl(&sp);
  assert(sp.current_tone == 0 && sp.current_dcs == 0 && sp.tone_samples == 0);
  sp.chan.dv.call_mode[0] = '\0';
  assert(session_has_analog_audio(&sp));
  sp.chan.dv.source[0] = '\0';
  assert(!session_has_analog_audio(&sp));
  sp.chan.demod_type = FM_DEMOD;
  assert(session_has_analog_audio(&sp));
  puts("Mixed FM: all 55 CTCSS tones detected; wire status, digital exclusion and reset passed");
  return 0;
}
