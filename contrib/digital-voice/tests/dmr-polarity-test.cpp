// DMR data/voice sync ambiguity: establish polarity using protected idle fields.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../dv.h"
#include "../upstream/golay2087.h"
#include "../upstream/frame_sync_magics.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
static unsigned frames;
static void audio(void *,int16_t const *) { frames++; }
static void check(bool ok,char const *what) {
  if(!ok) { std::fprintf(stderr,"FAIL: %s\n",what); std::exit(1); }
}
static std::vector<unsigned> idle(int cc) {
  std::vector<unsigned> d(144);
  for(unsigned i=0;i<144;i++) d[i]=(i*7+i/3)&3;
  for(unsigned i=0;i<24;i++) d[66+i]=(DMR_BS_DATA_SYNC_MAGIC>>(46-2*i))&3;
  bit_vector b(20,false);
  for(unsigned i=0;i<4;i++) { b[i]=(cc>>(3-i))&1; b[4+i]=(9>>(3-i))&1; }
  CGolay2087::encode(b);
  for(unsigned i=0;i<10;i++) {
    unsigned p=i<5?61+i:90+i-5;
    d[p]=2*b[2*i]+b[2*i+1];
  }
  return d;
}
int main(int argc,char **argv) {
  for(dv_mode mode : {DV_DMR,DV_AUTO}) {
    dv_config cfg{};
    dv_decoder *d=dv_create(mode,&cfg,audio,nullptr);
    check(d!=nullptr,"create DMR receiver");
    auto symbols=idle(10);
    double phase=0;
    // Both acquisition polarities, then transitions in both directions.
    for(int polarity : {-1,1,-1,1}) {
      dv_signal_lost(d);
      dv_stats before{}; dv_get_stats(d,&before);
      unsigned settled=0;
      for(int block=0;block<100;block++) {
        float iq[1920];
        constexpr float levels[]={1,3,-1,-3};
        for(int i=0;i<960;i++) {
          int index=block*960+i;
          phase+=polarity*2*3.141592653589793*648*levels[symbols[(index/10)%144]]/48000;
          iq[2*i]=0.2*std::cos(phase); iq[2*i+1]=0.2*std::sin(phase);
        }
        frames=0;dv_process(d,iq,960);
        if(block>=25) settled+=frames;
      }
      dv_stats s{}; dv_get_stats(d,&s);
      check(settled==0,"inverted data must not produce voice after acquisition");
      check(s.frames-before.frames>=20,"protected data acquired");
      if(mode==DV_DMR)
        check(s.dmr_cc==10,"stable protected color code after polarity transition");
    }
    dv_destroy(d);
  }
  if(argc>1) {
    FILE *fp=std::fopen(argv[1],"rb");check(fp!=nullptr,"open inverted idle IQ");
    dv_config cfg{};dv_decoder *d=dv_create(DV_DMR,&cfg,audio,nullptr);
    float iq[1920];size_t n;unsigned blocks=0,settled=0;
    while((n=std::fread(iq,2*sizeof(float),960,fp))) {
      frames=0;dv_process(d,iq,n);if(blocks++>=25)settled+=frames;
    }
    dv_stats s{};dv_get_stats(d,&s);
    check(s.dmr_cc==10 && s.frames>=50,"captured inverted idle validates color code 10");
    check(settled==0,"captured inverted idle produces no false voice");
    dv_destroy(d);std::fclose(fp);
  }
  std::puts("DMR polarity acquisition and transition tests passed");
}
