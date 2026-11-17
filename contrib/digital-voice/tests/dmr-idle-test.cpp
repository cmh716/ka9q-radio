// Protected DMR idle acquisition, plus optional real 48 kHz cf32 regression IQ.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../dmr_idle.h"
#include "../dv.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

static void check(bool condition, char const *message) {
  if(!condition) { std::fprintf(stderr,"FAIL: %s\n",message); std::exit(1); }
}
static std::vector<float> burst(int cc = 3, int type = 9, float gain = 1.5f, float dc = 0) {
  // Construct independently encoded slot types around the actual BS data sync.
  std::vector<unsigned> dibits(144,0);
  for(unsigned i = 0; i < 144; i++) dibits[i] = (i * 7 + i / 3) & 3;
  for(unsigned i = 0; i < 24; i++) dibits[66+i] = (DMR_BS_DATA_SYNC_MAGIC >> (46-2*i)) & 3;
  bit_vector bits(20,false);
  for(unsigned i = 0; i < 4; i++) { bits[i] = (cc >> (3-i)) & 1; bits[4+i] = (type >> (3-i)) & 1; }
  CGolay2087::encode(bits);
  for(unsigned i = 0; i < 10; i++) {
    unsigned pos = i < 5 ? 61+i : 90+i-5;
    dibits[pos] = 2*bits[2*i] + bits[2*i+1];
  }
  constexpr float levels[] = {1,3,-1,-3};
  std::vector<float> samples;
  for(unsigned dibit : dibits) for(int i = 0; i < 5; i++) samples.push_back(dc + gain * levels[dibit]);
  return samples;
}
static unsigned feed(dmr_idle_detector& d,std::vector<float> const& samples,int expected = 3) {
  unsigned count = 0;
  for(float sample : samples) {
    int cc = -1;
    if(d.process(sample,cc)) { check(cc == expected,"protected color code"); count++; }
  }
  return count;
}
static void observer_tests() {
  for(float gain : {0.7f,1.0f,1.5f,2.0f}) for(float dc : {-0.75f,0.0f,0.75f}) for(int phase = 0; phase < 5; phase++) {
    dmr_idle_detector d;
    feed(d,std::vector<float>(phase,0));
    check(feed(d,burst(3,9,gain,dc)) == 0,"one burst cannot confirm idle");
    check(feed(d,burst(3,9,gain,dc)) > 0,"second protected burst confirms idle");
    d.reset();
    check(feed(d,burst(3,9,gain,dc)) == 0,"signal loss requires fresh confirmation");
  }
  dmr_idle_detector d;
  check(feed(d,burst(3,1)) == 0 && feed(d,burst(3,1)) == 0,"voice LC header is not idle");
  d.reset();
  feed(d,burst());
  feed(d,std::vector<float>(24000,0));
  check(feed(d,burst()) == 0,"stale candidate cannot confirm after a gap");
  d.reset(); feed(d,burst());
  check(feed(d,burst(4,9),4) == 0,"different color codes cannot confirm each other");
  for(float frequency : {600,1000,1200,1800,2400}) {
    d.reset();
    for(int i = 0; i < 48000; i++) {
      int cc;
      check(!d.process(5*std::sin(6.283185307179586*frequency*i/24000),cc),"analog tone cannot confirm idle");
    }
  }
  d.reset(); uint32_t random = 0x12345678;
  for(int i = 0; i < 24000*120; i++) {
    random ^= random << 13; random ^= random >> 17; random ^= random << 5;
    float sample = 10.0f * (static_cast<double>(random)/4294967296.0 - 0.5);
    int cc;
    check(!d.process(sample,cc),"120 s of noise cannot confirm idle");
  }
  std::puts("DMR protected idle acquisition tests passed");
}
static unsigned audio_frames;
static void audio(void *,int16_t const *) { audio_frames++; }
static void replay(char const *path,dv_mode mode) {
  FILE *fp = std::fopen(path,"rb"); check(fp != nullptr,"open IQ regression file");
  dv_config config{}; dv_decoder *d = dv_create(mode,&config,audio,nullptr); check(d != nullptr,"create decoder");
  float iq[1920]; size_t count; audio_frames = 0;
  while((count = std::fread(iq,2*sizeof(float),960,fp))) dv_process(d,iq,count);
  dv_stats st; dv_get_stats(d,&st);
  check(audio_frames == 0,"idle must never produce vocoder audio");
  check(st.syncs >= 20 && st.frames >= 20,"recorded idle is acquired repeatedly");
  if(mode == DV_DMR) check(st.dmr_cc == 3,"recorded idle color code 3");
  std::printf("Recorded idle mode %d: syncs=%llu frames=%llu voice=%u\n",mode,
      (unsigned long long)st.syncs,(unsigned long long)st.frames,audio_frames);
  dv_destroy(d); std::fclose(fp);
}
int main(int argc,char **argv) {
  observer_tests();
  if(argc > 1) { replay(argv[1],DV_DMR); replay(argv[1],DV_AUTO); }
}
