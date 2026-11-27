// voice-detect-test: the classes of synthetic voice, noise, carrier, whistle
// and data. Exit status 0 when all are right
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "voice-detect.h"

static unsigned Seed = 1;
static float gauss(void){ // (Box-Muller, a fixed sequence)
  Seed = Seed * 1103515245u + 12345u; float const u1 = ((Seed >> 8) + 1.0f) / 16777217.0f;
  Seed = Seed * 1103515245u + 12345u; float const u2 = (Seed >> 8) / 16777216.0f;
  return sqrtf(-2 * logf(u1)) * cosf(2 * (float)M_PI * u2);
}

static char const *classify(int kind){
  static struct voice_detect v;
  vd_init(&v,8000);
  float lp = 0,phase = 0;
  for(int i = 0; i < 8000 * 6; i++){
    float const t = i / 8000.0f;
    float x = 0;
    switch(kind){
    case 0: { // voice: harmonics of a wandering pitch, shaped by formants at 700, 1200 and 2500 Hz, in syllables with pauses
      float const f0 = 120 + 15 * sinf(2 * (float)M_PI * 0.7f * t);
      phase += 2 * (float)M_PI * f0 / 8000;
      float sum = 0;
      for(int k = 1; k * 120 < 3400; k++){
	float const f = k * f0;
	float const a = expf(-powf((f - 700) / 150,2)) + 0.6f * expf(-powf((f - 1200) / 180,2))
	  + 0.25f * expf(-powf((f - 2500) / 250,2)) + 0.01f;
	sum += a * sinf(k * phase);
      }
      float const syllable = fmodf(t,0.25f) < 0.17f && fmodf(t,1.5f) < 1.1f ? 1.0f : 0.0f;
      x = 0.05f * sum * syllable + 0.0005f * gauss();
      break; }
    case 1: lp += 0.5f * (gauss() - lp); x = 0.05f * lp; break;                            // noise
    case 2: x = 0.0001f * gauss() + 0.05f * sinf(2 * (float)M_PI * 127.3f * t); break;     // a carrier, a CTCSS tone
    case 3: x = 0.2f * sinf(2 * (float)M_PI * 1000 * t); break;                            // a whistle
    case 4: x = 0.2f * sinf(2 * (float)M_PI * (1200 + 1000 * (int)fmodf(t * 1200,2)) * t); break; // data
    case 5: lp += 0.5f * (gauss() - lp); x = 0.05f * lp * (fmodf(t,0.5f) < 0.25f); break;  // pulsing noise
    }
    vd_sample(&v,x);
  }
  struct vd_measures const m = vd_measure(&v);
  char const *c = vd_classify(&v);
  printf("  loud %6.1f dB  spread %5.1f dB  peaky %4.2f  -> %s\n",m.loud,m.spread,m.peaky,c);
  return c;
}

int main(void){
  static char const *const want[] = {"voice","noise","carrier","noise","noise","noise"};
  static char const *const names[] = {"voice","noise","carrier","whistle","data","pulsing noise"};
  int bad = 0;
  for(int k = 0; k < 6; k++){
    printf("%s:",names[k]);
    char const *got = classify(k);
    if(strcmp(got,want[k]) != 0){
      printf("FAIL %s: %s, not %s\n",names[k],got,want[k]);
      bad++;
    }
  }
  printf("voice-detect tests %s\n",bad ? "FAILED" : "passed");
  return bad ? 1 : 0;
}
