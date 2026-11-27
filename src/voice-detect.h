// voice-detect.h: is an untoned FM transmission someone talking, an empty
// carrier, or noise? From the audio alone, 8 kHz:
//
//  - carrier: an unmodulated carrier demodulates to near silence: never
//    loud, above the sub-audible band
//  - voice: speech's voiced sounds have harmonics and formants: a peaky
//    spectrum (spectral flatness low) in many of its loud frames, and its
//    level rises and falls with the syllables
//  - noise: hiss, steady or pulsing, is spectrally flat; a steady whistle or
//    data tone is peaky but its level doesn't rise and fall like speech
//
// Measured over 32 ms frames, 300-3400 Hz (sub-audible tones and DCS left
// out): each frame's level, and its spectral flatness (the power
// spectrum's geometric mean over its arithmetic mean: near 1 for white
// noise, near 0 for harmonics).
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef VOICE_DETECT_H
#define VOICE_DETECT_H
#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VD_N 256               // a frame: 32 ms at 8 kHz
#define VD_MAX_FRAMES 940      // its first 30 s

struct voice_detect {
  float fs;
  float buf[VD_N];
  int fill;
  long frames;               // frames seen
  long silent;               // of them, digital silence (left out: a recording's gaps)
  int n;                     // frames kept
  float level[VD_MAX_FRAMES];   // dB, 300-3400 Hz
  float flat[VD_MAX_FRAMES];    // spectral flatness
};

static inline void vd_init(struct voice_detect *v,float fs){
  memset(v,0,sizeof *v);
  v->fs = fs;
}

// One frame: a Hann-windowed FFT (radix 2, in place), the band's level and flatness
static inline void vd_frame(struct voice_detect *v){
  static float complex x[VD_N];
  for(int i = 0; i < VD_N; i++)
    x[i] = v->buf[i] * (0.5f - 0.5f * cosf(2 * (float)M_PI * i / (VD_N - 1)));
  for(int i = 1,j = 0; i < VD_N; i++){ // bit reversal
    int bit = VD_N >> 1;
    for(; j & bit; bit >>= 1)
      j ^= bit;
    j ^= bit;
    if(i < j){
      float complex const t = x[i];
      x[i] = x[j];
      x[j] = t;
    }
  }
  for(int len = 2; len <= VD_N; len <<= 1){
    float complex const w = cexpf(-2 * (float)M_PI * I / len);
    for(int i = 0; i < VD_N; i += len){
      float complex wk = 1;
      for(int k = 0; k < len / 2; k++){
	float complex const a = x[i + k],b = x[i + k + len / 2] * wk;
	x[i + k] = a + b;
	x[i + k + len / 2] = a - b;
	wk *= w;
      }
    }
  }
  int const lo = (int)(300.0f * VD_N / v->fs),hi = (int)(3400.0f * VD_N / v->fs);
  double sum = 0,logsum = 0;
  for(int k = lo; k <= hi; k++){
    double const p = (double)crealf(x[k] * conjf(x[k])) + 1e-15;
    sum += p;
    logsum += log(p);
  }
  int const bins = hi - lo + 1;
  v->frames++;
  // (the band's mean square: by Parseval, both halves of the spectrum, over
  // N times the Hann window's energy, 3N/8)
  double const level = 10 * log10(2 * sum / ((double)VD_N * 0.375 * VD_N) + 1e-15);
  if(level < -100){
    v->silent++;
    return; // (digital silence: a recording's gap, not the transmission)
  }
  if(v->n >= VD_MAX_FRAMES)
    return;
  v->level[v->n] = (float)level;
  v->flat[v->n] = (float)(exp(logsum / bins) / (sum / bins));
  v->n++;
}

static inline void vd_sample(struct voice_detect *v,float x){
  v->buf[v->fill++] = x;
  if(v->fill == VD_N){
    vd_frame(v);
    v->fill = 0;
  }
}

static int vd_cmp(void const *a,void const *b){
  float const x = *(float const *)a,y = *(float const *)b;
  return x < y ? -1 : x > y;
}

// The measures, for testing and tuning: the loud frames' level (80th
// percentile, dB), the spread of frame levels (dB, 80th less 20th
// percentile), and the share of loud frames (within 20 dB of that level)
// that are peaky (flatness under VD_PEAKY)
struct vd_measures { double loud,spread,peaky; };

#define VD_PEAKY 0.2           // flatness under this: harmonics (voiced speech, a whistle)

static inline struct vd_measures vd_measure(struct voice_detect const *v){
  struct vd_measures m = {-120,0,0};
  if(v->n < 8)
    return m;
  float sorted[VD_MAX_FRAMES];
  memcpy(sorted,v->level,(size_t)v->n * sizeof sorted[0]);
  qsort(sorted,(size_t)v->n,sizeof sorted[0],vd_cmp);
  m.loud = sorted[(v->n * 8) / 10];
  m.spread = sorted[(v->n * 8) / 10] - sorted[(v->n * 2) / 10];
  int loud = 0,peaky = 0;
  for(int i = 0; i < v->n; i++){
    if(v->level[i] < m.loud - 20)
      continue;
    loud++;
    if(v->flat[i] < VD_PEAKY)
      peaky++;
  }
  m.peaky = loud > 0 ? (double)peaky / loud : 0;
  return m;
}

#define VD_CARRIER_DB (-50.0)  // never louder than this (80th percentile): an unmodulated carrier
#define VD_SPREAD_DB 8.0       // speech's levels rise and fall at least this much
#define VD_VOICED 0.15         // and this share of its loud frames is peaky (real recordings: speech 0.18-0.96,
                               // noise and weak carriers 0-0.06)

// "voice", "carrier" or "noise"
static inline char const *vd_classify(struct voice_detect const *v){
  struct vd_measures const m = vd_measure(v);
  if(m.loud < VD_CARRIER_DB)
    return "carrier";
  if(m.peaky >= VD_VOICED && m.spread >= VD_SPREAD_DB)
    return "voice";
  return "noise";
}
#endif
