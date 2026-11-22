// Two-tone and long-tone paging detection (tone-detect.c), on synthesised
// audio: the user's departments' pages with noise, CTCSS and a DCS-like
// subaudible code under them, off-frequency tones, gaps, back-to-back
// pages, a long tone; and things that mustn't be pages: speech-like audio,
// noise, a lone short tone.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tone-detect.h"

static int Failures;
static void check(int ok,char const *what){
  printf("%s %s\n",ok ? "ok  " : "FAIL",what);
  Failures += !ok;
}

static struct td_department const Departments[] = {
  {"Alden",2930.0,799.0},
  {"Bowmansville",1744.0,992.0},
  {"CD-122 (Town Emergency Manager)",2930.0,1344.0},
  {"CD-136 (Village Emergency Manager)",2930.0,1465.0},
  {"Crittenden",2930.0,950.0},
  {"Lancaster Village",2930.0,1130.0},
  {"Millgrove",2930.0,871.0},
  {"Town Line",2930.0,672.0},
  {"Town Line - Siren",1513.5,1830.5},
  {"Twin District",2930.0,732.0},
  {"Bowmansville/Millgrove Group Page",1153.4,1285.8},
  {"Long tone test",1000.0,0},
};
static int const N_departments = sizeof Departments / sizeof Departments[0];

// Audio being built
static float *Audio;
static size_t Length, Room;
static unsigned Seed = 1;
static double uniform(void){ Seed = Seed * 1103515245 + 12345; return ((Seed >> 8) & 0xffffff) / (double)0x1000000; }
static double gauss(void){ double u = uniform() + 1e-12, v = uniform(); return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }
static void add(double seconds,double hz,double level){ // a tone, or silence (hz 0)
  size_t const n = (size_t)(seconds * TD_SAMPRATE);
  if(Length + n > Room){
    Room = 2 * (Length + n);
    Audio = realloc(Audio,Room * sizeof *Audio);
  }
  for(size_t i = 0; i < n; i++)
    Audio[Length + i] = hz > 0 ? (float)(level * sin(2 * M_PI * hz * (double)(Length + i) / TD_SAMPRATE)) : 0;
  Length += n;
}
// Speech-like: a voiced sound with harmonics, its pitch gliding, syllables on and off
static void speech(double seconds){
  size_t const n = (size_t)(seconds * TD_SAMPRATE);
  size_t const at = Length;
  add(seconds,0,0);
  double phase = 0;
  for(size_t i = 0; i < n; i++){
    double const t = (double)i / TD_SAMPRATE;
    double const f0 = 120 + 40 * sin(2 * M_PI * 1.3 * t) + 20 * sin(2 * M_PI * 3.1 * t);
    phase += 2 * M_PI * f0 / TD_SAMPRATE;
    double s = 0;
    for(int h = 1; h <= 20; h++){
      double const fh = h * f0;
      // two formants, moving
      double const f1 = 600 + 250 * sin(2 * M_PI * 2.0 * t), f2 = 1700 + 500 * sin(2 * M_PI * 1.7 * t);
      double const g = exp(-pow((fh - f1) / 200,2)) + 0.6 * exp(-pow((fh - f2) / 300,2)) + 0.05;
      s += g * sin(h * phase);
    }
    double const syllable = 0.5 + 0.5 * sin(2 * M_PI * 4 * t); // about 4 a second
    Audio[at + i] += (float)(0.1 * s * syllable);
  }
}
// Under it all: noise, a CTCSS tone, and a DCS-like code (134 b/s NRZ, low-passed)
static void underneath(double noise,double ctcss,double dcs){
  double lp = 0;
  for(size_t i = 0; i < Length; i++){
    double const bit = ((size_t)(i * 134.4 / TD_SAMPRATE) * 2654435761u >> 7) & 1 ? 1 : -1;
    lp += 0.05 * (bit - lp); // a gentle low-pass, as a DCS encoder's
    Audio[i] += (float)(noise * gauss() + ctcss * sin(2 * M_PI * 100.0 * i / TD_SAMPRATE) + dcs * lp);
  }
}
static int run(struct td_event *ev,int max){
  struct tone_detector d;
  if(!td_init(&d))
    return -1;
  int n = 0;
  for(size_t i = 0; i < Length; i += 160) // 20 ms blocks, as a stream's
    n += td_process(&d,Audio + i,(int)(Length - i < 160 ? Length - i : 160),ev + n,max - n);
  // a stream's quiet afterwards
  float zero[160] = {0};
  for(int k = 0; k < 50; k++)
    n += td_process(&d,zero,160,ev + n,max - n);
  td_free(&d);
  return n;
}
static void reset(void){ Length = 0; }

int main(void){
  struct td_event ev[16];
  // Every department's page, 1 s and 3 s, with noise, CTCSS and DCS under it
  int all = 1;
  for(int k = 0; k < N_departments; k++){
    if(Departments[k].b_hz == 0)
      continue;
    reset();
    add(0.5,0,0);
    add(1.0,Departments[k].a_hz,0.3);
    add(3.0,Departments[k].b_hz,0.3);
    add(1.0,0,0);
    underneath(0.01,0.05,0.05);
    int const n = run(ev,16);
    int const m = n == 1 ? td_match(Departments,N_departments,&ev[0],0.015) : -1;
    if(m != k){
      all = 0;
      printf("     %s: %d events, matched %d\n",Departments[k].name,n,m);
    }
  }
  check(all,"each department's page (A 1 s, B 3 s), with noise, CTCSS and DCS under it");

  reset();
  add(0.5,0,0); add(1.0,2930,0.3); add(3.0,799,0.3); add(1.0,0,0);
  underneath(0.01,0.05,0.05);
  int n = run(ev,16);
  check(n == 1 && ev[0].type == TD_TWO_TONE && fabs(ev[0].a_hz - 2930) < 3 && fabs(ev[0].b_hz - 799) < 2,
	"Alden's tones measured within a few Hz");
  check(n == 1 && fabs(ev[0].a_seconds - 1.0) < 0.2 && fabs(ev[0].b_seconds - 3.0) < 0.2,"and their lengths");

  reset();
  add(0.5,0,0); add(1.0,2930 * 1.008,0.3); add(3.0,799 * 0.992,0.3); add(1.0,0,0);
  underneath(0.01,0,0);
  n = run(ev,16);
  check(n == 1 && td_match(Departments,N_departments,&ev[0],0.015) == 0,"tones 0.8% off still Alden");

  reset();
  add(0.5,0,0); add(1.0,2930,0.3); add(0.1,0,0); add(3.0,799,0.3); add(1.0,0,0);
  underneath(0.01,0,0);
  n = run(ev,16);
  check(n == 1 && td_match(Departments,N_departments,&ev[0],0.015) == 0,"100 ms between A and B");

  // True gaps up to 300 ms pair, wherever they fall in the 50 ms hop; 450 ms don't
  int near = 1, far = 1;
  for(int off = 0; off < 5; off++){
    for(int g = 0; g < 2; g++){
      double const gap = g == 0 ? 0.30 : 0.45;
      reset();
      add(0.5 + off * 0.01,0,0); add(1.0,2930,0.3); add(gap,0,0); add(3.0,799,0.3); add(1.0,0,0);
      underneath(0.01,0,0);
      n = run(ev,16);
      bool const paired = n == 1 && ev[0].type == TD_TWO_TONE && td_match(Departments,N_departments,&ev[0],0.015) == 0;
      if(g == 0 && !paired){ near = 0; printf("     300 ms gap, offset %d ms: %d events\n",off * 10,n); }
      if(g == 1 && paired){ far = 0; printf("     450 ms gap, offset %d ms: paired\n",off * 10); }
    }
  }
  check(near,"a 300 ms gap between A and B, at each offset in the hop: a page");
  check(far,"a 450 ms gap: not a page");
  reset();
  for(double gap = 0.20; gap <= 0.301; gap += 0.01){
    reset();
    add(0.5,0,0); add(1.0,2930,0.3); add(gap,0,0); add(3.0,799,0.3); add(1.0,0,0);
    n = run(ev,16);
    if(!(n == 1 && ev[0].type == TD_TWO_TONE)){ near = 0; printf("     %.0f ms gap: %d events\n",gap * 1000,n); }
  }
  check(near,"gaps 200-300 ms, clean tones: each a page");

  reset();
  add(0.5,0,0); add(1.0,2930,0.3); add(3.0,799,0.3); add(0.5,0,0); add(1.0,2930,0.3); add(3.0,871,0.3); add(1.0,0,0);
  underneath(0.01,0,0);
  n = run(ev,16);
  check(n == 2 && td_match(Departments,N_departments,&ev[0],0.015) == 0 && td_match(Departments,N_departments,&ev[1],0.015) == 6,
	"two pages back to back: Alden, then Millgrove");

  reset();
  add(0.5,0,0); add(6.0,1000,0.3); add(1.0,0,0);
  underneath(0.01,0,0);
  n = run(ev,16);
  check(n == 1 && ev[0].type == TD_LONG_TONE && td_match(Departments,N_departments,&ev[0],0.015) == 11,"a long tone (6 s)");

  reset();
  add(0.5,0,0); add(1.0,2930,0.2); add(3.0,799,0.2); add(1.0,0,0);
  underneath(0.05,0.05,0.05); // noise at about -12 dB to the tones
  n = run(ev,16);
  check(n == 1 && td_match(Departments,N_departments,&ev[0],0.015) == 0,"a page in moderate noise");

  reset();
  add(20.0,0,0); speech(20.0);
  underneath(0.005,0.05,0.05);
  n = run(ev,16);
  check(n == 0,"20 s of speech-like audio: no page");

  reset();
  add(10.0,0,0);
  underneath(0.1,0,0);
  n = run(ev,16);
  check(n == 0,"10 s of noise: no page");

  reset();
  add(0.5,0,0); add(1.0,1500,0.3); add(2.0,0,0);
  underneath(0.01,0,0);
  n = run(ev,16);
  check(n == 0,"a lone 1 s tone: no page");

  reset();
  add(0.5,0,0); add(1.0,2930,0.3); speech(3.0); add(1.0,0,0);
  underneath(0.01,0,0);
  n = run(ev,16);
  check(n == 0,"tone A then speech: no page");

  free(Audio);
  puts(Failures ? "Tone detection tests FAILED" : "Tone detection tests passed");
  return Failures != 0;
}
