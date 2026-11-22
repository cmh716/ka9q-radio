// Sequential tone paging (fire/EMS two-tone, "long tone") detection.
//
// Every TD_HOP samples, a Hann-windowed FFT of the last TD_FFT (128 ms):
// in 270-3400 Hz (above CTCSS and DCS, to the top of the voice band), is
// one peak most of the power (a pure tone; speech never is)? Its frequency
// comes from parabolic interpolation of the log magnitudes, to about 1 Hz.
// Looks at one frequency (within 1.5%) make a segment, a fade of up to
// 100 ms bridged. A segment of 0.4 s or more followed within 0.3 s by
// another at a different frequency is a two-tone page (A, B); a segment of
// 4 s or more paired with nothing is a long tone.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <complex.h>
#include <fftw3.h>
#include <math.h>
#include <string.h>
#include "tone-detect.h"

static double const Low_hz = 270, High_hz = 3400;
static double const Purity = 0.7;          // the peak's share of the band's power
static double const Min_power = 1e-7;      // mean square per sample: below it, silence
static double const Same = 0.015;          // looks this close are one tone
static int const Max_missing = 2;          // looks a tone may fade for (100 ms)
static double const Min_tone = 0.4;        // seconds, each tone of a page
static double const Max_gap = 0.3;         // seconds of silence between A's end and B's start
// Tones are timed at their looks' midpoints, a hop apart: measured that
// way, a silent gap is its true length and 0-100 ms (calibrated: a true
// 300 ms measures at most 350, a true 450 at least 450). Pairing and expiry
// both measure so, from the looks' midpoints
static double const Gap_allowance = Max_gap + 2.0 * TD_HOP / TD_SAMPRATE;
static double const Long_tone = 4.0;       // seconds, a long tone alone

bool td_init(struct tone_detector *d){
  memset(d,0,sizeof *d);
  for(int i = 0; i < TD_FFT; i++)
    d->window[i] = 0.5f - 0.5f * cosf(2 * (float)M_PI * i / TD_FFT);
  d->in = fftwf_alloc_real(TD_FFT);
  d->out = fftwf_alloc_complex(TD_FFT / 2 + 1);
  if(d->in != NULL && d->out != NULL)
    d->plan = fftwf_plan_dft_r2c_1d(TD_FFT,d->in,d->out,FFTW_ESTIMATE);
  if(d->plan == NULL){
    td_free(d); // nothing left allocated on failure
    return false;
  }
  return true;
}

void td_free(struct tone_detector *d){
  if(d->plan != NULL)
    fftwf_destroy_plan(d->plan);
  fftwf_free(d->in);
  fftwf_free(d->out);
  memset(d,0,sizeof *d);
}

bool td_look(struct tone_detector *d,float const *block,double *hz,double *purity){
  double energy = 0;
  for(int i = 0; i < TD_FFT; i++){
    d->in[i] = block[i] * d->window[i];
    energy += (double)block[i] * block[i];
  }
  *hz = 0;
  *purity = 0;
  if(energy / TD_FFT < Min_power)
    return false;
  fftwf_execute(d->plan);
  fftwf_complex const *out = d->out;
  double const bin_hz = (double)TD_SAMPRATE / TD_FFT;
  int const lo = (int)ceil(Low_hz / bin_hz), hi = (int)floor(High_hz / bin_hz);
  double band = 0, best = 0;
  int peak = lo;
  for(int k = lo; k <= hi; k++){
    double const p = crealf(out[k]) * crealf(out[k]) + cimagf(out[k]) * cimagf(out[k]);
    band += p;
    if(p > best){
      best = p;
      peak = k;
    }
  }
  if(band <= 0 || peak <= lo || peak >= hi)
    return false; // nothing, or at the band's edge
  // A Hann window spreads a tone over its peak bin and the two beside it
  double p[3];
  for(int j = -1; j <= 1; j++){
    int const k = peak + j;
    p[j + 1] = crealf(out[k]) * crealf(out[k]) + cimagf(out[k]) * cimagf(out[k]);
  }
  *purity = (p[0] + p[1] + p[2]) / band;
  // Parabolic interpolation of the log magnitudes
  double const a = log(p[0] + 1e-30), b = log(p[1] + 1e-30), c = log(p[2] + 1e-30);
  double const denom = a - 2 * b + c;
  double const delta = denom != 0 ? 0.5 * (a - c) / denom : 0;
  *hz = (peak + delta) * bin_hz;
  return *purity >= Purity;
}

static double seconds(uint64_t samples){
  return (double)samples / TD_SAMPRATE;
}

static double mean_hz(struct td_segment const *s){
  return s->looks > 0 ? s->hz_sum / s->looks : 0;
}

// The tone being heard has ended: a page's B (with the tone before it as
// A), a long tone, or a candidate A for the next
static int close_segment(struct tone_detector *d,struct td_event *events,int max){
  int n = 0;
  struct td_segment const s = d->cur;
  d->cur.open = false;
  double const dur = seconds(s.end - s.start);
  if(dur < Min_tone)
    return 0; // a blip: not a tone of a page (the tone before it stays a candidate A)
  if(d->last_valid && !d->last_used){
    double const a_dur = seconds(d->last.end - d->last.start);
    double const gap = s.start > d->last.end ? seconds(s.start - d->last.end) : 0;
    double const a = mean_hz(&d->last), b = mean_hz(&s);
    if(a_dur >= Min_tone && gap <= Gap_allowance && fabs(a - b) > Same * a){
      if(n < max)
	events[n++] = (struct td_event){.type = TD_TWO_TONE,.a_hz = a,.b_hz = b,.a_seconds = a_dur,
					 .b_seconds = dur,.start = d->last.start,.end = s.end};
      d->last_used = true;
      d->last = s;
      d->last_valid = true;
      d->last_used = true; // B isn't the next page's A
      return n;
    }
    // The tone before went unpaired: a long tone, if it was long enough
    if(a_dur >= Long_tone && n < max)
      events[n++] = (struct td_event){.type = TD_LONG_TONE,.a_hz = a,.a_seconds = a_dur,
				       .start = d->last.start,.end = d->last.end};
  }
  d->last = s;
  d->last_valid = true;
  d->last_used = false;
  return n;
}

// A candidate A that no B has followed in time: a long tone, or nothing
static int expire_last(struct tone_detector *d,struct td_event *events,int max){
  if(!d->last_valid || d->last_used || d->cur.open)
    return 0;
  // In the looks' time (their midpoints), as the gap is measured when pairing
  uint64_t const now = d->samples - TD_FFT / 2;
  if(now <= d->last.end || seconds(now - d->last.end) <= Gap_allowance)
    return 0;
  int n = 0;
  double const dur = seconds(d->last.end - d->last.start);
  if(dur >= Long_tone && n < max)
    events[n++] = (struct td_event){.type = TD_LONG_TONE,.a_hz = mean_hz(&d->last),.a_seconds = dur,
				     .start = d->last.start,.end = d->last.end};
  d->last_valid = false;
  return n;
}

int td_process(struct tone_detector *d,float const *x,int nx,struct td_event *events,int max){
  int found = 0;
  for(int i = 0; i < nx; i++){
    memmove(d->history,d->history + 1,(TD_FFT - 1) * sizeof d->history[0]);
    d->history[TD_FFT - 1] = x[i];
    d->samples++;
    if(++d->fill < TD_HOP || d->samples < TD_FFT)
      continue;
    d->fill = 0;
    double hz,purity;
    bool const tone = td_look(d,d->history,&hz,&purity);
    // The look covers the last TD_FFT samples: its middle is when it was
    uint64_t const at = d->samples - TD_FFT / 2;
    if(tone && d->cur.open && fabs(hz - mean_hz(&d->cur)) <= Same * mean_hz(&d->cur)){
      d->cur.hz_sum += hz;
      d->cur.looks++;
      d->cur.missing = 0;
      d->cur.end = at;
    } else {
      if(d->cur.open && (tone || ++d->cur.missing > Max_missing))
	found += close_segment(d,events + found,max - found); // another tone, or the fade went on
      if(tone && !d->cur.open)
	d->cur = (struct td_segment){.open = true,.hz_sum = hz,.looks = 1,.start = at,.end = at};
    }
    found += expire_last(d,events + found,max - found);
  }
  return found;
}

int td_match(struct td_department const *list,int n,struct td_event const *ev,double tolerance){
  for(int i = 0; i < n; i++){
    struct td_department const *p = &list[i];
    if(ev->type == TD_TWO_TONE && p->b_hz > 0
       && fabs(ev->a_hz - p->a_hz) <= tolerance * p->a_hz && fabs(ev->b_hz - p->b_hz) <= tolerance * p->b_hz)
      return i;
    if(ev->type == TD_LONG_TONE && p->b_hz == 0 && fabs(ev->a_hz - p->a_hz) <= tolerance * p->a_hz)
      return i;
  }
  return -1;
}
