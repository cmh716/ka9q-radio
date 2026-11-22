// Sequential tone paging (fire/EMS two-tone, "long tone") detection on
// audio: steady pure tones found with an FFT, joined into pages
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef _TONE_DETECT_H
#define _TONE_DETECT_H 1
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define TD_SAMPRATE 8000       // input sample rate, Hz
#define TD_FFT 1024            // 128 ms
#define TD_HOP 400             // 50 ms between looks

enum td_type { TD_TWO_TONE, TD_LONG_TONE };

struct td_event {
  enum td_type type;
  double a_hz, b_hz;           // b_hz 0 for a long tone
  double a_seconds, b_seconds;
  uint64_t start;              // the first tone's start, in input samples
  uint64_t end;                // the last tone's end
};

// A steady tone: a run of looks at one frequency
struct td_segment {
  bool open;
  double hz_sum;               // of its looks, for the mean
  int looks;
  int missing;                 // looks since the last with it (a fade)
  uint64_t start, end;         // in samples
};

struct tone_detector {
  float history[TD_FFT];       // the last TD_FFT samples
  int fill;                    // samples since the last look
  uint64_t samples;            // in all
  float window[TD_FFT];
  void *plan;                  // fftwf_plan
  float *in;                   // fftwf buffers
  void *out;
  struct td_segment cur;       // the tone being heard, if any
  struct td_segment last;      // the tone before it, closed (a page's A), if any
  bool last_valid;
  bool last_used;              // already paired into a page
};

bool td_init(struct tone_detector *d);
void td_free(struct tone_detector *d);
// Feed n samples (TD_SAMPRATE, about +/-1 full scale); pages that completed
// go in events (up to max); returns how many
int td_process(struct tone_detector *d,float const *x,int n,struct td_event *events,int max);

// One look at a block of TD_FFT samples: whether it is one steady tone, and
// its frequency (exposed for tests)
bool td_look(struct tone_detector *d,float const *block,double *hz,double *purity);

// A department's tones, and the one a page matches (within tolerance, a
// fraction: 0.015), or -1. A page matches on both tones; a long tone on A
struct td_department {
  char name[64];
  double a_hz, b_hz;           // b_hz 0: a long tone
};
int td_match(struct td_department const *list,int n,struct td_event const *ev,double tolerance);
#endif
