// monitor's receiver summary: each front end's level, gain and A/D overloads,
// with the last ten minutes' peak and overload count held, and the channels
// with the most audio over that time. No curses here: the dashboard draws
// what this keeps. Thread safe: the status thread gives it readings, the
// data threads audio, the display thread asks for views
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef _MONITOR_SUMMARY_H
#define _MONITOR_SUMMARY_H 1
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SUM_WINDOW_S 600          // what "recent" means: ten minutes
#define SUM_BUCKET_S 10           // held in buckets of ten seconds
#define SUM_BUCKETS (SUM_WINDOW_S / SUM_BUCKET_S)
#define SUM_FRONTENDS 4
#define SUM_CHANNELS 1000         // (monitor's NSESSIONS)

// A front end, as one of its channels' status gave it
struct sum_reading {
  char key[64];                   // which receiver: its radiod's status group, or its address
  char description[128];          // what it calls itself (a label: two can share one)
  int64_t clocktime;              // radiod's time of that status, ns
  double frequency;               // its LO, Hz
  double samprate;
  float if_dbfs;                  // IF level
  float rf_gain;                  // dB
  bool agc;
  int lna_gain,mixer_gain,if_gain; // tuners that give them (0 if not)
  uint64_t overranges;            // A/D overloads, since radiod started
  uint64_t samp_since_over;       // samples since the last
};

struct sum_frontend {
  bool used;
  struct sum_reading now;         // the latest
  int64_t first;                  // the first reading's time
  bool have_prev;
  int64_t last_over;              // when the last overload was (radiod's time), 0 if none known
  struct {
    int64_t id;                   // which ten seconds (time / bucket), 0 unused
    float peak;                   // the highest IF level in it
    int64_t peak_time;
    uint64_t overs;               // overloads in it (increases seen; a reset counted afresh)
  } bucket[SUM_BUCKETS];
};

struct sum_channel {
  uint32_t ssrc;                  // whose figures these are (a slot reused by another starts again)
  struct {
    int64_t id;                   // which ten seconds, 0 unused
    float seconds;                // of audio in them
  } bucket[SUM_BUCKETS];
};

struct rx_summary {
  pthread_mutex_t lock;           // (PTHREAD_MUTEX_INITIALIZER)
  struct sum_frontend fe[SUM_FRONTENDS];
  struct sum_channel chan[SUM_CHANNELS];
};

// A front end's status, from any of its channels: the newest counts
void summary_reading(struct rx_summary *s,struct sum_reading const *r);
// Audio a channel played (in its slot), at now
void summary_audio(struct rx_summary *s,int slot,uint32_t ssrc,double seconds,int64_t now);

// What the dashboard shows of front end k, as of now (monitor's time): false if none
struct sum_view {
  struct sum_reading now;
  float peak;                     // the last ten minutes' highest IF level, and when (-INFINITY: none)
  int64_t peak_time;
  uint64_t recent_overs;          // overloads in that time (or since monitor started, if less)
  bool whole_window;              // monitor has seen all of that time
  int64_t last_over;              // when the last overload was, 0 if none known
  bool recent_over;               // within the last ten minutes
};
bool summary_view(struct rx_summary *s,int k,int64_t now,struct sum_view *v);
// The channels with the most audio in the last ten minutes, most first:
// their slots and seconds, up to n; returns how many (those with any to speak of)
int summary_busiest(struct rx_summary *s,int *slots,double *seconds,int n,int64_t now);
// "3m", "45s", "2h": a duration (ns) in its largest unit
void summary_ago(int64_t ns,char *out,size_t size);
// The front end line: "fobos 445.6000 MHz | IF -31.2 dBFS (peak -18.4, 3m ago) | gain 22 dB | A/D over 0 | 4s ago"
void summary_frontend_line(struct sum_view const *v,int64_t now,char *out,size_t size);

#endif
