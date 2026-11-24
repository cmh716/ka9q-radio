// monitor's receiver summary (monitor-summary.c): the IF peak held for ten
// minutes then dropped, overloads counted over that time, a repeated or
// older status ignored, two front ends kept apart, and the busiest channels
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "monitor-summary.h"

static int Failures;
static void check(int ok,char const *what){
  printf("%s %s\n",ok ? "ok  " : "FAIL",what);
  Failures += !ok;
}
#define S (1000000000LL)
static struct rx_summary Sum;
static int64_t const T0 = 1795000000LL * S; // a GPS time, ns

static void reading(char const *desc,int64_t t,float level,uint64_t overs,uint64_t since){
  struct sum_reading r = { .description = desc, .clocktime = t, .frequency = 445600000, .samprate = 50e6,
			   .if_dbfs = level, .rf_gain = 22, .overranges = overs, .samp_since_over = since };
  summary_reading(&Sum,&r);
}

int main(void){
  struct sum_view v;
  check(!summary_view(&Sum,0,&v),"nothing yet: no front end");

  // A quiet minute at -31 dBFS, a peak of -12 at 0:30
  for(int t = 0; t <= 60; t += 2)
    reading("fobos",T0 + t * S,t == 30 ? -12.0f : -31.0f,0,0);
  check(summary_view(&Sum,0,&v) && fabsf(v.now->if_dbfs + 31) < 0.01f,"the latest IF level");
  check(fabsf(v.peak + 12) < 0.01f && v.peak_time == T0 + 30 * S,"the peak and when");
  check(v.recent_overs == 0 && v.last_over == 0 && !v.whole_window,"no overloads; under ten minutes seen");

  // A status repeated (every channel's carries the front end's) or older: ignored
  reading("fobos",T0 + 60 * S,-5.0f,0,0);
  reading("fobos",T0 + 50 * S,-5.0f,0,0);
  check(summary_view(&Sum,0,&v) && fabsf(v.now->if_dbfs + 31) < 0.01f && fabsf(v.peak + 12) < 0.01f,
	"a repeated or older status changes nothing");

  // Overloads: 5 at 2:00, 3 more at 4:00 (since: samples since the last)
  for(int t = 62; t <= 300; t += 2){
    uint64_t const overs = t >= 240 ? 8 : t >= 120 ? 5 : 0;
    uint64_t const since = t >= 240 ? (uint64_t)(t - 240) * 50000000ull : t >= 120 ? (uint64_t)(t - 120) * 50000000ull : 0;
    reading("fobos",T0 + t * S,-31.0f,overs,since);
  }
  check(summary_view(&Sum,0,&v) && v.recent_overs == 8,"8 overloads counted in the time seen");
  check(v.last_over == T0 + 240 * S,"the last one 4:00, from the samples since it");

  // Ten minutes on: the peak at 0:30 and the overloads at 2:00 fall out of the window
  for(int t = 302; t <= 720; t += 2)
    reading("fobos",T0 + t * S,-31.0f,8,(uint64_t)(t - 240) * 50000000ull);
  check(summary_view(&Sum,0,&v) && v.whole_window,"ten minutes seen");
  check(fabsf(v.peak + 31) < 0.01f,"the peak of 0:30 dropped after ten minutes");
  check(v.recent_overs == 3,"of the overloads, the 3 at 4:00 still in the last ten minutes");
  char line[256];
  summary_frontend_line(&v,T0 + 724 * S,line,sizeof line);
  printf("     %s\n",line);
  check(strstr(line,"fobos 445.6000 MHz | IF -31.0 dBFS") != NULL && strstr(line,"gain 22 dB") != NULL
	&& strstr(line,"A/D over 8, 10m: 3, last 8m ago") != NULL && strstr(line,"| 4s ago") != NULL,"the line");

  // A radiod restart: its count starts again from 0
  reading("fobos",T0 + 722 * S,-31.0f,1,0);
  check(summary_view(&Sum,0,&v) && v.recent_overs == 1,"after radiod restarts: what it has counted since");

  // A second front end is kept apart
  reading("rx888",T0 + 700 * S,-20.0f,0,0);
  struct sum_view w;
  check(summary_view(&Sum,1,&w) && strcmp(w.now->description,"rx888") == 0 && fabsf(w.now->if_dbfs + 20) < 0.01f
	&& summary_view(&Sum,0,&v) && fabsf(v.now->if_dbfs + 31) < 0.01f,"a second front end, apart from the first");

  // Busiest: slot 3 plays 60 s, slot 7 30 s, slot 9 a moment; then all quiet 60 s
  int64_t t = T0;
  for(int i = 0; i <= 1200; i++, t += S / 10){
    summary_activity(&Sum,3,111,i < 600,t);
    summary_activity(&Sum,7,222,i >= 600 && i < 900,t);
    summary_activity(&Sum,9,333,i == 1000,t);
  }
  int slots[3];
  int const n = summary_busiest(&Sum,slots,3,t);
  printf("     seconds: %.1f %.1f %.1f\n",summary_seconds(&Sum,3,t),summary_seconds(&Sum,7,t),summary_seconds(&Sum,9,t));
  check(n == 2 && slots[0] == 3 && slots[1] == 7,"busiest: 60 s, then 30 s (a moment doesn't count)");
  check(fabs(summary_seconds(&Sum,3,t) - 60) < 0.2 && fabs(summary_seconds(&Sum,7,t) - 30) < 0.2,"their seconds of audio");
  // Ten minutes more of quiet: gone from the window
  for(int i = 0; i < 6000; i++, t += S / 10)
    summary_activity(&Sum,3,111,false,t);
  check(summary_seconds(&Sum,3,t) == 0 && summary_busiest(&Sum,slots,3,t) == 0,"ten minutes on, gone");
  // Another stream in the slot starts from nothing
  summary_activity(&Sum,7,999,true,t);
  check(summary_seconds(&Sum,7,t) == 0,"another stream in a slot starts again");

  char ago[16];
  summary_ago(45 * S,ago,sizeof ago);
  bool ok = strcmp(ago,"45s") == 0;
  summary_ago(185 * S,ago,sizeof ago);
  ok = ok && strcmp(ago,"3m") == 0;
  summary_ago(7300 * S,ago,sizeof ago);
  ok = ok && strcmp(ago,"2h") == 0;
  check(ok,"durations: 45s, 3m, 2h");

  puts(Failures ? "Receiver summary tests FAILED" : "Receiver summary tests passed");
  return Failures != 0;
}
