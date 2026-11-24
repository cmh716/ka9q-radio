// monitor's receiver summary (monitor-summary.c): the IF peak held for ten
// minutes then dropped, by monitor's clock even when status stops;
// overloads counted as they rise, across a radiod restart; a repeated or
// older status ignored; receivers kept apart by identity, not description;
// the channels with the most audio
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
static struct rx_summary Sum = { .lock = PTHREAD_MUTEX_INITIALIZER };
static int64_t const T0 = 1795000000LL * S; // a GPS time, ns

static void reading_at(char const *key,char const *desc,int64_t t,float level,uint64_t overs,uint64_t since){
  struct sum_reading r = { .clocktime = t, .frequency = 445600000, .samprate = 50e6,
			   .if_dbfs = level, .rf_gain = 22, .overranges = overs, .samp_since_over = since };
  strcpy(r.key,key);
  strcpy(r.description,desc);
  summary_reading(&Sum,&r);
}
static void reading(int64_t t,float level,uint64_t overs,uint64_t since){
  reading_at("239.1.2.3:5006","fobos",t,level,overs,since);
}

int main(void){
  struct sum_view v;
  check(!summary_view(&Sum,0,T0,&v),"nothing yet: no front end");

  // A quiet minute at -31 dBFS, a peak of -12 at 0:30
  for(int t = 0; t <= 60; t += 2)
    reading(T0 + t * S,t == 30 ? -12.0f : -31.0f,0,0);
  check(summary_view(&Sum,0,T0 + 61 * S,&v) && fabsf(v.now.if_dbfs + 31) < 0.01f,"the latest IF level");
  check(fabsf(v.peak + 12) < 0.01f && v.peak_time == T0 + 30 * S,"the peak and when");
  check(v.recent_overs == 0 && v.last_over == 0 && !v.whole_window && !v.recent_over,"no overloads; under ten minutes seen");

  // A status repeated (every channel's carries the front end's) or older: ignored
  reading(T0 + 60 * S,-5.0f,0,0);
  reading(T0 + 50 * S,-5.0f,0,0);
  check(summary_view(&Sum,0,T0 + 61 * S,&v) && fabsf(v.now.if_dbfs + 31) < 0.01f && fabsf(v.peak + 12) < 0.01f,
	"a repeated or older status changes nothing");

  // Overloads: the count goes to 5 at 2:00, 8 at 4:00 (since: samples since the last)
  for(int t = 62; t <= 300; t += 2){
    uint64_t const overs = t >= 240 ? 8 : t >= 120 ? 5 : 0;
    uint64_t const since = t >= 240 ? (uint64_t)(t - 240) * 50000000ull : t >= 120 ? (uint64_t)(t - 120) * 50000000ull : 0;
    reading(T0 + t * S,-31.0f,overs,since);
  }
  check(summary_view(&Sum,0,T0 + 301 * S,&v) && v.recent_overs == 8,"8 overloads counted in the time seen");
  check(v.last_over == T0 + 240 * S && v.recent_over,"the last one 4:00, from the samples since it: recent");

  // Ten minutes on: the peak at 0:30 and the overloads at 2:00 fall out of the window
  for(int t = 302; t <= 720; t += 2)
    reading(T0 + t * S,-31.0f,8,(uint64_t)(t - 240) * 50000000ull);
  check(summary_view(&Sum,0,T0 + 724 * S,&v) && v.whole_window,"ten minutes seen");
  check(fabsf(v.peak + 31) < 0.01f,"the peak of 0:30 dropped after ten minutes");
  check(v.recent_overs == 3,"of the overloads, the 3 at 4:00 still in the last ten minutes");
  char line[256];
  summary_frontend_line(&v,T0 + 724 * S,line,sizeof line);
  printf("     %s\n",line);
  check(strstr(line,"fobos 445.6000 MHz | IF -31.0 dBFS") != NULL && strstr(line,"gain 22 dB") != NULL
	&& strstr(line,"A/D over 8, 10m: 3, last 8m ago") != NULL && strstr(line,"| 4s ago") != NULL,"the line");

  // Status stops: the window goes on by monitor's clock. Twelve minutes after
  // the last status, nothing of it is recent: no peak, no overloads, no warning
  check(summary_view(&Sum,0,T0 + (720 + 720) * S,&v) && !isfinite(v.peak) && v.recent_overs == 0 && !v.recent_over,
	"with status stopped, the peak and the overloads still age out");
  summary_frontend_line(&v,T0 + 1440 * S,line,sizeof line);
  check(strstr(line,"| 12m ago") != NULL,"and the figures say how old they are");

  // A radiod restart (Codex's case): the window starts at 100, the count
  // reaches 300, radiod restarts and counts 120: 320 in the window
  reading_at("239.9.9.9:5006","rx",T0,-30.0f,100,0);
  reading_at("239.9.9.9:5006","rx",T0 + 10 * S,-30.0f,300,0);
  reading_at("239.9.9.9:5006","rx",T0 + 20 * S,-30.0f,50,0);  // restarted: 50 since
  reading_at("239.9.9.9:5006","rx",T0 + 30 * S,-30.0f,120,0);
  check(summary_view(&Sum,1,T0 + 31 * S,&v) && v.recent_overs == 320,"across a radiod restart: 200 before, 120 since");

  // A gap in status: the first status after it counts 5 more overloads, the
  // last of them 20 minutes before it: none belong in the last ten minutes
  reading_at("239.7.7.7:5006","gap",T0,-30.0f,10,0);
  reading_at("239.7.7.7:5006","gap",T0 + 1800 * S,-30.0f,15,(uint64_t)1200 * 50000000ull);
  check(summary_view(&Sum,2,T0 + 1801 * S,&v) && v.recent_overs == 0 && !v.recent_over && v.last_over == T0 + 600 * S,
	"after a status gap, overloads known to be older than ten minutes: not counted recent");

  // Codex: an increase whose last overload was 9 minutes before its status
  // came: in the window now, gone about a minute later (not ten)
  memset(Sum.fe,0,sizeof Sum.fe);
  reading_at("late","late",T0,-30.0f,10,0);
  reading_at("late","late",T0 + 900 * S,-30.0f,13,(uint64_t)540 * 50000000ull); // last one at 6:00
  check(summary_view(&Sum,0,T0 + 901 * S,&v) && v.recent_overs == 3 && v.last_over == T0 + 360 * S,
	"overloads 9 minutes old when their status came: counted while within ten minutes");
  check(summary_view(&Sum,0,T0 + 980 * S,&v) && v.recent_overs == 0 && !v.recent_over,
	"and gone once the last of them is ten minutes old");

  // Codex: the last overload 599 s before its status, sixty buckets apart
  // (one ring slot): counted until it is ten minutes old, not overwritten
  memset(Sum.fe,0,sizeof Sum.fe);
  {
    int64_t const t1 = (T0 / (10 * S)) * (10 * S) + 600 * S + 8 * S; // its last overload 9 s into the bucket sixty back
    reading_at("edge","edge",t1 - 700 * S,-30.0f,10,0);
    reading_at("edge","edge",t1,-30.0f,12,(uint64_t)599 * 50000000ull); // last one at t1 - 599 s
    check(summary_view(&Sum,0,t1,&v) && v.recent_overs == 2 && v.recent_over,
	  "599 s old, sixty buckets back: counted, as the warning says (not overwritten by the new reading)");
    check((t1 - 599 * S) / (10 * S) == t1 / (10 * S) - 60,"(the overload's bucket is sixty before its status's)");
    check(summary_view(&Sum,0,t1 + 1 * S,&v) && v.recent_overs == 2 && v.recent_over,
	  "past ten minutes by the clock, still in the count's window: count and warning together");
    check(summary_view(&Sum,0,t1 + 2 * S,&v) && v.recent_overs == 0 && !v.recent_over,
	  "and both gone together");
  }

  // Two receivers with no description, sharing a status group, from two
  // addresses (monitor keys them by both): kept apart
  memset(Sum.fe,0,sizeof Sum.fe);
  reading_at("192.168.0.10:5004 239.5.5.5:5006","",T0 + 700 * S,-20.0f,0,0);
  reading_at("192.168.0.11:5004 239.5.5.5:5006","",T0 + 700 * S,-40.0f,0,0);
  reading_at("192.168.0.10:5004 239.5.5.5:5006","",T0 + 702 * S,-21.0f,0,0);
  struct sum_view a,b;
  check(summary_view(&Sum,0,T0 + 703 * S,&a) && summary_view(&Sum,1,T0 + 703 * S,&b)
	&& fabsf(a.now.if_dbfs + 21) < 0.01f && fabsf(b.now.if_dbfs + 40) < 0.01f && fabsf(a.peak + 20) < 0.01f,
	"two unnamed receivers on one status group, apart");

  // Busiest: seconds of audio played. Slot 3 60 s, slot 7 30 s, slot 9 a moment
  summary_channel_reset(&Sum,3,111);
  summary_channel_reset(&Sum,7,222);
  summary_channel_reset(&Sum,9,333);
  summary_channel_label(&Sum,3,111,424700000,"110");
  summary_channel_label(&Sum,7,222,453500000,"Alden");
  summary_channel_label(&Sum,7,999,1,"not its stream"); // another SSRC's label: ignored
  int64_t t = T0;
  for(int i = 0; i < 1200; i++, t += S / 10){
    if(i < 600) summary_audio(&Sum,3,111,0.1,t);
    if(i >= 600 && i < 900) summary_audio(&Sum,7,222,0.1,t);
    if(i == 1000) summary_audio(&Sum,9,333,0.02,t);
  }
  struct sum_busy busy[3];
  int n = summary_busiest(&Sum,busy,3,t);
  check(n == 2 && busy[0].slot == 3 && busy[1].slot == 7 && fabs(busy[0].seconds - 60) < 0.01 && fabs(busy[1].seconds - 30) < 0.01,
	"busiest: 60 s, then 30 s of audio (a moment doesn't count)");
  check(strcmp(busy[0].label,"110") == 0 && busy[0].freq == 424700000 && strcmp(busy[1].label,"Alden") == 0,
	"each with its own label and frequency, as its status gave them");
  check(summary_busiest(&Sum,busy,3,t + 700 * S) == 0,"ten minutes on, gone");
  // Slot 7 closed, then another session made in it: nothing of the old shows
  summary_channel_reset(&Sum,7,0);
  n = summary_busiest(&Sum,busy,3,t);
  check(n == 1 && busy[0].slot == 3,"a closed slot drops out");
  summary_channel_reset(&Sum,7,222); // even with the same SSRC
  summary_audio(&Sum,7,222,0.1,t);
  n = summary_busiest(&Sum,busy,3,t);
  check(n == 1 && busy[0].slot == 3,"a new session in it starts from nothing, its SSRC the same or not");
  summary_audio(&Sum,9,444,5.0,t); // audio of a stream the slot wasn't made for
  check(summary_busiest(&Sum,busy,3,t) == 1,"audio from a stream not the slot's: not counted");

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
