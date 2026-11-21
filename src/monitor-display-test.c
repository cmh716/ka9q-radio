// The dashboard, drawn into ncurses's own screen (no terminal) and read
// back: a held call fading after it ends, a new call replacing it, and the
// Call column growing with the terminal's width.
// SPDX-License-Identifier: GPL-3.0-or-later
#define main monitor_display_test_unused_main
#include "monitor-display.c"
#undef main

static int Failures;
static void check(int ok,char const *what){
  printf("%s %s\n",ok ? "ok  " : "FAIL",what);
  Failures += !ok;
}
// The screen's row r, as text
static void row_text(int r,char *out,int size){
  mvinnstr(r,0,out,size - 1);
}
// The attribute of the first character of text on row r
static attr_t attr_at(int r,char const *text){
  char line[512];
  row_text(r,line,sizeof line);
  char const *p = strstr(line,text);
  if(p == NULL)
    return (attr_t)-1;
  return mvinch(r,(int)(p - line)) & A_ATTRIBUTES & ~A_COLOR;
}
static int find_row(char const *text){
  char line[512];
  for(int r = 0; r < LINES; r++){
    row_text(r,line,sizeof line);
    if(strstr(line,text) != NULL)
      return r;
  }
  return -1;
}

int main(void){
  setlocale(LC_ALL,"C.UTF-8");
  FILE *null = fopen("/dev/null","w");
  SCREEN *scr = newterm("xterm-256color",null,stdin);
  if(scr == NULL){
    puts("no terminal description: skipped");
    return 0;
  }
  set_term(scr);
  resizeterm(30,160);
  Ui_colors = false;
  // Two sessions in use; the second is selected, so the first shows its own look
  for(int i = 0; i < NSESSIONS; i++)
    Sess_ptr[i] = &Sessions[i];
  for(int i = 0; i < 2; i++){
    Sess_ptr[i] = &Sessions[i];
    Sessions[i].inuse = true;
    Sessions[i].chan.tune.freq = 444375000 + 1000000 * i;
    Sessions[i].chan.demod_type = DMR_DEMOD;
    Sessions[i].packets = 1;
  }
  Current = 1;
  sess_t *sp = &Sessions[0];
  int64_t const now = gps_time_ns();
  sp->last_active = now;
  strcpy(sp->chan.dv.call_dest,"9");
  strcpy(sp->chan.dv.call_src,"3169098");
  strcpy(sp->chan.dv.call_alias,"KD2TYY James Buffalo NY USA");
  sp->held_call = sp->chan.dv;
  sp->held_live = true;
  dashboard();
  int r = find_row("TG9 KD2TYY James Buffalo NY USA");
  check(r >= 0,"a live call, its whole alias in the wide Call column");

  // It ends: held, normal for 30 s, then dim, then gone
  sp->chan.dv.call_dest[0] = sp->chan.dv.call_src[0] = sp->chan.dv.call_alias[0] = '\0';
  sp->held_live = false;
  sp->held_ended = gps_time_ns() - 5 * (int64_t)BILLION;
  sp->last_active = sp->held_ended;
  dashboard();
  r = find_row("TG9 KD2TYY");
  check(r >= 0 && !(attr_at(r,"TG9 KD2TYY") & A_DIM),"5 s after: still shown, not dim");
  sp->held_ended = gps_time_ns() - 60 * (int64_t)BILLION;
  dashboard();
  r = find_row("TG9 KD2TYY");
  check(r >= 0 && (attr_at(r,"TG9 KD2TYY") & A_DIM),"60 s after: dim");
  sp->held_ended = gps_time_ns() - (int64_t)(Call_hold + 1) * BILLION;
  dashboard();
  check(find_row("TG9 KD2TYY") < 0,"after the hold: gone");

  // A new call replaces a held one at once
  sp->held_ended = gps_time_ns() - 5 * (int64_t)BILLION;
  strcpy(sp->chan.dv.call_dest,"TG 3100");
  strcpy(sp->chan.dv.call_src,"3123456");
  dashboard();
  check(find_row("TG3100 3123456") >= 0 && find_row("TG9 KD2TYY") < 0,"a new call replaces the held one");

  // Narrower terminals: a narrower Call column, the text cut, not wrapped
  resizeterm(30,90);
  dashboard();
  char line[512];
  r = find_row("TG3100");
  row_text(r,line,sizeof line);
  check(r >= 0 && strstr(line,"3123456") != NULL,"90 columns: still the talkgroup and the talker");
  resizeterm(30,200);
  strcpy(sp->chan.dv.call_alias,"A VERY LONG TALKER ALIAS OF 27");
  dashboard();
  check(find_row("TG3100 A VERY LONG TALKER ALIAS OF 27") >= 0,"200 columns: the long alias whole");
  endwin();
  delscreen(scr);
  puts(Failures ? "Monitor display tests FAILED" : "Monitor display tests passed");
  return Failures != 0;
}
