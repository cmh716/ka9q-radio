// Control plane section of the multicast monitor program
// Moved out of monitor.c when it was getting way too big
// Copyright Aug 2024 Phil Karn, KA9Q

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <sys/stat.h>
#include <opus/opus.h>
#include <portaudio.h>
#include <ncurses.h>
#include <locale.h>
#include <signal.h>
#include <getopt.h>
#include <iniparser/iniparser.h>
#if __linux__
#include <alsa/asoundlib.h>
#endif
#include <string.h>
#include <sysexits.h>
#include <poll.h>
#include <regex.h>
#include <ctype.h>

#include "config.h"
#include "misc.h"
#include "multicast.h"
#include "radio.h"
#include "iir.h"
#include "morse.h"
#include "status.h"
#include "monitor.h"
#include "monitor-summary.h"
#include "database.h"
#include "timing.h"
#include "locator.h"

bool Auto_sort = false;
int Update_interval = 100;  // Default time in ms between display updates

static void update_monitor_display(void);
static void process_keyboard(void);
static int render_right(int const row, int const col, char scratch[LINES][COLS],int const nrows,int const ncols);
static int render_left(int const row, int const col, char scratch[LINES][COLS],int const nrows);

static int First_session = 0;
static int Sessions_per_screen = 0;
static int Current = 0;
static bool help = false;
static struct rx_summary Summary; // the receiver summary's history
static bool Show_summary = true;  // 'S'


static void show_help(void);

// Color is optional: attributes and ASCII frames also work on monochrome terminals.
enum { UI_FRAME = 1, UI_ACTIVE, UI_DIGITAL, UI_WARNING, UI_SELECTED };
static bool Ui_colors;

sess_t *Sess_ptr[NSESSIONS];

// Versions of ncurses routines that truncate at EOL
// Remaining problem: if I use the last column, the position will still
// wrap to the first column of the next row, and I can't stop that except by not using the last column.
// Return number of chars written

#if 0 // currently unused
int mvprintwt(int row,int col,char const *fmt,...){
  if(col < 0)
    return ERR;

  int space = COLS - col - 1; // leave last column open
  if(space <= 0)
    return ERR;

  va_list ap;
  va_start(ap,fmt);
  char string[COLS+1]; // Maximum line length plus null
  int r = vsnprintf(string,space,fmt,ap); // write only 'space' characters, space <= COLS
  va_end(ap);
  if(r < 0)
    return ERR;
  mvaddstr(row,col,string);
  return r;
}
#endif
// Same for printw() - truncate at EOL
// returns number of chars written
int printwt(char const *fmt,...){
  int row,col;
  getyx(stdscr,row,col);
  if(col < 0)
    return ERR;

  int space = COLS - col - 1; // leave last column open
  if(space <= 0)
    return ERR;

  va_list ap;
  va_start(ap,fmt);
  char string[COLS+1]; // Maximum line length plus null
  int r = vsnprintf(string,space,fmt,ap); // write only 'space' characters, space <= COLS
  va_end(ap);
  if(r < 0)
    return ERR;
  mvaddstr(row,col,string);
  return r;
}

// Same for mvaddstr() and addstr()
#if 0
int mvaddstrt(int row,int col,char const *str){
  size_t space = COLS - col - 1; // Leave last column open
  if(strlen(str) <= space)
    return mvaddstr(row,col,str); // fits
  char temp[space+1];
  strlcpy(temp,str,sizeof temp); // truncate
  return mvaddstr(row,col,temp);
}
#endif
int addstrt(char const *str){
  int row,col;
  getyx(stdscr,row,col);
  size_t space = COLS - col - 1; // Leave last column open
  if(strlen(str) <= space)
    return mvaddstr(row,col,str); // fits
  char temp[space+1];
  strlcpy(temp,str,sizeof(temp)); // truncate
  return mvaddstr(row,col,temp);
}

// Use ncurses to display streams
void *display(void *arg){
  pthread_setname("display");
  (void)arg; // unused

  if(initscr() == NULL){
    fprintf(stderr,"initscr() failed, disabling control/display thread\n");
    pthread_exit(NULL);
  }
  Ui_colors = has_colors() && start_color() == OK;
  if(Ui_colors){
    int background = use_default_colors() == OK ? -1 : COLOR_BLACK;
    init_pair(UI_FRAME,COLOR_CYAN,background);
    init_pair(UI_ACTIVE,COLOR_GREEN,background);
    init_pair(UI_DIGITAL,COLOR_CYAN,background);
    init_pair(UI_WARNING,COLOR_YELLOW,background);
    init_pair(UI_SELECTED,COLOR_CYAN,background);
  }
  keypad(stdscr,TRUE);
  timeout(Update_interval);
  cbreak();
  noecho();

  // Build list of pointers into Sessions array
  for(int i=0; i < NSESSIONS; i++)
    Sess_ptr[i] = Sessions + i;

  while(!atomic_load_explicit(&Terminate,memory_order_acquire)){
    // Start screen update
    move(0,0);
    clrtobot();
    if(Source == NULL)
      addstrt("KA9Q Multicast Audio Monitor:");
    else
      printwt("KA9Q Multicast Audio Monitor, only from %s:",Source);

    for(int i=0;i<Nfds;i++)
      printwt(" %s",Mcast_address_text[i]);
    int row,col;
    getyx(stdscr,row,col);
    if(col != 0){
      col = 0;
      move(++row,col);
    }

    if(help)
      show_help();
    if(Quiet_mode){
      addstrt("Hit 'q' to resume screen updates\n");
    } else if(!help)
      update_monitor_display();

    process_keyboard();
  }
  return NULL;
}
// sort callback for sort_session_active() for comparing sessions by most recently active (or currently longest active)
static int scompare(void const *a, void const *b){
  sess_t const * const s1 = *(sess_t **)a;
  sess_t const * const s2 = *(sess_t **)b;

  if(s1 == s2) // same session compares equal (shouldn't happen)
    return 0;
  if(s1 == NULL || !inuse(s1)){ // first is empty or idle, put it at end
    if(s2 == NULL || !inuse(s1))
      return 0;
    else
      return +1;
  } else if(s2 == NULL || !inuse(s2)){
    return -1; // s1 is not empty
  }
  // Both in use. Fuzz needed because active sessions are updated when packets arrive
  if(s1->active != 0 && s2->active != 0){
    if(fabs(s1->active - s2->active) < 1.0) {
      return s1->ssrc > s2->ssrc ? +1 : -1; // resolve ties by ssrc to stop rapid flipping
    } else if(s1->active > s2->active){
      return -1; // s1 Longer active, move to top
    } else {
      return +1; // s2 longer active, move it to the top
    }
  } else {
    if(s2->active != 0)
      return +1;   // s1 inctive
    else if(s1->active != 0)
      return -1; // s2 inactive
    else     // Both inactive, sort by last active times
      return s1->last_active > s2->last_active ? -1 : +1;    // last_active is in nanoseconds so chances of equality are nil
  }
}

// sort callback for sort_session() for comparing sessions by total time
static int tcompare(void const *a, void const *b){
  sess_t const * const s1 = *(sess_t **)a;
  sess_t const * const s2 = *(sess_t **)b;

  if(s1 == s2) // same session compares equal (shouldn't happen)
    return 0;
  if(s1 == NULL || !inuse(s1)) // first is empty or idle, put it at end
    return +1;
  if(s2 == NULL || !inuse(s2)) // second is empty or idle, put it at end
    return -1;

  if(fabs(s1->tot_active - s2->tot_active) < 1.0) // equal within margin
    return s1->ssrc > s2->ssrc ? +1 : -1; // resolve ties by ssrc to stop rapid flipping
  if(s1->tot_active > s2->tot_active)
    return -1;
  return +1;
}

// Defragment session list
static int defragment_session(void){
  pthread_mutex_lock(&Sess_mutex);
  for(int i=0; i < NSESSIONS-1; i++){
    // Find next unused slot
    if(inuse(Sess_ptr[i]))
      continue;

    // Find following used slot
    int j = i+1;
    while(j < NSESSIONS && !inuse(Sess_ptr[j]))
      j++;

    if(j == NSESSIONS)
      break;
    sess_t *save = Sess_ptr[i];
    Sess_ptr[i] = Sess_ptr[j];
    Sess_ptr[j] = save;
  }
  pthread_mutex_unlock(&Sess_mutex);
  return 0;
}


// Sort session list in increasing order of age
static int sort_session_active(void){
  pthread_mutex_lock(&Sess_mutex);
  qsort(Sess_ptr,NSESSIONS,sizeof(Sess_ptr[0]),scompare);
  pthread_mutex_unlock(&Sess_mutex);
  return 0;
}
static int sort_session_total(void){
  pthread_mutex_lock(&Sess_mutex);
  qsort(Sess_ptr,NSESSIONS,sizeof(Sess_ptr[0]),tcompare);
  pthread_mutex_unlock(&Sess_mutex);
  return 0;
}

static void ui_text(int row,int col,char const *text,attr_t attr,int pair){
  if(row < 0 || row >= LINES || col < 0 || col >= COLS-1)
    return;
  attrset(attr | (Ui_colors ? COLOR_PAIR(pair) : 0));
  mvaddnstr(row,col,text,COLS-col-1);
  attrset(A_NORMAL);
}

static void ui_rule(int row,char const *title){
  if(row < 0 || row >= LINES || COLS < 4)
    return;
  attrset(Ui_colors ? COLOR_PAIR(UI_FRAME) : A_NORMAL);
  mvhline(row,0,'-',COLS-1);
  mvaddch(row,0,'+');
  mvaddch(row,COLS-2,'+');
  attrset(A_NORMAL);
  if(title != NULL)
    ui_text(row,2,title,A_BOLD,UI_FRAME);
}

static void show_help(void){
  erase();
  ui_rule(0," MONITOR KEYBOARD HELP ");
  char path[PATH_MAX];
  dist_path(path,sizeof path,"monitor-help.txt");
  FILE *fp = fopen(path,"r");
  if(fp != NULL){
    char *line = NULL;
    size_t size = 0;
    int row = 1;
    while(row < LINES-2 && getline(&line,&size,fp) > 0){
      line[strcspn(line,"\r\n")] = '\0';
      ui_text(row++,2,line,A_NORMAL,UI_FRAME);
    }
    free(line);
    fclose(fp);
  } else {
    ui_text(1,2,"Up/Down select | m/u mute/unmute | +/- gain",A_NORMAL,UI_FRAME);
    ui_text(2,2,"M/U all | v diagnostic view | Q quit",A_NORMAL,UI_FRAME);
  }
  ui_rule(LINES-2,NULL);
  ui_text(LINES-1,0," h Return to channels | Q Quit",A_REVERSE,UI_FRAME);
  curs_set(0);
}

static char const *ui_detected(sess_t const *sp){
  if(sp->chan.dv.call_mode[0] != '\0')
    return sp->chan.dv.call_mode;
  // Automatic mode publishes the recognized mode at the front of source.
  static char const * const modes[] = {"DMR","DSTAR","P25","YSF","M17","NXDN","FM"};
  for(size_t i=0; i < sizeof modes / sizeof modes[0]; i++){
    size_t n = strlen(modes[i]);
    if(strncmp(sp->chan.dv.source,modes[i],n) == 0
       && (sp->chan.dv.source[n] == ' ' || sp->chan.dv.source[n] == '\0'))
      return modes[i];
  }
  return "-";
}

static void ui_tone(sess_t const *sp,char *out,size_t size){
  strlcpy(out,"-",size);
  if(!session_has_analog_audio(sp)){
    // P25: the NAC last received (a following channel's is its control channel's)
    if(sp->chan.demod_type == P25_DEMOD && sp->chan.p25.have_nac)
      snprintf(out,size,"N%03X",sp->chan.p25.last_nac);
    return;
  }
  if(sp->current_tone > 0)
    snprintf(out,size,"%.1f",sp->current_tone);
  else if(sp->current_dcs != 0)
    snprintf(out,size,"%c%03d",sp->current_dcs_inverted ? 'I' : 'D',sp->current_dcs);
}

typedef __typeof__(((chan_t *)0)->dv) dv_t;

// A call: talkgroup (or destination callsign) and talker, and on a trunked
// channel the voice frequency it is following (tune: where the channel is).
// brief: without the followed frequency, for a narrow column
static void ui_call_text(dv_t const *dv,double tune,bool brief,char *out,size_t size){
  out[0] = '\0';
  char const *dest = dv->call_dest;
  if(dest[0] == '\0')
    return;
  // "110", "TG 110" or "TG110" is talkgroup 110; anything else (a private
  // call "U2000", a callsign) as it is
  if(strncmp(dest,"TG",2) == 0)
    dest += dest[2] == ' ' ? 3 : 2;
  bool const number = dest[0] != '\0' && strspn(dest,"0123456789") == strlen(dest);
  snprintf(out,size,"%s%s",number ? "TG" : "",number ? dest : dv->call_dest);
  double const freq = dv->call_freq;
  if(!brief && freq > 0 && fabs(freq - tune) > 100){
    char where[24];
    snprintf(where,sizeof where," %.4f",freq * 1e-6);
    strlcat(out,where,size);
  }
  // The talker: by alias (DMR talker alias) when one has come, else ID
  char const *talker = dv->call_alias[0] != '\0' ? dv->call_alias : dv->call_src;
  if(talker[0] != '\0'){
    strlcat(out," ",size);
    strlcat(out,talker,size);
  }
  // where they are (D-STAR or YSF GPS), as a locator: only beside whose
  // position it is
  if(dv->call_has_position && talker[0] != '\0'){
    char grid[8];
    latlon_to_locator(dv->call_lat,dv->call_lon,grid,sizeof grid);
    if(grid[0] != '\0'){
      strlcat(out," ",size);
      strlcat(out,grid,size);
    }
  }
  // and what they've written (D-STAR's text, YSF's route), if there's room
  if(dv->call_message[0] != '\0'){
    strlcat(out," ",size);
    strlcat(out,dv->call_message,size);
  }
}

// How the Call column shows a channel: the call going on, else the last one,
// held for Call_hold after it ended, fading (normal for 30 s, then dim);
// else a trunked channel's state: "CC R2", "no CC", "CC search", "LTR R1".
// Returns how faded a held call is: 1 normal, 2 dim; 0 for a call going on
// or a state (they take the row's look)
static int ui_call(sess_t const *sp,int64_t now,bool brief,char *out,size_t size){
  if(sp->chan.dv.call_dest[0] != '\0'){
    ui_call_text(&sp->chan.dv,sp->chan.tune.freq,brief,out,size);
    return 0;
  }
  // The held call as the status thread last left it: copied whole under the
  // mutex it writes it under (the dashboard draws without it), so that one
  // call's fields aren't mixed with the next's
  pthread_mutex_lock(&Sess_mutex);
  dv_t const held = sp->held_call;
  double const held_tune = sp->held_tune_freq;
  int64_t const ended = sp->held_live ? 0 : sp->held_ended;
  pthread_mutex_unlock(&Sess_mutex);
  if(ended != 0 && now - ended < Call_hold){
    ui_call_text(&held,held_tune,brief,out,size);
    if(out[0] != '\0')
      return now - ended < 30 * (int64_t)BILLION ? 1 : 2;
  }
  out[0] = '\0';
  char const *s = sp->chan.dv.source;
  if(strncmp(s,"CC ",3) == 0 || strcmp(s,"no CC") == 0 || strcmp(s,"CC search") == 0
     || strncmp(s,"LTR R",5) == 0)
    strlcpy(out,s,size);
  return 0;
}

// in as exactly width columns: cut, or padded with spaces. Counts UTF-8
// characters, not bytes (a talker alias may have accents), each one column,
// and never cuts one in half
static void ui_fit(char *out,size_t size,char const *in,int width){
  size_t o = 0;
  int cols = 0;
  for(size_t i = 0; in[i] != '\0' && cols < width;){
    size_t n = 1;
    while((in[i+n] & 0xc0) == 0x80) // continuation bytes
      n++;
    if(o + n + 1 > size)
      break;
    memcpy(out + o,in + i,n);
    o += n;
    i += n;
    cols++;
  }
  for(; cols < width && o + 1 < size; cols++)
    out[o++] = ' ';
  out[o] = '\0';
}

// The last position a call on this channel sent: "Position: KD2TYY FN02qu
// 42.83530,-78.60750 | 23 km 284 deg | 3:12 ago" (distance and bearing with
// --location; once the call has gone, how long since that position came)
static void ui_position(sess_t const *sp,char *out,size_t size){
  if(!sp->have_position){
    strlcpy(out,"Position: -",size);
    return;
  }
  char grid[8];
  latlon_to_locator(sp->position_lat,sp->position_lon,grid,sizeof grid);
  snprintf(out,size,"Position: %s%s%s %.5f,%.5f",sp->position_call,sp->position_call[0] ? " " : "",grid,
           sp->position_lat,sp->position_lon);
  if(Have_location){
    double km,bearing;
    distance_bearing(My_lat,My_lon,sp->position_lat,sp->position_lon,&km,&bearing);
    char d[48];
    snprintf(d,sizeof d," | %.*f km %.0f deg",km < 10 ? 1 : 0,km,bearing);
    strlcat(out,d,size);
  }
  if(!sp->chan.dv.call_has_position){ // heard before: how long ago
    int64_t const s = (gps_time_ns() - sp->position_time) / BILLION;
    char ago[32];
    if(s < 3600)
      snprintf(ago,sizeof ago," | %d:%02d ago",(int)(s / 60),(int)(s % 60));
    else
      snprintf(ago,sizeof ago," | %dh%02d ago",(int)(s / 3600),(int)(s / 60 % 60));
    strlcat(out,ago,size);
  }
}

// How long since the channel last sent audio: "now", "4:07", "3h12", "2d"
static void ui_last(sess_t const *sp,int64_t now,char *out,size_t size){
  if(sp->packets == 0){
    strlcpy(out,"-",size);
    return;
  }
  int64_t const s = (now - sp->last_active) / BILLION;
  if(s < 1)
    strlcpy(out,"now",size);
  else if(s < 3600)
    snprintf(out,size,"%d:%02d",(int)(s / 60),(int)(s % 60));
  else if(s < 86400)
    snprintf(out,size,"%dh%02d",(int)(s / 3600),(int)(s / 60 % 60));
  else
    snprintf(out,size,"%dd",(int)(s / 86400));
}

// The receiver summary: each front end's level, gain and overloads, the
// last ten minutes' peak and overload count held, and the busiest channels.
// Fed from every session (each carries its front end's status) at each
// update; draws from row (if draw), returns the rows it took
static int ui_summary(int row,int count,int64_t now,bool draw){
  for(int i = 0; i < count; i++){
    sess_t const *sp = Sess_ptr[i];
    struct sum_reading const r = {
      .description = sp->frontend.description,
      .clocktime = sp->chan.clocktime,
      .frequency = sp->frontend.frequency,
      .samprate = sp->frontend.samprate,
      .if_dbfs = power2dB(sp->frontend.if_power),
      .rf_gain = sp->frontend.rf_gain,
      .agc = sp->frontend.rf_agc,
      .lna_gain = sp->frontend.lna_gain,
      .mixer_gain = sp->frontend.mixer_gain,
      .if_gain = sp->frontend.if_gain,
      .overranges = sp->frontend.overranges,
      .samp_since_over = sp->frontend.samp_since_over,
    };
    if(isfinite(r.if_dbfs))
      summary_reading(&Summary,&r);
    summary_activity(&Summary,(int)(sp - Sessions),sp->ssrc,now - sp->last_active < BILLION/2,now);
  }
  if(!draw)
    return 0;
  int rows = 0;
  char line[512];
  for(int k = 0; k < SUM_FRONTENDS && rows < 2; k++){
    struct sum_view v;
    if(!summary_view(&Summary,k,&v))
      continue;
    summary_frontend_line(&v,now,line,sizeof line);
    bool const overloaded = v.now->overranges > 0 && v.last_over != 0
      && v.now->clocktime - v.last_over < (int64_t)SUM_WINDOW_S * BILLION;
    ui_text(row + rows++,2,line,A_NORMAL,overloaded ? UI_WARNING : UI_FRAME);
  }
  if(rows == 0)
    ui_text(row + rows++,2,"Front end: waiting for status",A_DIM,UI_FRAME);
  int slots[3];
  int const n = summary_busiest(&Summary,slots,3,now);
  int len = snprintf(line,sizeof line,"Busiest, 10 min:");
  for(int i = 0; i < n && len < (int)sizeof line; i++){
    sess_t const *sp = &Sessions[slots[i]];
    int const secs = (int)lrint(summary_seconds(&Summary,slots[i],now));
    char const *name = sp->chan.dv.call_dest[0] ? sp->chan.dv.call_dest : sp->callsign;
    len += snprintf(line + len,sizeof line - (size_t)len,"%s %.5f%s%s %d:%02d",i ? " |" : "",
		    sp->chan.tune.freq/1e6,name[0] ? " " : "",name,secs / 60,secs % 60);
  }
  if(n == 0)
    strlcat(line," no audio",sizeof line);
  ui_text(row + rows++,2,line,A_DIM,UI_FRAME);
  return rows;
}

static void dashboard(void){
  erase();
  if(LINES < 7 || COLS < 24){
    Sessions_per_screen = 1;
    ui_text(0,0,"KA9Q Monitor",A_BOLD,UI_FRAME);
    ui_text(1,0,"Enlarge terminal",A_NORMAL,UI_WARNING);
    ui_text(LINES-1,0,"h Help  Q Quit",A_REVERSE,UI_FRAME);
    return;
  }
  defragment_session();
  if(Auto_sort)
    sort_session_total();
  int count = 0,active = 0;
  int64_t const now = gps_time_ns();
  while(count < NSESSIONS && inuse(Sess_ptr[count])){
    if(now - Sess_ptr[count]->last_active < BILLION/2)
      active++;
    count++;
  }
  bool const details = LINES >= 18;
  // The receiver summary, with room for it: a rule and two or three lines
  bool const summary = Show_summary && LINES >= 22;
  int top = 3; // the channels' rule
  if(summary){
    ui_rule(3," RECEIVER ");
    top = 4 + ui_summary(4,count,now,true);
  } else
    ui_summary(0,count,now,false); // (its history goes on)
  int const table_header = top + 1;
  int const table_end = details ? LINES-9 : LINES-3;
  Sessions_per_screen = max(1,table_end-table_header-1);
  Current = max(0,min(Current,count-1));
  First_session = max(0,min(First_session,max(0,count-Sessions_per_screen)));
  if(Current < First_session)
    First_session = Current;
  if(Current >= First_session+Sessions_per_screen)
    First_session = Current-Sessions_per_screen+1;

  char line[1024];
  ui_rule(0," KA9Q AUDIO MONITOR ");
  snprintf(line,sizeof line,"%d channels | %d active | %s | %.0f ms playout%s%s%s",
           count,active,Channels == 1 ? "Mono" : "Stereo",1000*Playout,
           Voting ? " | Voting" : "",Start_muted ? " | New muted" : "",
           Constant_delay ? " | Fixed delay" : "");
  ui_text(1,2,line,A_BOLD,UI_FRAME);
  snprintf(line,sizeof line,"Input: %s%s%s",Nfds > 0 ? Mcast_address_text[0] : "waiting",
           Nfds > 1 ? " (+ more)" : "",Source != NULL ? " [source filtered]" : "");
  ui_text(2,2,line,A_DIM,UI_FRAME);
  ui_rule(top," CHANNELS ");
  // Below 100 columns: no Config, SNR or Gain (the details panel has them).
  // Call takes what the terminal has beyond the other columns and some room
  // for Channel; the followed frequency only when it's wide enough
  bool const narrow = COLS < 100;
  int const fixed = narrow ? 35 : 61; // the other columns and their spaces
  int call_width = COLS - 3 - fixed - (narrow ? 12 : 20);
  call_width = narrow ? max(16,min(call_width,48)) : max(22,min(call_width,64));
  bool const brief = call_width < 30;
  if(narrow)
    snprintf(line,sizeof line,"  %10s %-7s %6s %-*s %5s %s","Frequency","Detect","Tone",call_width,"Call","Last","Channel");
  else
    snprintf(line,sizeof line,"  %10s  %-8s %-8s %6s %-*s %5s %6s %6s  %s","Frequency","Config","Detect",
             "Tone",call_width,"Call","Last","SNR","Gain","Channel");
  ui_text(table_header,2,line,A_BOLD,UI_FRAME);
  for(int i=0; i<Sessions_per_screen && First_session+i<count; i++){
    int const index = First_session+i;
    sess_t const *sp = Sess_ptr[index];
    bool const recent = now-sp->last_active < BILLION/2;
    char const *detected = ui_detected(sp);
    char const *configured = sp->chan.demod_type == DIGITAL_DEMOD && sp->chan.dv.analog
      ? "mixed" : demod_name_from_type(sp->chan.demod_type);
    char name[128];
    snprintf(name,sizeof name,"%s%s%s",sp->callsign,
             sp->callsign[0] && sp->id[0] ? " | " : "",sp->id);
    if(name[0] == '\0') strlcpy(name,"Unnamed channel",sizeof name);
    char snr[16] = "-",gain[16];
    if(isfinite(sp->snr)) snprintf(snr,sizeof snr,"%.1f",sp->snr);
    if(muted(sp)) strlcpy(gain,"mute",sizeof gain);
    else snprintf(gain,sizeof gain,"%+.0f",voltage2dB(sp->gain));
    char tone[16],full[192],call[320],last[16];
    ui_tone(sp,tone,sizeof tone);
    int const held = ui_call(sp,now,brief,full,sizeof full);
    ui_fit(call,sizeof call,full,call_width);
    ui_last(sp,now,last,sizeof last);
    if(narrow)
      snprintf(line,sizeof line,"%c %10.5f %-7.7s %6.6s %s %5.5s %s",index == Current ? '>' : ' ',
               sp->chan.tune.freq/1e6,detected,tone,call,last,name);
    else
      snprintf(line,sizeof line,"%c %10.5f  %-8.8s %-8.8s %6.6s %s %5.5s %6s %6s  %s",index == Current ? '>' : ' ',
               sp->chan.tune.freq/1e6,configured,detected,tone,call,last,snr,gain,name);
    attr_t attr = recent ? A_BOLD : A_DIM;
    int pair = UI_FRAME;
    if(muted(sp)) attr = A_DIM;
    else if(recent && isfinite(sp->snr) && sp->snr < 6) pair = UI_WARNING;
    else if(recent) pair = UI_ACTIVE;
    else if(strcmp(detected,"-") != 0 && strcmp(detected,"FM") != 0) pair = UI_DIGITAL;
    if(index == Current){
      attr = A_BOLD | A_REVERSE;
      pair = UI_SELECTED;
    }
    int const row = table_header+1+i;
    // Fill the selection's entire row, not only the text.
    attrset(attr | (Ui_colors ? COLOR_PAIR(pair) : 0));
    mvhline(row,1,' ',COLS-3);
    attrset(A_NORMAL);
    ui_text(row,2,line,attr,pair);
    // A held call, drawn again over its column, fading as it ages
    if(held != 0 && index != Current){
      char prefix[64];
      if(narrow)
        snprintf(prefix,sizeof prefix,"%c %10.5f %-7.7s %6.6s ",' ',sp->chan.tune.freq/1e6,detected,tone);
      else
        snprintf(prefix,sizeof prefix,"%c %10.5f  %-8.8s %-8.8s %6.6s ",' ',sp->chan.tune.freq/1e6,configured,detected,tone);
      ui_text(row,2 + (int)strlen(prefix),call,held == 1 ? A_NORMAL : A_DIM,strcmp(detected,"-") != 0 && strcmp(detected,"FM") != 0 ? UI_DIGITAL : UI_FRAME);
    }
  }
  if(count == 0)
    ui_text(table_header+1,2,"Waiting for multicast audio and status...",A_DIM,UI_FRAME);
  snprintf(line,sizeof line," %d-%d of %d | green active  cyan digital  yellow weak ",
           count ? First_session+1 : 0,min(count,First_session+Sessions_per_screen),count);
  ui_rule(table_end,line);
  if(details){
    ui_text(table_end+1,2,"SELECTED CHANNEL",A_BOLD,UI_FRAME);
    if(count > 0){
      sess_t const *sp = Sess_ptr[Current];
      snprintf(line,sizeof line,"%s%s%s",sp->callsign,sp->callsign[0] && sp->id[0] ? " | " : "",sp->id);
      ui_text(table_end+2,2,line,A_NORMAL,UI_FRAME);
      char slot[24] = "-";
      if(sp->chan.dv.call_slot >= 0 && sp->chan.dv.call_mode[0])
        snprintf(slot,sizeof slot,"%d",sp->chan.dv.call_slot+1);
      char tone[16];
      ui_tone(sp,tone,sizeof tone);
      char source[96];
      snprintf(source,sizeof source,"%s%s%s%s",sp->chan.dv.call_src[0] ? sp->chan.dv.call_src : "-",
               sp->chan.dv.call_alias[0] ? " (" : "",sp->chan.dv.call_alias,sp->chan.dv.call_alias[0] ? ")" : "");
      snprintf(line,sizeof line,"Detected: %s | Tone: %s | Source: %s | Destination: %s | Slot: %s",ui_detected(sp),tone,
               source,sp->chan.dv.call_dest[0] ? sp->chan.dv.call_dest : "-",slot);
      if(sp->chan.dv.call_message[0] != '\0'){
        strlcat(line," | Message: ",sizeof line);
        strlcat(line,sp->chan.dv.call_message,sizeof line);
      }
      ui_text(table_end+3,2,line,A_NORMAL,UI_DIGITAL);
      snprintf(line,sizeof line,"SSRC %u | Gain %+.0f dB | SNR %.1f dB | %s | %s",sp->ssrc,
               voltage2dB(sp->gain),sp->snr,muted(sp) ? "MUTED" : "Audio enabled",
               sp->chan.dv.source[0] ? sp->chan.dv.source : "No source label");
      ui_text(table_end+4,2,line,A_NORMAL,UI_FRAME);
      snprintf(line,sizeof line,"Packets %llu | Drops %llu | Late %llu | %s",(unsigned long long)sp->packets,
               (unsigned long long)sp->drops,(unsigned long long)sp->lates,
               sp->dest != NULL ? sp->dest : "");
      ui_text(table_end+5,2,line,A_DIM,UI_FRAME);
      ui_position(sp,line,sizeof line);
      ui_text(table_end+6,2,line,sp->chan.dv.call_has_position ? A_NORMAL : A_DIM,UI_FRAME);
    }
    ui_rule(LINES-2,NULL);
  }
  // Side frames avoid the terminal's last column, which can cause scrolling.
  for(int r=1;r<LINES-2;r++){
    if(r == 3 || r == top || r == table_end) continue;
    ui_text(r,0,"|",A_NORMAL,UI_FRAME);
    ui_text(r,COLS-2,"|",A_NORMAL,UI_FRAME);
  }
  ui_text(LINES-1,0,COLS < 76 ? "Up/Down Select  m Mute  u Unmute  h Help  Q Quit" :
          " Up/Down Select | m/u Mute | M/U All | +/- Gain | v Details | h Help | Q Quit",A_REVERSE,UI_FRAME);
  curs_set(0);
}

static void update_monitor_display(void){
  if(!Verbose){
    dashboard();
    return;
  }
  curs_set(1);
  int row,col;
  getyx(stdscr,row,col);

  // First header line
  if(Repeater_tail != 0){
    if(Last_id_time != 0)
      printwt("Last ID: %.1lf sec",((double)(gps_time_ns() - Last_id_time) * 1e-9));
    if(PTT_state)
      addstrt(" PTT On");
    else if(Last_xmit_time != 0)
      printwt(" PTT Off; Last xmit: %.1lf sec",(double)(gps_time_ns() - Last_xmit_time) * 1e-9);
    col = 0; // next line
    move(++row,col);
  }

  if(Constant_delay)
    printwt("Constant delay ");

  if(Start_muted)
    printwt("**Starting new sessions muted** ");

  if(Voting)
    printwt("SNR Voting enabled");

  getyx(stdscr,row,col);
  if(col != 0){
    col = 0;
    move(++row,col);
  }
  int64_t rptr = atomic_load_explicit(&Output_time,memory_order_relaxed);
  if(Verbose){
    // Measure skew between sampling clock and UNIX real time (hopefully NTP synched)
    double const pa_seconds = Pa_GetStreamTime(Pa_Stream) - Start_pa_time;

    uint64_t audio_frames = atomic_load_explicit(&Audio_frames,memory_order_relaxed);
    double const rate = audio_frames / pa_seconds;

    printwt("%s playout %.0lf ms latency %5.1lf ms D/A %'.1lf Hz",
	    opus_get_version_string(),1000*Playout,1000*Portaudio_delay,rate);
    printwt(" %+5.0lf ppm",1e6 * (rate / DAC_samprate - 1));
    // Time since last packet drop on any channel
    printwt(" EFS %'.1lf",(1e-9*(gps_time_ns() - Last_error_time)));
    //    int64_t total = atomic_load(&Output_total);
    //    int64_t calls = atomic_load(&Callbacks);
    int quant = atomic_load_explicit(&Callback_quantum,memory_order_relaxed);
    float level = atomic_load_explicit(&Output_level,memory_order_relaxed);
    level = power2dB(level);
    printwt(" Clock %.1lfs %.1f dBFS CB N %u",(double)rptr/DAC_samprate,level,quant);
    extern int Session_creates;
    printwt(" sessions %d",Session_creates);
    printwt("\n");
    printwt("Waits %llu gap waits %llu Wait timeouts %llu\n",Waits,Gap_waits,Wait_timeout);
  }
  getyx(stdscr,row,col);
  if(col != 0){
    col = 0;
    move(++row,col);
  }
  // Show channel statuses
  int const header_line = row;

  Sessions_per_screen = max(1,LINES - getcury(stdscr) - 1);

  defragment_session();
  if(Auto_sort)
    sort_session_total();

  if(First_session >= NSESSIONS)
    First_session = NSESSIONS-1;
  while(First_session > 0 && !inuse(Sess_ptr[First_session]))
    First_session--;

  // Bound current pointer to active list area
  while(Current > 0 && !inuse(Sess_ptr[Current]))
    Current--; // Current session is no valid, back up

  int first_line = header_line+1; // after header line

  if(Current <= First_session)
    Current = First_session;
  else if(Current >= First_session + Sessions_per_screen)
    Current = First_session + Sessions_per_screen - 1;

  {  // dB gain
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    snprintf(scratch[i++],COLS,"dB");
    for(; i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp))
	break;

      snprintf(scratch[i],COLS,"%+.0lf",muted(sp) ? -INFINITY : voltage2dB(sp->gain));
    }
    col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  if(Auto_position){
    // Pan column
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    snprintf(scratch[i++],COLS,"pan");
    for(; i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp))
	break;

      snprintf(scratch[i],COLS,"%+ld",lrint(100*sp->pan));
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  // SSRC
  if(col >= COLS)
    goto done;

  {  // ssrc
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    snprintf(scratch[i++],COLS,"ssrc");
    for(; i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp))
	break;

      snprintf(scratch[i],COLS,"%u",sp->ssrc);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
   char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"tone/DCS/NAC");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(session_has_analog_audio(sp)){
        ui_tone(sp,scratch[i],COLS);
      } else if(sp->chan.demod_type == P25_DEMOD){
	// The call being followed or decoded, else the NAC
	if(sp->chan.dv.source[0] != '\0')
	  snprintf(scratch[i],COLS,"%s",sp->chan.dv.source);
	else if(sp->chan.p25.have_nac)
	  snprintf(scratch[i],COLS,"N%03X",sp->chan.p25.last_nac);
      } else if(sp->chan.demod_type == DMR_DEMOD || sp->chan.demod_type == M17_DEMOD || sp->chan.demod_type == YSF_DEMOD
		|| sp->chan.demod_type == DSTAR_DEMOD || sp->chan.demod_type == NXDN_DEMOD
		|| sp->chan.demod_type == DIGITAL_DEMOD || sp->chan.demod_type == LTR_DEMOD){
	snprintf(scratch[i],COLS,"%s",sp->chan.dv.source);
      } else if(sp->notch_enable && sp->current_tone != 0)
	snprintf(scratch[i],COLS,"%.1f%c",sp->notch_tone,sp->current_tone == sp->notch_tone ? '*' : ' ');
      else if(sp->notch_enable && sp->current_dcs != 0)
	snprintf(scratch[i],COLS,"%c%03d",sp->current_dcs_inverted ? 'I' : 'D',sp->current_dcs);
      else if(sp->notch_enable && sp->notch_tone != 0)
	snprintf(scratch[i],COLS,"%.1f ",sp->notch_tone);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {  // freq
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"freq");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;

      snprintf(scratch[i],COLS,"%'.0lf",sp->chan.tune.freq);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {  // mode
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"mode");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;

      snprintf(scratch[i],COLS,"%s",demod_name_from_type(sp->chan.demod_type));
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {  // snr
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"snr");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(!isfinite(sp->snr))
	continue;

      snprintf(scratch[i],COLS,"%.1f",sp->snr);
    }
    col++; col += render_right(header_line,col,scratch,i,5); // Don't keep changing when SNR fluctuates
  }
  if(col >= COLS)
    goto done;

  {
    // callsign
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"call");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;

      if(strlen(sp->id) > 0){
	snprintf(scratch[i],COLS,"%s",sp->callsign);
	enable = true;
      }
    }
    if(enable){
      col++; col += render_left(header_line,col,scratch,i);
    }
  }
  if(col >= COLS)
    goto done;
  {
    // id
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;
    bool repeaters = false;

    i++; // the heading depends on what's below it
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;

      if(strlen(sp->id) > 0){
	if(sp->talkgroup_named && sp->talkgroup_ended != 0){
	  // The last call heard, and how long ago it ended
	  int64_t const age = (gps_time_ns() - sp->talkgroup_ended) / BILLION;
	  if(age < 3600)
	    snprintf(scratch[i],COLS,"%s  %d:%02d ago",sp->id,(int)(age / 60),(int)(age % 60));
	  else
	    snprintf(scratch[i],COLS,"%s  %d:%02d:%02d ago",sp->id,(int)(age / 3600),(int)(age / 60 % 60),(int)(age % 60));
	} else
	  snprintf(scratch[i],COLS,"%s",sp->id);
	enable = true;
	if(!sp->talkgroup_named)
	  repeaters = true;
      }
    }
    snprintf(scratch[0],COLS,"%s",repeaters ? "repeaterbook.com data" : "talkgroup");
    if(enable){
      col++; col += render_left(header_line,col,scratch,i);
    }
  }
  if(col >= COLS)
    goto done;
  {
    // distance
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"km");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;

      if(sp->distance  > 0){
	snprintf(scratch[i],COLS,"%.1lf",0.001 * sp->distance);
	enable = true;
      }
    }
    if(enable){
      col++; col += render_right(header_line,col,scratch,i,0);
    }
  }
  if(col >= COLS)
    goto done;

  {  // total active time
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    snprintf(scratch[i++],COLS,"tot");
    for(; i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp))
	break;

      char total_buf[100] = {0};
      ftime(total_buf,sizeof(total_buf),lrint(sp->tot_active));
      snprintf(scratch[i],COLS,"%s",total_buf);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    snprintf(scratch[i++],COLS,"cur");
    for(; i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp))
	break;

      double t = sp->active != 0 ? sp->active : (double)(gps_time_ns() - sp->last_active) * 1e-9;
      char buf[100];
      snprintf(scratch[i],COLS,"%s",ftime(buf,sizeof buf,t));
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"level");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->level <= 1e-10) // -100 dB
	continue;
      float dB = power2dB(sp->level);
      snprintf(scratch[i],COLS,"%.1lf",dB);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"queue");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(qlen(sp) <= 0)
	continue; // blank inactive sessions
      snprintf(scratch[i],COLS,"%ld",lrint(1000. * qlen(sp)/DAC_samprate));
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    // Opus/pcm
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"type");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      snprintf(scratch[i],COLS,"%s",encoding_string(sp->pt_table[sp->type].encoding));
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  // frame size, ms
  {
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"ms");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->samprate == 0)
	continue;
      snprintf(scratch[i],COLS,"%d",1000 * sp->last_framesize/sp->samprate); // frame size, ms
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    // channels
    // Opus is special - the decoder always produces stereo, so the channel count is just informational
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"%s","ch");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->pt_table[sp->type].encoding == OPUS
	 || sp->pt_table[sp->type].encoding == OPUS_VOIP)
	snprintf(scratch[i],COLS,"%d",sp->opus_channels);
      else
	snprintf(scratch[i],COLS,"%d",sp->channels);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    // Audio bandwidth
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"bw");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->samprate == 0)
	continue;
      snprintf(scratch[i],COLS,"%d",sp->bandwidth/1000); // to kHz
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

 {  // RTP payload type
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"pt");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->samprate == 0)
	continue;
      snprintf(scratch[i],COLS,"%d",sp->type);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    // Data rate, kb/s
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"rate");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      snprintf(scratch[i],COLS,"%.*f", sp->datarate < 1e5 ? 1 : 0, .001 * sp->datarate); // decimal only if < 1000
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  if(Verbose){
    // Processing delay, assuming synchronized system clocks
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    snprintf(scratch[i++],COLS,"Delay");
    for(; i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp))
	break;

      if(sp->chan.output.rtp.timestamp == 0)
	continue;
      // sp->frontend.timestamp (GPS time at front end) and sp->chan.output.rtp.timestamp (next RTP timestamp to be sent) are updated periodically by status packets
      // sp->last_timestamp contains most recent RTP packet processed
      // This needs further thought and cleanup
      if(gps_time_ns() >  sp->last_active + BILLION/2)
	continue;

      float delay = (float)(int32_t)(sp->chan.output.rtp.timestamp - sp->last_timestamp) / sp->samprate;
      delay += 1.0e-9f * (gps_time_ns() - sp->chan.clocktime);
      snprintf(scratch[i],COLS,"%'.3lf", delay);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
    if(col >= COLS)
      goto done;
  }
  if(Verbose){
    // T0: DAC clock time (ms) at which the RTP timestamp was 0
    // At 48 khz, the 32-bit RTP timestamp wraps in 89478.485 sec or 1.0356306 days
    // should be invariant +/- 1 frame (20 ms) for a given stream unless monitor or radiod is restarted
    // Checks for failures to update timestamp at sender, etc
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    snprintf(scratch[i++],COLS,"T0");
    for(; i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp))
	break;

      if(sp->samprate == 0)
	continue;

      int64_t wptr = atomic_load_explicit(&sp->wptr,memory_order_relaxed);
      int64_t t0 = 1000*wptr/DAC_samprate - 1000*(int64_t)sp->next_timestamp / sp->samprate;
      snprintf(scratch[i],COLS,"%'lld",(long long)t0); // ms
    }
    col++; col += render_right(header_line,col,scratch,i,0);
    if(col >= COLS)
      goto done;
  }
  {
    // Packet count
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"pkt");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      snprintf(scratch[i],COLS,"%llu",(unsigned long long)sp->packets);
    }
    col++; col += render_right(header_line,col,scratch,i,0);
  }
  if(col >= COLS)
    goto done;

  {
    // RTP resets
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"rst");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->resets > 0){
	enable = true;
	snprintf(scratch[i],COLS,"%llu",(unsigned long long)sp->resets);
      }
    }
    if(enable){
      col++; col += render_right(header_line,col,scratch,i,0);
    }
  }
  if(col >= COLS)
    goto done;

  {
    // RTP packet drops
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"drp");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->drops > 0){
	enable = true;
	snprintf(scratch[i],COLS,"%llu",(unsigned long long)sp->drops);
      }
    }
    if(enable){
      col++; col += render_right(header_line,col,scratch,i,0);
    }
  }
  if(col >= COLS)
    goto done;

  {
    // Late RTP packets
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"late");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->lates > 0){
	enable = true;
	snprintf(scratch[i],COLS,"%llu",(unsigned long long)sp->lates);
      }
    }
    if(enable){
      col++; col += render_right(header_line,col,scratch,i,0);
    }
  }
  if(col >= COLS)
    goto done;

  {
    // early RTP packets
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"early");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->earlies > 0){
	enable = true;
	snprintf(scratch[i],COLS,"%llu",(unsigned long long)sp->earlies);
      }
    }
    if(enable){
      col++; col += render_right(header_line,col,scratch,i,0);
    }
  }
  if(col >= COLS)
    goto done;

  {
    // resequenced RTP packets
    char scratch [LINES][COLS];
    memset(scratch, 0 , sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"resq");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->reseqs > 0){
	enable = true;
	snprintf(scratch[i],COLS,"%llu",(unsigned long long)sp->reseqs);
      }
    }
    if(enable){
      col++; col += render_right(header_line,col,scratch,i,0);
    }
  }
  if(col >= COLS)
    goto done;

  {
    // opus packet loss concealment events
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;
    bool enable = false;

    snprintf(scratch[i++],COLS,"plc");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      if(sp->plcs > 0){
	enable = true;
	snprintf(scratch[i],COLS,"%llu",(unsigned long long)sp->plcs);
      }
    }
    if(enable){
      col++; col += render_right(header_line,col,scratch,i,0);
    }
  }
  if(col >= COLS)
    goto done;

  {
    // sockets
    char scratch [LINES][COLS];
    memset(scratch,0,sizeof scratch);
    int session = First_session;
    int i = 0;

    snprintf(scratch[i++],COLS,"sockets");
    for(;i < LINES && session < NSESSIONS; i++,session++){
      sess_t const *sp = Sess_ptr[session];
      if(!inuse(sp)) break;
      snprintf(scratch[i],COLS,"%s -> %s",formatsock(&sp->sender,true),sp->dest);
    }
    col++; col += render_left(header_line,col,scratch,i);
  }
  if(col >= COLS)
    goto done;


 done:;
  // Embolden the active lines
  row = header_line + 1;
  attr_t attrs = 0;
  short pair = 0;
  attr_get(&attrs, &pair, NULL);
  for(int session = First_session; session < NSESSIONS && row < LINES-1; session++,row++){
    sess_t const *sp = Sess_ptr[session];
    if(!inuse(sp))
      break;

    attr_t attr = A_NORMAL;
    if(gps_time_ns() - sp->last_active < BILLION/2){
      // active within the past 500 ms
      attr |= A_BOLD;
    }
    if(Voting && sp == Best_session)
      attr |= A_UNDERLINE;


    // 1 adjusts for the titles
    // only underscore to just before the socket entry since it's variable length
   int r =  mvchgat(row,0,min(col,COLS-1),attr,pair,NULL);
   (void)r;
   assert(r == OK);
  }
  move(min(LINES-1,first_line + Current - First_session),0); // Cursor on current line
  // End of display writing
}
static void process_keyboard(void){

  int const c = getch(); // Waits for 'update interval' ms if no input
  if(c == EOF)
    return; // No key hit

  // These commands don't require session locking
  bool serviced = true;
  switch(c){
  case 'Q': // quit program
    atomic_store_explicit(&Terminate,true,memory_order_release);
    break;
  case 'v':
    Verbose = !Verbose;
    break;
  case 'C':
    Constant_delay = !Constant_delay;
    break;
  case 'S': // the receiver summary, on or off
    Show_summary = !Show_summary;
    break;
  case 'A': // Start all new sessions muted
    Start_muted = !Start_muted;
    break;
  case 'q':
    Quiet_mode = !Quiet_mode;
    break;
  case '\f':  // Screen repaint (formfeed, aka control-L)
    clearok(curscr,TRUE);
    break;
  case 'h': // Help screen
    help = !help;
    break;
  case 's': // Sort sessions by most recently active (or longest active)
    sort_session_active(); // locks Sess_mutex internally
    break;
  case 't': // Sort sessions by most total activity
    sort_session_total();
    break;
  case 'T':
    Auto_sort = !Auto_sort;
    break;
  case KEY_RESIZE:
  case EOF:
    break;
  default:
    serviced = false; // Not handled in this switch(), so fall through and handle in the next with the lock taken
    break;
  }
  if(serviced)
    return;

  serviced = true;


  // Commands manipulating the current session index
  switch(c){
  case KEY_NPAGE:
    if(First_session + Sessions_per_screen < NSESSIONS){
      First_session += Sessions_per_screen;
      Current = First_session;
    }
    break;
  case KEY_PPAGE:
    First_session = max(0,First_session-Sessions_per_screen);
    Current = First_session;
    break;
  case KEY_HOME: // first session
    First_session = Current = 0;
    break;
  case KEY_END: // last session
    {
      while(Current < NSESSIONS-1 && inuse(Sess_ptr[Current]))
	Current++;
      if(Current > First_session + Sessions_per_screen)
	First_session = Current - Sessions_per_screen;
    }
    First_session = max(0,NSESSIONS - Sessions_per_screen);
    break;
  case '\t':
  case KEY_DOWN:
    if(Current >= NSESSIONS-1 || !inuse(Sess_ptr[Current+1]))
      break;
    Current++;
    if(Current >= First_session + Sessions_per_screen)
      First_session++;
    break;
  case KEY_BTAB:
  case KEY_UP:
    if(Current > 0)
      Current--;
    if(Current < First_session)
      First_session--;
    break;
  default:
    serviced = false;
  }
  if(serviced)
    return;


  // Commands operating on all sessions
  // These *probably* don't require locking?
  serviced = true;
  switch(c){
  case 'U': // Unmute all sessions, resetting any that were muted
    for(int i = 0; i < NSESSIONS; i++){
      sess_t *sp = Sess_ptr[i];
      if(inuse(sp) && muted(sp)){
	sp->restart = true; // restart playout
	sp->muted = false;
      }
    }
    break;
  case 'M': // Mute all sessions
    for(int i = 0; i < NSESSIONS; i++){
      sess_t *sp = Sess_ptr[i];
      if(inuse(sp))
	atomic_store_explicit(&sp->muted,true,memory_order_release);
    }
    break;
  case 'N':
    Notch = true;
    for(int i=0; i < NSESSIONS; i++){
      sess_t *sp = Sess_ptr[i];
      if(inuse(sp)){
	sp->notch_enable = true;
      }
    }
    break;
  case 'R': // Reset all sessions
    for(int i=0; i < NSESSIONS;i++){
      sess_t *sp = Sess_ptr[i];
      if(inuse(sp))
	sp->restart = true;
    }
    break;
  case 'F':
    Notch = false;
    for(int i=0; i < NSESSIONS; i++){
      sess_t *sp = Sess_ptr[i];
      if(inuse(sp))
	sp->notch_enable = false;
    }
    break;
  default:
    serviced = false; // Not handled by this switch
    break;
  }
  if(serviced)
    return;


  // Commands operating on the current session
  // Check validity of current session pointer so individual cases don't have to
  // Do this last
  serviced = true;

  sess_t *sp = Sess_ptr[Current];
  if(!inuse(sp)){
    // Current index not valid
    beep();
    return;
  }

  switch(c){
  case 'f': // Turn off tone notching
    sp->notch_enable = false;
    break;
  case 'n':
    Notch = true;
    sp->notch_enable = true;
    break;
  case '=': // If the user doesn't hit the shift key (on a US keyboard) take it as a '+'
  case '+':
    sp->gain *= 1.122018454; // +1 dB
    break;
  case '_': // Underscore is shifted minus
  case '-':
    sp->gain /= 1.122018454; // -1 dB
    break;
  case KEY_LEFT:
    sp->pan = max(sp->pan - .01,-1.0);
    break;
  case KEY_RIGHT:
    sp->pan = min(sp->pan + .01,+1.0);
    break;
  case KEY_SLEFT: // Shifted left - decrease playout buffer 1 ms
    if(Playout >= -0.1){
      Playout -= .001;
    }
    break;
  case KEY_SRIGHT: // Shifted right - increase playout buffer 1 ms
    Playout += .001;
    break;
  case 'u': // Unmute and reset Current session
    atomic_store_explicit(&sp->muted,false,memory_order_release);
    break;
  case 'm': // Mute Current session
    atomic_store_explicit(&sp->muted,true,memory_order_release);
    break;
  case 'r':    // Manually reset playout queue
    sp->restart = true;
    break;
  case KEY_DC: // Delete
  case KEY_BACKSPACE:
  case 'd': // Delete current session
    close_session(sp);
    break;
  default:
    serviced = false;
    break;
  }
  if(!serviced)
    beep(); // Not serviced by anything
}

// Render a block with all lines right justified
// These routines keep columns of numbers nicely aligned without having to pre-reserve space for the widest possible entry
// and without the user having to specify field widths
static int render_right(int const row, int const col, char scratch[LINES][COLS],int const nrows,int width){
  if(COLS == 0)
    return 0; // pathological case
  // Ensure each line is terminated on the right
  for(int r = 0; r < nrows; r++)
    scratch[r][COLS-1] = 0;

  // Find leftmost occupied column
  // Usually 0 but don't assume that
  int first_column = COLS;
  for(int c = 0; c < COLS; c++){
    for(int r = 0; r < nrows; r++){
      if(isgraph((unsigned char)scratch[r][c])){
	first_column = c;
	goto foundit;
      }
    }
  }
  return 0; // nothing to render
 foundit:;
  // Find longest line; determines block width
  for(int r = 0; r < nrows; r++){
    int len = strlen(&scratch[r][first_column]);
    if(len > width)
      width = len;
  }
  if(width > COLS)
    width = COLS; // keep it sane
  for(int r = 0; r < nrows && row+r < LINES-1; r++){
    int len = strlen(&scratch[r][first_column]);
    int pad = (width > len) ? width - len : 0; // if shorter than width, pad on left to right justify
    int x = col + pad;
    int room = COLS - x;
    if(len > room)
      len = room;  // Too wide, clip on right

    if(len > 0) // could be < 0 if col + pad > COLS
      mvaddnstr(row+r, x, &scratch[r][first_column], len);
  }
  return width;
}
// Render a block with left justification
// Simpler than right justification, just have to keep track of the widest row
static int render_left(int const row, int const col, char scratch[LINES][COLS],int const nrows){
  int maxline = 0;
  int room = COLS - col;
  if(room <= 0)
    return 0;

  for(int r = 0; r < nrows && row+r < LINES-1; r++){
    scratch[r][COLS-1] = 0; // ensure each line is null terminated
    int len = strlen(scratch[r]);
    if(len > room)
      len = room;
    if(maxline < len)
      maxline = len;
    if(len > 0)
      mvaddnstr(row+r,col,scratch[r],len);
  }
  return maxline;
}
