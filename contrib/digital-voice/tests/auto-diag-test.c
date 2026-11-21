// DV_AUTO reports the DMR receiver's diagnostic counters (lc=, data=)
// whatever mode is playing: data alone never makes DMR the mode playing.
// The DMR recording through DV_AUTO, then silence: once nothing plays, the
// counts of its headers and terminators are still there.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <stdio.h>
#include <string.h>
#include "dv.h"

static void pcm(void *u,int16_t const *p){ (void)u; (void)p; }
int main(int argc,char **argv){
  if(argc < 2)
    return 2;
  dv_config cfg;
  memset(&cfg,0,sizeof cfg);
  dv_decoder *d = dv_create(DV_AUTO,&cfg,pcm,NULL);
  FILE *in = fopen(argv[1],"rb");
  if(d == NULL || in == NULL)
    return 2;
  float buf[2 * 960];
  size_t n;
  while((n = fread(buf,2 * sizeof buf[0],960,in)) > 0)
    dv_process(d,buf,n);
  fclose(in);
  memset(buf,0,sizeof buf);
  for(int i = 0; i < 250; i++) // 5 s of nothing
    dv_process(d,buf,960);
  dv_stats st;
  dv_get_stats(d,&st);
  int const ok = st.active_mode < 0 && st.dmr_burst_types[1] > 0 && st.dmr_burst_types[2] > 0 && st.dmr_lc_flcos != 0;
  printf("%s DV_AUTO with nothing playing: the DMR receiver's counts (headers %u, terminators %u, lc %llx)\n",
         ok ? "ok  " : "FAIL",st.dmr_burst_types[1],st.dmr_burst_types[2],(unsigned long long)st.dmr_lc_flcos);
  dv_destroy(d);
  puts(ok ? "Automatic mode diagnostics tests passed" : "Automatic mode diagnostics tests FAILED");
  return !ok;
}
