// The DMR fixture contains three key-ups with identical source/destination.
// Decode the whole capture in one block so polling after dv_process cannot
// possibly recover the boundaries: each PCM callback must retain them.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dv.h"

static dv_decoder *Decoder;
static unsigned Frames,Calls;
static uint64_t Serial;

static void check(bool ok,char const *message){
  if(!ok){
    fprintf(stderr,"FAIL: %s\n",message);
    exit(EXIT_FAILURE);
  }
}

static void receive(void *user,int16_t const pcm[DV_PCM_SAMPLES]){
  (void)user; (void)pcm;
  dv_stats st;
  dv_get_stats(Decoder,&st);
  check(strcmp(st.src_call,"4477055") == 0 && strcmp(st.dst_call,"TG 44772") == 0,
        "PCM callback did not retain its caller IDs");
  if(Frames == 0 || st.call_serial != Serial){
    Calls++;
    Serial = st.call_serial;
  }
  Frames++;
}

int main(void){
  FILE *fp = fopen("data/voice-dmr.c64","rb");
  check(fp != NULL,"open DMR fixture");
  check(fseek(fp,0,SEEK_END) == 0,"seek DMR fixture");
  long bytes = ftell(fp);
  check(bytes > 0 && bytes % (2 * sizeof(float)) == 0,"fixture IQ size");
  rewind(fp);
  float *iq = malloc((size_t)bytes);
  check(iq != NULL && fread(iq,1,(size_t)bytes,fp) == (size_t)bytes,"read DMR fixture");
  fclose(fp);
  dv_mode modes[] = {DV_DMR,DV_AUTO};
  for(unsigned i=0;i<sizeof modes / sizeof modes[0];i++){
    dv_config config = {0};
    Frames = Calls = 0;
    Decoder = dv_create(modes[i],&config,receive,NULL);
    check(Decoder != NULL,"create decoder");
    dv_process(Decoder,iq,(size_t)bytes / (2 * sizeof(float)));
    check(Frames == 108 && Calls == 3,"three same-talker key-ups must retain distinct call serials");
    dv_stats before,after;
    dv_get_stats(Decoder,&before);
    dv_signal_lost(Decoder);
    dv_get_stats(Decoder,&after);
    if(modes[i] == DV_DMR)
      check(before.call_serial != after.call_serial,"signal loss advances call serial");
    dv_destroy(Decoder);
  }
  free(iq);
  puts("Recorded same-talker call boundary tests passed");
  return EXIT_SUCCESS;
}
