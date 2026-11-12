// IQ file playback: a pseudo front end that feeds radiod a recorded
// complex float (cf32 little endian, interleaved I/Q) file in real time,
// e.g. for repeatable end-to-end tests of demodulators and recorders.
//
//   [global]
//   hardware = playback
//
//   [playback]
//   device = iq_file
//   file = capture.c64        ; cf32 samples
//   samprate = 48k            ; the file's sample rate
//   frequency = 851m9125      ; the frequency at the file's centre
//   loop = no                 ; at the end: start over (yes), or feed zeros (no)
//   gain = 0                  ; dB applied to the samples
//
// Copyright 2026 ka9q-radio contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include <assert.h>
#include <complex.h>
#include <errno.h>
#include <iniparser/iniparser.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#include "misc.h"
#include "config.h"
#include "radio.h"
#include "timing.h"

static char const *Iq_file_keys[] = {
  "device",
  "library",
  "file",
  "samprate",
  "frequency",
  "loop",
  "gain",
  "description",
  NULL
};

struct iq_file {
  struct frontend *frontend;
  FILE *fp;
  char *path;
  bool loop;
  float gain;
  pthread_t thread;
  atomic_bool run;
  bool ended;      // the file has been played out (loop = no)
};

int iq_file_setup(struct frontend * const frontend, dictionary const * const dictionary, char const * const section){
  assert(dictionary != NULL);
  char const * const device = config_getstring(dictionary,section,"device",section);
  if(strcasecmp(device,"iq_file") != 0)
    return -1; // Not for us
  config_validate_section(stderr,dictionary,section,Iq_file_keys,NULL);

  struct iq_file * const f = calloc(1,sizeof *f);
  assert(f != NULL);
  f->frontend = frontend;
  frontend->context = f;

  char const * const path = config_getstring(dictionary,section,"file",NULL);
  if(path == NULL){
    fprintf(stderr,"%s: no file given\n",section);
    return -1;
  }
  f->path = strdup(path);
  f->fp = fopen(path,"rb");
  if(f->fp == NULL){
    fprintf(stderr,"%s: can't open %s: %s\n",section,path,strerror(errno));
    return -1;
  }
  frontend->samprate = 48000;
  {
    char const *p = config_getstring(dictionary,section,"samprate",NULL);
    if(p != NULL)
      frontend->samprate = (int)parse_frequency(p,false);
  }
  frontend->frequency = 0;
  {
    char const *p = config_getstring(dictionary,section,"frequency",NULL);
    if(p != NULL)
      frontend->frequency = parse_frequency(p,false);
  }
  f->loop = config_getboolean(dictionary,section,"loop",false);
  f->gain = dB2voltage(config_getfloat(dictionary,section,"gain",0));
  frontend->isreal = false;
  frontend->bitspersample = 1; // floating point, no scaling
  frontend->min_IF = -0.5 * frontend->samprate;
  frontend->max_IF = +0.5 * frontend->samprate;
  frontend->lock = true; // can't be retuned
  frontend->rf_gain = NAN;
  frontend->rf_atten = NAN;
  frontend->rf_level_cal = NAN;
  strlcpy(frontend->description,config_getstring(dictionary,section,"description",path),sizeof frontend->description);
  fprintf(stderr,"IQ file %s, samprate %'lf, centre %'.3lf Hz%s\n",path,frontend->samprate,frontend->frequency,
	  f->loop ? ", looping" : "");
  return 0;
}

// Read up to n samples, looping or padding with zeros at the end
static void read_samples(struct iq_file *f,float complex *out,long n){
  long got = 0;
  while(got < n){
    size_t const r = f->ended ? 0 : fread(out + got,sizeof *out,(size_t)(n - got),f->fp);
    got += (long)r;
    if(got == n)
      break;
    if(!f->ended && f->loop){
      rewind(f->fp);
      continue;
    }
    if(!f->ended){
      f->ended = true;
      fprintf(stderr,"IQ file %s: end of file\n",f->path);
    }
    memset(out + got,0,(size_t)(n - got) * sizeof *out);
    got = n;
  }
}

static void *iq_file_thread(void *arg){
  pthread_setname("iq_file");
  struct iq_file * const f = arg;
  struct frontend * const frontend = f->frontend;
  float const scale = f->gain;
  long const max_block = lrint(1.5 * Blocktime * frontend->samprate);
  int64_t timesnap = gps_time_ns() - lrint(Blocktime * BILLION);
  while(atomic_load(&f->run)){
    // Real time: as many samples as the clock says are due
    int64_t const now = gps_time_ns();
    long block = lrint((double)(now - timesnap) * frontend->samprate / BILLION);
    if(block > max_block)
      block = max_block;
    timesnap += llrint((double)BILLION * block / frontend->samprate);
    if(block > 0){
      float complex * const wptr = frontend->in.input_write_pointer.c;
      read_samples(f,wptr,block);
      float energy = 0;
      for(long i = 0; i < block; i++){
	wptr[i] *= scale;
	energy += crealf(wptr[i] * conjf(wptr[i]));
      }
      write_cfilter(&frontend->in,NULL,(int)block); // advance the write pointer, run the FFT
      frontend->samples += block;
      if(isfinite(energy))
	frontend->if_power += 0.01f * (energy / block - frontend->if_power);
    }
    struct timespec const ts = { .tv_sec = 0, .tv_nsec = BILLION/200 };
    nanosleep(&ts,NULL);
  }
  return NULL;
}

int iq_file_startup(struct frontend * const frontend){
  struct iq_file * const f = frontend->context;
  atomic_store(&f->run,true);
  pthread_create(&f->thread,NULL,iq_file_thread,f);
  fprintf(stderr,"IQ file playback running\n");
  return 0;
}

int iq_file_shutdown(struct frontend * const frontend){
  struct iq_file * const f = frontend->context;
  atomic_store(&f->run,false);
  pthread_join(f->thread,NULL);
  fclose(f->fp);
  return 0;
}

double iq_file_tune(struct frontend * const frontend,double const freq){
  (void)freq;
  return frontend->frequency; // fixed: it's a recording
}
