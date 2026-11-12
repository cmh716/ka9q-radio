// dvtest: run one decoder over a 48 kHz cf32 IQ file and print what a
// listener or recorder would see: each change of the call (mode, slot, IDs,
// encryption), control channel events, and closing counts. Optionally
// inserts silence or a signal loss to test call endings.
//
//   dvtest -m mode [options] file.c64
//     -m mode     dv_mode number (0 P25 C4FM, 1 CQPSK, 2 DMR, 3 M17, 4 YSF,
//                 5 D-STAR, 6 NXDN48, 7 NXDN96, 8 auto, 9 P25 Phase 2)
//     -s slot     DMR slot (1, 2; 0 = either) or Phase 2 slot (0, 1)
//     -n nac -w wacn -y sysid   P25 NAC; Phase 2 WACN and system ID (hex)
//     -z t:len    feed len seconds of zeros at t seconds
//     -l t        call dv_signal_lost() at t seconds
//     -N seconds  instead of a file, that much Gaussian noise (seeded)
//     -o out.s16  write the speech (8 kHz s16le)
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "dv.h"

static FILE *Out;
static uint64_t Frames;
static double Now;

static void pcm(void *user, int16_t const p[DV_PCM_SAMPLES])
{
  (void)user;
  Frames++;
  if (Out != NULL)
    fwrite(p, sizeof p[0], DV_PCM_SAMPLES, Out);
}

static void event(void *user, dv_event const *e)
{
  (void)user;
  if (e->text[0] != '\0')
    printf("%7.2f event %s\n", Now, e->text);
}

// Reproducible noise (xorshift + Box-Muller): the same on every machine
static uint64_t Rng = 0x9e3779b97f4a7c15ULL;
static double uniform(void)
{
  Rng ^= Rng << 13; Rng ^= Rng >> 7; Rng ^= Rng << 17;
  return ((Rng >> 11) + 0.5) * 0x1p-53;
}
static float gauss(void)
{
  return (float)(sqrt(-2 * log(uniform())) * cos(2 * M_PI * uniform()));
}

int main(int argc, char **argv)
{
  dv_config cfg = {0};
  int mode = -1;
  double zero_at = -1, zero_len = 0, lost_at = -1, noise = 0;
  int c;
  while ((c = getopt(argc, argv, "m:s:n:w:y:z:l:N:o:")) != -1) {
    switch (c) {
    case 'm': mode = atoi(optarg); break;
    case 's': cfg.dmr_slot = cfg.p25_slot = atoi(optarg); break;
    case 'n': cfg.p25_nac = (uint16_t)strtoul(optarg, NULL, 16); break;
    case 'w': cfg.p25_wacn = (uint32_t)strtoul(optarg, NULL, 16); break;
    case 'y': cfg.p25_sysid = (uint16_t)strtoul(optarg, NULL, 16); break;
    case 'z': sscanf(optarg, "%lf:%lf", &zero_at, &zero_len); break;
    case 'l': lost_at = atof(optarg); break;
    case 'N': noise = atof(optarg); break;
    case 'o': Out = fopen(optarg, "wb"); break;
    default: return 2;
    }
  }
  if (mode < 0 || (noise == 0 && optind >= argc)) {
    fprintf(stderr, "usage: %s -m mode [-s slot] [-n nac] [-w wacn] [-y sysid] [-z t:len] [-l t] [-N secs] [-o out] file.c64\n", argv[0]);
    return 2;
  }
  if (mode == DV_DMR)
    cfg.p25_slot = 0;
  else
    cfg.dmr_slot = 0;
  FILE *in = noise > 0 ? NULL : fopen(argv[optind], "rb");
  if (noise == 0 && in == NULL) {
    perror(argv[optind]);
    return 2;
  }
  dv_decoder *d = dv_create((dv_mode)mode, &cfg, pcm, NULL);
  if (d == NULL) {
    fprintf(stderr, "bad mode or config\n");
    return 2;
  }
  dv_set_event_callback(d, event, NULL);

  enum { BLOCK = 960 }; // 20 ms
  float buf[2 * BLOCK];
  uint64_t samples = 0, noise_samples = (uint64_t)(noise * DV_IQ_SAMPRATE);
  bool lost_done = false;
  uint64_t zeros_left = 0;
  char last[160] = "";
  for (;;) {
    size_t n;
    if (zeros_left > 0) {
      n = zeros_left > BLOCK ? BLOCK : zeros_left;
      memset(buf, 0, sizeof buf);
      zeros_left -= n;
    } else if (in == NULL) {
      if (samples >= noise_samples)
        break;
      n = BLOCK;
      for (size_t i = 0; i < 2 * n; i++)
        buf[i] = 0.1f * gauss();
    } else {
      n = fread(buf, 2 * sizeof buf[0], BLOCK, in);
      if (n == 0)
        break;
    }
    dv_process(d, buf, n);
    samples += n;
    Now = (double)samples / DV_IQ_SAMPRATE;
    if (zero_at >= 0 && Now >= zero_at) {
      zeros_left = (uint64_t)(zero_len * DV_IQ_SAMPRATE);
      zero_at = -1;
    }
    if (!lost_done && lost_at >= 0 && Now >= lost_at) {
      dv_signal_lost(d);
      lost_done = true;
      printf("%7.2f signal lost\n", Now);
    }
    dv_stats s;
    dv_get_stats(d, &s);
    char now[160];
    snprintf(now, sizeof now, "mode %s slot %d src %s dst %s%s",
             mode != DV_AUTO ? dv_mode_name(mode) : s.active_mode >= 0 ? dv_mode_name(s.active_mode) : "-",
             s.active_slot, s.src_call[0] ? s.src_call : "-", s.dst_call[0] ? s.dst_call : "-",
             s.m17_encrypted ? " encrypted" : "");
    if (strcmp(now, last) != 0) {
      printf("%7.2f %s\n", Now, now);
      strcpy(last, now);
    }
  }
  dv_stats s;
  dv_get_stats(d, &s);
  printf("end: syncs %llu frames %llu voice %llu muted %llu played %llu tsbk %llu/%llu",
         (unsigned long long)s.syncs, (unsigned long long)s.frames,
         (unsigned long long)s.voice_decoded, (unsigned long long)s.voice_muted,
         (unsigned long long)Frames, (unsigned long long)s.tsbks,
         (unsigned long long)(s.tsbks + s.tsbk_errors));
  if (s.wacn != 0)
    printf(" wacn %05X sys %03X rfss %u site %u", s.wacn, s.sysid, s.rfss, s.site);
  printf("\n");
  dv_destroy(d);
  if (Out != NULL)
    fclose(Out);
  if (in != NULL)
    fclose(in);
  return 0;
}
