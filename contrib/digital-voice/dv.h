// Digital voice receivers (P25 Phase 1, DMR, M17): 48 kHz complex IQ in, 8 kHz speech out
// Adapted from OP25; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DV_H
#define KA9Q_DV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DV_IQ_SAMPRATE 48000
#define DV_PCM_SAMPRATE 8000
#define DV_PCM_SAMPLES 160   // One vocoder frame, 20 ms

typedef enum {
  DV_P25_C4FM = 0,
  DV_P25_CQPSK = 1,  // P25 simulcast (LSM/CQPSK)
  DV_DMR = 2,
  DV_M17 = 3,
} dv_mode;

typedef struct {
  uint16_t p25_nac;  // P25: NAC to accept, 0 = any
  int dmr_slot;      // DMR: timeslot 1 or 2; 0 = whichever starts a call first
  bool m17_can_filter; // M17: play only transmissions with this CAN
  int m17_can;       // M17: Channel Access Number 0-15, if m17_can_filter
} dv_config;

// Called synchronously from dv_process() for each 20 ms frame of speech
typedef void (*dv_pcm_callback)(void *user_data, int16_t const pcm[DV_PCM_SAMPLES]);

typedef struct {
  uint64_t samples;          // complex input samples
  uint64_t symbols;
  uint64_t syncs;            // frame sync words seen
  uint64_t frames;           // frames decoded (P25: passed NID/NAC check; DMR: bursts)
  uint64_t voice_decoded;    // vocoder frames played
  uint64_t voice_muted;      // vocoder frames muted (P25: encrypted; DMR: other slot or too many errors)
  float freq_offset;         // Hz, estimated carrier offset
  // P25
  uint64_t ldu1, ldu2;
  uint16_t last_nac;
  uint32_t last_bch_errors;
  // DMR
  int active_slot;           // slot being played, 1 or 2; 0 = none
  float error_rate;          // DMR: AMBE bit error rate estimate of the active slot; M17: Viterbi
  // M17, from the current transmission's link setup frame
  char m17_src[12];          // source callsign, "" if not yet known
  char m17_dst[12];          // destination callsign
  int m17_can;               // Channel Access Number, -1 if not yet known
  bool m17_encrypted;
} dv_stats;

typedef struct dv_decoder dv_decoder;

// Returns NULL on failure (bad mode or config)
dv_decoder *dv_create(dv_mode mode, dv_config const *config,
                      dv_pcm_callback callback, void *user_data);
void dv_destroy(dv_decoder *decoder);

// iq: count interleaved I/Q float pairs at DV_IQ_SAMPRATE
// (a C 'float complex' array may be passed directly)
void dv_process(dv_decoder *decoder, float const *iq, size_t count);
void dv_get_stats(dv_decoder const *decoder, dv_stats *stats);

#ifdef __cplusplus
}
#endif

#endif
