// P25 Phase 1 receiver: 48 kHz complex IQ in, 8 kHz IMBE speech out
// Adapted from OP25; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_P25_NATIVE_H
#define KA9Q_P25_NATIVE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define P25_NATIVE_IQ_SAMPRATE 48000
#define P25_NATIVE_PCM_SAMPRATE 8000
#define P25_NATIVE_PCM_SAMPLES 160   // One IMBE frame, 20 ms

typedef enum {
  P25_NATIVE_C4FM = 0,
  P25_NATIVE_CQPSK = 1,  // Simulcast LSM/CQPSK
} p25_native_modulation;

// Called synchronously from p25_native_process() for each clear voice frame
typedef void (*p25_native_pcm_callback)(void *user_data,
                                        int16_t const pcm[P25_NATIVE_PCM_SAMPLES]);

typedef struct {
  uint64_t samples;          // complex input samples
  uint64_t symbols;
  uint64_t syncs;            // frame sync words seen
  uint64_t frames;           // frames passing NID check (and NAC filter)
  uint64_t ldu1, ldu2;
  uint64_t voice_decoded;    // IMBE frames decoded
  uint64_t voice_encrypted;  // IMBE frames muted because the call is encrypted
  uint16_t last_nac;
  uint32_t last_bch_errors;
  float freq_offset;         // Hz, estimated carrier offset
} p25_native_stats;

typedef struct p25_native_decoder p25_native_decoder;

// nac == 0 accepts any NAC. Returns NULL on failure
p25_native_decoder *p25_native_create(p25_native_modulation modulation, uint16_t nac,
                                      p25_native_pcm_callback callback, void *user_data);
void p25_native_destroy(p25_native_decoder *decoder);

// iq: count interleaved I/Q float pairs at P25_NATIVE_IQ_SAMPRATE
// (a C 'float complex' array may be passed directly)
void p25_native_process(p25_native_decoder *decoder, float const *iq, size_t count);
void p25_native_get_stats(p25_native_decoder const *decoder, p25_native_stats *stats);

#ifdef __cplusplus
}
#endif

#endif
