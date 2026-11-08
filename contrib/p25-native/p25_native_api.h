#ifndef KA9Q_P25_NATIVE_API_H
#define KA9Q_P25_NATIVE_API_H

#include "phase1.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    P25_NATIVE_C4FM = 0,
    P25_NATIVE_CQPSK = 1
} p25_native_modulation;

typedef struct p25_native_decoder p25_native_decoder;

int p25_native_decoder_create(p25_native_modulation modulation,
                              p25_phase1_pcm_callback pcm_callback,
                              void *user_data,
                              p25_native_decoder **decoder);
void p25_native_decoder_destroy(p25_native_decoder *decoder);
int p25_native_decoder_reset(p25_native_decoder *decoder);
int p25_native_decoder_set_expected_nac(p25_native_decoder *decoder,
                                        uint16_t nac);
int p25_native_decoder_process_f32(p25_native_decoder *decoder,
                                   const float *samples, size_t count);
int p25_native_decoder_process_s16be(p25_native_decoder *decoder,
                                     const uint8_t *samples, size_t count);
int p25_native_decoder_get_stats(const p25_native_decoder *decoder,
                                 uint64_t *complex_samples, uint64_t *symbols,
                                 p25_phase1_stats *phase1_stats);
int p25_native_decoder_get_sync_count(const p25_native_decoder *decoder,
                                      uint64_t *sync_count);
int p25_native_decoder_get_frequency_offset(const p25_native_decoder *decoder,
                                            float *offset_hz);

#ifdef __cplusplus
}
#endif

#endif
