#ifndef KA9Q_P25_PHASE1_H
#define KA9Q_P25_PHASE1_H

#include "imbe.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct p25_phase1_decoder p25_phase1_decoder;

typedef int (*p25_phase1_pcm_callback)(
    void *user_data,
    uint16_t nac,
    uint8_t duid,
    uint32_t bch_errors,
    uint32_t imbe_corrected_errors,
    const int16_t pcm[P25_IMBE_PCM_SAMPLES]);

typedef struct {
    uint64_t dibits_received;
    uint64_t complete_frames;
    uint64_t ldu1_frames;
    uint64_t ldu2_frames;
    uint64_t voice_codewords_decoded;
    uint64_t voice_codewords_skipped_unknown_encryption;
    uint64_t voice_codewords_skipped_encrypted;
    uint64_t imbe_corrected_errors;
    uint16_t last_nac;
    uint8_t last_duid;
    uint32_t last_bch_errors;
    uint8_t encryption_known;
    uint8_t encrypted;
} p25_phase1_stats;

enum {
    P25_PHASE1_OK = 0,
    P25_PHASE1_INVALID_ARGUMENT = -1,
    P25_PHASE1_CALLBACK_FAILED = -2,
    P25_PHASE1_INTERNAL_ERROR = -3
};

/*
 * Create a decoder for 2-bit P25 Phase 1 dibits. Clear audio is delivered
 * synchronously in 160-sample, 8 kHz mono blocks. Calls on one instance must
 * be serialized; distinct instances may be processed concurrently.
 */
p25_phase1_decoder *p25_phase1_decoder_create(
    p25_phase1_pcm_callback pcm_callback, void *user_data);
void p25_phase1_decoder_destroy(p25_phase1_decoder *decoder);
int p25_phase1_decoder_reset(p25_phase1_decoder *decoder);
int p25_phase1_decoder_set_expected_nac(p25_phase1_decoder *decoder, uint16_t nac);

/*
 * Process any number of dibits (each input byte must be in [0,3]).
 * The block is validated before decoder state changes. A nonzero callback
 * return stops processing and is reported as P25_PHASE1_CALLBACK_FAILED.
 */
int p25_phase1_decoder_process_dibits(
    p25_phase1_decoder *decoder, const uint8_t *dibits, size_t count);
int p25_phase1_decoder_get_stats(
    const p25_phase1_decoder *decoder, p25_phase1_stats *stats);

#ifdef __cplusplus
}
#endif

#endif
