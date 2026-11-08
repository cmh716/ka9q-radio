#ifndef KA9Q_P25_IMBE_H
#define KA9Q_P25_IMBE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define P25_IMBE_CODEWORD_BYTES 18
#define P25_IMBE_PCM_SAMPLES 160

typedef struct p25_imbe_decoder p25_imbe_decoder;

typedef struct {
    uint32_t first_golay_errors;
    uint32_t corrected_errors;
} p25_imbe_stats;

/* Each instance owns independent, persistent vocoder state. */
p25_imbe_decoder *p25_imbe_decoder_create(void);
void p25_imbe_decoder_destroy(p25_imbe_decoder *decoder);
int p25_imbe_decoder_reset(p25_imbe_decoder *decoder);

/*
 * Decode one 144-bit IMBE codeword, packed most-significant bit first.
 * Writes exactly P25_IMBE_PCM_SAMPLES mono signed 16-bit samples at 8 kHz.
 * stats may be NULL. Returns 0 on success and -1 for invalid arguments or
 * allocation failure.
 */
int p25_imbe_decoder_decode(p25_imbe_decoder *decoder,
                            const uint8_t codeword[P25_IMBE_CODEWORD_BYTES],
                            int16_t pcm[P25_IMBE_PCM_SAMPLES],
                            p25_imbe_stats *stats);

#ifdef __cplusplus
}
#endif

#endif
