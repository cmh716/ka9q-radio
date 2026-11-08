#include "imbe.h"
#include "phase1.h"

#include <assert.h>
#include <stdint.h>

int main(void)
{
    uint8_t codeword[P25_IMBE_CODEWORD_BYTES] = {0};
    int16_t pcm[P25_IMBE_PCM_SAMPLES];
    p25_imbe_decoder *decoder = p25_imbe_decoder_create();
    p25_phase1_decoder *phase1 = p25_phase1_decoder_create(0, 0);

    assert(decoder != 0 && phase1 != 0);
    assert(p25_imbe_decoder_reset(decoder) == 0);
    assert(p25_imbe_decoder_decode(decoder, codeword, pcm, 0) == 0);
    assert(p25_phase1_decoder_process_dibits(phase1, 0, 0) == P25_PHASE1_OK);
    assert(p25_phase1_decoder_get_stats(phase1, 0) == P25_PHASE1_INVALID_ARGUMENT);
    p25_phase1_decoder_destroy(phase1);
    p25_imbe_decoder_destroy(decoder);
    return 0;
}
