#include "p25_native_api.h"

#include <stdio.h>
#include <stdlib.h>

struct audio_state {
    uint32_t blocks;
    uint32_t nonzero_samples;
    uint16_t nac;
    uint8_t last_duid;
};

static int pcm_callback(void *user_data, uint16_t nac, uint8_t duid,
                        uint32_t bch_errors, uint32_t imbe_errors,
                        const int16_t pcm[P25_IMBE_PCM_SAMPLES])
{
    struct audio_state *state = user_data;
    size_t i;
    (void)bch_errors;
    (void)imbe_errors;
    state->blocks++;
    state->nac = nac;
    state->last_duid = duid;
    for (i = 0; i < P25_IMBE_PCM_SAMPLES; ++i) {
        if (pcm[i] != 0)
            state->nonzero_samples++;
    }
    return 0;
}

static void require(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    FILE *input;
    uint8_t buffer[4096];
    float float_buffer[1024];
    struct audio_state audio = {0, 0, 0, 0};
    p25_native_decoder *decoder = NULL;
    p25_phase1_stats stats;
    uint64_t samples = 0;
    uint64_t symbols = 0;
    uint64_t sync_count = 0;
    size_t bytes;

    require(p25_native_decoder_create(P25_NATIVE_C4FM, pcm_callback, &audio,
                                      NULL) ==
                P25_PHASE1_INVALID_ARGUMENT,
            "C API rejects a null decoder output");
    require(p25_native_decoder_create(P25_NATIVE_C4FM, pcm_callback, &audio,
                                      &decoder) ==
                P25_PHASE1_OK &&
                decoder != NULL,
            "C API creates a decoder");
    require(p25_native_decoder_set_expected_nac(decoder, 0x293) ==
                P25_PHASE1_OK,
            "C API sets the expected NAC");
    require(p25_native_decoder_process_s16be(decoder, NULL, 0) ==
                P25_PHASE1_OK,
            "C API accepts an empty input block");

    input = fopen("fixtures/op25-c4fm-48k.s16be", "rb");
    require(input != NULL, "s16be fixture opens");
    while ((bytes = fread(buffer, 1, sizeof(buffer), input)) != 0) {
        require((bytes & 3U) == 0, "fixture block contains complete IQ samples");
        require(p25_native_decoder_process_s16be(decoder, buffer, bytes / 4) ==
                    P25_PHASE1_OK,
                "C API processes arbitrary IQ blocks");
    }
    require(!ferror(input), "fixture reads without error");
    require(fclose(input) == 0, "fixture closes without error");

    require(p25_native_decoder_get_stats(decoder, &samples, &symbols, &stats) ==
                P25_PHASE1_OK,
            "C API returns decoder statistics");
    require(samples == 19200 && symbols == 1920 &&
                stats.complete_frames == 2 &&
                stats.voice_codewords_decoded == 18,
            "C API reports recovered IQ, frames, and voice codewords");
    require(p25_native_decoder_get_sync_count(decoder, &sync_count) ==
                P25_PHASE1_OK &&
                sync_count == 2,
            "C API reports detected P25 sync words");
    require(audio.blocks == 18 && audio.nonzero_samples != 0 &&
                audio.nac == 0x293 && audio.last_duid == 5,
            "C API delivers PCM and frame metadata");

    require(p25_native_decoder_reset(decoder) == P25_PHASE1_OK,
            "C API resets the decoder");
    require(p25_native_decoder_get_stats(decoder, &samples, &symbols, &stats) ==
                P25_PHASE1_OK &&
                samples == 0 && symbols == 0 && stats.complete_frames == 0,
            "reset clears C API telemetry");
    require(p25_native_decoder_get_sync_count(decoder, &sync_count) ==
                P25_PHASE1_OK &&
                sync_count == 0,
            "reset clears sync-word telemetry");

    input = fopen("fixtures/op25-c4fm-48k.c64", "rb");
    require(input != NULL, "complex-float fixture opens");
    while ((bytes = fread(float_buffer, sizeof(float), 1024, input)) != 0) {
        require((bytes & 1U) == 0, "float block contains complete IQ samples");
        require(p25_native_decoder_process_f32(decoder, float_buffer, bytes / 2) ==
                    P25_PHASE1_OK,
                "C API processes interleaved float IQ blocks");
    }
    require(!ferror(input), "complex-float fixture reads without error");
    require(fclose(input) == 0, "complex-float fixture closes without error");
    require(p25_native_decoder_get_stats(decoder, &samples, &symbols, &stats) ==
                P25_PHASE1_OK &&
                samples == 19200 && symbols == 1920 &&
                stats.complete_frames == 2 &&
                stats.voice_codewords_decoded == 18,
            "float C API recovers the same C4FM stream");
    p25_native_decoder_destroy(decoder);
    puts("native P25 C API test passed");
    return 0;
}
