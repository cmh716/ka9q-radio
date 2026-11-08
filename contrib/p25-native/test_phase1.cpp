#include "phase1.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <vector>

#include "test_frame_builder.h"

struct AudioCapture {
    size_t blocks = 0;
    size_t samples = 0;
    bool nonzero = false;
    uint16_t nac = 0;
    uint8_t duid = 0;
    uint32_t nid_errors = 0;
    uint64_t nid_error_total = 0;
};

static int capture_audio(void *user_data, uint16_t nac, uint8_t duid,
                         uint32_t bch_errors, uint32_t,
                         const int16_t pcm[P25_IMBE_PCM_SAMPLES])
{
    auto *capture = static_cast<AudioCapture *>(user_data);
    ++capture->blocks;
    capture->samples += P25_IMBE_PCM_SAMPLES;
    capture->nonzero |= std::any_of(
        pcm, pcm + P25_IMBE_PCM_SAMPLES,
        [](int16_t sample) { return sample != 0; });
    capture->nac = nac;
    capture->duid = duid;
    capture->nid_errors = bch_errors;
    capture->nid_error_total += bch_errors;
    return 0;
}

static int reject_audio(void *, uint16_t, uint8_t, uint32_t, uint32_t,
                        const int16_t[P25_IMBE_PCM_SAMPLES])
{
    return -1;
}

static void feed_in_chunks(p25_phase1_decoder *decoder,
                           const std::vector<uint8_t>& dibits)
{
    static constexpr size_t chunks[] = {1, 7, 37, 2, 19, 53};
    size_t offset = 0;
    size_t chunk_index = 0;
    while (offset < dibits.size()) {
        const size_t count = std::min(chunks[chunk_index++ % std::size(chunks)],
                                      dibits.size() - offset);
        assert(p25_phase1_decoder_process_dibits(
                   decoder, dibits.data() + offset, count) == P25_PHASE1_OK);
        offset += count;
    }
}

int main()
{
    AudioCapture audio;
    p25_phase1_decoder *decoder = p25_phase1_decoder_create(capture_audio, &audio);
    assert(decoder != nullptr);
    assert(p25_phase1_decoder_set_expected_nac(decoder, 0x293) == P25_PHASE1_OK);

    std::array<uint8_t, 120> noise{};
    assert(p25_phase1_decoder_process_dibits(decoder, noise.data(), noise.size()) ==
           P25_PHASE1_OK);

    feed_in_chunks(decoder, p25_test::make_frame(10, 0x80, true));
    p25_phase1_stats stats{};
    assert(p25_phase1_decoder_get_stats(decoder, &stats) == P25_PHASE1_OK);
    assert(stats.complete_frames == 1 && stats.last_bch_errors == 1);
    feed_in_chunks(decoder, p25_test::make_frame(5));
    assert(audio.blocks == 18);
    assert(audio.samples == 18 * P25_IMBE_PCM_SAMPLES);
    assert(audio.nonzero);
    assert(audio.nid_error_total == nof_voice_codewords);
    assert(audio.nac == 0x293 && audio.duid == 5);

    feed_in_chunks(decoder, p25_test::make_frame(10, 0xaa));
    feed_in_chunks(decoder, p25_test::make_frame(5));
    assert(audio.blocks == 18);

    feed_in_chunks(decoder, p25_test::make_frame(3));
    feed_in_chunks(decoder, p25_test::make_frame(5));
    assert(audio.blocks == 18);

    feed_in_chunks(decoder, p25_test::make_frame(10, 0x80));
    assert(audio.blocks == 27);

    assert(p25_phase1_decoder_get_stats(decoder, &stats) == P25_PHASE1_OK);
    assert(stats.complete_frames == 7);
    assert(stats.ldu1_frames == 3);
    assert(stats.ldu2_frames == 3);
    assert(stats.voice_codewords_decoded == 27);
    assert(stats.voice_codewords_skipped_encrypted == 18);
    assert(stats.voice_codewords_skipped_unknown_encryption == 9);
    assert(stats.last_nac == 0x293 && stats.last_duid == 10);
    assert(stats.last_bch_errors == 0);
    assert(stats.encryption_known && !stats.encrypted);

    const uint64_t dibits_before_invalid = stats.dibits_received;
    const uint8_t invalid_dibit = 4;
    assert(p25_phase1_decoder_process_dibits(decoder, &invalid_dibit, 1) ==
           P25_PHASE1_INVALID_ARGUMENT);
    assert(p25_phase1_decoder_get_stats(decoder, &stats) == P25_PHASE1_OK);
    assert(stats.dibits_received == dibits_before_invalid);

    assert(p25_phase1_decoder_reset(decoder) == P25_PHASE1_OK);
    assert(p25_phase1_decoder_get_stats(decoder, &stats) == P25_PHASE1_OK);
    assert(stats.complete_frames == 0 && !stats.encryption_known);
    feed_in_chunks(decoder, p25_test::make_frame(10, 0x80, false, 0x123));
    assert(p25_phase1_decoder_get_stats(decoder, &stats) == P25_PHASE1_OK);
    assert(stats.complete_frames == 0);
    feed_in_chunks(decoder, p25_test::make_frame(10, 0x80));
    assert(p25_phase1_decoder_get_stats(decoder, &stats) == P25_PHASE1_OK);
    assert(stats.complete_frames == 1 && stats.voice_codewords_decoded == 9);
    assert(audio.blocks == 36);
    p25_phase1_decoder_destroy(decoder);

    p25_phase1_decoder *callback_error =
        p25_phase1_decoder_create(reject_audio, nullptr);
    assert(callback_error != nullptr);
    const auto clear_ldu2 = p25_test::make_frame(10, 0x80);
    assert(p25_phase1_decoder_process_dibits(
               callback_error, clear_ldu2.data(), clear_ldu2.size()) ==
           P25_PHASE1_CALLBACK_FAILED);
    assert(p25_phase1_decoder_get_stats(callback_error, &stats) == P25_PHASE1_OK);
    assert(stats.complete_frames == 1 && stats.voice_codewords_decoded == 1);
    assert(stats.dibits_received == clear_ldu2.size());
    p25_phase1_decoder_destroy(callback_error);
    return 0;
}
