#include "cqpsk.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "p25_native_api.h"
#include "test_frame_builder.h"

namespace {

struct AudioCapture {
    std::vector<int16_t> pcm;
    size_t blocks = 0;
};

int capture_audio(void *user_data, uint16_t, uint8_t, uint32_t, uint32_t,
                  const int16_t pcm[P25_IMBE_PCM_SAMPLES])
{
    auto *capture = static_cast<AudioCapture *>(user_data);
    ++capture->blocks;
    capture->pcm.insert(capture->pcm.end(), pcm,
                        pcm + P25_IMBE_PCM_SAMPLES);
    return 0;
}

void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

std::vector<uint8_t> make_dibits()
{
    std::vector<uint8_t> dibits(128);
    constexpr std::array<uint8_t, 4> preamble = {0, 3, 1, 2};
    for (size_t i = 0; i < dibits.size(); ++i)
        dibits[i] = preamble[i % preamble.size()];
    const auto ldu1 = p25_test::make_frame(10, 0x80);
    const auto ldu2 = p25_test::make_frame(5);
    dibits.insert(dibits.end(), ldu1.begin(), ldu1.end());
    dibits.insert(dibits.end(), ldu2.begin(), ldu2.end());
    dibits.insert(dibits.end(), 64, 0);
    return dibits;
}

std::vector<std::complex<float>> modulate_qpsk(
    const std::vector<uint8_t>& dibits, float amplitude = 0.7f,
    float phase_offset = 0.0f, float carrier_offset_hz = 0.0f,
    float samples_per_symbol = 10.0f)
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float sample_rate = 48000.0f;
    constexpr std::array<float, 4> phase_steps{
        pi / 4.0f, 3.0f * pi / 4.0f, -pi / 4.0f, -3.0f * pi / 4.0f
    };
    std::vector<float> symbol_phases;
    symbol_phases.reserve(dibits.size());
    float phase = 0.0f;
    for (uint8_t dibit : dibits) {
        phase = std::remainder(phase + phase_steps[dibit], 2.0f * pi);
        symbol_phases.push_back(phase);
    }

    const size_t sample_count = static_cast<size_t>(
        std::ceil(dibits.size() * samples_per_symbol));
    std::vector<std::complex<float>> iq;
    iq.reserve(sample_count);
    for (size_t sample_index = 0; sample_index < sample_count;
         ++sample_index) {
        const size_t dibit_index = std::min(
            static_cast<size_t>(sample_index / samples_per_symbol),
            dibits.size() - 1);
        const float total_phase =
            symbol_phases[dibit_index] + phase_offset +
            2.0f * pi * carrier_offset_hz *
                static_cast<float>(sample_index) / sample_rate;
        iq.push_back(std::polar(amplitude, total_phase));
    }
    return iq;
}

AudioCapture decode(const std::vector<std::complex<float>>& iq,
                    const std::vector<size_t>& blocks,
                    p25_phase1_stats *stats, uint64_t *symbols,
                    float *frequency_offset_hz)
{
    AudioCapture audio;
    p25_cqpsk_decoder_impl decoder(capture_audio, &audio);
    require(decoder.set_expected_nac(0x293) == P25_PHASE1_OK,
            "expected NAC configuration");

    size_t offset = 0;
    size_t block_index = 0;
    while (offset < iq.size()) {
        const size_t count =
            std::min(blocks[block_index++ % blocks.size()], iq.size() - offset);
        require(decoder.process(iq.data() + offset, count) == P25_PHASE1_OK,
                "CQPSK processing succeeds");
        offset += count;
    }

    uint64_t input_samples = 0;
    require(decoder.get_stats(&input_samples, symbols, stats) == P25_PHASE1_OK,
            "CQPSK statistics are available");
    require(input_samples == iq.size(), "all CQPSK IQ samples are consumed");
    *frequency_offset_hz = decoder.frequency_offset_hz();
    return audio;
}

void test_native_api_dispatch()
{
    const float iq[] = {0.0f, 0.0f};
    for (p25_native_modulation modulation :
         {P25_NATIVE_C4FM, P25_NATIVE_CQPSK}) {
        p25_native_decoder *decoder = nullptr;
        require(p25_native_decoder_create(modulation, capture_audio, nullptr,
                                          &decoder) == P25_PHASE1_OK &&
                    decoder != nullptr,
                "generic API creates each demodulator");
        require(p25_native_decoder_process_f32(decoder, iq, 1) ==
                    P25_PHASE1_OK,
                "generic API processes IQ for each demodulator");
        require(p25_native_decoder_reset(decoder) == P25_PHASE1_OK,
                "generic API resets each demodulator");
        p25_native_decoder_destroy(decoder);
    }
}

} // namespace

int main()
{
    test_native_api_dispatch();
    const auto iq = modulate_qpsk(make_dibits());
    p25_phase1_stats contiguous_stats{};
    uint64_t contiguous_symbols = 0;
    float offset_hz = 0.0f;
    const AudioCapture contiguous =
        decode(iq, {iq.size()}, &contiguous_stats, &contiguous_symbols,
               &offset_hz);
    p25_phase1_stats partitioned_stats{};
    uint64_t partitioned_symbols = 0;
    float partitioned_offset_hz = 0.0f;
    const AudioCapture partitioned =
        decode(iq, {1, 13, 4096, 31, 7, 501}, &partitioned_stats,
               &partitioned_symbols, &partitioned_offset_hz);

    require(std::abs(offset_hz) < 100.0f,
            "zero-offset CQPSK reports near-zero carrier offset");
    require(contiguous_stats.complete_frames == 2,
            "synthetic CQPSK recovers both P25 frames");
    require(contiguous_stats.ldu1_frames == 1 &&
                contiguous_stats.ldu2_frames == 1,
            "synthetic CQPSK recovers both voice frame types");
    require(contiguous_stats.last_nac == 0x293 &&
                contiguous_stats.last_bch_errors == 0,
            "CQPSK frame NID decodes correctly");
    require(contiguous_stats.voice_codewords_decoded == 18 &&
                contiguous.blocks == 18,
            "CQPSK clear voice frames reach IMBE");
    require(std::any_of(contiguous.pcm.begin(), contiguous.pcm.end(),
                        [](int16_t sample) { return sample != 0; }),
            "CQPSK decoded PCM is nonzero");
    require(contiguous.pcm == partitioned.pcm &&
                contiguous_symbols == partitioned_symbols &&
                contiguous_stats.complete_frames ==
                    partitioned_stats.complete_frames &&
                contiguous_stats.voice_codewords_decoded ==
                    partitioned_stats.voice_codewords_decoded &&
                std::abs(offset_hz - partitioned_offset_hz) < 0.01f,
            "arbitrary block sizes preserve CQPSK output and state");

    for (const auto& test_case :
         {std::array<float, 4>{0.07f, 1.1f, 0.0f, 10.0f},
          std::array<float, 4>{1.4f, -2.0f, 0.0f, 10.0f},
          std::array<float, 4>{0.7f, 0.0f, 300.0f, 10.0f},
          std::array<float, 4>{0.7f, 0.0f, -300.0f, 10.01f}}) {
        const auto varied_iq = modulate_qpsk(
            make_dibits(), test_case[0], test_case[1], test_case[2],
            test_case[3]);
        p25_phase1_stats varied_stats{};
        uint64_t varied_symbols = 0;
        float varied_offset_hz = 0.0f;
        (void)decode(varied_iq, {511}, &varied_stats, &varied_symbols,
                     &varied_offset_hz);
        require(varied_stats.complete_frames == 2 &&
                    varied_stats.voice_codewords_decoded == 18,
                "CQPSK retains frame recovery across level, phase, carrier, and clock variation");
    }

    std::puts("native OP25 CQPSK-to-PCM regression passed");
    return 0;
}
