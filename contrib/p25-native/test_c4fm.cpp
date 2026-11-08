#include "c4fm.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>

#include "upstream/frame_sync_magics.h"

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
    capture->pcm.insert(capture->pcm.end(), pcm, pcm + P25_IMBE_PCM_SAMPLES);
    return 0;
}

void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

std::vector<std::complex<float>> load_fixture()
{
    static_assert(sizeof(float) == 4, "fixture requires 32-bit float");
    std::ifstream input("fixtures/op25-c4fm-48k.c64", std::ios::binary);
    require(input.good(), "OP25 C4FM IQ fixture opens");

    std::vector<float> interleaved;
    float component;
    while (input.read(reinterpret_cast<char *>(&component), sizeof(component)))
        interleaved.push_back(component);
    require(input.eof(), "OP25 C4FM IQ fixture is fully readable");
    require(interleaved.size() % 2 == 0, "IQ fixture has complete samples");

    std::vector<std::complex<float>> iq;
    iq.reserve(interleaved.size() / 2);
    for (size_t i = 0; i < interleaved.size(); i += 2)
        iq.emplace_back(interleaved[i], interleaved[i + 1]);
    return iq;
}

std::vector<std::complex<float>> offset_iq(
    const std::vector<std::complex<float>>& iq, float offset_hz)
{
    constexpr float sample_rate = 48000.0f;
    constexpr float two_pi = 6.2831853071795864769f;
    std::vector<std::complex<float>> result;
    result.reserve(iq.size());
    for (size_t i = 0; i < iq.size(); ++i) {
        const float phase = two_pi * offset_hz *
                            static_cast<float>(i) / sample_rate;
        result.push_back(iq[i] * std::polar(1.0f, phase));
    }
    return result;
}

std::vector<std::complex<float>> transform_iq(
    const std::vector<std::complex<float>>& iq,
    float phase_radians, float amplitude)
{
    const std::complex<float> rotation =
        amplitude * std::polar(1.0f, phase_radians);
    std::vector<std::complex<float>> result;
    result.reserve(iq.size());
    for (const auto& sample : iq)
        result.push_back(sample * rotation);
    return result;
}

AudioCapture decode(const std::vector<std::complex<float>>& iq,
                   const std::vector<size_t>& blocks,
                   p25_phase1_stats *stats, uint64_t *symbols,
                   float *frequency_offset_hz = nullptr)
{
    AudioCapture audio;
    p25_c4fm_decoder_impl decoder(capture_audio, &audio);
    require(decoder.set_expected_nac(0x293) == P25_PHASE1_OK,
            "expected NAC configuration");

    size_t offset = 0;
    size_t block_index = 0;
    while (offset < iq.size()) {
        const size_t count =
            std::min(blocks[block_index++ % blocks.size()], iq.size() - offset);
        require(decoder.process(iq.data() + offset, count) == P25_PHASE1_OK,
                "C4FM processing succeeds");
        offset += count;
    }

    uint64_t input_samples = 0;
    require(decoder.get_stats(&input_samples, symbols, stats) == P25_PHASE1_OK,
            "C4FM decoder statistics are available");
    require(input_samples == iq.size(), "all IQ samples are consumed");
    if (frequency_offset_hz != nullptr)
        *frequency_offset_hz = decoder.frequency_offset_hz();
    return audio;
}

std::vector<uint8_t> encode_s16be(
    const std::vector<std::complex<float>>& iq)
{
    std::vector<uint8_t> bytes(iq.size() * 4);
    for (size_t i = 0; i < iq.size(); ++i) {
        const auto encode_component = [](float sample) {
            const long scaled = std::lround(sample * 32767.0f);
            return static_cast<uint16_t>(
                std::clamp(scaled, -32768L, 32767L));
        };
        const uint16_t i_sample = encode_component(iq[i].real());
        const uint16_t q_sample = encode_component(iq[i].imag());
        bytes[4 * i] = static_cast<uint8_t>(i_sample >> 8);
        bytes[4 * i + 1] = static_cast<uint8_t>(i_sample);
        bytes[4 * i + 2] = static_cast<uint8_t>(q_sample >> 8);
        bytes[4 * i + 3] = static_cast<uint8_t>(q_sample);
    }
    return bytes;
}

AudioCapture decode_s16be(const std::vector<uint8_t>& bytes,
                          size_t sample_count,
                          const std::vector<size_t>& blocks,
                          p25_phase1_stats *stats, uint64_t *symbols)
{
    AudioCapture audio;
    p25_c4fm_decoder_impl decoder(capture_audio, &audio);
    require(decoder.set_expected_nac(0x293) == P25_PHASE1_OK,
            "expected NAC configuration for s16be input");

    size_t offset = 0;
    size_t block_index = 0;
    while (offset < sample_count) {
        const size_t count =
            std::min(blocks[block_index++ % blocks.size()],
                     sample_count - offset);
        require(decoder.process_s16be(bytes.data() + offset * 4, count) ==
                    P25_PHASE1_OK,
                "s16be C4FM processing succeeds");
        offset += count;
    }

    uint64_t input_samples = 0;
    require(decoder.get_stats(&input_samples, symbols, stats) == P25_PHASE1_OK,
            "s16be decoder statistics are available");
    require(input_samples == sample_count, "all s16be IQ samples are consumed");
    return audio;
}

std::array<uint8_t, 24> dibits_for_sync(uint64_t sync)
{
    std::array<uint8_t, 24> dibits{};
    for (size_t i = 0; i < dibits.size(); ++i)
        dibits[i] = static_cast<uint8_t>((sync >> (46 - 2 * i)) & 0x3);
    return dibits;
}

void test_dibit_rotations()
{
    constexpr uint64_t sync = P25_FRAME_SYNC_MAGIC;
    constexpr std::array<uint64_t, 5> variants{{
        P25_FRAME_SYNC_MAGIC,
        P25_FRAME_SYNC_REV_P,
        P25_FRAME_SYNC_X2400,
        P25_FRAME_SYNC_N1200,
        P25_FRAME_SYNC_P1200,
    }};
    const auto expected = dibits_for_sync(sync);

    for (uint64_t variant : variants) {
        p25_dibit_mapper mapper;
        const auto raw = dibits_for_sync(variant);
        for (uint8_t dibit : raw)
            mapper.process(dibit);

        for (size_t i = 0; i < raw.size(); ++i)
            require(mapper.process(raw[i]) == expected[i],
                    "OP25 sync rotation maps subsequent dibits to canonical order");
    }

    p25_dibit_mapper mapper;
    const auto reverse = dibits_for_sync(P25_FRAME_SYNC_REV_P);
    for (uint8_t dibit : reverse)
        mapper.process(dibit);
    mapper.reset();
    for (uint8_t dibit = 0; dibit < 4; ++dibit)
        require(mapper.process(dibit) == dibit,
                "dibit mapper reset restores normal polarity");
}

} // namespace

int main()
{
    test_dibit_rotations();

    const auto iq = load_fixture();
    require(iq.size() == 19200, "fixture contains two complete P25 frames");

    p25_phase1_stats contiguous_stats{};
    uint64_t contiguous_symbols = 0;
    float contiguous_offset_hz = 0;
    const AudioCapture contiguous =
        decode(iq, {iq.size()}, &contiguous_stats, &contiguous_symbols,
               &contiguous_offset_hz);
    p25_phase1_stats partitioned_stats{};
    uint64_t partitioned_symbols = 0;
    const AudioCapture partitioned =
        decode(iq, {1, 13, 4096, 31, 7, 501}, &partitioned_stats,
               &partitioned_symbols);

    require(contiguous_symbols > 1900 && contiguous_symbols < 1940,
            "C4FM timing recovery produces the expected symbol count");
    require(std::abs(contiguous_offset_hz) < 100.0f,
            "zero-offset fixture reports near-zero carrier offset");
    require(contiguous_stats.complete_frames == 2,
            "OP25-generated IQ recovers both P25 frames");
    require(contiguous_stats.ldu1_frames == 1 &&
                contiguous_stats.ldu2_frames == 1,
            "both synthetic voice frame types are recovered");
    require(contiguous_stats.last_nac == 0x293 &&
                contiguous_stats.last_duid == 5 &&
                contiguous_stats.last_bch_errors == 0,
            "P25 frame metadata and NID decode correctly");
    require(contiguous_stats.voice_codewords_decoded == 18,
            "clear LDU1 and LDU2 voice codewords reach IMBE");
    require(contiguous.blocks == 18 && !contiguous.pcm.empty(),
            "OP25-generated IQ produces PCM");
    require(std::any_of(contiguous.pcm.begin(), contiguous.pcm.end(),
                        [](int16_t sample) { return sample != 0; }),
            "decoded PCM contains nonzero samples");
    require(contiguous.pcm == partitioned.pcm,
            "arbitrary IQ block sizes preserve PCM output");
    require(contiguous_symbols == partitioned_symbols &&
                contiguous_stats.complete_frames ==
                    partitioned_stats.complete_frames &&
                contiguous_stats.voice_codewords_decoded ==
                    partitioned_stats.voice_codewords_decoded,
            "arbitrary IQ blocks preserve decoder statistics");

    for (float phase : {1.57079632679f, 3.14159265359f,
                        4.71238898038f}) {
        const auto rotated_iq = transform_iq(iq, phase, 1.0f);
        p25_phase1_stats rotated_stats{};
        uint64_t rotated_symbols = 0;
        const AudioCapture rotated = decode(
            rotated_iq, {1, 13, 4096, 31, 7, 501}, &rotated_stats,
            &rotated_symbols);
        require(rotated_stats.complete_frames == contiguous_stats.complete_frames &&
                    rotated_stats.voice_codewords_decoded ==
                        contiguous_stats.voice_codewords_decoded &&
                    rotated.pcm == contiguous.pcm,
                "constant IQ phase rotations preserve C4FM decode output");
    }

    for (float amplitude : {0.1f, 2.0f}) {
        const auto scaled_iq = transform_iq(iq, 0.0f, amplitude);
        p25_phase1_stats scaled_stats{};
        uint64_t scaled_symbols = 0;
        const AudioCapture scaled = decode(
            scaled_iq, {1, 13, 4096, 31, 7, 501}, &scaled_stats,
            &scaled_symbols);
        require(scaled_stats.complete_frames == contiguous_stats.complete_frames &&
                    scaled_stats.voice_codewords_decoded ==
                        contiguous_stats.voice_codewords_decoded &&
                    scaled.pcm == contiguous.pcm,
                "RMS AGC preserves C4FM decode output across signal levels");
    }

    AudioCapture nac_rejected_audio;
    p25_c4fm_decoder_impl nac_rejected_decoder(capture_audio,
                                                &nac_rejected_audio);
    require(nac_rejected_decoder.set_expected_nac(0x294) == P25_PHASE1_OK,
            "different NAC filter is accepted");
    require(nac_rejected_decoder.process(iq.data(), iq.size()) ==
                P25_PHASE1_OK,
            "IQ with a different NAC still processes");
    p25_phase1_stats nac_rejected_stats{};
    uint64_t nac_rejected_samples = 0;
    uint64_t nac_rejected_symbols = 0;
    require(nac_rejected_decoder.get_stats(&nac_rejected_samples,
                                           &nac_rejected_symbols,
                                           &nac_rejected_stats) ==
                P25_PHASE1_OK &&
                nac_rejected_stats.complete_frames == 0 &&
                nac_rejected_audio.blocks == 0,
            "configured NAC rejects frames from another NAC");

    const auto iq_s16be = encode_s16be(iq);
    p25_phase1_stats s16be_stats{};
    uint64_t s16be_symbols = 0;
    const AudioCapture s16be =
        decode_s16be(iq_s16be, iq.size(), {1, 257, 5, 1024, 17},
                     &s16be_stats, &s16be_symbols);
    require(s16be_stats.complete_frames == 2 &&
                s16be_stats.voice_codewords_decoded == 18,
            "s16be IQ recovers both P25 frames");
    require(s16be.pcm == contiguous.pcm,
            "s16be conversion preserves decoded PCM");

    std::vector<std::complex<float>> repeated_iq;
    for (int repeat = 0; repeat < 5; ++repeat)
        repeated_iq.insert(repeated_iq.end(), iq.begin(), iq.end());
    for (float offset_hz : {-5000.0f, 5000.0f}) {
        const auto shifted_iq = offset_iq(repeated_iq, offset_hz);
        p25_phase1_stats shifted_stats{};
        uint64_t shifted_symbols = 0;
        float estimated_offset_hz = 0;
        (void)decode(shifted_iq, {1, 13, 4096, 31, 7, 501},
                     &shifted_stats, &shifted_symbols,
                     &estimated_offset_hz);
        require(shifted_stats.complete_frames >= 8,
                "repeated P25 IQ reacquires after a carrier offset");
        require(std::abs(estimated_offset_hz - offset_hz) < 100.0f,
                "FSK4 loop estimates carrier offset within 100 Hz");
    }

    AudioCapture reset_audio;
    p25_c4fm_decoder_impl reset_decoder(capture_audio, &reset_audio);
    require(reset_decoder.set_expected_nac(0x293) == P25_PHASE1_OK,
            "expected NAC configuration before reset test");
    require(reset_decoder.process(iq.data(), iq.size()) == P25_PHASE1_OK,
            "C4FM input processes before reset");
    require(reset_decoder.reset() == P25_PHASE1_OK,
            "C4FM decoder reset succeeds");
    reset_audio.pcm.clear();
    reset_audio.blocks = 0;
    require(reset_decoder.set_expected_nac(0x293) == P25_PHASE1_OK,
            "expected NAC configuration after reset");
    require(reset_decoder.process(iq.data(), iq.size()) == P25_PHASE1_OK,
            "C4FM input processes after reset");
    require(reset_audio.pcm == contiguous.pcm,
            "reset restores identical PCM output");

    std::puts("native OP25 C4FM-to-PCM regression passed");
    return 0;
}
