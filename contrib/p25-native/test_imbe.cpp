#include "imbe.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <thread>

#include "op25_imbe_frame.h"

static std::array<uint8_t, P25_IMBE_CODEWORD_BYTES> make_codeword()
{
    voice_codeword bits(voice_codeword_sz);
    imbe_header_encode(bits, 0x321, 0x456, 0x123, 0x5a5,
                       0x234, 0x567, 0x345, 0x52);

    std::array<uint8_t, P25_IMBE_CODEWORD_BYTES> packed{};
    for (size_t i = 0; i < voice_codeword_sz; ++i)
        packed[i / 8] |= static_cast<uint8_t>(bits[i] << (7 - (i % 8)));
    return packed;
}

int main()
{
    const auto codeword = make_codeword();
    p25_imbe_decoder *decoder = p25_imbe_decoder_create();
    p25_imbe_decoder *independent = p25_imbe_decoder_create();
    assert(decoder != nullptr && independent != nullptr);

    std::array<int16_t, P25_IMBE_PCM_SAMPLES> first{};
    std::array<int16_t, P25_IMBE_PCM_SAMPLES> subsequent{};
    std::array<int16_t, P25_IMBE_PCM_SAMPLES> fresh{};
    std::array<int16_t, P25_IMBE_PCM_SAMPLES> interleaved{};
    std::array<int16_t, P25_IMBE_PCM_SAMPLES> parallel_a{};
    std::array<int16_t, P25_IMBE_PCM_SAMPLES> parallel_b{};
    p25_imbe_stats stats{};

    assert(p25_imbe_decoder_decode(decoder, codeword.data(), first.data(), &stats) == 0);
    assert(stats.first_golay_errors == 0);
    assert(stats.corrected_errors == 0);
    assert(std::any_of(first.begin(), first.end(), [](int16_t sample) { return sample != 0; }));

    assert(p25_imbe_decoder_decode(decoder, codeword.data(), subsequent.data(), nullptr) == 0);
    assert(p25_imbe_decoder_decode(independent, codeword.data(), fresh.data(), nullptr) == 0);
    assert(first == fresh);
    assert(p25_imbe_decoder_decode(independent, codeword.data(), fresh.data(), nullptr) == 0);
    assert(subsequent == fresh);
    assert(p25_imbe_decoder_decode(independent, codeword.data(), fresh.data(), nullptr) == 0);
    assert(p25_imbe_decoder_decode(decoder, codeword.data(), interleaved.data(), nullptr) == 0);
    assert(fresh == interleaved);

    p25_imbe_decoder *parallel_decoder = p25_imbe_decoder_create();
    p25_imbe_decoder *other_parallel_decoder = p25_imbe_decoder_create();
    assert(parallel_decoder != nullptr && other_parallel_decoder != nullptr);
    std::thread first_thread([&] {
        assert(p25_imbe_decoder_decode(parallel_decoder, codeword.data(),
                                       parallel_a.data(), nullptr) == 0);
    });
    std::thread second_thread([&] {
        assert(p25_imbe_decoder_decode(other_parallel_decoder, codeword.data(),
                                       parallel_b.data(), nullptr) == 0);
    });
    first_thread.join();
    second_thread.join();
    assert(first == parallel_a);
    assert(first == parallel_b);

    assert(p25_imbe_decoder_reset(decoder) == 0);
    assert(p25_imbe_decoder_decode(decoder, codeword.data(), fresh.data(), nullptr) == 0);
    assert(first == fresh);

    auto damaged = codeword;
    damaged[0] ^= 0x80;
    assert(p25_imbe_decoder_decode(decoder, damaged.data(), fresh.data(), &stats) == 0);
    assert(stats.first_golay_errors == 1);
    assert(stats.corrected_errors == 1);

    assert(p25_imbe_decoder_decode(nullptr, codeword.data(), fresh.data(), nullptr) == -1);
    p25_imbe_decoder_destroy(decoder);
    p25_imbe_decoder_destroy(independent);
    p25_imbe_decoder_destroy(parallel_decoder);
    p25_imbe_decoder_destroy(other_parallel_decoder);
    std::puts("native OP25 IMBE tests passed");
    return 0;
}
