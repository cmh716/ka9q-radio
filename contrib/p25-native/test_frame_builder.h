#ifndef KA9Q_P25_TEST_FRAME_BUILDER_H
#define KA9Q_P25_TEST_FRAME_BUILDER_H

#include <array>
#include <cassert>
#include <cstdint>
#include <vector>

#include "ezpwd/rs"
#include "op25_hamming.h"
#include "op25_imbe_frame.h"
#include "op25_p25_frame.h"

namespace p25_test {

inline uint64_t encode_nid(uint16_t nac, uint8_t duid)
{
    static constexpr uint64_t matrix[16] = {
        0x8000cd930bdd3b2aULL, 0x4000ab5a8e33a6beULL,
        0x2000983e4cc4e874ULL, 0x10004c1f2662743aULL,
        0x0800eb9c98ec0136ULL, 0x0400b85d47ab3bb0ULL,
        0x02005c2ea3d59dd8ULL, 0x01002e1751eaceecULL,
        0x0080170ba8f56776ULL, 0x0040c616dfa78890ULL,
        0x0020630b6fd3c448ULL, 0x00103185b7e9e224ULL,
        0x000818c2dbf4f112ULL, 0x0004c1f2662743a2ULL,
        0x0002ad6a38ce9afbULL, 0x00019b2617ba7657ULL
    };

    const uint16_t data = static_cast<uint16_t>((nac << 4) | duid);
    uint64_t codeword = 0;
    for (int i = 0; i < 16; ++i) {
        if (data & (0x8000 >> i))
            codeword ^= matrix[i];
    }
    return codeword;
}

inline voice_codeword make_voice_codeword()
{
    voice_codeword codeword(voice_codeword_sz);
    imbe_header_encode(codeword, 0x321, 0x456, 0x123, 0x5a5,
                       0x234, 0x567, 0x345, 0x52);
    return codeword;
}

inline std::vector<uint8_t> make_frame(uint8_t duid, uint8_t algorithm = 0x80,
                                       bool corrupt_nid = false,
                                       uint16_t nac = 0x293)
{
    bit_vector body(P25_VOICE_FRAME_SIZE, false);
    p25_setup_frame_header(body, encode_nid(nac, duid));
    if (corrupt_nid)
        body[48] = !body[48];

    if (duid == 10) {
        std::array<uint8_t, 63> ess{};
        ess[51] = algorithm >> 2;
        ess[52] = static_cast<uint8_t>((algorithm & 3) << 4);
        ezpwd::RS<63, 55> rs;
        assert(rs.encode(ess, 39) >= 0);

        size_t index = 0;
        for (size_t i = 0; i < 24; ++i) {
            const uint8_t symbol = ess[39 + i];
            const uint32_t codeword =
                (static_cast<uint32_t>(symbol) << 4) | hmg1063EncTbl[symbol];
            for (int bit = 9; bit >= 0; --bit) {
                body[imbe_ldu_ls_data_bits[index++]] = (codeword >> bit) & 1;
            }
        }
    }

    const voice_codeword voice = make_voice_codeword();
    for (size_t i = 0; i < nof_voice_codewords; ++i)
        imbe_interleave(body, voice, i);

    const size_t frame_bits = (duid == 3 || duid == 15) ? 144 : P25_VOICE_FRAME_SIZE;
    std::vector<uint8_t> dibits(frame_bits / 2);
    for (size_t i = 0; i < dibits.size(); ++i)
        dibits[i] = static_cast<uint8_t>((body[2 * i] << 1) | body[2 * i + 1]);
    return dibits;
}

} // namespace p25_test

#endif
