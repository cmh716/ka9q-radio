#include "phase1.h"

#include <array>
#include <memory>
#include <vector>

#include "ezpwd/rs"
#include "log_ts.h"
#include "op25_hamming.h"
#include "op25_imbe_frame.h"
#include "p25_framer.h"

struct p25_phase1_decoder {
    explicit p25_phase1_decoder(p25_phase1_pcm_callback callback, void *data)
        : framer(logger), pcm_callback(callback), user_data(data),
          imbe(p25_imbe_decoder_create())
    {
    }

    ~p25_phase1_decoder()
    {
        p25_imbe_decoder_destroy(imbe);
    }

    log_ts logger;
    p25_framer framer;
    p25_phase1_pcm_callback pcm_callback;
    void *user_data;
    p25_imbe_decoder *imbe;
    ezpwd::RS<63, 55> ess_rs;
    p25_phase1_stats stats{};
    bool encryption_known = false;
    bool encrypted = false;
};

namespace {

bool read_ess(p25_phase1_decoder *decoder, const bit_vector& frame)
{
    std::vector<uint8_t> hex_bits(63, 0);
    size_t index = 0;
    for (size_t block = 0; block < 24; ++block) {
        uint32_t codeword = 0;
        for (size_t bit = 0; bit < 10; ++bit)
            codeword = (codeword << 1) + frame[imbe_ldu_ls_data_bits[index++]];
        hex_bits[39 + block] = hmg1063Dec(codeword >> 4, codeword & 0x0f);
    }

    const int errors = decoder->ess_rs.decode(hex_bits);
    if (errors < 0 || errors > 4)
        return false;

    const uint8_t algorithm = static_cast<uint8_t>(
        (hex_bits[51] << 2) + (hex_bits[52] >> 4));
    decoder->encryption_known = true;
    decoder->encrypted = algorithm != 0x80;
    decoder->stats.encryption_known = 1;
    decoder->stats.encrypted = decoder->encrypted ? 1 : 0;
    return true;
}

int decode_voice(p25_phase1_decoder *decoder, const bit_vector& frame)
{
    if (!decoder->encryption_known) {
        decoder->stats.voice_codewords_skipped_unknown_encryption += nof_voice_codewords;
        return P25_PHASE1_OK;
    }
    if (decoder->encrypted) {
        decoder->stats.voice_codewords_skipped_encrypted += nof_voice_codewords;
        return P25_PHASE1_OK;
    }

    voice_codeword bits(voice_codeword_sz);
    std::array<uint8_t, P25_IMBE_CODEWORD_BYTES> packed{};
    std::array<int16_t, P25_IMBE_PCM_SAMPLES> pcm{};
    for (size_t frame_index = 0; frame_index < nof_voice_codewords; ++frame_index) {
        imbe_deinterleave(frame, bits, frame_index);
        packed.fill(0);
        for (size_t bit = 0; bit < voice_codeword_sz; ++bit) {
            packed[bit / 8] |= static_cast<uint8_t>(
                bits[bit] << (7 - (bit % 8)));
        }

        p25_imbe_stats imbe_stats{};
        if (p25_imbe_decoder_decode(decoder->imbe, packed.data(), pcm.data(),
                                    &imbe_stats) != 0)
            return P25_PHASE1_INTERNAL_ERROR;
        ++decoder->stats.voice_codewords_decoded;
        decoder->stats.imbe_corrected_errors += imbe_stats.corrected_errors;

        if (decoder->pcm_callback != nullptr &&
            decoder->pcm_callback(decoder->user_data, decoder->stats.last_nac,
                                  decoder->stats.last_duid,
                                  decoder->stats.last_bch_errors,
                                  imbe_stats.corrected_errors, pcm.data()) != 0)
            return P25_PHASE1_CALLBACK_FAILED;
    }
    return P25_PHASE1_OK;
}

int process_frame(p25_phase1_decoder *decoder)
{
    const uint32_t duid = decoder->framer.duid;
    const bit_vector& frame = decoder->framer.frame_body;

    ++decoder->stats.complete_frames;
    decoder->stats.last_nac = static_cast<uint16_t>(decoder->framer.nac);
    decoder->stats.last_duid = static_cast<uint8_t>(duid);
    decoder->stats.last_bch_errors = decoder->framer.bch_errors;

    if (duid == 3 || duid == 15) {
        decoder->encryption_known = false;
        decoder->encrypted = false;
        decoder->stats.encryption_known = 0;
        decoder->stats.encrypted = 0;
        return P25_PHASE1_OK;
    }

    if (duid == 5) {
        ++decoder->stats.ldu1_frames;
    } else if (duid == 10) {
        ++decoder->stats.ldu2_frames;
        read_ess(decoder, frame);
    } else {
        return P25_PHASE1_OK;
    }
    return decode_voice(decoder, frame);
}

} // namespace

extern "C" p25_phase1_decoder *p25_phase1_decoder_create(
    p25_phase1_pcm_callback pcm_callback, void *user_data)
{
    try {
        auto decoder = std::make_unique<p25_phase1_decoder>(pcm_callback, user_data);
        if (decoder->imbe == nullptr)
            return nullptr;
        return decoder.release();
    } catch (...) {
        return nullptr;
    }
}

extern "C" void p25_phase1_decoder_destroy(p25_phase1_decoder *decoder)
{
    delete decoder;
}

extern "C" int p25_phase1_decoder_reset(p25_phase1_decoder *decoder)
{
    if (decoder == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    if (p25_imbe_decoder_reset(decoder->imbe) != 0)
        return P25_PHASE1_INTERNAL_ERROR;

    decoder->framer.reset();
    decoder->stats = {};
    decoder->encryption_known = false;
    decoder->encrypted = false;
    return P25_PHASE1_OK;
}

extern "C" int p25_phase1_decoder_set_expected_nac(p25_phase1_decoder *decoder,
                                                    uint16_t nac)
{
    if (decoder == nullptr || nac > 0x0fff)
        return P25_PHASE1_INVALID_ARGUMENT;
    decoder->framer.set_nac(nac);
    return P25_PHASE1_OK;
}

extern "C" int p25_phase1_decoder_process_dibits(
    p25_phase1_decoder *decoder, const uint8_t *dibits, size_t count)
{
    if (decoder == nullptr || (count != 0 && dibits == nullptr))
        return P25_PHASE1_INVALID_ARGUMENT;
    for (size_t i = 0; i < count; ++i) {
        if (dibits[i] > 3)
            return P25_PHASE1_INVALID_ARGUMENT;
    }

    try {
        for (size_t i = 0; i < count; ++i) {
            ++decoder->stats.dibits_received;
            if (decoder->framer.rx_sym(dibits[i])) {
                const int result = process_frame(decoder);
                if (result != P25_PHASE1_OK)
                    return result;
            }
        }
        return P25_PHASE1_OK;
    } catch (...) {
        return P25_PHASE1_INTERNAL_ERROR;
    }
}

extern "C" int p25_phase1_decoder_get_stats(
    const p25_phase1_decoder *decoder, p25_phase1_stats *stats)
{
    if (decoder == nullptr || stats == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    *stats = decoder->stats;
    return P25_PHASE1_OK;
}
