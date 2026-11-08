#include "imbe.h"

#include <memory>
#include <new>
#include <vector>

#include "op25_imbe_frame.h"
#include "imbe_vocoder/imbe_vocoder.h"

struct p25_imbe_decoder {
    std::unique_ptr<imbe_vocoder> vocoder;
    BasicOpContext arithmetic{0, 0};
};

class BasicOpContextScope {
public:
    explicit BasicOpContextScope(BasicOpContext *context)
        : previous_(basic_op_set_context(context))
    {
    }

    ~BasicOpContextScope()
    {
        basic_op_set_context(previous_);
    }

private:
    BasicOpContext *previous_;
};

extern "C" p25_imbe_decoder *p25_imbe_decoder_create(void)
{
    try {
        auto decoder = std::make_unique<p25_imbe_decoder>();
        BasicOpContextScope context(&decoder->arithmetic);
        decoder->vocoder = std::make_unique<imbe_vocoder>();
        return decoder.release();
    } catch (...) {
        return nullptr;
    }
}

extern "C" void p25_imbe_decoder_destroy(p25_imbe_decoder *decoder)
{
    delete decoder;
}

extern "C" int p25_imbe_decoder_reset(p25_imbe_decoder *decoder)
{
    if (decoder == nullptr)
        return -1;
    try {
        BasicOpContext arithmetic{0, 0};
        std::unique_ptr<imbe_vocoder> vocoder;
        {
            BasicOpContextScope context(&arithmetic);
            vocoder = std::make_unique<imbe_vocoder>();
        }
        decoder->vocoder = std::move(vocoder);
        decoder->arithmetic = arithmetic;
        return 0;
    } catch (...) {
        return -1;
    }
}

extern "C" int p25_imbe_decoder_decode(p25_imbe_decoder *decoder,
                                       const uint8_t codeword[P25_IMBE_CODEWORD_BYTES],
                                       int16_t pcm[P25_IMBE_PCM_SAMPLES],
                                       p25_imbe_stats *stats)
{
    if (decoder == nullptr || !decoder->vocoder || codeword == nullptr || pcm == nullptr)
        return -1;
    try {
        BasicOpContextScope context(&decoder->arithmetic);
        voice_codeword bits(voice_codeword_sz);
        for (size_t i = 0; i < voice_codeword_sz; ++i)
            bits[i] = (codeword[i / 8] >> (7 - (i % 8))) & 1;

        uint32_t parameters[8];
        uint32_t first_errors;
        uint32_t corrected_errors;
        imbe_header_decode(bits, parameters[0], parameters[1], parameters[2],
                           parameters[3], parameters[4], parameters[5],
                           parameters[6], parameters[7], first_errors,
                           corrected_errors);

        int16_t frame[8];
        for (size_t i = 0; i < 8; ++i)
            frame[i] = static_cast<int16_t>(parameters[i]);
        frame[7] >>= 1;
        decoder->vocoder->imbe_decode(frame, pcm);

        if (stats != nullptr) {
            stats->first_golay_errors = first_errors;
            stats->corrected_errors = corrected_errors;
        }
        return 0;
    } catch (...) {
        return -1;
    }
}
