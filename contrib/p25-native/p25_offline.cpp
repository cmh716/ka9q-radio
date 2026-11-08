#include "c4fm.h"

#include <array>
#include <cerrno>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <sys/stat.h>
#include <unistd.h>

namespace {

struct file_closer {
    void operator()(FILE *file) const
    {
        if (file != nullptr)
            std::fclose(file);
    }
};

using file_ptr = std::unique_ptr<FILE, file_closer>;

struct wav_output {
    FILE *file;
    uint64_t data_bytes = 0;
};

void store_le16(uint8_t *destination, uint16_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
}

void store_le32(uint8_t *destination, uint32_t value)
{
    destination[0] = static_cast<uint8_t>(value);
    destination[1] = static_cast<uint8_t>(value >> 8);
    destination[2] = static_cast<uint8_t>(value >> 16);
    destination[3] = static_cast<uint8_t>(value >> 24);
}

bool write_wav_header(FILE *file, uint32_t pcm_bytes)
{
    std::array<uint8_t, 44> header{};
    std::memcpy(header.data(), "RIFF", 4);
    store_le32(header.data() + 4, pcm_bytes + 36);
    std::memcpy(header.data() + 8, "WAVEfmt ", 8);
    store_le32(header.data() + 16, 16);
    store_le16(header.data() + 20, 1);
    store_le16(header.data() + 22, 1);
    store_le32(header.data() + 24, 8000);
    store_le32(header.data() + 28, 16000);
    store_le16(header.data() + 32, 2);
    store_le16(header.data() + 34, 16);
    std::memcpy(header.data() + 36, "data", 4);
    store_le32(header.data() + 40, pcm_bytes);
    return std::fwrite(header.data(), 1, header.size(), file) == header.size();
}

int write_pcm(void *user_data, uint16_t, uint8_t, uint32_t, uint32_t,
              const int16_t pcm[P25_IMBE_PCM_SAMPLES])
{
    auto *output = static_cast<wav_output *>(user_data);
    constexpr uint64_t block_bytes = P25_IMBE_PCM_SAMPLES * sizeof(int16_t);
    if (output->data_bytes >
        std::numeric_limits<uint32_t>::max() - 36 - block_bytes)
        return -1;

    std::array<uint8_t, block_bytes> bytes{};
    for (size_t i = 0; i < P25_IMBE_PCM_SAMPLES; ++i)
        store_le16(bytes.data() + 2 * i, static_cast<uint16_t>(pcm[i]));
    if (std::fwrite(bytes.data(), 1, bytes.size(), output->file) != bytes.size())
        return -1;
    output->data_bytes += block_bytes;
    return 0;
}

bool parse_nac(const char *text, uint16_t *nac)
{
    char *end = nullptr;
    errno = 0;
    const unsigned long value = std::strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value > 0x0fff)
        return false;
    *nac = static_cast<uint16_t>(value);
    return true;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 4) {
        std::fprintf(stderr,
                     "usage: p25-offline INPUT.s16be OUTPUT.wav NAC\n"
                     "INPUT is raw 48 kHz interleaved signed 16-bit big-endian IQ.\n"
                     "NAC accepts decimal or 0x-prefixed hexadecimal.\n");
        return 2;
    }
    if (std::strcmp(argv[1], argv[2]) == 0) {
        std::fprintf(stderr, "input and output paths must differ\n");
        return 2;
    }

    uint16_t nac;
    if (!parse_nac(argv[3], &nac)) {
        std::fprintf(stderr, "invalid NAC: %s\n", argv[3]);
        return 2;
    }

    file_ptr input(std::fopen(argv[1], "rb"));
    if (!input) {
        std::fprintf(stderr, "cannot open IQ input %s: %s\n", argv[1],
                     std::strerror(errno));
        return 1;
    }
    struct stat input_stat;
    if (fstat(fileno(input.get()), &input_stat) != 0) {
        std::fprintf(stderr, "cannot inspect IQ input %s: %s\n", argv[1],
                     std::strerror(errno));
        return 1;
    }
    struct stat output_stat;
    if (stat(argv[2], &output_stat) == 0) {
        if (input_stat.st_dev == output_stat.st_dev &&
            input_stat.st_ino == output_stat.st_ino) {
            std::fprintf(stderr, "input and output refer to the same file\n");
            return 2;
        }
    } else if (errno != ENOENT) {
        std::fprintf(stderr, "cannot inspect WAV output %s: %s\n", argv[2],
                     std::strerror(errno));
        return 1;
    }
    file_ptr output_file(std::fopen(argv[2], "wb+"));
    if (!output_file) {
        std::fprintf(stderr, "cannot open WAV output %s: %s\n", argv[2],
                     std::strerror(errno));
        return 1;
    }
    if (!write_wav_header(output_file.get(), 0)) {
        std::fprintf(stderr, "cannot write WAV header: %s\n",
                     std::strerror(errno));
        return 1;
    }

    wav_output wav{output_file.get()};
    try {
        p25_c4fm_decoder_impl decoder(write_pcm, &wav);
        if (decoder.set_expected_nac(nac) != P25_PHASE1_OK) {
            std::fprintf(stderr, "cannot configure expected NAC\n");
            return 1;
        }

        std::array<uint8_t, 4096> input_buffer{};
        uint64_t input_bytes = 0;
        for (;;) {
            const size_t bytes =
                std::fread(input_buffer.data(), 1, input_buffer.size(), input.get());
            if (bytes == 0) {
                if (std::ferror(input.get())) {
                    std::fprintf(stderr, "error reading IQ input: %s\n",
                                 std::strerror(errno));
                    return 1;
                }
                break;
            }
            if ((bytes & 3U) != 0) {
                std::fprintf(stderr,
                             "IQ input ends on a partial complex sample "
                             "(read %zu bytes)\n",
                             bytes);
                return 1;
            }
            input_bytes += bytes;
            const int result =
                decoder.process_s16be(input_buffer.data(), bytes / 4);
            if (result != P25_PHASE1_OK) {
                std::fprintf(stderr, "decoder stopped with error %d\n", result);
                return 1;
            }
        }
        if (input_bytes == 0) {
            std::fprintf(stderr, "IQ input is empty\n");
            return 1;
        }
        if (std::fseek(output_file.get(), 0, SEEK_SET) != 0 ||
            !write_wav_header(output_file.get(),
                              static_cast<uint32_t>(wav.data_bytes)) ||
            std::fflush(output_file.get()) != 0) {
            std::fprintf(stderr, "cannot finalize WAV output: %s\n",
                         std::strerror(errno));
            return 1;
        }

        p25_phase1_stats stats{};
        uint64_t samples = 0;
        uint64_t symbols = 0;
        if (decoder.get_stats(&samples, &symbols, &stats) != P25_PHASE1_OK) {
            std::fprintf(stderr, "cannot read decoder statistics\n");
            return 1;
        }
        std::fprintf(stderr,
                     "IQ samples=%" PRIu64 " symbols=%" PRIu64
                     " frames=%" PRIu64 " voice_codewords=%" PRIu64
                     " BCH_errors=%u PCM_bytes=%" PRIu64 "\n",
                     samples, symbols, stats.complete_frames,
                     stats.voice_codewords_decoded, stats.last_bch_errors,
                     wav.data_bytes);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "decoder initialization failed: %s\n", error.what());
        return 1;
    }
    return 0;
}
