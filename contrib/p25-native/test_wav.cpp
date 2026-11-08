#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace {

void require(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

uint16_t read_le16(const uint8_t *source)
{
    return static_cast<uint16_t>(source[0]) |
           (static_cast<uint16_t>(source[1]) << 8);
}

uint32_t read_le32(const uint8_t *source)
{
    return static_cast<uint32_t>(source[0]) |
           (static_cast<uint32_t>(source[1]) << 8) |
           (static_cast<uint32_t>(source[2]) << 16) |
           (static_cast<uint32_t>(source[3]) << 24);
}

} // namespace

int main()
{
    std::ifstream input("build/test-c4fm.wav", std::ios::binary);
    require(input.good(), "offline receiver produced a WAV file");

    std::array<uint8_t, 44> header{};
    input.read(reinterpret_cast<char *>(header.data()), header.size());
    require(input.gcount() == static_cast<std::streamsize>(header.size()),
            "WAV header is complete");
    require(std::memcmp(header.data(), "RIFF", 4) == 0 &&
                std::memcmp(header.data() + 8, "WAVEfmt ", 8) == 0 &&
                std::memcmp(header.data() + 36, "data", 4) == 0,
            "WAV chunk identifiers are valid");
    require(read_le32(header.data() + 4) == 5796 &&
                read_le32(header.data() + 16) == 16 &&
                read_le16(header.data() + 20) == 1 &&
                read_le16(header.data() + 22) == 1 &&
                read_le32(header.data() + 24) == 8000 &&
                read_le32(header.data() + 28) == 16000 &&
                read_le16(header.data() + 32) == 2 &&
                read_le16(header.data() + 34) == 16 &&
                read_le32(header.data() + 40) == 5760,
            "WAV metadata describes 8 kHz mono signed 16-bit PCM");
    input.seekg(0, std::ios::end);
    require(input.tellg() == 5804, "WAV data and RIFF sizes match the payload");

    std::puts("native offline WAV output test passed");
    return 0;
}
