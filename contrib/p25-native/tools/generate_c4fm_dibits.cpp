#include "test_frame_builder.h"

#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "usage: generate_c4fm_dibits OUTPUT\n";
        return 2;
    }

    std::vector<uint8_t> dibits(128);
    constexpr std::array<uint8_t, 4> preamble = {0, 3, 1, 2};
    for (size_t i = 0; i < dibits.size(); ++i)
        dibits[i] = preamble[i % preamble.size()];
    for (uint8_t dibit : p25_test::make_frame(10, 0x80))
        dibits.push_back(dibit);
    for (uint8_t dibit : p25_test::make_frame(5))
        dibits.push_back(dibit);
    dibits.insert(dibits.end(), 64, 0);

    std::ofstream output(argv[1], std::ios::binary);
    if (!output ||
        !output.write(reinterpret_cast<const char *>(dibits.data()),
                      static_cast<std::streamsize>(dibits.size()))) {
        std::cerr << "could not write dibit fixture\n";
        return 1;
    }
    return 0;
}
