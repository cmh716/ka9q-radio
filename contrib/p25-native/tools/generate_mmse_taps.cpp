#include <gnuradio/filter/mmse_fir_interpolator_cc.h>

#include <array>
#include <cstdio>

int main()
{
    gr::filter::mmse_fir_interpolator_cc interpolator;
    if (interpolator.ntaps() != 8 || interpolator.nsteps() != 128) {
        std::fprintf(stderr,
                     "expected GNU Radio's 8-tap, 128-phase MMSE interpolator\n");
        return 1;
    }

    std::puts("/* Generated from GNU Radio 3.10.9 MMSE interpolator taps.");
    std::puts(" * SPDX-License-Identifier: GPL-3.0-or-later");
    std::puts(" */");
    std::puts("#ifndef KA9Q_P25_GNURADIO_MMSE_TAPS_H");
    std::puts("#define KA9Q_P25_GNURADIO_MMSE_TAPS_H");
    std::puts("#include <array>");
    std::puts("constexpr std::array<std::array<float, 8>, 129> "
              "P25_GNURADIO_MMSE_TAPS = {{");
    for (unsigned phase = 0; phase <= interpolator.nsteps(); ++phase) {
        std::printf("    {{");
        for (unsigned tap = 0; tap < interpolator.ntaps(); ++tap) {
            alignas(16) std::array<gr_complex, 8> input{};
            input[tap] = {1.0f, 0.0f};
            const float coefficient =
                interpolator.interpolate(input.data(),
                    static_cast<float>(phase) / interpolator.nsteps()).real();
            std::printf("%s%.9ef", tap == 0 ? "" : ", ", coefficient);
        }
        std::printf("}}, // %u/128\n", phase);
    }
    std::puts("}};");
    std::puts("#endif");
}
