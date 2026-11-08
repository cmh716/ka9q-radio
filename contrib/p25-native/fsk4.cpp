/*
 * OP25 FSK4 MMSE timing and level tracking, adapted from
 * boatbod/op25 revision 718e589a8ef075143b0c861e7d4a1b01eac0cf64.
 * Copyright 2006, 2007 Frank (Radio Rausch); Copyright 2011 Steve Glass.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "fsk4.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

#include "upstream/fsk4_mmse_taps.h"

p25_fsk4_demod::p25_fsk4_demod(float sample_rate_hz, float symbol_rate_hz)
    : symbol_time_(0.0)
{
    if (!std::isfinite(sample_rate_hz) || !std::isfinite(symbol_rate_hz) ||
        sample_rate_hz < symbol_rate_hz || symbol_rate_hz <= 0.0f)
        throw std::invalid_argument("invalid FSK4 sample or symbol rate");
    symbol_time_ = static_cast<double>(symbol_rate_hz / sample_rate_hz);
    reset();
}

void p25_fsk4_demod::reset()
{
    symbol_clock_ = 0.0f;
    symbol_spread_ = 2.0f;
    fine_frequency_correction_ = 0.0f;
    coarse_frequency_correction_ = 0.0f;
    std::fill(std::begin(history_), std::end(history_), 0.0f);
    history_last_ = 0;
}

size_t p25_fsk4_demod::process(const float *input, size_t count, float *symbols)
{
    if (count != 0 && (input == nullptr || symbols == nullptr))
        throw std::invalid_argument("null FSK4 process buffer");

    size_t output_count = 0;
    for (size_t sample = 0; sample < count; ++sample) {
        symbol_clock_ += symbol_time_;
        history_[history_last_++] = input[sample];
        history_last_ %= P25_FSK4_NTAPS;

        if (symbol_clock_ <= 1.0f)
            continue;

        symbol_clock_ -= 1.0f;
        int phase = static_cast<int>(
            std::floor(0.5 + P25_FSK4_NSTEPS * (symbol_clock_ / symbol_time_)));
        int next_phase = phase + 1;
        if (phase >= P25_FSK4_NSTEPS) {
            phase = P25_FSK4_NSTEPS - 1;
            next_phase = P25_FSK4_NSTEPS;
        }

        double interpolated = 0.0;
        double next_interpolated = 0.0;
        size_t history_index = history_last_;
        for (int tap = 0; tap < P25_FSK4_NTAPS; ++tap) {
            interpolated += P25_FSK4_TAPS[phase][tap] * history_[history_index];
            next_interpolated +=
                P25_FSK4_TAPS[next_phase][tap] * history_[history_index];
            history_index = (history_index + 1) % P25_FSK4_NTAPS;
        }

        interpolated -= fine_frequency_correction_;
        next_interpolated -= fine_frequency_correction_;
        symbols[output_count++] = static_cast<float>(2.0 * interpolated / symbol_spread_);

        double symbol_error;
        constexpr double k_symbol_spread = 0.0100;
        if (interpolated < -symbol_spread_) {
            symbol_error = interpolated + 1.5 * symbol_spread_;
            symbol_spread_ -= symbol_error * 0.5 * k_symbol_spread;
        } else if (interpolated < 0.0) {
            symbol_error = interpolated + 0.5 * symbol_spread_;
            symbol_spread_ -= symbol_error * k_symbol_spread;
        } else if (interpolated < symbol_spread_) {
            symbol_error = interpolated - 0.5 * symbol_spread_;
            symbol_spread_ += symbol_error * k_symbol_spread;
        } else {
            symbol_error = interpolated - 1.5 * symbol_spread_;
            symbol_spread_ += symbol_error * 0.5 * k_symbol_spread;
        }

        constexpr double k_symbol_timing = 0.025;
        if (next_interpolated < interpolated)
            symbol_clock_ += symbol_error * k_symbol_timing;
        else
            symbol_clock_ -= symbol_error * k_symbol_timing;

        symbol_spread_ = std::clamp(symbol_spread_, 1.6, 2.4);
        constexpr double k_coarse_frequency = 0.00125;
        coarse_frequency_correction_ +=
            (fine_frequency_correction_ - coarse_frequency_correction_) *
            k_coarse_frequency;
        constexpr double k_fine_frequency = 0.125;
        fine_frequency_correction_ += symbol_error * k_fine_frequency;
    }

    return output_count;
}

bool p25_fsk4_demod::take_frequency_correction(float *correction)
{
    if (correction == nullptr)
        throw std::invalid_argument("null FSK4 frequency-correction buffer");
    if (coarse_frequency_correction_ > -1.66 &&
        coarse_frequency_correction_ < 1.66)
        return false;
    *correction = coarse_frequency_correction_;
    coarse_frequency_correction_ = 0.0f;
    return true;
}
