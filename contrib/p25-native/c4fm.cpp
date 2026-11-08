/*
 * OP25 C4FM receive chain adapted from boatbod/op25 revision
 * 718e589a8ef075143b0c861e7d4a1b01eac0cf64. The band-edge FLL follows
 * GNU Radio 3.8.5.0's fll_band_edge_cc implementation.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "c4fm.h"

#include "fsk4.h"
#include "upstream/frame_sync_magics.h"
#include "upstream/p25_c4fm_rx_taps.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace {

constexpr float kSampleRate = 24000.0f;
constexpr float kDeviation = 600.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kQuadratureGain =
    kSampleRate / (2.0f * kPi * kDeviation);
constexpr float kFllSamplesPerSymbol = kSampleRate / 4800.0f;
constexpr float kFllRolloff = 0.2f;

float sinc(float value)
{
    const float argument = kPi * value;
    return value == 0.0f ? 1.0f : std::sin(argument) / argument;
}

template <size_t N>
std::complex<float> filter_complex(
    const std::array<std::complex<float>, N>& taps,
    const std::complex<float> *history, size_t position)
{
    std::complex<float> result{};
    for (size_t tap = 0; tap < N; ++tap) {
        const size_t index = (position + N - tap) % N;
        result += taps[tap] * history[index];
    }
    return result;
}

struct DibitMap {
    uint64_t sync;
    std::array<uint8_t, 4> mapping;
};

constexpr std::array<DibitMap, 5> kDibitMaps{{
    {P25_FRAME_SYNC_MAGIC, {0, 1, 2, 3}},
    {P25_FRAME_SYNC_REV_P, {2, 3, 0, 1}},
    {P25_FRAME_SYNC_X2400, {3, 2, 1, 0}},
    {P25_FRAME_SYNC_N1200, {1, 3, 0, 2}},
    {P25_FRAME_SYNC_P1200, {2, 0, 3, 1}},
}};

template <size_t N>
float filter_real(const std::array<float, N>& taps, const float *history,
                  size_t position)
{
    float result = 0.0f;
    for (size_t tap = 0; tap < N; ++tap) {
        const size_t index = (position + N - tap) % N;
        result += taps[tap] * history[index];
    }
    return result;
}

template <size_t N>
std::complex<float> filter_complex(const std::array<float, N>& taps,
                                   const std::complex<float> *history,
                                   size_t position)
{
    std::complex<float> result{};
    for (size_t tap = 0; tap < N; ++tap) {
        const size_t index = (position + N - tap) % N;
        result += taps[tap] * history[index];
    }
    return result;
}

uint8_t slice_fsk4(float symbol)
{
    uint8_t dibit = 3;
    if (2.0f <= symbol && symbol < 4.0f)
        dibit = 1;
    if (-2.0f <= symbol && symbol < 0.0f)
        dibit = 2;
    if (0.0f <= symbol && symbol < 2.0f)
        dibit = 0;
    return dibit;
}

} // namespace

p25_band_edge_fll::p25_band_edge_fll(float bandwidth_divisor)
{
    if (!std::isfinite(bandwidth_divisor) || bandwidth_divisor <= 0.0f)
        throw std::invalid_argument("invalid band-edge FLL bandwidth divisor");
    const float bandwidth =
        2.0f * kPi / kFllSamplesPerSymbol / bandwidth_divisor;
    constexpr int half_filter = static_cast<int>(
        (kFilterSize / kFllSamplesPerSymbol) + 0.5f);
    std::array<float, kFilterSize> baseband_taps{};
    float power = 0.0f;
    for (size_t i = 0; i < kFilterSize; ++i) {
        const float k = -half_filter +
            static_cast<float>(i) * 2.0f / kFllSamplesPerSymbol;
        const float tap = sinc(kFllRolloff * k - 0.5f) +
                          sinc(kFllRolloff * k + 0.5f);
        baseband_taps[i] = tap;
        power += tap;
    }

    constexpr int center = (kFilterSize - 1) / 2;
    for (size_t i = 0; i < kFilterSize; ++i) {
        const float tap = baseband_taps[i] / power;
        const float k = (static_cast<int>(i) - center) /
                        (2.0f * kFllSamplesPerSymbol);
        const float angle = -2.0f * kPi * (1.0f + kFllRolloff) * k;
        const std::complex<float> lower =
            tap * std::complex<float>(std::cos(angle), std::sin(angle));
        lower_taps_[kFilterSize - i - 1] = lower;
        upper_taps_[kFilterSize - i - 1] = std::conj(lower);
    }

    constexpr float damping = 0.7071067811865475f;
    const float denominator = 1.0f + 2.0f * damping * bandwidth +
                              bandwidth * bandwidth;
    alpha_ = 4.0f * damping * bandwidth / denominator;
    beta_ = 4.0f * bandwidth * bandwidth / denominator;
    reset();
}

void p25_band_edge_fll::reset()
{
    history_.fill({});
    position_ = 0;
    phase_ = 0.0f;
    frequency_ = 0.0f;
}

float p25_band_edge_fll::frequency_offset_hz() const
{
    return -frequency_ * kSampleRate / (2.0f * kPi);
}

std::complex<float> p25_band_edge_fll::process(std::complex<float> sample)
{
    const std::complex<float> nco(std::cos(phase_), std::sin(phase_));
    const std::complex<float> corrected = sample * nco;
    history_[position_] = corrected;

    const std::complex<float> edge_lower =
        filter_complex(lower_taps_, history_.data(), position_);
    const std::complex<float> edge_upper =
        filter_complex(upper_taps_, history_.data(), position_);
    const float error = std::norm(edge_upper) - std::norm(edge_lower);
    frequency_ += beta_ * error;
    phase_ += frequency_ + alpha_ * error;
    constexpr float two_pi = 2.0f * kPi;
    if (phase_ > two_pi)
        phase_ -= two_pi;
    else if (phase_ < -two_pi)
        phase_ += two_pi;
    const float max_frequency = two_pi * 2.0f / kFllSamplesPerSymbol;
    frequency_ = std::clamp(frequency_, -max_frequency, max_frequency);
    position_ = (position_ + 1) % kFilterSize;
    return corrected;
}

uint8_t p25_dibit_mapper::process(uint8_t raw_dibit)
{
    // Detect sync in raw slicer space, then map subsequent dibits.
    raw_dibit_accum_ = (raw_dibit_accum_ << 2) | (raw_dibit & 0x3);
    const uint64_t sync = raw_dibit_accum_ & P25_FRAME_SYNC_MASK;
    for (size_t i = 0; i < kDibitMaps.size(); ++i) {
        if (sync == kDibitMaps[i].sync) {
            map_index_ = i;
            ++sync_count_;
            break;
        }
    }
    return kDibitMaps[map_index_].mapping[raw_dibit & 0x3];
}

void p25_dibit_mapper::reset()
{
    raw_dibit_accum_ = 0;
    sync_count_ = 0;
    map_index_ = 0;
}

p25_c4fm_decoder_impl::p25_c4fm_decoder_impl(
    p25_phase1_pcm_callback callback, void *user_data)
    : fsk4_(24000.0f, 4800.0f),
      phase1_(p25_phase1_decoder_create(callback, user_data))
{
    if (phase1_ == nullptr)
        throw std::runtime_error("could not create P25 Phase 1 decoder");
}

p25_c4fm_decoder_impl::~p25_c4fm_decoder_impl()
{
    p25_phase1_decoder_destroy(phase1_);
}

int p25_c4fm_decoder_impl::reset()
{
    const int result = p25_phase1_decoder_reset(phase1_);
    if (result != P25_PHASE1_OK)
        return result;

    fsk4_.reset();
    dibit_mapper_.reset();
    std::fill(std::begin(decimator_history_), std::end(decimator_history_),
              std::complex<float>{});
    std::fill(std::begin(if_history_), std::end(if_history_),
              std::complex<float>{});
    std::fill(std::begin(c4fm_history_), std::end(c4fm_history_), 0.0f);
    decimator_position_ = 0;
    if_position_ = 0;
    c4fm_position_ = 0;
    input_samples_ = 0;
    symbols_ = 0;
    agc_power_ = 0.0;
    fll_.reset();
    previous_sample_ = {};
    have_previous_sample_ = false;
    return P25_PHASE1_OK;
}

int p25_c4fm_decoder_impl::set_expected_nac(uint16_t nac)
{
    return p25_phase1_decoder_set_expected_nac(phase1_, nac);
}

int p25_c4fm_decoder_impl::process(const std::complex<float> *samples,
                                   size_t count)
{
    if (count != 0 && samples == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(samples[i].real()) || !std::isfinite(samples[i].imag()))
            return P25_PHASE1_INVALID_ARGUMENT;
    }

    for (size_t i = 0; i < count; ++i) {
        const uint64_t sample_index = input_samples_++;
        decimator_history_[decimator_position_] = samples[i];
        const std::complex<float> decimated =
            filter_complex(P25_C4FM_DECIMATOR_TAPS, decimator_history_,
                           decimator_position_);
        decimator_position_ = (decimator_position_ + 1) %
                              std::size(decimator_history_);
        if ((sample_index & 1U) != 0)
            continue;

        if_history_[if_position_] = decimated;
        const std::complex<float> filtered =
            filter_complex(P25_C4FM_IF_TAPS, if_history_, if_position_);
        if_position_ = (if_position_ + 1) % std::size(if_history_);
        const double magnitude_squared = std::norm(filtered);
        agc_power_ = (1.0 - 0.45) * agc_power_ + 0.45 * magnitude_squared;
        const float agc_gain = static_cast<float>(
            0.85 / (std::sqrt(agc_power_) + 1e-18));
        const std::complex<float> leveled =
            fll_.process(filtered * agc_gain);
        if (!have_previous_sample_) {
            previous_sample_ = leveled;
            have_previous_sample_ = true;
            continue;
        }

        const float product_real =
            leveled.real() * previous_sample_.real() +
            leveled.imag() * previous_sample_.imag();
        const float product_imag =
            leveled.imag() * previous_sample_.real() -
            leveled.real() * previous_sample_.imag();
        previous_sample_ = leveled;
        const float discriminator =
            std::atan2(product_imag, product_real) * kQuadratureGain;

        c4fm_history_[c4fm_position_] = discriminator;
        const float shaped =
            filter_real(P25_C4FM_RX_TAPS, c4fm_history_, c4fm_position_);
        c4fm_position_ = (c4fm_position_ + 1) % std::size(c4fm_history_);

        float symbol;
        if (fsk4_.process(&shaped, 1, &symbol) == 0)
            continue;

        const uint8_t raw_dibit = slice_fsk4(symbol);
        const uint8_t dibit = dibit_mapper_.process(raw_dibit);
        ++symbols_;
        const int phase1_result =
            p25_phase1_decoder_process_dibits(phase1_, &dibit, 1);
        if (phase1_result != P25_PHASE1_OK)
            return phase1_result;
    }
    return P25_PHASE1_OK;
}

int p25_c4fm_decoder_impl::process_s16be(const uint8_t *samples, size_t count)
{
    if ((count != 0 && samples == nullptr) ||
        count > std::numeric_limits<size_t>::max() / 4)
        return P25_PHASE1_INVALID_ARGUMENT;

    constexpr size_t kBlockSamples = 512;
    std::complex<float> converted[kBlockSamples];
    size_t offset = 0;
    while (offset < count) {
        const size_t block_count =
            std::min(kBlockSamples, count - offset);
        for (size_t i = 0; i < block_count; ++i) {
            const uint8_t *sample = samples + (offset + i) * 4;
            const uint32_t raw_i =
                (static_cast<uint32_t>(sample[0]) << 8) | sample[1];
            const uint32_t raw_q =
                (static_cast<uint32_t>(sample[2]) << 8) | sample[3];
            const int32_t signed_i =
                raw_i < 0x8000 ? static_cast<int32_t>(raw_i)
                               : static_cast<int32_t>(raw_i) - 0x10000;
            const int32_t signed_q =
                raw_q < 0x8000 ? static_cast<int32_t>(raw_q)
                               : static_cast<int32_t>(raw_q) - 0x10000;
            converted[i] = {
                static_cast<float>(signed_i) / 32768.0f,
                static_cast<float>(signed_q) / 32768.0f
            };
        }

        const int result = process(converted, block_count);
        if (result != P25_PHASE1_OK)
            return result;
        offset += block_count;
    }
    return P25_PHASE1_OK;
}

int p25_c4fm_decoder_impl::process_interleaved_f32(const float *samples,
                                                   size_t count)
{
    if ((count != 0 && samples == nullptr) ||
        count > std::numeric_limits<size_t>::max() / 2)
        return P25_PHASE1_INVALID_ARGUMENT;
    for (size_t i = 0; i < count * 2; ++i) {
        if (!std::isfinite(samples[i]))
            return P25_PHASE1_INVALID_ARGUMENT;
    }

    constexpr size_t kBlockSamples = 512;
    std::complex<float> converted[kBlockSamples];
    size_t offset = 0;
    while (offset < count) {
        const size_t block_count = std::min(kBlockSamples, count - offset);
        for (size_t i = 0; i < block_count; ++i)
            converted[i] = {samples[(offset + i) * 2],
                            samples[(offset + i) * 2 + 1]};
        const int result = process(converted, block_count);
        if (result != P25_PHASE1_OK)
            return result;
        offset += block_count;
    }
    return P25_PHASE1_OK;
}

int p25_c4fm_decoder_impl::get_stats(
    uint64_t *complex_samples, uint64_t *symbols,
    p25_phase1_stats *phase1_stats) const
{
    if (complex_samples == nullptr || symbols == nullptr || phase1_stats == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    *complex_samples = input_samples_;
    *symbols = symbols_;
    return p25_phase1_decoder_get_stats(phase1_, phase1_stats);
}

float p25_c4fm_decoder_impl::frequency_offset_hz() const
{
    return fll_.frequency_offset_hz() +
           static_cast<float>(fsk4_.fine_frequency_correction() * kDeviation);
}
