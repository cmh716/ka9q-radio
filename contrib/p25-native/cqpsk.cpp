/*
 * OP25 CQPSK receive chain adapted from boatbod/op25 revision
 * 718e589a8ef075143b0c861e7d4a1b01eac0cf64. Gardner and Costas loop
 * behavior follows OP25; the MMSE interpolator coefficients are generated
 * from GNU Radio 3.10.9.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cqpsk.h"

#include "upstream/gnuradio_mmse_taps.h"
#include "upstream/p25_c4fm_rx_taps.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

constexpr float kSampleRate = 24000.0f;
constexpr float kSymbolRate = 4800.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kAgcAlpha = 0.45f;
constexpr float kAgcReference = 0.85f;
constexpr float kGainMu = 0.025f;
constexpr float kGainOmega = 0.1f * kGainMu * kGainMu;
constexpr float kOmegaRelativeLimit = 0.002f;
constexpr float kCostasBandwidth = 0.008f;
constexpr float kCostasMaxPhase = kTwoPi / 4.0f;

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

template <size_t N>
std::complex<float> filter_complex(
    const std::array<float, N>& taps,
    const std::complex<float> *history, size_t position)
{
    std::complex<float> result{};
    for (size_t tap = 0; tap < N; ++tap) {
        const size_t index = (position + N - tap) % N;
        result += taps[tap] * history[index];
    }
    return result;
}

uint8_t slice_qpsk(float symbol)
{
    uint8_t dibit = 3;
    if (-2.0f <= symbol && symbol < 0.0f)
        dibit = 2;
    if (0.0f <= symbol && symbol < 2.0f)
        dibit = 0;
    if (2.0f <= symbol && symbol < 4.0f)
        dibit = 1;
    return dibit;
}

} // namespace

p25_cqpsk_decoder_impl::p25_cqpsk_decoder_impl(
    p25_phase1_pcm_callback callback, void *user_data)
    : fll_(250.0f),
      phase1_(p25_phase1_decoder_create(callback, user_data))
{
    if (phase1_ == nullptr)
        throw std::runtime_error("could not create P25 Phase 1 decoder");

    constexpr float damping = 0.7071067811865475f;
    const float denominator = 1.0f + 2.0f * damping * kCostasBandwidth +
                              kCostasBandwidth * kCostasBandwidth;
    costas_alpha_ = 4.0f * damping * kCostasBandwidth / denominator;
    costas_beta_ = 4.0f * kCostasBandwidth * kCostasBandwidth / denominator;
}

p25_cqpsk_decoder_impl::~p25_cqpsk_decoder_impl()
{
    p25_phase1_decoder_destroy(phase1_);
}

int p25_cqpsk_decoder_impl::reset()
{
    const int result = p25_phase1_decoder_reset(phase1_);
    if (result != P25_PHASE1_OK)
        return result;

    fll_.reset();
    dibit_mapper_.reset();
    std::fill(std::begin(decimator_history_), std::end(decimator_history_),
              std::complex<float>{});
    std::fill(std::begin(if_history_), std::end(if_history_),
              std::complex<float>{});
    timing_delay_.fill({});
    previous_diff_sample_ = {};
    decimator_position_ = 0;
    if_position_ = 0;
    timing_position_ = 0;
    input_samples_ = 0;
    symbols_ = 0;
    agc_power_ = 0.0;
    mu_ = 0.0f;
    omega_ = static_cast<float>(kTimingSamplesPerSymbol);
    last_symbol_i_ = 0.0f;
    last_symbol_q_ = 0.0f;
    costas_phase_ = 0.0f;
    costas_frequency_ = 0.0f;
    have_previous_diff_sample_ = false;
    return P25_PHASE1_OK;
}

int p25_cqpsk_decoder_impl::set_expected_nac(uint16_t nac)
{
    return p25_phase1_decoder_set_expected_nac(phase1_, nac);
}

std::complex<float> p25_cqpsk_decoder_impl::timing_interpolate(
    const std::complex<float> *input, float mu) const
{
    const long rounded_phase = std::lrint(mu * 128.0f);
    const size_t phase = static_cast<size_t>(
        std::clamp(rounded_phase, 0L, 128L));
    std::complex<float> result{};
    for (size_t tap = 0; tap < 8; ++tap)
        result += P25_GNURADIO_MMSE_TAPS[phase][tap] * input[tap];
    return result;
}

std::complex<float> p25_cqpsk_decoder_impl::timing_update()
{
    float half_omega = omega_ / 2.0f;
    size_t half_sps = static_cast<size_t>(std::floor(half_omega));
    float half_mu = mu_ + half_omega - static_cast<float>(half_sps);
    if (half_mu > 1.0f) {
        half_mu -= 1.0f;
        ++half_sps;
    }

    const std::complex<float> *delay = timing_delay_.data() + timing_position_;
    const std::complex<float> midpoint = timing_interpolate(delay, mu_);
    const std::complex<float> symbol =
        timing_interpolate(delay + half_sps, half_mu);
    float error =
        (last_symbol_i_ - symbol.real()) * midpoint.real() +
        (last_symbol_q_ - symbol.imag()) * midpoint.imag();
    if (!std::isfinite(error))
        error = 0.0f;
    error = std::clamp(error, -1.0f, 1.0f);
    last_symbol_i_ = symbol.real();
    last_symbol_q_ = symbol.imag();

    omega_ += kGainOmega * error * std::abs(symbol);
    const float omega_mid = static_cast<float>(kTimingSamplesPerSymbol);
    omega_ = omega_mid + std::clamp(
        omega_ - omega_mid, -kOmegaRelativeLimit, kOmegaRelativeLimit);
    mu_ += omega_ + kGainMu * error;
    return symbol;
}

std::complex<float> p25_cqpsk_decoder_impl::costas_process(
    std::complex<float> sample)
{
    const std::complex<float> nco(std::cos(costas_phase_),
                                  -std::sin(costas_phase_));
    const std::complex<float> corrected = sample * nco;
    const float error =
        (corrected.real() > 0.0f ? 1.0f : -1.0f) * corrected.imag() -
        (corrected.imag() > 0.0f ? 1.0f : -1.0f) * corrected.real();
    const float limited_error = std::clamp(error, -1.0f, 1.0f);
    costas_frequency_ = std::clamp(
        costas_frequency_ + costas_beta_ * limited_error, -1.0f, 1.0f);
    costas_phase_ = std::clamp(
        costas_phase_ + costas_frequency_ + costas_alpha_ * limited_error,
        -kCostasMaxPhase, kCostasMaxPhase);
    return corrected;
}

int p25_cqpsk_decoder_impl::process_symbol(std::complex<float> sample)
{
    if (!have_previous_diff_sample_) {
        previous_diff_sample_ = sample;
        have_previous_diff_sample_ = true;
        return P25_PHASE1_OK;
    }

    const std::complex<float> differential =
        sample * std::conj(previous_diff_sample_);
    previous_diff_sample_ = sample;
    const std::complex<float> corrected = costas_process(differential);
    constexpr float phase_scale = 4.0f / kPi;
    const float symbol =
        std::atan2(corrected.imag(), corrected.real()) * phase_scale;
    const uint8_t raw_dibit = slice_qpsk(symbol);
    const uint8_t dibit = dibit_mapper_.process(raw_dibit);
    ++symbols_;
    return p25_phase1_decoder_process_dibits(phase1_, &dibit, 1);
}

int p25_cqpsk_decoder_impl::process(const std::complex<float> *samples,
                                    size_t count)
{
    if (count != 0 && samples == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(samples[i].real()) ||
            !std::isfinite(samples[i].imag()))
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
        agc_power_ = (1.0 - kAgcAlpha) * agc_power_ +
                     kAgcAlpha * magnitude_squared;
        const float agc_gain = static_cast<float>(
            kAgcReference / (std::sqrt(agc_power_) + 1e-18));
        const std::complex<float> corrected =
            fll_.process(filtered * agc_gain);

        if (mu_ <= 1.0f) {
            const int result = process_symbol(timing_update());
            if (result != P25_PHASE1_OK)
                return result;
        }
        timing_delay_[timing_position_] = corrected;
        timing_delay_[timing_position_ + kTimingDelaySize] = corrected;
        timing_position_ = (timing_position_ + 1) % kTimingDelaySize;
        mu_ -= 1.0f;
        if (mu_ <= 1.0f) {
            const int result = process_symbol(timing_update());
            if (result != P25_PHASE1_OK)
                return result;
        }
    }
    return P25_PHASE1_OK;
}

int p25_cqpsk_decoder_impl::process_s16be(const uint8_t *samples,
                                          size_t count)
{
    if ((count != 0 && samples == nullptr) ||
        count > std::numeric_limits<size_t>::max() / 4)
        return P25_PHASE1_INVALID_ARGUMENT;

    constexpr size_t kBlockSamples = 512;
    std::complex<float> converted[kBlockSamples];
    size_t offset = 0;
    while (offset < count) {
        const size_t block_count = std::min(kBlockSamples, count - offset);
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

int p25_cqpsk_decoder_impl::process_interleaved_f32(const float *samples,
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

int p25_cqpsk_decoder_impl::get_stats(
    uint64_t *complex_samples, uint64_t *symbols,
    p25_phase1_stats *phase1_stats) const
{
    if (complex_samples == nullptr || symbols == nullptr ||
        phase1_stats == nullptr)
        return P25_PHASE1_INVALID_ARGUMENT;
    *complex_samples = input_samples_;
    *symbols = symbols_;
    return p25_phase1_decoder_get_stats(phase1_, phase1_stats);
}
