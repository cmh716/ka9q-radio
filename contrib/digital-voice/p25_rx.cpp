// P25 Phase 1 IQ receiver: C4FM or CQPSK demodulation feeding p25_voice
// Adapted from OP25 (boatbod/op25 718e589a) p25_demodulator_dev.py and
// gardner/costas blocks; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <array>
#include <cmath>

#include "dv_decoder.h"
#include "dv_frontend.h"
#include "p25_voice.h"
#include "upstream/frame_sync_magics.h"
#include "upstream/gnuradio_mmse_taps.h"

namespace {

// CQPSK Gardner timing and Costas loop parameters
constexpr size_t kTimingSamplesPerSymbol = 5;
constexpr size_t kTimingDelaySize = 2 * kTimingSamplesPerSymbol;
constexpr float kGainMu = 0.025f;
constexpr float kGainOmega = 0.1f * kGainMu * kGainMu;
constexpr float kOmegaRelativeLimit = 0.002f;
constexpr float kCostasBandwidth = 0.008f;
constexpr float kCostasMaxPhase = kDvTwoPi / 4.0f;

// Detects the frame sync word in any of OP25's polarity/rotation variants
// and remaps subsequent dibits accordingly
class dibit_mapper {
public:
  uint8_t process(uint8_t raw)
  {
    struct map { uint64_t sync; std::array<uint8_t, 4> mapping; };
    static constexpr std::array<map, 5> maps{{
      {P25_FRAME_SYNC_MAGIC, {0, 1, 2, 3}},
      {P25_FRAME_SYNC_REV_P, {2, 3, 0, 1}},
      {P25_FRAME_SYNC_X2400, {3, 2, 1, 0}},
      {P25_FRAME_SYNC_N1200, {1, 3, 0, 2}},
      {P25_FRAME_SYNC_P1200, {2, 0, 3, 1}},
    }};
    accum_ = (accum_ << 2) | (raw & 3);
    uint64_t const sync = accum_ & P25_FRAME_SYNC_MASK;
    for (size_t i = 0; i < maps.size(); i++) {
      if (sync == maps[i].sync) {
        index_ = i;
        syncs++;
        break;
      }
    }
    return maps[index_].mapping[raw & 3];
  }
  uint64_t syncs = 0;

private:
  uint64_t accum_ = 0;
  size_t index_ = 0;
};

// CQPSK: Gardner timing, differential decode, Costas loop
class cqpsk_demod {
public:
  cqpsk_demod()
  {
    constexpr float damping = 0.7071067811865475f;
    float const denominator = 1.0f + 2.0f * damping * kCostasBandwidth
                            + kCostasBandwidth * kCostasBandwidth;
    costas_alpha_ = 4.0f * damping * kCostasBandwidth / denominator;
    costas_beta_ = 4.0f * kCostasBandwidth * kCostasBandwidth / denominator;
  }

  // Calls emit(dibit) for each recovered symbol (0, 1 or 2 per sample)
  template <typename Emit>
  void process(cfloat sample, Emit&& emit)
  {
    if (mu_ <= 1.0f)
      symbol(timing_update(), emit);
    timing_delay_[timing_position_] = sample;
    timing_delay_[timing_position_ + kTimingDelaySize] = sample;
    timing_position_ = (timing_position_ + 1) % kTimingDelaySize;
    mu_ -= 1.0f;
    if (mu_ <= 1.0f)
      symbol(timing_update(), emit);
  }

private:
  cfloat interpolate(cfloat const *input, float frac) const
  {
    size_t const phase = static_cast<size_t>(std::clamp(std::lrint(frac * 128.0f), 0L, 128L));
    cfloat result{};
    for (size_t tap = 0; tap < 8; tap++)
      result += P25_GNURADIO_MMSE_TAPS[phase][tap] * input[tap];
    return result;
  }

  cfloat timing_update()
  {
    float const half_omega = omega_ / 2.0f;
    size_t half_sps = static_cast<size_t>(std::floor(half_omega));
    float half_mu = mu_ + half_omega - static_cast<float>(half_sps);
    if (half_mu > 1.0f) {
      half_mu -= 1.0f;
      half_sps++;
    }
    cfloat const *delay = timing_delay_.data() + timing_position_;
    cfloat const midpoint = interpolate(delay, mu_);
    cfloat const sym = interpolate(delay + half_sps, half_mu);
    float error = (last_symbol_i_ - sym.real()) * midpoint.real()
                + (last_symbol_q_ - sym.imag()) * midpoint.imag();
    if (!std::isfinite(error))
      error = 0.0f;
    error = std::clamp(error, -1.0f, 1.0f);
    last_symbol_i_ = sym.real();
    last_symbol_q_ = sym.imag();
    omega_ += kGainOmega * error * std::abs(sym);
    float const omega_mid = static_cast<float>(kTimingSamplesPerSymbol);
    omega_ = omega_mid + std::clamp(omega_ - omega_mid, -kOmegaRelativeLimit, kOmegaRelativeLimit);
    mu_ += omega_ + kGainMu * error;
    return sym;
  }

  template <typename Emit>
  void symbol(cfloat sample, Emit& emit)
  {
    if (!have_previous_) {
      previous_ = sample;
      have_previous_ = true;
      return;
    }
    cfloat const differential = sample * std::conj(previous_);
    previous_ = sample;

    // Costas loop on the differentially decoded phasor
    cfloat const corrected = differential * cfloat(std::cos(costas_phase_), -std::sin(costas_phase_));
    float const error = (corrected.real() > 0.0f ? 1.0f : -1.0f) * corrected.imag()
                      - (corrected.imag() > 0.0f ? 1.0f : -1.0f) * corrected.real();
    float const limited = std::clamp(error, -1.0f, 1.0f);
    costas_frequency_ = std::clamp(costas_frequency_ + costas_beta_ * limited, -1.0f, 1.0f);
    costas_phase_ = std::clamp(costas_phase_ + costas_frequency_ + costas_alpha_ * limited,
                               -kCostasMaxPhase, kCostasMaxPhase);
    emit(dv_slice(std::atan2(corrected.imag(), corrected.real()) * (4.0f / kDvPi)));
  }

  std::array<cfloat, 2 * kTimingDelaySize> timing_delay_{};
  size_t timing_position_ = 0;
  float mu_ = 0.0f;
  float omega_ = static_cast<float>(kTimingSamplesPerSymbol);
  float last_symbol_i_ = 0.0f;
  float last_symbol_q_ = 0.0f;
  cfloat previous_{};
  bool have_previous_ = false;
  float costas_phase_ = 0.0f;
  float costas_frequency_ = 0.0f;
  float costas_alpha_ = 0.0f;
  float costas_beta_ = 0.0f;
};

class p25_decoder : public dv_decoder {
public:
  p25_decoder(bool cqpsk, uint16_t nac, dv_pcm_callback callback, void *user_data)
    : cqpsk_(cqpsk),
      frontend_(cqpsk ? 250.0f : 350.0f),
      voice_(nac, callback, user_data),
      fsk4_(DV_P25_C4FM_RX_TAPS)
  {
  }

  void process(cfloat const *samples, size_t count) override
  {
    for (size_t i = 0; i < count; i++) {
      cfloat lev;
      if (frontend_.process(samples[i], lev))
        leveled(lev);
    }
  }

  void process_leveled(cfloat const *samples, size_t count) override
  {
    for (size_t i = 0; i < count; i++)
      leveled(samples[i]);
  }

  void leveled(cfloat lev)
  {
    auto emit = [this](uint8_t raw) {
      symbols_++;
      voice_.rx_dibit(mapper_.process(raw));
    };
    if (cqpsk_) {
      cqpsk_demod_.process(lev, emit);
    } else {
      uint8_t raw;
      if (fsk4_.process(lev, raw))
        emit(raw);
    }
  }

  void get_stats(dv_stats *stats) const override
  {
    *stats = {};
    stats->samples = frontend_.samples;
    stats->symbols = symbols_;
    stats->syncs = mapper_.syncs;
    voice_.get_stats(stats);
    stats->freq_offset = frontend_.frequency_offset_hz();
    if (!cqpsk_)
      stats->freq_offset += fsk4_.fine_offset_hz();
  }

private:
  bool const cqpsk_;
  dv_frontend frontend_;
  dibit_mapper mapper_;
  p25_voice voice_;
  dv_fsk4 fsk4_;
  cqpsk_demod cqpsk_demod_;
  uint64_t symbols_ = 0;
};

} // namespace

dv_decoder *dv_create_p25(bool cqpsk, uint16_t nac, dv_pcm_callback callback, void *user_data)
{
  return new p25_decoder(cqpsk, nac, callback, user_data);
}
