// Receive chain shared by the digital voice modes
// Adapted from OP25 (boatbod/op25 718e589a) p25_demodulator_dev.py and the
// GNU Radio 3.8.5 fll_band_edge_cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DV_FRONTEND_H
#define KA9Q_DV_FRONTEND_H

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>

#include "fsk4.h"

using cfloat = std::complex<float>;

constexpr float kDvPi = 3.14159265358979323846f;
constexpr float kDvTwoPi = 2.0f * kDvPi;
constexpr float kDvSampleRate = 24000.0f;   // after the front end's 2:1 decimation
constexpr float kDvSymbolRate = 4800.0f;
constexpr float kDvDeviation = 600.0f;      // Hz per symbol level (P25 C4FM nominal)

// FIR on a circular history buffer; position is the newest sample
template <typename Tap, size_t N, typename Sample>
Sample dv_fir(std::array<Tap, N> const& taps, Sample const *history, size_t position)
{
  Sample result{};
  for (size_t tap = 0; tap < N; tap++)
    result += taps[tap] * history[(position + N - tap) % N];
  return result;
}

// Symbol (+/-1, +/-3 scale) to dibit; same thresholds for FSK and CQPSK phase
uint8_t dv_slice(float symbol);

// GNU Radio fll_band_edge_cc
class dv_band_edge_fll {
public:
  explicit dv_band_edge_fll(float bandwidth_divisor);
  cfloat process(cfloat sample);
  float frequency_offset_hz() const { return -frequency_ * kDvSampleRate / kDvTwoPi; }

private:
  static constexpr size_t kFilterSize = 11;
  std::array<cfloat, kFilterSize> lower_taps_{};
  std::array<cfloat, kFilterSize> upper_taps_{};
  std::array<cfloat, kFilterSize> history_{};
  size_t position_ = 0;
  float phase_ = 0.0f;
  float frequency_ = 0.0f;
  float alpha_ = 0.0f;
  float beta_ = 0.0f;
};

// 48k -> 24k decimator, IF filter, RMS AGC, band-edge FLL
class dv_frontend {
public:
  explicit dv_frontend(float fll_bandwidth_divisor) : fll_(fll_bandwidth_divisor) {}
  // Returns true and sets out when a 24 kHz sample is ready
  bool process(cfloat in, cfloat& out);
  float frequency_offset_hz() const { return fll_.frequency_offset_hz(); }
  uint64_t samples = 0;  // 48 kHz input samples

private:
  dv_band_edge_fll fll_;
  cfloat decimator_history_[9]{};
  cfloat if_history_[49]{};
  size_t decimator_position_ = 0;
  size_t if_position_ = 0;
  double agc_power_ = 0.0;
};

// 4-level FSK symbol recovery: quadrature discriminator, receive filter,
// FSK4 MMSE timing, slicer
class dv_fsk4 {
public:
  using taps = std::array<float, 45>;
  explicit dv_fsk4(taps const& rx_taps) : rx_taps_(rx_taps), fsk4_(kDvSampleRate, kDvSymbolRate) {}
  // Returns true and sets dibit when a symbol is ready
  bool process(cfloat in, uint8_t& dibit);
  float fine_offset_hz() const { return static_cast<float>(fsk4_.fine_frequency_correction() * kDvDeviation); }

private:
  taps const& rx_taps_;
  p25_fsk4_demod fsk4_;
  cfloat previous_{};
  bool have_previous_ = false;
  float history_[45]{};
  size_t position_ = 0;
};

// Receive filters for dv_fsk4
extern dv_fsk4::taps const DV_P25_C4FM_RX_TAPS;  // P25 C4FM (OP25 transfer_function_rx)
extern dv_fsk4::taps const DV_RRC_TAPS;          // root raised cosine, alpha 0.2 (DMR)

#endif
