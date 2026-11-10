// Receive chain shared by the digital voice modes
// Adapted from OP25 (boatbod/op25 718e589a) p25_demodulator_dev.py and the
// GNU Radio 3.8.5 fll_band_edge_cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include "dv_frontend.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "upstream/p25_c4fm_rx_taps.h"

dv_fsk4::taps const DV_P25_C4FM_RX_TAPS = P25_C4FM_RX_TAPS;

// GNU Radio firdes::root_raised_cosine(1.0, 24000, 4800, 0.2, 35), as OP25
// uses for DMR (filter_type "rrc"); zero padded to the 45-tap buffer
dv_fsk4::taps const DV_RRC_TAPS = {
  -0.0101122207f,-0.0117263387f,-0.00765813889f,0.00154603764f,0.0125888896f,
  0.0203803405f,0.0200748036f,0.00945543142f,-0.00934728653f,-0.0296814449f,
  -0.0420840681f,-0.0374620991f,-0.0106650591f,0.0369144397f,0.0968281422f,
  0.155544466f,0.198375586f,0.214057038f,0.198375586f,0.155544466f,
  0.0968281422f,0.0369144397f,-0.0106650591f,-0.0374620991f,-0.0420840681f,
  -0.0296814449f,-0.00934728653f,0.00945543142f,0.0200748036f,0.0203803405f,
  0.0125888896f,0.00154603764f,-0.00765813889f,-0.0117263387f,-0.0101122207f,
  0.0f,0.0f,0.0f,0.0f,0.0f,
  0.0f,0.0f,0.0f,0.0f,0.0f,
};

// GNU Radio firdes::root_raised_cosine(1.0, 24000, 4800, 0.5, 41): M17's
// receive filter (alpha 0.5, 8-symbol span, as libm17 rrc_taps_5) with unity
// DC gain; zero padded to the 45-tap buffer
dv_fsk4::taps const DV_RRC_05_TAPS = {
  -0.002021535f,-0.00122762602f,0.000978607106f,0.00301227773f,0.00305203388f,
  0.0006064605f,-0.00268130948f,-0.00389052395f,-0.00107803483f,0.0045644226f,
  0.00849044701f,0.00534501182f,-0.00679097771f,-0.023087893f,-0.0322482841f,
  -0.0212261175f,0.0175202555f,0.080140853f,0.150604459f,0.20624626f,
  0.227382427f,0.20624626f,0.150604459f,0.080140853f,0.0175202555f,
  -0.0212261175f,-0.0322482841f,-0.023087893f,-0.00679097771f,0.00534501182f,
  0.00849044701f,0.0045644226f,-0.00107803483f,-0.00389052395f,-0.00268130948f,
  0.0006064605f,0.00305203388f,0.00301227773f,0.000978607106f,-0.00122762602f,
  -0.002021535f,0.0f,0.0f,0.0f,0.0f,
};

namespace {

constexpr float kFllSamplesPerSymbol = kDvSampleRate / kDvSymbolRate;
constexpr float kFllRolloff = 0.2f;

float sinc(float x)
{
  float const arg = kDvPi * x;
  return x == 0.0f ? 1.0f : std::sin(arg) / arg;
}

} // namespace

uint8_t dv_slice(float symbol)
{
  if (0.0f <= symbol && symbol < 2.0f)
    return 0;
  if (2.0f <= symbol && symbol < 4.0f)
    return 1;
  if (-2.0f <= symbol && symbol < 0.0f)
    return 2;
  return 3;
}

dv_band_edge_fll::dv_band_edge_fll(float bandwidth_divisor)
{
  float const bandwidth = 2.0f * kDvPi / kFllSamplesPerSymbol / bandwidth_divisor;
  constexpr int half_filter = static_cast<int>((kFilterSize / kFllSamplesPerSymbol) + 0.5f);
  std::array<float, kFilterSize> baseband_taps{};
  float power = 0.0f;
  for (size_t i = 0; i < kFilterSize; i++) {
    float const k = -half_filter + static_cast<float>(i) * 2.0f / kFllSamplesPerSymbol;
    float const tap = sinc(kFllRolloff * k - 0.5f) + sinc(kFllRolloff * k + 0.5f);
    baseband_taps[i] = tap;
    power += tap;
  }
  constexpr int center = (kFilterSize - 1) / 2;
  for (size_t i = 0; i < kFilterSize; i++) {
    float const tap = baseband_taps[i] / power;
    float const k = (static_cast<int>(i) - center) / (2.0f * kFllSamplesPerSymbol);
    float const angle = -2.0f * kDvPi * (1.0f + kFllRolloff) * k;
    cfloat const lower = tap * cfloat(std::cos(angle), std::sin(angle));
    lower_taps_[kFilterSize - i - 1] = lower;
    upper_taps_[kFilterSize - i - 1] = std::conj(lower);
  }
  constexpr float damping = 0.7071067811865475f;
  float const denominator = 1.0f + 2.0f * damping * bandwidth + bandwidth * bandwidth;
  alpha_ = 4.0f * damping * bandwidth / denominator;
  beta_ = 4.0f * bandwidth * bandwidth / denominator;
}

cfloat dv_band_edge_fll::process(cfloat sample)
{
  cfloat const corrected = sample * cfloat(std::cos(phase_), std::sin(phase_));
  history_[position_] = corrected;
  float const error = std::norm(dv_fir(upper_taps_, history_.data(), position_))
                    - std::norm(dv_fir(lower_taps_, history_.data(), position_));
  frequency_ += beta_ * error;
  phase_ += frequency_ + alpha_ * error;
  if (phase_ > kDvTwoPi)
    phase_ -= kDvTwoPi;
  else if (phase_ < -kDvTwoPi)
    phase_ += kDvTwoPi;
  float const max_frequency = kDvTwoPi * 2.0f / kFllSamplesPerSymbol;
  frequency_ = std::clamp(frequency_, -max_frequency, max_frequency);
  position_ = (position_ + 1) % kFilterSize;
  return corrected;
}

bool dv_frontend::process(cfloat s, cfloat& out)
{
  if (!std::isfinite(s.real()) || !std::isfinite(s.imag()))
    s = 0; // never let a NaN into the loop filters
  uint64_t const sample_index = samples++;
  decimator_history_[decimator_position_] = s;
  cfloat const decimated = dv_fir(P25_C4FM_DECIMATOR_TAPS, decimator_history_, decimator_position_);
  decimator_position_ = (decimator_position_ + 1) % std::size(decimator_history_);
  if (sample_index & 1)
    return false;

  if_history_[if_position_] = decimated;
  cfloat const filtered = dv_fir(P25_C4FM_IF_TAPS, if_history_, if_position_);
  if_position_ = (if_position_ + 1) % std::size(if_history_);
  agc_power_ = (1.0 - 0.45) * agc_power_ + 0.45 * std::norm(filtered);
  float const agc_gain = static_cast<float>(0.85 / (std::sqrt(agc_power_) + 1e-18));
  out = fll_.process(filtered * agc_gain);
  return true;
}

bool dv_fsk4::process(cfloat sample, uint8_t& dibit)
{
  if (!have_previous_) {
    previous_ = sample;
    have_previous_ = true;
    return false;
  }
  float const re = sample.real() * previous_.real() + sample.imag() * previous_.imag();
  float const im = sample.imag() * previous_.real() - sample.real() * previous_.imag();
  previous_ = sample;
  history_[position_] = std::atan2(im, re) * quadrature_gain_;
  float const shaped = dv_fir(rx_taps_, history_, position_);
  position_ = (position_ + 1) % std::size(history_);
  float sym;
  if (fsk4_.process(&shaped, 1, &sym) == 0)
    return false;
  dibit = dv_slice(sym);
  return true;
}
