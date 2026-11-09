// P25 Phase 1 IQ receiver: C4FM and CQPSK demodulators feeding p25_voice
// Adapted from OP25 (boatbod/op25 718e589a) p25_demodulator_dev.py and the
// GNU Radio 3.8.5 fll_band_edge_cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include "p25_native.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <iterator>

#include "fsk4.h"
#include "p25_voice.h"
#include "upstream/frame_sync_magics.h"
#include "upstream/gnuradio_mmse_taps.h"
#include "upstream/p25_c4fm_rx_taps.h"

namespace {

using cfloat = std::complex<float>;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kSampleRate = 24000.0f;  // after 2:1 decimation
constexpr float kSymbolRate = 4800.0f;
constexpr float kDeviation = 600.0f;     // C4FM Hz per symbol level
constexpr float kQuadratureGain = kSampleRate / (2.0f * kPi * kDeviation);
constexpr float kFllSamplesPerSymbol = kSampleRate / kSymbolRate;
constexpr float kFllRolloff = 0.2f;

// CQPSK Gardner timing and Costas loop parameters
constexpr size_t kTimingSamplesPerSymbol = 5;
constexpr size_t kTimingDelaySize = 2 * kTimingSamplesPerSymbol;
constexpr float kGainMu = 0.025f;
constexpr float kGainOmega = 0.1f * kGainMu * kGainMu;
constexpr float kOmegaRelativeLimit = 0.002f;
constexpr float kCostasBandwidth = 0.008f;
constexpr float kCostasMaxPhase = kTwoPi / 4.0f;

// FIR on a circular history buffer; position is the newest sample
template <typename Tap, size_t N, typename Sample>
Sample fir(std::array<Tap, N> const& taps, Sample const *history, size_t position)
{
  Sample result{};
  for (size_t tap = 0; tap < N; tap++)
    result += taps[tap] * history[(position + N - tap) % N];
  return result;
}

float sinc(float x)
{
  float const arg = kPi * x;
  return x == 0.0f ? 1.0f : std::sin(arg) / arg;
}

// Same thresholds for C4FM frequency and CQPSK phase, both scaled to +/-3
uint8_t slice(float symbol)
{
  if (0.0f <= symbol && symbol < 2.0f)
    return 0;
  if (2.0f <= symbol && symbol < 4.0f)
    return 1;
  if (-2.0f <= symbol && symbol < 0.0f)
    return 2;
  return 3;
}

// GNU Radio fll_band_edge_cc
class band_edge_fll {
public:
  explicit band_edge_fll(float bandwidth_divisor)
  {
    float const bandwidth = 2.0f * kPi / kFllSamplesPerSymbol / bandwidth_divisor;
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
      float const angle = -2.0f * kPi * (1.0f + kFllRolloff) * k;
      cfloat const lower = tap * cfloat(std::cos(angle), std::sin(angle));
      lower_taps_[kFilterSize - i - 1] = lower;
      upper_taps_[kFilterSize - i - 1] = std::conj(lower);
    }
    constexpr float damping = 0.7071067811865475f;
    float const denominator = 1.0f + 2.0f * damping * bandwidth + bandwidth * bandwidth;
    alpha_ = 4.0f * damping * bandwidth / denominator;
    beta_ = 4.0f * bandwidth * bandwidth / denominator;
  }

  cfloat process(cfloat sample)
  {
    cfloat const corrected = sample * cfloat(std::cos(phase_), std::sin(phase_));
    history_[position_] = corrected;
    float const error = std::norm(fir(upper_taps_, history_.data(), position_))
                      - std::norm(fir(lower_taps_, history_.data(), position_));
    frequency_ += beta_ * error;
    phase_ += frequency_ + alpha_ * error;
    if (phase_ > kTwoPi)
      phase_ -= kTwoPi;
    else if (phase_ < -kTwoPi)
      phase_ += kTwoPi;
    float const max_frequency = kTwoPi * 2.0f / kFllSamplesPerSymbol;
    frequency_ = std::clamp(frequency_, -max_frequency, max_frequency);
    position_ = (position_ + 1) % kFilterSize;
    return corrected;
  }

  float frequency_offset_hz() const { return -frequency_ * kSampleRate / kTwoPi; }

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

} // namespace

struct p25_native_decoder {
  p25_native_decoder(p25_native_modulation mod, uint16_t nac,
                     p25_native_pcm_callback callback, void *user_data)
    : cqpsk(mod == P25_NATIVE_CQPSK),
      fll(cqpsk ? 250.0f : 350.0f),
      voice(nac, callback, user_data),
      fsk4(kSampleRate, kSymbolRate)
  {
    constexpr float damping = 0.7071067811865475f;
    float const denominator = 1.0f + 2.0f * damping * kCostasBandwidth
                            + kCostasBandwidth * kCostasBandwidth;
    costas_alpha = 4.0f * damping * kCostasBandwidth / denominator;
    costas_beta = 4.0f * kCostasBandwidth * kCostasBandwidth / denominator;
  }

  void process(cfloat const *samples, size_t count);
  void symbol(uint8_t raw_dibit)
  {
    symbols++;
    voice.rx_dibit(mapper.process(raw_dibit));
  }

  bool const cqpsk;
  band_edge_fll fll;
  dibit_mapper mapper;
  p25_voice voice;
  uint64_t samples = 0;
  uint64_t symbols = 0;

  // Common front end: 48k -> 24k decimator, IF filter, RMS AGC
  cfloat decimator_history[9]{};
  cfloat if_history[49]{};
  size_t decimator_position = 0;
  size_t if_position = 0;
  double agc_power = 0.0;

  // C4FM: quadrature discriminator, RX shaping filter, FSK4 timing
  void c4fm_sample(cfloat sample);
  p25_fsk4_demod fsk4;
  cfloat previous_sample{};
  bool have_previous_sample = false;
  float c4fm_history[45]{};
  size_t c4fm_position = 0;

  // CQPSK: Gardner timing, differential decode, Costas loop
  void cqpsk_sample(cfloat sample);
  cfloat timing_interpolate(cfloat const *input, float frac) const;
  cfloat timing_update();
  void cqpsk_symbol(cfloat sample);
  std::array<cfloat, 2 * kTimingDelaySize> timing_delay{};
  size_t timing_position = 0;
  float mu = 0.0f;
  float omega = static_cast<float>(kTimingSamplesPerSymbol);
  float last_symbol_i = 0.0f;
  float last_symbol_q = 0.0f;
  cfloat previous_diff_sample{};
  bool have_previous_diff_sample = false;
  float costas_phase = 0.0f;
  float costas_frequency = 0.0f;
  float costas_alpha = 0.0f;
  float costas_beta = 0.0f;
};

void p25_native_decoder::process(cfloat const *input, size_t count)
{
  for (size_t i = 0; i < count; i++) {
    cfloat s = input[i];
    if (!std::isfinite(s.real()) || !std::isfinite(s.imag()))
      s = 0; // never let a NaN into the loop filters
    uint64_t const sample_index = samples++;
    decimator_history[decimator_position] = s;
    cfloat const decimated = fir(P25_C4FM_DECIMATOR_TAPS, decimator_history, decimator_position);
    decimator_position = (decimator_position + 1) % std::size(decimator_history);
    if (sample_index & 1)
      continue;

    if_history[if_position] = decimated;
    cfloat const filtered = fir(P25_C4FM_IF_TAPS, if_history, if_position);
    if_position = (if_position + 1) % std::size(if_history);
    agc_power = (1.0 - 0.45) * agc_power + 0.45 * std::norm(filtered);
    float const agc_gain = static_cast<float>(0.85 / (std::sqrt(agc_power) + 1e-18));
    cfloat const leveled = fll.process(filtered * agc_gain);
    if (cqpsk)
      cqpsk_sample(leveled);
    else
      c4fm_sample(leveled);
  }
}

void p25_native_decoder::c4fm_sample(cfloat sample)
{
  if (!have_previous_sample) {
    previous_sample = sample;
    have_previous_sample = true;
    return;
  }
  float const re = sample.real() * previous_sample.real() + sample.imag() * previous_sample.imag();
  float const im = sample.imag() * previous_sample.real() - sample.real() * previous_sample.imag();
  previous_sample = sample;
  c4fm_history[c4fm_position] = std::atan2(im, re) * kQuadratureGain;
  float const shaped = fir(P25_C4FM_RX_TAPS, c4fm_history, c4fm_position);
  c4fm_position = (c4fm_position + 1) % std::size(c4fm_history);
  float sym;
  if (fsk4.process(&shaped, 1, &sym) != 0)
    symbol(slice(sym));
}

void p25_native_decoder::cqpsk_sample(cfloat sample)
{
  if (mu <= 1.0f)
    cqpsk_symbol(timing_update());
  timing_delay[timing_position] = sample;
  timing_delay[timing_position + kTimingDelaySize] = sample;
  timing_position = (timing_position + 1) % kTimingDelaySize;
  mu -= 1.0f;
  if (mu <= 1.0f)
    cqpsk_symbol(timing_update());
}

cfloat p25_native_decoder::timing_interpolate(cfloat const *input, float frac) const
{
  size_t const phase = static_cast<size_t>(std::clamp(std::lrint(frac * 128.0f), 0L, 128L));
  cfloat result{};
  for (size_t tap = 0; tap < 8; tap++)
    result += P25_GNURADIO_MMSE_TAPS[phase][tap] * input[tap];
  return result;
}

cfloat p25_native_decoder::timing_update()
{
  float const half_omega = omega / 2.0f;
  size_t half_sps = static_cast<size_t>(std::floor(half_omega));
  float half_mu = mu + half_omega - static_cast<float>(half_sps);
  if (half_mu > 1.0f) {
    half_mu -= 1.0f;
    half_sps++;
  }
  cfloat const *delay = timing_delay.data() + timing_position;
  cfloat const midpoint = timing_interpolate(delay, mu);
  cfloat const sym = timing_interpolate(delay + half_sps, half_mu);
  float error = (last_symbol_i - sym.real()) * midpoint.real()
              + (last_symbol_q - sym.imag()) * midpoint.imag();
  if (!std::isfinite(error))
    error = 0.0f;
  error = std::clamp(error, -1.0f, 1.0f);
  last_symbol_i = sym.real();
  last_symbol_q = sym.imag();
  omega += kGainOmega * error * std::abs(sym);
  float const omega_mid = static_cast<float>(kTimingSamplesPerSymbol);
  omega = omega_mid + std::clamp(omega - omega_mid, -kOmegaRelativeLimit, kOmegaRelativeLimit);
  mu += omega + kGainMu * error;
  return sym;
}

void p25_native_decoder::cqpsk_symbol(cfloat sample)
{
  if (!have_previous_diff_sample) {
    previous_diff_sample = sample;
    have_previous_diff_sample = true;
    return;
  }
  cfloat const differential = sample * std::conj(previous_diff_sample);
  previous_diff_sample = sample;

  // Costas loop on the differentially decoded phasor
  cfloat const corrected = differential * cfloat(std::cos(costas_phase), -std::sin(costas_phase));
  float const error = (corrected.real() > 0.0f ? 1.0f : -1.0f) * corrected.imag()
                    - (corrected.imag() > 0.0f ? 1.0f : -1.0f) * corrected.real();
  float const limited = std::clamp(error, -1.0f, 1.0f);
  costas_frequency = std::clamp(costas_frequency + costas_beta * limited, -1.0f, 1.0f);
  costas_phase = std::clamp(costas_phase + costas_frequency + costas_alpha * limited,
                            -kCostasMaxPhase, kCostasMaxPhase);

  symbol(slice(std::atan2(corrected.imag(), corrected.real()) * (4.0f / kPi)));
}

extern "C" p25_native_decoder *p25_native_create(p25_native_modulation modulation, uint16_t nac,
                                                 p25_native_pcm_callback callback, void *user_data)
{
  if ((modulation != P25_NATIVE_C4FM && modulation != P25_NATIVE_CQPSK) || nac > 0xfff)
    return nullptr;
  try {
    return new p25_native_decoder(modulation, nac, callback, user_data);
  } catch (...) {
    return nullptr;
  }
}

extern "C" void p25_native_destroy(p25_native_decoder *decoder)
{
  delete decoder;
}

extern "C" void p25_native_process(p25_native_decoder *decoder, float const *iq, size_t count)
{
  // std::complex<float> is layout-compatible with float[2]
  decoder->process(reinterpret_cast<cfloat const *>(iq), count);
}

extern "C" void p25_native_get_stats(p25_native_decoder const *decoder, p25_native_stats *stats)
{
  *stats = {};
  stats->samples = decoder->samples;
  stats->symbols = decoder->symbols;
  stats->syncs = decoder->mapper.syncs;
  decoder->voice.get_stats(stats);
  stats->freq_offset = decoder->fll.frequency_offset_hz();
  if (!decoder->cqpsk)
    stats->freq_offset += static_cast<float>(decoder->fsk4.fine_frequency_correction() * kDeviation);
}
