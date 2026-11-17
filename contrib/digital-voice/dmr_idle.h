// Protected acquisition of DMR idle bursts when decision-directed timing fails.
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DMR_IDLE_H
#define KA9Q_DMR_IDLE_H

#include <array>
#include <cmath>
#include <cstdint>
#include "upstream/frame_sync_magics.h"
#include "upstream/golay2087.h"

// Input: the 24 kHz RRC-filtered discriminator, in nominal symbol levels.
// This observer never produces audio or changes the voice receiver's timing.
// A sync-sign match alone is insufficient: two Golay-protected idle slot types
// must agree on color code and occur on the 30 ms burst grid before reporting.
class dmr_idle_detector {
public:
  bool process(float sample, int& color) {
    history_[clock_ % history_.size()] = sample;
    bool detected = false;
    unsigned const phase = clock_ % 5;
    if(clock_ >= 165 && pending_[phase] && clock_ == pending_[phase] + 25) {
      uint16_t const colors = slot_colors();
      uint64_t const gap = pending_[phase] - previous_[phase];
      unsigned const remainder = gap % 720;
      if(colors && previous_[phase] && gap >= 718 && gap <= 8 * 720 + 2
         && (remainder <= 2 || remainder >= 718)) {
        unsigned const common = colors & previous_colors_[phase];
        // Ambiguous threshold hypotheses cannot establish a color code.
        if(common && !(common & (common - 1)) && clock_ - reported_ >= 360) {
          color = __builtin_ctz(common);
          reported_ = clock_;
          detected = true;
        }
      }
      if(colors) {
        previous_[phase] = pending_[phase];
        previous_colors_[phase] = colors;
      }
      pending_[phase] = 0;
    }
    // Each of the five sample phases is searched independently. Small DC
    // hypotheses cover residual tuning error without training on noise.
    constexpr float offsets[] = {-1.0f, 0.0f, 1.0f};
    for(unsigned i = 0; i < 3; i++) {
      uint32_t& reg = signs_[phase][i];
      reg = ((reg << 1) | (sample < offsets[i])) & 0xffffff;
      if(clock_ >= 140 && !pending_[phase]
         && __builtin_popcount(reg ^ sync_signs()) <= 1)
        pending_[phase] = clock_;
    }
    clock_++;
    return detected;
  }

  void reset() { *this = dmr_idle_detector{}; }

private:
  static constexpr uint32_t sync_signs() {
    uint32_t result = 0;
    for(int i = 23; i >= 0; i--)
      result = (result << 1) | ((DMR_BS_DATA_SYNC_MAGIC >> (2 * i + 1)) & 1);
    return result;
  }

  float ago(unsigned n) const { return history_[(clock_ - n) % history_.size()]; }

  uint16_t slot_colors() const {
    // Sync ends 25 samples ago; estimate DC and the outer level from its
    // known signs. Data decisions use those estimates, not the MMSE loop.
    float positive = 0, negative = 0;
    unsigned np = 0, nn = 0;
    for(unsigned i = 0; i < 24; i++) {
      float const value = ago(25 + 5 * (23 - i));
      if((sync_signs() >> (23 - i)) & 1) { negative += value; nn++; }
      else { positive += value; np++; }
    }
    positive /= np; negative /= nn;
    float const center = (positive + negative) * 0.5f;
    float const outer = (positive - negative) * 0.5f;
    if(!std::isfinite(outer) || outer < 0.4f || outer > 12.0f)
      return 0;
    uint16_t colors = 0;
    // Filtered, distorted idle patterns can bias inner/outer spacing. Each
    // hypothesis still needs the full protected slot type and grid confirmation.
    constexpr float thresholds[] = {0.5f, 2.0f/3, 5.0f/6, 1.0f, 7.0f/6, 4.0f/3};
    for(float threshold : thresholds) {
      bit_vector bits;
      for(unsigned i = 0; i < 10; i++) {
        unsigned const delay = i < 5 ? 165 - 5 * i : 20 - 5 * (i - 5);
        float const value = ago(delay) - center;
        bits.push_back(value < 0);
        bits.push_back(std::fabs(value) >= threshold * outer);
      }
      bit_vector corrected = bits;
      if(CGolay2087::decode(corrected) > 2)
        continue;
      // Re-encode as well: verify all 20 bits, including extended parity.
      bit_vector encoded = corrected;
      CGolay2087::encode(encoded);
      unsigned errors = 0;
      for(unsigned i = 0; i < 20; i++) errors += bits[i] != encoded[i];
      unsigned type = 0, cc = 0;
      for(unsigned i = 0; i < 4; i++) {
        cc = (cc << 1) | corrected[i];
        type = (type << 1) | corrected[i + 4];
      }
      if(errors <= 2 && type == 9)
        colors |= 1U << cc;
    }
    return colors;
  }

  std::array<float, 192> history_{};
  std::array<std::array<uint32_t, 3>, 5> signs_{};
  std::array<uint64_t, 5> pending_{}, previous_{};
  std::array<uint16_t, 5> previous_colors_{};
  uint64_t clock_ = 0, reported_ = 0;
};
#endif
