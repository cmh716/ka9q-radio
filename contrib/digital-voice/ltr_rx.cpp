// LTR Standard (E.F. Johnson) trunked analog FM: the 300 bit/s data under
// the voice on every repeater (who is talking, on which repeater, and which
// repeater is free), and the voice itself with the data filtered out.
// There is no control channel: each repeater carries the words for its own
// call and, for the groups homed on it, where their calls are.
// Written from the signal: the word layout is the published one, the
// checksum table was derived from received words and agrees with the one
// in sdrtrunk (CRCLTR).
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

#include "dv_decoder.h"
#include "dv_frontend.h"

namespace {

constexpr float kInRate = 48000.0f;
constexpr int kDecimation = 6;             // to 8 kHz
constexpr float kRate = 8000.0f;
constexpr float kBitRate = 300.0f;
// A word: sync 9 bits, area 1, go-to repeater 5, home repeater 5, group 8,
// free repeater 5, checksum 7
constexpr uint64_t kSync = 0x158;          // 101011000
constexpr int kWordBits = 40;
constexpr uint8_t kCheck[24] = {
  0x38,                                           // area
  0x1c, 0x0e, 0x46, 0x23, 0x51,                   // go-to
  0x68, 0x75, 0x7a, 0x3d, 0x1f,                   // home
  0x4f, 0x26, 0x52, 0x29, 0x15, 0x0b, 0x45, 0x62, // group
  0x31, 0x19, 0x0d, 0x07, 0x43,                   // free
};
constexpr unsigned kIdleGroup = 255;       // nobody: the repeater's own number is in go-to and home
constexpr unsigned kTurnOff = 31;          // go-to: the transmission is over
constexpr int kSquelchBlock = 960;         // 20 ms at 48 kHz
constexpr float kSquelchOpen = 10.0f;      // envelope mean^2 / variance (noise alone: 3.7)
constexpr float kSquelchClose = 5.0f;
constexpr int kWordHoldBits = 6 * kWordBits; // a call whose words stop this long is over
constexpr int kRecent = 6;
constexpr int kPhases = 4;
constexpr int kHold = 3200;               // 0.4 s of audio
constexpr uint64_t kRecentBits = 600;      // two seconds
constexpr uint64_t kIdleBits = 18000;      // a minute: an idle repeater sends a word or two every ten seconds
constexpr float kAudioGain = 26.0f;        // 1 kHz at 1.5 kHz deviation: about a quarter of full scale

std::vector<float> lowpass(int ntaps, float cutoff, float rate)
{
  std::vector<float> taps(ntaps);
  float sum = 0;
  for (int k = 0; k < ntaps; k++) {
    float const t = k - (ntaps - 1) / 2.0f;
    float const x = 2.0f * kDvPi * cutoff / rate * t;
    float const sinc = t == 0 ? 1.0f : std::sin(x) / x;
    float const window = 0.54f - 0.46f * std::cos(2.0f * kDvPi * k / (ntaps - 1));
    taps[k] = sinc * window;
    sum += taps[k];
  }
  for (float& tap : taps)
    tap /= sum;
  return taps;
}

// Second order Butterworth high-pass section
struct biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  void highpass(float cutoff, float rate, float q)
  {
    float const w = 2.0f * kDvPi * cutoff / rate;
    float const alpha = std::sin(w) / (2.0f * q), c = std::cos(w);
    float const a0 = 1.0f + alpha;
    b0 = (1.0f + c) / 2.0f / a0;
    b1 = -(1.0f + c) / a0;
    b2 = b0;
    a1 = -2.0f * c / a0;
    a2 = (1.0f - alpha) / a0;
  }
  float process(float x)
  {
    float const y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

class ltr_decoder : public dv_decoder {
public:
  ltr_decoder(uint32_t group, dv_pcm_callback callback, void *user_data)
    : callback_(callback), user_data_(user_data), only_(group),
      audio_taps_(lowpass(73, 3200.0f, kInRate)), audio_history_(audio_taps_.size(), 0.0f),
      data_taps_(lowpass(121, 220.0f, kRate)), data_history_(data_taps_.size(), 0.0f)
  {
    // Fourth order at 300 Hz: the data is 40 dB down at 150 Hz
    high_[0].highpass(300.0f, kRate, 0.5412f);
    high_[1].highpass(300.0f, kRate, 1.3066f);
  }

  void process(cfloat const *samples, size_t count) override
  {
    for (size_t i = 0; i < count; i++)
      sample(samples[i]);
  }

  // DV_AUTO's shared front end is for the digital modes
  void process_leveled(cfloat const *, size_t) override {}

  void get_stats(dv_stats *stats) const override
  {
    *stats = {};
    stats->call_serial = call_serial_;
    stats->samples = samples_;
    stats->symbols = bits_;
    stats->syncs = syncs_;
    stats->frames = words_;
    stats->voice_decoded = voice_decoded_;
    stats->voice_muted = voice_muted_;
    stats->freq_offset = centre_;
    std::memcpy(stats->dst_call, dst_, sizeof dst_);
    stats->m17_can = -1;
    stats->active_mode = -1;
    // The data, in the fields the P25 control channel uses
    stats->tsbks = words_;
    stats->tsbk_errors = syncs_ - words_;
    stats->site = static_cast<uint8_t>(own_ < 0 ? 0 : own_);
    stats->last_nac = static_cast<uint16_t>(own_ < 0 ? 0 : own_);
    stats->sysid = static_cast<uint16_t>(area_ < 0 ? 0 : area_);
  }

  void set_event_callback(dv_event_callback callback, void *user_data) override
  {
    event_callback_ = callback;
    event_user_ = user_data;
  }

  void signal_lost() override
  {
    open_ = false;
    above_ = below_ = 0;
    end_call();
  }

private:
  void sample(cfloat x)
  {
    samples_++;
    // Carrier squelch: an FM carrier's envelope is steady, noise's is not
    float const mag = std::abs(x);
    env_sum_ += mag;
    env_sq_ += mag * mag;
    if (++env_n_ == kSquelchBlock) {
      float const mean = env_sum_ / kSquelchBlock;
      float const var = env_sq_ / kSquelchBlock - mean * mean;
      float const ratio = var > 0 ? mean * mean / var : (mean > 0 ? 1e6f : 0.0f);
      env_sum_ = env_sq_ = 0;
      env_n_ = 0;
      if (ratio >= kSquelchOpen) {
        below_ = 0;
        if (!open_ && ++above_ >= 2)
          open_ = true;
      } else if (ratio < kSquelchClose) {
        above_ = 0;
        if (open_ && ++below_ >= 3) {
          open_ = false;
          end_call();
        }
      } else {
        above_ = below_ = 0;
      }
    }
    // Discriminator, in Hz
    float const hz = std::arg(x * std::conj(previous_)) * (kInRate / kDvTwoPi);
    previous_ = x;
    audio_history_[audio_position_] = hz;
    if (++decimate_ == kDecimation) {
      decimate_ = 0;
      demodulated(dv_fir(audio_taps_, audio_history_.data(), audio_position_));
    }
    audio_position_ = (audio_position_ + 1) % audio_history_.size();
  }

  // 8 kHz discriminator output
  void demodulated(float hz)
  {
    data_history_[data_position_] = hz;
    float const y = dv_fir(data_taps_, data_history_.data(), data_position_);
    data_position_ = (data_position_ + 1) % data_history_.size();
    data(y);
    audio(hz);
  }

  // The data: NRZ about the carrier's offset. An idle repeater sends one
  // word in a burst hardly longer than the word, so nothing may need time
  // to settle. Instead of a clock recovery loop, four samplers a quarter of
  // a bit apart each keep their last 40 samples (the clocks agree to well
  // within a bit over a word); and each window of 40 is sliced about the
  // middle of its own extremes, which a word's sync alone is enough to set
  void data(float y)
  {
    float const before = phase_;
    phase_ += kPhases * kBitRate / kRate;
    if (static_cast<int>(phase_) != static_cast<int>(before)) {
      int const sampler = static_cast<int>(phase_) % kPhases;
      if (phase_ >= kPhases)
        phase_ -= kPhases;
      bit(sampler, y);
    }
  }

  void bit(int sampler, float y)
  {
    uint64_t const mask = (uint64_t{1} << kWordBits) - 1;
    float * const window = window_[sampler];
    int& at = window_at_[sampler];
    window[at] = y;
    at = (at + 1) % kWordBits;
    uint64_t shift = 0;
    if (open_) {
      float sorted[kWordBits];
      std::copy(window, window + kWordBits, sorted);
      std::sort(sorted, sorted + kWordBits);
      // The sync has four ones and five zeros: the extremes, but for the
      // outermost of each, are surely data
      float const centre = (sorted[1] + sorted[2] + sorted[3] + sorted[kWordBits - 4] + sorted[kWordBits - 3] + sorted[kWordBits - 2]) / 6.0f;
      for (int k = 0; k < kWordBits; k++)
        shift = (shift << 1) | (window[(at + k) % kWordBits] > centre ? 1 : 0);
      word_centre_ = centre;
    }
    if (sampler == 0) {
      bits_++;
      if (since_word_ < 1000000)
        since_word_++;
      if (here_ && since_word_ > kWordHoldBits)
        end_call(); // the words stopped: it was the talker's carrier we heard, or it has gone
    }
    if (!open_)
      return;
    for (int inverted = 0; inverted < 2; inverted++) {
      uint64_t const w = inverted ? ~shift & mask : shift;
      if ((w >> (kWordBits - 9)) != kSync)
        continue;
      unsigned check = 0;
      for (int k = 0; k < 24; k++)
        if ((w >> (30 - k)) & 1)
          check ^= kCheck[k];
      bool const good = check == (w & 0x7f) && (w & 0x7fffffff) != 0;
      // One word reaches several samplers
      if (w == last_word_ && bits_ - last_word_at_ < kWordBits / 2 && good == last_good_)
        break;
      if (!good && bits_ - last_word_at_ < 2)
        break; // another sampler has just read this one correctly, or will
      last_word_ = w;
      last_word_at_ = bits_;
      last_good_ = good;
      syncs_++;
      if (!good)
        break;
      words_++;
      centre_ = word_centre_;
      word(static_cast<unsigned>((w >> 30) & 1), static_cast<unsigned>((w >> 25) & 0x1f),
           static_cast<unsigned>((w >> 20) & 0x1f), static_cast<unsigned>((w >> 12) & 0xff),
           static_cast<unsigned>((w >> 7) & 0x1f));
      break;
    }
  }

  void emit(dv_event& e)
  {
    if (event_callback_ != nullptr)
      event_callback_(event_user_, &e);
  }

  // The talkgroup as RadioReference numbers it: area, home repeater (two
  // digits), group (three)
  static uint32_t talkgroup(unsigned area, unsigned home, unsigned group)
  {
    return area * 100000u + home * 1000u + group;
  }

  void word(unsigned area, unsigned go_to, unsigned home, unsigned group, unsigned free_repeater)
  {
    (void)free_repeater;
    // A 7-bit check lets one random word in 128 through: believe a word
    // only when the one before it agreed (each is sent again and again)
    // (words for several groups alternate, so not necessarily the last one)
    uint32_t const key = (area << 18) | (go_to << 13) | (home << 8) | group;
    bool repeated = false;
    int oldest = 0;
    for (int k = 0; k < kRecent; k++) {
      if (recent_[k].key == key && bits_ - recent_[k].when <= (group == kIdleGroup ? kIdleBits : kRecentBits)) {
        repeated = true;
        recent_[k].when = bits_;
      }
      if (recent_[k].when < recent_[oldest].when)
        oldest = k;
    }
    if (!repeated) {
      recent_[oldest] = {key, bits_};
      return;
    }
    if (group == kIdleGroup) {
      if (go_to == home && home >= 1 && home <= 20 && (own_ != static_cast<int>(home) || area_ != static_cast<int>(area))) {
        own_ = static_cast<int>(home);
        area_ = static_cast<int>(area);
        dv_event e{};
        e.type = DV_EVENT_SITE;
        std::snprintf(e.text, sizeof e.text, "repeater %u, area %u", home, area);
        emit(e);
      }
      return;
    }
    if (home < 1 || home > 20)
      return;
    uint32_t const tg = talkgroup(area, home, group);
    if (go_to == kTurnOff) {
      // The transmission is over
      granted_.erase(tg);
      if (here_ && current_ == tg) {
        call_serial_++;
        end_call();
      }
      return;
    }
    if (go_to < 1 || go_to > 20)
      return;
    // Where the group's call is: this repeater or another
    dv_event e{};
    auto const it = granted_.find(tg);
    bool const fresh = it == granted_.end() || it->second.go_to != go_to || bits_ - it->second.when > kRecentBits;
    e.type = fresh ? DV_EVENT_GRANT : DV_EVENT_UPDATE;
    if (granted_.size() > 64)
      granted_.clear();
    granted_[tg] = {go_to, bits_};
    if (fresh) {
      transmissions_++;
      std::snprintf(e.text, sizeof e.text, "grant TG %u on repeater %u", tg, go_to);
    }
    e.transmission = transmissions_;
    e.channel = static_cast<uint16_t>(go_to);
    e.slot = -1;
    e.group = tg;
    emit(e);
    // On this repeater: the call whose voice this is. Until an idle word
    // has given this repeater's number (one comes every ten seconds), take
    // any call to be here
    if (own_ < 0 || static_cast<int>(go_to) == own_) {
      since_word_ = 0;
      if (!here_ || current_ != tg) {
        if (here_)
          call_serial_++;
        here_ = true;
        current_ = tg;
        std::snprintf(dst_, sizeof dst_, "%u", tg);
      }
    }
  }

  void end_call()
  {
    if (here_)
      call_serial_++;
    here_ = false;
    current_ = 0;
    dst_[0] = '\0';
  }

  // The voice: the data filtered off, de-emphasised. It is played only
  // while the words say whose it is, which takes a couple of them: the
  // audio runs that much behind, so a transmission is heard from its start
  void audio(float hz)
  {
    float v = high_[1].process(high_[0].process(hz));
    deemphasis_ += 0.1535f * (v - deemphasis_); // 750 microseconds
    v = deemphasis_ * kAudioGain;
    bool const wanted = open_ && here_ && (only_ == 0 || current_ == only_);
    if (wanted && !was_wanted_) {
      // Everything held since the carrier came up belongs to this call
      for (int k = 0; k < std::min(since_open_, kHold); k++)
        hold_play_[(hold_at_ + kHold - 1 - k) % kHold] = true;
    }
    was_wanted_ = wanted;
    since_open_ = open_ ? std::min(since_open_ + 1, kHold) : 0;
    int16_t const out = hold_[hold_at_];
    bool const play = hold_play_[hold_at_];
    hold_[hold_at_] = static_cast<int16_t>(std::lrint(std::clamp(v, -32767.0f, 32767.0f)));
    hold_play_[hold_at_] = wanted;
    hold_at_ = (hold_at_ + 1) % kHold;
    pcm_[pcm_n_++] = play ? out : 0;
    played_ |= play;
    if (pcm_n_ == DV_PCM_SAMPLES) {
      pcm_n_ = 0;
      if (played_) {
        voice_decoded_++;
        if (callback_ != nullptr)
          callback_(user_data_, pcm_);
      } else if (open_) {
        voice_muted_++;
      }
      played_ = false;
    }
  }

  dv_pcm_callback const callback_;
  void * const user_data_;
  uint32_t const only_;          // play only this talkgroup (0: whatever the repeater carries)
  dv_event_callback event_callback_ = nullptr;
  void *event_user_ = nullptr;

  // Squelch
  float env_sum_ = 0, env_sq_ = 0;
  int env_n_ = 0;
  int above_ = 0, below_ = 0;
  bool open_ = false;

  // Discriminator and decimator
  cfloat previous_{};
  std::vector<float> const audio_taps_;
  std::vector<float> audio_history_;
  size_t audio_position_ = 0;
  int decimate_ = 0;

  // Data
  std::vector<float> const data_taps_;
  std::vector<float> data_history_;
  size_t data_position_ = 0;
  float phase_ = 0;              // in quarter bits
  float window_[kPhases][kWordBits]{};
  int window_at_[kPhases]{};
  float word_centre_ = 0, centre_ = 0; // the carrier's offset, Hz, as of the last good word
  uint64_t last_word_ = 0, last_word_at_ = 0;
  bool last_good_ = false;
  struct seen {
    uint32_t key = ~0u;
    uint64_t when = 0;
  };
  seen recent_[kRecent];         // the last few different words
  int since_word_ = 0;           // bits since a word of the call on this repeater

  // What the words say
  int own_ = -1;                 // this repeater's number, once an idle word has given it
  int area_ = -1;
  bool here_ = false;            // a call is on this repeater
  uint32_t current_ = 0;         // its talkgroup
  struct grant {
    unsigned go_to;
    uint64_t when;
  };
  std::map<uint32_t, grant> granted_; // talkgroups announced, and where
  uint32_t transmissions_ = 0;
  char dst_[12]{};

  // Voice
  biquad high_[2];
  float deemphasis_ = 0;
  int16_t hold_[kHold]{};
  bool hold_play_[kHold]{};
  int hold_at_ = 0;
  int since_open_ = 0;
  bool was_wanted_ = false;
  int16_t pcm_[DV_PCM_SAMPLES]{};
  int pcm_n_ = 0;
  bool played_ = false;

  uint64_t samples_ = 0, bits_ = 0, syncs_ = 0, words_ = 0;
  uint64_t voice_decoded_ = 0, voice_muted_ = 0;
  uint64_t call_serial_ = 0;
};

} // namespace

dv_decoder *dv_create_ltr(uint32_t group, dv_pcm_callback callback, void *user_data)
{
  return new ltr_decoder(group, callback, user_data);
}
