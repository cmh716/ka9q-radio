// Automatic digital voice mode: every receiver runs on the same IQ, and the
// first one to decode voice owns the audio until its transmission ends; the
// others are not run meanwhile
// SPDX-License-Identifier: GPL-3.0-or-later
#include <array>
#include <cstring>
#include <memory>
#include <vector>

#include "dv_decoder.h"
#include "dv_frontend.h"

namespace {

// P25 CQPSK (simulcast) is left out: configure it explicitly
constexpr dv_mode kModes[] = {DV_P25_C4FM, DV_DMR, DV_YSF, DV_DSTAR, DV_NXDN48, DV_NXDN96, DV_M17};
constexpr size_t kNumModes = sizeof kModes / sizeof kModes[0];

// All but NXDN48 run at 4800 Bd behind identical front ends (FLL divisor
// 350): they share one, which halves the cost of the channel
bool shares_frontend(dv_mode m) { return m != DV_NXDN48; }

// The owner lets go after this long without decoding a frame
constexpr uint64_t kHoldSamples = DV_IQ_SAMPRATE / 2; // 500 ms

using pcm_frame = std::array<int16_t, DV_PCM_SAMPLES>;

class auto_decoder : public dv_decoder {
public:
  auto_decoder(dv_config const& config, dv_pcm_callback callback, void *user_data)
    : callback_(callback), user_data_(user_data), frontend_(350.0f)
  {
    for (size_t i = 0; i < kNumModes; i++) {
      tap_[i] = {this, i};
      dv_decoder *d = nullptr;
      switch (kModes[i]) {
      case DV_P25_C4FM: d = dv_create_p25(false, config.p25_nac, &auto_decoder::collect, &tap_[i]); break;
      case DV_DMR: d = dv_create_dmr(config.dmr_slot, &auto_decoder::collect, &tap_[i]); break;
      case DV_YSF: d = dv_create_ysf(&auto_decoder::collect, &tap_[i]); break;
      case DV_DSTAR: d = dv_create_dstar(&auto_decoder::collect, &tap_[i]); break;
      case DV_NXDN48: d = dv_create_nxdn(false, &auto_decoder::collect, &tap_[i]); break;
      case DV_NXDN96: d = dv_create_nxdn(true, &auto_decoder::collect, &tap_[i]); break;
      case DV_M17: d = dv_create_m17(config.m17_can_filter ? config.m17_can : -1, &auto_decoder::collect, &tap_[i]); break;
      default: break;
      }
      sub_[i].reset(d);
    }
  }

  void process(std::complex<float> const *samples, size_t count) override
  {
    leveled_.clear();
    for (size_t i = 0; i < count; i++) {
      cfloat lev;
      if (frontend_.process(samples[i], lev))
        leveled_.push_back(lev);
    }
    for (size_t i = 0; i < kNumModes; i++) {
      // While one receiver plays a call, the rest rest: a repeater carries
      // one transmission at a time
      if (owner_ >= 0 && static_cast<int>(i) != owner_)
        continue;
      if (shares_frontend(kModes[i]))
        sub_[i]->process_leveled(leveled_.data(), leveled_.size());
      else
        sub_[i]->process(samples, count);
    }
    samples_ += count;

    // Who decoded what in this block
    int best = -1;
    uint64_t best_voice = 0;
    for (size_t i = 0; i < kNumModes; i++) {
      dv_stats st;
      sub_[i]->get_stats(&st);
      uint64_t const voice = st.voice_decoded - last_voice_[i];
      bool const framed = st.frames != last_frames_[i];
      last_voice_[i] = st.voice_decoded;
      last_frames_[i] = st.frames;
      if (static_cast<int>(i) == owner_ && (framed || voice != 0))
        owner_seen_ = samples_;
      if (voice > best_voice) {
        best_voice = voice;
        best = static_cast<int>(i);
      }
    }
    if (owner_ >= 0 && samples_ - owner_seen_ > kHoldSamples)
      owner_ = -1; // its transmission is over
    if (owner_ < 0 && best >= 0) {
      owner_ = best;
      owner_seen_ = samples_;
    }

    // Pass on the owner's audio, drop everyone else's
    for (size_t i = 0; i < kNumModes; i++) {
      if (static_cast<int>(i) == owner_ && callback_ != nullptr)
        for (auto const& f : queue_[i])
          callback_(user_data_, f.data());
      queue_[i].clear();
    }
  }

  void process_leveled(std::complex<float> const *, size_t) override {} // auto is never shared

  void set_event_callback(dv_event_callback callback, void *user_data) override
  {
    for (auto& d : sub_)
      d->set_event_callback(callback, user_data);
  }

  void get_stats(dv_stats *stats) const override
  {
    if (owner_ >= 0) {
      sub_[owner_]->get_stats(stats);
      stats->active_mode = kModes[owner_];
      if (shares_frontend(kModes[owner_])) {
        stats->samples = frontend_.samples;
        stats->freq_offset += frontend_.frequency_offset_hz();
      }
    } else {
      *stats = {};
      sub_[0]->get_stats(stats);
      stats->samples = frontend_.samples;
      stats->freq_offset = frontend_.frequency_offset_hz();
      // Nothing playing: report activity summed over all receivers
      stats->syncs = stats->frames = stats->voice_decoded = stats->voice_muted = 0;
      for (size_t i = 0; i < kNumModes; i++) {
        dv_stats st;
        sub_[i]->get_stats(&st);
        stats->syncs += st.syncs;
        stats->frames += st.frames;
      }
      stats->src_call[0] = stats->dst_call[0] = '\0';
      stats->active_mode = -1;
      stats->m17_can = -1;
    }
  }

private:
  struct tap {
    auto_decoder *self;
    size_t index;
  };

  static void collect(void *user_data, int16_t const pcm[DV_PCM_SAMPLES])
  {
    tap const *t = static_cast<tap const *>(user_data);
    pcm_frame f;
    std::memcpy(f.data(), pcm, sizeof f);
    t->self->queue_[t->index].push_back(f);
  }

  dv_pcm_callback const callback_;
  void * const user_data_;
  dv_frontend frontend_;            // shared by all but NXDN48
  std::vector<cfloat> leveled_;
  std::unique_ptr<dv_decoder> sub_[kNumModes];
  tap tap_[kNumModes];
  std::vector<pcm_frame> queue_[kNumModes];
  uint64_t last_voice_[kNumModes]{};
  uint64_t last_frames_[kNumModes]{};
  uint64_t samples_ = 0;
  int owner_ = -1;
  uint64_t owner_seen_ = 0;
};

} // namespace

dv_decoder *dv_create_auto(dv_config const& config, dv_pcm_callback callback, void *user_data)
{
  return new auto_decoder(config, callback, user_data);
}
