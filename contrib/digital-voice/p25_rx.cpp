// P25 Phase 1 IQ receiver: C4FM or CQPSK demodulation feeding p25_voice
// Adapted from OP25 (boatbod/op25 718e589a) p25_demodulator_dev.py and
// gardner/costas blocks; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <array>
#include <cmath>

#include "dv_decoder.h"
#include "dv_frontend.h"
#include "cqpsk.h"
#include "p25_voice.h"
#include "upstream/frame_sync_magics.h"

namespace {

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

  void set_event_callback(dv_event_callback callback, void *user_data) override
  {
    voice_.set_event_callback(callback, user_data);
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
  cqpsk_demod<5> cqpsk_demod_;
  uint64_t symbols_ = 0;
};

} // namespace

dv_decoder *dv_create_p25(bool cqpsk, uint16_t nac, dv_pcm_callback callback, void *user_data)
{
  return new p25_decoder(cqpsk, nac, callback, user_data);
}
