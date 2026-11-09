// DMR (Tier I/II) voice receiver: 4FSK demodulation, burst framing,
// timeslot tracking and AMBE+2 decoding
// Framing and voice handling follow OP25 (boatbod/op25 718e589a) rx_sync.cc
// and dmr_cai.cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cstring>

#include "dv_decoder.h"
#include "dv_frontend.h"
#include "upstream/bit_utils.h"
#include "upstream/check_frame_sync.h"
#include "upstream/dmr_const.h"
#include "upstream/frame_sync_magics.h"
#include "upstream/mbelib.h"
#include "upstream/ambe.h" // needs mbelib.h first
#include "upstream/p25p2_vf.h"
#include "upstream/software_imbe_decoder.h"

namespace {

constexpr int kBurstDibits = 144;      // CACH 12 + payload 54 + sync/EMB 24 + payload 54
constexpr int kSyncOffset = 66;        // dibits from burst start to sync
constexpr int kSyncDibits = 24;
constexpr int kExpiration = 1728;      // dibits without a sync before giving up (12 bursts)
constexpr unsigned kBurstSyncThreshold = 6; // bit errors allowed when classifying a burst's sync

constexpr uint64_t kSyncs[] = {
  DMR_BS_VOICE_SYNC_MAGIC, DMR_BS_DATA_SYNC_MAGIC, DMR_MS_VOICE_SYNC_MAGIC,
  DMR_MS_DATA_SYNC_MAGIC, DMR_MS_RC_SYNC_MAGIC, DMR_T1_VOICE_SYNC_MAGIC,
  DMR_T1_DATA_SYNC_MAGIC, DMR_T2_VOICE_SYNC_MAGIC, DMR_T2_DATA_SYNC_MAGIC,
};
constexpr unsigned slot_ids[] = {0, 1, 0, 0, 1, 1, 0, 1}; // dmr_cai.h

bool is_voice_sync(uint64_t sync)
{
  return sync == DMR_BS_VOICE_SYNC_MAGIC || sync == DMR_MS_VOICE_SYNC_MAGIC
      || sync == DMR_T1_VOICE_SYNC_MAGIC || sync == DMR_T2_VOICE_SYNC_MAGIC;
}

// Per-timeslot AMBE+2 decoder state (rx_sync.h)
struct ambe_slot {
  ambe_slot()
  {
    mbe_initMbeParms(&cur, &prev, &enh);
    mbe_initErrParms(&errs);
    mbe_initToneParms(&tone);
  }
  mbe_parms cur, prev, enh;
  mbe_errs errs;
  mbe_tone tone;
  int err_cnt = 0;
  software_imbe_decoder synth;
};

class dmr_decoder : public dv_decoder {
public:
  dmr_decoder(int slot, dv_pcm_callback callback, void *user_data)
    : wanted_slot_(slot - 1), callback_(callback), user_data_(user_data),
      frontend_(350.0f), fsk4_(DV_RRC_TAPS)
  {
  }

  void process(cfloat const *samples, size_t count) override
  {
    for (size_t i = 0; i < count; i++) {
      cfloat leveled;
      uint8_t dibit;
      if (frontend_.process(samples[i], leveled) && fsk4_.process(leveled, dibit))
        rx_dibit(dibit);
    }
  }

  void get_stats(dv_stats *stats) const override
  {
    *stats = {};
    stats->samples = frontend_.samples;
    stats->symbols = symbols_;
    stats->syncs = syncs_;
    stats->frames = bursts_;
    stats->voice_decoded = voice_decoded_;
    stats->voice_muted = voice_muted_;
    stats->freq_offset = frontend_.frequency_offset_hz() + fsk4_.fine_offset_hz();
    stats->active_slot = active_slot_ + 1;
    if (active_slot_ >= 0)
      stats->error_rate = slots_[active_slot_].errs.ER;
  }

private:
  // rx_sync::rx_sym(), DMR only
  void rx_dibit(uint8_t dibit)
  {
    symbols_++;
    symbol_count_++;
    sync_reg_ = (sync_reg_ << 2) | (dibit & 3);
    bool sync_detected = false;
    for (uint64_t magic : kSyncs) {
      if (check_frame_sync(magic ^ sync_reg_, locked_ ? threshold_ : 0, 2 * kSyncDibits)) {
        sync_detected = true;
        break;
      }
    }
    std::memmove(buf_, buf_ + 1, sizeof buf_ - 1);
    buf_[sizeof buf_ - 1] = dibit;
    if (!locked_ && !sync_detected)
      return;
    rx_count_++;
    if (sync_detected) {
      syncs_++;
      if (!locked_) {
        locked_ = true;
        rx_count_ = kSyncOffset + kSyncDibits;
      } else if (rx_count_ != kSyncOffset + kSyncDibits) {
        sync_reset();
        locked_ = true;
        rx_count_ = kSyncOffset + kSyncDibits;
      } else {
        threshold_ = std::min(threshold_ + 1, 2);
      }
      expires_ = symbol_count_ + kExpiration;
    }
    if (locked_ && symbol_count_ >= expires_) {
      sync_reset();
      return;
    }
    if (rx_count_ < kBurstDibits)
      return;
    rx_count_ = 0;
    burst(buf_ + sizeof buf_ - kBurstDibits);
  }

  void sync_reset()
  {
    symbol_count_ = 0;
    rx_count_ = 0;
    threshold_ = 0;
    sync_reg_ = 0;
    expires_ = 0;
    locked_ = false;
    for (int s = 0; s < 2; s++)
      unmute_until_[s] = 0;
    active_slot_ = -1;
  }

  // dmr_cai::load_frame() plus the RX_TYPE_DMR case of rx_sync::rx_sym()
  void burst(uint8_t const *sym)
  {
    bursts_++;
    uint8_t bits[2 * kBurstDibits];
    dibits_to_bits(bits, sym, kBurstDibits);

    bool sync_rxd = false;
    uint64_t sl_sync = load_reg64(bits + 2 * kSyncOffset, 2 * kSyncDibits);
    for (uint64_t magic : kSyncs) {
      if (static_cast<unsigned>(__builtin_popcountll(sl_sync ^ magic)) <= kBurstSyncThreshold) {
        sl_sync = magic;
        sync_rxd = true;
        break;
      }
    }

    // Timeslot: from the CACH TACT on base station bursts, else by alternation
    if (sync_rxd) {
      switch (sl_sync) {
      case DMR_BS_VOICE_SYNC_MAGIC:
      case DMR_BS_DATA_SYNC_MAGIC: {
        uint8_t tact_bits[sizeof cach_tact_bits];
        for (size_t i = 0; i < sizeof cach_tact_bits; i++)
          tact_bits[i] = bits[cach_tact_bits[i]];
        int const tact = hamming_7_4_decode[load_i(tact_bits, 7)];
        shift_reg_ = (shift_reg_ << 1) + ((tact >> 2) & 1);
        chan_ = slot_ids[shift_reg_ & 7];
        break;
      }
      case DMR_T1_VOICE_SYNC_MAGIC:
        shift_reg_ = 0;
        chan_ = 0;
        break;
      case DMR_T2_VOICE_SYNC_MAGIC:
        shift_reg_ = 1;
        chan_ = 1;
        break;
      default:
        break;
      }
      slot_type_[chan_] = sl_sync; // dmr_slot::load_slot: voice/data decision follows the latest sync
    } else {
      shift_reg_ = (shift_reg_ << 1) + ((chan_ + 1) % 2);
      chan_ = slot_ids[shift_reg_ & 7];
    }
    if (sync_rxd)
      expires_ = symbol_count_ + kExpiration;

    int const chan = chan_;
    bool unmute = is_voice_sync(slot_type_[chan]);
    if (unmute)
      unmute_until_[chan] = symbol_count_ + kExpiration;
    else
      unmute_until_[chan] = 0;
    if (unmute && symbol_count_ >= unmute_until_[chan])
      unmute = false;

    // Slot selection: a fixed slot, or the first slot to start a call
    if (active_slot_ >= 0 && unmute_until_[active_slot_] == 0)
      active_slot_ = -1; // that call ended
    if (!unmute)
      return;
    if (wanted_slot_ >= 0 ? chan != wanted_slot_ : (active_slot_ >= 0 && chan != active_slot_)) {
      voice_muted_ += 3;
      return;
    }
    active_slot_ = chan;

    // Three 72-bit AMBE codewords per voice burst; the middle one straddles the sync
    uint8_t cw[36];
    codeword(sym + 12, chan);
    std::memcpy(cw, sym + 48, 18);
    std::memcpy(cw + 18, sym + 90, 18);
    codeword(cw, chan);
    codeword(sym + 108, chan);
  }

  // rx_sync::codeword(), CODEWORD_DMR case
  void codeword(uint8_t const *cw, int slot_id)
  {
    ambe_slot& s = slots_[slot_id];
    int b[9];
    int U[4];
    bool do_silence = false;
    bool do_tone = false;

    interleaver_.process_vcw(&s.errs, cw, b, U);
    int rc = mbe_dequantizeAmbeTone(&s.tone, &s.errs, U);
    if (rc >= 0) { // Tone frame
      if (rc == 0) {
        do_tone = true;
        s.err_cnt = 0;
      } else if ((++s.err_cnt < 4) && do_tone) {
        mbe_useLastMbeParms(&s.cur, &s.prev);
      } else {
        do_tone = false;
        do_silence = true;
      }
    } else {
      rc = mbe_dequantizeAmbe2250Parms(&s.cur, &s.prev, &s.errs, b);
      if (rc == 0) { // Voice frame
        do_tone = false;
        s.err_cnt = 0;
      } else if ((++s.err_cnt < 4) && !do_tone) { // Erasure with frame repeat per TIA-102.BABA.5.6
        mbe_useLastMbeParms(&s.cur, &s.prev);
      } else {
        do_tone = false; // Mute after 3 successive frame repeats
        do_silence = true;
      }
    }
    if (s.errs.ER > 0.096) { // Mute if error rate exceeds threshold
      do_tone = false;
      do_silence = true;
    }

    if (do_tone) {
      s.synth.decode_tone(s.tone.ID, s.tone.AD, &s.tone.n);
    } else {
      mbe_moveMbeParms(&s.cur, &s.prev);
      if (!do_silence)
        s.synth.decode_tap(s.cur.L, 0, s.cur.w0, &s.cur.Vl[1], &s.cur.Ml[1]);
    }
    audio_samples *samples = s.synth.audio();
    int16_t pcm[DV_PCM_SAMPLES];
    for (int i = 0; i < DV_PCM_SAMPLES; i++) {
      float snd = 0;
      if (!do_silence && !samples->empty()) {
        snd = samples->front();
        samples->pop_front();
      }
      pcm[i] = static_cast<int16_t>(std::clamp(snd, -32768.0f, 32767.0f));
    }
    if (do_silence)
      voice_muted_++;
    else
      voice_decoded_++;
    if (callback_ != nullptr)
      callback_(user_data_, pcm);
  }

  int const wanted_slot_;   // 0 or 1; -1 = either
  dv_pcm_callback const callback_;
  void * const user_data_;
  dv_frontend frontend_;
  dv_fsk4 fsk4_;

  // Framing (rx_sync)
  uint8_t buf_[kBurstDibits]{};
  uint64_t sync_reg_ = 0;
  uint64_t symbol_count_ = 0;
  uint64_t expires_ = 0;
  int rx_count_ = 0;
  int threshold_ = 0;
  bool locked_ = false;

  // Timeslots (dmr_cai, dmr_slot)
  int shift_reg_ = 0;
  int chan_ = 0;
  uint64_t slot_type_[2]{};
  uint64_t unmute_until_[2]{};
  int active_slot_ = -1;

  // Vocoder
  p25p2_vf interleaver_;
  ambe_slot slots_[2];

  uint64_t symbols_ = 0;
  uint64_t syncs_ = 0;
  uint64_t bursts_ = 0;
  uint64_t voice_decoded_ = 0;
  uint64_t voice_muted_ = 0;
};

} // namespace

dv_decoder *dv_create_dmr(int slot, dv_pcm_callback callback, void *user_data)
{
  return new dmr_decoder(slot, callback, user_data);
}
