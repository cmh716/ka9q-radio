// DMR (Tier I/II) voice receiver: 4FSK demodulation, burst framing,
// timeslot tracking and AMBE+2 decoding
// Framing and voice handling follow OP25 (boatbod/op25 718e589a) rx_sync.cc
// and dmr_cai.cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

#include "dv_decoder.h"
#include "dv_frontend.h"
#include "upstream/bit_utils.h"
#include "upstream/check_frame_sync.h"
#include "upstream/dmr_const.h"
#include "upstream/frame_sync_magics.h"
#include "dv_ambe.h"
#include "upstream/bptc19696.h"
#include "upstream/ezpwd/rs"
#include "upstream/golay2087.h"
#include "upstream/hamming.h"

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

// dmr_slot.h: burst field positions (bits from the start of the slot, after the CACH)
constexpr int kSlotTypeL = 98, kSlotTypeR = 156, kSyncEmb = 108;
constexpr uint8_t kVoiceLcHeaderMask[24] = {1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0}; // 0x969696
constexpr uint8_t kDataTypePiHeader = 0, kDataTypeVoiceLc = 1, kDataTypeTerminator = 2;

bool is_voice_sync(uint64_t sync)
{
  return sync == DMR_BS_VOICE_SYNC_MAGIC || sync == DMR_MS_VOICE_SYNC_MAGIC
      || sync == DMR_T1_VOICE_SYNC_MAGIC || sync == DMR_T2_VOICE_SYNC_MAGIC;
}

class dmr_decoder : public dv_decoder {
  struct call; // per-timeslot call information, below
public:
  dmr_decoder(int slot, dv_pcm_callback callback, void *user_data)
    : wanted_slot_(slot - 1), callback_(callback), user_data_(user_data),
      frontend_(350.0f), fsk4_(DV_RRC_TAPS)
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
    uint8_t dibit;
    if (fsk4_.process(lev, dibit))
      rx_dibit(dibit);
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
    stats->dmr_cc = cc_;
    if (active_slot_ >= 0) {
      stats->error_rate = slots_[active_slot_].error_rate();
      call const& c = calls_[active_slot_];
      if (c.have_ids) {
        std::snprintf(stats->src_call, sizeof stats->src_call, "%u", c.source);
        std::snprintf(stats->dst_call, sizeof stats->dst_call, "%s%u", c.group ? "TG " : "", c.destination);
      }
      stats->m17_encrypted = c.encrypted;
    }
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
      sync_reset();  // no sync for too long: the transmission is over
      end_calls();
      return;
    }
    if (rx_count_ < kBurstDibits)
      return;
    rx_count_ = 0;
    burst(buf_ + sizeof buf_ - kBurstDibits);
  }

  void signal_lost() override
  {
    sync_reset();
    end_calls();
  }

  // Forget both slots' calls: encryption, IDs, embedded LC fragments. (Not on
  // a mere resync, which happens within a call)
  void end_calls()
  {
    for (int s = 0; s < 2; s++) {
      calls_[s] = call{};
      emb_lc_[s].clear();
      voice_burst_[s] = 5;
      slot_type_[s] = 0;
    }
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
      if (is_voice_sync(sl_sync))
        voice_burst_[chan_] = 0; // burst A of a voice superframe
      if (sl_sync == DMR_BS_DATA_SYNC_MAGIC || sl_sync == DMR_MS_DATA_SYNC_MAGIC)
        data_burst(bits + 24, chan_);
    } else {
      shift_reg_ = (shift_reg_ << 1) + ((chan_ + 1) % 2);
      chan_ = slot_ids[shift_reg_ & 7];
      // Voice bursts B-F carry the EMB (and pieces of the embedded LC) where
      // the sync would be: they identify a call joined after its header
      if (is_voice_sync(slot_type_[chan_]) && voice_burst_[chan_] < 5)
        emb_burst(bits + 24, chan_, ++voice_burst_[chan_]);
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

  // dmr_slot::decode_slot_type() and the headers we need for voice: the
  // color code, who is calling whom, encryption, and the end of a call
  void data_burst(uint8_t const *slot, int chan)
  {
    bit_vector st;
    for (int i = kSlotTypeL; i < kSyncEmb; i++)
      st.push_back(slot[i]);
    for (int i = kSlotTypeR; i < kSlotTypeR + 10; i++)
      st.push_back(slot[i]);
    if (CGolay2087::decode(st) > 2)
      return; // golay(20,8) reliably corrects at most 2 bit errors
    int const cc = (st[0] << 3) | (st[1] << 2) | (st[2] << 1) | st[3];
    int const type = (st[4] << 3) | (st[5] << 2) | (st[6] << 1) | st[7];
    cc_ = cc;
    call& c = calls_[chan];
    uint8_t info[96];
    switch (type) {
    case kDataTypePiHeader: // only sent on encrypted calls
      if (bptc_.decode(slot, info))
        c.encrypted = true;
      break;
    case kDataTypeVoiceLc: { // dmr_slot::decode_vlch(), decode_lc()
      if (!bptc_.decode(slot, info))
        break;
      for (int i = 0; i < 24; i++)
        info[72 + i] ^= kVoiceLcHeaderMask[i];
      std::vector<uint8_t> lc(12, 0);
      for (int i = 0; i < 96; i++)
        lc[i / 8] = static_cast<uint8_t>((lc[i / 8] << 1) | info[i]);
      if (rs12_.decode(lc) < 0)
        break;
      int const flco = lc[0] & 0x3f;
      c.group = flco == 0;                  // group voice call (3 = unit to unit)
      c.encrypted = (lc[2] & 0x40) != 0;    // service options: privacy
      c.destination = (lc[3] << 16) | (lc[4] << 8) | lc[5];
      c.source = (lc[6] << 16) | (lc[7] << 8) | lc[8];
      c.have_ids = true;
      break;
    }
    case kDataTypeTerminator:
      c = call{};
      break;
    default:
      break;
    }
  }

  // dmr_slot::decode_emb() and decode_embedded_lc(): the EMB's PI bit marks
  // an encrypted call, and bursts B-E of a voice superframe carry the four
  // fragments of the embedded LC (who is calling whom, and the privacy bit).
  // OP25 places fragments by the EMB's LCSS; here they go by burst position
  // (B first ... E last, as the LCSS would say), so an EMB that fails its
  // QR(16,7) check doesn't lose the LC, which has its own Hamming, parity
  // and checksum
  void emb_burst(uint8_t const *slot, int chan, int burst)
  {
    call& c = calls_[chan];
    bit_vector emb;
    for (int i = kSyncEmb; i < kSyncEmb + 8; i++)
      emb.push_back(slot[i]);
    for (int i = kSlotTypeR - 8; i < kSlotTypeR; i++)
      emb.push_back(slot[i]);
    if (CQR1676::decode(emb) <= 2) { // QR(16,7) reliably corrects 2 bit errors
      int const cc = (emb[0] << 3) | (emb[1] << 2) | (emb[2] << 1) | emb[3];
      if (cc_ < 0 || cc == cc_) { // FEC can pass garbage: the color code must match
        cc_ = cc;
        if (emb[4])
          c.encrypted = true; // PI: privacy
      }
    }
    if (burst < 1 || burst > 4)
      return; // burst F: no LC
    std::vector<uint8_t>& frag = emb_lc_[chan];
    if (burst == 1)
      frag.clear();
    if (frag.size() != static_cast<size_t>(32 * (burst - 1)))
      return; // missed a fragment of this LC
    for (int i = 0; i < 32; i++)
      frag.push_back(slot[kSyncEmb + 8 + i]);
    if (burst == 4)
      embedded_lc(frag, c);
  }

  static void embedded_lc(std::vector<uint8_t> const& frag, call& c)
  {
    bool data[128] = {};
    for (unsigned a = 0, b = 0; a < 128; a++) { // unpacked downwards in columns
      data[b] = frag[a];
      b += 16;
      if (b > 127)
        b -= 127;
    }
    for (unsigned a = 0; a < 112; a += 16)
      if (!CHamming::decode16114(data + a))
        return;
    for (unsigned a = 0; a < 16; a++) {
      bool p = false;
      for (unsigned r = 0; r < 128; r += 16)
        p ^= data[a + r];
      if (p)
        return;
    }
    uint8_t bits[72];
    int n = 0;
    for (unsigned a = 0; a < 11; a++) bits[n++] = data[a];
    for (unsigned a = 16; a < 27; a++) bits[n++] = data[a];
    for (unsigned row = 32; row < 112; row += 16)
      for (unsigned a = row; a < row + 10; a++) bits[n++] = data[a];
    uint8_t lc[9];
    for (int i = 0; i < 9; i++) {
      lc[i] = 0;
      for (int b = 0; b < 8; b++)
        lc[i] = static_cast<uint8_t>((lc[i] << 1) | bits[8 * i + b]);
    }
    // 5-bit checksum (OP25 leaves it switched off; it makes a false LC much less likely)
    unsigned const rx = (data[42] << 4) | (data[58] << 3) | (data[74] << 2) | (data[90] << 1) | data[106];
    unsigned sum = 0;
    for (int i = 0; i < 9; i++)
      sum += lc[i];
    if (rx != sum % 31)
      return;
    int const flco = lc[0] & 0x3f;
    if (flco != 0 && flco != 3)
      return; // not a voice call LC (group or unit to unit)
    c.group = flco == 0;
    if (lc[2] & 0x40)
      c.encrypted = true;
    c.destination = (lc[3] << 16) | (lc[4] << 8) | lc[5];
    c.source = (lc[6] << 16) | (lc[7] << 8) | lc[8];
    c.have_ids = true;
  }

  // rx_sync::codeword(), CODEWORD_DMR case (dv_ambe2450)
  void codeword(uint8_t const *cw, int slot_id)
  {
    int16_t pcm[DV_PCM_SAMPLES];
    bool const ok = slots_[slot_id].decode(cw, pcm);
    if (calls_[slot_id].encrypted) {
      std::memset(pcm, 0, sizeof pcm);
      voice_muted_++;
    } else if (ok) {
      voice_decoded_++;
    } else {
      voice_muted_++;
    }
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
  unsigned shift_reg_ = 0; // only the low 3 bits matter
  int chan_ = 0;
  uint64_t slot_type_[2]{};
  uint64_t unmute_until_[2]{};
  int active_slot_ = -1;

  // Vocoder, one per timeslot
  dv_ambe2450 slots_[2];

  // Per-timeslot call information from the voice LC header / PI header
  struct call {
    bool have_ids = false;
    bool group = true;
    bool encrypted = false;
    unsigned source = 0, destination = 0;
  };
  call calls_[2];
  std::vector<uint8_t> emb_lc_[2]; // embedded LC fragments being collected
  int voice_burst_[2] = {5, 5};     // bursts since the last voice sync (A = 0)
  int cc_ = -1;                 // color code, -1 until a slot type decodes
  CBPTC19696 bptc_;
  ezpwd::RS<255, 252> rs12_;    // Reed-Solomon(12,9), shortened

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
