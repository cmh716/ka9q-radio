// DMR (Tier I/II) voice receiver: 4FSK demodulation, burst framing,
// timeslot tracking and AMBE+2 decoding
// Framing and voice handling follow OP25 (boatbod/op25 718e589a) rx_sync.cc
// and dmr_cai.cc; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "dv_decoder.h"
#include "dv_frontend.h"
#include "upstream/bit_utils.h"
#include "upstream/check_frame_sync.h"
#include "upstream/dmr_const.h"
#include "upstream/frame_sync_magics.h"
#include "dv_ambe.h"
#include "dmr_idle.h"
#include "dmr_alias.h"
#include "dmr_data.h"
#include "upstream/bptc19696.h"
#include "upstream/ezpwd/rs"
#include "upstream/golay2087.h"
#include "upstream/hamming.h"

namespace {

constexpr int kBurstDibits = 144;      // CACH 12 + payload 54 + sync/EMB 24 + payload 54
constexpr int kSyncOffset = 66;        // dibits from burst start to sync
constexpr int kSyncDibits = 24;
constexpr int kExpiration = 1728;      // dibits without a sync before giving up (12 bursts)
constexpr uint64_t kCallTimeout = 4800; // dibits (1 s) without any sync: the calls are over
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
constexpr uint8_t kDataTypeDataHeader = 6, kDataTypeRate12 = 7, kDataTypeRate34 = 8, kDataTypeRate1 = 10;

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
      frontend_(350.0f), fsk4_(new dv_fsk4(DV_RRC_TAPS))
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
    bool have_symbol = fsk4_->process(inverted_ ? std::conj(lev) : lev, dibit);
    float const original_sample = (inverted_ ? -1 : 1) * fsk4_->filtered_sample();
    int normal_color = -1, inverted_color = -1;
    bool const normal_idle = idle_detector_.process(original_sample, normal_color);
    bool const inverted_idle = inverted_idle_detector_.process(-original_sample, inverted_color);
    // DMR voice and data syncs differ by sign inversion (XOR 0xaaaa...).
    // Sync alone therefore cannot establish polarity: inverted idle looks like
    // voice. Require two full Golay-protected idle fields on the burst grid.
    if(normal_idle != inverted_idle) {
      bool const inverted = inverted_idle;
      if(inverted != inverted_) {
        inverted_ = inverted;
        fsk4_.reset(new dv_fsk4(DV_RRC_TAPS));
        have_symbol = false;
        idle_detector_.reset();
        inverted_idle_detector_.reset();
        sync_reset();
        end_calls(); // discard false voice/encryption state from wrong polarity
      }
    }
    if(have_symbol)
      rx_dibit(dibit);
    bool const idle = inverted_ ? inverted_idle : normal_idle;
    int const color = inverted_ ? inverted_color : normal_color;
    // Two Golay-protected idle slot types agreeing, on the burst grid: a
    // color code noise can't fake, so it may set it, locked or not
    if(idle)
      cc_ = color;
    if(idle && !locked_ && symbols_ - last_sync_ > kExpiration) {
      // Protected idle activity can supplement the ordinary framer when its
      // decision-directed timing cannot acquire a repeating idle pattern.
      syncs_++;
      bursts_++;
    }
  }

  void get_stats(dv_stats *stats) const override
  {
    *stats = {};
    stats->call_serial = call_serial_;
    stats->samples = frontend_.samples;
    stats->symbols = symbols_;
    stats->syncs = syncs_;
    stats->frames = bursts_;
    stats->voice_decoded = voice_decoded_;
    stats->voice_muted = voice_muted_;
    stats->freq_offset = frontend_.frequency_offset_hz() + (inverted_ ? -1 : 1) * fsk4_->fine_offset_hz();
    stats->active_slot = active_slot_ + 1;
    stats->dmr_cc = cc_;
    stats->dmr_lc_flcos = lc_flcos_;
    stats->dmr_lc_bad = lc_bad_;
    std::memcpy(stats->dmr_burst_types, burst_types_, sizeof burst_types_);
    stats->dmr_cc_other = cc_mismatch_;
    // Packet data, for diagnosis: counts, and the last message (one whose
    // CRC-32 checked, if any has), on either slot
    {
      uint32_t hdr = 0, hdr_bad = 0, msgs = 0, ok = 0, broken = 0, repeats = 0, crc9 = 0, crc9_all = 0;
      char const *last = "";
      for (int s = 0; s < 2; s++) {
        dmr_data::assembler const& a = data_[s];
        hdr += a.headers();
        hdr_bad += a.headers_bad();
        msgs += a.messages();
        ok += a.crc_ok();
        broken += a.broken();
        repeats += a.repeats();
        crc9 += a.crc9_good();
        crc9_all += a.crc9_good() + a.crc9_bad();
      }
      // The latest message whose CRC-32 checked, on either slot; else the latest of any kind
      if (data_good_slot_ >= 0)
        last = data_[data_good_slot_].best_report();
      else if (data_any_slot_ >= 0)
        last = data_[data_any_slot_].last_report();
      if (hdr + hdr_bad != 0)
        std::snprintf(stats->dmr_pdu, sizeof stats->dmr_pdu,
                      "hdr=%u/%u msgs=%u crc_ok=%u broken=%u repeats=%u crc9=%u/%u r34_clean=%u/%u r34_swapped=%u last=%s",
                      hdr, hdr + hdr_bad, msgs, ok, broken, repeats, crc9, crc9_all, r34_clean_, r34_, r34_swapped_,
                      *last ? last : "-");
    }
    if (active_slot_ >= 0)
      stats->error_rate = slots_[active_slot_].error_rate();
    // The call last played, until it ends (terminator, or sync lost): not
    // just while its audio is unmuted, which drops out on a weak signal
    if (call_slot_ >= 0) {
      call const& c = calls_[call_slot_];
      if (c.have_ids) {
        std::snprintf(stats->src_call, sizeof stats->src_call, "%u", c.source);
        std::snprintf(stats->dst_call, sizeof stats->dst_call, "%s%u", c.group ? "TG " : "", c.destination);
        if (c.alias.complete())
          std::snprintf(stats->talker_alias, sizeof stats->talker_alias, "%s", c.alias.text());
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
    if (symbols_ - last_sync_ == kCallTimeout)
      end_calls(); // no sync at all for 1 s: the transmission is over
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
      last_sync_ = symbols_;
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
      sync_reset(); // lost lock; the call may well continue
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
    idle_detector_.reset();
    inverted_idle_detector_.reset();
  }

  // Forget both slots' calls: encryption, IDs, embedded LC fragments. (Not on
  // a mere resync, which happens within a call)
  void end_calls()
  {
    call_serial_++;
    for (int s = 0; s < 2; s++) {
      calls_[s] = call{};
      emb_lc_[s].clear();
      voice_burst_[s] = 5;
      slot_type_[s] = 0;
    }
    call_slot_ = -1;
  }

  void sync_reset()
  {
    // Packet data in progress can't continue across lost sync (blocks may
    // have gone by unseen, and a later packet's blocks fit a stale header);
    // the counters and the reports made stay
    for (int s = 0; s < 2; s++)
      data_[s].abandon();
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
      case DMR_T1_DATA_SYNC_MAGIC:  // direct mode (simplex): the sync names the slot
        shift_reg_ = 0;
        chan_ = 0;
        break;
      case DMR_T2_VOICE_SYNC_MAGIC:
      case DMR_T2_DATA_SYNC_MAGIC:
        shift_reg_ = 1;
        chan_ = 1;
        break;
      default:
        break;
      }
      slot_type_[chan_] = sl_sync; // dmr_slot::load_slot: voice/data decision follows the latest sync
      if (is_voice_sync(sl_sync))
        voice_burst_[chan_] = 0; // burst A of a voice superframe
      if (sl_sync == DMR_BS_DATA_SYNC_MAGIC || sl_sync == DMR_MS_DATA_SYNC_MAGIC
          || sl_sync == DMR_T1_DATA_SYNC_MAGIC || sl_sync == DMR_T2_DATA_SYNC_MAGIC)
        data_burst(bits + 24, chan_); // headers, terminators and data, direct mode's too
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
    call_slot_ = chan;

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
    // Noise passes Golay(20,8) about 1 time in 20 while the framer's lock
    // lingers after a transmission: such a burst has a random color code
    // and type. Only a burst whose contents check (a voice LC header's
    // Reed-Solomon, a data header's CRC) may set the color code; any other
    // must match it, or is ignored (a noise terminator mustn't end a call)
    bool const self_checked = type == kDataTypeVoiceLc || type == kDataTypeDataHeader;
    if (!self_checked && cc != cc_) {
      cc_mismatch_++;
      return;
    }
    // (Not BPTC as well: on a weak signal real terminators and idle bursts
    // fail its strict check, where the slot type and color code pass)
    if (!self_checked)
      burst_types_[type]++; // for diagnosis: which kinds of data come at all
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
      cc_ = cc;
      burst_types_[type]++;
      int const flco = lc[0] & 0x3f;
      c.group = flco == 0;                  // group voice call (3 = unit to unit)
      c.encrypted = (lc[2] & 0x40) != 0;    // service options: privacy
      c.destination = (lc[3] << 16) | (lc[4] << 8) | lc[5];
      set_source(c, (lc[6] << 16) | (lc[7] << 8) | lc[8]);
      c.have_ids = true;
      break;
    }
    case kDataTypeDataHeader: {
      if (!bptc_.decode(slot, info))
        break;
      data_before(chan);
      uint32_t const bad = data_[chan].headers_bad();
      data_[chan].header(info);
      data_after(chan);
      if (data_[chan].headers_bad() == bad) { // its CRC checked
        cc_ = cc;
        burst_types_[type]++;
      }
      break;
    }
    case kDataTypeRate12: {
      if (!bptc_.decode(slot, info))
        break;
      uint8_t bytes[12] = {};
      for (int i = 0; i < 96; i++)
        bytes[i / 8] = static_cast<uint8_t>(bytes[i / 8] << 1 | info[i]);
      data_before(chan);
      data_[chan].block(12, bytes, 12);
      data_after(chan);
      break;
    }
    case kDataTypeRate34:
    case kDataTypeRate1: {
      uint8_t coded[196]; // the two halves, either side of slot type and sync
      std::memcpy(coded, slot, 98);
      std::memcpy(coded + 98, slot + 166, 98);
      if (type == kDataTypeRate34) {
        uint8_t bytes[18];
        bool swapped = false;
        int const metric = dmr_data::trellis34_decode(coded, bytes, &swapped);
        r34_++;
        r34_clean_ += metric <= 16; // a few symbols off at most
        r34_swapped_ += swapped;
        data_before(chan);
        data_[chan].block(34, bytes, 18);
        data_after(chan);
      } else {
        uint8_t bytes[24];
        dmr_data::rate1_bytes(coded, bytes);
        data_before(chan);
        data_[chan].block(1, bytes, 24);
        data_after(chan);
      }
      break;
    }
    case kDataTypeTerminator:
      data_[chan].abandon();
      c = call{};
      if (call_slot_ == chan) {
        call_slot_ = -1;
        call_serial_++;
      }
      break;
    default:
      break;
    }
  }

  // Which slot's packet data report is latest: by what completed (a report
  // made, and whether its CRC-32 checked), not by which slot last had a header
  uint32_t data_reports_ = 0, data_ok_ = 0;
  void data_before(int chan)
  {
    data_reports_ = data_[chan].reports();
    data_ok_ = data_[chan].crc_ok();
  }
  void data_after(int chan)
  {
    if (data_[chan].reports() != data_reports_)
      data_any_slot_ = chan;
    if (data_[chan].crc_ok() != data_ok_)
      data_good_slot_ = chan;
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

  void embedded_lc(std::vector<uint8_t> const& frag, call& c)
  {
    bool data[128] = {};
    for (unsigned a = 0, b = 0; a < 128; a++) { // unpacked downwards in columns
      data[b] = frag[a];
      b += 16;
      if (b > 127)
        b -= 127;
    }
    for (unsigned a = 0; a < 112; a += 16)
      if (!CHamming::decode16114(data + a)) {
        lc_bad_++;
        return;
      }
    for (unsigned a = 0; a < 16; a++) {
      bool p = false;
      for (unsigned r = 0; r < 128; r += 16)
        p ^= data[a + r];
      if (p) {
        lc_bad_++;
        return;
      }
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
    if (rx != sum % 31) {
      lc_bad_++;
      return;
    }
    int const flco = lc[0] & 0x3f;
    lc_flcos_ |= uint64_t{1} << flco;
    if (flco >= 4 && flco <= 7) { // talker alias header, blocks 1-3
      if (lc[1] == 0) // FID: standard (ETSI TS 102 361-2 7.1.1.4)
        talker_alias(c, flco - 4, lc + 2);
      return;
    }
    if (flco != 0 && flco != 3)
      return; // not a voice call LC (group or unit to unit)
    c.group = flco == 0;
    if (lc[2] & 0x40)
      c.encrypted = true;
    c.destination = (lc[3] << 16) | (lc[4] << 8) | lc[5];
    set_source(c, (lc[6] << 16) | (lc[7] << 8) | lc[8]);
    c.have_ids = true;
  }

  // A different talker's alias isn't this one's
  static void set_source(call& c, unsigned source)
  {
    if (c.have_ids && source != c.source)
      c.alias.reset();
    c.source = source;
  }

  static void talker_alias(call& c, int part, uint8_t const *payload)
  {
    c.alias.add(part, payload);
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
  std::unique_ptr<dv_fsk4> fsk4_;
  dmr_idle_detector idle_detector_;
  dmr_idle_detector inverted_idle_detector_;
  bool inverted_ = false;

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
  int call_slot_ = -1;          // slot of the call last played, -1 once it has ended
  uint64_t call_serial_ = 0;    // advances when a call ends
  uint64_t last_sync_ = 0;      // symbols_ at the last sync

  // Vocoder, one per timeslot
  dv_ambe2450 slots_[2];

  // Per-timeslot call information from the voice LC header / PI header
  struct call {
    bool have_ids = false;
    bool group = true;
    bool encrypted = false;
    unsigned source = 0, destination = 0;
    dmr_talker_alias alias; // embedded LC, FLCO 4-7
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
  uint64_t lc_flcos_ = 0;   // FLCOs of the embedded LCs seen (bit n: FLCO n)
  uint64_t lc_bad_ = 0;     // embedded LCs that failed their checks
  uint32_t burst_types_[16]{}; // data and control bursts by slot type data type (color code, BPTC checked)
  uint32_t cc_mismatch_ = 0; // bursts refused: another color code than the one established (noise, mostly)
  dmr_data::assembler data_[2]; // packet data, per slot, for diagnosis
  int data_good_slot_ = -1, data_any_slot_ = -1;
  uint32_t r34_ = 0, r34_clean_ = 0, r34_swapped_ = 0; // rate 3/4 blocks: how well the trellis fit
};

} // namespace

dv_decoder *dv_create_dmr(int slot, dv_pcm_callback callback, void *user_data)
{
  return new dmr_decoder(slot, callback, user_data);
}
