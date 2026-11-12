// P25 Phase 2 (TDMA) voice channel receiver: H-DQPSK at 6000 Bd, one of the
// two logical voice channels, AMBE+2 speech.
// Burst framing, ISCH slot tracking, descrambling, ESS and MAC PTT handling
// adapted from OP25 (boatbod/op25) p25p2_framer.cc, p25p2_sync.cc,
// p25p2_tdma.cc and apps/tdma/lfsr.py; see README.md for attribution
// SPDX-License-Identifier: GPL-3.0-or-later
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

#include "cqpsk.h"
#include "dv_ambe.h"
#include "dv_decoder.h"
#include "dv_frontend.h"
#include "upstream/ezpwd/rs"
#include "upstream/frame_sync_magics.h"
#include "upstream/p25p2_tables.h"

namespace {

constexpr int kBurst = 180;          // dibits per timeslot
constexpr int kSuperframe = 12;      // timeslots
constexpr uint8_t kAlgClear = 0x80;
constexpr uint64_t kCallTimeout = 6000; // symbols (1 s) without a burst in sync: the call is over

// The 4320-bit (2160-dibit) superframe scrambling sequence, seeded with the
// WACN, system ID and NAC (OP25 apps/tdma/lfsr.py)
std::vector<uint8_t> make_xormask(uint32_t wacn, uint16_t sysid, uint16_t nac)
{
  uint64_t const seed = (static_cast<uint64_t>(wacn & 0xfffff) << 24) | (static_cast<uint64_t>(sysid & 0xfff) << 12) | (nac & 0xfff);
  // Initial state: the seed (bit 43 first) times a fixed 44x44 matrix whose
  // row i has ones in columns i, i+4, i+9, i+15, i+20 and i+34
  uint8_t in[44], st[44] = {};
  for (int i = 0; i < 44; i++)
    in[i] = (seed >> (43 - i)) & 1;
  static constexpr int kTaps[] = {0, 4, 9, 15, 20, 34};
  for (int i = 0; i < 44; i++)
    if (in[i])
      for (int t : kTaps)
        if (i + t < 44)
          st[i + t] ^= 1;
  uint64_t reg = 0;
  for (int i = 0; i < 44; i++)
    reg = (reg << 1) | st[i];

  // Six shift registers of 4, 5, 6, 5, 14 and 10 bits, packed MSB first
  auto cycle = [](uint64_t r) {
    uint64_t s1 = (r >> 40) & 0xf, s2 = (r >> 35) & 0x1f, s3 = (r >> 29) & 0x3f;
    uint64_t s4 = (r >> 24) & 0x1f, s5 = (r >> 10) & 0x3fff, s6 = r & 0x3ff;
    uint64_t const c1 = (s1 >> 3) & 1;
    uint64_t const x1 = c1 ^ ((s2 >> 4) & 1), x2 = c1 ^ ((s3 >> 5) & 1), x3 = c1 ^ ((s4 >> 4) & 1);
    uint64_t const x4 = c1 ^ ((s5 >> 13) & 1), x5 = c1 ^ ((s6 >> 9) & 1);
    s1 = ((s1 << 1) & 0xf) | x1;
    s2 = ((s2 << 1) & 0x1f) | x2;
    s3 = ((s3 << 1) & 0x3f) | x3;
    s4 = ((s4 << 1) & 0x1f) | x4;
    s5 = ((s5 << 1) & 0x3fff) | x5;
    s6 = ((s6 << 1) & 0x3ff) | c1;
    return s1 << 40 | s2 << 35 | s3 << 29 | s4 << 24 | s5 << 10 | s6;
  };
  std::vector<uint8_t> mask(kBurst * kSuperframe);
  for (auto& d : mask) {
    uint8_t const hi = (reg >> 43) & 1;
    reg = cycle(reg);
    uint8_t const lo = (reg >> 43) & 1;
    reg = cycle(reg);
    d = static_cast<uint8_t>(hi << 1 | lo);
  }
  return mask;
}

// 7-bit I-ISCH value, -2 for the S-ISCH (sync), -1 if uncorrectable
int isch_lookup(uint8_t const d[20])
{
  uint64_t cw = 0;
  for (int i = 0; i < 20; i++)
    cw = (cw << 2) | d[i];
  if (cw == P25P2_SISCH)
    return -2;
  int best = -1, best_distance = 8; // a (40,9,16) code corrects 7 errors
  for (int i = 0; i < 128; i++) {
    int const distance = __builtin_popcountll(cw ^ P25P2_ISCH_CODEWORDS[i]);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

uint16_t crc12(uint8_t const *bits, unsigned len)
{
  static constexpr uint8_t poly[13] = {1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1};
  uint8_t buf[256 + 13] = {};
  std::memcpy(buf, bits, len);
  for (unsigned i = 0; i < len; i++)
    if (buf[i])
      for (unsigned j = 0; j < 13; j++)
        buf[i + j] ^= poly[j];
  uint16_t crc = 0;
  for (unsigned i = 0; i < 12; i++)
    crc = static_cast<uint16_t>((crc << 1) | buf[len + i]);
  return crc ^ 0xfff;
}

class p25p2_decoder : public dv_decoder {
public:
  p25p2_decoder(dv_config const& config, dv_pcm_callback callback, void *user_data)
    : callback_(callback), user_data_(user_data), frontend_(250.0f),
      slot_(config.p25_slot), nac_(config.p25_nac),
      xormask_(make_xormask(config.p25_wacn, config.p25_sysid, config.p25_nac))
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

  void get_stats(dv_stats *stats) const override
  {
    *stats = stats_;
    stats->samples = frontend_.samples;
    stats->freq_offset = frontend_.frequency_offset_hz();
    stats->active_slot = in_call_ ? slot_ + 1 : 0;
    stats->m17_encrypted = in_call_ && algid_ != kAlgClear;
    stats->last_nac = nac_;
    stats->dmr_cc = -1;
    stats->m17_can = -1;
    stats->active_mode = -1;
    if (in_call_ && source_ != 0)
      snprintf(stats->src_call, sizeof stats->src_call, "%u", source_);
    if (in_call_ && group_ != 0)
      snprintf(stats->dst_call, sizeof stats->dst_call, "%u", group_);
  }

  void signal_lost() override
  {
    allowance_ = 0;
    next_ = 0;
    reset_call();
  }

private:
  // Forget the call and the superframe position (the counters stay)
  void reset_call()
  {
    confident_ = false;
    slotid_ = 0;
    in_call_ = false;
    burst_id_ = -1;
    ess_b_.fill(0);
    algid_ = next_algid_ = kAlgClear;
    alg_known_ = false;
    source_ = 0;
    group_ = 0;
  }

  void leveled(cfloat lev)
  {
    demod_.process(lev, [this](uint8_t raw) { dibit(raw); });
  }

  // Burst framing (OP25 p25p2_framer): a frame sync starts a burst; up to
  // 10 more follow back to back without one
  void dibit(uint8_t raw)
  {
    stats_.symbols++;
    if ((in_call_ || confident_) && stats_.symbols - last_good_ > kCallTimeout)
      reset_call(); // the signal faded away without a MAC_END_PTT
    accum_ = (accum_ << 2) | (raw & 3);
    struct variant { uint64_t sync; int tolerance; std::array<uint8_t, 4> map; };
    static constexpr variant kVariants[] = {
      {P25P2_FRAME_SYNC_MAGIC, 4, {0, 1, 2, 3}},
      {P25P2_FRAME_SYNC_REV_P, 0, {2, 3, 0, 1}},
      {P25P2_FRAME_SYNC_X2400, 0, {3, 2, 1, 0}},
      {P25P2_FRAME_SYNC_N1200, 0, {1, 3, 0, 2}},
      {P25P2_FRAME_SYNC_P1200, 0, {2, 0, 3, 1}},
    };
    uint64_t const word = accum_ & P25P2_FRAME_SYNC_MASK;
    for (size_t v = 0; v < sizeof kVariants / sizeof kVariants[0]; v++) {
      if (__builtin_popcountll(word ^ kVariants[v].sync) <= kVariants[v].tolerance) {
        map_ = v;
        stats_.syncs++;
        for (int i = 0; i < 20; i++)
          burst_[i] = (P25P2_FRAME_SYNC_MAGIC >> (38 - 2 * i)) & 3;
        next_ = 20;
        allowance_ = 10;
        return;
      }
    }
    if (allowance_ == 0)
      return;
    burst_[next_++] = kVariants[map_].map[raw & 3];
    if (next_ == kBurst) {
      next_ = 0;
      allowance_--;
      burst();
    }
  }

  // Superframe position from the ISCH (OP25 p25p2_sync)
  bool track(uint8_t const *d)
  {
    static constexpr int kExpected[12] = {0, 1, -2, -2, 4, 5, -2, -2, 8, 9, -2, -2};
    if (++slotid_ >= kSuperframe)
      slotid_ = 0;
    int const rc = isch_lookup(d);
    if (rc >= 0) {
      int const chn = (rc >> 5) & 3;
      int const loc = (rc >> 3) & 3;
      int const check = loc * 4 + chn;
      if (check >= kSuperframe) {
        // Not a position in the 12-burst superframe: a corrupted I-ISCH.
        // (Used as a slot number, it would index past the descrambling sequence.)
        confident_ = false;
        return false;
      }
      // An I-ISCH says where in the superframe we are
      confident_ = true;
      slotid_ = check;
    } else if (rc == -2 && kExpected[slotid_] != -2) {
      confident_ = false;
    }
    return confident_;
  }

  void burst()
  {
    if (!track(burst_.data()))
      return;
    last_good_ = stats_.symbols;
    uint8_t const *b = burst_.data() + 10;
    uint8_t const duid_bits = static_cast<uint8_t>(b[10] << 6 | b[47] << 4 | b[132] << 2 | b[169]);
    int const type = P25P2_DUID_LOOKUP[duid_bits];
    static constexpr int kWhichSlot[12] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0};
    if (kWhichSlot[slotid_] != slot_)
      return;
    stats_.frames++;
    uint8_t x[kBurst - 10];
    for (int i = 0; i < kBurst - 10; i++)
      x[i] = b[i] ^ xormask_[slotid_ * kBurst + i];

    switch (type) {
    case 0: // 4V
    case 6: // 2V
      voice_burst(x, type == 6);
      break;
    case 3: // SACCH, scrambled
      acch(x, false);
      break;
    case 9: // FACCH, scrambled
      acch(x, true);
      break;
    case 12: // SACCH
      acch(b, false);
      break;
    case 15: // FACCH
      acch(b, true);
      break;
    default:
      break;
    }
  }

  void voice_burst(uint8_t const *x, bool two_v)
  {
    if (!in_call_) {
      in_call_ = true;
      burst_id_ = -1;
    }
    burst_id_ = two_v ? 4 : (burst_id_ + 1) % 5;
    ess(x + 84);
    voice(x + 11);
    voice(x + 48);
    if (!two_v) {
      voice(x + 96);
      voice(x + 133);
    } else {
      algid_ = next_algid_; // the ESS just completed applies from here
    }
  }

  void voice(uint8_t const *dibits)
  {
    int16_t pcm[DV_PCM_SAMPLES];
    // Joined mid-call: stay quiet until a MAC_PTT or an ESS says whether
    // the call is clear (at most one superframe), rather than play ciphertext
    if (algid_ != kAlgClear || !alg_known_) {
      stats_.voice_muted++;
      return;
    }
    if (ambe_.decode(dibits, pcm))
      stats_.voice_decoded++;
    else
      stats_.voice_muted++;
    stats_.error_rate = ambe_.error_rate();
    if (callback_ != nullptr)
      callback_(user_data_, pcm);
  }

  // Encryption sync: ESS-B in 4 hexbits per 4V burst, ESS-A (its parity) in the 2V
  void ess(uint8_t const *d)
  {
    if (burst_id_ < 4) {
      for (int i = 0; i < 12; i += 3)
        ess_b_[4 * burst_id_ + i / 3] = static_cast<uint8_t>(d[i] << 4 | d[i + 1] << 2 | d[i + 2]);
      return;
    }
    std::vector<uint8_t> a(28);
    for (int i = 0, j = 0; i < 28; i++) {
      a[i] = static_cast<uint8_t>(d[j] << 4 | d[j + 1] << 2 | d[j + 2]);
      j += i == 15 ? 4 : 3; // skip the DUID dibit
    }
    std::vector<uint8_t> bb(ess_b_.begin(), ess_b_.end());
    int const ec = rs28_.decode(bb, a);
    if (ec >= 0 && ec <= 14) {
      next_algid_ = static_cast<uint8_t>(bb[0] << 2 | bb[1] >> 4);
      algid_ = next_algid_; // the 2V burst that completes the ESS applies it anyway
      alg_known_ = true;
    }
  }

  // SACCH / FACCH: RS(63,35) shortened, CRC-12, then a MAC PDU
  void acch(uint8_t const *d, bool fast)
  {
    uint8_t bits[512];
    unsigned n = 0;
    auto take = [&](int from, int count) {
      for (int i = from; i < from + count; i++) {
        bits[n++] = (d[i] >> 1) & 1;
        bits[n++] = d[i] & 1;
      }
    };
    std::vector<uint8_t> hb(63, 0);
    std::vector<int> erasures;
    unsigned j, len;
    if (fast) {
      take(11, 36); take(48, 31); take(100, 32); take(133, 36);
      j = 9;
      len = 270;
      erasures = {0, 1, 2, 3, 4, 5, 6, 7, 8, 54, 55, 56, 57, 58, 59, 60, 61, 62};
    } else {
      take(11, 36); take(48, 84); take(133, 36);
      j = 5;
      len = 312;
      erasures = {0, 1, 2, 3, 4, 57, 58, 59, 60, 61, 62};
    }
    unsigned const first = j;
    for (unsigned i = 0; i < len; i += 6)
      hb[j++] = static_cast<uint8_t>(bits[i] << 5 | bits[i + 1] << 4 | bits[i + 2] << 3
                                     | bits[i + 3] << 2 | bits[i + 4] << 1 | bits[i + 5]);
    if (rs28_.decode(hb, erasures) < 0)
      return;
    // The corrected message and its CRC-12
    unsigned const data = fast ? 144 : 168;
    j = first;
    for (unsigned i = 0; i < data + 12; i += 6, j++)
      for (int k = 0; k < 6; k++)
        bits[i + k] = (hb[j] >> (5 - k)) & 1;
    uint16_t sent = 0;
    for (unsigned i = 0; i < 12; i++)
      sent = static_cast<uint16_t>(sent << 1 | bits[data + i]);
    if (crc12(bits, data) != sent)
      return;
    uint8_t pdu[21];
    for (unsigned i = 0; i < data / 8; i++) {
      pdu[i] = 0;
      for (int k = 0; k < 8; k++)
        pdu[i] = static_cast<uint8_t>(pdu[i] << 1 | bits[i * 8 + k]);
    }
    mac(pdu);
  }

  void mac(uint8_t const *pdu)
  {
    switch (pdu[0] >> 5) {
    case 1: // MAC_PTT: a call starts
      in_call_ = true;
      burst_id_ = -1;
      algid_ = next_algid_ = pdu[10];
      alg_known_ = true;
      source_ = static_cast<uint32_t>(pdu[13] << 16 | pdu[14] << 8 | pdu[15]);
      group_ = static_cast<uint16_t>(pdu[16] << 8 | pdu[17]);
      break;
    case 3: // MAC_IDLE
    case 4: // MAC_ACTIVE
    case 6: // MAC_HANGTIME
      // The first message: GRP_V_CH_USER names the call's talkgroup and talker
      if ((pdu[1] & 0x3f) == 0x01 && (pdu[1] >> 6) == 0) {
        group_ = static_cast<uint16_t>(pdu[3] << 8 | pdu[4]);
        uint32_t const src = static_cast<uint32_t>(pdu[5] << 16 | pdu[6] << 8 | pdu[7]);
        if (src != 0)
          source_ = src;
      }
      break;
    case 2: // MAC_END_PTT
      in_call_ = false;
      algid_ = next_algid_ = kAlgClear;
      alg_known_ = false;
      source_ = 0;
      group_ = 0;
      break;
    default:
      break;
    }
  }

  dv_pcm_callback const callback_;
  void * const user_data_;
  dv_frontend frontend_;
  cqpsk_demod<4, 12> demod_;
  int const slot_;
  uint16_t const nac_;
  std::vector<uint8_t> const xormask_;
  ezpwd::RS<63, 35> rs28_;
  dv_ambe2450 ambe_;
  dv_stats stats_{};

  uint64_t accum_ = 0;
  size_t map_ = 0;
  std::array<uint8_t, kBurst> burst_{};
  int next_ = 0;
  int allowance_ = 0;
  int slotid_ = 0;
  bool confident_ = false;
  uint64_t last_good_ = 0;   // symbol count of the last burst in sync

  bool in_call_ = false;
  int burst_id_ = -1;
  std::array<uint8_t, 16> ess_b_{};
  uint8_t algid_ = kAlgClear, next_algid_ = kAlgClear;
  bool alg_known_ = false;   // a MAC_PTT or ESS of this call has given the ALGID
  uint32_t source_ = 0;
  uint16_t group_ = 0;
};

} // namespace

dv_decoder *dv_create_p25p2(dv_config const& config, dv_pcm_callback callback, void *user_data)
{
  return new p25p2_decoder(config, callback, user_data);
}
