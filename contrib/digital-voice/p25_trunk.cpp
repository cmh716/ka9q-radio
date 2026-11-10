// P25 Phase 1 control channel: TSDU trunking signalling blocks (TSBKs).
// Block layout, interleave table and CRC follow OP25 p25p1_fdma.cc
// (block_deinterleave, from Michael Ossmann's p25craft); message layouts per
// TIA-102.AABC. The rate-1/2 trellis is decoded with a Viterbi search rather
// than OP25's greedy one.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "p25_trunk.h"

#include <cstdio>
#include <cstring>

namespace {

// Bit positions of the 98 interleaved dibits (OP25 deinterleave_tb)
constexpr uint8_t kDeinterleave[196] = {
  0,  1,  2,  3,  52, 53, 54, 55, 100,101,102,103, 148,149,150,151,
  4,  5,  6,  7,  56, 57, 58, 59, 104,105,106,107, 152,153,154,155,
  8,  9, 10, 11,  60, 61, 62, 63, 108,109,110,111, 156,157,158,159,
  12, 13, 14, 15,  64, 65, 66, 67, 112,113,114,115, 160,161,162,163,
  16, 17, 18, 19,  68, 69, 70, 71, 116,117,118,119, 164,165,166,167,
  20, 21, 22, 23,  72, 73, 74, 75, 120,121,122,123, 168,169,170,171,
  24, 25, 26, 27,  76, 77, 78, 79, 124,125,126,127, 172,173,174,175,
  28, 29, 30, 31,  80, 81, 82, 83, 128,129,130,131, 176,177,178,179,
  32, 33, 34, 35,  84, 85, 86, 87, 132,133,134,135, 180,181,182,183,
  36, 37, 38, 39,  88, 89, 90, 91, 136,137,138,139, 184,185,186,187,
  40, 41, 42, 43,  92, 93, 94, 95, 140,141,142,143, 188,189,190,191,
  44, 45, 46, 47,  96, 97, 98, 99, 144,145,146,147, 192,193,194,195,
  48, 49, 50, 51 };

// Rate-1/2 trellis: the 4-bit codeword sent for input dibit j in state s
// (the state is the previous input dibit)
constexpr uint8_t kTrellis[4][4] = {
  {0x2, 0xC, 0x1, 0xF},
  {0xE, 0x0, 0xD, 0x3},
  {0x9, 0x7, 0xA, 0x4},
  {0x5, 0xB, 0x6, 0x8}
};

constexpr size_t kSyncNidBits = 48 + 64;
constexpr size_t kBlockBits = 196;
// Calls not mentioned for this long are over (in 4800 Bd symbols)
constexpr uint64_t kCallHold = 4800 * 3;

uint16_t be16(uint8_t const *p) { return static_cast<uint16_t>(p[0] << 8 | p[1]); }
uint32_t be24(uint8_t const *p) { return static_cast<uint32_t>(p[0] << 16 | p[1] << 8 | p[2]); }
uint32_t be32(uint8_t const *p) { return static_cast<uint32_t>(p[0]) << 24 | be24(p + 1); }

} // namespace

int p25_trunk::block_decode(uint8_t const *bits, uint8_t out[12])
{
  constexpr int kSteps = 49; // 48 data dibits and a flush dibit
  uint8_t cw[kSteps];
  for (int k = 0; k < kSteps; k++)
    cw[k] = static_cast<uint8_t>(bits[kDeinterleave[4 * k]] << 3 | bits[kDeinterleave[4 * k + 1]] << 2
                                 | bits[kDeinterleave[4 * k + 2]] << 1 | bits[kDeinterleave[4 * k + 3]]);

  // Viterbi over 4 states, Hamming metric; the state after a step is the input dibit
  constexpr int kInf = 1 << 20;
  int metric[4] = {0, kInf, kInf, kInf};
  uint8_t from[kSteps][4];
  for (int k = 0; k < kSteps; k++) {
    int next[4] = {kInf, kInf, kInf, kInf};
    for (int s = 0; s < 4; s++) {
      if (metric[s] >= kInf)
        continue;
      for (int j = 0; j < 4; j++) {
        int const m = metric[s] + __builtin_popcount(cw[k] ^ kTrellis[s][j]);
        if (m < next[j]) {
          next[j] = m;
          from[k][j] = static_cast<uint8_t>(s);
        }
      }
    }
    std::memcpy(metric, next, sizeof metric);
  }
  // The flush dibit is 0; trace back from whichever state is best anyway
  int state = 0;
  for (int s = 1; s < 4; s++)
    if (metric[s] < metric[state])
      state = s;
  int const errors = metric[state];
  uint8_t dibits[kSteps];
  for (int k = kSteps - 1; k >= 0; k--) {
    dibits[k] = static_cast<uint8_t>(state);
    state = from[k][state];
  }
  std::memset(out, 0, 12);
  for (int d = 0; d < 48; d++)
    out[d >> 2] |= static_cast<uint8_t>(dibits[d] << (6 - (d % 4) * 2));
  return errors;
}

bool p25_trunk::crc_ok(uint8_t const block[12])
{
  // CRC-CCITT over 10 octets plus the inverted check, as OP25 crc16()
  uint32_t const poly = (1 << 12) | (1 << 5) | 1;
  uint32_t crc = 0;
  for (int i = 0; i < 12; i++) {
    for (int j = 0; j < 8; j++) {
      crc = ((crc << 1) | ((block[i] >> (7 - j)) & 1)) & 0x1ffff;
      if (crc & 0x10000)
        crc = (crc & 0xffff) ^ poly;
    }
  }
  return ((crc ^ 0xffff) & 0xffff) == 0;
}

void p25_trunk::tsdu(std::vector<bool> const& frame, size_t bits, uint16_t nac)
{
  // Drop the status dibit after every 35 data dibits
  uint8_t bv[720];
  size_t n = 0;
  for (size_t d = 0; d < bits / 2 && n + 2 <= sizeof bv; d++) {
    if ((d + 1) % 36 == 0)
      continue;
    bv[n++] = frame[2 * d];
    bv[n++] = frame[2 * d + 1];
  }
  if (n < kSyncNidBits)
    return;
  size_t const blocks = (n - kSyncNidBits) / kBlockBits;
  for (size_t b = 0; b < blocks; b++) {
    uint8_t t[12];
    block_decode(bv + kSyncNidBits + b * kBlockBits, t);
    if (!crc_ok(t)) {
      tsbk_errors_++;
      return;
    }
    tsbks_++;
    tsbk(t, nac);
    if (t[0] & 0x80) // last block
      break;
  }
}

uint64_t p25_trunk::channel_hz(uint16_t ch, int *slot) const
{
  iden const& id = idens_[ch >> 12];
  uint32_t const number = ch & 0xfff;
  *slot = -1;
  if (!id.valid)
    return 0;
  if (id.slots > 1)
    *slot = static_cast<int>(number % id.slots);
  return id.base_hz + static_cast<uint64_t>(id.spacing_hz) * (number / id.slots);
}

void p25_trunk::emit(dv_event& e)
{
  if (callback_ != nullptr)
    callback_(user_data_, &e);
}

void p25_trunk::grant(uint16_t ch, uint32_t group, uint32_t source, uint8_t opts, bool unit)
{
  // Forget calls nobody has mentioned lately
  for (auto it = calls_.begin(); it != calls_.end();) {
    if (now_ - it->second.seen > kCallHold)
      it = calls_.erase(it);
    else
      ++it;
  }
  auto it = calls_.find(ch);
  bool const known = it != calls_.end() && it->second.group == group;
  if (known) {
    it->second.seen = now_;
    if (source != 0)
      it->second.source = source; // perhaps a new talker; not worth a GRANT
    if (opts != 0)
      it->second.opts = opts;
  } else {
    calls_[ch] = call{group, source, now_, opts};
    grants_++;
  }

  dv_event e{};
  e.type = known ? DV_EVENT_UPDATE : DV_EVENT_GRANT;
  e.channel = ch;
  e.freq_hz = channel_hz(ch, &e.slot);
  e.group = group;
  e.source = known ? calls_[ch].source : source;
  e.unit = unit;
  uint8_t const o = calls_[ch].opts;
  e.encrypted = o & 0x40;
  e.emergency = o & 0x80;
  if (known) {
    emit(e);
    return;
  }
  char where[40];
  if (e.freq_hz == 0)
    snprintf(where, sizeof where, "ch %X-%u", ch >> 12, ch & 0xfff);
  else if (e.slot >= 0)
    snprintf(where, sizeof where, "%.4f MHz TDMA slot %d", e.freq_hz / 1e6, e.slot);
  else
    snprintf(where, sizeof where, "%.4f MHz", e.freq_hz / 1e6);
  char from[20] = "";
  if (source != 0)
    snprintf(from, sizeof from, " from %u", source);
  snprintf(e.text, sizeof e.text, "grant %s %u%s on %s%s%s", unit ? "unit" : "TG", group, from, where,
           e.encrypted ? " encrypted" : "", e.emergency ? " EMERGENCY" : "");
  emit(e);
}

void p25_trunk::set_iden(uint8_t opcode, uint8_t const t[12])
{
  uint32_t const bits = be32(t + 2);
  iden id;
  id.valid = true;
  id.base_hz = static_cast<uint64_t>(be32(t + 6)) * 5;
  id.spacing_hz = (bits & 0x3ff) * 125;
  auto const signed_offset = [](uint32_t v, int width, int64_t unit) {
    int64_t const mag = static_cast<int64_t>(v & ((1u << (width - 1)) - 1)) * unit;
    return (v >> (width - 1)) & 1 ? mag : -mag;
  };
  switch (opcode) {
  case 0x3d: // IDEN_UP (VHF/UHF/700/800/900)
    id.bandwidth_hz = (bits >> 19 & 0x1ff) * 125;
    id.tx_offset_hz = signed_offset(bits >> 10 & 0x1ff, 9, 250000);
    break;
  case 0x34: // IDEN_UP_VU
    id.bandwidth_hz = (bits >> 24 & 0xf) == 4 ? 6250 : (bits >> 24 & 0xf) == 5 ? 12500 : 0;
    id.tx_offset_hz = signed_offset(bits >> 10 & 0x3fff, 14, id.spacing_hz);
    break;
  default: { // 0x33 IDEN_UP_TDMA: channel type gives bandwidth and slots
    static constexpr struct { uint32_t bw; int slots; } kType[] = {
      {12500, 1}, {12500, 1}, {6250, 1}, {12500, 2}, {25000, 4}, {12500, 2}};
    unsigned const type = bits >> 24 & 0xf;
    if (type >= sizeof kType / sizeof kType[0])
      return;
    id.bandwidth_hz = kType[type].bw;
    id.slots = kType[type].slots;
    id.tx_offset_hz = signed_offset(bits >> 10 & 0x3fff, 14, id.spacing_hz);
    break;
  }
  }
  iden& old = idens_[bits >> 28];
  bool const changed = !old.valid || old.base_hz != id.base_hz || old.spacing_hz != id.spacing_hz
                       || old.slots != id.slots;
  old = id;
  if (changed) {
    dv_event e{};
    e.type = DV_EVENT_SITE;
    snprintf(e.text, sizeof e.text, "band plan %u: base %.5f MHz, spacing %.2f kHz, %s, %u Hz wide",
             bits >> 28, id.base_hz / 1e6, id.spacing_hz / 1e3,
             id.slots > 1 ? "TDMA" : "FDMA", id.bandwidth_hz);
    emit(e);
  }
}

void p25_trunk::tsbk(uint8_t const t[12], uint16_t nac)
{
  (void)nac;
  if (t[0] & 0x40) // protected (encrypted) TSBK
    return;
  uint8_t const opcode = t[0] & 0x3f;
  uint8_t const mfid = t[1];
  if (mfid == 0x90) { // Motorola
    if (opcode == 0x02) // MOT_GRG_CN_GRANT: supergroup (patch) grant
      grant(be16(t + 3), be16(t + 5), be24(t + 7), t[2], false);
    else if (opcode == 0x03) { // MOT_GRG_CN_GRANT_UPDT
      grant(be16(t + 2), be16(t + 4), 0, 0, false);
      if (be16(t + 6) != be16(t + 2))
        grant(be16(t + 6), be16(t + 8), 0, 0, false);
    }
    return;
  }
  if (mfid > 1)
    return;
  switch (opcode) {
  case 0x00: // GRP_V_CH_GRANT
    grant(be16(t + 3), be16(t + 5), be24(t + 7), t[2], false);
    break;
  case 0x02: // GRP_V_CH_GRANT_UPDT: two channel/group pairs
    grant(be16(t + 2), be16(t + 4), 0, 0, false);
    if (be16(t + 6) != be16(t + 2))
      grant(be16(t + 6), be16(t + 8), 0, 0, false);
    break;
  case 0x03: // GRP_V_CH_GRANT_UPDT_EXP
    grant(be16(t + 4), be16(t + 8), 0, t[2], false);
    break;
  case 0x04: // UU_V_CH_GRANT
  case 0x06: // UU_V_CH_GRANT_UPDT
    grant(be16(t + 2), be24(t + 4), be24(t + 7), 0, true);
    break;
  case 0x33: // IDEN_UP_TDMA
  case 0x34: // IDEN_UP_VU
  case 0x3d: // IDEN_UP
    set_iden(opcode, t);
    break;
  case 0x3a: { // RFSS_STS_BCST
    uint16_t const sysid = be16(t + 3) & 0xfff;
    if (!site_known_ || sysid != sysid_ || t[5] != rfss_ || t[6] != site_) {
      sysid_ = sysid;
      rfss_ = t[5];
      site_ = t[6];
      site_known_ = true;
      dv_event e{};
      e.type = DV_EVENT_SITE;
      snprintf(e.text, sizeof e.text, "site: system %03X RFSS %u site %u", sysid_, rfss_, site_);
      emit(e);
    }
    break;
  }
  case 0x3b: { // NET_STS_BCST
    uint32_t const v = be32(t + 3);
    uint32_t const wacn = v >> 12;
    if (wacn != wacn_) {
      wacn_ = wacn;
      sysid_ = v & 0xfff;
      dv_event e{};
      e.type = DV_EVENT_SITE;
      snprintf(e.text, sizeof e.text, "network: WACN %05X system %03X", wacn_, sysid_);
      emit(e);
    }
    break;
  }
  default:
    break;
  }
}

void p25_trunk::get_stats(dv_stats *stats) const
{
  stats->tsbks = tsbks_;
  stats->tsbk_errors = tsbk_errors_;
  stats->wacn = wacn_;
  stats->sysid = sysid_;
  stats->rfss = rfss_;
  stats->site = site_;
}
