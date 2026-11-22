// DMR packet data, for diagnosis (positions travel this way: Motorola LRRP,
// Hytera and others, as UDP/IP or short data): a data header (ETSI TS
// 102 361-1 9.2.6, BPTC(196,96), CRC-CCITT masked 0xCCCC) and the blocks it
// says follow (rate 1/2 BPTC, rate 3/4 trellis, rate 1 uncoded), the last
// ending in a CRC-32 over the message. Each message is reported as text:
// the header's fields, its length, whether its CRC-32 checked (and how:
// which of the plausible bit orders matched), the IP/UDP header if it is
// one, and its bytes in hex, so its format can be found from the log
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DMR_DATA_H
#define KA9Q_DMR_DATA_H

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dmr_data {

// --- rate 3/4 trellis (ETSI TS 102 361-1 B.2) ---
// Each of the 49 constellation points is a pair of symbols; the encoder's
// state is the last tribit in; the point sent is kTransition[state][tribit]
constexpr int8_t kPoint[16][2] = {
  {+1,-1},{-1,-1},{+3,-3},{-3,-3},{-3,-1},{+3,-1},{-1,-3},{+1,-3},
  {-3,+3},{+3,+3},{-1,+1},{+1,+1},{+1,+3},{-1,+3},{+3,+1},{-3,+1},
};
constexpr uint8_t kTransition[8][8] = {
  {0,8,4,12,2,10,6,14},{4,12,2,10,6,14,0,8},{1,9,5,13,3,11,7,15},{5,13,3,11,7,15,1,9},
  {3,11,7,15,1,9,5,13},{7,15,1,9,5,13,3,11},{2,10,6,14,0,8,4,12},{6,14,0,8,4,12,2,10},
};
// The 98 dibits are sent in this order (dibit sent k is encoded dibit kInterleave[k])
constexpr uint8_t kInterleave[98] = {
  0,1,8,9,16,17,24,25,32,33,40,41,48,49,56,57,64,65,72,73,80,81,88,89,96,97,
  2,3,10,11,18,19,26,27,34,35,42,43,50,51,58,59,66,67,74,75,82,83,90,91,
  4,5,12,13,20,21,28,29,36,37,44,45,52,53,60,61,68,69,76,77,84,85,92,93,
  6,7,14,15,22,23,30,31,38,39,46,47,54,55,62,63,70,71,78,79,86,87,94,95,
};
inline int symbol_of(int dibit) { return dibit == 1 ? 3 : dibit == 0 ? 1 : dibit == 2 ? -1 : -3; }
inline int dibit_of(int symbol) { return symbol == 3 ? 1 : symbol == 1 ? 0 : symbol == -1 ? 2 : 3; }

// The 196 coded bits (a burst's two halves, sync and slot type taken out)
// to 18 bytes. Viterbi, the path's squared distance its metric: 0 for a
// clean block. Returns the metric (the end state must be 0, the encoder's
// tail tribit). Both readings of the interleave are tried, the better kept
// (which one the standard means is settled by the metrics in practice)
inline int trellis34_decode(uint8_t const *bits, uint8_t out[18], bool *swapped = nullptr)
{
  int best = 1 << 30;
  for (int way = 0; way < 2; way++) {
    int sym[98];
    for (int k = 0; k < 98; k++) {
      int const at = way == 0 ? k : kInterleave[k];   // the dibit received
      int const d = bits[2 * at] << 1 | bits[2 * at + 1];
      sym[way == 0 ? kInterleave[k] : k] = symbol_of(d); // the encoder's
    }
    int metric[8], next[8];
    uint8_t from[49][8];
    for (int s = 0; s < 8; s++)
      metric[s] = s == 0 ? 0 : 1 << 28;
    for (int p = 0; p < 49; p++) {
      int const a = sym[2 * p], b = sym[2 * p + 1];
      for (int t = 0; t < 8; t++) {
        next[t] = 1 << 30;
        for (int s = 0; s < 8; s++) {
          int8_t const *q = kPoint[kTransition[s][t]];
          int const m = metric[s] + (a - q[0]) * (a - q[0]) + (b - q[1]) * (b - q[1]);
          if (m < next[t]) {
            next[t] = m;
            from[p][t] = static_cast<uint8_t>(s);
          }
        }
      }
      std::memcpy(metric, next, sizeof metric);
    }
    if (metric[0] >= best)
      continue;
    best = metric[0];
    if (swapped)
      *swapped = way == 1;
    uint8_t tribit[49];
    int s = 0; // the tail tribit leaves the encoder in state 0
    for (int p = 48; p >= 0; p--) {
      tribit[p] = static_cast<uint8_t>(s);
      s = from[p][s];
    }
    std::memset(out, 0, 18);
    for (int p = 0; p < 48; p++)
      for (int i = 0; i < 3; i++)
        if (tribit[p] & (4 >> i))
          out[(3 * p + i) / 8] |= static_cast<uint8_t>(0x80 >> ((3 * p + i) % 8));
  }
  return best;
}

// The encoder, for tests: 18 bytes to 196 coded bits
inline void trellis34_encode(uint8_t const in[18], uint8_t *bits, bool swapped = false)
{
  int sym[98];
  int state = 0;
  for (int p = 0; p < 49; p++) {
    int t = 0;
    if (p < 48)
      for (int i = 0; i < 3; i++)
        t = t << 1 | ((in[(3 * p + i) / 8] >> (7 - (3 * p + i) % 8)) & 1);
    int8_t const *q = kPoint[kTransition[state][t]];
    sym[2 * p] = q[0];
    sym[2 * p + 1] = q[1];
    state = t;
  }
  for (int k = 0; k < 98; k++) {
    int const d = dibit_of(swapped ? sym[k] : sym[kInterleave[k]]);
    int const at = swapped ? kInterleave[k] : k;
    bits[2 * at] = static_cast<uint8_t>(d >> 1);
    bits[2 * at + 1] = static_cast<uint8_t>(d & 1);
  }
}

// --- checks ---
inline uint16_t crc_ccitt(uint8_t const *p, int n)
{
  uint16_t crc = 0;
  for (int i = 0; i < n; i++) {
    crc ^= static_cast<uint16_t>(p[i] << 8);
    for (int b = 0; b < 8; b++)
      crc = static_cast<uint16_t>(crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1);
  }
  return static_cast<uint16_t>(~crc);
}
// A confirmed block's CRC-9 (B.3.10): x^9+x^6+x^4+x^3+1 over its data
// octets then its 7-bit serial, inverted, masked by rate (1/2 0x0f0, 3/4
// 0x1ff, 1 0x10f). The block's first two bytes are the serial and the CRC
inline bool crc9_ok(uint8_t const *block, int n, int rate)
{
  uint16_t crc = 0;
  auto bit = [&crc](int b) {
    int const top = (crc >> 8) & 1;
    crc = static_cast<uint16_t>((crc << 1) & 0x1ff);
    if (top ^ b)
      crc ^= 0x059;
  };
  for (int i = 2; i < n; i++)
    for (int j = 7; j >= 0; j--)
      bit((block[i] >> j) & 1);
  for (int j = 7; j >= 1; j--) // the serial: the first byte's top 7 bits
    bit((block[0] >> j) & 1);
  uint16_t const mask = rate == 12 ? 0x0f0 : rate == 34 ? 0x1ff : 0x10f;
  uint16_t const got = static_cast<uint16_t>((block[0] & 1) << 8 | block[1]);
  return static_cast<uint16_t>(~crc & 0x1ff) == (got ^ mask);
}
// For tests: a confirmed block's serial and CRC-9 set
inline void crc9_set(uint8_t *block, int n, int rate, int serial)
{
  block[0] = static_cast<uint8_t>(serial << 1);
  block[1] = 0;
  for (int c = 0; c < 512; c++) {
    block[0] = static_cast<uint8_t>((block[0] & 0xfe) | (c >> 8));
    block[1] = static_cast<uint8_t>(c);
    if (crc9_ok(block, n, rate))
      return;
  }
}

// A data header: 12 bytes, its CRC masked
inline bool header_ok(uint8_t const h[12])
{
  return (crc_ccitt(h, 10) ^ 0xcccc) == (h[10] << 8 | h[11]);
}
inline uint32_t crc32_msb(uint8_t const *p, int n, uint32_t init, bool swap)
{
  uint32_t crc = init;
  for (int i = 0; i < n; i++) {
    uint8_t const byte = swap ? p[i ^ 1] : p[i];
    crc ^= static_cast<uint32_t>(byte) << 24;
    for (int b = 0; b < 8; b++)
      crc = crc & 0x80000000u ? (crc << 1) ^ 0x04c11db7u : crc << 1;
  }
  return crc;
}
// The message's CRC-32 (its last 4 bytes, over those before): which of the
// plausible conventions checks, as a short name, or nullptr. The standard
// (B.3.9) takes the octets in pairs, swapped; the others are tried so a
// misreading of it shows in the log rather than as "bad"
inline char const *crc32_check(uint8_t const *p, int n)
{
  if (n < 6 || (n & 1))
    return nullptr; // pairs: the data before the CRC is a whole number of them
  int const m = n - 4;
  uint32_t const le = p[m] | p[m + 1] << 8 | p[m + 2] << 16 | static_cast<uint32_t>(p[m + 3]) << 24;
  uint32_t const be = static_cast<uint32_t>(p[m]) << 24 | p[m + 1] << 16 | p[m + 2] << 8 | p[m + 3];
  static char const *const names[2][8] = {
    {"swap/0", "swap/0/inv", "swap/f", "swap/f/inv", "plain/0", "plain/0/inv", "plain/f", "plain/f/inv"},
    {"swap/0/be", "swap/0/inv/be", "swap/f/be", "swap/f/inv/be",
     "plain/0/be", "plain/0/inv/be", "plain/f/be", "plain/f/inv/be"}};
  for (int v = 0; v < 8; v++) {
    uint32_t c = crc32_msb(p, m, v & 2 ? 0xffffffffu : 0, v < 4);
    if (v & 1)
      c = ~c;
    if (c == le)
      return names[0][v];
    if (c == be)
      return names[1][v];
  }
  return nullptr;
}

// --- a message: header, then its blocks ---
constexpr int kMaxBlocks = 16;          // 16 of 24 bytes: 384, well beyond a position report
constexpr int kMaxReport = 256;

class assembler {
public:
  // A data header's 96 bits, BPTC decoded (as bits): starts a message if it
  // checks. Returns whether it did
  bool header(uint8_t const bits[96])
  {
    uint8_t h[12] = {};
    for (int i = 0; i < 96; i++)
      h[i / 8] = static_cast<uint8_t>(h[i / 8] << 1 | bits[i]);
    if (!header_ok(h)) {
      headers_bad_++;
      return false;
    }
    headers_++;
    int const dpf = h[0] & 0x0f;
    if (dpf == 15) {
      // A proprietary header: the second of a packet's, counted in its
      // blocks to follow (ETSI TS 102 361-1 8.2.1). The first's fields stand
      if (want_ > 0 && !started_ && !proprietary_) {
        proprietary_ = true;
        mfid_ = h[1];
        if (++have_ >= want_)
          finish(true);
      } else
        stray_++;
      return true;
    }
    if (want_ > 0)
      broken_++; // a packet in progress, never completed
    std::memcpy(h_, h, sizeof h_);
    dpf_ = dpf;
    sap_ = h[1] >> 4;
    group_ = (h[0] & 0x80) != 0;
    dst_ = static_cast<uint32_t>(h[2] << 16 | h[3] << 8 | h[4]);
    src_ = static_cast<uint32_t>(h[5] << 16 | h[6] << 8 | h[7]);
    pad_ = 0;
    proprietary_ = false;
    mfid_ = 0;
    switch (dpf_) {
    case 2: // unconfirmed
    case 3: // confirmed
      want_ = h[8] & 0x7f;
      pad_ = (h[0] & 0x10) | (h[1] & 0x0f);
      break;
    case 13: // short data, defined
    case 14: // short data, raw or status/precoded
      want_ = (h[0] & 0x30) | (h[1] & 0x0f);
      break;
    default:
      // Unified data transport (0: a 2-octet CRC and a pad nibble count,
      // not followed here), responses (1) and the rest: the header alone
      want_ = 0;
      break;
    }
    have_ = 0;
    len_ = 0;
    rate_ = 0;
    std::memset(placed_, 0, sizeof placed_);
    started_ = false;
    if (want_ <= 0 || want_ > kMaxBlocks) {
      if (want_ > kMaxBlocks)
        too_long_++;
      want_ = 0;
      report("", nullptr, 0); // the header alone, all there is
    }
    return true;
  }
  // A data block (rate 12 = 1/2, 34 = 3/4, 1 = rate 1), its bytes: 12, 18 or 24
  void block(int rate, uint8_t const *bytes, int n)
  {
    if (want_ <= 0)
      return; // no header before it (missed, or not one we follow)
    if (rate_ != 0 && rate != rate_) {
      want_ = 0; // a block of another message: this one is lost
      broken_++;
      return;
    }
    rate_ = rate;
    started_ = true;
    if (dpf_ == 3) {
      confirmed(bytes, n);
      return;
    }
    if (len_ + n > static_cast<int>(sizeof data_)) {
      want_ = 0;
      return;
    }
    std::memcpy(data_ + len_, bytes, n);
    len_ += n;
    if (++have_ >= want_)
      finish(true);
  }

private:
  // Confirmed data: each block begins with its serial number (7 bits) and a
  // CRC-9, after which its data. Blocks are kept in their places by serial:
  // one its CRC-9 vouches for goes where its serial says (a repeat of one
  // there already dropped), replacing one held there provisionally; one
  // failing it is held provisionally in the first empty place. Full, the
  // packet is reported if its CRC-32 checks, or if every block was vouched
  // for (then bad); else it waits for vouched blocks to replace the others.
  // Serials run from 0: one beyond the packet's blocks breaks it
  void confirmed(uint8_t const *bytes, int n)
  {
    int const blocks = want_ - (proprietary_ ? 1 : 0); // the data blocks
    int const size = n - 2;
    if (blocks <= 0 || blocks * size > static_cast<int>(sizeof data_)) {
      want_ = 0;
      return;
    }
    int at = -1;
    bool const vouched = crc9_ok(bytes, n, rate_);
    if (vouched) {
      crc9_good_++;
      at = bytes[0] >> 1;
      if (at >= blocks) {
        want_ = 0;
        broken_++;
        return;
      }
      if (placed_[at] == 2) {
        repeats_++;
        return;
      }
    } else {
      crc9_bad_++;
      for (int i = 0; i < blocks && at < 0; i++)
        if (placed_[i] == 0)
          at = i;
      if (at < 0)
        return; // nowhere to hold it: every place taken
    }
    std::memcpy(data_ + at * size, bytes + 2, size);
    placed_[at] = vouched ? 2 : 1;
    for (int i = 0; i < blocks; i++)
      if (placed_[i] == 0)
        return;
    len_ = blocks * size;
    have_ = want_; // every block in (the proprietary header too, if any)
    bool all_vouched = true;
    for (int i = 0; i < blocks; i++)
      all_vouched = all_vouched && placed_[i] == 2;
    finish(all_vouched);
  }

public:
  // All its blocks in: the message, its CRC-32 checked. Not final (blocks
  // held provisionally): reported only if the CRC-32 checks
  void finish(bool final_ = true)
  {
    char const *crc = crc32_check(data_, len_);
    if (!crc && !final_)
      return; // wait for blocks vouched for
    want_ = 0;
    messages_++;
    if (crc)
      crc_ok_++;
    // The message itself: without its pad octets and the CRC-32
    int body = len_ - 4 - pad_;
    if (body < 0)
      body = 0;
    report(crc ? crc : "bad", data_, body);
  }
  // A voice call or a terminator: a message in progress is over
  void abandon()
  {
    if (want_ > 0)
      broken_++;
    want_ = 0;
  }

  char const *last_report() const { return report_; }
  uint32_t headers() const { return headers_; }
  uint32_t headers_bad() const { return headers_bad_; }
  uint32_t messages() const { return messages_; }
  uint32_t crc_ok() const { return crc_ok_; }
  uint32_t reports() const { return reports_; } // reports made: one more each time last_report() changes

private:
  // "dpf=2 sap=4 G dst=91 src=3169098 blocks=3/r12 pad=6 crc=swap/0 ipv4 udp 4001>4001 45 00 ..."
  void report(char const *crc, uint8_t const *body, int n)
  {
    char line[kMaxReport + 1];
    int k = std::snprintf(line, sizeof line, "dpf=%d sap=%d %s dst=%u src=%u", dpf_, sap_,
                          group_ ? "G" : "I", dst_, src_);
    if (*crc)
      k += std::snprintf(line + k, sizeof line - k, " blocks=%d/r%d pad=%d crc=%s", have_, rate_, pad_, crc);
    else
      k += std::snprintf(line + k, sizeof line - k, " hdr=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", h_[0], h_[1],
                         h_[2], h_[3], h_[4], h_[5], h_[6], h_[7], h_[8], h_[9]);
    if (proprietary_)
      k += std::snprintf(line + k, sizeof line - k, " mfid=%02x", mfid_); // the proprietary header's maker
    if (body && n >= 28 && (body[0] >> 4) == 4 && body[9] == 17) { // IPv4, UDP
      int const ihl = (body[0] & 0x0f) * 4;
      if (ihl >= 20 && n >= ihl + 8)
        k += std::snprintf(line + k, sizeof line - k, " ipv4 %d.%d.%d.%d>%d.%d.%d.%d udp %d>%d", body[12], body[13],
                           body[14], body[15], body[16], body[17], body[18], body[19],
                           body[ihl] << 8 | body[ihl + 1], body[ihl + 2] << 8 | body[ihl + 3]);
    }
    if (body && n > 0 && k < static_cast<int>(sizeof line) - 4) {
      k += std::snprintf(line + k, sizeof line - k, " ");
      for (int i = 0; i < n && k + 3 < static_cast<int>(sizeof line); i++)
        k += std::snprintf(line + k, sizeof line - k, "%02x", body[i]);
    }
    reports_++;
    std::snprintf(report_, sizeof report_, "%s", line);
    // The last with a good CRC is kept apart, so a later bad one doesn't hide it
    if (*crc && std::strcmp(crc, "bad") != 0)
      std::snprintf(good_, sizeof good_, "%s", line);
  }

public:
  // The last message whose CRC-32 checked, else the last of any kind
  char const *best_report() const { return good_[0] ? good_ : report_; }
  uint32_t broken() const { return broken_; }
  uint32_t repeats() const { return repeats_; }   // confirmed blocks sent again
  uint32_t stray() const { return stray_; }       // proprietary headers with no packet to go with
  uint32_t crc9_good() const { return crc9_good_; }
  uint32_t crc9_bad() const { return crc9_bad_; }

private:
  uint8_t h_[12]{};
  int dpf_ = 0, sap_ = 0, want_ = 0, have_ = 0, len_ = 0, rate_ = 0, pad_ = 0;
  uint8_t placed_[kMaxBlocks]{}; // confirmed blocks: 0 empty, 1 held provisionally, 2 vouched for
  bool proprietary_ = false;
  bool started_ = false; // a block of the packet has come
  int mfid_ = 0;
  bool group_ = false;
  uint32_t dst_ = 0, src_ = 0;
  uint8_t data_[kMaxBlocks * 24]{};
  char report_[kMaxReport + 1] = "";
  char good_[kMaxReport + 1] = "";
  uint32_t headers_ = 0, headers_bad_ = 0, messages_ = 0, crc_ok_ = 0, broken_ = 0, too_long_ = 0;
  uint32_t reports_ = 0, stray_ = 0, repeats_ = 0, crc9_good_ = 0, crc9_bad_ = 0;
};

// Rate 1 data: 192 of the 196 bits (the 2 either side of the middle reserved)
inline void rate1_bytes(uint8_t const *bits, uint8_t out[24])
{
  std::memset(out, 0, 24);
  for (int i = 0; i < 192; i++) {
    int const at = i < 96 ? i : i + 4;
    if (bits[at])
      out[i / 8] |= static_cast<uint8_t>(0x80 >> (i % 8));
  }
}

} // namespace dmr_data
#endif
