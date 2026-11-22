// DMR packet data (dmr_data.h): the rate 3/4 trellis both ways, with symbol
// errors corrected; data headers checked; messages assembled from rate 1/2,
// 3/4 and 1 blocks, confirmed blocks' serials taken out, the CRC-32 checked
// and the IP/UDP header shown; broken messages counted, not reported.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <initializer_list>
#include "dmr_data.h"

using namespace dmr_data;
static int failures;
static void check(bool ok, char const *what)
{
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  failures += !ok;
}
static unsigned seed = 1;
static uint8_t rnd() { seed = seed * 1103515245 + 12345; return static_cast<uint8_t>(seed >> 16); }

static void to_bits(uint8_t const *bytes, int n, uint8_t *bits)
{
  for (int i = 0; i < 8 * n; i++)
    bits[i] = (bytes[i / 8] >> (7 - i % 8)) & 1;
}
// A data header: dpf, sap, group, dst, src, blocks to follow, pad octets
static void make_header(uint8_t bits[96], int dpf, int sap, bool group, uint32_t dst, uint32_t src, int blocks, int pad)
{
  uint8_t h[12] = {};
  h[0] = static_cast<uint8_t>((group ? 0x80 : 0) | (pad & 0x10) | dpf);
  h[1] = static_cast<uint8_t>(sap << 4 | (pad & 0x0f));
  h[2] = dst >> 16; h[3] = dst >> 8; h[4] = dst;
  h[5] = src >> 16; h[6] = src >> 8; h[7] = src;
  h[8] = static_cast<uint8_t>(0x80 | blocks);
  uint16_t const crc = crc_ccitt(h, 10) ^ 0xcccc;
  h[10] = crc >> 8; h[11] = crc & 0xff;
  to_bits(h, 12, bits);
}
// A message's bytes, padded, with its CRC-32 (the standard's: octets in pairs swapped, little-endian)
static int make_message(uint8_t *out, uint8_t const *body, int n, int room)
{
  std::memset(out, 0, room);
  std::memcpy(out, body, n);
  uint32_t const c = crc32_msb(out, room - 4, 0, true);
  out[room - 4] = c; out[room - 3] = c >> 8; out[room - 2] = c >> 16; out[room - 1] = c >> 24;
  return room - 4 - n; // the pad
}

int main()
{
  // The trellis: clean blocks, both interleave readings, and errors
  bool clean = true, seen = true;
  int fixed = 0;
  for (int t = 0; t < 200; t++) {
    uint8_t in[18], out[18], bits[196];
    for (auto& b : in) b = rnd();
    bool const way = t & 1;
    trellis34_encode(in, bits, way);
    bool swapped = !way;
    int m = trellis34_decode(bits, out, &swapped);
    clean = clean && m == 0 && swapped == way && std::memcmp(in, out, 18) == 0;
    // one bit wrong: always seen (the metric), mostly corrected (hard
    // decisions: a wrong path can be as near as the right one)
    bits[rnd() % 196] ^= 1;
    m = trellis34_decode(bits, out, &swapped);
    seen = seen && m > 0;
    fixed += std::memcmp(in, out, 18) == 0;
  }
  check(clean, "rate 3/4 trellis: 200 clean blocks decode, metric 0, either interleave reading");
  std::printf("     one bit wrong: %d of 200 corrected\n", fixed);
  check(seen && fixed >= 140, "a wrong bit always raises the metric, and is mostly corrected");

  // A UDP/IP message in three rate 1/2 blocks
  uint8_t ip[30] = {0x45, 0, 0, 30, 0, 0, 0, 0, 64, 17, 0, 0, 12, 0, 0, 1, 13, 0, 0, 2, 0x0f, 0xa1, 0x0f, 0xa1, 0, 10, 0, 0, 0xab, 0xcd};
  uint8_t msg[36];
  int pad = make_message(msg, ip, sizeof ip, sizeof msg);
  assembler as;
  uint8_t hb[96];
  make_header(hb, 2, 4, true, 91, 3169098, 3, pad);
  check(as.header(hb), "a data header checks");
  for (int k = 0; k < 3; k++)
    as.block(12, msg + 12 * k, 12);
  std::printf("     %s\n", as.last_report());
  check(as.messages() == 1 && as.crc_ok() == 1, "three rate 1/2 blocks: one message, its CRC-32 good");
  check(std::strstr(as.last_report(), "dpf=2 sap=4 G dst=91 src=3169098 blocks=3/r12 pad=2 crc=swap/0") != nullptr,
        "its header's fields, blocks, pad and how the CRC checked");
  check(std::strstr(as.last_report(), "ipv4 12.0.0.1>13.0.0.2 udp 4001>4001") != nullptr, "the IP/UDP header shown");
  check(std::strstr(as.last_report(), "abcd") != nullptr && std::strstr(as.last_report(), "abcd00") == nullptr,
        "the bytes in hex, without the pad or CRC");

  // A bad CRC: reported so, the good one kept as best
  make_header(hb, 2, 4, true, 91, 3169098, 3, pad);
  as.header(hb);
  msg[5] ^= 1;
  for (int k = 0; k < 3; k++)
    as.block(12, msg + 12 * k, 12);
  msg[5] ^= 1;
  check(std::strstr(as.last_report(), "crc=bad") != nullptr && as.crc_ok() == 1, "a damaged message: crc=bad");
  check(std::strstr(as.best_report(), "crc=swap/0") != nullptr, "the last good one still the best report");

  // A header failing its CRC is refused
  make_header(hb, 2, 4, true, 91, 3169098, 3, pad);
  hb[40] ^= 1;
  check(!as.header(hb) && as.headers_bad() == 1, "a damaged header: refused");

  // Confirmed (dpf 3), rate 3/4: each block's serial and CRC-9 (2 bytes) taken out
  {
    uint8_t m2[48];
    int const p2 = make_message(m2, ip, sizeof ip, sizeof m2);
    assembler c;
    make_header(hb, 3, 4, false, 3100001, 3169098, 3, p2);
    c.header(hb);
    for (int k = 0; k < 3; k++) {
      uint8_t blk[18], bits[196], out[18];
      std::memcpy(blk + 2, m2 + 16 * k, 16);
      crc9_set(blk, 18, 34, k); // serial, CRC-9
      trellis34_encode(blk, bits);
      trellis34_decode(bits, out);
      c.block(34, out, 18);
    }
    std::printf("     %s\n", c.last_report());
    check(c.crc_ok() == 1 && std::strstr(c.last_report(), "dpf=3 sap=4 I dst=3100001 src=3169098 blocks=3/r34 pad=14 crc=swap/0 ipv4") != nullptr,
          "confirmed rate 3/4: serials out, CRC-32 good");
  }

  // Rate 1: 192 of the 196 bits, the 4 middle ones skipped
  {
    uint8_t in[24], bits[196] = {}, out[24];
    for (auto& b : in) b = rnd();
    uint8_t flat[192];
    to_bits(in, 24, flat);
    for (int i = 0; i < 192; i++)
      bits[i < 96 ? i : i + 4] = flat[i];
    bits[96] = bits[97] = bits[98] = bits[99] = 1;
    rate1_bytes(bits, out);
    check(std::memcmp(in, out, 24) == 0, "rate 1 blocks: 24 bytes");
  }

  // A message cut short by another rate's block, or a terminator: broken, no report
  {
    assembler b;
    make_header(hb, 2, 4, true, 91, 1, 3, pad);
    b.header(hb);
    b.block(12, msg, 12);
    b.block(34, msg, 18);
    make_header(hb, 2, 4, true, 91, 1, 3, pad);
    b.header(hb);
    b.block(12, msg, 12);
    b.abandon();
    b.block(12, msg + 12, 12);
    b.block(12, msg + 24, 12);
    check(b.messages() == 0 && b.broken() == 2, "cut short twice: two broken, no message");
  }
  // A proprietary second header (DPF 15): counted in blocks to follow, the first header's fields kept
  {
    assembler b;
    make_header(hb, 2, 4, true, 91, 3169098, 4, pad); // the proprietary header and three blocks
    b.header(hb);
    uint8_t ph[12] = {0x4f, 0x10, 1, 2, 3, 4, 5, 6, 7, 8}; // SAP 4, DPF 15; MFID 0x10
    uint16_t const crc = crc_ccitt(ph, 10) ^ 0xcccc;
    ph[10] = crc >> 8; ph[11] = crc & 0xff;
    uint8_t pb[96];
    to_bits(ph, 12, pb);
    b.header(pb);
    for (int k = 0; k < 3; k++)
      b.block(12, msg + 12 * k, 12);
    std::printf("     %s\n", b.last_report());
    check(b.messages() == 1 && b.crc_ok() == 1 && std::strstr(b.last_report(), "dpf=2 sap=4 G dst=91 src=3169098") != nullptr
          && std::strstr(b.last_report(), "mfid=10") != nullptr,
          "a proprietary second header: the packet still assembled, the first header's fields, its MFID shown");
    b.header(pb);
    check(b.stray() == 1 && b.messages() == 1, "a proprietary header with no packet: counted stray");
  }
  // Unified data transport: its header alone reported, its blocks not taken as a message
  {
    assembler b;
    make_header(hb, 0, 0, false, 5, 6, 2, 0);
    b.header(hb);
    b.block(12, msg, 12);
    b.block(12, msg + 12, 12);
    check(b.messages() == 0 && std::strstr(b.last_report(), "dpf=0") != nullptr && std::strstr(b.last_report(), "hdr=") != nullptr,
          "UDT: the header reported alone, its blocks not taken for a message");
  }
  // Confirmed blocks: a repeated serial dropped, a missed one breaks the packet
  {
    uint8_t m2[48];
    int const p2 = make_message(m2, ip, sizeof ip, sizeof m2);
    uint8_t blocks[3][18];
    for (int k = 0; k < 3; k++) {
      std::memcpy(blocks[k] + 2, m2 + 16 * k, 16);
      crc9_set(blocks[k], 18, 34, k);
    }
    assembler b;
    make_header(hb, 3, 4, false, 1, 2, 3, p2);
    b.header(hb);
    b.block(34, blocks[0], 18);
    b.block(34, blocks[1], 18);
    b.block(34, blocks[1], 18); // sent again
    b.block(34, blocks[2], 18);
    check(b.messages() == 1 && b.crc_ok() == 1 && b.repeats() == 1, "confirmed: a block sent twice taken once");
    make_header(hb, 3, 4, false, 1, 2, 3, p2);
    b.header(hb);
    b.block(34, blocks[0], 18);
    b.block(34, blocks[2], 18); // the second missed
    b.block(34, blocks[2], 18);
    check(b.messages() == 1 && b.broken() == 1, "confirmed: a block missed: broken, not a bad CRC");
    // A serial damaged (its CRC-9 fails): not trusted, the block taken in turn
    make_header(hb, 3, 4, false, 1, 2, 3, p2);
    b.header(hb);
    b.block(34, blocks[0], 18);
    uint8_t hurt[18];
    std::memcpy(hurt, blocks[1], 18);
    hurt[0] ^= 0x04; // serial 1 reads as 3
    b.block(34, hurt, 18);
    b.block(34, blocks[2], 18);
    check(b.messages() == 2 && b.crc_ok() == 2 && b.broken() == 1 && b.crc9_bad() == 1,
          "confirmed: a damaged serial (CRC-9 fails) not taken for a gap; the packet still assembled");
    // the CRC-9 itself: each rate's mask, one bit wrong anywhere fails it
    bool crc9 = true;
    for (int rate : {12, 34, 1}) {
      int const n = rate == 12 ? 12 : rate == 34 ? 18 : 24;
      uint8_t blk[24];
      for (int i = 0; i < n; i++) blk[i] = rnd();
      crc9_set(blk, n, rate, 5);
      crc9 = crc9 && crc9_ok(blk, n, rate) && (blk[0] >> 1) == 5;
      for (int bit = 0; bit < 8 * n; bit++) {
        blk[bit / 8] ^= static_cast<uint8_t>(0x80 >> (bit % 8));
        crc9 = crc9 && !crc9_ok(blk, n, rate);
        blk[bit / 8] ^= static_cast<uint8_t>(0x80 >> (bit % 8));
      }
    }
    check(crc9, "CRC-9: set for each rate, and any one bit wrong fails it");
  }
  // Blocks with no header before them: ignored
  {
    assembler b;
    b.block(12, msg, 12);
    check(b.messages() == 0 && b.last_report()[0] == '\0', "blocks without a header: ignored");
  }
  std::puts(failures ? "DMR data tests FAILED" : "DMR data tests passed");
  return failures != 0;
}
