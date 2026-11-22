// DMR packet data (dmr_data.h): the rate 3/4 trellis both ways, with symbol
// errors corrected; data headers checked; messages assembled from rate 1/2,
// 3/4 and 1 blocks, confirmed blocks' serials taken out, the CRC-32 checked
// and the IP/UDP header shown; broken messages counted, not reported.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <cstring>
#include <cstdlib>
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
      blk[0] = static_cast<uint8_t>(k << 1); blk[1] = 0x5a; // serial, CRC-9 (not checked)
      std::memcpy(blk + 2, m2 + 16 * k, 16);
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
  // Blocks with no header before them: ignored
  {
    assembler b;
    b.block(12, msg, 12);
    check(b.messages() == 0 && b.last_report()[0] == '\0', "blocks without a header: ignored");
  }
  std::puts(failures ? "DMR data tests FAILED" : "DMR data tests passed");
  return failures != 0;
}
