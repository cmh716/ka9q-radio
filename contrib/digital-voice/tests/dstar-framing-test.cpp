// D-STAR acquisition must reject isolated syncs in noise or other modes.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../dstar_rx.cpp"
#include <cstdlib>
#include <cstdio>

namespace {
struct dstar_framing_test {
  static void check(bool ok, char const *message) {
    if (!ok) {
      std::fprintf(stderr, "FAIL: %s\n", message);
      std::exit(EXIT_FAILURE);
    }
  }
  static void zeros(dstar_decoder& d, int count) {
    while (count--) d.rx_bit(0);
  }
  static void sync_bits(dstar_decoder& d, bool inverted = false) {
    for (uint8_t b : kFs) d.rx_bit(b ^ inverted);
  }
  static void run() {
    dstar_decoder lone(nullptr, nullptr);
    zeros(lone, 72);
    sync_bits(lone);
    check(lone.syncs_ == 1 && lone.frames_ == 0, "isolated sync produces no audio");
    zeros(lone, 4 * kSuperframe * kFrameBits);
    check(lone.frames_ == 0 && !lone.locked_, "missing confirmation abandons candidate");

    for (bool inverted : {false, true}) {
      dstar_decoder late(nullptr, nullptr);
      zeros(late, 72);
      sync_bits(late, inverted);
      zeros(late, kSuperframe * kFrameBits - 24);
      check(late.frames_ == 0, "late entry waits for confirmation");
      sync_bits(late, inverted);
      check(late.locked_ && late.frames_ == 1, "second correctly spaced sync admits late entry");
      zeros(late, kFrameBits);
      check(late.frames_ == 2, "confirmed call continues producing frames");
      late.signal_lost();
      zeros(late, 72);
      sync_bits(late, inverted);
      check(late.frames_ == 2 && !late.locked_, "signal loss requires fresh confirmation");
    }

    dstar_decoder header(nullptr, nullptr);
    header.header_valid_ = true; // the CRC-checked header acquisition path
    zeros(header, 72);
    sync_bits(header);
    check(header.frames_ == 1, "valid header permits immediate entry");
  }
};
}

int main() {
  dstar_framing_test::run();
  std::puts("D-STAR framing tests passed");
}
