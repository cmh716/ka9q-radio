// D-STAR acquisition must reject isolated syncs in noise or other modes.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../dstar_rx.cpp"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace {
struct dstar_framing_test {
  static void check(bool ok, char const *message, char const *got = nullptr) {
    if (!ok) {
      std::fprintf(stderr, "FAIL: %s%s%s\n", message, got ? ": got " : "", got ? got : "");
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

    // Slow data text message: four type 0x4n blocks of 5 characters, each
    // block two frames of 3 scrambled bytes sent LSB first
    dstar_decoder text(nullptr, nullptr);
    send_message(text, "James  Buffalo NY   ", {0, 1, 2, 3});
    check(std::strcmp(text.message_, "James  Buffalo NY") == 0, "text message, trailing spaces dropped");
    dstar_decoder order(nullptr, nullptr);
    send_message(order, "ABCDEFGHIJKLMNOPQRST", {2, 0, 3, 1});
    check(std::strcmp(order.message_, "ABCDEFGHIJKLMNOPQRST") == 0, "blocks in any order");
    dstar_decoder partial(nullptr, nullptr);
    send_message(partial, "ABCDEFGHIJKLMNOPQRST", {0, 1, 3});
    check(partial.message_[0] == '\0', "three of four blocks: no message yet");
    send_message(partial, "ABCDEFGHIJKLMNOPQRST", {2});
    check(std::strcmp(partial.message_, "ABCDEFGHIJKLMNOPQRST") == 0, "the fourth completes it");
    send_message(partial, "Second message here ", {1});
    check(partial.message_[0] == '\0', "a changed block starts a new message");
    send_message(partial, "Second message here ", {0, 2, 3});
    check(std::strcmp(partial.message_, "Second message here") == 0, "and the new one completes");
    dstar_decoder junk(nullptr, nullptr);
    send_message(junk, "A\x01" "B\x7f" "C" "               ", {0, 1, 2, 3}); // 5 + 15 spaces
    check(std::strcmp(junk.message_, "A B C") == 0, "unprintable characters as spaces", junk.message_);
    junk.signal_lost();
    check(junk.message_[0] == '\0', "signal loss forgets it");
    dstar_decoder header_block(nullptr, nullptr);
    slow_block(header_block, 0x55, "KD2TY");
    check(header_block.text_have_ == 0, "a header block is not text");
  }

  // One 6-byte slow data block, as the two frames that carry it
  static void slow_block(dstar_decoder& d, uint8_t first, char const *five) {
    uint8_t block[6] = {first};
    std::memcpy(block + 1, five, 5);
    for (int f = 0; f < 2; f++) {
      uint8_t bits[24];
      for (int k = 0; k < 3; k++) {
        uint8_t const b = block[3 * f + k] ^ kScramble[k];
        for (int i = 0; i < 8; i++)
          bits[8 * k + i] = (b >> i) & 1;
      }
      d.slow_data(bits);
    }
  }
  static void send_message(dstar_decoder& d, char const *msg, std::initializer_list<int> blocks) {
    check(std::strlen(msg) == 20, "test message is 20 characters");
    for (int n : blocks)
      slow_block(d, static_cast<uint8_t>(0x40 | n), msg + 5 * n);
  }
};
}

int main() {
  dstar_framing_test::run();
  std::puts("D-STAR framing tests passed");
}
