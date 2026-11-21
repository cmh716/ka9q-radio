// D-STAR acquisition must reject isolated syncs in noise or other modes.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../dstar_rx.cpp"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <array>
#include <initializer_list>
#include <string>
#include <vector>

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
    check(std::strcmp(partial.message_, "ABCDEFGHIJKLMNOPQRST") == 0, "a changed block: the last message stays meanwhile");
    send_message(partial, "Second message here ", {0, 2, 3});
    check(std::strcmp(partial.message_, "Second message here") == 0, "and the new one replaces it");

    // Straddling a superframe boundary completes; blocks two superframes
    // apart (perhaps another message's) don't make a hybrid
    dstar_decoder straddle(nullptr, nullptr);
    send_message(straddle, "ABCDEFGHIJKLMNOPQRST", {0, 1});
    straddle.superframes_++;
    send_message(straddle, "ABCDEFGHIJKLMNOPQRST", {2, 3});
    check(std::strcmp(straddle.message_, "ABCDEFGHIJKLMNOPQRST") == 0, "blocks from two consecutive superframes");
    dstar_decoder hybrid(nullptr, nullptr);
    send_message(hybrid, "AAAAABBBBBCCCCCDDDDD", {0});
    hybrid.superframes_ += 2;
    send_message(hybrid, "WWWWWXXXXXYYYYYZZZZZ", {1, 2, 3});
    check(hybrid.message_[0] == '\0', "no hybrid from blocks two superframes apart", hybrid.message_);
    send_message(hybrid, "WWWWWXXXXXYYYYYZZZZZ", {0});
    check(std::strcmp(hybrid.message_, "WWWWWXXXXXYYYYYZZZZZ") == 0, "the new message's own block 0 completes it");
    dstar_decoder junk(nullptr, nullptr);
    send_message(junk, "A\x01" "B\x7f" "C" "               ", {0, 1, 2, 3}); // 5 + 15 spaces
    check(std::strcmp(junk.message_, "A B C") == 0, "unprintable characters as spaces", junk.message_);
    junk.signal_lost();
    check(junk.message_[0] == '\0', "signal loss forgets it");
    // The message is its sender's: hidden while another callsign is on
    dstar_decoder owner(nullptr, nullptr);
    std::strcpy(owner.src_, "M0HPL");
    send_message(owner, "Richard NE England  ", {0, 1, 2, 3});
    dv_stats st;
    owner.get_stats(&st);
    check(std::strcmp(st.message, "Richard NE England") == 0, "reported with its sender", st.message);
    std::strcpy(owner.src_, "G4SPF");
    owner.get_stats(&st);
    check(st.message[0] == '\0', "not with another callsign", st.message);
    std::strcpy(owner.src_, "M0HPL");
    owner.get_stats(&st);
    check(std::strcmp(st.message, "Richard NE England") == 0, "again once its sender is back (a relock)", st.message);
    dstar_decoder early(nullptr, nullptr);
    send_message(early, "Before the callsign ", {0, 1, 2, 3});
    std::strcpy(early.src_, "M0HPL");
    early.adopt_message();
    early.get_stats(&st);
    check(std::strcmp(st.message, "Before the callsign") == 0, "a message before the callsign is its", st.message);

    lock_loss();

    dstar_decoder header_block(nullptr, nullptr);
    slow_block(header_block, 0x55, "KD2TY");
    check(header_block.text_have_ == 0, "a header block is not text");
  }

  // The decoder's real path, bit by bit: late entry (no radio header), the
  // text message, the header repeated in slow data for the callsign, a fade
  // that loses the lock, a relock (the same sender, then another), and the
  // end-of-transmission marker
  static void bits_of(dstar_decoder& d, uint8_t const *b, int n) {
    for (int i = 0; i < n; i++) d.rx_bit(b[i]);
  }
  static void voice(dstar_decoder& d) { zeros(d, kVoiceBits); }
  // A superframe: frame 0 (voice, then the sync) and 20 frames whose slow
  // data are these 6-byte blocks, two frames each (zeros after them)
  static void superframe(dstar_decoder& d, std::vector<std::array<uint8_t, 6>> const& blocks) {
    voice(d);
    sync_bits(d);
    for (int f = 1; f < kSuperframe; f++) {
      voice(d);
      int const b = (f - 1) / 2, half = (f - 1) % 2;
      uint8_t bytes[3] = {0, 0, 0};
      if (b < static_cast<int>(blocks.size()))
        std::memcpy(bytes, blocks[b].data() + 3 * half, 3);
      uint8_t bits[24];
      for (int k = 0; k < 3; k++) {
        uint8_t const x = bytes[k] ^ kScramble[k];
        for (int i = 0; i < 8; i++) bits[8 * k + i] = (x >> i) & 1;
      }
      bits_of(d, bits, 24);
    }
  }
  static std::vector<std::array<uint8_t, 6>> text_blocks(char const *msg) {
    std::vector<std::array<uint8_t, 6>> v;
    for (int n = 0; n < 4; n++) {
      std::array<uint8_t, 6> b{static_cast<uint8_t>(0x40 | n)};
      std::memcpy(b.data() + 1, msg + 5 * n, 5);
      v.push_back(b);
    }
    return v;
  }
  // The 41-byte radio header (MY = call), CRC'd, in nine type 0x5n blocks
  static std::vector<std::array<uint8_t, 6>> header_blocks(char const *call) {
    uint8_t h[41];
    std::memset(h, ' ', sizeof h);
    h[0] = h[1] = h[2] = 0;
    std::memcpy(h + 19, "CQCQCQ  ", 8);
    std::memcpy(h + 27, call, std::strlen(call));
    uint16_t const crc = dstar_crc(h, 39);
    h[39] = crc & 0xff;
    h[40] = crc >> 8;
    std::vector<std::array<uint8_t, 6>> v;
    for (int off = 0; off < 41; off += 5) {
      int const n = std::min(5, 41 - off);
      std::array<uint8_t, 6> b{static_cast<uint8_t>(0x50 | n)};
      std::memcpy(b.data() + 1, h + off, n);
      v.push_back(b);
    }
    return v;
  }
  static std::string reported(dstar_decoder const& d) {
    dv_stats st;
    d.get_stats(&st);
    return st.message;
  }
  static void lock_loss() {
    for (char const *second : {"M0HPL", "G4SPF"}) {
      dstar_decoder d(nullptr, nullptr);
      zeros(d, 72);
      superframe(d, {});                                   // a candidate sync
      superframe(d, text_blocks("Richard NE England  ")); // confirmed: locked
      check(d.locked_, "late entry locks on the second sync");
      superframe(d, header_blocks("M0HPL"));
      check(std::strcmp(d.src_, "M0HPL") == 0, "the callsign from the repeated header", d.src_);
      check(reported(d) == "Richard NE England", "the message, its sender's", reported(d).c_str());
      zeros(d, 4 * kSuperframe * kFrameBits);              // a fade: the lock goes
      check(!d.locked_ && d.src_[0] == '\0', "the fade loses the lock and the callsign");
      check(reported(d).empty(), "no message while the source is unknown", reported(d).c_str());
      superframe(d, {});
      superframe(d, {});
      superframe(d, header_blocks(second));               // relocked; whose is it?
      check(d.locked_ && std::strcmp(d.src_, second) == 0, "relocked, the callsign again", d.src_);
      bool const same = std::strcmp(second, "M0HPL") == 0;
      check(reported(d) == (same ? "Richard NE England" : ""),
            same ? "the same sender's message back after the relock" : "another sender: not the old message",
            reported(d).c_str());
      voice(d);
      bits_of(d, kEot, 48);                                // the end of the transmission
      check(d.message_[0] == '\0' && reported(d).empty(), "the end-of-transmission marker clears it");
    }
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
