// DMR talker alias: assembled from the embedded LC's talker alias header and
// blocks (FLCO 4-7, ETSI TS 102 361-2 7.2.18 and 7.2.19), which arrive one per
// voice superframe, in any order, repeated through the call.
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DMR_ALIAS_H
#define KA9Q_DMR_ALIAS_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

class dmr_talker_alias {
public:
  void reset() { have_ = 0; text_[0] = '\0'; done_ = false; }
  bool complete() const { return done_ && text_[0] != '\0'; }
  char const *text() const { return text_; }

  // part 0: the header, 1-3: blocks 1-3; payload: the LC's 7 bytes after the FID.
  // The header's first byte gives the format (7-bit, ISO 8859-1, UTF-8,
  // UTF-16BE) and the length in characters; the text follows it, 7-bit
  // characters from its last bit, the others from the next byte, and goes on
  // through the blocks, 56 bits each. It is complete when the header and the
  // blocks its length needs are in
  void add(int part, uint8_t const *payload) {
    if (part < 0 || part > 3)
      return;
    if ((have_ & (1u << part)) && std::memcmp(ta_[part], payload, 7) != 0)
      reset(); // a new alias (or this one changed): start again
    std::memcpy(ta_[part], payload, 7);
    have_ |= 1u << part;
    if (!(have_ & 1) || done_)
      return;
    int const format = ta_[0][0] >> 6;
    int const length = (ta_[0][0] >> 1) & 0x1f;
    int const width = format == 0 ? 7 : format == 3 ? 16 : 8;
    int const first = format == 0 ? 7 : 8;
    int const need = length * width; // bits
    int const blocks = need <= 56 - first ? 0 : (need - (56 - first) + 55) / 56;
    if (length == 0 || blocks > 3)
      return;
    for (int b = 1; b <= blocks; b++)
      if (!(have_ & (1u << b)))
        return; // not all in yet
    std::string text;
    for (int ch = 0; ch < length; ch++) {
      unsigned v = 0;
      for (int k = 0; k < width; k++)
        v = (v << 1) | bit(first + ch * width + k);
      if (format == 2) { // UTF-8: bytes as they are, checked below
        text += static_cast<char>(v);
        continue;
      }
      if (v == 0)
        break;
      if (v < 0x20 || v == 0x7f)
        v = '?';
      append_utf8(text, v); // 7-bit, ISO 8859-1 and UTF-16 (the basic plane)
    }
    if (format == 2)
      text = valid_utf8(text);
    while (!text.empty() && text.back() == ' ')
      text.pop_back();
    std::snprintf(text_, sizeof text_, "%s", text.c_str());
    done_ = true;
  }

private:
  unsigned bit(int at) const { // bit at of the header and blocks, in order
    int const part = at / 56, in = at % 56;
    return (ta_[part][in / 8] >> (7 - in % 8)) & 1;
  }
  static void append_utf8(std::string& s, unsigned v) {
    if (v < 0x80) {
      s += static_cast<char>(v);
    } else if (v < 0x800) {
      s += static_cast<char>(0xc0 | (v >> 6));
      s += static_cast<char>(0x80 | (v & 0x3f));
    } else if (v >= 0xd800 && v < 0xe000) {
      s += '?'; // half of a pair, beyond the basic plane
    } else {
      s += static_cast<char>(0xe0 | (v >> 12));
      s += static_cast<char>(0x80 | ((v >> 6) & 0x3f));
      s += static_cast<char>(0x80 | (v & 0x3f));
    }
  }
  // UTF-8 as sent, with control characters and malformed sequences as '?',
  // and a sequence the buffer can't hold whole left out
  static std::string valid_utf8(std::string const& in) {
    std::string out;
    for (size_t i = 0; i < in.size();) {
      unsigned char const b = static_cast<unsigned char>(in[i]);
      if (b == 0)
        break;
      size_t const n = b < 0x80 ? 1 : (b & 0xe0) == 0xc0 ? 2 : (b & 0xf0) == 0xe0 ? 3 : (b & 0xf8) == 0xf0 ? 4 : 0;
      bool ok = n != 0 && i + n <= in.size();
      for (size_t k = 1; ok && k < n; k++)
        ok = (static_cast<unsigned char>(in[i + k]) & 0xc0) == 0x80;
      if (!ok || (n == 1 && (b < 0x20 || b == 0x7f))) {
        out += '?';
        i++;
        continue;
      }
      if (out.size() + n >= sizeof text_)
        break;
      out.append(in, i, n);
      i += n;
    }
    return out;
  }

  uint8_t ta_[4][7]{};
  unsigned have_ = 0;  // bit n: part n received
  bool done_ = false;
  char text_[64] = "";
};

#endif
