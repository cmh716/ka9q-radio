// YSF position reports (ysf_gps.h): messages built here from the layout,
// fed as V/D mode 2 frames 6 and 7.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <cstdio>
#include <cstring>
#include "ysf_gps.h"

static int Failures;
static void check(bool ok, char const *what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  Failures += !ok;
}

// The 20-byte short GPS message for a position (degrees, minutes, hundredths)
struct msg { uint8_t b[20]; };
static msg encode(int lat_d, int lat_m, int lat_h, bool north, int lon_d, int lon_m, int lon_h, bool west, uint8_t radio = 0x30) {
  msg m{};
  m.b[0] = 0x00; m.b[1] = 0x22; m.b[2] = 0x62; m.b[3] = 0x00; m.b[4] = radio;
  int const digits[6] = {lat_d / 10, lat_d % 10, lat_m / 10, lat_m % 10, lat_h / 10, lat_h % 10};
  for (int i = 0; i < 6; i++)
    m.b[5 + i] = static_cast<uint8_t>(0x30 | digits[i]);
  if (north) m.b[8] = static_cast<uint8_t>(0x50 | (m.b[8] & 0x0f));
  if (west) m.b[10] = static_cast<uint8_t>(0x50 | (m.b[10] & 0x0f));
  bool const high = lon_d < 10 || lon_d >= 100; // byte 9's longitude range flag
  if (high) m.b[9] = static_cast<uint8_t>(0x50 | (m.b[9] & 0x0f));
  m.b[11] = static_cast<uint8_t>(lon_d < 10 ? 0x76 + lon_d : lon_d < 100 ? 0x26 + (lon_d - 10)
                                 : lon_d < 110 ? 0x6c + (lon_d - 100) : 0x26 + (lon_d - 110));
  m.b[12] = static_cast<uint8_t>(lon_m < 10 ? 0x58 + lon_m : 0x26 + (lon_m - 10));
  m.b[13] = static_cast<uint8_t>(0x1c + lon_h);
  m.b[14] = 0x20; m.b[15] = 0x1c; m.b[16] = 0x5b; m.b[17] = 0x2f; // speed, course, symbol
  m.b[18] = 0x03;
  uint8_t sum = 0;
  for (int i = 0; i <= 18; i++) sum = static_cast<uint8_t>(sum + m.b[i]);
  m.b[19] = sum;
  return m;
}
static bool fed(msg const& m, double lat, double lon, ysf_gps* keep = nullptr) {
  ysf_gps local;
  ysf_gps& g = keep ? *keep : local;
  g.superframe();
  g.frame(6, 7, m.b, 4800);
  bool const got = g.frame(7, 7, m.b + 10, 4800 + 480);
  return got && std::fabs(g.latitude() - lat) < 1e-6 && std::fabs(g.longitude() - lon) < 1e-6;
}

// A message the decoder must refuse: the last frame doesn't complete a
// position, and a fresh decoder is left with none
static bool rejected(msg const& m) {
  ysf_gps g;
  g.superframe();
  g.frame(6, 7, m.b, 4800);
  bool const got = g.frame(7, 7, m.b + 10, 4800 + 480);
  return !got && !g.valid();
}

int main() {
  check(fed(encode(42, 50, 12, true, 78, 36, 45, true), 42 + 50.12 / 60, -(78 + 36.45 / 60)), "N and W, longitude 10-99");
  check(fed(encode(33, 52, 7, false, 151, 12, 50, false), -(33 + 52.07 / 60), 151 + 12.50 / 60), "S and E, longitude 110-179");
  check(fed(encode(51, 28, 40, true, 0, 5, 0, true), 51 + 28.40 / 60, -(0 + 5.0 / 60)), "longitude 0-9");
  check(fed(encode(1, 17, 0, false, 103, 51, 99, false), -(1 + 17.0 / 60), 103 + 51.99 / 60), "longitude 100-109");
  check(fed(encode(89, 59, 99, true, 179, 59, 99, true), 89 + 59.99 / 60, -(179 + 59.99 / 60)), "the extremes");
  {
    // A live message, as received (M9PZL's FT-991A, 2026-11-22): 52.06683 N, 1.14217 E
    msg live{{0x08, 0x22, 0x62, 0x5f, 0x27, 0x55, 0x32, 0x50, 0x54, 0x50, 0x31, 0x77, 0x60, 0x51, 0x6c, 0x20, 0x1c, 0x20, 0x03, 0xb1}};
    ysf_gps g;
    check(fed(live, 52 + 4.01 / 60, 1 + 8.53 / 60, &g) && std::strcmp(g.radio(), "FT-991A") == 0,
          "a live message (M9PZL, FT-991A): 52.06683 N, 1.14217 E");
    // and the no-position form many radios send (22 61): refused, no position
    msg none{{0x01, 0x22, 0x61, 0x5f, 0x2b, 0x03, 0x11}};
    ysf_gps h;
    h.superframe();
    check(!h.frame(6, 6, none.b, 4800) && !h.valid(), "a live no-position message (22 61): no position");
  }
  {
    ysf_gps g;
    fed(encode(42, 50, 12, true, 78, 36, 45, true), 0, 0, &g);
    check(std::strcmp(g.radio(), "FT-3D") == 0, "the radio's model");
    check(std::strncmp(g.last_message(), "00 22 62 00 30", 14) == 0, "and the message in hex, for diagnosis");
  }
  msg bad = encode(42, 50, 12, true, 78, 36, 45, true);
  bad.b[19] ^= 1;
  check(rejected(bad), "a wrong sum");
  msg sig = encode(42, 50, 12, true, 78, 36, 45, true);
  sig.b[2] = 0x63;
  sig.b[19] = static_cast<uint8_t>(sig.b[19] + 1);
  check(rejected(sig), "not a GPS message (signature)");
  {
    ysf_gps g;
    msg m = encode(42, 50, 12, true, 78, 36, 45, true);
    g.superframe();
    check(!g.frame(7, 7, m.b + 10, 480), "frame 7 without 6: nothing");
    g.superframe();
    g.frame(6, 7, m.b, 4800);
    g.superframe(); // a new superframe: frame 6 is the last one's
    check(!g.frame(7, 7, m.b + 10, 5280), "frame 6 of the last superframe isn't joined to this one's 7");
    // Frames missed between them, frame 0 too: a superframe (8 frames) later
    g.frame(6, 7, m.b, 4800);
    check(!g.frame(7, 7, m.b + 10, 4800 + 9 * 480), "frame 7 a superframe after frame 6 (frames missed): not joined");
    g.frame(6, 7, m.b, 4800);
    check(g.frame(7, 7, m.b + 10, 4800 + 483), "frame 7 just after 6, with a timing slip: joined");
  }
  {
    // The end marker must follow the position, so that the sum covers it
    msg early = encode(42, 50, 12, true, 78, 36, 45, true);
    early.b[3] = 0x03;
    early.b[4] = static_cast<uint8_t>(early.b[0] + early.b[1] + early.b[2] + early.b[3]);
    early.b[18] = 0x00; // no marker after the position
    check(rejected(early), "a summed end marker before the position: refused");
  }
  {
    ysf_gps g;
    fed(encode(42, 50, 12, true, 78, 36, 45, true), 0, 0, &g);
    check(g.valid(), "a position");
    g.reset();
    check(!g.valid() && g.radio()[0] == '\0', "reset forgets it");
  }
  std::printf("%s\n", Failures ? "YSF GPS tests FAILED" : "YSF GPS tests passed");
  return Failures != 0;
}
