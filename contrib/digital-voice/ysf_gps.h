// YSF position reports, from V/D mode 2's data channel: frames 6 and 7 of
// a superframe carry 10 bytes each of a short GPS message, assembled when
// the last frame (FN = FT) is in. Layout as YSFGateway (G4KLX) reads it,
// implemented independently here:
//   [1..2]  22 62, a short GPS message (47 64: a long one)
//   [4]     the radio's type
//   [5..10] latitude digits DD MM hh in the low nibbles; the high nibbles
//           0x30 or 0x50: byte 8's N (0x50) / S, byte 10's W (0x50) / E,
//           byte 9's the longitude's range (below)
//   [11]    longitude degrees: with byte 9 high 0x50, 0x76+d (0-9),
//           0x6c+(d-100) (100-109), 0x26+(d-110) (110-179); with 0x30,
//           0x26+(d-10) (10-99)
//   [12]    longitude minutes: 0x58+m (0-9), 0x26+(m-10) (10-59)
//   [13]    hundredths of a minute: 0x1c+h
//   then an end marker 03 and the 8-bit sum of every byte before it.
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_YSF_GPS_H
#define KA9Q_YSF_GPS_H

#include <cstdint>
#include <cstdio>
#include <cstring>

class ysf_gps {
public:
  bool valid() const { return valid_; }
  double latitude() const { return lat_; }
  double longitude() const { return lon_; }
  char const *last_message() const { return hex_; } // the last one assembled, in hex (diagnosis)
  char const *radio() const { return radio_; }      // its radio's model, if known

  void reset() {
    have_ = 0;
    valid_ = false;
    hex_[0] = radio_[0] = '\0';
  }
  // A superframe starts (frame 0): its GPS frames are its own
  void superframe() { have_ = 0; }

  // A V/D mode 2 data channel frame fn of ft (the superframe's last), its
  // 10 bytes; true if it completed a message with a position
  bool frame(int fn, int ft, uint8_t const data[10]) {
    if (fn == 6 || fn == 7) {
      std::memcpy(buf_ + 10 * (fn - 6), data, 10);
      have_ |= 1u << (fn - 6);
    }
    if (fn != ft || fn < 6 || !(have_ & 1))
      return false;
    int const n = fn == 7 && (have_ & 2) ? 20 : 10;
    char *p = hex_;
    for (int i = 0; i < n; i++)
      p += std::snprintf(p, 4, "%s%02x", i ? " " : "", buf_[i]);
    have_ = 0;
    double lat, lon;
    if (!decode(buf_, n, lat, lon))
      return false;
    lat_ = lat;
    lon_ = lon;
    valid_ = true;
    char const *m = model(buf_[4]);
    if (m != nullptr)
      std::snprintf(radio_, sizeof radio_, "%s", m);
    else
      std::snprintf(radio_, sizeof radio_, "0x%02X", buf_[4]);
    return true;
  }

  // One assembled message of n bytes: its position, if it passes
  static bool decode(uint8_t const *b, int n, double& lat, double& lon) {
    // The end marker with the sum of all before it after it: the last that
    // has one (a 03 may also be data)
    bool summed = false;
    for (int i = n - 2; i >= 1 && !summed; i--) {
      if (b[i] != 0x03)
        continue;
      uint8_t sum = 0;
      for (int k = 0; k <= i; k++)
        sum = static_cast<uint8_t>(sum + b[k]);
      summed = sum == b[i + 1];
    }
    if (!summed || n < 14)
      return false;
    if (!((b[1] == 0x22 && b[2] == 0x62) || (b[1] == 0x47 && b[2] == 0x64)))
      return false;
    for (int i = 5; i <= 10; i++)
      if ((b[i] & 0xf0) != 0x50 && (b[i] & 0xf0) != 0x30)
        return false;
    auto pair = [&](int i, int most, unsigned& v) {
      unsigned const t = b[i] & 0x0f, u = b[i + 1] & 0x0f;
      v = 10 * t + u;
      return t <= 9 && u <= 9 && v <= static_cast<unsigned>(most);
    };
    unsigned lat_deg, lat_min, lat_hund;
    if (!pair(5, 89, lat_deg) || !pair(7, 59, lat_min))
      return false;
    // Hundredths: YSFGateway allows a units nibble of 10 ("buggy Yaesu firmware")
    unsigned const t = b[9] & 0x0f, u = b[10] & 0x0f;
    lat_hund = 10 * t + u;
    if (t > 9 || u > 10 || lat_hund > 99)
      return false;
    double const ns = (b[8] & 0xf0) == 0x50 ? 1 : -1;
    double const ew = (b[10] & 0xf0) == 0x50 ? -1 : 1;
    unsigned lon_deg;
    uint8_t const d = b[11];
    if ((b[9] & 0xf0) == 0x50) {
      if (d >= 0x76 && d <= 0x7f) lon_deg = d - 0x76;
      else if (d >= 0x6c && d <= 0x75) lon_deg = 100 + (d - 0x6c);
      else if (d >= 0x26 && d <= 0x6b) lon_deg = 110 + (d - 0x26);
      else return false;
    } else {
      if (d >= 0x26 && d <= 0x7f) lon_deg = 10 + (d - 0x26);
      else return false;
    }
    unsigned lon_min;
    if (b[12] >= 0x58 && b[12] <= 0x61) lon_min = b[12] - 0x58;
    else if (b[12] >= 0x26 && b[12] <= 0x57) lon_min = 10 + (b[12] - 0x26);
    else return false;
    if (b[13] < 0x1c || b[13] > 0x7f)
      return false;
    unsigned const lon_hund = b[13] - 0x1c;
    if (lon_deg > 179 || lon_min > 59 || lon_hund > 99)
      return false;
    lat = ns * (lat_deg + (lat_min + lat_hund / 100.0) / 60);
    lon = ew * (lon_deg + (lon_min + lon_hund / 100.0) / 60);
    return true;
  }

  static char const *model(uint8_t type) {
    switch (type) {
    case 0x20: return "DR-2X";
    case 0x24: return "FT-1D";
    case 0x25: return "FTM-400D";
    case 0x26: return "DR-1X";
    case 0x27: return "FT-991A";
    case 0x28: return "FT-2D";
    case 0x29: return "FTM-100D";
    case 0x30: return "FT-3D";
    case 0x31: return "FTM-300D";
    case 0x33: return "FT-5D";
    default: return nullptr;
    }
  }

private:
  uint8_t buf_[20]{};
  unsigned have_ = 0;  // bit 0: frame 6 in, bit 1: frame 7
  bool valid_ = false;
  double lat_ = 0, lon_ = 0;
  char hex_[64] = "";
  char radio_[12] = "";
};

#endif
