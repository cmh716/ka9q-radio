// D-STAR position reports, from the slow data's GPS blocks (type 0x3n,
// 1-5 ASCII characters each): lines of NMEA ($GPRMC, $GPGGA; any talker)
// or D-PRS ($$CRCxxxx,CALL>DEST,DSTAR*:!DDMM.mmN/DDDMM.mmW>...), as a radio's
// GPS TX mode selects. Each line must pass its checksum.
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef KA9Q_DSTAR_GPS_H
#define KA9Q_DSTAR_GPS_H

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

class dstar_gps {
public:
  bool valid() const { return valid_; }
  double latitude() const { return lat_; }   // degrees north
  double longitude() const { return lon_; }  // degrees east
  // For diagnosis: the exact report that last passed its checksum, or the
  // last line if none in it did
  char const *last_line() const { return last_.c_str(); }

  void reset() {
    valid_ = false;
    line_.clear();
  }

  // True if a line completed here gave a position
  bool feed(uint8_t const *data, int n) {
    bool got = false;
    for (int i = 0; i < n; i++) {
      char const c = static_cast<char>(data[i]);
      if (c == '\r' || c == '\n') {
        if (!line_.empty())
          got |= line(line_);
        line_.clear();
      } else if (c == '$' && !line_.empty() && line_ != "$" && line_.compare(0, 2, "$$") != 0) {
        line_ = "$"; // a new NMEA sentence: the last lost its end (a D-PRS packet may hold a '$')
      } else {
        if (line_.size() >= kMaxLine)
          got |= overflow();
        line_ += c;
      }
    }
    return got;
  }

  // One complete line; true if it gave a position. A report that lost its
  // end runs into the next, and a report's comment may hold what looks like
  // the start of one (or even a whole checksummed sentence), so each start
  // of a report ($$CRC, or an NMEA sentence's "$ttsss,") is tried to each
  // later start and to the line's end, the checksum deciding, the shortest
  // span that passes first. A start inside a report that passed is part of
  // it, not a report. Of the rest the latest that passed is taken. Only the
  // last kMaxStarts starts are tried, so that noise can't make it slow
  bool line(std::string const& s) {
    last_ = s;
    size_t starts[kMaxStarts];
    size_t n = 0;
    for (size_t at = s.empty() || s[0] != '$' ? next_start(s, 0) : 0; at != std::string::npos; at = next_start(s, at + 1)) {
      if (n == kMaxStarts) {
        std::memmove(starts, starts + 1, (kMaxStarts - 1) * sizeof starts[0]);
        n--;
      }
      starts[n++] = at;
    }
    struct { bool ok; size_t end; double lat, lon; } found[kMaxStarts] = {};
    size_t covered_to = 0; // the end of the last report that passed
    size_t best = kMaxStarts;
    for (size_t i = 0; i < n; i++) {
      if (starts[i] < covered_to)
        continue; // inside a report
      for (size_t j = i + 1; j <= n && !found[i].ok; j++) {
        size_t const end = j < n ? starts[j] : s.size();
        if (report(s.substr(starts[i], end - starts[i]), found[i].lat, found[i].lon)) {
          found[i].ok = true;
          found[i].end = end;
        }
      }
      if (found[i].ok) {
        covered_to = found[i].end;
        best = i;
      }
    }
    if (best == kMaxStarts)
      return false;
    last_ = s.substr(starts[best], found[best].end - starts[best]); // what passed, exactly
    lat_ = found[best].lat;
    lon_ = found[best].lon;
    valid_ = true;
    return true;
  }

  static constexpr size_t kMaxLine = 1024;   // a few reports run together
  static constexpr size_t kMaxReport = 256;  // longer than any one report
  static constexpr size_t kMaxStarts = 16;

  // The line is full without an end: reports have run together. It is tried
  // as a line, and the collection goes on with its last kMaxReport
  // characters from their first '$': a report being received started there
  // or later, so it is kept whole, a prefix just begun included
  bool overflow() {
    bool const got = line(line_);
    line_.erase(0, line_.size() - kMaxReport);
    size_t const dollar = line_.find('$');
    if (dollar == std::string::npos)
      line_.clear(); // noise
    else
      line_.erase(0, dollar);
    return got;
  }

  static size_t next_start(std::string const& s, size_t from) {
    for (size_t i = s.find('$', from); i != std::string::npos; i = s.find('$', i + 1)) {
      if (s.compare(i, 5, "$$CRC") == 0)
        return i;
      if (i + 6 < s.size() && s[i + 6] == ',') {
        bool upper = true;
        for (size_t k = i + 1; k < i + 6; k++)
          upper = upper && std::isupper(static_cast<unsigned char>(s[k]));
        if (upper)
          return i;
      }
    }
    return std::string::npos;
  }

  // One report, the whole of s: its position, if it passes
  bool report(std::string const& s, double& lat, double& lon) {
    bool ok = false;
    // NMEA says whether it has a fix; D-PRS doesn't, and a radio without one
    // may send zeros: there 0, 0 isn't taken
    if (s.compare(0, 5, "$$CRC") == 0)
      ok = dprs(s, lat, lon) && !(lat == 0 && lon == 0);
    else if (s.size() > 6 && s[0] == '$')
      ok = nmea(s, lat, lon);
    return ok && std::fabs(lat) <= 90 && std::fabs(lon) <= 180;
  }

  // The D-STAR CRC (CRC-CCITT reflected, 0x8408, init 0xffff, inverted)
  static uint16_t crc(char const *p, size_t n) {
    uint16_t c = 0xffff;
    for (size_t i = 0; i < n; i++) {
      c ^= static_cast<uint8_t>(p[i]);
      for (int b = 0; b < 8; b++)
        c = (c & 1) ? (c >> 1) ^ 0x8408 : c >> 1;
    }
    return static_cast<uint16_t>(~c);
  }

private:
  // D-PRS: "$$CRC" 4 hex digits "," then an APRS packet. The CRC covers the
  // packet and the line's CR, the digits in the usual order (checked on live
  // reports, KJ4VO's "$$CRCBBB2,KJ4VO>API52,DSTAR*:/211437z...")
  static bool dprs(std::string const& s, double& lat, double& lon) {
    if (s.size() < 11 || s[9] != ',')
      return false;
    std::string const digits = s.substr(5, 4);
    char *end = nullptr;
    unsigned long const want = std::strtoul(digits.c_str(), &end, 16);
    if (end != digits.c_str() + 4)
      return false;
    std::string const packet = s.substr(10);
    std::string const with_cr = packet + "\r";
    if (want != crc(with_cr.data(), with_cr.size()))
      return false;
    size_t const colon = packet.find(':');
    if (colon == std::string::npos)
      return false;
    return aprs_position(packet.substr(colon + 1), lat, lon);
  }

  // An APRS position report's information field: '!' or '=' then the
  // position, or '/' or '@' then a 7-character time and the position;
  // uncompressed (DDMM.mmN/DDDMM.mmW, spaces for ambiguity) or compressed
  static bool aprs_position(std::string const& info, double& lat, double& lon) {
    if (info.empty())
      return false;
    size_t at;
    if (info[0] == '!' || info[0] == '=')
      at = 1;
    else if (info[0] == '/' || info[0] == '@')
      at = 8;
    else
      return false;
    if (info.size() < at + 13)
      return false;
    std::string p = info.substr(at);
    if (std::isdigit(static_cast<unsigned char>(p[0])) || p[0] == ' ') {
      if (p.size() < 19)
        return false;
      for (int i : {0, 1, 2, 3, 5, 6, 9, 10, 11, 12, 13, 15, 16})
        if (p[i] == ' ')
          p[i] = '0'; // position ambiguity
      if (p[4] != '.' || p[14] != '.' || (p[7] != 'N' && p[7] != 'S') || (p[17] != 'E' && p[17] != 'W'))
        return false;
      for (int i : {0, 1, 2, 3, 5, 6, 9, 10, 11, 12, 13, 15, 16})
        if (!std::isdigit(static_cast<unsigned char>(p[i])))
          return false;
      lat = std::atoi(p.substr(0, 2).c_str()) + std::atof(p.substr(2, 5).c_str()) / 60;
      lon = std::atoi(p.substr(9, 3).c_str()) + std::atof(p.substr(12, 5).c_str()) / 60;
      if (p[7] == 'S') lat = -lat;
      if (p[17] == 'W') lon = -lon;
      return true;
    }
    // Compressed: symbol table, 4 base-91 latitude, 4 longitude, ...
    if (p.size() < 13)
      return false;
    auto b91 = [&](int from, double& v) {
      v = 0;
      for (int i = from; i < from + 4; i++) {
        int const d = static_cast<unsigned char>(p[i]) - 33;
        if (d < 0 || d > 90)
          return false;
        v = v * 91 + d;
      }
      return true;
    };
    double y, x;
    if (!b91(1, y) || !b91(5, x))
      return false;
    lat = 90 - y / 380926.0;
    lon = -180 + x / 190463.0;
    return true;
  }

  // NMEA RMC (status A) or GGA (fix quality > 0), checksum required
  static bool nmea(std::string const& s, double& lat, double& lon) {
    size_t const star = s.rfind('*');
    if (star == std::string::npos || star + 3 > s.size())
      return false;
    unsigned sum = 0;
    for (size_t i = 1; i < star; i++)
      sum ^= static_cast<uint8_t>(s[i]);
    std::string const digits = s.substr(star + 1, 2);
    char *end = nullptr;
    unsigned long const want = std::strtoul(digits.c_str(), &end, 16);
    if (end != digits.c_str() + 2 || want != sum)
      return false;
    std::string f[16];
    int n = 0;
    for (size_t i = 1, start = 1; i <= star && n < 16; i++)
      if (i == star || s[i] == ',') {
        f[n++] = s.substr(start, i - start);
        start = i + 1;
      }
    if (f[0].size() != 5)
      return false;
    std::string const type = f[0].substr(2);
    int la; // index of the latitude field
    if (type == "RMC") {
      if (n < 7 || f[2] != "A")
        return false;
      la = 3;
    } else if (type == "GGA") {
      if (n < 7 || f[6].empty() || f[6] == "0")
        return false;
      la = 2;
    } else {
      return false;
    }
    return coordinate(f[la], f[la + 1], 2, lat) && coordinate(f[la + 2], f[la + 3], 3, lon);
  }

  // NMEA (d)ddmm.mmmm and its hemisphere
  static bool coordinate(std::string const& v, std::string const& h, int degree_digits, double& out) {
    if (v.size() < static_cast<size_t>(degree_digits) + 2 || h.size() != 1)
      return false;
    for (int i = 0; i < degree_digits + 2; i++)
      if (!std::isdigit(static_cast<unsigned char>(v[i])))
        return false;
    out = std::atoi(v.substr(0, degree_digits).c_str()) + std::atof(v.substr(degree_digits).c_str()) / 60;
    if (h == "S" || h == "W")
      out = -out;
    else if (h != "N" && h != "E")
      return false;
    return true;
  }

  std::string line_;
  std::string last_;
  bool valid_ = false;
  double lat_ = 0, lon_ = 0;
};

#endif
