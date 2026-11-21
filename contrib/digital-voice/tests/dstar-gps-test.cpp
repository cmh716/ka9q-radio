// D-STAR position reports (dstar_gps.h): NMEA and D-PRS lines, checksums,
// fed as the slow data's GPS blocks deliver them.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <cstdio>
#include <string>
#include "dstar_gps.h"

static int Failures;
static void check(bool ok, char const *what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  Failures += !ok;
}
static bool near(double a, double b) { return std::fabs(a - b) < 1e-5; }

// An NMEA sentence with its checksum
static std::string nmea(std::string body) {
  unsigned sum = 0;
  for (char c : body) sum ^= static_cast<uint8_t>(c);
  char tail[8];
  std::snprintf(tail, sizeof tail, "*%02X", sum);
  return "$" + body + tail;
}
// A D-PRS line: the CRC over the packet (with its CR, or not; byte-swapped, or not)
static std::string dprs(std::string packet, bool with_cr = true, bool swapped = false) {
  std::string const covered = with_cr ? packet + "\r" : packet;
  uint16_t c = dstar_gps::crc(covered.data(), covered.size());
  if (swapped) c = static_cast<uint16_t>((c >> 8) | (c << 8));
  char head[16];
  std::snprintf(head, sizeof head, "$$CRC%04X,", c);
  return head + packet;
}
// As the slow data delivers it: 5 characters a block, the line ended by CR
static void deliver(dstar_gps& g, std::string const& line) {
  std::string s = line + "\r";
  for (size_t i = 0; i < s.size(); i += 5)
    g.feed(reinterpret_cast<uint8_t const *>(s.data() + i), static_cast<int>(std::min<size_t>(5, s.size() - i)));
}
static bool position(std::string const& line, double lat, double lon) {
  dstar_gps g;
  deliver(g, line);
  return g.valid() && near(g.latitude(), lat) && near(g.longitude(), lon);
}
static bool none(std::string const& line) {
  dstar_gps g;
  deliver(g, line);
  return !g.valid();
}

int main() {
  double const lat = 42 + 50.1234 / 60, lon = -(78 + 36.4567 / 60);
  check(position(nmea("GPRMC,123519,A,4250.1234,N,07836.4567,W,000.5,054.7,211126,,,A"), lat, lon), "NMEA RMC");
  check(position(nmea("GNRMC,123519,A,4250.1234,N,07836.4567,W,000.5,054.7,211126,,,A"), lat, lon), "NMEA RMC, another talker (GN)");
  check(position(nmea("GPGGA,123519,4250.1234,N,07836.4567,W,1,08,0.9,245.4,M,-34.0,M,,"), lat, lon), "NMEA GGA");
  check(position(nmea("GPRMC,123519,A,3352.1234,S,15112.5000,E,0,0,211126,,,A"), -(33 + 52.1234 / 60), 151 + 12.5 / 60), "NMEA south and east");
  check(none(nmea("GPRMC,123519,V,4250.1234,N,07836.4567,W,0,0,211126,,,N")), "RMC without a fix (V)");
  check(none(nmea("GPGGA,123519,4250.1234,N,07836.4567,W,0,00,,,M,,M,,")), "GGA without a fix (0)");
  std::string bad = nmea("GPRMC,123519,A,4250.1234,N,07836.4567,W,0,0,211126,,,A");
  bad[10] = '9';
  check(none(bad), "NMEA with a wrong checksum");
  check(none("$GPRMC,123519,A,4250.1234,N,07836.4567,W,0,0,211126,,,A"), "NMEA without a checksum");

  double const plat = 42 + 50.12 / 60, plon = -(78 + 36.45 / 60);
  std::string const packet = "KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>IC-705";
  check(position(dprs(packet), plat, plon), "D-PRS, CRC with the CR");
  check(position(dprs(packet, false), plat, plon), "D-PRS, CRC without the CR");
  check(position(dprs(packet, true, true), plat, plon), "D-PRS, CRC digits byte-swapped");
  std::string wrong = dprs(packet);
  wrong[20] = 'X';
  check(none(wrong), "D-PRS with a wrong CRC");
  check(position(dprs("KD2TYY>API705,DSTAR*:/123456z4250.12N/07836.45W>"), plat, plon), "D-PRS with a time");
  check(position(dprs("KD2TYY>API705,DSTAR*:=4250.  N/07836.  W>"), 42 + 50.0 / 60, -(78 + 36.0 / 60)), "D-PRS with position ambiguity");
  check(position(dprs("KD2TYY>API705,DSTAR*:!/5L!!<*e7>{?!"), 49.5, -72.75), "D-PRS compressed (the APRS spec's example)");
  check(none(dprs("KD2TYY>API705,DSTAR*:>status text only")), "D-PRS status, no position");

  {
    dstar_gps g;
    deliver(g, nmea("GPGGA,123519,4250.1234,N,07836.4567,W,1,08,0.9,245.4,M,-34.0,M,,"));
    deliver(g, nmea("GPRMC,123520,A,4251.0000,N,07836.4567,W,0,0,211126,,,A"));
    check(g.valid() && near(g.latitude(), 42 + 51.0 / 60), "a later position replaces the earlier");
    deliver(g, "garbage");
    check(g.valid() && near(g.latitude(), 42 + 51.0 / 60), "a bad line keeps the last position");
    g.reset();
    check(!g.valid(), "reset forgets it");
  }
  {
    dstar_gps g;
    std::string s = nmea("GPRMC,123519,A,4250.1234,N,07836.4567,W,0,0,211126,,,A");
    std::string const lost = "$GPGGA,12351"; // a sentence whose end was lost
    std::string all = lost + s + "\r";
    g.feed(reinterpret_cast<uint8_t const *>(all.data()), static_cast<int>(all.size()));
    check(g.valid() && near(g.latitude(), lat), "a sentence after one cut short");
  }
  std::printf("%s\n", Failures ? "D-STAR GPS tests FAILED" : "D-STAR GPS tests passed");
  return Failures != 0;
}
