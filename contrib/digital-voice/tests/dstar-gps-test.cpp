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
  check(none(nmea("GPRMC,123519,A,4250.1234,N,07836.4567,W,0,0,211126,,,A") + "junk"), "NMEA with bytes after its checksum");
  check(none(dprs("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>" + std::string(300, 'z'))), "a D-PRS report longer than any (300 characters)");
  {
    dstar_gps g;
    deliver(g, "$GPRMC," + std::string(400, 'x'));
    std::string const l = g.last_line();
    check(l.size() == dstar_gps::kMaxReport && l.compare(l.size() - 3, 3, "...") == 0, "an unchecked long line: cut, and marked");
  }

  double const plat = 42 + 50.12 / 60, plon = -(78 + 36.45 / 60);
  std::string const packet = "KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>IC-705";
  check(position(dprs(packet), plat, plon), "D-PRS, CRC with the CR");
  check(none(dprs(packet, false)), "D-PRS, CRC without the CR: not D-PRS's");
  check(none(dprs(packet, true, true)), "D-PRS, CRC digits byte-swapped: not D-PRS's");
  // Live reports, as received (KJ4VO, 2026-11-21)
  check(position("$$CRCBBB2,KJ4VO>API52,DSTAR*:/211437z3417.34N/08351.09W[/", 34 + 17.34 / 60, -(83 + 51.09 / 60)),
        "a live D-PRS report (KJ4VO)");
  check(position("$$CRC70F2,KJ4VO>API52,DSTAR*:/211438z3417.34N/08351.09W[/", 34 + 17.34 / 60, -(83 + 51.09 / 60)),
        "and the next");
  std::string wrong = dprs(packet);
  wrong[20] = 'X';
  check(none(wrong), "D-PRS with a wrong CRC");
  check(position(dprs("KD2TYY>API705,DSTAR*:/123456z4250.12N/07836.45W>"), plat, plon), "D-PRS with a time");
  check(position(dprs("KD2TYY>API705,DSTAR*:=4250.  N/07836.  W>"), 42 + 50.0 / 60, -(78 + 36.0 / 60)), "D-PRS with position ambiguity");
  check(position(dprs("KD2TYY>API705,DSTAR*:!/5L!!<*e7>{?!"), 49.5, -72.75), "D-PRS compressed (the APRS spec's example)");
  check(none(dprs("KD2TYY>API705,DSTAR*:>status text only")), "D-PRS status, no position");
  check(position(dprs("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>cost $5 $GP"), plat, plon), "D-PRS with '$' in its comment");
  check(position(nmea("GPRMC,000000,A,0000.0000,N,00000.0000,E,0,0,211126,,,A"), 0, 0), "NMEA with a fix at 0, 0");
  check(none(dprs("KD2TYY>API705,DSTAR*:!0000.00N/00000.00E>")), "D-PRS at 0, 0 (perhaps no fix): not taken");
  {
    // A report that lost its line end runs into the next: the next is still taken
    std::string const first = dprs("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>first");
    std::string const second = dprs("KD2TYY>API705,DSTAR*:!4251.00N/07836.45W>second");
    check(position(first.substr(0, first.size() - 3) + second, 42 + 51.0 / 60, plon), "D-PRS after one cut short, no line end between");
    check(position(first.substr(0, 30) + nmea("GPRMC,123519,A,4250.1234,N,07836.4567,W,0,0,211126,,,A"), lat, lon),
          "NMEA after a D-PRS report cut short");
    check(position(dprs("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>see $$CRC1234,x"), plat, plon),
          "a '$$CRC' in a valid report's comment: the report is still taken whole");
    // Two long reports run together past the line's limit: both are taken
    std::string const pad(90, 'x'); // about 136 bytes a report
    std::string const a = dprs("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>" + pad);
    std::string const b = dprs("KD2TYY>API705,DSTAR*:!4251.00N/07836.45W>" + pad);
    check(a.size() > 100 && a.size() + b.size() > 256, "two reports longer together than the line's limit");
    dstar_gps g;
    std::string const joined = a + b + "\r"; // no line end after the first
    for (size_t i = 0; i < joined.size(); i += 5)
      g.feed(reinterpret_cast<uint8_t const *>(joined.data() + i), static_cast<int>(std::min<size_t>(5, joined.size() - i)));
    check(g.valid() && near(g.latitude(), 42 + 51.0 / 60), "the second (the latest) is taken");
    // Codex's cases: a 255-byte report then another; and a "$$CRC" in the
    // second of two run-together reports' comment
    auto sized = [](std::string const& head, size_t total, char fill) { // a report of total bytes
      return dprs(head + std::string(total - dprs(head).size(), fill));
    };
    std::string const long1 = sized("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>", 255, 'y');
    check(long1.size() == 255, "a 255-byte report");
    std::string const next = dprs("KD2TYY>API705,DSTAR*:!4251.00N/07836.45W>next");
    check(position(long1 + next, 42 + 51.0 / 60, plon), "a 255-byte report, then another without a line end");
    std::string const r142 = sized("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>", 142, 'p');
    std::string const r158 = sized("KD2TYY>API705,DSTAR*:!4251.00N/07836.45W>see $$CRC1234,x", 158, 'q');
    check(r142.size() == 142 && r158.size() == 158, "142- and 158-byte reports");
    check(position(r142 + r158, 42 + 51.0 / 60, plon), "and run together, the second with \"$$CRC\" in its comment");
    // Five reports run together past the buffer, its boundary slid across
    // the last one's "$$CRC": the last is taken wherever the boundary falls
    bool all = true;
    for (int shift = 0; shift < 10; shift++) {
      // The last report starts at 1018 ... 1027: the 1024th character is
      // each of its "$$CRC" in turn, and just before and after
      std::string run;
      for (int k = 0; k < 3; k++)
        run += sized("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>", 255, 'r');
      run += sized("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>", 1018 + shift - 765, 'r');
      std::string const last = dprs("KD2TYY>API705,DSTAR*:!4251.00N/07836.45W>last");
      dstar_gps h;
      deliver(h, run + last);
      all = all && h.valid() && near(h.latitude(), 42 + 51.0 / 60);
    }
    check(all, "past the buffer, wherever its boundary falls in the last report's start");
    // Codex's case: a D-PRS report for 42.5 N whose comment holds a valid
    // NMEA sentence for 49 N: the packet's own position is taken
    std::string const inner = nmea("GPRMC,123519,A,4900.0000,N,07836.4567,W,0,0,211126,,,A");
    std::string const outer = dprs("KD2TYY>API705,DSTAR*:!4230.00N/07836.45W>see " + inner + " here");
    check(position(outer, 42.5, plon), "a checksummed sentence in a D-PRS comment doesn't override its position");
    std::string const after = dprs("KD2TYY>API705,DSTAR*:!4251.00N/07836.45W>after");
    check(position(outer.substr(0, outer.size() - 4) + after, 42 + 51.0 / 60, plon) ,
          "that report cut short, then another: the other (the quoted sentence isn't taken)");
    {
      // The diagnostic line is the report that passed, exactly
      dstar_gps d;
      deliver(d, outer.substr(0, outer.size() - 4) + after);
      check(d.last_line() == after, "the diagnostic line is the span that passed, not the whole line");
      dstar_gps f;
      deliver(f, "$GPRMC,no checksum");
      check(std::string(f.last_line()) == "$GPRMC,no checksum", "and the line itself when nothing in it passed");
    }
    dstar_gps noise;
    std::string const junk(600, 'z');
    deliver(noise, junk + dprs("KD2TYY>API705,DSTAR*:!4250.12N/07836.45W>"));
    check(noise.valid() && near(noise.latitude(), plat), "a report after a long run of noise");
  }

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
