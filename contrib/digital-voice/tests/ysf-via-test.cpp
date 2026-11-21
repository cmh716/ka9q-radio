// YSF: the "via" message from the V/D mode 2 downlink and uplink fields.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../ysf_rx.cpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

namespace {
struct ysf_via_test {
  static int failures;
  static void check(char const *up, char const *down, char const *want) {
    ysf_decoder d(nullptr, nullptr);
    std::strcpy(d.up_, up);
    std::strcpy(d.down_, down);
    dv_stats st;
    d.get_stats(&st);
    bool const ok = std::strcmp(st.message, want) == 0;
    std::printf("%s up \"%s\" down \"%s\": \"%s\"\n", ok ? "ok  " : "FAIL", up, down, st.message);
    failures += !ok;
    d.end_call();
    d.get_stats(&st);
    if (st.message[0] != '\0') {
      std::printf("FAIL the call's end doesn't clear it\n");
      failures++;
    }
  }
  static void run() {
    check("KD2TYY-RPT", "AMERICA", "via KD2TYY-RPT/AMERICA");
    check("W1AW", "", "via W1AW");
    check("", "AMERICA", "via AMERICA");
    check("**********", "AMERICA", "via AMERICA");
    check("**********", "*****", "");
    check("N0CALL", "N0CALL", "via N0CALL");
    check("", "", "");
    destination("**********", "ALL");
    destination("*********", "ALL");
    destination("CQCQCQ    ", "CQCQCQ");
    destination("K*        ", "K*");
    destination("*****EAFAJ", "ALL");
    {
      ysf_decoder d(nullptr, nullptr);
      char ok_field[11] = "**********", junk[11] = "3~^bl q   ";
      d.store_field(0, ok_field);
      d.store_field(0, junk);
      bool const kept = std::strcmp(d.dst_, "ALL") == 0;
      std::printf("%s a field with characters no callsign has (\"3~^bl q\"): left out\n", kept ? "ok  " : "FAIL");
      failures += !kept;
      char stray[11] = "W1AW\x01     "; // a control byte, then padding
      char src[11] = "MM3TWA    ";
      d.store_field(1, src);
      d.store_field(1, stray);
      bool const same = std::strcmp(d.src_, "MM3TWA") == 0;
      std::printf("%s a control byte in a field: not tidied into another caller (\"W1AW\")\n", same ? "ok  " : "FAIL");
      failures += !same;
      char nul_padded[11] = {'G', '4', 'S', 'P', 'F', 0, 0, 0, 0, 0, 0};
      d.store_field(1, nul_padded);
      bool const padded = std::strcmp(d.src_, "G4SPF") == 0;
      std::printf("%s a field padded with NULs: taken\n", padded ? "ok  " : "FAIL");
      failures += !padded;
    }
    position();
  }
  // A GPS message from frames 6 and 7: reported with its sender, cleared at the end
  static void position() {
    uint8_t m[20] = {0x00, 0x22, 0x62, 0x00, 0x30, 0x34, 0x32, 0x35, 0x50, 0x31, 0x52, 0x26 + 68, 0x26 + 26, 0x1c + 45,
                     0x20, 0x1c, 0x5b, 0x2f, 0x03, 0};
    for (int i = 0; i <= 18; i++) m[19] = static_cast<uint8_t>(m[19] + m[i]);
    ysf_decoder d(nullptr, nullptr);
    std::strcpy(d.src_, "MM3TWA");
    d.gps_.superframe();
    d.gps_.frame(6, 7, m, 4800);
    if (d.gps_.frame(7, 7, m + 10, 5280))
      std::memcpy(d.position_owner_, d.src_, sizeof d.src_);
    dv_stats st;
    d.get_stats(&st);
    bool ok = st.has_position && std::fabs(st.latitude - (42 + 50.12 / 60)) < 1e-6 && std::fabs(st.longitude + (78 + 36.45 / 60)) < 1e-6;
    std::printf("%s a YSF position with its sender (%.5f,%.5f; %s)\n", ok ? "ok  " : "FAIL", st.latitude, st.longitude, st.gps_line);
    failures += !ok;
    std::strcpy(d.src_, "G4SPF");
    d.get_stats(&st);
    ok = !st.has_position;
    std::printf("%s not with another callsign\n", ok ? "ok  " : "FAIL");
    failures += !ok;
    d.end_call();
    d.get_stats(&st);
    ok = !st.has_position;
    std::printf("%s cleared at the call's end\n", ok ? "ok  " : "FAIL");
    failures += !ok;

    // A fix before any callsign: the first callsign's, once, not each one's
    ysf_decoder e(nullptr, nullptr);
    e.gps_.frame(6, 7, m, 4800);
    e.gps_.frame(7, 7, m + 10, 5280);
    e.get_stats(&st);
    ok = !st.has_position;
    char first[11] = "MM3TWA    ", second[11] = "G4SPF     ";
    e.store_field(1, first);
    e.get_stats(&st);
    ok = ok && st.has_position;
    e.store_field(1, second);
    e.get_stats(&st);
    ok = ok && !st.has_position;
    std::printf("%s a fix before the callsign goes to the first callsign only\n", ok ? "ok  " : "FAIL");
    failures += !ok;

    // A new header without the last call's terminator: the last caller and
    // their position go, and the call ends (its serial advances)
    ysf_decoder h(nullptr, nullptr);
    std::strcpy(h.src_, "MM3TWA");
    std::strcpy(h.dst_, "ALL");
    h.gps_.frame(6, 7, m, 4800);
    if (h.gps_.frame(7, 7, m + 10, 5280))
      std::memcpy(h.position_owner_, h.src_, sizeof h.src_);
    uint64_t const serial = h.call_serial_;
    h.boundary(true, kFiCommunication);
    h.boundary(false, kFiHeader); // a header whose FICH failed: can't be sure
    ok = std::strcmp(h.src_, "MM3TWA") == 0 && h.gps_.valid() && h.call_serial_ == serial;
    std::printf("%s a communication frame, or an unchecked header, changes nothing\n", ok ? "ok  " : "FAIL");
    failures += !ok;
    h.boundary(true, kFiHeader);
    h.get_stats(&st);
    ok = h.src_[0] == '\0' && h.dst_[0] == '\0' && !h.gps_.valid() && !st.has_position && h.call_serial_ == serial + 1;
    std::printf("%s a checked header with no terminator before it: a new transmission\n", ok ? "ok  " : "FAIL");
    failures += !ok;
    std::strcpy(h.src_, "G4SPF"); // the new caller, their GPS frames missed
    h.get_stats(&st);
    ok = !st.has_position;
    std::printf("%s and the new caller doesn't get the old position\n", ok ? "ok  " : "FAIL");
    failures += !ok;
    ysf_decoder fresh(nullptr, nullptr);
    fresh.boundary(true, kFiHeader);
    ok = fresh.call_serial_ == 0;
    std::printf("%s a header with no call before it ends none\n", ok ? "ok  " : "FAIL");
    failures += !ok;
    ysf_decoder route(nullptr, nullptr); // only the route was decoded
    std::strcpy(route.up_, "W1AW");
    route.boundary(true, kFiHeader);
    route.get_stats(&st);
    ok = st.message[0] == '\0' && route.call_serial_ == 1;
    std::printf("%s a route alone doesn't carry past a checked header (it ends that transmission)\n", ok ? "ok  " : "FAIL");
    failures += !ok;
  }
  static void destination(char const *field, char const *want) {
    ysf_decoder d(nullptr, nullptr);
    char call[11];
    std::snprintf(call, sizeof call, "%s", field);
    d.store_field(0, call);
    bool const ok = std::strcmp(d.dst_, want) == 0;
    std::printf("%s destination \"%s\": \"%s\"\n", ok ? "ok  " : "FAIL", field, d.dst_);
    failures += !ok;
  }
};
int ysf_via_test::failures = 0;
}

int main() {
  ysf_via_test::run();
  std::puts(ysf_via_test::failures ? "YSF via tests FAILED" : "YSF via tests passed");
  return ysf_via_test::failures != 0;
}
