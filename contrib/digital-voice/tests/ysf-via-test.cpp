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
