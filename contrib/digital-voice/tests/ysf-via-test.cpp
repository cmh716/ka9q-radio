// YSF: the "via" message from the V/D mode 2 downlink and uplink fields.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "../ysf_rx.cpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
