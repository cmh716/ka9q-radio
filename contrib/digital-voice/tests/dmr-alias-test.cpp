// DMR talker alias assembly (dmr_alias.h): each format, parts in any order,
// incomplete and changed aliases. The parts are encoded here independently,
// bit by bit, from ETSI TS 102 361-2 7.2.18 and 7.2.19.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "dmr_alias.h"

static int Failures;
static void check(bool ok, char const *what, char const *got) {
  std::printf("%s %s%s%s\n", ok ? "ok  " : "FAIL", what, ok ? "" : ": got ", ok ? "" : got);
  Failures += !ok;
}

// The four parts (header, blocks 1-3), 7 bytes each, of an alias of
// format f whose characters (code units) are units
struct parts { uint8_t p[4][7]{}; };
static parts encode(int format, std::vector<unsigned> const& units) {
  int const width = format == 0 ? 7 : format == 3 ? 16 : 8;
  int const first = format == 0 ? 7 : 8;
  std::vector<int> bits(224, 0);
  for (int b = 0; b < 2; b++) bits[b] = (format >> (1 - b)) & 1;
  for (int b = 0; b < 5; b++) bits[2 + b] = (static_cast<int>(units.size()) >> (4 - b)) & 1;
  int at = first;
  for (unsigned u : units)
    for (int b = width - 1; b >= 0; b--)
      bits[at++] = (u >> b) & 1;
  parts out;
  for (int i = 0; i < 224; i++)
    out.p[i / 56][(i % 56) / 8] |= static_cast<uint8_t>(bits[i] << (7 - i % 8));
  return out;
}
static std::vector<unsigned> chars(char const *s) {
  std::vector<unsigned> v;
  for (; *s; s++) v.push_back(static_cast<unsigned char>(*s));
  return v;
}
static std::string run(parts const& p, std::vector<int> const& order, dmr_talker_alias& a) {
  for (int k : order) a.add(k, p.p[k]);
  return a.complete() ? a.text() : "(incomplete)";
}

int main() {
  {
    dmr_talker_alias a;
    std::string t = run(encode(0, chars("N0CALL Chris")), {0, 1}, a);
    check(t == "N0CALL Chris", "7-bit, 12 characters: header and block 1", t.c_str());
  }
  {
    dmr_talker_alias a;
    std::string t = run(encode(0, chars("KD2ABC")), {0}, a);
    check(t == "KD2ABC", "7-bit, 6 characters: the header alone", t.c_str());
  }
  {
    dmr_talker_alias a;
    char const *s = "ABCDEFGHIJKLMNOPQRSTUVWXYZ01234"; // 31: all three blocks
    std::string t = run(encode(0, chars(s)), {3, 1, 0, 2}, a);
    check(t == s, "7-bit, 31 characters, parts out of order", t.c_str());
  }
  {
    dmr_talker_alias a;
    std::string t = run(encode(0, chars("ABCDEFGHIJKLMNOPQRSTUVWXYZ01234")), {0, 1, 3}, a);
    check(t == "(incomplete)", "block 2 missing: not complete", t.c_str());
  }
  {
    dmr_talker_alias a;
    std::string t = run(encode(1, {'M', 0xfc, 'l', 'l', 'e', 'r'}), {0, 1}, a);
    check(t == "M\xc3\xbcller", "ISO 8859-1 as UTF-8", t.c_str());
  }
  {
    dmr_talker_alias a;
    std::string t = run(encode(2, chars("Zo\xc3\xab")), {0}, a);
    check(t == "Zo\xc3\xab", "UTF-8 as sent", t.c_str());
  }
  {
    dmr_talker_alias a;
    std::string t = run(encode(2, {'A', 0xc3, 'B'}), {0}, a);
    check(t == "A?B", "malformed UTF-8 as ?", t.c_str());
  }
  {
    dmr_talker_alias a;
    std::string t = run(encode(3, {0x03a9, 'm', 'e', 'g', 'a'}), {0, 1}, a);
    check(t == "\xce\xa9mega", "UTF-16 as UTF-8", t.c_str());
  }
  {
    dmr_talker_alias a;
    std::string t = run(encode(0, {'A', 0x07, 'B', ' ', ' '}), {0}, a);
    check(t == "A?B", "control character as ?, trailing spaces dropped", t.c_str());
  }
  {
    dmr_talker_alias a;
    run(encode(0, chars("N0CALL Chris")), {0, 1}, a);
    std::string t = run(encode(0, chars("W1AW Hiram Maxim")), {0}, a);
    check(t == "(incomplete)", "a new alias's header: the old one is gone", t.c_str());
    t = run(encode(0, chars("W1AW Hiram Maxim")), {1, 2}, a);
    check(t == "W1AW Hiram Maxim", "and the new one assembles", t.c_str());
  }
  {
    dmr_talker_alias a;
    parts p = encode(0, chars("N0CALL Chris"));
    a.add(0, p.p[0]); a.add(1, p.p[1]); a.add(0, p.p[0]); a.add(1, p.p[1]);
    check(a.complete() && std::strcmp(a.text(), "N0CALL Chris") == 0, "repeated parts keep it", a.text());
    a.reset();
    check(!a.complete(), "reset forgets it", a.text());
  }
  std::printf("%s\n", Failures ? "DMR talker alias tests FAILED" : "DMR talker alias tests passed");
  return Failures != 0;
}
