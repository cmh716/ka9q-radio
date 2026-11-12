#!/bin/sh
# Regression and lifecycle tests for the digital voice decoders.
#   ./run.sh            build, run every case, compare with expected/
#   ./run.sh --bless    accept the current output as expected
#   SANITIZE=1 ./run.sh build and run under AddressSanitizer and UBSan
# Run ./fetch.sh first for the recordings. Expected output is from an
# x86-64 gcc build; floating point on other platforms may differ slightly.
# SPDX-License-Identifier: GPL-3.0-or-later
set -e
cd "$(dirname "$0")"
TOP=..
if [ -n "$SANITIZE" ]; then
  FLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer"
  make -s -C $TOP BUILD_DIR=build-asan CXXFLAGS="$FLAGS" CFLAGS="$FLAGS"
  LIB=$TOP/build-asan/libdv.a
  export UBSAN_OPTIONS=print_stacktrace=1:suppressions=$(pwd)/ubsan.supp
else
  FLAGS="-O2 -g"
  make -s -C $TOP
  LIB=$TOP/build/libdv.a
fi
mkdir -p out
# radiod's call tracking and PCM queue (no hardware or recordings needed).
# It compiles src/dv.c itself, so it needs radiod's tree configured (the
# generated src/config_paths.h) and GNU ld to discard the radio functions
# the test never reaches.
if [ -f ../../../src/config_paths.h ]; then
  cc $FLAGS -std=gnu11 -D_GNU_SOURCE -ffunction-sections -fdata-sections \
    -iquote ../../../src -iquote .. ../../../src/dv-test.c \
    -Wl,--gc-sections -lm -lbsd -o out/calltest
  out/calltest
  cc $FLAGS -std=gnu11 -D_GNU_SOURCE -ffunction-sections -fdata-sections \
    -isystem /usr/include/x86_64-linux-gnu/bsd -DLIBBSD_OVERLAY \
    -iquote ../../../src -iquote .. ../../../src/follow-test.c \
    -Wl,--gc-sections -lm -lbsd -o out/follow-test
  out/follow-test
else
  echo "skipping radiod call queue test: build radiod (make -C src) first"
fi
# pcmrecord --by-talkgroup on reordered packets, late status and bursts,
# over local multicast
if [ -x ../../../src/pcmrecord ] && [ -f ../../../src/libradio.a ]; then
  cc $FLAGS -iquote ../../../src recorder-test.c ../../../src/libstatus.a ../../../src/libradio.a \
    -lm -lbsd -o out/recorder-test
  out/recorder-test ../../../src/pcmrecord out/recorder
else
  echo "skipping pcmrecord test: build radiod (make -C src) first"
fi
c++ $FLAGS -std=c++17 -I$TOP -I$TOP/upstream auto-call-test.cpp \
  $LIB -lcodec2 -lm -o out/auto-call-test
out/auto-call-test
cc $FLAGS -I$TOP -o out/dvtest dvtest.c $LIB -lcodec2 -lstdc++ -lm
[ -d data ] || { echo "no recordings: run ./fetch.sh first"; exit 1; }
cc $FLAGS -I$TOP call-serial-test.c $LIB -lcodec2 -lstdc++ -lm -o out/call-serial-test
out/call-serial-test

P2KEY="-n 443 -w BEE00 -y 445" # the voice-p25-phase2 system
fail=0
while read -r name args; do
  [ -z "$name" ] && continue
  case "$name" in \#*) continue;; esac
  # shellcheck disable=SC2086
  out/dvtest $args -o out/$name.s16 > out/$name.txt 2> out/$name.err || true
  echo "speech sha256 $(sha256sum < out/$name.s16 | cut -c1-16)" >> out/$name.txt
  if [ -s out/$name.err ] && grep -qE "runtime error|ERROR: AddressSanitizer|LeakSanitizer" out/$name.err; then
    echo "FAIL $name: sanitizer"; sed -n 1,20p out/$name.err; fail=1; continue
  fi
  if [ "$1" = "--bless" ]; then
    cp out/$name.txt expected/$name.txt
    echo "blessed $name"
  elif ! cmp -s out/$name.txt expected/$name.txt; then
    echo "FAIL $name"; diff expected/$name.txt out/$name.txt | head -20; fail=1
  else
    echo "ok   $name"
  fi
done <<CASES
# Decoding: talkgroups, sources, encryption, speech
p25-voice          -m 0 data/voice-p25.c64
p25p2-slot0        -m 9 -s 0 $P2KEY data/voice-p25-phase2.c64
p25p2-slot1-enc    -m 9 -s 1 $P2KEY data/voice-p25-phase2.c64
dmr-voice          -m 2 data/voice-dmr.c64
auto-dmr           -m 8 data/voice-dmr.c64
p25-control        -m 1 data/control-p25.c64
# Call endings: the signal fades (zeros) or is declared lost mid-call
p25-voice-fade     -m 0 -z 5:1.5 data/voice-p25.c64
p25-voice-lost     -m 0 -l 5 data/voice-p25.c64
p25p2-enc-fade     -m 9 -s 1 $P2KEY -z 5:1.5 data/voice-p25-phase2.c64
p25p2-enc-lost     -m 9 -s 1 $P2KEY -l 5 data/voice-p25-phase2.c64
dmr-fade           -m 2 -z 0.9:1.5 data/voice-dmr.c64
dmr-lost           -m 2 -l 0.9 data/voice-dmr.c64
# Noise: nothing decoded, nothing crashes
noise-auto         -m 8 -N 60
noise-p25p2        -m 9 $P2KEY -N 60
noise-cqpsk        -m 1 -N 60
CASES
exit $fail
