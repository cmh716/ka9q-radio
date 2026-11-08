# Native P25 decoder work

This directory contains a native P25 Phase 1 decoder with C4FM and CQPSK IQ
receive paths. The dibit decoder performs OP25 frame synchronization and NID BCH
decoding, extracts LDU1/LDU2 IMBE codewords, and emits clear audio as 160-sample
signed 16-bit mono blocks at 8 kHz. Both receive paths accept 48 kHz complex
floating-point IQ or interleaved signed 16-bit big-endian IQ, decimate and
filter to 24 kHz, then use either the C4FM quadrature/FSK4 receiver or the
CQPSK AGC/FLL/Gardner/differential/Costas receiver before feeding the same
Phase 1 decoder. The current `p25-offline` and `p25-pcap` tools exercise C4FM:
the raw-IQ runner writes decoded 8 kHz mono PCM as WAV, while `p25-pcap`
extracts 48 kHz `s16be` IQ from one RTP SSRC in a classic Ethernet/IPv4 PCAP.
The lower-level IMBE API accepts one packed 144-bit
codeword, most-significant bit first. Decoder instances keep persistent state
and support reset. No GNU Radio, Python, networking, or audio-device
dependency is required at build or runtime.

The generic public C API in `p25_native_api.h` selects either demodulator and
creates independent decoder instances. It accepts streaming interleaved float
or `s16be` IQ blocks, supports reset/NAC selection, and reports frame and
decoding statistics. PCM is delivered synchronously through the callback.
Calls on one instance must be serialized.
The C4FM path applies RMS AGC and an OP25-compatible band-edge FLL before
quadrature demodulation; its reported carrier offset combines the FLL
correction with the residual FSK4 loop estimate.

`radiod` also has a `p25` demodulator mode using the same library. Select the
modulation with optional `p25-demod = c4fm` or `p25-demod = cqpsk`; C4FM is the
default. Its native channelizer runs at a fixed 48 kHz complex-IQ rate while decoded audio and RTP
timestamps use 8 kHz mono. The existing `samprate` setting refers to audio in
this mode and is fixed to 8 kHz; `p25-iq-samprate` is not configurable yet.
An optional `p25-nac` value accepts decimal or `0x`-prefixed hexadecimal and
filters frames by NAC; zero (the default) accepts any NAC. In `control`, TP1
reports recognized sync words and TP2 reports completed frames. With
`radiod -v`, periodic P25 diagnostics also include decoded/skipped voice counts,
the last NAC, NID BCH errors, and the combined FLL/FSK4 carrier-offset estimate.
The offset is also sent in the existing `FREQ_OFFSET` status field. Repeated
synthetic captures retain at least eight of ten frames with +/-5 kHz offsets;
the combined carrier-offset estimate is within 100 Hz on those captures.
These results measure the synthetic fixture path, not live-RF performance.
Example:

```ini
[EriePD]
mode = p25
# p25-demod = c4fm
p25-nac = 0x293
data = police-audio.local
freq = "460m075"
encoding = opus
```

The decoder currently emits valid clear IMBE speech frames only. Its bounded
audio queue drains at real-time cadence; while empty, RTP timestamps advance
without sending zero-filled packets, so recordings still close on inactivity.

The IQ-to-PCM paths remain unvalidated receivers. The offline PCAP tool handles
RTP extraction, but live RTP reception is not implemented. RMS AGC and
band-edge FLL follow OP25/GNU Radio, with the FLL port based on GNU Radio
3.8.5.0. CQPSK additionally ports OP25's Gardner timing recovery,
differential decoding, Costas loop, and symbol slicing; its 8-tap/129-phase
MMSE interpolator table was generated with GNU Radio 3.10.9 and is embedded,
so no GNU Radio runtime is needed. The C4FM slicer
applies OP25's P25 Phase 1 dibit maps after detecting the corresponding exact
sync word; tests cover the normal, reversed-polarity, X2400, N1200, and P1200
maps. IQ tests also cover 90-degree phase rotations, 0.1x/2x input amplitude,
and +/-5 kHz carrier offsets. The 48 kHz complex-float and `s16be` chains have
only been checked against synthetic fixtures; real IQ and RF reception remain
unvalidated. A 12-second live 48 kHz stream check received
1601 RTP packets with no sequence or timestamp gaps, but detected no P25 sync
words or frames; the stream may have been idle. That check used stats-only
output, and no live IQ or decoded audio was retained.

Build and run the focused test:

```sh
make -C contrib/p25-native test
```

Build and run the offline receiver on a raw interleaved 48 kHz `s16be` IQ file:

```sh
make -C contrib/p25-native offline
contrib/p25-native/build/p25-offline input.s16be output.wav 0x293
```

The input is raw IQ payload data, not RTP or a pcap file. The command reports
sample, symbol, frame, and voice-codeword counts to stderr.

Decode a `tcpdump` capture directly:

```sh
make -C contrib/p25-native pcap
contrib/p25-native/build/p25-pcap p25-iq.pcap output.wav 0x293 460075
```

The NAC defaults to zero (accept any); the SSRC is optional and defaults to
the first RTP stream. Specify the SSRC when the capture contains multiple
streams. The tool reports lost RTP packets, timestamp discontinuities,
recognized syncs, frame counts, and estimated carrier offset. It handles
variable RTP headers, VLAN-tagged Ethernet, and sequence/timestamp wraparound.
Use `-` instead of an output filename to run stats-only without retaining
decoded PCM.
It currently supports classic PCAP with Ethernet, IPv4, UDP, and 48 kHz
interleaved `s16be` IQ; pcapng, IPv6, fragmented IP, other link types, and
other IQ encodings/rates are not supported.

The tests exercise the C API with upstream-generated IMBE codewords,
synthetic P25 Phase 1 dibit frames, and an OP25-generated C4FM IQ fixture.
They check state isolation, reset, NID error correction, arbitrary input block
sizes, clear/encrypted ESS gating, and the frame-to-PCM path. The C4FM fixture
uses the pinned OP25 transmitter at 48 kHz and contains a preamble, two
synthetic frames, and a tail for receiver timing latency; OP25's receiver
recovers both sync words from the same waveform. The native regression recovers
both frames and all 18 voice codewords and produces PCM identically across
contiguous and irregular IQ blocks. Repeated fixtures with +/-5 kHz carrier
offset recover at least eight of ten frames, and the combined offset estimate
is within 100 Hz. This tests algorithm plumbing on synthetic signals, not
real-signal audio quality or RF performance. The suite
also exercises the C API with `s16be` blocks and runs `p25-offline`, checking
that it writes a valid 8 kHz mono PCM WAV. The synthetic CQPSK regression uses
ideal rectangular differential-QPSK symbols, recovers two frames and 18 voice
codewords, and checks block-boundary continuity, amplitude/phase variation,
300 Hz carrier offsets, and a 0.1% sample-clock offset. This validates the DSP
plumbing only; it is not an LSM waveform or evidence of simulcast performance.
A generated PCAP exercises VLAN and
variable RTP headers, an unrelated SSRC, and sequence/timestamp wraparound;
the PCM from `p25-pcap` must match the raw-IQ runner exactly.
Calls on the same decoder instance must be serialized; separate instances have
independent vocoder and synthesis state, including arithmetic flags. The
tests check independent, interleaved, and parallel instances. The FSK4 test
checks symbol rate and verifies that arbitrary block partitions preserve
recovered output.

The IQ fixture is 48 kHz, interleaved complex-float32 little-endian data. Its
one-byte-per-dibit source is `fixtures/op25-c4fm-test.dibits`. To regenerate
the fixtures using the pinned OP25 checkout:

```sh
make -C contrib/p25-native fixture-tool
contrib/p25-native/build/generate_c4fm_dibits \
    contrib/p25-native/fixtures/op25-c4fm-test.dibits
python3 contrib/p25-native/tools/generate_c4fm_fixture.py \
    --op25-apps /tmp/boatbod-op25-ref-718e589/op25/gr-op25_repeater/apps \
    --dibits contrib/p25-native/fixtures/op25-c4fm-test.dibits \
    --output contrib/p25-native/fixtures/op25-c4fm-48k.c64 \
    --s16be-output contrib/p25-native/fixtures/op25-c4fm-48k.s16be
```

Fixture generation requires GNU Radio and OP25; running the native tests does
not.

AddressSanitizer and the non-arithmetic UndefinedBehaviorSanitizer checks pass
with the current tests. Enabling UBSan's signed-overflow and shift checks
reports operations in the imported fixed-point IMBE implementation (including
`basicop2.cc`, `sa_decode.cc`, and `v_synt.cc`). These arithmetic paths have not
been rewritten because doing so without OP25 reference-audio comparisons could
change decoded speech.

The Phase 1 symbol API waits until a valid LDU2 ESS says the stream is clear
before emitting speech. It suppresses speech when ESS identifies encryption,
and after a TDU until a new ESS is decoded. Encryption/decryption is not
implemented. PCM callbacks are synchronous and must not throw; a nonzero
return stops processing with an explicit callback error.

## Upstream attribution and licensing

The decoder adapts code from [boatbod/op25 at revision
`718e589a8ef075143b0c861e7d4a1b01eac0cf64`](https://github.com/boatbod/op25/tree/718e589a8ef075143b0c861e7d4a1b01eac0cf64),
principally its `op25/gr-op25_repeater/lib` and `op25/gr-op25/lib` trees.
Original notices are retained in imported files. In particular:

- The P25 frame synchronizer and BCH sources credit KA1RBI (2010-2013) and
  Graham J. Norbury (2020).
- The fixed-point IMBE vocoder credits Pavel Yazev (2009, `pyazev@gmail.com`).
- The FSK4 MMSE table credits Frank (Radio Rausch) (2006-2007) and Steve
  Glass (2011).
- The retained `ezpwd/rs` and `ezpwd/rs_base` headers credit Hard Consulting
  Corporation (2014); `rs_base` also credits Phil Karn (KA9Q), with conversion
  by Perry Kundert. The upstream project is
  [pjkundert/ezpwd-reed-solomon](https://github.com/pjkundert/ezpwd-reed-solomon);
  its GPL and LGPL notices remain in the retained headers.

The C4FM receive filters and timing table are derived from OP25's
`p25_demodulator_dev.py` and `op25_c4fm_mod.py`. The CQPSK MMSE coefficient
table was generated using [GNU Radio](https://github.com/gnuradio/gnuradio)
3.10.9; no GNU Radio code is linked into the decoder. OP25 is GPL-3.0-or-later
and KA9Q-radio is GPL-3; the LGPL-covered `ezpwd/rs_base` notice is retained.
See the repository `LICENSE.TXT` and the imported source headers for the
applicable terms.

The default fixed-point `imbe_vocoder` used here matches OP25's default
`p25p1_voice_decode` path. OP25's optional `IMBE=soft` implementation is not
included in this milestone. The C wrapper adapts the 144-bit codeword and
error counters to a library API; no OP25 DSP or decoding algorithm was
reimplemented. One upstream seam was changed: OP25's random generator used a
file-global seed despite `imbe_vocoder` already having a per-instance seed.
The extracted implementation passes that seed explicitly so independent
decoder instances do not affect each other's synthesis state. The upstream
constructor's banner print was also removed to keep a library instance from
writing unsolicited messages to the host application's stderr. The fixed-point
arithmetic `Overflow` and `Carry` flags were also global in OP25; they are now
scoped to each decoder instance through a thread-local context selector.

## Remaining work

1. Validate both C4FM and CQPSK against OP25 using identical real recorded IQ.
   No suitable active P25 capture is available yet; the brief live stream
   check detected no P25 frames.
2. Measure acquisition, recovery, decoded audio quality, latency, and CPU on
   representative C4FM and simulcast LSM captures.
3. Add the standalone live RTP receiver, reusing the library now integrated
   into `radiod`.

Changes to the upstream decoder are documented here: per-instance IMBE random
state and fixed-point arithmetic flags, removal of constructor output, plus
the C API and radiod integration. New decoder, test, and radiod glue is part
of ka9q-radio and uses its project license. The OP25 checkout is not a
build-time dependency.

## Capturing IQ for comparison

No recorded IQ fixture is currently available. To create a repeatable capture,
record the configured KA9Q IQ multicast stream with its RTP headers intact:

```sh
sudo tcpdump -i <multicast-interface> -s 0 \
    -w p25-iq.pcap 'udp dst port <configured-rtp-port>'
```

Replace both placeholders with the interface and RTP port used by the local
`radiod` configuration; the example port `5004` is installation-specific.
Keep the matching `radiod` configuration and note the capture's center
frequency, sample rate, encoding, SSRC, and any known frequency offset. Include
several seconds before and after a transmission so acquisition, end-of-call,
and idle behavior can be examined. The file is a packet capture, not raw IQ.
The `p25-pcap` offline runner removes variable-length RTP headers, reports
packet sequence/timestamp discontinuities, and passes the `s16be` IQ payloads
to the decoder.
