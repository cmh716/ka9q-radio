# Native P25 Phase 1 decoder

Static library linked into `radiod` for the `p25` demodulator
(`src/p25.c`). It takes 48 kHz complex IQ from the radiod channel filter
and returns 8 kHz IMBE speech; the public API is `p25_native.h`.
`src/Makefile` builds it automatically.

    p25_rx.cpp     48k->24k decimation, IF filter, RMS AGC, band-edge FLL, then
                   C4FM: discriminator, RX shaping filter, FSK4 timing (fsk4.cpp)
                   CQPSK: Gardner timing, differential decode, Costas loop
                   and sync-word polarity detection, slicing to dibits
    p25_voice.cpp  NID framing, HDU/LDU2 encryption detection, IMBE decoding
    upstream/      code taken from OP25 (framer, BCH, Golay/Hamming, tables,
                   ezpwd Reed-Solomon, fixed-point IMBE decoder)

Audio is assumed clear until the HDU or an LDU2 ESS carries a non-clear
ALGID, as OP25 does; encrypted voice is muted until the next TDU.
Encryption is not supported.

Example channel:

```ini
[Police]
mode = p25
p25-nac = 0x293          ; optional, 0 = any
# p25-demod = cqpsk      ; for simulcast systems
data = police.local
freq = "460m075"
encoding = opus
```

`control` shows recognized sync words as TP1 and decoded frames as TP2; the
carrier offset is in the frequency-offset field. `radiod -v` logs a status
line per channel each second.

## Attribution and changes from upstream

Adapted from [boatbod/op25](https://github.com/boatbod/op25/tree/718e589a8ef075143b0c861e7d4a1b01eac0cf64)
revision `718e589a`, `op25/gr-op25_repeater/lib` and `op25/gr-op25/lib`;
original notices are kept in the imported files.

- Frame synchronizer and BCH: KA1RBI (2010-2013), Graham J. Norbury (2020)
- Fixed-point IMBE vocoder: Pavel Yazev (2009)
- FSK4 MMSE timing: Frank (Radio Rausch) (2006-2007), Steve Glass (2011)
- `ezpwd/rs`: Hard Consulting Corporation (2014); `rs_base` also credits
  Phil Karn, KA9Q, with conversion by Perry Kundert
  ([ezpwd-reed-solomon](https://github.com/pjkundert/ezpwd-reed-solomon))
- C4FM filters and timing table from OP25 `p25_demodulator_dev.py`; band-edge
  FLL from GNU Radio 3.8.5 `fll_band_edge_cc`; CQPSK MMSE interpolator taps
  generated with GNU Radio 3.10.9 (no GNU Radio code is linked)

Local changes: IMBE encoder removed; the vocoder's random seed and
fixed-point Overflow/Carry flags are per instance (thread-local context) so
channels don't interfere; constructor banner removed; framer no longer uses
OP25's logger and prints NAC mismatches only in debug mode.

OP25 is GPL-3.0-or-later and ka9q-radio is GPL-3; see `LICENSE.TXT`.
