# Digital voice decoders

Static library linked into `radiod` for the `p25`, `dmr` and `m17` demodulators
(`src/dv.c`). It takes 48 kHz complex IQ from the radiod channel filter and
returns 8 kHz speech; the public API is `dv.h`. `src/Makefile` builds it
automatically.

    dv_frontend.cpp  shared receive chain: 48k->24k decimation, IF filter, RMS AGC,
                     band-edge FLL; 4FSK discriminator, receive filter (P25 C4FM
                     or root raised cosine), FSK4 timing (fsk4.cpp), slicer
    p25_rx.cpp       P25: C4FM or CQPSK (Gardner, differential, Costas) to dibits
    p25_voice.cpp    P25: NID framing, HDU/LDU2 encryption detection, IMBE decoding
    dmr_rx.cpp       DMR: burst framing, timeslot tracking, AMBE+2 decoding
    m17_rx.cpp       M17: discriminator, RRC, sync-word timing, LSF/LICH, Codec2
    dv_api.cpp       C API
    upstream/        code taken from OP25; upstream/libm17 from the M17 Foundation

P25 audio is assumed clear until the HDU or an LDU2 ESS carries a non-clear
ALGID, as OP25 does; encrypted voice is muted until the next TDU.

DMR follows OP25: a timeslot plays while its latest sync is a voice sync.
With `dmr-slot = 0` the decoder plays whichever slot starts a call first and
stays with it until that call ends. Encrypted DMR is not detected yet, and
color codes are not checked. AMBE+2 output level follows the talker and
varies a lot between transmissions; the `[dmr]` preset leaves the gain at
0 dB, which sounds about as loud as P25 on live signals.

M17 has no symbol timing loop: as in the M17 reference receivers, each
frame's sync word sets the sample phase, gain and DC offset for its payload
(OP25's FSK4 timing loop can settle on false equilibria on M17's
alternating preamble). A transmission starts on a valid LSF (CRC checked),
or late, on a stream frame whose LICH decodes and whose Viterbi error rate is
low; the callsigns, CAN and encryption come from the LSF or the six LICH
chunks. Scrambled or AES-encrypted transmissions are muted, and `m17-can`
selects one Channel Access Number. Codec2 3200 and 1600 (voice + data)
are decoded with the system libcodec2. M17 has only been tested on
synthesized signals (libm17's encoder, FM modulated, 2-20 dB SNR, +/-1 kHz
offset, +/-10% deviation): every frame decoded down to 10 dB, no false
audio from 120 s of noise.

Example channels:

```ini
[Police]
mode = p25
p25-nac = 0x293          ; optional, 0 = any
# p25-demod = cqpsk      ; for simulcast systems
data = police.local
freq = "460m075"
encoding = opus

[Repeater]
mode = dmr
dmr-slot = 1             ; optional, 0 = whichever slot starts first
data = dmr.local
freq = "441m000"
encoding = opus

[M17]
mode = m17
# m17-can = 0            ; optional, -1 = any
data = m17.local
freq = "433m475"
encoding = opus
```

`control` shows recognized sync words as TP1 and decoded frames (P25) or
bursts (DMR) as TP2; the carrier offset is in the frequency-offset field.
`radiod -v` logs a status line per channel each second.

## Attribution and changes from upstream

Adapted from [boatbod/op25](https://github.com/boatbod/op25/tree/718e589a8ef075143b0c861e7d4a1b01eac0cf64)
revision `718e589a`, `op25/gr-op25_repeater/lib` and `op25/gr-op25/lib`;
original notices are kept in the imported files.

- P25 frame synchronizer and BCH: KA1RBI (2010-2013), Graham J. Norbury (2020)
- DMR framing and timeslot logic (`dmr_rx.cpp`, from `rx_sync.cc`/`dmr_cai.cc`):
  KA1RBI, Graham J. Norbury (2019-2022)
- Fixed-point IMBE vocoder: Pavel Yazev (2009)
- AMBE+2 parameter decoding: mbelib (2010, ISC license) as carried in OP25;
  `p25p2_vf.cc` (AMBE codeword error correction): Max H. Parke, KA1RBI;
  `software_imbe_decoder.cc` (synthesis): Steve Glass (2008-2009)
- FSK4 MMSE timing: Frank (Radio Rausch) (2006-2007), Steve Glass (2011)
- `upstream/libm17` (M17 frame encoding/decoding): libm17 1.4.1 by Wojciech
  Kaczmarski, SP5WWP, and the M17 Foundation,
  [M17-Project/libm17](https://github.com/M17-Project/libm17),
  GPL-2.0-or-later, unmodified (its unit tests and build files are omitted);
  Codec2 is the system library (LGPL-2.1)
- `ezpwd/rs`: Hard Consulting Corporation (2014); `rs_base` also credits
  Phil Karn, KA9Q, with conversion by Perry Kundert
  ([ezpwd-reed-solomon](https://github.com/pjkundert/ezpwd-reed-solomon))
- C4FM filters and timing table from OP25 `p25_demodulator_dev.py`; band-edge
  FLL from GNU Radio 3.8.5 `fll_band_edge_cc`; CQPSK MMSE interpolator and DMR
  root-raised-cosine taps generated with GNU Radio formulas (no GNU Radio code
  is linked)

Local changes: IMBE encoder removed; the IMBE vocoder's random seed and
fixed-point Overflow/Carry flags are per instance (thread-local context) so
channels don't interfere; constructor banner removed; the P25 framer no
longer uses OP25's logger and prints NAC mismatches only in debug mode. The
DMR vocoder files (`p25p2_vf`, `rs`, `mbelib`, `ambe`, `software_imbe_decoder`,
`imbe_decoder`, `dmr_const.h`, `bit_utils.h`) are unmodified.

OP25 is GPL-3.0-or-later, libm17 GPL-2.0-or-later and ka9q-radio GPL-3; mbelib is ISC; see
`LICENSE.TXT`. AMBE+2 is a DVSI codec; mbelib is an independent
implementation, and its patent status is for the user to judge.
