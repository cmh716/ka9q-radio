# Digital voice decoders

Static library linked into `radiod` for the `p25`, `dmr`, `m17`, `ysf`, `dstar`, `nxdn` and `digital` demodulators
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
    ysf_rx.cpp       YSF: frame sync, FICH, DN (AMBE+2) and VW (IMBE) voice, DN callsigns
    dstar_rx.cpp     D-STAR: GMSK (M&M timing), frame sync, slow-data header, AMBE voice
    nxdn_rx.cpp      NXDN48/96: frame sync, descrambling, LICH, SACCH/FACCH1 (VCALL), AMBE+2 voice
    dv_ambe.cpp      AMBE+2 voice decoding shared by DMR and NXDN
    dv_auto.cpp      "digital": every receiver on the same IQ, the first to decode voice wins
    dv_api.cpp       C API
    upstream/        code taken from OP25; upstream/libm17 from the M17 Foundation

P25 audio is assumed clear until the HDU or an LDU2 ESS carries a non-clear
ALGID, as OP25 does; encrypted voice is muted until the next TDU.

DMR follows OP25: a timeslot plays while its latest sync is a voice sync.
With `dmr-slot = 0` the decoder plays whichever slot starts a call first and
stays with it until that call ends. Data bursts give the color code (slot
type, Golay 20,8) and, from the voice LC header (BPTC 196,96 and RS 12,9,
OP25's dmr_slot.cc), the source radio ID, the talkgroup and the privacy
bit; a PI header also marks a call encrypted, and encrypted calls are
muted. A call joined after its header has no IDs and isn't checked for
encryption (the embedded LC in the voice bursts would cover that). Tested on
the trunk-recorder-pro DMR capture (CC 13, 4477055 to TG 44772); the
encryption path hasn't met an encrypted call yet. AMBE+2 output level follows the talker and
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

YSF follows OP25's receive path (rx_sync.cc), with three changes: the FICH
and DN data channel are decoded with libm17's Viterbi decoder (the same K=5
code; OP25 walks the trellis with hard decisions and fails on any bit
error), which moves the weak-signal cliff about 2 dB lower; VW (full rate)
audio isn't multiplied by 32768, which in OP25 clips every sample; and
voice is taken only from communication frames. The source callsign comes
from the DN data channel. The receiver is centred on 750 Hz per symbol
level and decodes 550-1000 Hz (OP25's transmitter uses DMR's 648). VD mode 1
and data-only frames are muted, as in OP25. Tested only on synthesized
signals (OP25's YSF transmitter code with its AMBE and IMBE encoders, FM
modulated): every frame down to 9 dB SNR, about 70% at 6 dB, nothing from
noise.

D-STAR uses the shared front end, then an FM discriminator, a root raised
cosine filter (alpha 0.2), Mueller & Muller timing as in OP25's GMSK path,
decision-directed DC/level tracking and a bit slicer. The framing is new
code: OP25's rx_sync passes D-STAR's one-bit symbols through
dibits_to_bits() before decode_dstar(), which doesn't look right. A
transmission is found by its header sync, or late, by the frame sync in
the slow data of every 21st frame. The MY and UR callsigns come from the
radio header at the start (descrambled, de-interleaved and Viterbi decoded,
K=3; CRC checked; verified on the live capture with no bit errors), or
otherwise from the slow-data copy of the header. The header is tried after
every header sync while unlocked, since an openSPOT 4 sometimes sends zeros
rather than the 1010 preamble before it. Voice is decoded with
OP25's decode_dstar() and mbelib's AMBE 2400 tables; OP25's synthesizer
renders it ~14 dB hotter than P25 and clips, so the spectral magnitudes are
scaled down 12 dB first. Tested on a 3-minute capture of an openSPOT 4
relaying reflector traffic (two callers decoded, 7836 voice frames), and on
that capture with added noise: about 2600 of 2700 voice frames down to
6 dB SNR, nothing from noise.

NXDN follows trunk-recorder-pro's NXDN module (frame sync, PN9
descrambling, LICH, the SACCH/FACCH1 channel coding and the VCALL layer 3
message), ported from Rust; its convolutional code is M17's, so libm17's
Viterbi decoder does it. The front end runs at the NXDN rate (2400 or 4800
baud) with an RRC (alpha 0.2) receive filter. Two additions: the LICH has no
error correction, so within a call a failed one is replaced by the last
good one when the frame sync was where it belonged (9 dB SNR: 344 -> 595 of
664 voice codewords); and the `[nxdn]` preset narrows radiod's channel
filter to +/-4 kHz for NXDN48 (6 dB SNR: nothing -> 87%). Calls with a
non-zero cipher type in VCALL are muted. The source unit ID shows as the
callsign. Tested on a 3-minute openSPOT 4 NXDN48 capture (unit 9998, every
voice codeword decoded) and on it with added noise; NXDN96 is untested.

`digital` (DV_AUTO) runs P25 C4FM, DMR, YSF, D-STAR, NXDN48, NXDN96 and M17
receivers side by side. The first to decode voice owns the audio; it lets go
500 ms after its last frame, and the next to decode voice takes over. On a
test stream of all six modes separated by noise (real P25, DMR, D-STAR and
NXDN captures, synthesized YSF and M17) each was detected with its
callsign/ID within half a second, and noise never claimed the audio. It
costs about 8% of an N150 core per channel, against ~1% for one mode; six
of the receivers could share one front end if that matters.

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

[Fusion]
mode = ysf
data = ysf.local
freq = "444m000"
encoding = opus

[DSTAR]
mode = dstar
data = dstar.local
freq = "433m900"
encoding = opus

[NXDN]
mode = nxdn              ; NXDN48; mode = nxdn96 for 12.5 kHz NXDN96
data = nxdn.local
freq = "433m900"
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
- DMR slot type, BPTC(196,96) and Hamming codes (`bptc19696`, `golay2087`,
  `hamming`): Ian Wraith, Jonathan Naylor G4KLX, Graham J. Norbury,
  GPL-2.0-or-later, unmodified
- Fixed-point IMBE vocoder: Pavel Yazev (2009)
- AMBE+2 parameter decoding: mbelib (2010, ISC license) as carried in OP25;
  `p25p2_vf.cc` (AMBE codeword error correction): Max H. Parke, KA1RBI;
  `software_imbe_decoder.cc` (synthesis): Steve Glass (2008-2009)
- FSK4 MMSE timing: Frank (Radio Rausch) (2006-2007), Steve Glass (2011)
- YSF receive (`ysf_rx.cpp`, from `rx_sync.cc`) and `ysf_const.h`, `crc16.h`:
  Max H. Parke KA1RBI, Graham J. Norbury
- D-STAR timing (`dstar_rx.cpp`) follows GNU Radio's clock_recovery_mm_ff as
  configured by OP25; frame layout and slow-data header from OP25's
  dstar_tx_sb / dstar_header.h; the header CRC is reimplemented here (OP25's
  CCITTChecksumReverse is GPL-2.0-only, so it isn't included)
- NXDN (`nxdn_rx.cpp`): framing, scrambler, LICH, channel coding and VCALL
  parsing from trunk-recorder-pro (TrunkRecorder/trunk-recorder-pro,
  `crates/trunk-core/src/nxdn`, GPL-3.0-or-later)
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
DMR and YSF vocoder files (`p25p2_vf`, `rs`, `mbelib`, `ambe`, `software_imbe_decoder`,
`imbe_decoder`, `dmr_const.h`, `bit_utils.h`, `ysf_const.h`, `crc16.h`) are unmodified.

OP25 is GPL-3.0-or-later, libm17 GPL-2.0-or-later and ka9q-radio GPL-3; mbelib is ISC; see
`LICENSE.TXT`. AMBE+2 is a DVSI codec; mbelib is an independent
implementation, and its patent status is for the user to judge.
