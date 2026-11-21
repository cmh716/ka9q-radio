# Digital voice, trunking and VHF/UHF monitoring: to do

The `digital` branch is for VHF and UHF (a Fobos SDR, 50 MHz at a time
around 445.6 MHz, and a VHF tuning). HF is left to UberSDR: FreeDV is
dropped here. Local systems: P25 Phase II (trunked, 800 MHz), LTR
Standard, NXDN NEXEDGE 9600 (VHF and UHF), DMR Tier III Capacity Max
(going encrypted), and ham repeaters and an OpenSpot 4 hotspot (433.9 MHz)
for DMR, D-STAR and YSF.

## Next, in order

1. [ ] **YSF GPS, live** (`ysf_gps.h`, V/D mode 2 frames 6 and 7, the
   layout YSFGateway reads): a Yaesu radio with GPS on a YSF room. Look for
   `position=` and `gps_line="00 22 62 ..."` in the -v log. Everything is
   unit-tested only.
2. [ ] **DMR GPS**, if it can be tested: in all the logs so far no DMR call
   carried embedded GPS (no FLCO 8 in `lc=`); DMR radios send position as
   data (LRRP), which Brandmeister handles itself.
3. [ ] **Two-tone sequential paging (fire/EMS)**: recognise a
   department's two-tone page on an analog channel; log it, and start or
   tag a recording or run a command. Shares an alert mechanism with the
   watchlist (below).
4. [ ] **P25 multi-call following** (`p25-voice-channels = N`): one
   control-channel decode, N tuners, one output stream each.
5. [ ] **monitor: receiver summary**: the front end's level, A/D overloads,
   gain and the busiest channels, with recent peaks and counter deltas held
   for a while, so a brief event stays visible after it passes.
6. [ ] **monitor: a wider Call field**, so less is truncated (talkgroup,
   talker or alias, locator, D-STAR text / YSF route).
7. [ ] **A spectrum display for the VHF/UHF tunings**, the way UberSDR
   shows HF. Look first at ka9q-web (`~/ka9q-web`, a web spectrum and
   waterfall for ka9q-radio); extending it may be less work than a new app.

## Ideas, not yet ordered

- [ ] **Activity finder**: log carriers that come up on frequencies not in
      the configuration (frequency, time, length, a guess at the mode), to
      find unknown users (the "AldenMystery" channels) and build the list.
- [ ] **NOAA weather radio alerts (SAME)**, 162.400-162.550 MHz: decode
      the alert headers (event, counties, times); the weekly test is a
      ready source.
- [ ] **Watchlist alerts**: notify, log or record when a talkgroup,
      callsign, radio ID or tone is heard. Two-tone paging is one trigger.
- [ ] **POCSAG/FLEX paging** (hospitals, some fire departments).
- [ ] **APRS positions** (144.390): `aprs`/`aprsfeed` exist; feed their
      positions into monitor's locator display with D-STAR's and YSF's.
- [ ] **Fobos self-generated tone**: at lna_gain 2 the level jumps about
      24 dB between vga_gain 3 and 4 with no A/D clipping (2026-11-21), most
      likely a tone of its own. Find its frequency; if it's a fixed spur, a
      notch or another sample rate may allow more gain.
- [ ] Even out digital voice levels across modes, radios and talkers.
- [ ] Recording: optionally notch CTCSS out of analog recordings
      (radiod's `ctcss-notch` does it for all of a channel's output).

## Waiting on live testing

- [ ] YSF downlink/uplink ("via UPLINK/DOWNLINK"): no reflector heard so
      far filled them.
- [ ] LTR Standard: `mode = ltr` and following (`follow = yes`,
      `ltr-repeaters`) not yet heard live in radiod; voice inversion;
      several calls interleaved on one repeater.
- [ ] NXDN: the control channel number in CCH_INFO (nothing logged on the
      UHF system); the 0.4 s call whose LICH didn't decode.
- [ ] P25 Phase 1 trunked system one county over (Phase 1 voice following
      tested on recordings only).
- [ ] Mixed FM + digital channel (`analog = yes`), talkgroup names
      side by side for NXDN and P25, `pcmrecord --by-talkgroup` with
      p25-follow.
- [ ] M17, once an M17 radio or hotspot is available.
- [ ] P25 Phase 2: about 0.4 s of an encrypted call can play before the ESS
      says it's encrypted, when the MAC_PTT is missed.
- [ ] Control channel search: a real control channel change, a search past
      several dead frequencies; across sites (needs the band plan, NAC reset).

## Done, and confirmed on the air

- [x] P25 Phase 1/2, DMR, M17, YSF, D-STAR, NXDN48/96 decoders, DV_AUTO.
- [x] P25 grant following and Phase 2 voice; control channel search.
- [x] NXDN Type-C following, by direct frequency and by channel number
      (`nxdn-channels`), VHF and UHF.
- [x] DMR: polarity inversion; idle bursts; **talker alias** (FLCO 4-7, all
      formats) shown in monitor in place of the radio ID, e.g.
      `TG9 AB5JK`; `lc=` diagnostic in the -v log. Private calls are
      `U<id>`, not taken for talkgroups.
- [x] **D-STAR**: the slow data's text message (`M0HPL says "Richard NE
      England"`); **GPS** from D-PRS (CRC over the packet and its CR, as
      live reports settled) and NMEA, shown as a locator in Call
      (`CQCQCQ KJ4VO EM84bg`) and a Position line with distance and bearing
      (`monitor --location`); callsign fields trimmed at both ends.
- [x] **YSF**: destination `ALL` for a call to everyone; source callsigns.
- [x] Fobos: A/D overload counting and logging; an opt-in AGC (`agc = yes`,
      VGA only), which showed the overload was the tone above, not the A/D.
- [x] Frequency catalog (`frequency-catalog.py`) and `rr2radiod.py`:
      RadioReference and RepeaterBook downloads to a curated master list, a
      generated `50-conventional.conf` and monitor's names; overrides,
      CTCSS notch, mixed channels, previews (`--replaces`).
- [x] monitor dashboard: Call (talkgroup, talker or alias, locator,
      message, trunking state) and Last columns, tones and NAC, positions.
- [x] Talkgroup names (`talkgroups.d`, per system), `pcmrecord
      --by-talkgroup` and `--call-log`.
- [x] Carrier gate for DV_AUTO / mixed channels; JMBE vs mbelib A/B (no
      audible difference).

## Housekeeping

- [ ] Merge `digital` to main (on hold): it's large; plan how to split it
      into reviewable pieces.
- [ ] More test recordings: YSF, D-STAR, NXDN, M17, a mixed repeater, and
      live captures with GPS, talker alias and text.
- [ ] P25 trunking: show site and system in monitor, not just the -v log.
- [ ] The noise estimate's quickselect (src/n0.c) is ~4% of radiod on the
      UHF load; its second pass could be a linear scan (upstream code).
