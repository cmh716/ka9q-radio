# Digital voice and trunking: to do

Ordered by priority within each section. Local systems: P25 Phase II
(trunked, 800 MHz), LTR Standard, NXDN NEXEDGE 9600, DMR Tier III
Capacity Max (going encrypted).

## Waiting on hardware or live testing

- [x] P25 grant following (`p25-follow`) and Phase 2 voice on the live
      system: working well with the 800 MHz Yagi (2026-11-13).
- [ ] Talkgroup names (`talkgroups.d`): live test in monitor and pcmrecord.
- [x] Control channel search (`p25-control-channels`, broadcast
      secondaries): works live (2026-11-13), started on a frequency with no
      control channel: found it on the first hop after 5 s, and the site's
      broadcast secondaries match RadioReference. Not yet seen: a real
      control channel change, and a search past several dead frequencies.
- [ ] P25 Phase 1 trunked system one county over: live test. Following
      Phase 1 (FDMA) voice grants is implemented and tested on recordings,
      but has never run on a live Phase 1 system.
- [ ] Mixed FM + digital channel (`analog = yes`): live test.
- [ ] M17: live test once an M17 radio or hotspot is available.

## Trunking and scanning (local systems)

- [x] **Call recording with metadata**: `pcmrecord --by-talkgroup`, with
      `--call-log`; radiod sends CALL_DEST/CALL_SOURCE/CALL_MODE status.
      Needs a live test with p25-follow.
- [x] Talkgroup names from CSV files (a RadioReference download) in
      `/var/lib/ka9q-radio/talkgroups.d`: monitor's display, pcmrecord's
      folders, attributes and call log.
- [ ] P25 Phase 2: about 0.4 s of an encrypted call can play before the ESS
      says it is encrypted, when the MAC_PTT is missed.
- [ ] **LTR Standard**: `mode = ltr` decodes the 300 bit/s sub-audible data
      on a repeater's FM channel (area, go-to repeater, home repeater, group,
      free repeater), plays its calls with the data filtered out and reports
      each call's talkgroup; `follow = yes` on a home repeater follows its
      talkgroups to the repeaters their calls are sent to (`ltr-repeaters`
      gives the numbers' frequencies). The decoder was written against an
      eleven minute capture of three local systems (RadioReference 4643,
      4704, 4705: nine repeaters identified, one call decoded from start to
      end). Not yet run live in radiod, and following has only its unit
      test. Still to do:
      - a live test, and of following;
      - `talkgroups` on a channel that doesn't follow;
      - words for several calls interleaved on one repeater weren't in the
        capture;
      - voice inversion (some talkgroups scramble with it).
- [ ] **NXDN NEXEDGE trunking** (Type-C): the control channel is decoded
      (site, control channel, neighbours; checked live on the VHF system,
      RadioReference 10105, Attica site, 153.4925 MHz) and `follow = yes`
      follows voice assignments made by direct frequency assignment.
      Following works live (2026-11-13): seven assignments on three
      talkgroups, all followed to the right frequency, six with clear speech
      and one muted (encrypted), including moves between frequencies within
      a conversation.
      Still to do:
      - an earlier 0.4 s call's five frames on the traffic channel carried
        LICH values 1b 15 13 13 11 and didn't decode: unexplained, not seen
        since;
      - sites that assign channel numbers: `nxdn-channels` gives the plan
        (number:frequency). Works live (2026-11-14) on the UHF system
        (RadioReference 8099, NXDN96, system 157): Attica (site 1, RAN 1,
        454.550 MHz) followed by channel number with clear speech, with
        Tonawanda (site 2, 424.700) and South Wales (site 3, 424.975)
        decoded from the same tuning (Fobos at 444.6 MHz, 50 MHz). Channel
        numbers are per site: the same number is a different frequency on
        another. Still to do: the site sent no control channel number that
        the CCH_INFO layout assumed here could read (nothing was logged);
      - Type-D (no control channel) if a local system uses it.
- [x] Talkgroup names per system: `system = name` on the channel, and that
      system's CSV files in `talkgroups.d/name/`; pcmrecord files its calls
      under `name/`. Needs a live test (NXDN and P25 side by side).
- [x] Backup control channels: on losing the control channel the follower
      searches the listed and broadcast control channel frequencies. The
      search (a list of candidates, a wait, a dwell) isn't specific to P25:
      move it out of `dv_follow.c` when a second trunking type arrives.
- [ ] Control channel search across sites (a neighbouring site's control
      channel when this one is gone): needs the band plan and NAC reset.
- [ ] **Two-tone paging (fire/EMS) detection**: recognise a department's
      two-tone sequential page on an analog (or decoded) channel; log it,
      and use it to start or tag a recording or run a command.
- [ ] **P25 multi-call following** (`p25-voice-channels = N`): one
      control-channel decode, N tuners, one output stream each.
- [ ] P25: decode multi-block trunking messages (PDU, DUID 12), which some
      systems use for explicit and Phase 2 grants.
- [ ] DMR Tier III Capacity Max: parked, since the system is going encrypted.

- [x] Carrier gate (`dv-gate`) and mode lock for DV_AUTO / mixed: idle
      channels near zero CPU. Live test on a mixed repeater; tune the 0 dB /
      -3 dB thresholds.

- [x] Vocoder synthesis: JMBE (SDRTrunk) vs mbelib, blind A/B on five error-free
      Phase 2 calls from the live system (2026-11-12): no audible difference.
      Not worth porting without new evidence.

## Conventional (non-trunked) systems

- [ ] Recording: optionally notch the CTCSS tone out of analog recordings
      (pcmrecord or radiod option), as monitor does for playback.
- [ ] Even out digital voice levels: decoded audio from different modes,
      radios and talkers varies in loudness; normalise it (optionally).

- [x] A frequency list for non-trunked analog and digital systems:
      `contrib/rr2radiod.py` turns radioreference.com conventional
      frequency downloads and repeaterbook.com exports into radiod channel
      sections (mode, CTCSS/DCS/NAC/DMR slot, a stream per category, only
      what the front end covers, no encrypted channels) and a names file
      for monitor's `repeaters.d`. The files given are pooled by frequency
      (a frequency one calls FM and another digital becomes a `mixed`
      channel); `--overrides` is a CSV of one's own corrections (disable,
      rename, re-tone, add). Run in radiod with 62 generated channels; the
      names in monitor, the overrides and a config directory as the base
      are unchecked live.
- [ ] A master frequency list kept outside this repository (the
      RadioReference data is licensed for personal use: a private one),
      from which each device's files are generated. The suggested layout:

      ```
      radio-lists/
        downloads/            ctid_*.csv, RB_*.csv, trs_tg_*.csv,
                              trs_sites_*.csv: as downloaded, never edited
        freq-overrides.csv    one's own corrections (rr2radiod.py --overrides)
        uhf.d/                a radiod config directory for one tuning
          00-hardware.conf      [global] and the front end: by hand
          10-trunked.conf       the trunked systems' following channels,
                                with their channel plans and repeater
                                numbers: by hand
          50-conventional.conf  everything from the downloads: generated
        vhf.d/                the same three files for the VHF tuning
        names/                uhf-names.csv ...: generated, for repeaters.d
      ```

      The generated files are committed too, so that after a fresh download
      `git diff` shows which channels appeared, went or changed before
      anything is installed. Generating into a directory:

      ```
      contrib/rr2radiod.py --base radio-lists/uhf.d \
        --overrides radio-lists/freq-overrides.csv \
        -o radio-lists/uhf.d/50-conventional.conf \
        -n radio-lists/names/uhf-names.csv \
        radio-lists/downloads/ctid_*.csv radio-lists/downloads/RB_*.csv
      ```

      Trunked systems stay hand-written in `10-trunked.conf` for now: it
      holds what no download has (the NXDN channel numbers and LTR repeater
      numbers found by listening), and there are few of them. Still to do:
      - split the script into importers (RadioReference, RepeaterBook) that
        write the master files, and exporters that read them;
      - an exporter for CHIRP's CSV, for the Icom rigs (ham rows only, with
        the input frequency and uplink tone);
      - an exporter for the Uniden SDS scanners' favourites lists, once one
        is at hand to test against;
      - an overrides file for talkgroups (rename, disable), and generating
        the trunked sections from a device-neutral description of each
        system when a second device needs them.

## Amateur radio

- [ ] **FreeDV 700D, 700E, 1600** on HF through libcodec2's FreeDV API
      (already linked): a mode fed by an SSB channel. Test on air (e.g.
      14.236 MHz) and with codec2's freedv_tx.
- [ ] FreeDV RADE (neural, FreeDV 2.0): not in libcodec2; investigate after
      the classic modes.
- [ ] Text and data from ham digital voice, shown in monitor:
      D-STAR slow-data messages, DMR talker alias, YSF and M17 text.
- [ ] GPS positions carried in D-STAR, DMR and YSF transmissions.

## User interface

- [ ] A better TUI for monitor: a layout meant for scanning (state, current
      and recent calls, names), not just the wide table.

## Housekeeping

- [x] Test suite: `tests/` (public corpus recordings, call-ending cases,
      sanitizers) and the `iq_file` radiod front end for end-to-end runs.
- [ ] More test recordings: YSF, D-STAR, NXDN, M17 and a mixed FM/digital
      repeater; the user's own captures if he's willing to publish them.

- [ ] Merge `digital` to main (on hold).
- [ ] P25 trunking: show site and system in monitor, not just the -v log.
