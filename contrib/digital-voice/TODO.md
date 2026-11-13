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
      folders, attributes and call log. One table for all systems: to keep
      two systems' numbers apart radiod would have to send the WACN and
      system ID.
- [ ] P25 Phase 2: about 0.4 s of an encrypted call can play before the ESS
      says it is encrypted, when the MAC_PTT is missed.
- [ ] **LTR Standard**: decode the 300 bps sub-audible data on each repeater's
      FM channel (area, go-to repeater, home repeater, group ID, free
      repeater); play the repeaters carrying chosen IDs. No control channel:
      each repeater channel carries its own status.
- [ ] **NXDN NEXEDGE trunking** (Type-C): the control channel is decoded
      (site, control channel, neighbours; checked live on the VHF system,
      RadioReference 10105, Attica site, 153.4925 MHz) and `follow = yes`
      follows voice assignments made by direct frequency assignment.
      Still to do:
      - follow a voice call live. One assignment has been received and its
        frequency confirmed, but that call lasted 0.4 s; its five frames on
        the traffic channel carried LICH values 1b 15 13 13 11 (RF type 0
        with the functional types of a traffic channel) and neither their
        SACCH nor FACCH1 decoded, where the site's data bursts there carry
        the expected 29 (RTCH). Needs a longer call's recording;
      - sites that assign channel numbers: a channel plan option;
      - the UHF system (RadioReference 8099) and the NXDN96 one;
      - Type-D (no control channel) if a local system uses it.
- [ ] Talkgroup names per system: `talkgroups.d` is one table and is used
      for P25 calls only, since the NXDN and P25 systems here reuse the
      same small numbers. radiod should say which system a call is on.
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

- [ ] A master frequency list for non-trunked analog and digital systems:
      one CSV that both names the channels in monitor (as `repeaters.d`
      does for RepeaterBook data) and generates the channel definitions for
      a radiod conf file.

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
