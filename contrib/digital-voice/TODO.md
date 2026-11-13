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
- [ ] **NXDN NEXEDGE trunking**: first check on RadioReference whether the
      system is Type-C (a control channel, like P25) or Type-D (distributed,
      each repeater announces its own traffic). Then decode its control
      messages and follow calls with the existing NXDN96 receiver.
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
