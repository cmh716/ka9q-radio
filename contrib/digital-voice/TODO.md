# Digital voice and trunking: to do

Ordered by priority within each section. Local systems: P25 Phase II
(trunked, 800 MHz), LTR Standard, NXDN NEXEDGE 9600, DMR Tier III
Capacity Max (going encrypted).

## Waiting on hardware or live testing

- [ ] P25 grant following (`p25-follow`): first live test, after the new
      800 MHz antenna. Tune the 2 s hang and 3 s listen timeouts.
- [ ] P25 Phase 2 voice quality on the live system (simulcast fading).
- [ ] Mixed FM + digital channel (`analog = yes`): live test.
- [ ] M17: live test once an M17 radio or hotspot is available.

## Trunking and scanning (local systems)

- [x] **Call recording with metadata**: `pcmrecord --by-talkgroup`, with
      `--call-log`; radiod sends CALL_DEST/CALL_SOURCE/CALL_MODE status.
      Needs a live test with p25-follow.
- [ ] Recording: talkgroup names from a list (e.g. a RadioReference CSV) for
      the folder names.
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
- [ ] **P25 multi-call following** (`p25-voice-channels = N`): one
      control-channel decode, N tuners, one output stream each.
- [ ] P25: decode multi-block trunking messages (PDU, DUID 12), which some
      systems use for explicit and Phase 2 grants.
- [ ] DMR Tier III Capacity Max: parked, since the system is going encrypted.

## Amateur radio

- [ ] **FreeDV 700D, 700E, 1600** on HF through libcodec2's FreeDV API
      (already linked): a mode fed by an SSB channel. Test on air (e.g.
      14.236 MHz) and with codec2's freedv_tx.
- [ ] FreeDV RADE (neural, FreeDV 2.0): not in libcodec2; investigate after
      the classic modes.
- [ ] Text and data from ham digital voice, shown in monitor:
      D-STAR slow-data messages, DMR talker alias, YSF and M17 text.
- [ ] GPS positions carried in D-STAR, DMR and YSF transmissions.

## Housekeeping

- [ ] Merge `digital` to main (on hold).
- [ ] P25 trunking: show site and system in monitor, not just the -v log.
