# On-air channel audit

Use a catalog profile such as `verify-uhf` for candidate channels. Assign it
in the master CSV or in the inbox before running `apply`. The audit covers
analog FM with a listed receive CTCSS tone or DCS code. Listings with the same
frequency and squelch share evidence. Distinct codes and polarities have
separate channels. When both FM and FMN specify the same squelch, the audit
uses the wider FM preset. Unsupported rows and rows outside coverage appear
in the review as `not monitored`.

Prepare a dedicated multicast stream and catalog manifest. Use the center and
sample rate of the receiver that will run the audit. The example uses an
unused local IPv4 multicast address and a 50 MS/s receiver at 445.6 MHz.

```sh
python3 ~/ka9q-radio/contrib/channel-audit.py prepare \
  --workspace ~/Frequencies/catalog --directory ~/Frequencies/uhf-audit \
  --group 239.194.70.23 --center 445.6 --samprate 50000000
```

Copy `90-channel-audit.conf` from that directory into the radiod configuration
directory, then restart radiod when convenient. Start with a small profile if
the receiver is already busy. Every distinct tone-gated receiver adds a demodulator.

Run the auditor with the `status` group from radiod's global configuration.
Use its hostname if it resolves on the logger machine, or its numeric IPv4
multicast address. If the machine has multiple network interfaces, use
`--interface` with the local IPv4 address on the radiod network. Use
`--source` with the server's IPv4 address if multiple radiod instances share
a status group.

```sh
python3 ~/ka9q-radio/contrib/channel-audit.py run \
  --directory ~/Frequencies/uhf-audit --status fobos-uhf.local
```

The listener polls status without changing channels. Advancing samples from a
matching receiver within the current passband count toward monitored time.
By default a transmission needs 0.5 s of nonzero PCM, and gaps of 0.75 s
separate transmissions. The utility writes `review.csv` about every 30 seconds
and on Ctrl-C or SIGTERM. `evidence.json` holds the accumulated evidence
across restarts. Use a separate directory for an independent survey.

Use the catalog IDs in the review to select channels for your normal profile.
`Heard with expected squelch` means the receiver passed audio with the configured
tone. Verify a community label from a station ID or recognizable audio.
`Not heard` is evidence only for the reported monitoring time. Digital calls
and discovery of unexpected tones need separate logging.
