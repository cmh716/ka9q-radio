# Digital voice decoder tests

```
./fetch.sh            # once: the test recordings (about 45 MB, into data/)
./run.sh              # build, decode every case, compare with expected/
SANITIZE=1 ./run.sh   # the same under AddressSanitizer and UBSan
./run.sh --bless      # accept a deliberate change of output
```

`make test` in the library directory runs `run.sh`.

Before replaying recordings, the runner tests radiod's PCM queue and call
tracking, including a talkgroup change with old speech still queued,
immediate same-talker re-keying after a terminator, brief fades, and late
identification. It also tests DV_AUTO with several calls and a terminator
inside one IQ block, checking the metadata seen in each PCM callback.
The DMR recording is additionally decoded as one large block, directly and
through DV_AUTO, to verify that its three same-talker key-ups have distinct
call serials while every PCM frame retains its original caller IDs.

The recordings are public captures from the Trunk Recorder Pro regression corpus
([trunkrecorder.pro/corpus](https://trunkrecorder.pro/corpus)), fetched by SHA-256 and
resampled from 37.5 to 48 kHz (python3 with numpy and scipy):

| recording | what it is |
|---|---|
| voice-p25 | P25 Phase 1 voice channel, TG 2501 |
| voice-p25-phase2 | P25 Phase 2 voice channel: TG 2197 clear on slot 0, TG 11033 encrypted on slot 1 |
| voice-dmr | DMR, three key-ups from 4477055 to TG 44772 |
| control-p25 | P25 control channel (CQPSK), WACN BEE00 system 445 |

Each case runs `dvtest` (see the top of `dvtest.c`), which prints every change of the
call a listener or recorder would see (mode, slot, source, destination, encryption),
control channel events and closing counts; `run.sh` adds a hash of the speech.

The cases cover decoding, call endings (a fade to silence mid-call, or
`dv_signal_lost()`), encrypted calls that must play nothing, and noise. Expected
output is from x86-64 gcc builds; other compilers or platforms may round differently,
so a mismatch there is a reason to look, not necessarily a bug.

`ubsan.supp` exempts only the IMBE vocoder's deliberate fixed-point wraparound.

End-to-end tests of radiod and pcmrecord can play any of these recordings through
radiod with the `iq_file` front end (see the top of `src/iq_file.c`).
