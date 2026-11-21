# monitor

`monitor` is the **Multicast Audio Monitor** utility integrated in **ka9q-radio** SW.

```
monitor [OPTIONS]... <MCAST_IP>
```

## Description

This will start a CLI where you can see details about every monitored session (freq, mode, SNR, ...), and hear the audio output.

By default the audio for all active sessions will be played on your default audio device.

Depending on the configuration it is OK for `monitor` to start without audio and sessions.

A session does not appear in the list until it's active (audio is decoded).

### CLI Options

**-I, --input**

> multicast IP to monitor. Can be specified multiple.times to monitor multiple sources.

**-L, --list-audio**

> list audio devices and exit.

**-R, --device**

> output to audio device, default use PortAudio.

**-S, --autosort**

> enable auto-sorting sessions for TUI, default false.

**-c, --channels**

> audio channels, default 2.

**-f, --config**

> config file path.

**-g, --gain**

> audio gain, default 0 (no gain).

**-n, --notch**

> enable CTCSS tone notching, default false. CTCSS/DCS detection and display work independently of this option.

**-o, --source**

> select only one source from a multicast group.

**-P, --pipe**

> output to named pipe.

**-p, --playout**

> playout delay, default 100.

**-q, --quiet**

> do not display TUI, default false.

**-r, --samprate**

> audio sample rate, default 48000.

**-s, --voting**

> enable voting (automatic selection of repeater), default false.

**-u, --update**

> update interval, default 0.

**-v, --verbose**

> show more messages, default false.

**-V, --version**

> show version and exit.

### Runtime Commands

| keybind                 | description  |
|-------------------------|--------------|
| Q                       | quit program |
| ↑, ↓ (arrow up/down)    | select prev/next session |
| ⤒, ⤓ (home/end)         | select first/last session |
| ⇞, ⇟ (page up/down)     | select prev/next session page |
| d                       | delete session |
| r                       | reset playout buffer |
| R                       | reset all playout buffers |
| m                       | mute current session |
| M                       | mute all sessions |
| u                       | unmute current session |
| U                       | unmute all sessions |
| f                       | turn off PL notch |
| F                       | turn off PL notch, all sessions |
| n                       | turn on PL notch |
| N                       | turn on PL notch, all sessions |
| A                       | toggle, start all future sessions muted |
| C                       | toggle, constant playout delay |
| s                       | sort sessions by most recently active |
| S                       | autosort sessions by most recently active |
| t                       | sort sessions by most active |
| -, +                    | volume -1/+1 dB |
| ←, → (arrow left/right) | stereo position left/right |
| SHIFT + ← (arrow left)  | playout buffer -1 ms |
| SHIFT + → (arrow right) | playout buffer +1 ms |
| v                       | toggle verbose display |
| q                       | toggle quiet mode |

### IDs

If you want to show IDs for your `monitor` sessions you have to create (or edit) the file */usr/local/share/ka9q-radio/id.txt*.

The format of this file is:

```
freq1 [tone] ID of freq1
freq2 [tone] ID of freq2
```

Where **freq** is the frequency in Hz, **tone** is the optional CTCSS/PL tone in Hz, and the rest of the line is the ID.

### Terminal dashboard

The default display has ASCII frames, optional colors, a highlighted selection,
and a shortcut bar. Green indicates recent audio, yellow indicates recent audio
below 6 dB SNR, and cyan indicates a reported digital mode. Idle or muted channels
are dimmed. Config and Detect distinguish the configured demodulator from the
reported voice mode; a dash means no voice mode is reported. Frequencies are in MHz.
The Tone column shows detected CTCSS in Hz or DCS as D023/I023 (normal/inverted).
Detection also works during analog FM calls on mixed channels, with notching off.
Digital voice is excluded to avoid false tone detections. On P25 channels it
shows the NAC last received (N293; a following channel's is its control channel's).

The Call column shows the call in progress: talkgroup (TG2000), the radio
a private call is to (U2000) or the destination callsign, the talker (their DMR talker alias once it has come, a few seconds into
the call, else their ID), and on a trunked channel the voice frequency being
followed (TG110 463.7250 1757). Between calls a trunked channel shows its state:
CC R2 / CC N469 while its control channel is received, no CC when it is lost,
CC search while other control channels are tried, LTR R1 for an LTR home
repeater. Last is the time since the channel last sent audio (now, 4:07, 3h12,
2d). Terminals narrower than 100 columns leave out Config, SNR and Gain, and the
followed frequency in Call.

Larger terminals show caller, destination, timeslot, signal, and packet details
for the selected channel. Smaller terminals use a compact table. Press `v` to
switch between the dashboard and full diagnostics; `h` opens keyboard help.
