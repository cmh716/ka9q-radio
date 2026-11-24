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

**-l, --location**

> where you are, as lat,lon (`42.835,-78.607`) or a locator (`FN02qu`); also
> `location` in the `[display]` section of the config file. Positions heard
> are then shown with their distance and bearing from here, and of the
> repeaters.d entries for one frequency and tone the nearest is used.

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
| S                       | toggle the receiver summary |
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
the call, else their ID, then any D-STAR text message or the way a YSF call came, via UPLINK/DOWNLINK), and on a trunked channel the voice frequency being
followed (TG110 463.7250 1757). Between calls a trunked channel shows its state:
CC R2 / CC N469 while its control channel is received, no CC when it is lost,
CC search while other control channels are tried, LTR R1 for an LTR home
repeater. Last is the time since the channel last sent audio (now, 4:07, 3h12,
2d). After a call ends it stays in Call, fading (normal for 30 seconds, then dim),
until another call replaces it or `call-hold` seconds have passed (default 600,
in the `[display]` section of the config file); then a trunked channel's state
shows again. Call is as wide as the terminal allows, up to 64 columns.
Terminals narrower than 100 columns leave out Config, SNR and Gain, and the
followed frequency in Call.

A D-STAR or YSF call that sends its position (D-STAR's D-PRS or NMEA GPS in
the slow data, the GPS message of a YSF radio such as an FT-3D)
shows the sender's locator after their callsign in Call (CQCQCQ KD2TYY
FN02qu).

Larger terminals show caller, destination, timeslot, signal, and packet details
for the selected channel, and the last position heard on it: the sender, the
locator and latitude,longitude, the distance and bearing with --location, and
how long since that position came once the call has ended.

radiod sends the positions it decodes on its status stream, as it does the
callsigns and the audio: they are what the stations broadcast, and D-PRS
normally reaches the APRS network too. --location is only monitor's own point
of reference; it is not sent anywhere. Smaller terminals use a compact table. Press `v` to
switch between the dashboard and full diagnostics; `h` opens keyboard help.

Above the channels (terminals of 22 lines or more; `S` hides or shows it) the
receiver summary has a line for each front end: its description and LO
frequency, IF level in dBFS with the highest of the last ten minutes and how
long ago it was, gain (and AGC), A/D overloads since radiod started, in the
last ten minutes, and how long since the last (in yellow while that was
within ten minutes), and how old these figures are: they come with the
channels' status, which radiod sends while a channel is active. Below it,
the three channels with the most audio in the last ten minutes, and how
much:

    fobos 445.6000 MHz | IF -31.5 dBFS (peak -18.2, 3m ago) | gain 22 dB | A/D over 0 | 1s ago
    Busiest, 10 min: 424.70000 110 2:10 | 453.50000 0:45 | 454.55000 4 0:30
