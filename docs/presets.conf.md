# *presets.conf* Summary

The following presets are defined in the current version of */usr/local/share/ka9q-radio/presets.conf*.

See the file itself for complete details.

You may add or delete entries in this file as needed, but be careful as it will be overwritten by the next `make install`.

| name      | demod     | BW [Hz] | description |
|-----------|-----------|---------|-------------|
| pm        | fm        |  16.0k  | amateur radio narrowband FM, 6dB/octave de-emphasis, use for voice modes |
| npm       | fm        |  12.5k  | same as above |
| fm        | fm        |  16.0k  | amateur radio "true" FM, no de-emphasis, use for data modes |
| nfm       | fm        |  12.5k  | same as above |
| wfm       | wfm       | 220.0k  | commercial broadcast FM, with North America de-emphasisis, audioforced to 48k, stereo |
| am        | linear    |  10.0k  | envelope detected AM, passes DC |
| sam       | linear    |  10.0k  | coherent AM with carrier tracking |
| ame       | linear    |   5.1k  | coherent AM with carrier tracking, USB only |
| iq        | linear    |  10.0k  | raw I/Q output, usr for extra processing |
| cwu       | linear    |   0.4k  | CW on USB voice bands, same tone pitch as USB but with narrow filter centered at 500 Hz |
| cwl       | linear    |   0.4k  | CW on LSB voice bands, same tone pitch as LSB but with narrow filter centered at 500 Hz ||
| usb       | linear    |   3.05k | amateur radio USB |
| lsb       | linear    |   3.05k | amateur radio LSB |
| dsb       | linear    |  10.0k  | amateur radio DSB-SC, uses PLL squaring to recover carrier |
| amsq      | linear    |   6.0k  | envelope detected AM with carrier squelch |
| wspr      | linear    |   3.05k | same as **usb** but with AGC disabled |
| spectrum  | spectrum  |   NA    | experimental |
| nam       | linear    |   6.0k  | narrow (completely flat) AM, passes DC|

## Optional CTCSS output notch

Set `ctcss-notch` to a tone frequency in Hz in an FM channel section. It defaults
to `0` (disabled); supported frequencies are 60–260 Hz. For example:

```ini
[repeater-audio]
mode = pm
freq = 146m52
data = ham.local
ctcss-notch = 123.0
```

This removes a fixed tone from outgoing audio before PCM or Opus encoding. It
works with `fm`, `nfm`, `pm`, `npm`, and the analog FM portion of `mixed`; it does
not filter digital voice or wideband broadcast FM. Each section has one notch
frequency, so put channels requiring different tones in separate sections.
The example frequency and tone are illustrative; choose the tone actually used.

The setting is independent of `tone`/`pl`/`ctcss` receive squelch. Specifying a
notch does not enable tone squelch; when both are set, squelch detection runs
before the notch. Use `ctcss-notch = 0` to disable an inherited setting.

Every listener and recording of that stream receives the filtered audio.
Monitor's audio-based detector may no longer report the removed tone. To retain
an unfiltered recording, leave the option off or provide a separate unfiltered
channel/stream. pcmrecord still saves received Opus packets without re-encoding.
Keep personal configuration outside generated files that rr2radiod replaces;
the catalog/exporter generates this option from the `CTCSS Notch Hz` column.
