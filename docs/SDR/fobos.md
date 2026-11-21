# RigExpert Fobos SDR

## Description

The [Fobos SDR](https://rigexpert.com/software-defined-radio-sdr/fobos-sdr/) is a 100 kHz to 6 GHz SDR with 50 MHz bandwidth and 14-bit signal sampling resolution. The ADC can sample up to 80 Msps.

## SW Installation

You can use **ka9q-radio** with the Fobos SDR, but support is currently optional because `libfobos`, required to use the device, is not yet available as a Debian Linux package. It must be separately downloaded and installed.

Fobos is currently only supported on Linux, but MacOS Fobos could be added if `radiod` running on MacOS is fully functional. I've successfully run it on MacOS but had multicast issues.

Install [libfobos](https://github.com/rigexpert/libfobos) using the documented procedure.

The only change to these steps are the addition of running `ldconfig` to refresh the library cache.

```
sudo apt -y install cmake git
git clone https://github.com/rigexpert/libfobos.git
cd libfobos
mkdir build
cd build
cmake ..
make
sudo make install
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
```

Build and install **ka9q-radio** using the normal documented procedures in [INSTALL.md](/docs/INSTALL.md).

If `libfobos` was properly installed, **ka9q-radio** will automatically build with Fobos support. If you've already installed **ka9q-radio**, just re-run make and the *fobos.so* driver should build automatically.

Run these commands to modify the udev rule that was included with libfobos to include the `plugdev` group.

```
sudo sed -i 's/TAG+="uaccess"/GROUP="plugdev"/' /etc/udev/rules.d/fobos-sdr.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Configuration

See below for an basic example.

```
[global]
hardware = fobos
status = fobos.local

[fobos]
device = fobos
description = "My Fobos SDR"
```

You can also reference the [generic config file](/config/examples/radiod@fobos-generic.conf).

Multiple instances of `radiod` can run on the same system, subject to your CPU and USB limits.

The "description" parameter is advertised with mDNS (multicast DNS) service discovery on the LAN and this constrains its content. It should be 63 characters or less and not contain slashes ('/') or control characters (spaces are ok).

### device (mandatory)

In the example above, the `hardware` entry in the `[global]` section specifies the section containing SDR configuration information. In this example the name of the hardware section happens to be the same as the device type, but it is not essential.

The `device` key is mandatory.

### serial (optional)

If not specified, `radiod` uses the first device discovered. Since this is probably not what you want, you should explicitly specify the serial number if more than one is present.

The `serial` must exactly match the SDR serial number, in hex (the leading 0x is optional).

### samprate (optional)

Double, default is the highest speed advertised by the device, usually 80 MHz.

### clk_source (optional)

Integer, default 0 (internal 1). If 1 use external CLKIN (untested).

### direct_sampling (optional)

Boolean, default false (RF Input). If true, direct sampling on HF1 and/or HF2 inputs.

### hf_input (optional)

Integer, default 0. HF input selection in direct sampling mode: 0 -> I/Q; 1 -> HF1 only; 2 -> HF2 only.

### lna_gain (optional)

Integer, default 0. If 0 or 1 -> 0 dB; 2 -> +16 dB; 3 -> +33 dB (disregarded in direct sample mode).

### vga_gain (optional)

Integer, default 0. 0 to +62 dB in 2 dB steps (disregarded in direct sample mode).

The total gain is the LNA's plus the VGA's: `lna_gain = 2` and `vga_gain = 2` is
20 dB. Too much gain overloads the A/D, which puts spurs and noise on every
channel in the passband. radiod counts A/D samples at full scale (the A/D
overrange count in status, as `control` shows it) and, when there are any,
logs how many once a minute.

### agc (optional)

Boolean, default false (ignored in direct sample mode). Protects the A/D from
overload by moving `vga_gain`; `lna_gain` stays as set. When samples reach full scale, or the level goes above
`agc-high-threshold`, the gain comes down at once, at least 6 dB and enough to
put the level 6 dB below the threshold. After `agc-hold` seconds without
either, it goes back up 2 dB every 5 seconds, while that keeps the level 6 dB
below the threshold, but never above `agc-max-gain`. Each cut is logged; the
steps back up are logged with `-v`. A gain command (eg from `control`) turns
the AGC off.

With the LNA fixed the AGC has the VGA's range: `lna_gain = 2` gives 16 to
78 dB. So set the highest gain you want (eg `vga_gain = 15`, 46 dB) and let the
AGC take it down when it must, rather than the lowest that never overloads.

It sees only the A/D. Overload in the analog stages before it, such as
intermodulation from a strong signal outside the passband, is not detected:
if it persists, set a lower gain or `agc-max-gain`.

Samples already queued in the driver when the gain changes were taken at the
old gain, so samples at full scale among them don't count toward another cut
(otherwise nearly every cut would bring a second). The driver can't tell
exactly which queued samples came before the change: it leaves out as many
as its buffers hold, 22 ms at 50 MHz sample rate (139 ms at 8 MHz). A burst
of overload shorter than that, just after a cut, can go unanswered, if the
average level stays below `agc-high-threshold`. One strong signal anywhere
in the passband lowers the gain, and the sensitivity, for every channel until
it has been gone for `agc-hold` seconds.

### agc-max-gain (optional)

Decimal dB, total gain, default the gain set by `lna_gain` and `vga_gain`. The
AGC never goes above it (the highest VGA step at or below it), so by default it
only ever takes gain away. Set it higher to let the AGC add gain when the band
is quiet.

### agc-min-gain (optional)

Decimal dB, total gain, default 0. The AGC never goes below it (the lowest VGA
step at or above it), nor below the LNA's gain with the VGA at 0.

If the configured `vga_gain` is outside these limits, radiod starts at the
nearest step inside them. If they leave no VGA step at all (eg `agc-max-gain`
below the LNA's gain), radiod says so and leaves the AGC off.

### agc-high-threshold (optional)

Decimal dBFS, -80 to below 0, default -15. The average A/D level above which
the gain comes down. Many signals together have peaks well above their average, so the level
needs room below 0 dBFS.

### agc-hold (optional)

Decimal seconds, default 30. How long without overload before the gain starts
going back up.

Example, for a 50 MHz UHF tuning:

```
[fobos]
device = fobos
frequency = 444m6
samprate = 50000000
lna_gain = 2
vga_gain = 15
agc = yes
```

starts at 46 dB, and the AGC goes as low as 16 dB when it must.
