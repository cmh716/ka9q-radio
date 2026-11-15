#!/usr/bin/env python3
# rr2radiod: turn radioreference.com conventional frequency downloads and
# repeaterbook.com exports (CSV) into radiod channel sections, and a names
# file for monitor.
#
#   rr2radiod.py --base uhf.conf -o uhf-all.conf -n names.csv ctid_1839.csv RB_export.csv
#
# Several files are pooled: a frequency two of them list the same way is one
# channel; where they disagree (FM in one, DMR in the other) it becomes a
# "mixed" channel, which is right whichever of them is. A repeaterbook.com
# export (recognised by its "Output Freq" column) goes in the category Ham.
#
# The front end's tuning and sample rate are read from the base config (its
# [global] hardware section), or given with --center and --samprate; only
# frequencies the front end covers are used. The output is the base config
# followed by the generated sections (or the sections alone without --base).
#
# radiod also takes a directory of *.conf files. With --base naming one, only
# the generated sections are written, to go in it as a file of their own:
#
#   rr2radiod.py --base ~/uhf.d -o ~/uhf.d/50-conventional.conf ctid_1839.csv
#
# Each channel gets the mode for its listing (FM/FMN, DMR, P25, NXDN, YSF,
# D-STAR), its CTCSS tone, DCS code, P25 NAC or DMR slot, and an output
# stream named after its category (the Tag column: fire-dispatch.local ...).
# A frequency listed as analog without a tone and as digital too becomes one
# "mixed" channel. A comment says what each frequency is.
# Encrypted channels, trunked system frequencies (tag TRS) and deprecated
# entries are left out, as are frequencies the base config already has.
#
# --overrides names a CSV of your own corrections, applied on top of the
# downloads (which are then never edited, and can be downloaded afresh):
#
#   Frequency,Mode,Enabled,Name,Description,Category,Tone,Set Mode,Notes
#   453.300,,no,,,,,,constant data, not voice
#   444.200,FM,no,,,,,,only ever DMR here
#   460.550,,,TCA Disp,Twin City Ambulance dispatch,EMS Dispatch,,,
#   442.325,,,,,,none,,the listed tone is wrong
#   446.000,FM,,Simplex,National simplex calling,Ham,,,my own entry
#
# A line applies to every entry on its frequency (MHz), or with Mode (FM,
# DMR, P25, NXDN, NXDN48, YSF, D-STAR) only to those. Blank columns leave
# the download as it is. Enabled = no leaves the entry out; Name,
# Description and Category replace the alpha tag, description and Tag;
# Tone is a CTCSS frequency, a DCS code (D411), a P25 NAC (293 NAC) or
# "none"; Set Mode corrects the mode. Notes go into the config's comments.
# A line for a frequency no download has, with a Mode, adds an entry. Lines
# that match nothing are reported.
#
# The names file goes in /var/lib/ka9q-radio/repeaters.d (any name ending
# .csv); monitor then shows each channel's alpha tag and description.
#
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse
import collections
import csv
import os
import re
import sys

# RadioReference mode -> radiod preset. Analog voice is pre-emphasised: pm.
# RadioReference's "NXDN" is the 12.5 kHz (9600 bit/s) kind, "NXDN48" the
# 6.25 kHz one; radiod's nxdn is the latter and nxdn96 the former
MODES = {
    'FM': 'pm', 'FMN': 'npm', 'DMR': 'dmr', 'P25': 'p25', 'NXDN': 'nxdn96',
    'NXDN48': 'nxdn', 'YSF': 'ysf', 'D-STAR': 'dstar', 'DSTAR': 'dstar', 'LTR': 'ltr',
}
ANALOG = ('pm', 'npm')
HALF_CHANNEL = 8000.0  # Hz kept clear of the edge of the front end's coverage


def parse_freq(text):
    """radiod's notation: 444m6, 50000000, 162k5, 146.52m"""
    text = text.strip().strip('"').lower()
    m = re.fullmatch(r'(\d*)([kmg])(\d*)', text)
    if m:
        scale = {'k': 1e3, 'm': 1e6, 'g': 1e9}[m.group(2)]
        return float((m.group(1) or '0') + '.' + (m.group(3) or '0')) * scale
    m = re.fullmatch(r'([\d.]+)([kmg]?)', text)
    if not m:
        raise ValueError(text)
    return float(m.group(1)) * {'': 1.0, 'k': 1e3, 'm': 1e6, 'g': 1e9}[m.group(2)]


def format_freq(hz):
    mhz, rest = divmod(int(round(hz)), 1000000)
    frac = ('%06d' % rest).rstrip('0')
    return '%dm%s' % (mhz, frac or '0')


def read_base(path, skip=None):
    """The base config's text, its front end's (centre, sample rate), and the
    frequencies it already has. A directory is read as radiod reads one: all
    its *.conf files in order (but for the one being written)"""
    if os.path.isdir(path):
        names = sorted(n for n in os.listdir(path) if n.endswith('.conf'))
        text = '\n'.join(open(os.path.join(path, n)).read() for n in names
                         if skip is None or os.path.abspath(os.path.join(path, n)) != os.path.abspath(skip))
    else:
        text = open(path).read()
    sections = collections.OrderedDict()
    current = None
    for line in text.splitlines():
        line = line.split('#')[0].split(';')[0].strip()
        m = re.fullmatch(r'\[(.+)\]', line)
        if m:
            current = sections.setdefault(m.group(1).strip().lower(), {})
        elif '=' in line and current is not None:
            key, value = line.split('=', 1)
            current[key.strip().lower()] = value.strip()
    centre = samprate = None
    hardware = sections.get('global', {}).get('hardware', '').strip('"').lower()
    if hardware in sections:
        try:
            centre = parse_freq(sections[hardware].get('frequency', ''))
            samprate = parse_freq(sections[hardware].get('samprate', ''))
        except ValueError:
            pass
    have = set()
    for name, keys in sections.items():
        for key, value in keys.items():
            if re.fullmatch(r'freq\d?', key):
                for token in re.split(r'[\s,]+', value.strip('"')):
                    try:
                        have.add(int(round(parse_freq(token))))
                    except ValueError:
                        pass
    return text, centre, samprate, have


def slug(text):
    return re.sub(r'-+', '-', re.sub(r'[^a-z0-9]+', '-', text.lower())).strip('-') or 'other'


def squelch(mode, tone):
    """The listing's tone column as radiod options: [(key, value)], label"""
    tone = tone.strip()
    if mode in ('pm', 'npm'):
        m = re.fullmatch(r'([\d.]+) PL', tone)
        if m:
            return [('tone', m.group(1))], 'pl' + m.group(1).replace('.', '_')
        m = re.fullmatch(r'(\d{3}) DPL', tone)
        if m:
            return [('dcs', m.group(1))], 'dcs' + m.group(1)
    elif mode == 'p25':
        m = re.fullmatch(r'([0-9A-Fa-f]{1,3}) NAC', tone)
        if m:
            return [('p25-nac', '0x' + m.group(1).lower())], 'nac' + m.group(1).lower()
    elif mode == 'dmr':
        m = re.search(r'SL (\d)', tone)
        if m:
            return [('dmr-slot', m.group(1))], 'slot' + m.group(1)
    return [], ''


def repeaterbook(row):
    """A repeaterbook.com export line as RadioReference lines: one for each
    mode the repeater has (FM, and each digital voice mode)"""
    modes = (row.get('Modes') or '').upper().replace('-', '').split()
    wanted = []
    if 'FM' in modes:
        wanted.append('FM')
    for theirs, ours in (('DMR', 'DMR'), ('DSTAR', 'D-STAR'), ('P25', 'P25'), ('FUSION', 'YSF'), ('NXDN', 'NXDN48')):
        if theirs in modes:
            wanted.append(ours)
    tone = (row.get('Downlink Tone') or '').strip()
    access = (row.get('Digital Access') or '').strip()
    call = (row.get('Call') or '').strip()
    place = (row.get('Location') or '').strip()
    out = []
    for mode in wanted:
        pl = ''
        if mode == 'FM':
            if re.fullmatch(r'[\d.]+', tone):
                pl = tone + ' PL'
            elif re.fullmatch(r'D\d{3}', tone, re.I):
                pl = tone[1:] + ' DPL'
        elif mode == 'P25' and re.fullmatch(r'[0-9A-Fa-f]{1,3}', access):
            pl = access + ' NAC'
        out.append({
            'Frequency Output': row.get('Output Freq') or '', 'Mode': mode, 'PL Output Tone': pl,
            'Alpha Tag': call, 'Description': ' '.join(x for x in (call, place) if x),
            'Agency/Category': 'RepeaterBook %s' % (row.get('County') or '').strip(), 'Tag': 'Ham',
        })
    return out


def mode_class(text):
    """A mode as written in a download or an overrides file, for comparing:
    FM and FMN are one, and an encrypted mode is its clear one"""
    text = (text or '').strip().upper().replace('DSTAR', 'D-STAR')
    if text[-1:] == 'E' and text[:-1] in MODES:
        text = text[:-1]
    return 'FM' if text in ('FM', 'FMN') else text


def tone_text(text):
    """An overrides file's Tone as the downloads write it; None if it isn't one"""
    text = text.strip()
    if text.lower() in ('none', 'csq', 'off'):
        return ''
    if re.fullmatch(r'[\d.]+', text):
        return text + ' PL'
    m = re.fullmatch(r'D(\d{3})', text, re.I)
    if m:
        return m.group(1) + ' DPL'
    if re.fullmatch(r'([\d.]+ PL|\d{3} DPL|[0-9A-Fa-f]{1,3} NAC)', text):
        return text
    return None


def read_overrides(path):
    overrides = []
    with open(path, newline='', encoding='utf-8-sig') as f:
        for n, row in enumerate(csv.DictReader(f), 2):
            row = {(k or '').strip().lower(): (v or '').strip() for k, v in row.items() if k is not None}
            if not any(row.values()) or row.get('frequency', '').startswith('#'):
                continue
            try:
                hz = int(round(float(row.get('frequency', '')) * 1e6))
            except ValueError:
                sys.exit('%s line %d: no frequency in MHz' % (path, n))
            if row.get('tone') and tone_text(row['tone']) is None:
                sys.exit('%s line %d: tone "%s" (expected 141.3, D411, 293 NAC or none)' % (path, n, row['tone']))
            for column in ('mode', 'set mode'):
                if row.get(column) and mode_class(row[column]) != 'FM' and mode_class(row[column]) not in MODES:
                    sys.exit('%s line %d: mode "%s" (expected one of %s)' % (path, n, row[column], ', '.join(sorted(MODES))))
            row['_hz'], row['_line'], row['_used'] = hz, n, False
            overrides.append(row)
    return overrides


def apply_overrides(row, hz, overrides):
    """Change a download's entry as the overrides file says; False if it is disabled"""
    enabled = True
    for o in overrides:
        if o['_hz'] != hz or (o.get('mode') and mode_class(o['mode']) != mode_class(row.get('Mode'))):
            continue
        o['_used'] = True
        if o.get('enabled', '').lower() in ('no', 'n', 'off', 'false', '0'):
            enabled = False
        elif o.get('enabled'):
            enabled = True
        if o.get('name'):
            row['Alpha Tag'] = o['name']
        if o.get('description'):
            row['Description'] = o['description']
        if o.get('category'):
            row['Tag'] = o['category']
        if o.get('tone'):
            row['PL Output Tone'] = tone_text(o['tone'])
        if o.get('set mode'):
            row['Mode'] = o['set mode']
        if o.get('notes'):
            row['_note'] = o['notes']
    return enabled


def main():
    ap = argparse.ArgumentParser(description='radioreference.com and repeaterbook.com frequency CSVs to radiod channel sections')
    ap.add_argument('csv', nargs='+', help='radioreference.com conventional frequency downloads and repeaterbook.com exports')
    ap.add_argument('--base', help='radiod config, or config directory, to extend: gives the tuning and the frequencies already defined; a file is copied to the output')
    ap.add_argument('--fragment', action='store_true', help='write only the generated sections, not the base config (always so when --base is a directory)')
    ap.add_argument('--center', help='front end centre frequency (444m6), if not from --base')
    ap.add_argument('--samprate', help='front end sample rate (50000000), if not from --base')
    ap.add_argument('--coverage', type=float, default=0.47, help='usable fraction of the sample rate each side of centre (default 0.47)')
    ap.add_argument('-o', '--output', help='config to write (default: standard output)')
    ap.add_argument('-n', '--names', help='names file to write, for /var/lib/ka9q-radio/repeaters.d')
    ap.add_argument('--tags', help='only these categories (Tag column), comma separated')
    ap.add_argument('--exclude-tags', default='TRS,Deprecated', help='leave these categories out (default TRS,Deprecated)')
    ap.add_argument('--group-by', choices=['tag', 'agency', 'one'], default='tag', help='what names the output streams (default tag)')
    ap.add_argument('--prefix', default='', help='put this in front of each stream name (uhf- gives uhf-business.local)')
    ap.add_argument('--encrypted', action='store_true', help='include encrypted channels')
    ap.add_argument('--overrides', help='CSV of your own corrections to the downloads: disable, rename, re-tone, add')
    args = ap.parse_args()

    base_text, centre, samprate, have = ('', None, None, set())
    if args.base:
        base_text, centre, samprate, have = read_base(args.base, args.output)
        if args.fragment or os.path.isdir(args.base):
            base_text = ''  # the generated sections go in a file of their own
    if args.center:
        centre = parse_freq(args.center)
    if args.samprate:
        samprate = parse_freq(args.samprate)
    if centre is None or samprate is None:
        sys.exit('the front end tuning is unknown: give --base (a config with the hardware section) or --center and --samprate')
    low = centre - args.coverage * samprate + HALF_CHANNEL
    high = centre + args.coverage * samprate - HALF_CHANNEL

    only = {t.strip().lower() for t in args.tags.split(',')} if args.tags else None
    exclude = {t.strip().lower() for t in args.exclude_tags.split(',') if t.strip()}
    skipped = collections.Counter()
    # (stream, mode, squelch options) -> {frequency: [rows]}
    groups = collections.OrderedDict()
    by_freq = collections.defaultdict(list)  # for the names file
    overrides = read_overrides(args.overrides) if args.overrides else []
    listed = set()
    added = []

    def entries():
        for path in args.csv:
            with open(path, newline='', encoding='utf-8-sig') as f:
                reader = csv.DictReader(f)
                book = 'Output Freq' in (reader.fieldnames or [])
                for line in reader:
                    for row in (repeaterbook(line) if book else [line]):
                        try:
                            listed.add(int(round(float(row['Frequency Output']) * 1e6)))
                        except (KeyError, ValueError):
                            pass
                        yield row
        # The overrides file's own entries: a frequency no download has
        for o in overrides:
            if o['_hz'] not in listed and o.get('mode') and not o['_used']:
                added.append(o)
                yield {'Frequency Output': '%.6f' % (o['_hz'] / 1e6), 'Mode': o['mode'], 'Tag': 'Other',
                       'Alpha Tag': '', 'Description': '', 'Agency/Category': 'overrides', 'PL Output Tone': ''}

    for row in entries():
        try:
            hz = int(round(float(row['Frequency Output']) * 1e6))
        except (KeyError, ValueError):
            skipped['no frequency'] += 1
            continue
        if not apply_overrides(row, hz, overrides):
            skipped['disabled in the overrides file'] += 1
            continue
        if not low <= hz <= high:
            skipped['outside the front end coverage'] += 1
            continue
        tag = (row.get('Tag') or 'Other').strip()
        if tag.lower() in exclude or (only is not None and tag.lower() not in only):
            skipped['category left out (%s)' % tag] += 1
            continue
        rr_mode = (row.get('Mode') or '').strip()
        if rr_mode[-1:] in ('E', 'e') and rr_mode[:-1].upper() in MODES:
            if not args.encrypted:
                skipped['encrypted'] += 1
                continue
            rr_mode = rr_mode[:-1]
        mode = MODES.get(rr_mode.upper())
        if mode is None:
            skipped['mode %s' % (rr_mode or '?')] += 1
            continue
        if hz in have:
            skipped['already in the base config'] += 1
            continue
        options, label = squelch(mode, row.get('PL Output Tone') or '')
        stream = {'tag': tag, 'agency': row.get('Agency/Category') or tag, 'one': 'scan'}[args.group_by]
        row['_hz'], row['_mode'], row['_options'], row['_label'], row['_stream'] = hz, mode, options, label, slug(stream)
        by_freq[hz].append(row)

    # One channel per frequency and squelch. Where a frequency is listed
    # both with and without a tone (or, DMR, on both slots), the open
    # channel hears them all: the rest would only repeat it. Where it is
    # listed as analog without a tone and as digital too, an FM channel
    # would play the digital signal as noise: one "mixed" channel decodes
    # the digital and plays the analog
    for hz, rows in sorted(by_freq.items()):
        analog = [r for r in rows if r['_mode'] in ANALOG]
        digital = [r for r in rows if r['_mode'] not in ANALOG]
        if digital and any(r['_label'] == '' for r in analog):
            key = (rows[0]['_stream'], 'mixed', (), '')
            groups.setdefault(key, []).append((hz, rows))
            continue
        seen = {}
        for mode in sorted({r['_mode'] for r in rows}):
            these = [r for r in rows if r['_mode'] == mode]
            labels = {r['_label'] for r in these}
            open_squelch = '' in labels or (mode == 'dmr' and len(labels) > 1)
            for r in these:
                options, label = ([], '') if open_squelch else (r['_options'], r['_label'])
                key = (r['_stream'], mode, tuple(options), label)
                if (mode, label) in seen:
                    seen[(mode, label)].append(r)
                    continue
                seen[(mode, label)] = [r]
                groups.setdefault(key, []).append((hz, seen[(mode, label)]))

    out = []
    if base_text:
        out.append(base_text.rstrip('\n') + '\n')
    out.append('\n# Generated by rr2radiod.py from %s\n' % ', '.join(args.csv))
    out.append('# front end %s, %s samples/s: %s to %s\n' % (format_freq(centre), int(samprate), format_freq(low), format_freq(high)))
    counts = collections.Counter()
    for (stream, mode, options, label), channels in groups.items():
        name = '-'.join(x for x in (stream, mode, label) if x)
        out.append('\n[%s]\n' % name)
        out.append('mode = %s\n' % mode)
        for key, value in options:
            out.append('%s = %s\n' % (key, value))
        out.append('data = %s%s.local\n' % (args.prefix, stream))
        # What each frequency is
        said = set()
        for hz, rows in channels:
            for r in rows:
                what = (r.get('Alpha Tag') or '').strip()
                description = (r.get('Description') or '').strip()
                if description.startswith(what):
                    what = description
                elif description:
                    what += ': ' + description
                agency = (r.get('Agency/Category') or '').strip()
                if agency:
                    what += ' (%s)' % agency
                if r.get('_note'):
                    what += ' -- ' + r['_note']
                line = '# %-11s %s\n' % (format_freq(hz), what.replace('\n', ' '))
                if line not in said:
                    said.add(line)
                    out.append(line)
        freqs = [hz for hz, rows in channels]
        # radiod reads freq and freq0..freq9; keep the lines a readable length
        chunks = [freqs[i:i + 8] for i in range(0, len(freqs), 8)]
        if len(chunks) > 11:
            sys.exit('%s: too many frequencies for one section' % name)
        for i, chunk in enumerate(chunks):
            out.append('%s = "%s"\n' % ('freq' if i == 0 else 'freq%d' % (i - 1), ' '.join(format_freq(f) for f in chunk)))
        counts[args.prefix + stream + '.local'] += len(channels)
    text = ''.join(out)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(text)
    else:
        sys.stdout.write(text)

    if args.names:
        with open(args.names, 'w', newline='') as f:
            w = csv.writer(f)
            # Headers as monitor's repeater database reads them
            w.writerow(['Callsign', 'Frequency (MHz)', 'TSQ Tone', 'Landmark', 'City'])
            for hz, rows in sorted(by_freq.items()):
                def joined(column, rows):
                    values = []
                    for r in rows:
                        v = (r.get(column) or '').strip()
                        if v and v not in values:
                            values.append(v)
                    return ' / '.join(values)
                # monitor looks a channel up by frequency and the CTCSS tone
                # it hears, then by frequency alone: a line for each tone
                # where they differ, and one for everything on the frequency
                tones = collections.OrderedDict()
                for r in rows:
                    m = re.fullmatch(r'([\d.]+) PL', (r.get('PL Output Tone') or '').strip())
                    if m and r['_mode'] in ('pm', 'npm'):
                        tones.setdefault(m.group(1), []).append(r)
                if len(rows) > 1:
                    for tone, tr in tones.items():
                        w.writerow([joined('Alpha Tag', tr), '%.6f' % (hz / 1e6), tone, joined('Description', tr), joined('Agency/Category', tr)])
                w.writerow([joined('Alpha Tag', rows), '%.6f' % (hz / 1e6), '', joined('Description', rows), joined('Agency/Category', rows)])

    total = sum(counts.values())
    print('%d channels in %d sections, on %d streams:' % (total, len(groups), len(counts)), file=sys.stderr)
    for stream, n in sorted(counts.items()):
        print('  %-28s %d' % (stream, n), file=sys.stderr)
    for o in added:
        print('%s line %d: %.6f MHz %s added (no download lists it)' % (args.overrides, o['_line'], o['_hz'] / 1e6, o['mode']), file=sys.stderr)
    for o in overrides:
        if not o['_used']:
            print('%s line %d: %.6f MHz%s matches nothing in the downloads' % (
                args.overrides, o['_line'], o['_hz'] / 1e6, ' ' + o['mode'] if o.get('mode') else ''), file=sys.stderr)
    if skipped:
        print('left out:', file=sys.stderr)
        for reason, n in skipped.most_common():
            print('  %-40s %d' % (reason, n), file=sys.stderr)


if __name__ == '__main__':
    main()
