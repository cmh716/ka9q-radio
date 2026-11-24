#!/usr/bin/env python3
"""Add on-air evidence from activity's log to candidate frequencies.

  frequency-survey.py [-a activity.csv] [-o out.csv] [--range 422.1-469.1] candidates.csv

The candidates: a RadioReference CSV download ("Frequency Output", "PL
Output Tone", ...) or the frequency catalog's inbox.csv or master.csv
("RX MHz", "RX Squelch Value", ...). The output is the same rows, with
evidence added from activity's log (activity sees every carrier in its
front end's coverage, whatever its tone, with no radiod configuration):
transmissions, airtime, first and last heard, busiest hours, whether they
come at regular intervals (telemetry), and a verdict: RF activity near the
frequency, or none in this log. It is evidence of carriers only: their
tone and who they are stay unverified (candidates sharing a frequency
share its evidence). Silence means only that the log holds nothing there:
the log doesn't record when activity ran or what the receiver covered.
With --range (the front end's coverage, which you know), candidates
outside it are marked so and get no evidence. A service used rarely (fire,
EMS, mutual aid...) is flagged, so silence isn't taken against it. Nothing
is changed: decide in the catalog.

Each logged transmission counts for the nearest candidate within 5 kHz,
among those within --range.
"""
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse
import collections
import csv
import datetime
import re
import statistics
import sys

MATCH_HZ = 5000
RARE = re.compile(r'fire|ems\b|ambulance|rescue|mutual|interop|tac\b|tact|tactical|emergency|hospital|med\b|'
                  r'fireground|incident|evac|hazmat|swat|disaster|ares|races|civil def', re.I)
EVIDENCE = ['Survey', 'Transmissions', 'Airtime s', 'First heard', 'Last heard', 'Busiest hours', 'Pattern', 'Rare-use']
ACTIVE = 'RF activity near frequency (tone, identity unverified)'
SILENT = 'no RF activity in this log'
OUTSIDE = 'outside --range' 


def frequency_of(row):
    """A candidate's receive frequency in Hz, or None"""
    for key in ('RX MHz', 'Frequency Output', 'Frequency', 'RX Frequency'):
        value = (row.get(key) or '').strip()
        if value:
            try:
                mhz = float(value)
            except ValueError:
                return None
            return mhz * 1e6 if mhz > 0 else None
    return None


def rare_use(row):
    text = ' '.join(row.get(k) or '' for k in ('Service', 'Group', 'Tag', 'Agency/Category', 'Description',
                                                 'Alpha Tag', 'Name'))
    return bool(RARE.search(text))


def read_activity(path):
    """The log's transmissions: (frequency Hz, start, seconds), and the times it covers"""
    out = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                hz = float(r['frequency_mhz']) * 1e6
                start = datetime.datetime.strptime(r['start'], '%Y-%m-%d %H:%M:%S')
                end = datetime.datetime.strptime(r['end'], '%Y-%m-%d %H:%M:%S')
                secs = float(r['seconds'])
            except (KeyError, ValueError, TypeError):
                continue  # an incomplete row (activity skips them too)
            out.append((hz, start, end, secs))
    return out


def pattern(starts):
    """Regular intervals (telemetry, meter reading), or ''"""
    if len(starts) < 20:
        return ''
    starts = sorted(starts)
    gaps = [(b - a).total_seconds() for a, b in zip(starts, starts[1:])]
    gaps = [g for g in gaps if g > 0]
    if len(gaps) < 19:
        return ''
    median = statistics.median(gaps)
    # the middle half of the gaps within 25% of the median: regular
    spread = statistics.quantiles(gaps, n=4)
    if median > 0 and spread[0] >= 0.75 * median and spread[2] <= 1.25 * median:
        return f'regular, every ~{median:.0f} s'
    return ''


def busiest_hours(starts):
    if not starts:
        return ''
    count = collections.Counter(s.hour for s in starts)
    top = [h for h, _ in count.most_common(3)]
    return ' '.join(f'{h:02d}h' for h in sorted(top))


def survey(rows, transmissions, lo, hi):
    """lo, hi: the coverage given (Hz), or None"""
    cands = [(i, frequency_of(r)) for i, r in enumerate(rows)]
    inside = (lambda f: True) if lo is None else (lambda f: lo <= f <= hi)
    # Only candidates within the coverage compete for transmissions
    freqs = sorted((f, i) for i, f in cands if f is not None and inside(f))
    keys = [f for f, _ in freqs]
    heard = collections.defaultdict(list)
    import bisect
    for hz, start, end, secs in transmissions:
        k = bisect.bisect_left(keys, hz)
        best = None
        for j in (k - 1, k):
            if 0 <= j < len(keys) and abs(keys[j] - hz) <= MATCH_HZ:
                if best is None or abs(keys[j] - hz) < abs(keys[best] - hz):
                    best = j
        if best is not None:
            # (candidates at one frequency share its evidence)
            heard[keys[best]].append((start, end, secs))
    for i, f in cands:
        row = rows[i]
        for e in EVIDENCE:
            row[e] = ''  # an annotated file surveyed again: nothing of the last survey kept
        rare = rare_use(row)
        row['Rare-use'] = 'yes' if rare else ''
        if f is None:
            row['Survey'] = 'no frequency'
            continue
        if not inside(f):
            row['Survey'] = OUTSIDE
            continue
        h = heard.get(f, [])
        row['Transmissions'] = str(len(h))
        row['Airtime s'] = f'{sum(s for _, _, s in h):.0f}'
        if h:
            row['First heard'] = min(s for s, _, _ in h).strftime('%Y-%m-%d %H:%M')
            row['Last heard'] = max(e for _, e, _ in h).strftime('%Y-%m-%d %H:%M')
            row['Busiest hours'] = busiest_hours([s for s, _, _ in h])
            row['Pattern'] = pattern([s for s, _, _ in h])
            row['Survey'] = ACTIVE
        else:
            row['Survey'] = SILENT + (' (rare-use: keep by service)' if rare else '')
    return rows


def same_file(a, b):
    """Two paths naming one file (symlinks, spellings), existing or not"""
    import os
    def canon(p):
        if os.path.exists(p):
            return os.path.realpath(p)
        return os.path.join(os.path.realpath(os.path.dirname(p) or '.'), os.path.basename(p))
    return canon(a) == canon(b)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('candidates')
    ap.add_argument('-a', '--activity', default='activity.csv', help="activity's log")
    ap.add_argument('-o', '--output', help='where the annotated rows go (default: <candidates>-survey.csv)')
    ap.add_argument('--range', help='the coverage, MHz: lo-hi (default: the span of the log)')
    args = ap.parse_args(argv)

    transmissions = read_activity(args.activity)
    if not transmissions:
        sys.exit(f'{args.activity}: no transmissions logged')
    lo = hi = None
    if args.range:
        m = re.fullmatch(r'\s*([\d.]+)\s*-\s*([\d.]+)\s*', args.range)
        if not m or float(m.group(1)) >= float(m.group(2)):
            sys.exit('--range lo-hi, in MHz')
        lo, hi = float(m.group(1)) * 1e6, float(m.group(2)) * 1e6
    first = min(t[1] for t in transmissions)
    last = max(t[2] for t in transmissions)

    with open(args.candidates, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    rows = survey(rows, transmissions, lo, hi)
    out = args.output or re.sub(r'(\.csv)?$', '-survey.csv', args.candidates, count=1)
    for source in (args.candidates, args.activity):
        if same_file(out, source):
            sys.exit(f'{out}: that is {source}, an input: give another output')
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields + [e for e in EVIDENCE if e not in fields], extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    verdicts = collections.Counter(r.get('Survey', '').split(' (rare')[0] for r in rows)
    print(f'{len(rows)} candidates; the log: {len(transmissions)} transmissions, {first:%Y-%m-%d %H:%M} to '
          f'{last:%Y-%m-%d %H:%M} (not a record of how long activity ran)' +
          (f'; coverage {lo / 1e6:.3f}-{hi / 1e6:.3f} MHz' if lo is not None else '; no --range: nothing marked outside'))
    for v, n in verdicts.most_common():
        print(f'  {n:5d}  {v}')
    print(f'written to {out}')


if __name__ == '__main__':
    main()
