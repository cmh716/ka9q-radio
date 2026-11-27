#!/usr/bin/env python3
"""Add on-air evidence from activity's log to candidate frequencies.

  frequency-survey.py [-a activity.csv] [-u coverage.csv] [-o out.csv] [--range 422.1-469.1] [--tones results.csv] candidates.csv
  frequency-survey.py [-a activity.csv] --unknowns out.csv list.csv ...

The candidates: a RadioReference CSV download ("Frequency Output", "PL
Output Tone", ...), a trunked system's site download (each site's
frequencies: a row each), or the frequency catalog's inbox.csv or
master.csv ("RX MHz", "RX Squelch Value", ...).

With --unknowns OUT.csv and every list you have: instead, the log's
channels heard 3 or more times that none of them has (nor a radiod channel,
activity's known column). OUT.csv: the ones worth looking up, ranked by
airtime (--shortlist, 40); OUT-all.csv: every one, with why the rest were
left out (heard under 30 times a day, the receiver's centre frequency
(from activity's coverage file: its DC spur), splatter, 433.92 MHz
devices, the input of a known output, data or telemetry, one event with a
listed frequency or a stronger unknown, beside a busier one within
12.5 kHz and one signal with it: heard while it was on, or at its level,
within its width and a fifth as often or less). Hints: regular intervals,
short bursts (data, telemetry), the band (433.92 MHz devices, 70 cm
amateur, GMRS/FRS), a repeater input 5 MHz above a listed or configured
output, splatter (mostly heard while a configured channel within 100 kHz
was on, 15 dB or more stronger: that channel named), and one event on
several frequencies (starting in the same second as others in 80% of its
seconds, all of them together: spurious emissions, or products of strong
signals in the receiver).
Each is something to look up (RadioReference's FCC search) or a list to
download.

Otherwise, the output is the candidates' rows, with
evidence added from activity's log (activity sees every carrier in its
front end's coverage, whatever its tone, with no radiod configuration):
transmissions, airtime, first and last heard, busiest hours, whether they
come at regular intervals (telemetry), and a verdict: RF activity near the
frequency, or none in this log. It is evidence of carriers only: their
tone and who they are stay unverified (candidates sharing a frequency
share its evidence). With activity's coverage file (-u; by default the
log's name with -coverage, if it's there), each candidate gets the time
its frequency was actually watched (the log from before coverage was
recorded counts as watched from its first transmission, as without one): silence is "no RF activity in N days
monitored", and a frequency never watched is "not covered by activity".
Without it, silence means only that the log holds nothing there. With
--range (the front end's coverage, which you know), candidates outside it
are marked so too. Either way they get no evidence. A service used rarely (fire,
EMS, mutual aid...) is flagged, so silence isn't taken against it. Nothing
is changed: decide in the catalog.

Each logged transmission counts for the nearest candidate within 5 kHz,
among those within --range. Logged signals wider than --max-width (50 kHz)
aren't transmissions: strong local transmitters' skirts, run together over
a megahertz or more, or impulses, whose centre may fall anywhere.
"""
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse
import collections
import csv
import datetime
import math
import re
import statistics
import sys

MATCH_HZ = 5000


def parse_time(text):
    """activity's times: UTC if marked Z, else local (rows written before):
    as aware datetimes, so spans across daylight saving keep their length"""
    if not isinstance(text, str):
        raise ValueError('no time')  # (a truncated row's missing cell)
    text = text.strip()
    if text.endswith('Z'):
        return datetime.datetime.strptime(text[:-1], '%Y-%m-%d %H:%M:%S').replace(tzinfo=datetime.timezone.utc)
    return datetime.datetime.strptime(text, '%Y-%m-%d %H:%M:%S').astimezone()
RARE = re.compile(r'fire|ems\b|ambulance|rescue|mutual|interop|tac\b|tact|tactical|emergency|hospital|med\b|'
                  r'fireground|incident|evac|hazmat|swat|disaster|ares|races|civil def', re.I)
EVIDENCE = ['Survey', 'Monitored days', 'Transmissions', 'Airtime s', 'First heard', 'Last heard', 'Busiest hours',
            'Pattern', 'Rare-use', 'Heard', 'Tone check']
ACTIVE = 'RF activity near frequency (tone, identity unverified)'
SILENT = 'no RF activity in this log'
OUTSIDE = 'outside --range'
UNCOVERED = 'not covered by activity'


def read_coverage(path):
    """activity's coverage rows: (start, end, lo Hz, hi Hz)"""
    out = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                start = parse_time(r['start'])
                end = parse_time(r['end'])
                lo, hi = float(r['lo_mhz']) * 1e6, float(r['hi_mhz']) * 1e6
            except (KeyError, ValueError, TypeError):
                continue
            if end >= start and hi > lo:
                out.append((start, end, lo, hi))
    return out


def merged_seconds(spans):
    """The time a set of (start, end) spans covers, overlaps counted once"""
    spans = sorted(spans)
    total, cur_s, cur_e = 0.0, None, None
    for s, e in spans:
        if cur_e is None or s > cur_e:
            if cur_e is not None:
                total += (cur_e - cur_s).total_seconds()
            cur_s, cur_e = s, e
        elif e > cur_e:
            cur_e = e
    if cur_e is not None:
        total += (cur_e - cur_s).total_seconds()
    return total


def monitored_seconds(f, coverage):
    """How long f was watched: the stretches covering it, overlaps merged"""
    return merged_seconds((s, e) for s, e, lo, hi in coverage if lo <= f <= hi)


SLACK = datetime.timedelta(seconds=10)  # (rows are to the second; a stretch's end is its last spectrum)


def eligible_at(coverage, when, hz):
    """Who can be credited a transmission at hz, when: a test of a
    candidate's frequency, or None if activity wasn't watching then.
    Stretches containing when (endpoints included): a candidate inside any
    of them. At a hand-off (one stretch ending in the second another begins:
    a retune, rows being to the second) which held the transmission can't be
    told: a candidate must be inside both, unless some other stretch has
    when strictly inside it and covers it. With none containing when, those
    within SLACK of it: inside all of them"""
    matches = [(s, e, lo, hi) for s, e, lo, hi in coverage if s <= when <= e and lo <= hz <= hi]
    if matches:
        ending = [(lo, hi) for s, e, lo, hi in matches if e == when]
        starting = [(lo, hi) for s, e, lo, hi in matches if s == when]
        if ending and starting:
            inner = [(lo, hi) for s, e, lo, hi in matches if s < when < e]
            pair = ending + starting
            return lambda f: any(lo <= f <= hi for lo, hi in inner) or all(lo <= f <= hi for lo, hi in pair)
        spans = [(lo, hi) for _, _, lo, hi in matches]
        return lambda f: any(lo <= f <= hi for lo, hi in spans)
    near = [(lo, hi) for s, e, lo, hi in coverage if s - SLACK <= when <= e + SLACK and lo <= hz <= hi]
    if near:
        return lambda f: all(lo <= f <= hi for lo, hi in near)
    return None


def frequency_of(row):
    """A candidate's receive frequency in Hz, or None"""
    for key in ('RX MHz', 'Frequency Output', 'Frequency', 'RX Frequency', 'Output Freq'):  # (the last: RepeaterBook)
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


SITE_USE = {'c': 'control', 'a': 'alternate control'}


def read_candidates(path):
    """A list's fields and rows. A RadioReference trunked system's site
    download ("Site Dec, ..., Frequencies" with each site's frequencies in
    the columns after it, "452.0125c" a control channel) becomes a row per
    frequency: the site's fields, its Frequency, and Channel use"""
    with open(path, newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f, restkey='_more')
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    if 'Frequencies' not in fields:
        return fields, rows
    site = [k for k in fields if k != 'Frequencies']
    out = []
    for r in rows:
        for text in [r.get('Frequencies') or ''] + list(r.get('_more') or []):
            m = re.fullmatch(r'\s*(\d+(?:\.\d+)?)([a-z]?)\s*', text or '')
            if not m:
                continue
            row = {k: r.get(k, '') for k in site}
            row['Frequency'] = f'{float(m.group(1)):.6f}'
            row['Channel use'] = SITE_USE.get(m.group(2), m.group(2))
            out.append(row)
    return site + ['Frequency', 'Channel use'], out


def name_of(row):
    for k in ('Alpha Tag', 'Name', 'Description'):
        if (row.get(k) or '').strip():
            return row[k].strip()
    return ''


GMRS_FRS = [462.5500 + 0.025 * k for k in range(8)] + [462.5625 + 0.025 * k for k in range(7)] + \
    [467.5625 + 0.025 * k for k in range(7)] + [467.5500 + 0.025 * k for k in range(8)]


def band_hint(hz):
    """What a frequency's band says of it (UHF, the US; 421-430 MHz is land
    mobile near Buffalo, Detroit and Cleveland), or ''"""
    mhz = hz / 1e6
    if abs(mhz - 433.92) <= 0.25:
        return 'near 433.92: unlicensed devices (sensors, remotes) or 70 cm amateur'
    if 420 <= mhz < 450 and not 421 <= mhz <= 430:
        return '70 cm amateur band'
    if any(abs(mhz - g) < 0.001 for g in GMRS_FRS):
        return 'GMRS/FRS'
    return ''


def input_output(hz):
    """A repeater input's output, 5 MHz below (UHF pairs: outputs 421-425,
    450-455 and 460-465 MHz, inputs 5 MHz above), or None"""
    mhz = hz / 1e6
    if 426 <= mhz <= 430 or 455 <= mhz < 460 or 465 <= mhz <= 470:
        return hz - 5000000
    return None


def unknowns(paths, activity_path, coverage=None, tuned=()):
    """coverage: activity's stretches (each frequency's monitored time);
    tuned: the stretches it recorded itself (start, end, centre Hz): the
    receiver's centre while each was heard"""
    """The log's channels no list has (within MATCH_HZ) and no radiod
    channel was on (its known column): per 6.25 kHz channel, busiest first"""
    listed = {}
    for path in paths:
        _, rows = read_candidates(path)
        for r in rows:
            f = frequency_of(r)
            if f is not None:
                listed.setdefault(round(f), name_of(r) or path)
    keys = sorted(listed)
    import bisect

    def near(hz, within=MATCH_HZ):
        k = bisect.bisect_left(keys, hz)
        best = None
        for j in (k - 1, k):
            if 0 <= j < len(keys) and abs(keys[j] - hz) <= within and (best is None or abs(keys[j] - hz) < abs(best - hz)):
                best = keys[j]
        return best
    chans = collections.defaultdict(list)
    configured = set()
    names = {}
    levels = collections.defaultdict(list)  # configured channels' transmissions: (start, end, dBFS, channel)
    by_second = collections.defaultdict(set)  # every channel starting a transmission in each second
    with open(activity_path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                ch = round(float(r['channel_mhz']) * 1e6)
                start = parse_time(r['start'])
                secs = float(r['seconds'])
                over = float(r['db_over_floor'])
                dbfs = float(r['dbfs'])
            except (KeyError, ValueError, TypeError):
                continue
            if too_wide(r):
                continue
            if (r.get('known') or '').strip():
                configured.add(ch)
                names.setdefault(ch, r['known'].strip())
                levels[ch].append((start, start + datetime.timedelta(seconds=secs), dbfs, ch))
                by_second[start].add(ch)
                continue
            by_second[start].add(ch)
            chans[ch].append((start, secs, over, dbfs, r.get('width_khz') or '', r.get('mode_guess') or ''))
    conf = sorted(configured)
    out = []
    first_heard = last_heard = None
    tuned = sorted(tuned)
    tuned_starts = [t[0] for t in tuned]
    for ch, tx in chans.items():
        if ch in configured or near(ch) is not None or len(tx) < 3:
            continue
        starts = [t[0] for t in tx]
        airtime = sum(t[1] for t in tx)
        avg = airtime / len(tx)
        hints = []
        # At the receiver's centre (its DC spur), as it was tuned when heard:
        # most of its transmissions within 12.5 kHz of the centre then
        at_centre = 0
        for t in tx:
            k = bisect.bisect_right(tuned_starts, t[0]) - 1
            if k >= 0 and tuned[k][0] <= t[0] <= tuned[k][1] and abs(ch - tuned[k][2]) <= 12500:
                at_centre += 1
        flags = {'data': False, 'input': False, 'splatter': False, 'devices': False, 'event': None,
                 'centre': at_centre > len(tx) / 2, 'known_peer': None}
        p = pattern(starts)
        if p:
            hints.append(p)
            flags['data'] = True
        if avg < 1.0:
            hints.append('short bursts: data or telemetry')
            flags['data'] = True
        b = band_hint(ch)
        if b:
            hints.append(b)
            flags['devices'] = b.startswith('near 433.92')
        # Mostly heard while a configured channel within 100 kHz was on, 15 dB
        # or more stronger (a transmitter nearby, the house's own): its
        # splatter, maybe
        side = conf[bisect.bisect_left(conf, ch - 100000):bisect.bisect_right(conf, ch + 100000)]
        if side:
            spans = sorted(x for c in side for x in levels[c])
            starts_c = [x[0] for x in spans]
            longest = max((x[1] - x[0] for x in spans), default=datetime.timedelta(0))
            beside = collections.Counter()  # by the channel that was on, and stronger
            for t in tx:
                t_end = t[0] + datetime.timedelta(seconds=t[1])
                j = bisect.bisect_right(starts_c, t_end)
                i = bisect.bisect_left(starts_c, t[0] - longest)
                sources = {x[3] for x in spans[i:j] if x[1] >= t[0] and x[2] - t[3] >= 15}
                for c in sources:  # (each credited: one transmission can sit beside two)
                    beside[c] += 1
            if beside:
                c, n = beside.most_common(1)[0]
                if n >= len(tx) / 2:
                    flags['splatter'] = True
                    hints.append(f'{100 * n // len(tx)}% heard while {c / 1e6:.5f} ({names[c]}) was on, '
                                 f'15+ dB stronger, {abs(c - ch) / 1e3:.0f} kHz away: its splatter?')
        # Starting in the same second as other channels, nearly every time:
        # one event on several frequencies (a transmitter's spurious
        # emissions, or products of strong signals in the receiver), not
        # stations of their own
        # (seconds both ways: several starts in one logged second count once)
        seconds = set(starts)
        if len(seconds) >= 10:
            together = collections.Counter()
            for s0 in seconds:
                together.update(by_second[s0] - {ch})
            group = [c for c, n in together.most_common() if n >= 0.8 * len(seconds)]
            if group:
                def label(c):
                    return f'{c / 1e6:.5f}' + (f' ({listed[c]})' if c in listed else f' ({names[c]})' if c in names else '')
                peers = ', '.join(f'{label(c)} {100 * together[c] // len(seconds)}%' for c in group[:4]) \
                    + (f' and {len(group) - 4} more' if len(group) > 4 else '')
                # One event only if they start together, all of them (those
                # not shown too), as often
                joint = sum(1 for s0 in seconds if all(c in by_second[s0] for c in group))
                if joint >= 0.8 * len(seconds):
                    flags['event'] = set(group)
                    flags['known_peer'] = next((label(c) for c in group if c in listed or c in names), None)
                    hints.append(f'starts in the same second as {peers}; all together {100 * joint // len(seconds)}%: '
                                 'one event; one of them may be a station, the rest its spurious or receiver '
                                 'products (or a linked pair)')
                else:
                    hints.append(f'starts in the same second as {peers} (each; together {100 * joint // len(seconds)}%)')
        # A repeater's input (mobiles), 5 MHz above its output
        hz = input_output(ch)
        if hz is not None:
            k = near(hz, 3125)
            if k is not None:
                hints.append(f'input of {k / 1e6:.5f} ({listed[k]})')
                flags['input'] = True
            elif any(abs(c - hz) <= 3125 for c in configured):
                hints.append(f'input of {hz / 1e6:.5f} (configured)')
                flags['input'] = True
            elif any(abs(c - hz) <= 3125 and len(chans[c]) >= 3 for c in chans):
                hints.append(f'maybe input of {hz / 1e6:.5f} (also heard, unlisted)')
        out.append({
            'frequency_mhz': f'{ch / 1e6:.5f}',
            'transmissions': len(tx),
            'airtime_s': f'{airtime:.0f}',
            'average_s': f'{avg:.1f}',
            'first': min(starts).strftime('%Y-%m-%d %H:%M %Z'),
            'last': max(starts).strftime('%Y-%m-%d %H:%M %Z'),
            'busiest_hours': busiest_hours(starts),
            'db_over_floor': f'{statistics.median(t[2] for t in tx):.0f}',
            'dbfs': f'{statistics.median(t[3] for t in tx):.0f}',
            'width_khz': collections.Counter(t[4] for t in tx).most_common(1)[0][0],
            'mode_guess': collections.Counter(t[5] for t in tx).most_common(1)[0][0],
            'hint': '; '.join(hints),
            '_flags': flags,
            '_airtime': airtime,
            '_spans': sorted((t[0], t[0] + datetime.timedelta(seconds=t[1])) for t in tx),
            '_hz': ch,
        })
        first_heard = min(first_heard, min(starts)) if first_heard else min(starts)
        last_heard = max(last_heard, max(starts)) if last_heard else max(starts)
    out.sort(key=lambda r: -r['transmissions'])
    # Each one's monitored time: from the coverage, else the log's span
    span = (last_heard - first_heard).total_seconds() if out else 0
    for r in out:
        r['_monitored'] = monitored_seconds(r['_hz'], coverage) if coverage is not None else span
    triage(out)
    return out


def overlap(a, b):
    """The share of transmissions a (sorted (start, end) spans) overlapping one of b"""
    import bisect
    if not a:
        return 0
    starts = [x[0] for x in b]
    longest = max((x[1] - x[0] for x in b), default=datetime.timedelta(0))
    n = 0
    for s0, e0 in a:
        i = bisect.bisect_left(starts, s0 - longest)
        j = bisect.bisect_right(starts, e0)
        if any(x[1] >= s0 for x in b[i:j]):
            n += 1
    return n / len(a)


def triage(rows):
    """Which unknowns are worth looking up, and why the rest aren't: a
    'skip' reason for each, and a rank (by airtime) for the others"""
    by_freq = {r['frequency_mhz']: r for r in rows}
    # What rules a frequency out by itself
    for r in rows:
        fl = r['_flags']
        least = max(10, round(30 * r['_monitored'] / 86400))  # 30 a day it was watched
        why = ''
        if r['transmissions'] < least:
            why = f'heard under {least} times'
        elif fl['centre']:
            why = "the receiver's centre frequency (its DC spur)"
        elif fl['splatter']:
            why = 'splatter'
        elif fl['devices']:
            why = '433.92 MHz devices'
        elif fl['input']:
            why = 'input of a known output'
        elif fl['data']:
            why = 'data or telemetry'
        elif fl['known_peer']:
            why = f"one event with {fl['known_peer']}"
        r['skip'] = why
    kept = []
    for r in sorted(rows, key=lambda r: -r['_airtime']):
        why = r['skip']
        fl = r['_flags']
        if not why and fl['event']:
            # One event on several frequencies: only its strongest looked up,
            # of those not ruled out already
            members = [by_freq[f'{c / 1e6:.5f}'] for c in fl['event'] if f'{c / 1e6:.5f}' in by_freq] + [r]
            members = [m for m in members if not m['skip'] or m is r]
            top = max(members, key=lambda m: (float(m['db_over_floor']), m['frequency_mhz']))
            if top is not r:
                why = f"one event with {top['frequency_mhz']} (stronger)"
        if not why:
            # Beside a busier one kept, and one signal with it, read off its
            # centre: heard while it was on (mostly); or, heard apart, at its
            # level (3 dB), a fifth as often or less, and within its width
            # (a wide signal's centre estimate wanders a channel or two)
            f = float(r['frequency_mhz'])
            for k in kept:
                sep = abs(float(k['frequency_mhz']) - f) * 1e3  # kHz
                if sep > 12.5:
                    continue
                if overlap(r['_spans'], k['_spans']) >= 0.5:
                    why = f"beside {k['frequency_mhz']}, heard while it was on (one signal, read off its centre?)"
                    break
                try:
                    width = float(k['width_khz'] or 0)
                except ValueError:
                    width = 0
                if (abs(float(k['db_over_floor']) - float(r['db_over_floor'])) <= 3
                        and k['transmissions'] >= 5 * r['transmissions'] and width >= 2 * sep):
                    why = f"beside {k['frequency_mhz']}, at its level and within its width, far less often (its signal off centre?)"
                    break
        r['skip'] = why
        if not why:
            kept.append(r)
            r['rank'] = len(kept)
        else:
            r['rank'] = ''


MAX_WIDTH_KHZ = 50  # (--max-width) wider: a broadband event, not a transmission


def too_wide(r):
    """A logged signal far wider than any channel (strong local
    transmitters' skirts, run together over a megahertz or more; impulses):
    not a transmission on the frequency at its centre"""
    try:
        return float(r.get('width_khz') or 0) > MAX_WIDTH_KHZ
    except ValueError:
        return False


def read_activity(path, wide=False):
    """The log's transmissions: (frequency Hz, start, end, seconds). wide:
    every row, each with whether it's a broadband event (evidence the
    receiver was watching, though not of any frequency): (..., wide)"""
    out = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                hz = float(r['frequency_mhz']) * 1e6
                start = parse_time(r['start'])
                end = parse_time(r['end'])
                secs = float(r['seconds'])
            except (KeyError, ValueError, TypeError):
                continue  # an incomplete row (activity skips them too)
            broad = too_wide(r)
            if wide:
                out.append((hz, start, end, secs, broad))
            elif not broad:
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
    count = collections.Counter(s.astimezone().hour for s in starts)  # (local hours, whatever the log's zone)
    top = [h for h, _ in count.most_common(3)]
    return ' '.join(f'{h:02d}h' for h in sorted(top))


def evidence(activity_path, coverage_path=None, lo=None, hi=None):
    """The log and the coverage: (every row, broadband events too, with
    their flag; the transmissions; the coverage, or None). Every row says
    the receiver was watching; only the narrower are transmissions on a
    frequency. The coverage: the given file, else the log's name with
    -coverage if it's there; the log from before it began counted as
    watched from its first transmission, across lo-hi or the log's span"""
    import os
    logged = read_activity(activity_path, wide=True) if os.path.exists(activity_path) else []
    transmissions = [t[:4] for t in logged if not t[4]]
    cover_path = coverage_path or re.sub(r'(\.csv)?$', '-coverage.csv', activity_path, count=1)
    coverage = None
    if coverage_path or os.path.exists(cover_path):
        coverage = read_coverage(cover_path)
        if not coverage:
            sys.exit(f'{cover_path}: no coverage rows')
    if not logged and coverage is None:
        sys.exit(f'{activity_path}: no transmissions logged, and no coverage file to say it was watching')
    if coverage is not None:
        began = min(s for s, _, _, _ in coverage)
        before = [(t[0], t[1]) for t in logged if t[1] < began - SLACK]
        if before:
            blo = lo if lo is not None else min(hz for hz, _ in before)
            bhi = hi if hi is not None else max(hz for hz, _ in before)
            coverage.append((min(t for _, t in before), began - datetime.timedelta(seconds=1), blo, bhi))
    return logged, transmissions, coverage


def survey(rows, transmissions, lo, hi, coverage=None):
    """lo, hi: the coverage given (Hz), or None; coverage: activity's rows, or None"""
    cands = [(i, frequency_of(r)) for i, r in enumerate(rows)]
    watched = {}
    if coverage is not None:
        for _, f in cands:
            if f is not None and f not in watched:
                watched[f] = monitored_seconds(f, coverage)
    in_range = (lambda f: True) if lo is None else (lambda f: lo <= f <= hi)
    covered = (lambda f: True) if coverage is None else (lambda f: watched.get(f, 0) > 0)
    inside = lambda f: in_range(f) and covered(f)
    # Only candidates within the coverage compete for transmissions
    freqs = sorted((f, i) for i, f in cands if f is not None and inside(f))
    keys = [f for f, _ in freqs]
    heard = collections.defaultdict(list)
    import bisect
    for hz, start, end, secs in transmissions:
        # With coverage: only a transmission heard where and when activity
        # was watching counts, and only for candidates inside that span
        ok = None
        if coverage is not None:
            ok = eligible_at(coverage, start, hz)
            if ok is None:
                continue
        k = bisect.bisect_left(keys, hz)
        best = None
        for j in (k - 1, k):
            if 0 <= j < len(keys) and abs(keys[j] - hz) <= MATCH_HZ:
                if ok is not None and not ok(keys[j]):
                    continue
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
        if not in_range(f):
            row['Survey'] = OUTSIDE
            continue
        if not covered(f):
            row['Survey'] = UNCOVERED
            continue
        days = watched[f] / 86400 if coverage is not None else None
        if days is not None:
            row['Monitored days'] = f'{days:.2f}' 
        h = heard.get(f, [])
        row['Transmissions'] = str(len(h))
        row['Airtime s'] = f'{sum(s for _, _, s in h):.0f}'
        if h:
            row['First heard'] = min(s for s, _, _ in h).strftime('%Y-%m-%d %H:%M %Z')
            row['Last heard'] = max(e for _, e, _ in h).strftime('%Y-%m-%d %H:%M %Z')
            row['Busiest hours'] = busiest_hours([s for s, _, _ in h])
            row['Pattern'] = pattern([s for s, _, _ in h])
            row['Survey'] = ACTIVE
        else:
            row['Survey'] = ((f'no RF activity in {days:.1f} days monitored' if days is not None else SILENT)
                             + (' (rare-use: keep by service)' if rare else ''))
    return rows


def listed_tone(row):
    """A candidate's listed receive tone, as tone-survey writes them: "127.3 PL",
    "D243", "none" (carrier squelch), or its text as listed, or ''"""
    kind = (row.get('RX Squelch Type') or '').strip().upper()
    value = (row.get('RX Squelch Value') or '').strip()
    if kind:  # the catalog's form
        if kind == 'CTCSS' and value:
            try:
                return f'{float(value):.1f} PL'
            except ValueError:
                return value
        if kind == 'DCS' and value:
            inverted = (row.get('RX DCS Polarity') or '').strip().lower().startswith('inv')
            return ('I' if inverted else 'D') + value.zfill(3)
        if kind in ('NONE', 'CSQ', 'CARRIER'):
            return 'none'
        return f'{kind} {value}'.strip()
    # RadioReference's, the catalog's source, RepeaterBook's (the tone the
    # repeater sends: what's heard)
    text = (row.get('PL Output Tone') or row.get('Source RX Tone') or row.get('Downlink Tone') or '').strip()
    if not text:
        return ''
    m = re.fullmatch(r'(\d+\.\d+)\s*(PL)?|(\d+)\s*PL', text, re.I)  # "127.3 PL", "127.3" (RepeaterBook)
    if m:
        return f'{float(m.group(1) or m.group(3)):.1f} PL'
    m = re.fullmatch(r'([DI])?(\d{3})\s*(N|I|DPL|DCS)?', text, re.I)
    if m:
        inverted = (m.group(1) or '').upper() == 'I' or (m.group(3) or '').upper() == 'I'
        return ('I' if inverted else 'D') + m.group(2)
    if text.upper() == 'CSQ':
        return 'none'
    return text


ENOUGH = 10    # identified transmissions before a change is proposed
SHARE = 0.6    # of those heard, the share that must agree
DIGITAL_MODES = {'P25': 'P25', 'DMR': 'DMR', 'NXDN': 'NXDN', 'YSF': 'YSF', 'DSTAR': 'D-STAR', 'M17': None}


def fingerprint(row, fields):
    """A listing as it was surveyed, every column of master.csv as the file
    holds it (frequency-catalog.py correct checks a removal against it: an
    old sheet mustn't remove a listing changed since). The same in both
    scripts"""
    import hashlib
    text = '\x1e'.join(f'{k}\x1f{(row.get(k) or "").strip()}' for k in sorted(fields))
    return 'row:' + hashlib.sha1(text.encode()).hexdigest()[:12]


UNTONED = ('none', 'voice')     # no tone: "voice" someone talking ("none" from before voice detection)
NOT_EVIDENCE = ('carrier', 'noise')  # an empty carrier, or noise: nobody's tone, for or against


def heard_kinds(heard, evidence=False):
    """tone-survey's 'heard' text: [(what, count)]. evidence: what speaks to
    a listing's tone or mode: carriers and noise left out, "voice" counted
    as "none" (untoned)"""
    out = collections.Counter() if evidence else []
    for m in re.finditer(r'(?:^|; )(.+?) x(\d+)(?=; |$)', heard or ''):
        what, n = m.group(1), int(m.group(2))
        if not evidence:
            out.append((what, n))
        elif what not in NOT_EVIDENCE:
            out['none' if what in UNTONED else what] += n
    return list(out.items()) if evidence else out


def corrections(rows, skip=None, fields=()):
    """Proposed changes to catalog rows (ID, RX Squelch Type...) from their
    survey and tone check: (ID, Name, RX MHz, Action, Field, From, To,
    Evidence). Action: change and remove (applied by frequency-catalog.py
    correct), listen (the evidence conflicts: hear it first), wait (too
    little yet). skip: a pattern of Service, Group or Name left alone"""
    out = []
    for r in rows:
        if not r.get('ID'):
            continue
        if skip and any(re.search(skip, r.get(k) or '', re.I) for k in ('Service', 'Group', 'Name')):
            continue
        base = {'ID': r['ID'], 'Name': r.get('Name', ''), 'RX MHz': r.get('RX MHz', '')}
        def add(action, field, old, new, evidence):
            out.append(dict(base, Action=action, Field=field, From=old, To=new, Evidence=evidence))
        survey = r.get('Survey', '')
        if survey.startswith('no RF activity') and r.get('Rare-use') != 'yes':
            days = float(r.get('Monitored days') or 0)
            add('remove' if days >= 2 else 'wait', '', fingerprint(r, fields), '', f'{survey} (not rare-use)')
            continue
        if survey != ACTIVE:
            continue
        kinds = heard_kinds(r.get('Heard'), evidence=True)
        if not kinds:
            continue
        total = sum(n for _, n in kinds)
        heard = r.get('Heard', '')
        mode = (r.get('RX Mode') or '').upper()
        # Digital: the mode heard most, if most of what's heard
        digital = collections.Counter()
        nacs = collections.Counter()
        for what, n in kinds:
            word = what.split()[0]
            if word in DIGITAL_MODES:
                digital[word] += n
                m = re.match(r'P25 NAC ([0-9A-F]{3})', what)
                if m:
                    nacs[m.group(1)] += n
        # (tone-survey's "other": kinds past its sixteenth, nothing to go by)
        analog = [(w, n) for w, n in kinds if w.split()[0] not in DIGITAL_MODES]
        listed = listed_tone(r)
        listed_heard = bool(listed) and listed != 'none' and any(w == listed for w, _ in kinds)
        if digital:
            word, n = digital.most_common(1)[0]
            new = DIGITAL_MODES[word]
            if mode in ('FM', 'FMN') and new:
                if n >= ENOUGH and n >= SHARE * total and listed_heard:
                    # The listed tone heard too: analog users still on it, maybe
                    add('listen', '', '', '', f'heard {word} x{n} of {total}, and the listed {listed} too: shared? {heard}')
                    continue
                if n >= ENOUGH and n >= SHARE * total:
                    add('change', 'RX Mode', r.get('RX Mode', ''), new, f'heard {word} x{n} of {total}: {heard}')
                    if (r.get('RX Squelch Type') or '') != 'Digital':
                        add('change', 'RX Squelch Type', r.get('RX Squelch Type', ''), 'Digital', 'with the mode')
                    if r.get('RX Squelch Value'):
                        add('change', 'RX Squelch Value', r['RX Squelch Value'], '', 'with the mode')
                    if r.get('RX DCS Polarity'):
                        add('change', 'RX DCS Polarity', r['RX DCS Polarity'], '', 'with the mode')
                    if r.get('CTCSS Notch Hz'):
                        add('change', 'CTCSS Notch Hz', r['CTCSS Notch Hz'], '', 'no notch for digital')
                    if new == 'P25' and nacs and not r.get('P25 NAC'):
                        # (the NAC on evidence of its own: else left blank)
                        nac, k = nacs.most_common(1)[0]
                        if k >= ENOUGH and k >= SHARE * n:
                            add('change', 'P25 NAC', '', nac, f'NAC {nac} x{k} of {n} P25')
                    continue
                if n >= 3 and n >= SHARE * total:
                    add('wait', '', '', '', f'digital {word} x{n} of {total}: {heard}')
                    continue
                # (a few digital hits among analog: the analog question below)
        # Analog: the tone heard most, if most of what's analog
        atotal = sum(n for _, n in analog)
        if not analog or mode not in ('FM', 'FMN'):
            continue
        # (the most heard of what's a value; "other", tone-survey's kinds
        # past its sixteenth, counted in atotal, never proposed)
        values = [(w, k) for w, k in analog if w != 'other']
        if not values:
            continue
        what, n = max(values, key=lambda k: k[1])
        if what == listed:
            continue
        if what == 'none':
            if listed and listed != 'none':
                add('listen' if n >= ENOUGH else 'wait', '', '', '',
                    f'listed {listed}, heard no tone x{n} of {total}: shared, or untoned paging? {heard}')
            continue
        if not re.fullmatch(r'\d+\.\d PL|[DI][0-7]{3}', what):
            continue  # (not a tone: nothing to change it to)
        if listed_heard:
            # The listed tone heard too: a shared frequency, both users maybe
            add('listen', '', '', '', f'listed {listed}, heard {what} x{n} and the listed one too: shared? {heard}')
            continue
        if n < ENOUGH or n < SHARE * atotal:
            add('wait', '', '', '', f'listed {listed or "none"}, heard {what} x{n} of {total}: {heard}')
            continue
        evidence = f'listed {listed or "none"}, heard {what} x{n} of {total}: {heard}'
        old_type, old_value, old_pol = (r.get('RX Squelch Type') or ''), (r.get('RX Squelch Value') or ''), (r.get('RX DCS Polarity') or '')
        if what.endswith(' PL'):
            tone = what[:-3]
            new_type, new_value, new_pol = 'CTCSS', tone, ''
        else:
            new_type, new_value, new_pol = 'DCS', what[1:], 'Inverted' if what[0] == 'I' else 'Normal'
        for field, old, new in (('RX Squelch Type', old_type, new_type), ('RX Squelch Value', old_value, new_value),
                                ('RX DCS Polarity', old_pol, new_pol)):
            if old != new:
                add('change', field, old, new, evidence)
                evidence = 'with the tone'
        notch = r.get('CTCSS Notch Hz') or ''
        if notch and notch not in ('0',):
            add('change', 'CTCSS Notch Hz', notch, new_value if new_type == 'CTCSS' else '', 'the notch follows the tone')
    return out


def read_tones(path):
    """tone-survey's results: frequency (Hz) -> (commonest, all heard)"""
    out = {}
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                out[round(float(r['frequency_mhz']) * 1e6)] = (r.get('commonest') or '', r.get('heard') or '')
            except (KeyError, ValueError, TypeError):
                continue
    return out


def tone_check(row, results):
    """Its listed tone against what tone-survey heard on its frequency"""
    f = frequency_of(row)
    if f is None:
        return
    found = None
    for hz, v in results.items():
        if abs(hz - f) <= 1000:
            found = v
            break
    if found is None:
        # (only a frequency with activity is audited: the rest wait for none)
        row['Tone check'] = 'not surveyed yet' if row.get('Survey') == ACTIVE else ''
        return
    _, heard = found
    row['Heard'] = heard
    listed = listed_tone(row)
    # (from what speaks to a tone: carriers and noise left out, voice untoned)
    evidence = heard_kinds(heard, evidence=True)
    if not evidence:
        row['Tone check'] = 'carrier or noise only'
        return
    commonest = max(evidence, key=lambda k: k[1])[0]
    kinds = {w for w, _ in evidence}
    if listed and commonest == listed:
        row['Tone check'] = 'confirmed'
    elif listed and listed != 'none' and listed in kinds:
        # A shared frequency: the listed tone heard, another oftener
        row['Tone check'] = f'heard, less often than {commonest}'
    elif commonest and not re.fullmatch(r'[\d.]+ PL|[DI]\d{3}|none|other', commonest):
        row['Tone check'] = f'digital: {commonest}'
    elif commonest == 'none':
        row['Tone check'] = 'no tone heard' + (f' (listed {listed})' if listed and listed != 'none' else '')
    else:
        row['Tone check'] = f'differs: heard {commonest}' + (f' (listed {listed})' if listed else ' (none listed)')


def write_csv(path, fields, rows):
    """Written to a new file beside it, then renamed over it: a reader
    (a Mac over Samba, caching by time and size) sees a new file, never a
    stale or half written one"""
    import os
    import tempfile
    path = os.path.realpath(path)  # (a symlink's target replaced, not the link)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.' + os.path.basename(path) + '.')
    try:
        with os.fdopen(fd, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
        try:
            os.chmod(tmp, os.stat(path).st_mode & 0o7777)  # (an existing file's permissions kept)
        except FileNotFoundError:
            os.chmod(tmp, 0o666 & ~_umask())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except FileNotFoundError:
            pass
        raise


def _umask():
    import os
    m = os.umask(0)
    os.umask(m)
    return m


def same_file(a, b):
    """Two paths naming one file (symlinks, spellings, hard links), existing or not"""
    import os
    if os.path.exists(a) and os.path.exists(b) and os.path.samefile(a, b):
        return True
    def canon(p):
        if os.path.exists(p):
            return os.path.realpath(p)
        return os.path.join(os.path.realpath(os.path.dirname(p) or '.'), os.path.basename(p))
    return canon(a) == canon(b)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('candidates', nargs='+',
                    help='a list of candidates (with --unknowns, every list you have: files, or folders of them)')
    ap.add_argument('-a', '--activity', default='activity.csv', help="activity's log")
    ap.add_argument('-u', '--coverage', help="activity's coverage file (default: the log's name with -coverage, if there)")
    ap.add_argument('-o', '--output', help='where the annotated rows go (default: <candidates>-survey.csv)')
    ap.add_argument('--range', help='the coverage, MHz: lo-hi (default: the span of the log)')
    ap.add_argument('--tones', help="tone-survey's results: each candidate's listed tone checked against what was heard")
    ap.add_argument('--unknowns', metavar='OUT.csv',
                    help="instead: the log's channels that none of the lists (or radiod's channels) has: "
                         "OUT.csv the ones worth looking up, ranked; OUT-all.csv every one, with why the rest aren't")
    ap.add_argument('--shortlist', type=int, default=40, help='how many to rank in OUT.csv (default 40)')
    ap.add_argument('--corrections', metavar='OUT.csv',
                    help="with --tones and the catalog's master.csv: proposed changes, for frequency-catalog.py correct")
    ap.add_argument('--skip', metavar='PATTERN', help="with --corrections: rows whose Service, Group or Name match are left alone")
    ap.add_argument('--max-width', type=float, default=50,
                    help='kHz: logged signals wider are broadband events, not transmissions (default 50)')
    args = ap.parse_args(argv)
    if not (math.isfinite(args.max_width) and args.max_width > 0):
        sys.exit('--max-width: a positive number of kHz')
    globals()['MAX_WIDTH_KHZ'] = args.max_width  # (too_wide()'s, for this run)
    # A folder: every .csv in it (with --unknowns)
    import os
    expanded = []
    for c in args.candidates:
        if os.path.isdir(c):
            expanded += sorted(os.path.join(c, n) for n in os.listdir(c)
                               if n.lower().endswith('.csv') and not n.startswith('.')
                               and os.path.isfile(os.path.join(c, n)))  # (a regular file: not a folder, a pipe)
        else:
            expanded.append(c)
    args.candidates = expanded
    if args.unknowns:
        everything = re.sub(r'(\.csv)?$', '-all.csv', args.unknowns, count=1)
        import os
        cover_path = args.coverage or re.sub(r'(\.csv)?$', '-coverage.csv', args.activity, count=1)
        have_cover = bool(args.coverage) or os.path.exists(cover_path)
        inputs = args.candidates + [args.activity] + ([cover_path] if have_cover else [])
        if same_file(args.unknowns, everything):
            sys.exit(f'{args.unknowns} and {everything}: the same file: give another output')
        for out_path in (args.unknowns, everything):
            for source in inputs:
                if same_file(out_path, source):
                    sys.exit(f'{out_path}: that is {source}, an input: give another output')
        # The coverage (each frequency's monitored time; the log before it
        # began counted as watched, as for a survey), and the stretches
        # activity recorded (the receiver's centre while each was heard)
        coverage, tuned = None, []
        if have_cover:
            coverage = read_coverage(cover_path)
            tuned = [(s0, e0, round((lo0 + hi0) / 2)) for s0, e0, lo0, hi0 in coverage]
            logged = read_activity(args.activity, wide=True)
            if coverage and logged:
                began = min(s0 for s0, _, _, _ in coverage)
                before = [(t[0], t[1]) for t in logged if t[1] < began - SLACK]
                if before:
                    coverage.append((min(t for _, t in before), began - datetime.timedelta(seconds=1),
                                     min(hz for hz, _ in before), max(hz for hz, _ in before)))
            if not coverage:
                coverage = None
        found = unknowns(args.candidates, args.activity, coverage, tuned)
        fields = ['rank', 'frequency_mhz', 'transmissions', 'airtime_s', 'average_s', 'first', 'last', 'busiest_hours',
                  'db_over_floor', 'dbfs', 'width_khz', 'mode_guess', 'hint', 'skip']
        short = sorted((r for r in found if r['rank'] != '' and r['rank'] <= args.shortlist), key=lambda r: r['rank'])
        write_csv(args.unknowns, fields[:-1], short)
        write_csv(everything, fields, found)
        worth = sum(1 for r in found if r['rank'] != '')
        skipped = collections.Counter(re.sub(r' \(.*| [0-9]{3}\.[0-9]+.*', '', r['skip']).strip() for r in found if r['skip'])
        print(f'{len(found)} channels heard 3 or more times that no list has; {worth} worth looking up '
              f'(the {len(short)} with the most airtime in {args.unknowns}); the rest:')
        for why, n in skipped.most_common():
            print(f'  {n:5d}  {why}')
        print('the first:')
        for r in short[:15]:
            print(f"  {r['rank']:3d}  {r['frequency_mhz']}  {r['transmissions']:6d} x {r['average_s']:>5} s  "
                  f"+{r['db_over_floor']} dB  {r['busiest_hours']}  {r['hint']}")
        print(f'every one, with why it was left out: {everything}')
        return
    if len(args.candidates) != 1:
        sys.exit('one list at a time (several only with --unknowns)')
    args.candidates = args.candidates[0]

    import os
    lo = hi = None
    if args.range:
        m = re.fullmatch(r'\s*([\d.]+)\s*-\s*([\d.]+)\s*', args.range)
        if not m or float(m.group(1)) >= float(m.group(2)):
            sys.exit('--range lo-hi, in MHz')
        lo, hi = float(m.group(1)) * 1e6, float(m.group(2)) * 1e6

    fields, rows = read_candidates(args.candidates)
    cover_path = args.coverage or re.sub(r'(\.csv)?$', '-coverage.csv', args.activity, count=1)
    logged, transmissions, coverage = evidence(args.activity, args.coverage, lo, hi)
    rows = survey(rows, transmissions, lo, hi, coverage)
    if args.tones:
        results = read_tones(args.tones)
        for r in rows:
            tone_check(r, results)
    out = args.output or re.sub(r'(\.csv)?$', '-survey.csv', args.candidates, count=1)
    inputs = (args.candidates, args.activity) + ((cover_path,) if coverage is not None else ()) + \
        ((args.tones,) if args.tones else ())
    for source in inputs:
        if same_file(out, source):
            sys.exit(f'{out}: that is {source}, an input: give another output')
    write_csv(out, fields + [e for e in EVIDENCE if e not in fields], rows)
    if args.corrections:
        if not args.tones:
            sys.exit('--corrections needs --tones')
        if 'ID' not in fields or 'RX Squelch Type' not in fields:
            sys.exit('--corrections: the catalog (master.csv) only: its rows have IDs to correct')
        for source in inputs + (out,):
            if same_file(args.corrections, source):
                sys.exit(f'{args.corrections}: that is {source}: give another output')
        found = corrections(rows, args.skip, fields)
        write_csv(args.corrections, ['ID', 'Name', 'RX MHz', 'Action', 'Field', 'From', 'To', 'Evidence'], found)
        actions = collections.Counter(r['Action'] for r in found)
        changed = len({r['ID'] for r in found if r['Action'] == 'change'})
        print(f"corrections: {changed} channels to change, {actions['remove']} to remove, "
              f"{actions['listen']} to listen to first, {actions['wait']} waiting for more; in {args.corrections}")
    verdicts = collections.Counter(r.get('Survey', '').split(' (rare')[0] for r in rows)
    span = (f'{min(t[1] for t in logged):%Y-%m-%d %H:%M} to {max(t[2] for t in logged):%Y-%m-%d %H:%M}'
            if logged else 'none')
    print(f'{len(rows)} candidates; the log: {len(transmissions)} transmissions, {span}' +
          (f'; range {lo / 1e6:.3f}-{hi / 1e6:.3f} MHz' if lo is not None else '') +
          (f'; activity watched {merged_seconds((s, e) for s, e, _, _ in coverage) / 86400:.2f} days in all'
           if coverage is not None else '; no coverage file: silence is only what the log lacks'))
    for v, n in verdicts.most_common():
        print(f'  {n:5d}  {v}')
    print(f'written to {out}')


if __name__ == '__main__':
    main()
