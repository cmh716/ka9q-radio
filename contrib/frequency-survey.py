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
activity's known column), busiest first, with hints: regular intervals,
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


def unknowns(paths, activity_path):
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
    for ch, tx in chans.items():
        if ch in configured or near(ch) is not None or len(tx) < 3:
            continue
        starts = [t[0] for t in tx]
        airtime = sum(t[1] for t in tx)
        avg = airtime / len(tx)
        hints = []
        p = pattern(starts)
        if p:
            hints.append(p)
        if avg < 1.0:
            hints.append('short bursts: data or telemetry')
        b = band_hint(ch)
        if b:
            hints.append(b)
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
            group = [c for c, n in together.most_common() if n >= 0.8 * len(seconds)][:4]
            if group:
                def label(c):
                    return f'{c / 1e6:.5f}' + (f' ({listed[c]})' if c in listed else f' ({names[c]})' if c in names else '')
                peers = ', '.join(f'{label(c)} {100 * together[c] // len(seconds)}%' for c in group)
                # One event only if they start together, all of them, as often
                joint = sum(1 for s0 in seconds if all(c in by_second[s0] for c in group))
                if joint >= 0.8 * len(seconds):
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
            elif any(abs(c - hz) <= 3125 for c in configured):
                hints.append(f'input of {hz / 1e6:.5f} (configured)')
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
        })
    out.sort(key=lambda r: -r['transmissions'])
    return out


def read_activity(path):
    """The log's transmissions: (frequency Hz, start, seconds), and the times it covers"""
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
    text = (row.get('PL Output Tone') or row.get('Source RX Tone') or '').strip()
    if not text:
        return ''
    m = re.fullmatch(r'([\d.]+)\s*PL', text, re.I)
    if m:
        return f'{float(m.group(1)):.1f} PL'
    m = re.fullmatch(r'([DI])?(\d{3})\s*(N|I|DPL|DCS)?', text, re.I)
    if m:
        inverted = (m.group(1) or '').upper() == 'I' or (m.group(3) or '').upper() == 'I'
        return ('I' if inverted else 'D') + m.group(2)
    if text.upper() == 'CSQ':
        return 'none'
    return text


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
    commonest, heard = found
    row['Heard'] = heard
    listed = listed_tone(row)
    kinds = {m.group(1) for m in re.finditer(r'(?:^|; )(.+?) x\d+(?=; |$)', heard)}
    if listed and commonest == listed:
        row['Tone check'] = 'confirmed'
    elif listed and listed != 'none' and listed in kinds:
        # A shared frequency: the listed tone heard, another oftener
        row['Tone check'] = f'heard, less often than {commonest}'
    elif commonest and not re.fullmatch(r'[\d.]+ PL|[DI]\d{3}|none', commonest):
        row['Tone check'] = f'digital: {commonest}'
    elif commonest == 'none':
        row['Tone check'] = 'no tone heard' + (f' (listed {listed})' if listed and listed != 'none' else '')
    else:
        row['Tone check'] = f'differs: heard {commonest}' + (f' (listed {listed})' if listed else ' (none listed)')


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
    ap.add_argument('candidates', nargs='+', help='a list of candidates (with --unknowns, every list you have)')
    ap.add_argument('-a', '--activity', default='activity.csv', help="activity's log")
    ap.add_argument('-u', '--coverage', help="activity's coverage file (default: the log's name with -coverage, if there)")
    ap.add_argument('-o', '--output', help='where the annotated rows go (default: <candidates>-survey.csv)')
    ap.add_argument('--range', help='the coverage, MHz: lo-hi (default: the span of the log)')
    ap.add_argument('--tones', help="tone-survey's results: each candidate's listed tone checked against what was heard")
    ap.add_argument('--unknowns', metavar='OUT.csv',
                    help="instead: the log's busiest channels that none of the lists (or radiod's channels) has")
    args = ap.parse_args(argv)
    if args.unknowns:
        for source in args.candidates + [args.activity]:
            if same_file(args.unknowns, source):
                sys.exit(f'{args.unknowns}: that is {source}, an input: give another output')
        found = unknowns(args.candidates, args.activity)
        with open(args.unknowns, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(found[0]) if found else ['frequency_mhz'])
            w.writeheader()
            w.writerows(found)
        print(f'{len(found)} channels heard 3 or more times that no list has; the busiest:')
        for r in found[:15]:
            print(f"  {r['frequency_mhz']}  {r['transmissions']:6d} x {r['average_s']:>5} s  "
                  f"+{r['db_over_floor']} dB  {r['busiest_hours']}  {r['hint']}")
        print(f'written to {args.unknowns}')
        return
    if len(args.candidates) != 1:
        sys.exit('one list at a time (several only with --unknowns)')
    args.candidates = args.candidates[0]

    import os
    transmissions = read_activity(args.activity) if os.path.exists(args.activity) else []
    lo = hi = None
    if args.range:
        m = re.fullmatch(r'\s*([\d.]+)\s*-\s*([\d.]+)\s*', args.range)
        if not m or float(m.group(1)) >= float(m.group(2)):
            sys.exit('--range lo-hi, in MHz')
        lo, hi = float(m.group(1)) * 1e6, float(m.group(2)) * 1e6

    fields, rows = read_candidates(args.candidates)
    cover_path = args.coverage or re.sub(r'(\.csv)?$', '-coverage.csv', args.activity, count=1)
    coverage = None
    if args.coverage or os.path.exists(cover_path):
        coverage = read_coverage(cover_path)
        if not coverage:
            sys.exit(f'{cover_path}: no coverage rows')
    if not transmissions and coverage is None:
        sys.exit(f'{args.activity}: no transmissions logged, and no coverage file to say it was watching')
    if coverage is not None:
        # The log from before coverage was recorded (an older activity):
        # watched as the log says, from its first transmission until the
        # coverage starts, across --range or the log's span, as with no
        # coverage file at all
        began = min(s for s, _, _, _ in coverage)
        before = [(hz, start) for hz, start, _, _ in transmissions if start < began - SLACK]
        if before:
            blo = lo if lo is not None else min(hz for hz, _ in before)
            bhi = hi if hi is not None else max(hz for hz, _ in before)
            coverage.append((min(t for _, t in before), began - datetime.timedelta(seconds=1), blo, bhi))
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
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields + [e for e in EVIDENCE if e not in fields], extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    verdicts = collections.Counter(r.get('Survey', '').split(' (rare')[0] for r in rows)
    span = (f'{min(t[1] for t in transmissions):%Y-%m-%d %H:%M} to {max(t[2] for t in transmissions):%Y-%m-%d %H:%M}'
            if transmissions else 'none')
    print(f'{len(rows)} candidates; the log: {len(transmissions)} transmissions, {span}' +
          (f'; range {lo / 1e6:.3f}-{hi / 1e6:.3f} MHz' if lo is not None else '') +
          (f'; activity watched {merged_seconds((s, e) for s, e, _, _ in coverage) / 86400:.2f} days in all'
           if coverage is not None else '; no coverage file: silence is only what the log lacks'))
    for v, n in verdicts.most_common():
        print(f'  {n:5d}  {v}')
    print(f'written to {out}')


if __name__ == '__main__':
    main()
