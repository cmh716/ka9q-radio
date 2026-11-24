#!/usr/bin/env python3
"""frequency-survey.py on a made-up activity log: each transmission to the
nearest candidate within 5 kHz, outside coverage, silence (and rare-use
services flagged), a regular pattern found, both input formats."""
# SPDX-License-Identifier: GPL-3.0-or-later
import csv
import datetime
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

spec = importlib.util.spec_from_file_location('survey', Path(__file__).with_name('frequency-survey.py'))
survey = importlib.util.module_from_spec(spec)
spec.loader.exec_module(survey)

failures = 0


def check(ok, what):
    global failures
    print(('ok   ' if ok else 'FAIL ') + what)
    failures += not ok


d = tempfile.mkdtemp()
log = os.path.join(d, 'activity.csv')
t0 = datetime.datetime(2026, 11, 24, 6, 0, 0)
with open(log, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'frequency_mhz', 'channel_mhz', 'width_khz', 'seconds', 'db_over_floor', 'dbfs', 'known',
                'mode_guess'])
    # 30 regular bursts on 468.654 (between 468.650 and 468.65625: nearest 468.65625)
    for k in range(30):
        s = t0 + datetime.timedelta(seconds=64 * k)
        w.writerow([s.strftime('%Y-%m-%d %H:%M:%S'), (s + datetime.timedelta(seconds=4)).strftime('%Y-%m-%d %H:%M:%S'),
                    '468.6540', '468.65625', '18.75', '4.5', '20', '-80', '', ''])
    # 3 irregular on 453.5 (a candidate), one at 460.0 to stretch the log over a day
    for k, gap in enumerate((0, 700, 5000)):
        s = t0 + datetime.timedelta(seconds=gap)
        w.writerow([s.strftime('%Y-%m-%d %H:%M:%S'), (s + datetime.timedelta(seconds=10)).strftime('%Y-%m-%d %H:%M:%S'),
                    '453.5012', '453.50000', '12.5', '10', '30', '-60', 'Alden', ''])
    s = t0 + datetime.timedelta(days=1)
    w.writerow([s.strftime('%Y-%m-%d %H:%M:%S'), s.strftime('%Y-%m-%d %H:%M:%S'), '460.0000', '460.00000', '12.5', '1',
                '15', '-90', '', ''])
    w.writerow(['2026-11-24 07:00:00', '2026-11-24 07:00', '461.0', 'bad'])  # an incomplete row: skipped

# A RadioReference download
rr = os.path.join(d, 'rr.csv')
with open(rr, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['Frequency Output', 'Frequency Input', 'FCC Callsign', 'Agency/Category', 'Description', 'Alpha Tag',
                'PL Output Tone', 'PL Input Tone', 'Mode', 'Class Station Code', 'Tag'])
    w.writerow(['468.65625', '0', '', 'Utility', 'Meter', 'Meter', '', '', 'FMN', 'FB', 'Utilities'])
    w.writerow(['468.65000', '0', '', 'Utility', 'Other', 'Other', '', '', 'FMN', 'FB', 'Utilities'])
    w.writerow(['453.50000', '0', '', 'Lancaster FD', 'Dispatch', 'LFD Disp', 'D243', '', 'FMN', 'FB', 'Fire Dispatch'])
    w.writerow(['424.12500', '0', '', 'Fire', 'Fireground', 'FG 1', '127.3 PL', '', 'FMN', 'FB', 'Fire-Tac'])
    w.writerow(['425.00000', '0', '', 'Business', 'Shop', 'Shop', '', '', 'FMN', 'FB', 'Business'])
    w.writerow(['155.00000', '0', '', 'Business', 'VHF', 'VHF', '', '', 'FMN', 'FB', 'Business'])
out = os.path.join(d, 'out.csv')
survey.main(['-a', log, '-o', out, '--range', '422.1-469.1', rr])
rows = {r['Frequency Output']: r for r in csv.DictReader(open(out))}
check(rows['468.65625']['Survey'] == survey.ACTIVE and rows['468.65625']['Transmissions'] == '30',
      'transmissions go to the nearest candidate (468.654: 468.65625, not 468.650)')
check(rows['468.65000']['Survey'] == survey.SILENT, 'the farther candidate gets none')
check(rows['468.65625']['Pattern'] == 'regular, every ~64 s', 'a regular pattern found')
check(rows['453.50000']['Transmissions'] == '3' and rows['453.50000']['Airtime s'] == '30'
      and rows['453.50000']['Pattern'] == '', 'three irregular transmissions: counted, airtime, no pattern')
check(rows['424.12500']['Survey'].startswith(survey.SILENT) and 'rare-use' in rows['424.12500']['Survey']
      and rows['424.12500']['Rare-use'] == 'yes', 'a silent fireground channel: silent, flagged rare-use')
check(rows['425.00000']['Survey'] == survey.SILENT and rows['425.00000']['Rare-use'] == '',
      'a silent business channel: silent, not flagged')
check(rows['155.00000']['Survey'] == survey.OUTSIDE, 'VHF: outside --range')
check(rows['453.50000']['Busiest hours'] == '06h 07h', 'busiest hours')

# The catalog's format
cat = os.path.join(d, 'inbox.csv')
with open(cat, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['Decision', 'ID', 'Name', 'RX MHz', 'RX Squelch Type', 'RX Squelch Value', 'Service'])
    w.writerow(['Undecided', 'x1', 'LncstrFD D1', '453.500000', 'DCS', '243', 'Fire Dispatch'])
out2 = os.path.join(d, 'out2.csv')
survey.main(['-a', log, '-o', out2, cat])
r = next(csv.DictReader(open(out2)))
check(r['Survey'] == survey.ACTIVE and r['Decision'] == 'Undecided' and r['Name'] == 'LncstrFD D1',
      "the catalog's format: evidence added, its columns kept")
# Re-surveyed: nothing of the last survey kept (an annotated row, now outside the range)
out3 = os.path.join(d, 'out3.csv')
survey.main(['-a', log, '-o', out3, '--range', '422.1-460.0', out])
rows3 = {r['Frequency Output']: r for r in csv.DictReader(open(out3))}
r = rows3['468.65625']
check(r['Survey'] == survey.OUTSIDE and r['Transmissions'] == '' and r['Pattern'] == '' and r['First heard'] == '',
      'surveyed again, now outside: its old evidence cleared')
# Near the range's edge, an outside candidate takes nothing from one inside
edge = os.path.join(d, 'edge.csv')
with open(edge, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['RX MHz', 'Name'])
    w.writerow(['468.656250', 'inside'])
    w.writerow(['468.655000', 'outside'])
# 468.655 (outside) is nearer 468.654 than 468.65625 (inside): with both inside,
# the nearer gets the transmissions; with only 468.65625 inside, it gets them
out5 = os.path.join(d, 'out5.csv')
survey.main(['-a', log, '-o', out5, '--range', '422.1-468.6560', edge])
rows5 = {r['Name']: r for r in csv.DictReader(open(out5))}
check(rows5['outside']['Survey'] == survey.ACTIVE, '(both inside: the nearer gets them)')
out6 = os.path.join(d, 'out6.csv')
survey.main(['-a', log, '-o', out6, '--range', '468.6554-469.0', edge])
rows6 = {r['Name']: r for r in csv.DictReader(open(out6))}
check(rows6['outside']['Survey'] == survey.OUTSIDE and rows6['inside']['Transmissions'] == '30',
      'the nearer outside the range: the one inside gets the transmissions')
# Never an input as the output
import subprocess
for target in (rr, log, os.path.join(d, '.', 'rr.csv')):
    p = subprocess.run([sys.executable, str(Path(__file__).with_name('frequency-survey.py')), '-a', log, '-o', target, rr],
                       capture_output=True, text=True)
    check(p.returncode != 0 and 'an input' in p.stderr, f'output {os.path.basename(target)} (an input): refused')
link = os.path.join(d, 'alias.csv')
os.symlink(rr, link)
p = subprocess.run([sys.executable, str(Path(__file__).with_name('frequency-survey.py')), '-a', log, '-o', link, rr],
                   capture_output=True, text=True)
check(p.returncode != 0, 'a symlink to an input: refused')
# With activity's coverage: the time each frequency was watched
cov = os.path.join(d, 'cov.csv')
with open(cov, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    w.writerow(['2026-11-24 00:00:00', '2026-11-25 00:00:00', '440.00000', '469.00000'])
    w.writerow(['2026-11-25 06:00:00', '2026-11-26 06:00:00', '450.00000', '460.00000'])
    w.writerow(['bad', 'row'])
cands = os.path.join(d, 'cands.csv')
with open(cands, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['RX MHz', 'Name', 'Service'])
    w.writerow(['455.000000', 'quiet business', 'Business'])
    w.writerow(['465.000000', 'quiet other', 'Business'])
    w.writerow(['425.000000', 'never watched', 'Business'])
    w.writerow(['453.500000', 'heard', 'Fire Dispatch'])
log2 = os.path.join(d, 'act2.csv')
with open(log, newline='') as src, open(log2, 'w', newline='') as dst:
    dst.write(src.read())
    # a transmission at 425.0: activity wasn't watching there, so none is credited
    csv.writer(dst).writerow(['2026-11-24 08:00:00', '2026-11-24 08:00:05', '425.0000', '425.00000', '12.5', '5', '20',
                              '-80', '', ''])
out7 = os.path.join(d, 'out7.csv')
survey.main(['-a', log2, '-u', cov, '-o', out7, cands])
rows7 = {r['Name']: r for r in csv.DictReader(open(out7))}
check(rows7['quiet business']['Survey'] == 'no RF activity in 2.0 days monitored'
      and rows7['quiet business']['Monitored days'] == '2.00', 'silence measured in time watched: 455 MHz in both stretches, 2 days')
check(rows7['quiet other']['Survey'] == 'no RF activity in 1.0 days monitored', '465 MHz in one of them: 1 day')
check(rows7['never watched']['Survey'] == survey.UNCOVERED and rows7['never watched']['Transmissions'] == '',
      'a frequency never watched: not covered, and no transmissions credited to it')
check(rows7['heard']['Survey'] == survey.ACTIVE and rows7['heard']['Monitored days'] == '2.00', 'heard: active, with its time watched')
p = subprocess.run([sys.executable, str(Path(__file__).with_name('frequency-survey.py')), '-a', log2, '-u', cov, '-o', cov, cands],
                   capture_output=True, text=True)
check(p.returncode != 0 and 'an input' in p.stderr, 'the coverage file as the output: refused')
# Overlapping stretches count once
cov2 = os.path.join(d, 'cov2.csv')
with open(cov2, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    w.writerow(['2026-11-24 00:00:00', '2026-11-24 02:00:00', '440.00000', '469.00000'])
    w.writerow(['2026-11-24 01:00:00', '2026-11-24 03:00:00', '440.00000', '469.00000'])  # overlaps the first by an hour
    w.writerow(['2026-11-24 10:00:00', '2026-11-24 11:00:00', '450.00000', '452.00000'])  # later, a narrower span
check(abs(survey.monitored_seconds(455e6, survey.read_coverage(cov2)) - 3 * 3600) < 1, 'overlapping stretches: 3 h, not 4')
check(abs(survey.monitored_seconds(451e6, survey.read_coverage(cov2)) - 4 * 3600) < 1, 'and the later narrow one adds its hour')
# Transmissions credited only where and when watched
log3 = os.path.join(d, 'act3.csv')
with open(log3, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'frequency_mhz', 'channel_mhz', 'width_khz', 'seconds', 'db_over_floor', 'dbfs', 'known',
                'mode_guess'])
    w.writerow(['2026-11-24 01:30:00', '2026-11-24 01:30:05', '455.0000', '455.00000', '12.5', '5', '20', '-80', '', ''])  # watched
    w.writerow(['2026-11-24 05:00:00', '2026-11-24 05:00:05', '455.0000', '455.00000', '12.5', '5', '20', '-80', '', ''])  # not watched then
    w.writerow(['2026-11-24 10:30:00', '2026-11-24 10:30:05', '455.0000', '455.00000', '12.5', '5', '20', '-80', '', ''])  # outside that span
cands3 = os.path.join(d, 'cands3.csv')
with open(cands3, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['RX MHz', 'Name'])
    w.writerow(['455.000000', 'a'])
out8 = os.path.join(d, 'out8.csv')
survey.main(['-a', log3, '-u', cov2, '-o', out8, cands3])
r8 = next(csv.DictReader(open(out8)))
check(r8['Transmissions'] == '1', 'only the transmission heard where and when activity watched is credited (1 of 3)')
# A quiet run: no log rows (or no log), coverage only
empty = os.path.join(d, 'empty.csv')
with open(empty, 'w', newline='') as f:
    csv.writer(f).writerow(['start', 'end', 'frequency_mhz', 'channel_mhz', 'width_khz', 'seconds', 'db_over_floor', 'dbfs',
                            'known', 'mode_guess'])
out9 = os.path.join(d, 'out9.csv')
survey.main(['-a', empty, '-u', cov2, '-o', out9, cands3])
r9 = next(csv.DictReader(open(out9)))
check(r9['Survey'] == 'no RF activity in 0.1 days monitored' and r9['Transmissions'] == '0', 'a quiet run: reported from the coverage')
out10 = os.path.join(d, 'out10.csv')
survey.main(['-a', os.path.join(d, 'no-such-log.csv'), '-u', cov2, '-o', out10, cands3])
check(next(csv.DictReader(open(out10)))['Transmissions'] == '0', 'no log at all, with coverage: reported')
# A hard link to an input as the output
hard = os.path.join(d, 'hard.csv')
os.link(cov2, hard)
p = subprocess.run([sys.executable, str(Path(__file__).with_name('frequency-survey.py')), '-a', log3, '-u', cov2, '-o', hard, cands3],
                   capture_output=True, text=True)
check(p.returncode != 0 and 'an input' in p.stderr, 'a hard link to an input as the output: refused')
# A retune: a transmission just before it goes to the candidate covered then,
# not to a closer one covered only after
cov3 = os.path.join(d, 'cov3.csv')
with open(cov3, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    # the transmission (450.0035) in both spans; 450.000 only in the first, 450.005 only in the second
    w.writerow(['2026-11-24 00:00:00Z', '2026-11-24 01:00:00Z', '440.00000', '450.00400'])
    w.writerow(['2026-11-24 01:00:05Z', '2026-11-24 02:00:00Z', '450.00300', '460.00000'])
log4 = os.path.join(d, 'act4.csv')
with open(log4, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'frequency_mhz', 'channel_mhz', 'width_khz', 'seconds', 'db_over_floor', 'dbfs', 'known',
                'mode_guess'])
    w.writerow(['2026-11-24 00:59:58Z', '2026-11-24 01:00:00Z', '450.0035', '450.00000', '12.5', '2', '20', '-80', '', ''])
cands4 = os.path.join(d, 'cands4.csv')
with open(cands4, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['RX MHz', 'Name'])
    w.writerow(['450.000000', 'covered then'])
    w.writerow(['450.005000', 'covered after'])
out11 = os.path.join(d, 'out11.csv')
survey.main(['-a', log4, '-u', cov3, '-o', out11, cands4])
rows11 = {r['Name']: r for r in csv.DictReader(open(out11))}
check(rows11['covered then']['Transmissions'] == '1' and rows11['covered after']['Transmissions'] == '0',
      'just before a retune: credited to the candidate covered then, not a nearer one covered after')
# Daylight saving: a stretch across the spring change, in local time (rows from before Z), is 10 minutes
import time
os.environ['TZ'] = 'America/New_York'
time.tzset()
cov4 = os.path.join(d, 'cov4.csv')
with open(cov4, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    w.writerow(['2026-03-08 01:55:00', '2026-03-08 03:05:00', '440.00000', '469.00000'])
    w.writerow(['2026-03-08 07:55:00Z', '2026-03-08 08:05:00Z', '440.00000', '469.00000'])
c4 = survey.read_coverage(cov4)
check(abs(survey.monitored_seconds(455e6, c4) - 1200) < 1, 'across the spring change (local 01:55-03:05 EST/EDT): 10 minutes; in UTC: 10')
# The console's total: overlaps once
import io, contextlib
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    survey.main(['-a', log3, '-u', cov2, '-o', os.path.join(d, 'out12.csv'), cands3])
check('activity watched 0.17 days in all' in buf.getvalue(), "the console's total: overlapping stretches once (4 h)")
# A retune within one second: both stretches match the transmission's time
# exactly; a candidate only the later one watched is not credited
cov5 = os.path.join(d, 'cov5.csv')
with open(cov5, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    w.writerow(['2026-11-24 00:00:00Z', '2026-11-24 00:59:58Z', '440.00000', '450.00400'])
    w.writerow(['2026-11-24 00:59:58Z', '2026-11-24 02:00:00Z', '450.00300', '460.00000'])
out13 = os.path.join(d, 'out13.csv')
survey.main(['-a', log4, '-u', cov5, '-o', out13, cands4])
rows13 = {r['Name']: r for r in csv.DictReader(open(out13))}
check(rows13['covered after']['Transmissions'] == '0', 'a retune in the same second: the later-only candidate not credited')
# Truncated rows (missing cells): skipped, not fatal
cov6 = os.path.join(d, 'cov6.csv')
with open(cov6, 'w', newline='') as f:
    f.write('start,end,lo_mhz,hi_mhz\n2026-11-24 00:00:00Z,2026-11-24 01:00:00Z,440.0,469.0\n2026-11-24 01:00:00Z\n')
log5 = os.path.join(d, 'act5.csv')
with open(log5, 'w', newline='') as f:
    f.write('start,end,frequency_mhz,channel_mhz,width_khz,seconds,db_over_floor,dbfs,known,mode_guess\n'
            '2026-11-24 00:30:00Z,2026-11-24 00:30:05Z,455.0000,455.00000,12.5,5,20,-80,"",""\n'
            '2026-11-24 00:40:00Z\n')
out14 = os.path.join(d, 'out14.csv')
survey.main(['-a', log5, '-u', cov6, '-o', out14, cands3])
check(next(csv.DictReader(open(out14)))['Transmissions'] == '1', 'truncated rows in the log and coverage: skipped, the rest read')
# Overlapping runs (no boundary): a candidate inside either qualifies
cov7 = os.path.join(d, 'cov7.csv')
with open(cov7, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    w.writerow(['2026-11-24 00:00:00Z', '2026-11-24 02:00:00Z', '440.00000', '450.00400'])
    w.writerow(['2026-11-24 00:00:00Z', '2026-11-24 02:00:00Z', '450.00300', '460.00000'])
out15 = os.path.join(d, 'out15.csv')
survey.main(['-a', log4, '-u', cov7, '-o', out15, cands4])
rows15 = {r['Name']: r for r in csv.DictReader(open(out15))}
check(rows15['covered after']['Transmissions'] == '1', 'overlapping runs, well inside both: the nearer candidate credited')
# A run ending at the transmission's second while another (no hand-off: it
# began earlier) goes on: the ending run's candidate is credited
cov8 = os.path.join(d, 'cov8.csv')
with open(cov8, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    w.writerow(['2026-11-24 00:00:00Z', '2026-11-24 00:59:58Z', '440.00000', '450.00400'])  # ends then
    w.writerow(['2026-11-24 00:30:00Z', '2026-11-24 02:00:00Z', '450.00300', '460.00000'])  # strictly inside
cands5 = os.path.join(d, 'cands5.csv')
with open(cands5, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['RX MHz', 'Name'])
    w.writerow(['450.002000', 'ending run only'])
out16 = os.path.join(d, 'out16.csv')
survey.main(['-a', log4, '-u', cov8, '-o', out16, cands5])
check(next(csv.DictReader(open(out16)))['Transmissions'] == '1', 'a run ending at that second (no hand-off): its candidate credited')
# Stage 2: tone-survey's results against the listed tones; the survey config
tones = os.path.join(d, 'tones.csv')
with open(tones, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['frequency_mhz', 'transmissions', 'first', 'last', 'commonest', 'heard'])
    w.writerow(['453.50000', '5', '', '', 'D243', 'D243 x5'])
    w.writerow(['468.65625', '9', '', '', 'none', 'none x9'])
    w.writerow(['424.12500', '3', '', '', '131.8 PL', '131.8 PL x2; none x1'])
    w.writerow(['425.00000', '2', '', '', 'P25 NAC 469 TG 10003', 'P25 NAC 469 TG 10003 x2'])
out17 = os.path.join(d, 'out17.csv')
conf = os.path.join(d, 'survey.conf')
survey.main(['-a', log, '-o', out17, '--range', '422.1-469.1', '--tones', tones, '--survey-config', conf, rr])
rows17 = {r['Frequency Output']: r for r in csv.DictReader(open(out17))}
check(rows17['453.50000']['Tone check'] == 'confirmed', 'D243 listed, D243 heard: confirmed')
check(rows17['424.12500']['Tone check'] == 'differs: heard 131.8 PL (listed 127.3 PL)'
      and rows17['424.12500']['Heard'] == '131.8 PL x2; none x1', '127.3 listed, 131.8 heard: differs, with what was heard')
check(rows17['468.65625']['Tone check'] == 'no tone heard', 'nothing listed, none heard: no tone heard')
check(rows17['425.00000']['Tone check'] == 'digital: P25 NAC 469 TG 10003', 'a digital call heard: its mode and IDs')
check(rows17['468.65000']['Tone check'] == 'not surveyed yet', 'a frequency tone-survey has no results for: not surveyed yet')
text = open(conf).read()
check('mode = mixed' in text and 'data = tone-survey.local' in text and '468m656250' in text and '453m500000' in text
      and '425m000000' not in text and '468m650000' not in text, 'the survey config: the active candidates only, mixed, its group')
# The catalog's form of the listed tone
cat2 = os.path.join(d, 'cat2.csv')
with open(cat2, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['ID', 'Name', 'RX MHz', 'RX Squelch Type', 'RX Squelch Value'])
    w.writerow(['a', 'x', '453.500000', 'DCS', '243'])
    w.writerow(['b', 'y', '424.125000', 'CTCSS', '127.3'])
out18 = os.path.join(d, 'out18.csv')
survey.main(['-a', log, '-o', out18, '--tones', tones, cat2])
rows18 = {r['Name']: r for r in csv.DictReader(open(out18))}
check(rows18['x']['Tone check'] == 'confirmed' and rows18['y']['Tone check'].startswith('differs: heard 131.8 PL'),
      "the catalog's squelch columns read as listed tones")
# Inverted DCS: the polarity column read; a shared frequency's less common
# listed tone still heard
tones2 = os.path.join(d, 'tones2.csv')
with open(tones2, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['frequency_mhz', 'transmissions', 'first', 'last', 'commonest', 'heard'])
    w.writerow(['453.50000', '5', '', '', 'D243', 'D243 x5'])
    w.writerow(['424.12500', '5', '', '', '131.8 PL', '131.8 PL x4; 127.3 PL x1'])
    w.writerow(['451.00000', '3', '', '', 'I023', 'I023 x3'])
cat3 = os.path.join(d, 'cat3.csv')
with open(cat3, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['ID', 'Name', 'RX MHz', 'RX Squelch Type', 'RX Squelch Value', 'RX DCS Polarity'])
    w.writerow(['a', 'inv listed, D heard', '453.500000', 'DCS', '243', 'Inverted'])
    w.writerow(['b', 'shared', '424.125000', 'CTCSS', '127.3', ''])
    w.writerow(['c', 'inv listed, I heard', '451.000000', 'DCS', '23', 'Inverted'])
out19 = os.path.join(d, 'out19.csv')
survey.main(['-a', log, '-o', out19, '--tones', tones2, cat3])
rows19 = {r['Name']: r for r in csv.DictReader(open(out19))}
check(rows19['inv listed, D heard']['Tone check'] == 'differs: heard D243 (listed I243)', 'I243 listed, D243 heard: differs')
check(rows19['inv listed, I heard']['Tone check'] == 'confirmed', 'I023 listed, I023 heard: confirmed')
check(rows19['shared']['Tone check'] == 'heard, less often than 131.8 PL', 'a shared frequency: the listed tone heard, less often')
print('Frequency survey tests ' + ('FAILED' if failures else 'passed'))
sys.exit(1 if failures else 0)
