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
print('Frequency survey tests ' + ('FAILED' if failures else 'passed'))
sys.exit(1 if failures else 0)
