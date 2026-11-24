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
check(rows['468.65625']['Survey'] == 'active' and rows['468.65625']['Transmissions'] == '30',
      'transmissions go to the nearest candidate (468.654: 468.65625, not 468.650)')
check(rows['468.65000']['Survey'].startswith('silent'), 'the farther candidate gets none')
check(rows['468.65625']['Pattern'] == 'regular, every ~64 s', 'a regular pattern found')
check(rows['453.50000']['Transmissions'] == '3' and rows['453.50000']['Airtime s'] == '30'
      and rows['453.50000']['Pattern'] == '', 'three irregular transmissions: counted, airtime, no pattern')
check(rows['424.12500']['Survey'].startswith('silent 1.0 days') and 'rare-use' in rows['424.12500']['Survey']
      and rows['424.12500']['Rare-use'] == 'yes', 'a silent fireground channel: silent, flagged rare-use')
check(rows['425.00000']['Survey'] == 'silent 1.0 days' and rows['425.00000']['Rare-use'] == '',
      'a silent business channel: silent, not flagged')
check(rows['155.00000']['Survey'] == 'outside coverage', 'VHF: outside the coverage')
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
check(r['Survey'] == 'active' and r['Decision'] == 'Undecided' and r['Name'] == 'LncstrFD D1',
      "the catalog's format: evidence added, its columns kept")
print('Frequency survey tests ' + ('FAILED' if failures else 'passed'))
sys.exit(1 if failures else 0)
