#!/usr/bin/env python3
"""freqs: review, approve, reject, apply, on a catalog of its own"""
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse
import contextlib
import csv
import datetime
import importlib.util
import io
import os
import sys
import tempfile
from importlib.machinery import SourceFileLoader
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True
loader = SourceFileLoader('freqs', str(HERE / 'freqs'))
spec = importlib.util.spec_from_loader('freqs', loader)
freqs = importlib.util.module_from_spec(spec)
loader.exec_module(freqs)
catalog = freqs.catalog

failures = 0


def check(ok, what):
    global failures
    if not ok:
        failures += 1
        print('FAIL', what)


def quiet(f, *a):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return f(*a)


d = Path(tempfile.mkdtemp())
cat = d / 'catalog'
downloads = d / 'downloads'
downloads.mkdir()
(downloads / 'sites').mkdir()
# The catalog: one channel listed with the wrong tone, one silent (a ham listing: fire's rare-use)
catalog.write_csv(cat / 'source-rules.csv', ['File', 'Service', 'Group'], [])
rows = []
for ident, name, mhz, kind, value in (('m1', 'Wrong tone', '453.475000', 'DCS', '364'),
                                       ('m2', 'Silent', '453.900000', 'Carrier', ''),
                                       ('m3', 'On a trunk', '463.300000', 'Carrier', '')):
    r = dict.fromkeys(catalog.FIELDS, '')
    r.update({'ID': ident, 'Name': name, 'RX MHz': mhz, 'RX Mode': 'FMN', 'RX Squelch Type': kind,
              'RX Squelch Value': value, 'RX DCS Polarity': 'Normal' if kind == 'DCS' else '',
              'Service': 'Ham' if ident == 'm2' else 'Fire-Tac', 'Group': 'ham' if ident == 'm2' else 'fire-tac',
              'Receive Only': 'yes', 'Operation': 'Unknown',
              'Profiles': 'uhf'})
    rows.append(catalog.upgrade(r))
catalog.write_csv(cat / 'master.csv', catalog.FIELDS, rows)
catalog.write_csv(cat / 'ignored.csv', catalog.FIELDS, [])
# A download: one heard with its tone, one heard digital, one never heard, one a trunked system's
with open(downloads / 'County.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['Frequency Output', 'Frequency Input', 'FCC Callsign', 'Agency/Category', 'Description', 'Alpha Tag',
                'PL Output Tone', 'PL Input Tone', 'Mode', 'Class Station Code', 'Tag'])
    w.writerow(['460.100000', '', '', 'Town', 'Town Highway', 'Twn Hwy', '151.4 PL', '', 'FMN', 'FB', 'Public Works'])
    w.writerow(['461.200000', '', '', 'Town', 'Town Police', 'Twn PD', '', '', 'FMN', 'FB', 'Law Dispatch'])
    w.writerow(['462.900000', '', '', 'Town', 'Never heard', 'Quiet', '', '', 'FMN', 'FB', 'Business'])
    w.writerow(['424.300000', '', '', 'County', 'Trunked', 'Trunk', '', '', 'P25', 'FB', 'Law Dispatch'])
    w.writerow(['463.300000', '', '', 'SaiaNet', 'Site 008', '', '', '', 'DMR', '', 'TRS'])          # a trunked site, in the county list
    w.writerow(['463.300000', '', '', 'Town', 'Town Fire Tac', 'Fire Tac', '', '', 'FMN', 'FB', 'Fire-Tac'])  # sharing it
with open(downloads / 'sites' / 'trs_sites_1.csv', 'w', newline='') as f:
    f.write('Site Dec,Site Hex,Description,County Name,Lat,Lon,Range,Frequencies\n001,1,"Site","County",43,-78,10,424.300000c\n')
# The evidence: two days watched; transmissions on each heard one
log = d / 'activity.csv'
with open(log, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'frequency_mhz', 'channel_mhz', 'width_khz', 'seconds', 'db_over_floor', 'dbfs', 'known', 'mode_guess'])
    for mhz in (453.475, 460.1, 461.2, 424.3, 463.3):
        stamp = '2026-11-25 10:00:00Z'
        w.writerow([stamp, stamp, f'{mhz:.4f}', f'{mhz:.5f}', '12.5', 3, 20, -90, '', ''])
    for k in range(100):  # in no list, busy (40 a day), irregular: audited by activity -U
        t = 3600 * 24 + 1700 * k + (k * k * 97) % 1500
        stamp = (datetime.datetime(2026, 11, 24, tzinfo=datetime.timezone.utc) + datetime.timedelta(seconds=t)).strftime('%Y-%m-%d %H:%M:%SZ')
        w.writerow([stamp, stamp, '455.5000', '455.50000', '12.5', 4, 20, -90, '', 'narrow'])
with open(d / 'activity-coverage.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['start', 'end', 'lo_mhz', 'hi_mhz'])
    w.writerow(['2026-11-24 00:00:00Z', '2026-11-26 12:00:00Z', '422.00000', '469.00000'])
tones = d / 'tone-survey.csv'
with open(tones, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['frequency_mhz', 'transmissions', 'first', 'last', 'commonest', 'heard'])
    w.writerow(['453.47500', '56', '', '', '127.3 PL', '127.3 PL x56'])
    w.writerow(['460.10000', '30', '', '', '151.4 PL', '151.4 PL x30'])
    w.writerow(['461.20000', '40', '', '', 'DMR TG 5', 'DMR TG 5 x40'])
    w.writerow(['424.30000', '40', '', '', 'P25 TG 1', 'P25 TG 1 x40'])
    w.writerow(['455.50000', '30', '', '', '127.3 PL', '127.3 PL x30'])
    w.writerow(['463.30000', '40', '', '', 'DMR TG 9', 'DMR TG 9 x40'])
conf = d / 'freqs.conf'
conf.write_text(f'[freqs]\ncatalog = {cat}\ndownloads = {downloads}\nsites = {downloads / "sites"}\n'
                f'activity = {log}\ntones = {tones}\nbase = {d}\nconfig = {d / "conv.conf"}\nnames = {d / "names.csv"}\n')
s = freqs.settings(str(conf))

rows = quiet(freqs.review, s)
by = {r['Name']: r for r in rows}
check(by.get('Wrong tone', {}).get('Proposal') == 'Change tone D364 -> 127.3 PL', 'a catalog channel heard with another tone: change')
check(by.get('Silent', {}).get('Proposal') == 'Remove', 'a silent one: remove')
check(by.get('Twn Hwy', {}).get('Proposal') == 'Add (151.4 PL, as listed)', 'a download heard with its tone: add')
check(by.get('Twn PD', {}).get('Proposal') == 'Add as DMR (listed FMN)', 'a download heard digital: add as that')
check('Quiet' not in by and 'Trunk' not in by, 'neither the unheard nor a trunked system\'s channel proposed')
check('On a trunk' not in by, "a catalog channel on a trunked site's frequency: the system's DMR not taken for its own")
check('Fire Tac' not in by and not any(n.startswith('Site 008') for n in by), "a county list's TRS row: neither it nor a listing sharing it added")
check(by.get('Unknown 455.50000', {}).get('Proposal') == 'Add (in no list) with 127.3 PL',
      'a busy frequency in no list, measured: added under a placeholder')

# Approve the change and the tone add; reject the DMR add; leave the removal
sheet = cat / 'review.csv'
marked = []
for r in catalog.read_csv(sheet):
    r['Decision'] = {'Wrong tone': 'Approve', 'Twn Hwy': 'approve', 'Twn PD': 'Reject', 'Unknown 455.50000': 'Approve'}.get(r['Name'], '')
    marked.append(r)
catalog.write_csv(sheet, freqs.COLUMNS, marked)
quiet(freqs.apply, s)
master = {r['Name']: r for r in catalog.read_csv(cat / 'master.csv')}
check((master['Wrong tone']['RX Squelch Type'], master['Wrong tone']['RX Squelch Value']) == ('CTCSS', '127.3'), 'the change applied')
check('Twn Hwy' in master and master['Twn Hwy']['RX Squelch Value'] == '151.4', 'the add applied')
check('Twn PD' not in master and 'Silent' in master, 'the rejected not added; the undecided left')
check(master.get('Unknown 455.50000', {}).get('Service') == 'Unknown' and master['Unknown 455.50000']['RX Squelch Value'] == '127.3',
      'the unlisted one in the catalog: Service Unknown, its tone')
left = {r['Name']: r for r in catalog.read_csv(sheet)}
check(set(left) == {'Silent'}, 'the sheet afterwards: only the undecided; the rejected not proposed again')

# A bad Decision changes nothing
rows = catalog.read_csv(sheet)
rows[0]['Decision'] = 'maybe'
catalog.write_csv(sheet, freqs.COLUMNS, rows)
before = (cat / 'master.csv').read_text()
try:
    quiet(freqs.apply, s)
    check(False, 'a bad Decision refused')
except SystemExit:
    pass
check((cat / 'master.csv').read_text() == before, 'a bad Decision: nothing written')

# One batch: an approved change with an invalid add: nothing written
quiet(freqs.review, s)
rows = catalog.read_csv(sheet)
# (a fresh change to approve: the catalog's channel listed wrongly again)
m = catalog.read_csv(cat / 'master.csv')
for r in m:
    if r['Name'] == 'Wrong tone':
        r.update({'RX Squelch Type': 'DCS', 'RX Squelch Value': '364', 'RX DCS Polarity': 'Normal'})
catalog.write_csv(cat / 'master.csv', catalog.FIELDS, m)
quiet(freqs.review, s)
rows = catalog.read_csv(sheet)
import json
with open(downloads / 'County.csv', 'a', newline='') as f:
    pass
for r in rows:
    if r['Name'] == 'Wrong tone':
        r['Decision'] = 'Approve'
bad = {'kind': 'add', 'row': dict(catalog.upgrade(dict.fromkeys(catalog.FIELDS, '')), ID='bad1', Name='Bad', **{'RX MHz': '455.000000', 'RX Mode': 'NOPE', 'Group': 'x', 'Receive Only': 'yes'}), 'source': {}}
rows.append({'Decision': 'Approve', 'Frequency': '455.0000', 'Name': 'Bad', 'Proposal': 'Add', 'Why': '', 'List': 'x',
             'Key': 'add:bad1:0', 'Payload': json.dumps(bad)})
catalog.write_csv(sheet, freqs.COLUMNS, rows)
before = (cat / 'master.csv').read_text()
try:
    quiet(freqs.apply, s)
    check(False, 'a batch with an invalid add refused')
except SystemExit:
    pass
check((cat / 'master.csv').read_text() == before, 'an invalid add: the approved change not written either')

# Every listing at a frequency tried: the first sorted, not viable, doesn't hide the next
r1 = catalog.upgrade(dict(dict.fromkeys(catalog.FIELDS, ''), ID='a1', Name='Aaa', **{'RX MHz': '459.000000', 'RX Mode': 'AM',
                     'RX Squelch Type': 'CTCSS', 'RX Squelch Value': '127.3', 'Survey': freqs.survey.ACTIVE,
                     'Heard': '127.3 PL x60', 'Tone check': 'confirmed'}))
r2 = catalog.upgrade(dict(dict.fromkeys(catalog.FIELDS, ''), ID='b1', Name='Bbb', **{'RX MHz': '459.000000', 'RX Mode': 'FMN',
                     'RX Squelch Type': 'Carrier', 'Survey': freqs.survey.ACTIVE, 'Heard': '127.3 PL x60', 'Tone check': 'differs'}))
check(freqs.add_verdict(r1) is None and freqs.add_verdict(r2) is not None, 'the fixture: the first not viable, the second viable')
chosen = freqs.choose([r1, r2])
check(chosen is not None and chosen[0] is r2, 'the viable listing chosen though another sorts first')

# A config's channels: freq and freq0..freq9 lines
c = d / 'c.conf'
c.write_text('[a]\nmode = npm\nfreq = "450m0 451m0"\nfreq0 = "452m0"\nfreq9 = "453m0"\n')
check(freqs.sections(c) == {('a', '450m0'), ('a', '451m0'), ('a', '452m0'), ('a', '453m0')}, 'freq0..freq9 read')

# Files replaced together: a failure writing leaves all as they were
a_ = d / 'one.txt'; b_ = d / 'sub'
a_.write_text('old a')
b_.mkdir(exist_ok=True)
(b_ / 'two.txt').mkdir(exist_ok=True)   # (a folder where the file should go: its rename fails)
try:
    catalog.write_together({a_: 'new a', b_ / 'two.txt': 'new b'})
    check(False, 'a failing batch write raises')
except OSError:
    pass
check(a_.read_text() == 'old a' and not [p for p in d.glob('.one.txt*')], 'a failing batch write: the first file as it was, no temporary left')
# A rename failing partway: those already replaced put back
c_ = d / 'three.txt'; e_ = d / 'four.txt'
c_.write_text('old c'); e_.write_text('old e')
real = catalog.os.replace
calls = []
def failing(src, dst):
    calls.append(dst)
    if len(calls) == 2:
        raise OSError('the second rename fails')
    return real(src, dst)
catalog.os.replace = failing
try:
    catalog.write_together({c_: 'new c', e_: 'new e'})
    check(False, 'a rename failing raises')
except OSError:
    pass
finally:
    catalog.os.replace = real
check(c_.read_text() == 'old c' and e_.read_text() == 'old e' and not [p for p in d.glob('.three*')] and not [p for p in d.glob('.four*')],
      'a rename failing partway: the first put back, no temporary left')
# The first restore failing too: its original kept, named in the error
c_.write_text('old c'); e_.write_text('old e')
calls = []
def failing2(src, dst):
    calls.append((src, dst))
    if len(calls) == 2:
        raise OSError('the second rename fails')
    if len(calls) == 3:
        raise OSError('and the restore fails')
    return real(src, dst)
catalog.os.replace = failing2
try:
    catalog.write_together({c_: 'new c', e_: 'new e'})
    check(False, 'a failed restore raises')
except OSError as err:
    kept_old = [Path(p) for p in d.glob('.three.txt.old*')]
    check('could not be put back' in str(err) and kept_old and kept_old[0].read_text() == 'old c',
          "a restore failing: the original kept, and named")
    for p in kept_old:
        p.unlink()
finally:
    catalog.os.replace = real
# Ctrl-C just after a rename completes: rolled back all the same
c_.write_text('old c'); e_.write_text('old e')
calls = []
def interrupted(src, dst):
    real(src, dst)
    calls.append(dst)
    if len(calls) == 1:
        raise KeyboardInterrupt
catalog.os.replace = interrupted
try:
    catalog.write_together({c_: 'new c', e_: 'new e'})
    check(False, 'an interrupt passes through')
except KeyboardInterrupt:
    pass
finally:
    catalog.os.replace = real
check(c_.read_text() == 'old c' and e_.read_text() == 'old e' and not list(d.glob('.three*')) and not list(d.glob('.four*')),
      'Ctrl-C after a rename: put back, nothing left over')
# Voice detection: untoned in no list only on "voice"; carriers and noise no evidence
def verdict(heard, listed_kind='Carrier', voice_only=False):
    r = catalog.upgrade(dict(dict.fromkeys(catalog.FIELDS, ''), ID='v', Name='v', **{'RX MHz': '459.000000', 'RX Mode': 'FMN',
                         'RX Squelch Type': listed_kind, 'Heard': heard}))
    return freqs.add_verdict(r, voice_only=voice_only)
check(verdict('voice x30', voice_only=True) is not None and verdict('voice x30', voice_only=True)[0] == 'Add (no tone)',
      'in no list, someone heard talking 30 times, no tone: proposed')
check(verdict('none x30', voice_only=True) is None, 'in no list, "none" from before voice detection: not enough')
check(verdict('carrier x30', voice_only=True) is None and verdict('noise x30') is None, 'carriers and noise: never proposed')
check(verdict('voice x12; carrier x40') is not None, 'a listed untoned channel: its voice counts, carriers between it no evidence against')
check(verdict('127.3 PL x10; noise x40') is not None and verdict('127.3 PL x10; noise x40')[0].startswith('Add with 127.3 PL'),
      'a tone among noise: the noise no evidence against it')
print('freqs tests ' + ('FAILED' if failures else 'passed'))
sys.exit(1 if failures else 0)
