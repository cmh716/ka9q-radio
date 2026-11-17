#!/usr/bin/env python3
"""Regression checks for review decisions and master-data preservation."""
import argparse
import contextlib
import csv
import importlib.util
import io
from pathlib import Path
import tempfile
import sys
import unittest

spec = importlib.util.spec_from_file_location('catalog', Path(__file__).with_name('frequency-catalog.py'))
catalog = importlib.util.module_from_spec(spec)
sys.dont_write_bytecode = True
spec.loader.exec_module(catalog)


class CatalogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.w = self.root / 'catalog'
        self.source = self.root / 'GMRS.csv'
        self.rows = [{'Frequency Output': '462.550', 'Frequency Input': '467.550',
                      'Alpha Tag': 'GMRS 15', 'Description': 'Original description',
                      'Mode': 'FM', 'Tag': 'Other', 'PL Output Tone': ''},
                     {'Frequency Output': '462.575', 'Alpha Tag': 'GMRS 16',
                      'Description': 'Second channel', 'Mode': 'FM', 'Tag': 'Other'}]
        catalog.write_csv(self.w / 'source-rules.csv', ['File', 'Service', 'Group'],
                          [{'File': 'GMRS.csv', 'Service': 'GMRS', 'Group': 'gmrs-frs'}])
        self.stage()

    def call(self, func, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            func(argparse.Namespace(workspace=self.w, **kwargs))

    def stage(self):
        catalog.write_csv(self.source, catalog.RR_FIELDS, self.rows)
        self.call(catalog.import_files, files=[self.source])

    def read(self, name):
        return catalog.read_csv(self.w / name)

    def decisions(self):
        rows = self.read('inbox.csv')
        rows[0]['Decision'] = 'Keep'
        rows[0]['Name'] = 'Family repeater'
        rows[1]['Decision'] = 'Ignore'
        catalog.write_csv(self.w / 'inbox.csv', catalog.INBOX, rows)
        self.call(catalog.apply)

    def test_selection_and_ignore_survive_reimport(self):
        self.assertEqual(self.read('master.csv'), [])
        self.decisions()
        self.stage()
        self.assertEqual(len(self.read('master.csv')), 1)
        self.assertEqual(self.read('master.csv')[0]['Name'], 'Family repeater')
        self.assertEqual(self.read('inbox.csv'), [])
        self.assertEqual(len(self.read('ignored.csv')), 1)
        self.assertTrue(list((self.w / 'backups').glob('*/master.csv')))

    def test_update_preserves_personal_values_and_requires_review(self):
        self.decisions()
        self.rows[0]['Description'] = 'Updated description'
        self.stage()
        pending = self.read('inbox.csv')
        self.assertEqual(len(pending), 1)
        self.assertEqual(pending[0]['Decision'], 'Undecided')
        self.assertEqual(pending[0]['Name'], 'Family repeater')
        self.assertEqual(self.read('master.csv')[0]['Description'], 'Original description')
        pending[0]['Decision'] = 'Ignore'
        catalog.write_csv(self.w / 'inbox.csv', catalog.INBOX, pending)
        self.call(catalog.apply)
        self.stage()
        self.assertEqual(self.read('inbox.csv'), [])
        self.assertEqual(len(self.read('master.csv')), 1)

    def test_bad_decision_writes_nothing(self):
        rows = self.read('inbox.csv')
        rows[0]['Decision'] = 'Keep'
        rows[1]['Decision'] = 'Oops'
        catalog.write_csv(self.w / 'inbox.csv', catalog.INBOX, rows)
        before = (self.w / 'master.csv').read_bytes()
        with self.assertRaises(ValueError):
            self.call(catalog.apply)
        self.assertEqual((self.w / 'master.csv').read_bytes(), before)

    def test_export_only_master_profile(self):
        self.decisions()
        out = self.root / 'selected.csv'
        self.call(catalog.export, profile='uhf', output=out)
        rows = catalog.read_csv(out)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['Tag'], 'gmrs-frs')
        self.call(catalog.export, profile='hf', output=out)
        self.assertEqual(catalog.read_csv(out), [])

    def test_disappearing_source_does_not_delete_master(self):
        self.decisions()
        self.rows = []
        self.stage()
        self.assertEqual(len(self.read('master.csv')), 1)

    def test_pending_spreadsheet_edits_survive_reimport(self):
        rows = self.read('inbox.csv')
        rows[0]['Name'] = 'My edited label'
        rows[0]['Decision'] = 'Keep'
        catalog.write_csv(self.w / 'inbox.csv', catalog.INBOX, rows)
        self.stage()
        self.assertEqual(self.read('inbox.csv')[0]['Name'], 'My edited label')
        self.assertEqual(self.read('inbox.csv')[0]['Decision'], 'Keep')

    def test_receiver_export_combines_fm_widths(self):
        self.decisions()
        master = self.read('master.csv')
        other = dict(master[0], ID='frs-equivalent', **{'RX Mode': 'FMN'}, Service='FRS', Name='FRS 15')
        catalog.write_csv(self.w / 'master.csv', catalog.FIELDS, master + [other])
        out = self.root / 'selected.csv'
        self.call(catalog.export, profile='uhf', output=out)
        rows = catalog.read_csv(out)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['Mode'], 'FM')
        self.assertIn('FRS 15', rows[0]['Alpha Tag'])

    def test_receiver_group_conflict_is_not_silently_resolved(self):
        self.decisions()
        master = self.read('master.csv')
        other = dict(master[0], ID='conflicting', Group='different')
        catalog.write_csv(self.w / 'master.csv', catalog.FIELDS, master + [other])
        out = self.root / 'selected.csv'
        out.write_text('previous export')
        with self.assertRaises(ValueError):
            self.call(catalog.export, profile='uhf', output=out)
        self.assertEqual(out.read_text(), 'previous export')

    def test_legacy_migration_preserves_decisions_and_raw_values(self):
        legacy_fields = ['ID', 'Name', 'RX MHz', 'TX MHz', 'Mode', 'RX Tone', 'TX Tone',
                         'Service', 'Group', 'Profiles', 'Receive Only', 'Description', 'Source', 'Notes']
        old = {'ID': 'legacy', 'Name': 'Test repeater', 'RX MHz': '146.940', 'TX MHz': '146.340',
               'Mode': 'FM', 'RX Tone': 'D023', 'TX Tone': '107.2', 'Group': 'ham',
               'Receive Only': 'yes', 'Notes': 'Personal note'}
        catalog.write_csv(self.w / 'master.csv', legacy_fields, [old])
        pending = dict(old, ID='pending', Decision='Keep')
        catalog.write_csv(self.w / 'inbox.csv', ['Decision'] + legacy_fields, [pending])
        self.call(catalog.migrate)
        migrated = self.read('master.csv')[0]
        self.assertEqual(migrated['Offset MHz'], '-0.600000')
        self.assertEqual(migrated['RX Mode'], 'FM')
        self.assertEqual(migrated['TX Mode'], '')
        self.assertEqual(migrated['Operation'], 'Unknown')
        self.assertEqual(migrated['Source RX Tone'], 'D023')
        self.assertEqual(migrated['TX Access Type'], 'CTCSS')
        self.assertEqual(migrated['TX Access Value'], '107.2')
        self.assertEqual(migrated['RX DCS Polarity'], 'Normal')
        self.assertEqual(migrated['Notes'], 'Personal note')
        self.assertEqual(self.read('inbox.csv')[0]['Decision'], 'Keep')
        self.assertTrue((self.w / 'device-profiles.csv').exists())
        before = (self.w / 'master.csv').read_bytes()
        self.call(catalog.migrate)
        self.assertEqual((self.w / 'master.csv').read_bytes(), before)
        self.assertNotIn('Mode', migrated)

    def test_disabled_tone_values_do_not_enable_squelch(self):
        row = catalog.upgrade(dict(ID='off', Name='Off', **{'RX MHz': '462.55', 'RX Mode': 'FM',
                              'Group': 'ham', 'RX Squelch Type': 'Carrier', 'RX Squelch Value': '88.5',
                              'TX Access Type': 'None', 'TX Access Value': '67.0'}))
        catalog.validate(row)
        self.assertEqual(catalog.receiver_tone(row), '')
        self.assertEqual(catalog.access_text(row['TX Access Type'], row['TX Access Value']), '')
        self.assertEqual(catalog.upgrade(row)['TX Access Value'], '67.0')

    def test_explicit_transmit_settings_and_derived_offset(self):
        row = catalog.upgrade(dict(ID='tx', Name='TX', **{'RX MHz': '146.940', 'RX Mode': 'FM',
                              'Group': 'ham', 'Receive Only': 'no'}))
        with self.assertRaises(ValueError): catalog.validate(row)
        row.update({'TX MHz': '146.340', 'TX Mode': 'FM', 'Operation': 'Repeater', 'Offset MHz': '999'})
        catalog.validate(row)
        self.assertEqual(catalog.upgrade(row)['Offset MHz'], '-0.600000')
        row['Operation'] = 'Simplex'
        with self.assertRaises(ValueError): catalog.validate(row)
        row.update({'TX MHz': '146.940', 'Operation': 'Simplex'})
        catalog.validate(row)
        self.assertEqual(catalog.upgrade(row)['Offset MHz'], '+0.000000')

    def test_digital_fields_and_dcs_polarity(self):
        p25 = catalog.upgrade(dict(ID='p25', Name='P25', Mode='P25', **{'RX MHz': '460',
                               'RX Tone': '293 NAC', 'Group': 'law'}))
        self.assertEqual(catalog.receiver_tone(p25), '293 NAC')
        dmr = catalog.upgrade(dict(ID='dmr', Name='DMR', Mode='DMR', **{'RX MHz': '444.375',
                               'RX Tone': 'CC 1 SL 2', 'Group': 'ham'}))
        self.assertEqual(dmr['DMR Color Code'], '1')
        self.assertEqual(catalog.receiver_tone(dmr), 'CC 1 SL 2')
        dmr['DMR Timeslot'] = '3'
        with self.assertRaises(ValueError): catalog.validate(dmr)
        dcs = catalog.upgrade(dict(ID='dcs', Name='DCS', Mode='FM', **{'RX MHz': '460',
                               'RX Tone': 'D023 I', 'Group': 'law'}))
        self.assertEqual(dcs['RX DCS Polarity'], 'Inverted')
        with self.assertRaises(ValueError): catalog.receiver_tone(dcs)

    def test_new_fields_and_custom_columns_survive_apply_and_refresh(self):
        self.decisions()
        master = self.read('master.csv')
        master[0].update({'TX Mode': 'FM', 'Operation': 'Repeater', 'Bandwidth kHz': '12.5',
                          'D-STAR RPT1': 'MYRPT  B', 'My Custom Column': 'Do not lose'})
        catalog.write_csv(self.w / 'master.csv', catalog.FIELDS, master)
        self.rows[0]['Description'] = 'Updated description'
        self.stage()
        inbox = self.read('inbox.csv')
        self.assertEqual(inbox[0]['TX Mode'], 'FM')
        self.assertEqual(inbox[0]['My Custom Column'], 'Do not lose')
        inbox[0]['Decision'] = 'Keep'
        catalog.write_csv(self.w / 'inbox.csv', catalog.INBOX, inbox)
        self.call(catalog.apply)
        self.assertEqual(self.read('master.csv')[0]['D-STAR RPT1'], 'MYRPT  B')
        self.assertEqual(self.read('master.csv')[0]['My Custom Column'], 'Do not lose')


if __name__ == '__main__':
    unittest.main()
