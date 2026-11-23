#!/usr/bin/env python3
"""Synthetic radiod status and RTP tests for channel-audit.py."""
# SPDX-License-Identifier: GPL-3.0-or-later
import importlib.util
from pathlib import Path
import socket
import struct
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('channel_audit', Path(__file__).with_name('channel-audit.py'))
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)

GROUP = '239.194.70.23'
SOURCE = '192.0.2.1'
STAMP = '2026-11-22T20:00:00+00:00'


def status(ssrc, samples, *, code=31, tone=0, inverted=False, center=445600000):
    options = {
        audit.SSRC: struct.pack('!I', ssrc),
        audit.INPUT_SAMPLES: struct.pack('!Q', samples),
        audit.DATA_DEST: socket.inet_aton(GROUP) + struct.pack('!H', 5004),
        audit.DEMOD: b'\x01',
        audit.FREQUENCY: struct.pack('!d', 453125000),
        audit.FIRST_LO: struct.pack('!d', center),
        audit.FE_LOW: struct.pack('!f', -23500000),
        audit.FE_HIGH: struct.pack('!f', 23500000),
        audit.SAMPLE_RATE: struct.pack('!I', audit.PCM_RATE),
        audit.ENCODING: b'\x02',
        audit.RTP_PT: b'\x60',
        audit.DCS_CODE: bytes([code]),
        audit.DCS_INVERTED: bytes([inverted]),
        audit.PL_TONE: struct.pack('!f', tone),
    }
    for tag in options.keys() - {audit.DATA_DEST}:
        options[tag] = options[tag].lstrip(b'\0')
    return b'\0' + b''.join(bytes((tag, len(value))) + value
                            for tag, value in options.items()) + b'\0'


def audio(ssrc, seq, timestamp, sample=b'\x00\x40'):
    return struct.pack('!BBHII', 128, 96, seq, timestamp, ssrc) + sample * 480


class AuditTest(unittest.TestCase):
    def setUp(self):
        tones = ['D031', 'D072', 'I072', '100.8 PL']
        self.rows = [dict(zip(audit.MANIFEST_FIELDS, [
            str(i), label, '453.125000', 'FMN', label, '453.125000|' + label, '']))
                     for i, label in enumerate(tones)]
        self.log = audit.Audit(self.rows, GROUP, {}, 0.06, 0.75)

    def test_cochannel_tones_and_receive_polarity(self):
        self.log.status(status(11, 100), SOURCE, 1)
        self.log.status(status(12, 100, code=72), SOURCE, 1)
        self.log.status(status(13, 100, code=72, inverted=True), SOURCE, 1)
        self.log.status(status(14, 100, code=0, tone=100.8), SOURCE, 1)
        for ssrc in (11, 13, 14):
            for seq in range(3):
                pkt = audio(ssrc, seq, 480 * seq)
                self.log.audio(pkt, SOURCE, 2 + seq * 0.02, STAMP)
                self.log.audio(pkt, SOURCE, 2 + seq * 0.02, STAMP)
        counts = [self.log.stats(row['Channel'])['count'] for row in self.rows]
        self.assertEqual(counts, [1, 0, 1, 1])

    def test_silent_packets_and_timestamp_gap(self):
        self.log.status(status(11, 100), SOURCE, 1)
        for seq in range(4):
            self.log.audio(audio(11, seq, 480 * seq, b'\0\0'), SOURCE, 2, STAMP)
        self.assertEqual(self.log.stats(self.rows[0]['Channel'])['count'], 0)
        for seq in range(4, 10):
            offset = 24000 if seq >= 7 else 0
            self.log.audio(audio(11, seq, 480 * seq + offset), SOURCE, 2, STAMP)
        self.assertEqual(self.log.stats(self.rows[0]['Channel'])['count'], 2)

    def test_coverage_status_loss_and_review(self):
        self.log.status(status(11, 100), SOURCE, 1)
        self.log.status(status(11, 1000), SOURCE, 11)
        self.log.status(status(11, 1000), SOURCE, 21)
        for seq in range(3):
            self.log.audio(audio(11, seq, 480 * seq), SOURCE, 42, STAMP)
        self.log.status(status(11, 2000, center=400000000), SOURCE, 43)
        self.assertFalse(self.log.streams)
        stats = self.log.stats(self.rows[0]['Channel'])
        self.assertEqual((stats['seconds'], stats['count']), (10, 0))
        catalog = audit.load_catalog()
        with tempfile.TemporaryDirectory() as directory:
            self.log.report(catalog, Path(directory))
            rows = catalog.read_csv(Path(directory) / 'review.csv')
        self.assertEqual([row['Result'] for row in rows], ['not heard'] + 3 * ['not monitored'])


if __name__ == '__main__':
    unittest.main()
