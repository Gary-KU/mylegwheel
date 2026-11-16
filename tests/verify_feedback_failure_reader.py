"""Verify attribution decoding and fail-closed stopped capture; no probe access."""
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import read_feedback_failure as reader


def snapshot(**overrides):
    values = dict.fromkeys(reader.FIELDS, 0)
    values.update(valid=1, schema_version=1, controller_tick_ms=130197,
                  feedback_detail=2, observer_age_ms=6)
    values.update({'sample.valid': 1, 'sample.motor_id': 5,
                   'sample.sample_tick_ms': 130197, 'sample.fast_age_ms': 6,
                   'sample.status_age_ms': 41, 'sample.diagnostic_age_ms': 7,
                   'sample.reject_mask': 1 << 2})
    values.update(overrides)
    return reader.LAYOUT.pack(*(values[name] for name in reader.FIELDS))


def identity(suffix=''):
    return {'cpuid' + suffix: struct.pack('<I', 0x411fc272),
            'device' + suffix: struct.pack('<I', 0x10016483),
            'uid' + suffix: struct.pack('<3I', 0x002b0038, 0x3233510f, 0x31333635)}


def stopped_data():
    data = identity('_after')
    data.update(failure=snapshot(), failure_after=snapshot())
    for name, _ in reader.GUARDS:
        data[name] = data[name + '_after'] = bytes(4)
    return data


ADDRESSES = dict(failure=0x20001000,
    guards={name: 0x20002000 + 4*i for i, (name, _) in enumerate(reader.GUARDS)})


class ReaderTests(unittest.TestCase):
    def test_fast6_does_not_hide_status41_and_id5_attribution(self):
        report = reader.decode_snapshot(snapshot())
        self.assertEqual(report['sample']['motor_id'], 5)
        self.assertEqual(report['sample']['fast_age_ms'], 6)
        self.assertEqual(report['sample']['status_age_ms'], 41)
        self.assertEqual(report['online_reject_reasons'], ['status_stale'])
        self.assertEqual(report['fields']['controller_tick_ms'], 130197)
        self.assertEqual(report['feedback_detail_name'], 'motor_offline')

    def test_multiple_reasons_unknown_bits_and_post_group_stale(self):
        report = reader.decode_snapshot(snapshot(**{'sample.reject_mask': (1<<4)|(1<<16)|(1<<24)}))
        self.assertEqual(report['online_reject_reasons'], ['diagnostic_stale', 'communication_blocked'])
        self.assertEqual(report['unknown_reject_bits'], 1<<24)
        # The observer can reject at the group-end age although the original
        # GetFeedback qualification passed. Keep both ages and a zero mask.
        report = reader.decode_snapshot(snapshot(feedback_detail=3, observer_age_ms=11,
            **{'sample.reject_mask': 0, 'sample.fast_age_ms': 10}))
        self.assertEqual(report['online_reject_reasons'], [])
        self.assertEqual(report['fields']['observer_age_ms'], 11)
        self.assertEqual(report['sample']['fast_age_ms'], 10)

    def test_absent_or_invalid_record(self):
        self.assertEqual(reader.decode_snapshot(bytes(reader.LAYOUT.size))['fields']['valid'], 0)
        for data in (snapshot(valid=2), snapshot(schema_version=2),
                     snapshot(**{'sample.valid': 0}), snapshot(**{'sample.motor_id': 9}),
                     bytes(reader.LAYOUT.size-4), bytes(reader.LAYOUT.size+4)):
            with self.subTest(length=len(data)), self.assertRaises(RuntimeError):
                reader.decode_snapshot(data)

    def test_stable_stop_capture_and_each_guard(self):
        with patch.object(reader, 'read_blocks', return_value=stopped_data()):
            raw, guards, target = reader.read_stopped_snapshot(ADDRESSES, 'fake-ocd')
        self.assertEqual(raw, snapshot())
        self.assertTrue(all(value == 0 for value in guards.values()))
        self.assertEqual(target['device_id'], '0x483')
        for name, _ in reader.GUARDS:
            for suffix in ('', '_after'):
                data = stopped_data()
                data[name+suffix] = struct.pack('<I', 1)
                with self.subTest(guard=name+suffix), patch.object(reader, 'read_blocks', return_value=data):
                    with self.assertRaises(RuntimeError):
                        reader.read_stopped_snapshot(ADDRESSES, 'fake-ocd')
        data = stopped_data()
        data['failure_after'] = snapshot(**{'sample.motor_id': 6})
        with patch.object(reader, 'read_blocks', return_value=data), self.assertRaises(RuntimeError):
            reader.read_stopped_snapshot(ADDRESSES, 'fake-ocd')

    def test_image_mismatch_aborts_before_ram_or_symbols(self):
        with patch.object(reader, 'local_image_info', return_value={}), \
             patch.object(reader, 'read_blocks', return_value=identity()), \
             patch.object(reader, 'check_firmware', side_effect=RuntimeError('mismatched image')), \
             patch.object(reader, 'symbols') as symbols, \
             patch.object(reader, 'read_stopped_snapshot') as read:
            with self.assertRaisesRegex(RuntimeError, 'mismatched image'):
                reader.collect(Path('fake.axf'), Path('fake.hex'))
        symbols.assert_not_called()
        read.assert_not_called()


if __name__ == '__main__':
    unittest.main()
