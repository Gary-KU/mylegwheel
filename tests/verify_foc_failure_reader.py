"""Offline failure-decoder, stopped-read guards and local AXF/HEX binding tests.

All debugger/target access is fake; these tests never open a probe.
"""
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import read_foc_failure as reader


def snapshot(**overrides):
    values = dict.fromkeys(reader.FIELDS, 0)
    values.update(valid=1, reason=6, node_id=3, tick_ms=123456, active_mask=0x77,
                  pending=1, opcode=1, sequence=91, pending_age_ms=41,
                  ack_age_ms=0xFFFFFFFF, fast_age_ms=4, session_ready=1,
                  mode=3, flags=0xBF, timeout_count=2)
    values.update(overrides)
    return reader.LAYOUT.pack(*(values[name] for name in reader.FIELDS))


def identity(suffix=''):
    return {'cpuid'+suffix: struct.pack('<I', 0x411FC271),
            'device'+suffix: struct.pack('<I', 0x10000483),
            'uid'+suffix: struct.pack('<3I', 0x12345678, 0x90ABCDEF, 7)}


def stopped_data():
    data = identity('_after')
    for name, _ in reader.GUARDS:
        data[name] = data[name+'_after'] = bytes(4)
    data['failure'] = data['failure_after'] = snapshot()
    return data


ADDRESSES = dict(failure=0x20001000,
                 guards={name: 0x20002000+4*i for i, (name, _) in enumerate(reader.GUARDS)})


class FailureDecodeTests(unittest.TestCase):
    def test_all_fields_and_missing_age_keep_original_bits(self):
        report = reader.decode_failure(snapshot())
        self.assertEqual(report['bytes'], 140)
        self.assertEqual(len(report['fields']), 35)
        self.assertEqual(report['fields']['pending_age_ms'], 41)
        self.assertIsNone(report['fields']['ack_age_ms'])
        self.assertEqual(report['raw_fields']['ack_age_ms'], 0xFFFFFFFF)
        self.assertEqual(report['reason_name'], 'ack_timeout')
        self.assertEqual(report['fields']['flags'], 0xBF)
        json.dumps(report, allow_nan=False)

    def test_each_current_reason_and_unknown_future_value(self):
        for reason, (name, _) in reader.REASONS.items():
            self.assertEqual(reader.decode_failure(snapshot(reason=reason))['reason_name'], name)
        self.assertEqual(reader.decode_failure(snapshot(reason=123))['reason_name'], 'unknown')
        self.assertEqual(reader.decode_failure(bytes(140))['fields']['valid'], 0)

    def test_wrong_layout_or_invalid_words_are_rejected(self):
        for data in (bytes(136), bytes(144), snapshot(valid=2), snapshot(node_id=9)):
            with self.subTest(length=len(data)), self.assertRaises(RuntimeError):
                reader.decode_failure(data)


class StoppedReadTests(unittest.TestCase):
    def test_byte_identical_read_with_all_stop_guards(self):
        with patch.object(reader, 'read_blocks', return_value=stopped_data()) as read:
            raw, guards, target = reader.read_stopped_snapshot(ADDRESSES, 'fake-ocd')
        self.assertEqual(raw, snapshot())
        self.assertEqual(guards, dict.fromkeys((name for name, _ in reader.GUARDS), 0))
        self.assertEqual(target['device_id'], '0x483')
        self.assertEqual(target['uid_words'][0], '0x12345678')
        self.assertEqual(read.call_args.args[1], 'fake-ocd')
        self.assertEqual([name for name, _, _ in read.call_args.args[0]].count('failure_after'), 1)

    def test_every_guard_before_and_after_must_be_off(self):
        for name, _ in reader.GUARDS:
            for suffix in ('', '_after'):
                data = stopped_data();data[name+suffix] = struct.pack('<I', 1)
                with self.subTest(guard=name+suffix), patch.object(reader, 'read_blocks', return_value=data):
                    with self.assertRaises(RuntimeError):
                        reader.read_stopped_snapshot(ADDRESSES, 'fake-ocd')

    def test_changed_snapshot_or_wrong_target_is_rejected(self):
        data = stopped_data();data['failure_after'] = snapshot(tx_failed=1)
        with patch.object(reader, 'read_blocks', return_value=data), self.assertRaises(RuntimeError):
            reader.read_stopped_snapshot(ADDRESSES, 'fake-ocd')
        for field, value in (('cpuid_after', 0x411FC231), ('device_after', 0x10000410)):
            data = stopped_data();data[field] = struct.pack('<I', value)
            with patch.object(reader, 'read_blocks', return_value=data), self.assertRaises(RuntimeError):
                reader.read_stopped_snapshot(ADDRESSES, 'fake-ocd')

    def test_image_verification_precedes_ram_and_identity_must_stay_same(self):
        order = []
        def verify(*args):
            order.append('verify')
        def addresses(*args):
            self.assertEqual(order, ['verify']);order.append('symbols');return ADDRESSES
        initial = identity();final = stopped_data();final['uid_after'] = struct.pack('<3I', 9, 8, 7)
        with patch.object(reader, 'local_image_info', return_value={}), \
             patch.object(reader, 'check_firmware', side_effect=verify), \
             patch.object(reader, 'symbols', side_effect=addresses), \
             patch.object(reader, 'read_blocks', side_effect=[initial, final]):
            with self.assertRaisesRegex(RuntimeError, 'identity changed'):
                reader.collect(Path('fake.axf'), Path('fake.hex'), 'fake-ocd')

    def test_gdb_field_offsets_are_checked_not_just_total_size(self):
        values = [0x20001000, 140, *range(0, 140, 4), *ADDRESSES['guards'].values()]
        def stdout(words):
            return '\n'.join(f'${i+1} = {value:#x}' for i, value in enumerate(words))
        with patch.object(reader.subprocess, 'run', return_value=SimpleNamespace(stdout=stdout(values))):
            self.assertEqual(reader.symbols(Path('fake.axf')), ADDRESSES)
        values[3] = 12
        with patch.object(reader.subprocess, 'run', return_value=SimpleNamespace(stdout=stdout(values))):
            with self.assertRaises(RuntimeError):
                reader.symbols(Path('fake.axf'))


def hex_record(address, kind, payload):
    data = bytes([len(payload), address >> 8, address & 255, kind])+payload
    return ':'+(data+bytes([-sum(data) & 255])).hex().upper()


class LocalImageTests(unittest.TestCase):
    def test_all_load_bytes_must_match_the_axf_and_reserved_flash_is_excluded(self):
        with tempfile.TemporaryDirectory(prefix='failure_image_') as directory:
            work = Path(directory);axf = work/'test.axf';image = work/'test.hex'
            payload = bytes(range(32))
            ident = b'\x7fELF\x01\x01'+bytes(10)
            header = struct.pack('<16sHHIIIIIHHHHHH', ident, 2, 40, 1, 0x08000001,
                                 52, 0, 0, 52, 32, 1, 0, 0, 0)
            segment = struct.pack('<8I', 1, 256, 0x08000000, 0x08000000, 32, 32, 5, 4)
            axf.write_bytes((header+segment).ljust(256, b'\0')+payload)
            def write_hex(data, upper=0x0800):
                image.write_text('\n'.join((hex_record(0, 4, upper.to_bytes(2, 'big')),
                    hex_record(0, 0, data), hex_record(0, 1, b'')))+'\n', encoding='ascii')
            write_hex(payload)
            info = reader.local_image_info(axf, image)
            self.assertEqual(info['application_bytes'], 32)
            self.assertEqual(info['flash_last'], '0x800001f')
            write_hex(payload[:-1]+b'\xff')
            with self.assertRaisesRegex(RuntimeError, 'load bytes differ'):
                reader.local_image_info(axf, image)
            write_hex(payload, 0x080C)
            with self.assertRaises(RuntimeError):
                reader.local_image_info(axf, image)
            write_hex(payload);text = image.read_text();image.write_text(text[:12]+'00'+text[14:])
            with self.assertRaises(RuntimeError):
                reader.local_image_info(axf, image)


if __name__ == '__main__':
    unittest.main()
