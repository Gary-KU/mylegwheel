"""Offline64/128/180-byte balance-ring decoder and stopped-export regressions.

Fixtures use independent byte offsets. The real C row layout is also compiled
with offsetof assertions. All target/debugger calls are mocked; no probe opens.
"""
import json
import ast
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import read_balance_ring as reader

PREFIX_OFFSETS = {'tick_ms': 0, 'flags': 4, 'state': 8, 'position_error_m': 32,
    'wheel_requested_nm': 36, 'wheel_applied_nm': 40, 'leg_tp_nm': 44,
    'motion_component_nm': 48, 'wheel_voltage_mv': 56, 'leg_length_m': 60}
EXTENSION_OFFSETS = {'raw_ch2': 64, 'motion_status': 66, 'input_flags': 67,
    'source_frame_count': 68, 'sbus_age_ms': 72, 'normalized_forward': 76,
    'requested_forward_m_s': 80, 'slewed_forward_m_s': 84, 'reference': 88,
    'parking_goal_m': 104, 'bias_offset_m': 108, 'bias_applied_phi_rad': 112,
    'bias_gate_flags': 116, 'requested_yaw_rad_s': 120, 'slewed_yaw_rad_s': 124}
SERVO_EXTENSION_OFFSETS = {'servo_integral_next_m_s': 128, 'servo_blend': 132,
    'servo_wheel_integral_nm': 136, 'servo_leg_integral_nm': 140,
    'servo_motion_flags': 144, 'leg_angle_rad': 148, 'leg_angle_speed_rad_s': 156,
    'leg_lengths_m': 164, 'measured_leg_tp_nm': 172, 'parking_blend': 176}
GUARD_NAMES = ('robot', 'balance', 'dual', 'wheel_test', 'wheel_zero', 'ch5', 'storage')
ADDRESSES = dict(trace=0x24001000, row_bytes=128, capacity=256,
    guards={name: 0x24010000+4*i for i, name in enumerate(GUARD_NAMES)})


def row_bytes(size=128, tick=1000, **overrides):
    # Deliberately not reader.PREFIX/EXTENSION.pack: catch a decoder format shift.
    data = bytearray(size)
    fields = {'tick_ms': ('I', 0, tick), 'flags': ('I', 4, 0x311),
        'state': ('6f', 8, [.125, -.25, 1.5, -.5, .03125, -.0625]),
        'position_error_m': ('f', 32, -.25), 'T_request_nm': ('f', 36, .125),
        'T_applied_nm': ('f', 40, .0625), 'Tp_nm': ('f', 44, -.03125),
        'dtheta_component_nm': ('f', 48, .015625), 'dphi_component_nm': ('f', 52, -.0078125),
        'wheel_voltage_mv': ('2h', 56, [-1234, 2345]), 'length_m': ('f', 60, .125)}
    if size >= 128:
        fields.update({'raw_ch2': ('H', 64, 1698), 'motion_status': ('B', 66, 4),
            'input_flags': ('B', 67, 255), 'source_frame_count': ('I', 68, 0x89abcdef),
            'sbus_age_ms': ('I', 72, 17), 'normalized_forward': ('f', 76, -.75),
            'requested_forward_m_s': ('f', 80, -.125), 'slewed_forward_m_s': ('f', 84, -.0625),
            'reference': ('4f', 88, [.25, 3.5, -.125, -.0625]),
            'parking_goal_m': ('f', 104, 4.), 'bias_offset_m': ('f', 108, .0009765625),
            'bias_applied_phi_rad': ('f', 112, .015625), 'bias_gate_flags': ('I', 116, 0xabcd1234),
            'requested_yaw_rad_s': ('f', 120, .375), 'slewed_yaw_rad_s': ('f', 124, .25)})
    if size == 180:
        fields.update({'servo_integral_next_m_s': ('f', 128, -.125),
            'servo_blend': ('f', 132, .5), 'servo_wheel_integral_nm': ('f', 136, -.0078125),
            'servo_leg_integral_nm': ('f', 140, .00390625),
            'servo_motion_flags': ('I', 144, 31|32|64|128|(2<<8)|1024|2048|4096),
            'leg_angle_rad': ('2f', 148, [1.5,1.625]),
            'leg_angle_speed_rad_s': ('2f', 156, [-.25,.5]),
            'leg_lengths_m': ('2f', 164, [.125,.0625]),
            'measured_leg_tp_nm': ('f', 172, -.03125), 'parking_blend': ('f', 176, .75)})
    for name, (kind, at, value) in fields.items():
        value = overrides.get(name, value)
        struct.pack_into('<'+kind, data, at, *(value if isinstance(value, (list, tuple)) else [value]))
    return bytes(data)


def ring(size=128, count=2, next_row=None, sequence=901, **overrides):
    next_row = count % 256 if next_row is None else next_row
    raw = bytearray(256*size)
    period_ms = 80 if size == 180 else 20
    for i in range(count):
        slot = (next_row-count+i) % 256
        raw[slot*size:(slot+1)*size] = row_bytes(size, tick=1000+i*period_ms, **overrides)
    return struct.pack('<3I', next_row, count, sequence), bytes(raw)


def identity(suffix=''):
    return {'cpuid'+suffix: struct.pack('<I', 0x411fc271),
        'device'+suffix: struct.pack('<I', 0x10000483),
        'uid'+suffix: struct.pack('<3I', 0x12345678, 0x90abcdef, 7)}


def stopped_data(size=128):
    header, raw = ring(size)
    data = identity('_after')
    data.update(header=header, header_after=header, rows=raw)
    for name in GUARD_NAMES:
        data[name] = data[name+'_after'] = bytes(4)
    return data


def gdb_output(words):
    return '\n'.join(f'${i+1} = {word:#x}' for i, word in enumerate(words))


def hex_record(address, kind, payload):
    data = bytes([len(payload), address >> 8, address & 255, kind])+payload
    return ':'+(data+bytes([-sum(data) & 255])).hex().upper()


def image_pair(directory):
    directory = Path(directory)
    axf, image = directory/'test.axf', directory/'test.hex'
    text, initialized_ram = bytes(range(32)), bytes(range(128, 144))
    header = struct.pack('<16sHHIIIIIHHHHHH', b'\x7fELF\x01\x01'+bytes(10),
        2, 40, 1, 0x08000001, 52, 0, 0, 52, 32, 2, 0, 0, 0)
    code = struct.pack('<8I', 1, 256, 0x08000000, 0x08000000, 32, 32, 5, 4)
    # Initialised RAM has a virtual RAM address but load bytes in Flash.
    ram = struct.pack('<8I', 1, 288, 0x24000000, 0x08000020, 16, 32, 6, 4)
    axf.write_bytes((header+code+ram).ljust(256, b'\0')+text+initialized_ram)
    def write(payload=text+initialized_ram):
        image.write_text('\n'.join([hex_record(0, 4, b'\x08\x00'),
            hex_record(0, 0, payload), hex_record(0, 1, b'')])+'\n', encoding='ascii')
    write()
    return axf, image, text+initialized_ram, write


class DecodeTests(unittest.TestCase):
    def test_servo_rows_preserve_prefix_and_record_post_update_integral(self):
        report = reader.decode_ring(*ring(180),180)
        modern, legacy = report['records'][0], reader.decode_ring(*ring(128),128)['records'][0]
        for name,value in legacy.items():
            if value is not None:
                self.assertEqual(modern[name],value,name)
        expected = dict(servo_integral_next_m_s=-.125, servo_blend=.5,
            servo_wheel_integral_nm=-.0078125, servo_leg_integral_nm=.00390625,
            servo_motion_flags=31|32|64|128|(2<<8)|1024|2048|4096,
            servo_gate_mask=31, servo_enabled=True, servo_active=True, servo_learning=True,
            motion_reference_phase=2, motion_reference_phase_name='braking',
            motion_reference_enabled=True, joint_saturated=True,
            support_saturated=[True,False], leg_angle_rad=[1.5,1.625],
            leg_angle_speed_rad_s=[-.25,.5], leg_lengths_m=[.125,.0625],
            measured_leg_tp_nm=-.03125, parking_blend=.75)
        for name,value in expected.items():
            self.assertEqual(modern[name],value,name)
        self.assertEqual((report['nominal_period_ms'],report['nominal_capacity_s']), (80,20.48))
        self.assertEqual(report['observed_duration_ms'],80)
        self.assertEqual(report['last_tick_ms'],1080)
        for name in expected:
            self.assertIsNone(legacy[name],name)
        json.dumps(report,allow_nan=False)

    def test_servo_unknown_phase_nan_and_individual_saturation_flags(self):
        overrides = dict(servo_integral_next_m_s=math.nan, servo_blend=math.inf,
            servo_motion_flags=(3<<8)|8192, leg_angle_rad=[math.nan,1.5],
            measured_leg_tp_nm=math.nan)
        row = reader.decode_ring(*ring(180,count=1,**overrides),180)['records'][0]
        self.assertIsNone(row['servo_integral_next_m_s'])
        self.assertIsNone(row['servo_blend'])
        self.assertIsNone(row['measured_leg_tp_nm'])
        self.assertEqual(row['leg_angle_rad'],[None,1.5])
        self.assertEqual(row['motion_reference_phase_name'],'unknown')
        self.assertEqual(row['support_saturated'],[False,True])
        self.assertFalse(row['joint_saturated'])
        self.assertFalse(row['servo_active'])

    def test_all_extended_fields_and_exact_prefix_values(self):
        header, raw = ring()
        report = reader.decode_ring(header, raw, 128)
        row = report['records'][0]
        self.assertEqual((report['count'], report['sequence'], report['next']), (2, 901, 2))
        self.assertEqual((report['first_tick_ms'], report['last_tick_ms']), (1000, 1020))
        expected = {'tick_ms': 1000, 'flags': 0x311, 'phase': 3,
            'state': [.125, -.25, 1.5, -.5, .03125, -.0625], 'position_error_m': -.25,
            'T_request_nm': .125, 'T_applied_nm': .0625, 'Tp_nm': -.03125,
            'dtheta_component_nm': .015625, 'dphi_component_nm': -.0078125,
            'wheel_voltage_mv': [-1234, 2345], 'length_m': .125, 'position_locked': True,
            'raw_ch2': 1698, 'motion_status': 4, 'motion_status_name': 'target_ready',
            'input_flags': 255, 'source_frame_count': 0x89abcdef,
            'sbus_age_ms': 17, 'sbus_age_ms_raw': 17, 'normalized_forward': -.75,
            'requested_forward_m_s': -.125, 'slewed_forward_m_s': -.0625,
            'reference': [.25, 3.5, -.125, -.0625], 'parking_goal_m': 4.,
            'bias_offset_m': .0009765625, 'bias_applied_phi_rad': .015625,
            'bias_gate_flags': 0xabcd1234, 'requested_yaw_rad_s': .375, 'slewed_yaw_rad_s': .25}
        for name, value in expected.items():
            with self.subTest(field=name):
                self.assertEqual(row[name], value)
        self.assertAlmostEqual(row['body_pitch_deg'], -math.degrees(.03125))
        self.assertAlmostEqual(row['body_rate_deg_s'], math.degrees(.0625))
        self.assertEqual(report['max_abs_speed_m_s'], .5)
        self.assertTrue(all(row['input_permissions'].values()))
        json.dumps(report, allow_nan=False)

    def test_legacy_fields_are_preserved_and_new_fields_are_unknown(self):
        legacy = reader.decode_ring(*ring(64), 64)
        modern = reader.decode_ring(*ring(128), 128)
        old, new = legacy['records'][0], modern['records'][0]
        for field in ('tick_ms','flags','phase','state','position_error_m','T_request_nm','T_applied_nm',
            'Tp_nm','dtheta_component_nm','dphi_component_nm','wheel_voltage_mv','length_m',
            'body_pitch_deg','body_rate_deg_s'):
            self.assertEqual(old[field], new[field], field)
        for field in set(new)-set(('tick_ms','flags','phase','state','position_error_m','T_request_nm',
            'T_applied_nm','Tp_nm','dtheta_component_nm','dphi_component_nm','wheel_voltage_mv',
            'length_m','body_pitch_deg','body_rate_deg_s')):
            self.assertIsNone(old[field], field)
        json.dumps(legacy, allow_nan=False)

    def test_wrapped_full_ring_and_partially_filled_order(self):
        for size in (64, 128, 180):
            period_ms = 80 if size == 180 else 20
            report = reader.decode_ring(*ring(size, count=256, next_row=7), size)
            self.assertEqual([r['tick_ms'] for r in report['records']],
                [1000+i*period_ms for i in range(256)])
            self.assertEqual(report['observed_duration_ms'],255*period_ms)
            partial = reader.decode_ring(*ring(size, count=3), size)
            self.assertEqual([r['tick_ms'] for r in partial['records']],
                [1000,1000+period_ms,1000+2*period_ms])

    def test_invalid_source_sentinels_and_nonfinite_values_become_json_null(self):
        header, raw = ring(count=1, raw_ch2=65535, sbus_age_ms=0xffffffff,
            normalized_forward=math.nan, requested_forward_m_s=math.nan,
            reference=[math.nan, math.inf, -math.inf, .125], bias_offset_m=math.nan,
            bias_applied_phi_rad=math.inf, requested_yaw_rad_s=-math.inf,
            input_flags=0, motion_status=255, bias_gate_flags=0xffffffff)
        report = reader.decode_ring(header, raw, 128);row = report['records'][0]
        for name in ('raw_ch2','sbus_age_ms','normalized_forward','requested_forward_m_s',
                     'bias_offset_m','bias_applied_phi_rad','requested_yaw_rad_s'):
            self.assertIsNone(row[name], name)
        self.assertEqual(row['sbus_age_ms_raw'], 0xffffffff)
        self.assertEqual(row['reference'], [None, None, None, .125])
        self.assertEqual(row['bias_gate_flags'], 0xffffffff)
        self.assertEqual(row['motion_status_name'], 'unknown')
        self.assertFalse(any(row['input_permissions'].values()))
        json.dumps(report, allow_nan=False)

    def test_gate_failure_preserves_raw_input_without_inventing_accepted_motion(self):
        header, raw = ring(count=1, raw_ch2=1700, normalized_forward=1.,
            input_flags=3, motion_status=3, requested_forward_m_s=0., slewed_forward_m_s=0.,
            requested_yaw_rad_s=0., slewed_yaw_rad_s=0., bias_gate_flags=0, flags=0x301)
        row = reader.decode_ring(header, raw, 128)['records'][0]
        self.assertEqual((row['raw_ch2'],row['normalized_forward']), (1700,1.))
        self.assertEqual(row['motion_status_name'], 'not_ready')
        self.assertTrue(row['input_permissions']['valid'])
        self.assertTrue(row['input_permissions']['calibrated'])
        self.assertFalse(row['input_permissions']['command_ready'])
        self.assertEqual((row['requested_forward_m_s'],row['slewed_forward_m_s']), (0.,0.))
        self.assertEqual(row['bias_gate_flags'], 0)
        self.assertFalse(row['position_locked'])

    def test_bad_lengths_and_header_invariants_are_rejected(self):
        header, raw = ring()
        cases = [(header[:-1],raw,128),(header+b'\0',raw,128),(header,raw[:-1],128),
            (header,raw,96),(header,raw,64)]
        for words in ((256,2,1),(2,0,1),(1,257,1),(3,2,1)):
            cases.append((struct.pack('<3I',*words),raw,128))
        for args in cases:
            with self.subTest(header=args[0], size=args[2]), self.assertRaises(RuntimeError):
                reader.decode_ring(*args)


class LayoutTests(unittest.TestCase):
    def test_real_c_layout_and_independent_decoder_offsets(self):
        self.assertEqual(reader.PREFIX.size, 64)
        self.assertEqual(reader.EXTENSION.size, 64)
        self.assertEqual(reader.SERVO_EXTENSION.size, 52)
        self.assertEqual(reader.PREFIX_FIELDS, PREFIX_OFFSETS)
        self.assertEqual(reader.EXTENSION_FIELDS, EXTENSION_OFFSETS)
        self.assertEqual(reader.SERVO_EXTENSION_FIELDS,SERVO_EXTENSION_OFFSETS)
        fields = dict(PREFIX_OFFSETS, **EXTENSION_OFFSETS, **SERVO_EXTENSION_OFFSETS)
        source = '#include <stddef.h>\n#include "wheel_leg_task.h"\n'
        source += '_Static_assert(sizeof(WheelBalanceTraceRow)==180,"row size");\n'
        source += '_Static_assert(offsetof(WheelBalanceTraceDebug,rows)==12,"header");\n'
        source += '_Static_assert(sizeof(((WheelBalanceTraceDebug*)0)->rows)/sizeof(WheelBalanceTraceRow)==256,"capacity");\n'
        for name, offset in fields.items():
            source += f'_Static_assert(offsetof(WheelBalanceTraceRow,{name})=={offset},"{name}");\n'
        source += '_Static_assert(WHEEL_TRACE_INPUT_VALID==1U && WHEEL_TRACE_COMMAND_READY==128U,"input bits");\nint main(void){return 0;}\n'
        source += '_Static_assert(WHEEL_BALANCE_TRACE_PERIOD_MS==80U,"sample period");\n'
        source += '_Static_assert(WHEEL_TRACE_SERVO_ENABLED==32U && WHEEL_TRACE_SERVO_ACTIVE==64U && WHEEL_TRACE_SERVO_LEARNING==128U,"servo bits");\n'
        source += '_Static_assert(WHEEL_TRACE_MOTION_PHASE_SHIFT==8U && WHEEL_TRACE_MOTION_PHASE_MASK==768U && WHEEL_TRACE_MOTION_ENABLED==1024U,"governor bits");\n'
        source += '_Static_assert(WHEEL_TRACE_JOINT_SATURATED==2048U && WHEEL_TRACE_SUPPORT_LEFT_SATURATED==4096U && WHEEL_TRACE_SUPPORT_RIGHT_SATURATED==8192U,"saturation bits");\n'
        with tempfile.TemporaryDirectory(prefix='balance_layout_') as directory:
            code = Path(directory)/'layout.c';code.write_text(source,encoding='ascii')
            result = subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fsyntax-only',
                '-I',str(ROOT/'App/Tasks/Inc'),'-I',str(ROOT/'App/Protocol/Inc'),
                '-I',str(ROOT/'Control/WheelLeg/Inc'),str(code)], capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_gdb_validates_every_field_for_both_layouts(self):
        for size in (64,128,180):
            fields = dict(PREFIX_OFFSETS)
            if size >= 128:
                fields.update(EXTENSION_OFFSETS)
            if size == 180:
                fields.update(SERVO_EXTENSION_OFFSETS)
            first = [ADDRESSES['trace'],size,256,12,*ADDRESSES['guards'].values()]
            response = [SimpleNamespace(stdout=gdb_output(first)),
                SimpleNamespace(stdout=gdb_output(fields.values()))]
            with patch.object(reader.subprocess,'run',side_effect=response) as run:
                actual = reader.symbols(Path('fake.axf'),'fake-gdb')
            self.assertEqual(actual,dict(ADDRESSES,row_bytes=size))
            self.assertIn('p sizeof(WheelBalanceTraceRow)',run.call_args_list[0].args[0])
            expressions = run.call_args_list[1].args[0]
            for name in fields:
                self.assertIn(f'p (unsigned long)&((WheelBalanceTraceRow *)0)->{name}',expressions)
            for index in range(len(fields)):
                wrong = list(fields.values());wrong[index] += 4
                response = [SimpleNamespace(stdout=gdb_output(first)),SimpleNamespace(stdout=gdb_output(wrong))]
                with self.subTest(size=size,index=index),patch.object(reader.subprocess,'run',side_effect=response):
                    with self.assertRaisesRegex(RuntimeError,'offsets'):
                        reader.symbols(Path('fake.axf'))

    def test_wrong_size_capacity_header_or_missing_symbols_rejected(self):
        first = [ADDRESSES['trace'],128,256,12,*ADDRESSES['guards'].values()]
        for index,value in ((1,96),(2,128),(3,16)):
            wrong = first.copy();wrong[index] = value
            with patch.object(reader.subprocess,'run',return_value=SimpleNamespace(stdout=gdb_output(wrong))):
                with self.assertRaises(RuntimeError):
                    reader.symbols(Path('fake.axf'))
        with patch.object(reader.subprocess,'run',return_value=SimpleNamespace(stdout=gdb_output(first[:-1]))):
            with self.assertRaisesRegex(RuntimeError,'Missing'):
                reader.symbols(Path('fake.axf'))

    def test_production_capture_matches_same_tick_and_freezes_restarts(self):
        # Reuse only prescribed sensor/transport fixtures; compile the real
        # controller and capture function as separate production sources.
        def strings(path):
            return {node.targets[0].id: ast.literal_eval(node.value)
                for node in ast.parse(path.read_text(encoding='utf-8')).body
                if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name)
                and isinstance(node.value,ast.Constant) and isinstance(node.value.value,str)}
        base = strings(ROOT/'tests/verify_wheel_leg_control.py')
        governor = strings(ROOT/'tests/verify_native_motion_governor.py')['fixture']
        helpers = governor.split('static void neutral_stop(float sign)')[0]
        fixture = r'''
static void trace_equal(float a,float b) {assert(a==b || (isnan(a)&&isnan(b)));}
int main(void) {
 configure(0);g_balance_servo.enable=1;
 unsigned captures=0,last_capture=0;
 for(unsigned n=0;n<4200;++n) {
  velocity(n%200<100 ? .1f : -.1f);
  uint32_t sequence=g_balance_trace.sequence;
  active_step();
  if(g_balance_trace.sequence==sequence)continue;
  assert(g_balance_trace.sequence==sequence+1U);
  const volatile WheelBalanceTraceRow *r=&g_balance_trace.rows[
      (g_balance_trace.next+WHEEL_BALANCE_TRACE_ROWS-1U)%WHEEL_BALANCE_TRACE_ROWS];
  assert(r->tick_ms==g_wheel_leg.tick_ms);
  if(captures)assert(r->tick_ms-last_capture==80U);
  last_capture=r->tick_ms;++captures;
  trace_equal(r->servo_integral_next_m_s,g_balance_servo.integral_m_s);
  trace_equal(r->servo_blend,g_balance_servo.blend);
  trace_equal(r->servo_wheel_integral_nm,g_balance_servo.wheel_integral_nm);
  trace_equal(r->servo_leg_integral_nm,g_balance_servo.leg_integral_nm);
  trace_equal(r->parking_blend,g_balance_servo.parking_blend);
  assert((r->servo_motion_flags&31U)==g_balance_servo.gate_mask);
  assert(!!(r->servo_motion_flags&32U)==!!g_balance_servo.enable);
  assert(!!(r->servo_motion_flags&64U)==!!g_balance_servo.active);
  assert(!!(r->servo_motion_flags&128U)==!!g_balance_servo.learning);
  assert(((r->servo_motion_flags>>8)&3U)==g_balance_motion_reference.phase);
  assert(!!(r->servo_motion_flags&1024U)==!!g_balance_motion_reference.enable);
  assert(!!(r->servo_motion_flags&2048U)==!!g_joint_output.saturated);
  assert(!!(r->servo_motion_flags&4096U)==!!g_dual_hold.force_saturated[0]);
  assert(!!(r->servo_motion_flags&8192U)==!!g_dual_hold.force_saturated[1]);
  for(unsigned side=0;side<2;++side) {
   trace_equal(r->leg_angle_rad[side],g_wheel_leg.leg[side].angle);
   trace_equal(r->leg_angle_speed_rad_s[side],g_wheel_leg.angle_speed_rad_s[side]);
   trace_equal(r->leg_lengths_m[side],g_wheel_leg.leg[side].length);
  }
  trace_equal(r->measured_leg_tp_nm,g_balance_bias.actual_pitch_torque_nm);
 }
 assert(captures>WHEEL_BALANCE_TRACE_ROWS && g_balance_trace.count==WHEEL_BALANCE_TRACE_ROWS);
 WheelBalanceTraceDebug frozen=g_balance_trace;
 disarm();
 assert(memcmp(&frozen,(const void*)&g_balance_trace,sizeof(frozen))==0);
 assert(g_balance_servo.integral_m_s==0 && g_balance_servo.parking_blend==0);
 configure(0);g_balance_servo.enable=1;active_step();
 assert(g_balance_trace.count==1U && g_balance_trace.next==1U);
 assert(g_balance_trace.sequence==frozen.sequence+1U);
 return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='balance_capture_') as directory:
            work = Path(directory)
            (work/'runtime.h').write_text(base['runtime'],encoding='utf-8')
            for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):
                (work/name).write_text('#include "runtime.h"\n',encoding='ascii')
            code = work/'test.c'
            code.write_text(base['harness'].split('int main(void) {')[0]+helpers+fixture,encoding='utf-8')
            command = ['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',
                '-DROBOT_NATIVE_LQR=1','-DROBOT_LEGS_ONLY_CALIBRATION=0','-DMOTOR_COMMAND_TX_ENABLED=1',
                '-I',str(work)]
            for folder in ('App/Tasks/Inc','App/Services/Inc','App/Protocol/Inc',
                           'Control/WheelLeg/Inc','tests/support/wheel_leg/Inc'):
                command += ['-I',str(ROOT/folder)]
            sources = ('App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c',
                'App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_command.c',
                'tests/support/wheel_leg/Src/wheel_leg_control.c','Control/WheelLeg/Src/lqr_k.c',
                'Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c')
            executable = work/'capture.exe'
            result = subprocess.run(command+[str(code),*[str(ROOT/path) for path in sources],
                '-lm','-o',str(executable)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            result = subprocess.run([str(executable)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)


class CaptureTests(unittest.TestCase):
    def test_stopped_capture_requests_stable_headers_guards_and_target_identity(self):
        for size in (64,128,180):
            data=stopped_data(size);addresses=dict(ADDRESSES,row_bytes=size)
            with patch.object(reader,'read_blocks',return_value=data) as read:
                header,raw,guards,target=reader.read_stopped_ring(addresses,'fake-ocd')
            self.assertEqual((header,raw),(data['header'],data['rows']))
            self.assertEqual(guards,dict.fromkeys(GUARD_NAMES,0))
            self.assertEqual(target['device_id'],'0x483')
            blocks=read.call_args.args[0]
            self.assertEqual(read.call_args.args[1],'fake-ocd')
            self.assertIn(('rows',ADDRESSES['trace']+12,256*size//4),blocks)
            self.assertLess([x[0] for x in blocks].index('header'),[x[0] for x in blocks].index('rows'))
            self.assertLess([x[0] for x in blocks].index('rows'),[x[0] for x in blocks].index('header_after'))
            for name in GUARD_NAMES:
                self.assertEqual(sum(x[0]==name for x in blocks),1)
                self.assertEqual(sum(x[0]==name+'_after' for x in blocks),1)

    def test_each_guard_and_changed_header_reject_capture(self):
        for name in GUARD_NAMES:
            for suffix in ('','_after'):
                data=stopped_data();data[name+suffix]=struct.pack('<I',1)
                with self.subTest(guard=name+suffix),patch.object(reader,'read_blocks',return_value=data):
                    with self.assertRaisesRegex(RuntimeError,'guard'):
                        reader.read_stopped_ring(ADDRESSES,'fake-ocd')
        data=stopped_data();data['header_after']=struct.pack('<3I',2,2,902)
        with patch.object(reader,'read_blocks',return_value=data),self.assertRaisesRegex(RuntimeError,'headers'):
            reader.read_stopped_ring(ADDRESSES,'fake-ocd')

    def test_wrong_mcu_before_or_after_and_swapped_uid_are_rejected(self):
        for field,value in (('cpuid_after',0x411fc231),('device_after',0x10000410)):
            data=stopped_data();data[field]=struct.pack('<I',value)
            with patch.object(reader,'read_blocks',return_value=data),self.assertRaisesRegex(RuntimeError,'Cortex-M7'):
                reader.read_stopped_ring(ADDRESSES,'fake-ocd')
        final=stopped_data();final['uid_after']=struct.pack('<3I',9,8,7)
        with patch.object(reader,'local_image_info',return_value={}),patch.object(reader,'check_firmware'), \
             patch.object(reader,'symbols',return_value=ADDRESSES), \
             patch.object(reader,'read_blocks',side_effect=[identity(),final]):
            with self.assertRaisesRegex(RuntimeError,'identity changed'):
                reader.collect(Path('fake.axf'),Path('fake.hex'),'fake-ocd')
        wrong=identity();wrong['device']=struct.pack('<I',0x10000410)
        with patch.object(reader,'local_image_info',return_value={}), \
             patch.object(reader,'read_blocks',return_value=wrong),patch.object(reader,'check_firmware') as verify:
            with self.assertRaises(RuntimeError):
                reader.collect(Path('fake.axf'),Path('fake.hex'),'fake-ocd')
            verify.assert_not_called()

    def test_complete_image_verification_precedes_ram_symbols_and_success_is_bound(self):
        events=[]
        with tempfile.TemporaryDirectory(prefix='balance_image_') as directory:
            axf,image,payload,_=image_pair(directory)
            def read(blocks,ocd):
                self.assertEqual(ocd,'fake-ocd')
                if blocks == reader.IDENTITY_BLOCKS:
                    events.append('identity');return identity()
                self.assertEqual(events,['identity','verify','symbols']);events.append('ring')
                return stopped_data()
            def verify(path,ocd):
                self.assertEqual((path,ocd),(image,'fake-ocd'));events.append('verify')
            def symbols(path,gdb):
                self.assertEqual((path,gdb),(axf,'fake-gdb'))
                self.assertEqual(events,['identity','verify']);events.append('symbols');return ADDRESSES
            with patch.object(reader,'read_blocks',side_effect=read),patch.object(reader,'check_firmware',side_effect=verify), \
                 patch.object(reader,'symbols',side_effect=symbols):
                report=reader.collect(axf,image,'fake-ocd','fake-gdb')
            self.assertEqual(events,['identity','verify','symbols','ring'])
            self.assertTrue(report['full_application_hex_verified'])
            self.assertEqual(report['image']['application_bytes'],len(payload))
            self.assertEqual(report['image']['axf'],str(axf.resolve()))
            self.assertEqual(report['target']['uid_words'][0],'0x12345678')
            self.assertEqual(report['reference_order'],['theta_rad','x_m','dx_m_s','phi_rad'])
            json.dumps(report,allow_nan=False)

    def test_wrong_target_flash_image_prevents_any_ring_interpretation(self):
        with patch.object(reader,'local_image_info',return_value={}), \
             patch.object(reader,'read_blocks',return_value=identity()) as read, \
             patch.object(reader,'check_firmware',side_effect=RuntimeError('HEX differs')), \
             patch.object(reader,'symbols') as symbols,patch.object(reader,'decode_ring') as decode:
            with self.assertRaisesRegex(RuntimeError,'HEX differs'):
                reader.collect(Path('fake.axf'),Path('fake.hex'),'fake-ocd')
            self.assertEqual(read.call_count,1)
            symbols.assert_not_called();decode.assert_not_called()

    def test_axf_hex_binding_includes_ram_load_bytes_before_any_probe_access(self):
        with tempfile.TemporaryDirectory(prefix='balance_wrong_image_') as directory:
            axf,image,payload,write=image_pair(directory)
            self.assertEqual(reader.local_image_info(axf,image)['application_bytes'],48)
            write(payload[:-1]+b'\xff')
            with patch.object(reader,'read_blocks') as read,patch.object(reader,'symbols') as symbols:
                with self.assertRaisesRegex(RuntimeError,'load bytes differ'):
                    reader.collect(axf,image,'fake-ocd')
                read.assert_not_called();symbols.assert_not_called()


if __name__ == '__main__':
    unittest.main()
