"""Export the latest frozen balance ring, without halt/reset/target writes.

Pass matching AXF/HEX files. Legacy 64/128-byte rows retain 5.12 seconds at20ms;
180-byte rows retain20.48 seconds at80ms with servo/governor and each leg's
response. Persistent V5 Flash records use a separate, unchanged format.
"""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import subprocess

from read_foc_failure import GUARDS, IDENTITY_BLOCKS, local_image_info, target_identity
from read_lqr_startup_record import GDB, OCD, check_firmware, finite_json, read_blocks

CAPACITY = 256
PREFIX = struct.Struct('<II6f4f2f2hf')
EXTENSION = struct.Struct('<HBBII10fI2f')
SERVO_EXTENSION = struct.Struct('<4fI8f')
# Validate offsets as well as size: interpreting another layout can look plausible.
PREFIX_FIELDS = {'tick_ms': 0, 'flags': 4, 'state': 8, 'position_error_m': 32,
                 'wheel_requested_nm': 36, 'wheel_applied_nm': 40, 'leg_tp_nm': 44,
                 'motion_component_nm': 48, 'wheel_voltage_mv': 56, 'leg_length_m': 60}
EXTENSION_FIELDS = {'raw_ch2': 64, 'motion_status': 66, 'input_flags': 67,
                    'source_frame_count': 68, 'sbus_age_ms': 72, 'normalized_forward': 76,
                    'requested_forward_m_s': 80, 'slewed_forward_m_s': 84,
                    'reference': 88, 'parking_goal_m': 104, 'bias_offset_m': 108,
                    'bias_applied_phi_rad': 112, 'bias_gate_flags': 116,
                    'requested_yaw_rad_s': 120, 'slewed_yaw_rad_s': 124}
SERVO_EXTENSION_FIELDS = {'servo_integral_next_m_s': 128, 'servo_blend': 132,
    'servo_wheel_integral_nm': 136, 'servo_leg_integral_nm': 140,
    'servo_motion_flags': 144, 'leg_angle_rad': 148, 'leg_angle_speed_rad_s': 156,
    'leg_lengths_m': 164, 'measured_leg_tp_nm': 172, 'parking_blend': 176}
INPUT_FLAGS = ('valid', 'calibrated', 'remote_enabled', 'ch5_on', 'controller_on',
               'normal_balance', 'neutral_ready', 'command_ready')
MOTION_STATUS = ('off', 'remote_invalid', 'uncalibrated', 'not_ready', 'target_ready', 'bad_parameters')
MOTION_REFERENCE_PHASE = ('parked', 'tracking', 'braking')


def symbols(axf, gdb=GDB):
    def query(expressions):
        command = [gdb, '-nx', '-batch', str(axf)]
        for expression in expressions:
            command += ['-ex', expression]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        values = [int(v, 0) for v in re.findall(r'\$\d+ = (0x[0-9a-fA-F]+|\d+)', result.stdout)]
        if len(values) != len(expressions):
            raise RuntimeError('Missing AXF balance ring symbols/layout')
        return values
    expressions = ['p/x &g_balance_trace', 'p sizeof(WheelBalanceTraceRow)',
                   'p sizeof(g_balance_trace.rows)/sizeof(WheelBalanceTraceRow)',
                   'p (unsigned long)&((WheelBalanceTraceDebug *)0)->rows']
    expressions += [f'p/x &{expression}' for _, expression in GUARDS]
    values = query(expressions)
    if values[1] not in (64, 128, 180) or values[2:4] != [CAPACITY, 12]:
        raise RuntimeError('Unsupported AXF balance ring row/header/capacity layout')
    fields = dict(PREFIX_FIELDS)
    if values[1] >= 128:
        fields.update(EXTENSION_FIELDS)
    if values[1] == 180:
        fields.update(SERVO_EXTENSION_FIELDS)
    offsets = query([f'p (unsigned long)&((WheelBalanceTraceRow *)0)->{name}' for name in fields])
    if offsets != list(fields.values()):
        raise RuntimeError('AXF balance row field offsets differ from decoder')
    return dict(trace=values[0], row_bytes=values[1], capacity=CAPACITY,
                guards=dict(zip((name for name, _ in GUARDS), values[4:])))


def read_stopped_ring(addresses, ocd):
    blocks = [(name, at, 1) for name, at in addresses['guards'].items()]
    blocks += [('header', addresses['trace'], 3),
               ('rows', addresses['trace']+12, CAPACITY*addresses['row_bytes']//4),
               ('header_after', addresses['trace'], 3)]
    blocks += [(name+'_after', at, 1) for name, at in addresses['guards'].items()]
    blocks += [(name+'_after', at, count) for name, at, count in IDENTITY_BLOCKS]
    data = read_blocks(blocks, ocd)  # Existing reader chunks every transfer to <=8 words.
    guards = {}
    for name in addresses['guards']:
        before, = struct.unpack('<I', data[name])
        after, = struct.unpack('<I', data[name+'_after'])
        if before or after:
            raise RuntimeError(f'Stop/disarm guard {name} is not zero; use CH5 OFF and wait for storage')
        guards[name] = before
    if data['header'] != data['header_after']:
        raise RuntimeError('New run changed balance ring headers; retry after disarm')
    return data['header'], data['rows'], guards, target_identity(data, '_after')


def decode_ring(header, raw, row_bytes):
    if row_bytes not in (64, 128, 180) or len(header) != 12 or len(raw) != CAPACITY*row_bytes:
        raise RuntimeError('Unsupported or incomplete balance ring bytes')
    next_row, count, sequence = struct.unpack('<3I', header)
    if next_row >= CAPACITY or not 0 < count <= CAPACITY or (count < CAPACITY and next_row != count):
        raise RuntimeError('Invalid balance ring next/count; use the matching AXF and stopped capture')
    records = []
    for i in range(count):
        at = ((next_row-count+i) % CAPACITY)*row_bytes
        v = PREFIX.unpack_from(raw, at)
        record = dict(tick_ms=v[0], flags=v[1], phase=(v[1]>>8)&255,
            state=list(v[2:8]), position_error_m=v[8], T_request_nm=v[9], T_applied_nm=v[10],
            Tp_nm=v[11], dtheta_component_nm=v[12], dphi_component_nm=v[13],
            wheel_voltage_mv=list(v[14:16]), length_m=v[16],
            body_pitch_deg=-math.degrees(v[6]), body_rate_deg_s=-math.degrees(v[7]),
            position_locked=None, raw_ch2=None, motion_status=None, motion_status_name=None,
            input_flags=None, input_permissions=None, source_frame_count=None, sbus_age_ms=None,
            sbus_age_ms_raw=None, normalized_forward=None, requested_forward_m_s=None,
            slewed_forward_m_s=None, reference=None, parking_goal_m=None, bias_offset_m=None,
            bias_applied_phi_rad=None, bias_gate_flags=None,
            requested_yaw_rad_s=None, slewed_yaw_rad_s=None,
            servo_integral_next_m_s=None, servo_blend=None, servo_wheel_integral_nm=None,
            servo_leg_integral_nm=None, servo_motion_flags=None, servo_gate_mask=None,
            servo_enabled=None, servo_active=None, servo_learning=None,
            motion_reference_phase=None, motion_reference_phase_name=None,
            motion_reference_enabled=None, joint_saturated=None, support_saturated=None,
            leg_angle_rad=None, leg_angle_speed_rad_s=None, leg_lengths_m=None,
            measured_leg_tp_nm=None, parking_blend=None)
        if row_bytes >= 128:
            e = EXTENSION.unpack_from(raw, at+64)
            record.update(position_locked=bool(v[1]&16), raw_ch2=None if e[0] == 65535 else e[0],
                motion_status=e[1], motion_status_name=MOTION_STATUS[e[1]] if e[1] < len(MOTION_STATUS) else 'unknown',
                input_flags=e[2], input_permissions={name: bool(e[2] & (1 << bit))
                    for bit, name in enumerate(INPUT_FLAGS)}, source_frame_count=e[3],
                sbus_age_ms=None if e[4] == 0xffffffff else e[4], sbus_age_ms_raw=e[4],
                normalized_forward=e[5], requested_forward_m_s=e[6], slewed_forward_m_s=e[7],
                reference=list(e[8:12]), parking_goal_m=e[12], bias_offset_m=e[13],
                bias_applied_phi_rad=e[14], bias_gate_flags=e[15],
                requested_yaw_rad_s=e[16], slewed_yaw_rad_s=e[17])
        if row_bytes == 180:
            s = SERVO_EXTENSION.unpack_from(raw, at+128)
            flags = s[4]
            motion_phase = (flags >> 8) & 3
            record.update(servo_integral_next_m_s=s[0], servo_blend=s[1],
                servo_wheel_integral_nm=s[2], servo_leg_integral_nm=s[3],
                servo_motion_flags=flags, servo_gate_mask=flags&31,
                servo_enabled=bool(flags&32), servo_active=bool(flags&64),
                servo_learning=bool(flags&128), motion_reference_phase=motion_phase,
                motion_reference_phase_name=MOTION_REFERENCE_PHASE[motion_phase]
                    if motion_phase < len(MOTION_REFERENCE_PHASE) else 'unknown',
                motion_reference_enabled=bool(flags&1024), joint_saturated=bool(flags&2048),
                support_saturated=[bool(flags&4096), bool(flags&8192)],
                leg_angle_rad=list(s[5:7]), leg_angle_speed_rad_s=list(s[7:9]),
                leg_lengths_m=list(s[9:11]), measured_leg_tp_nm=s[11], parking_blend=s[12])
        records.append(record)
    period_ms = 80 if row_bytes == 180 else 20
    return finite_json(dict(row_bytes=row_bytes, count=count, sequence=sequence, next=next_row,
        nominal_period_ms=period_ms, nominal_capacity_s=CAPACITY*period_ms/1000,
        first_tick_ms=records[0]['tick_ms'], last_tick_ms=records[-1]['tick_ms'],
        observed_duration_ms=(records[-1]['tick_ms']-records[0]['tick_ms'])&0xffffffff,
        max_abs_speed_m_s=max(abs(r['state'][3]) for r in records),
        max_abs_pitch_deg=max(abs(r['body_pitch_deg']) for r in records), records=records))


def collect(axf, hex_path, ocd, gdb=GDB):
    image = local_image_info(axf, hex_path)
    identity_before = target_identity(read_blocks(IDENTITY_BLOCKS, ocd))
    check_firmware(hex_path, ocd)  # Every application HEX byte, before using RAM addresses.
    addresses = symbols(axf, gdb)
    header, raw, guards, identity_after = read_stopped_ring(addresses, ocd)
    if identity_before != identity_after:
        raise RuntimeError('Target identity changed during the export')
    report = decode_ring(header, raw, addresses['row_bytes'])
    report.update(source='Latest frozen balance ring; all stop guards zero and stable headers; no target writes',
        axf=str(axf.resolve()), image=image, full_application_hex_verified=True,
        target=identity_after, guards=guards,
        input_semantics='128/180-byte rows: raw/normalized input is the coherent SBUS snapshot consumed before gates; requested is accepted unslewed target, slewed is actual command; invalid input is null.',
        reference_order=['theta_rad', 'x_m', 'dx_m_s', 'phi_rad'],
        bias_semantics='bias_offset_m is the estimate; bias_applied_phi_rad is actual observer contribution before manual trim; bias_gate_flags copies firmware gates.',
        servo_semantics='180-byte rows: servo_integral_next_m_s is z_(k+1), after this tick updates the integral; servo_wheel_integral_nm and servo_leg_integral_nm are the actual terms computed using z_k. Gates, governor phase and complete parking-row blend are copied from the same control iteration. Legacy rows have null servo fields.',
        leg_semantics='180-byte rows: angles/speeds/lengths are left/right measured geometry; measured_leg_tp_nm is the fresh-Iq/Jacobian mean estimate consumed by the controller, not an independent load-cell measurement.',
        limitation='Latest256 rows, reset each accepted run and frozen on disarm.64/128-byte rows:20ms/5.12s capacity;180-byte rows:80ms/20.48s capacity. First-to-last span is one sample interval shorter when full. No sub-sample latency resolution; persistent V5 Flash format unchanged.')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axf', type=Path, required=True)
    parser.add_argument('--hex', dest='hex_path', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--openocd', default=OCD)
    parser.add_argument('--gdb', default=GDB)
    args = parser.parse_args()
    report = collect(args.axf, args.hex_path or args.axf.with_suffix('.hex'), args.openocd, args.gdb)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
