"""Read the frozen 50Hz ring after CH5 OFF, without halt or target writes."""
import argparse
import json
from pathlib import Path
import re
import socket
import struct
import subprocess
import math
import csv

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--axf', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--csv', type=Path, help='Also export readable startup diagnostics to CSV')
args = parser.parse_args()
gdb = 'D:/app/clion/clion+stm32/gcc-arm-none-eabi-10.3-2021.10-win32/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gdb.exe'
command = [gdb, '-nx', '-batch', str(args.axf)]
for expression in ('p/x &g_robot_control', 'p/x &g_balance_trace', 'p sizeof(WheelBalanceTraceRow)',
                   'p sizeof(g_balance_trace)'):
    command += ['-ex', expression]
result = subprocess.run(command, capture_output=True, text=True, check=True)
values = re.findall(r'\$\d+ = (0x[0-9a-f]+|\d+)', result.stdout)
assert len(values) == 4, result.stdout
control_at, trace_at, row_size, total_size = [int(v, 0) for v in values]
assert row_size == 64 and total_size == 12+256*64
connection = socket.create_connection(('127.0.0.1', 6666), 3)
connection.settimeout(5)
def read(at, count):
    words = []
    while count:
        take = min(count, 8)  # Keep each CMSIS-DAP transfer small.
        connection.sendall(f'read_memory {at:#x} 32 {take}\x1a'.encode())
        response = b''
        while not response.endswith(b'\x1a'):
            chunk = connection.recv(65536)
            if not chunk:
                raise RuntimeError('Debug connection closed')
            response += chunk
        block = [int(v, 0) for v in response[:-1].decode().split()]
        assert len(block) == take
        words += block
        at += 4*take
        count -= take
    return words

try:
    assert read(control_at, 1)[0] == 0, 'CH5 must be OFF before reading frozen trace'
    header = read(trace_at, 3)
    next_row, count, sequence = header
    assert next_row < 256 and count <= 256
    first = (next_row-count) % 256
    records = []
    for i in range(count):
        words = read(trace_at+12+((first+i) % 256)*64, 16)
        values = struct.unpack('<II6f4f2f2hf', struct.pack('<16I', *words))
        records.append({'tick_ms': values[0], 'flags': values[1], 'phase': (values[1]>>8)&255,
                        'state': list(values[2:8]), 'position_error_m': values[8],
                        'T_request_nm': values[9], 'T_applied_nm': values[10], 'Tp_nm': values[11],
                        'dtheta_component_nm': values[12], 'dphi_component_nm': values[13],
                        'wheel_voltage_mv': list(values[14:16]), 'length_m': values[16],
                        'body_pitch_deg': -values[6]*180/math.pi,
                        'body_rate_deg_s': -values[7]*180/math.pi})
    assert read(control_at, 1)[0] == 0 and read(trace_at, 3) == header, 'Trace changed during read; retry after CH5 OFF'
finally:
    connection.close()
report = {'axf': str(args.axf.resolve()), 'next': next_row, 'count': count, 'sequence': sequence,
          'sample_period_ms': 20, 'source': 'Coherent producer records, frozen after disarm; RAM only',
          'records': records}
active = [r for r in records if r['flags'] & 1]
if active:
    report['startup_summary'] = {
        'active_records': len(active),
        'first_phase': active[0]['phase'],
        'first_pitch_deg': active[0]['body_pitch_deg'],
        'first_left_torque_command_nm': active[0]['T_applied_nm'],
        'first_left_voltage_mv': active[0]['wheel_voltage_mv'][0],
        'max_abs_pitch_deg': max(abs(r['body_pitch_deg']) for r in active),
        'max_abs_left_torque_command_nm': max(abs(r['T_applied_nm']) for r in active),
        'max_abs_left_voltage_mv': max(abs(r['wheel_voltage_mv'][0]) for r in active),
        'max_abs_speed_m_s': max(abs(r['state'][3]) for r in active),
        'output_saturated_records': sum(bool(r['flags'] & 2) for r in active)}
normal = [r for r in records if r['phase'] == 3]
if normal:
    report['summary'] = {
        'normal_records': len(normal),
        'wheel_nm_saturated_fraction': sum(bool(r['flags'] & 4) for r in normal)/len(normal),
        'wheel_output_saturated_fraction': sum(bool(r['flags'] & 2) for r in normal)/len(normal),
        'Tp_limit_fraction': sum(bool(r['flags'] & 8) for r in normal)/len(normal),
        'max_abs_position_error_m': max(abs(r['position_error_m']) for r in normal),
        'max_abs_speed_m_s': max(abs(r['state'][3]) for r in normal),
        'max_abs_T_requested_nm': max(abs(r['T_request_nm']) for r in normal),
        'max_abs_T_applied_nm': max(abs(r['T_applied_nm']) for r in normal),
        'max_abs_pitch_deg': max(abs(r['body_pitch_deg']) for r in normal),
        'max_abs_body_rate_deg_s': max(abs(r['body_rate_deg_s']) for r in normal),
        'max_abs_dtheta_component_nm': max(abs(r['dtheta_component_nm']) for r in normal),
        'max_abs_dphi_component_nm': max(abs(r['dphi_component_nm']) for r in normal)}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
if args.csv:
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open('w', newline='', encoding='utf-8-sig') as output:
        writer = csv.writer(output)
        writer.writerow(('elapsed_ms','phase','left_torque_command_mNm','left_voltage_mv',
                         'right_voltage_mv','body_pitch_deg','body_rate_deg_s','speed_mm_s',
                         'position_error_mm','leg_length_mm','leg_Tp_mNm','wheel_output_saturated'))
        for r in records:
            writer.writerow(((r['tick_ms']-records[0]['tick_ms']) & 0xffffffff,
                             r['phase'],1000*r['T_applied_nm'],*r['wheel_voltage_mv'],
                             r['body_pitch_deg'],r['body_rate_deg_s'],1000*r['state'][3],
                             1000*r['position_error_m'],1000*r['length_m'],1000*r['Tp_nm'],
                             int(bool(r['flags'] & 2))))
    print(f'CSV: {args.csv}')
print(json.dumps({k: v for k, v in report.items() if k != 'records'}, indent=2))
