"""Read running control RAM through OpenOCD TCL; never halt or write target RAM.

Start OpenOCD with the matching target first. The AXF must match flashed code.
Snapshots are sequential debugger reads, not atomic controller snapshots.
"""
import argparse
import json
from pathlib import Path
import re
import socket
import struct
import subprocess
import time
import math

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--axf', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--wait-on', type=float, default=60)
parser.add_argument('--seconds', type=float, default=6)
args = parser.parse_args()
assert 0 <= args.wait_on <= 60 and 0 < args.seconds <= 10
gdb = Path('D:/app/clion/clion+stm32/gcc-arm-none-eabi-10.3-2021.10-win32/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gdb.exe')
names = ['g_robot_control', 'g_balance', 'g_wheel_leg', 'g_wheel_output']
command = [str(gdb), '-nx', '-batch', str(args.axf)]
for name in names:
    command += ['-ex', f'p/x &{name}', '-ex', f'p sizeof({name})']
result = subprocess.run(command, capture_output=True, text=True, check=True)
values = re.findall(r'\$\d+ = (0x[0-9a-f]+|\d+)', result.stdout)
assert len(values) == 2*len(names), result.stdout
symbols = {name: (int(values[2*i], 0), int(values[2*i+1], 0)//4) for i, name in enumerate(names)}
connection = socket.create_connection(('127.0.0.1', 6666), 3)
connection.settimeout(5)
def read(name):
    at, count = symbols[name]
    connection.sendall(f'read_memory {at:#x} 32 {count}\x1a'.encode())
    data = b''
    while not data.endswith(b'\x1a'):
        chunk = connection.recv(65536)
        if not chunk:
            raise RuntimeError('OpenOCD disconnected')
        data += chunk
    words = [int(word, 0) for word in data[:-1].decode().split()]
    assert len(words) == count
    return words
def floats(words):
    return [struct.unpack('<f', struct.pack('<I', word))[0] for word in words]

records = []
report = {'axf': str(args.axf.resolve()), 'symbols': symbols,
          'limitation': 'Sequential live RAM reads; no target halt or actuator commands; not atomic snapshots.',
          'records': records}
wait_start = time.monotonic()
print('Waiting for human CH5 activation; no software rearm or output commands.', flush=True)
try:
    while True:
        control = read('g_robot_control')
        if control[0] == 1 or time.monotonic()-wait_start >= args.wait_on:
            break
        time.sleep(.05)
    if control[0] != 1:
        report['result'] = 'No active control observed in waiting window'
    else:
        print('Active control observed; recording bounded live trace.', flush=True)
        start = time.monotonic()
        while time.monotonic()-start < args.seconds:
            cycle = time.monotonic()
            control = read('g_robot_control')
            balance_raw = read('g_balance')
            observation = read('g_wheel_leg')
            wheel_raw = read('g_wheel_output')
            b, o, w = floats(balance_raw), floats(observation), floats(wheel_raw)
            voltage = struct.unpack('<hh', struct.pack('<I', wheel_raw[16]))
            record = {'host_elapsed_s': cycle-start, 'read_duration_s': time.monotonic()-cycle,
                      'tick_ms': observation[1], 'period_ms': observation[2],
                      'enable': control[0], 'status': control[1], 'phase': control[16],
                      'fault': control[4], 'CH5': control[26], 'link': control[27],
                      'wheel_scale': b[2], 'leg_scale': b[3], 'state': b[7:13], 'reference': b[13:19],
                      'length_m': b[19], 'T_request_nm': b[20], 'Tp_nm': b[21],
                      'wheel_components_nm': b[22:28], 'balance_saturated': balance_raw[28],
                      'parking_locked': balance_raw[35], 'body_pitch_deg': o[5]*180/math.pi,
                      'wheel_request_mv': w[10:12], 'wheel_speed_rad_s': w[12:14],
                      'wheel_voltage_mv': list(voltage), 'wheel_saturated': wheel_raw[17],
                      'left_length_m': o[9], 'right_length_m': o[15]}
            if len(wheel_raw) >= 21:
                record.update(T_before_nm=w[18], T_applied_nm=w[19], T_saturated=wheel_raw[20])
            records.append(record)
            if len(records) > 1 and control[0] != 1:
                break
            remaining = .02-(time.monotonic()-cycle)
            if remaining > 0:
                time.sleep(remaining)
        report['result'] = 'Captured'
finally:
    connection.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
print(f'Saved {len(records)} records: {args.output}', flush=True)
if records:
    active = [row for row in records if row['enable'] == 1 and row['phase'] == 3]
    if active:
        print(json.dumps({'phase3_records': len(active),
                          'wheel_saturated_fraction': sum(bool(r['wheel_saturated']) for r in active)/len(active),
                          'Tp_limit_fraction': sum(abs(r['Tp_nm'])>=.0199 for r in active)/len(active),
                          'max_abs_T_requested_nm': max(abs(r['T_request_nm']) for r in active),
                          'max_abs_position_error_m': max(abs(r['state'][2]-r['reference'][2]) for r in active),
                          'max_abs_speed_m_s': max(abs(r['state'][3]) for r in active)}, indent=2), flush=True)
