"""Export the latest frozen FOC DQ ring without halt, reset or target writes.

Trace pages are asynchronous; age fields qualify their values. This is the
latest run, which may differ from the first trial preserved in Flash.
"""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import subprocess
from read_lqr_startup_record import GDB, OCD, read_blocks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--openocd', default=OCD)
    args = parser.parse_args()
    command = [GDB, '-nx', '-batch', str(args.axf)]
    for expression in ('p/x &g_robot_control.enable', 'p/x &g_motor_foc_dq_trace',
                       'p sizeof(MotorFocDqTraceRow)', 'p/x &g_motor_foc_command_timing'):
        command += ['-ex', expression]
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    values = re.findall(r'\$\d+ = (0x[0-9a-f]+|\d+)', result.stdout)
    assert len(values) == 4, result.stdout
    control, trace, size, timing = [int(value, 0) for value in values]
    row = struct.Struct('<II36f12H')
    assert size == row.size == 176
    blocks = [('off', control, 1), ('header', trace, 3),
              ('rows', trace+12, 128*size//4), ('timing', timing, 32),
              ('off_after', control, 1), ('header_after', trace, 3)]
    data = read_blocks(blocks, args.openocd)
    assert data['off'] == data['off_after'] == b'\0'*4, 'Stop CH5 before reading'
    assert data['header'] == data['header_after'], 'Trace changed during export'
    next_row, count, sequence = struct.unpack('<3I', data['header'])
    assert next_row < 128 and 0 < count <= 128
    names = ('requested_iq_a', 'actual_iq_a', 'reference_iq_a', 'id_a', 'vd_v', 'vq_v')
    records = []
    for index in range(count):
        values = row.unpack_from(data['rows'], ((next_row-count+index) % 128)*size)
        record = dict(tick_ms=values[0], active_mask=values[1])
        for field, name in enumerate(names):
            record[name] = [value if math.isfinite(value) else None
                            for value in values[2+6*field:8+6*field]]
        record['page_age_ms'] = [list(values[38+2*i:40+2*i]) for i in range(6)]
        records.append(record)
    timing_rows = [dict(zip(('count', 'first_ms', 'last_ms', 'max_gap_ms'),
                           struct.unpack_from('<4I', data['timing'], 16*i))) for i in range(8)]
    report = dict(source=__doc__, axf=str(args.axf.resolve()), node_ids=[1,2,3,5,6,7],
                  count=count, sequence=sequence, next=next_row, command_timing=timing_rows,
                  first_tick_ms=records[0]['tick_ms'], last_tick_ms=records[-1]['tick_ms'],
                  records=records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({key:value for key,value in report.items() if key != 'records'}, indent=2))


if __name__ == '__main__':
    main()
