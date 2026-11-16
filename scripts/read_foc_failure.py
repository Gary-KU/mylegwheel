"""Export the first latched FOC failure after disarm, without target writes.

Verify matching AXF/HEX and every application HEX byte before interpreting RAM.
This RAM-only snapshot is cleared by reset or the next accepted active run;
it cannot recover the detailed cause of a historical Flash record after reset.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess

from read_lqr_startup_record import FLASH, GDB, OCD, check_firmware, read_blocks

FIELDS = (
    'valid', 'reason', 'node_id', 'tick_ms', 'active_mask',
    'transport_epoch', 'node_epoch', 'session_ready', 'inhibit',
    'pending', 'opcode', 'sequence', 'pending_age_ms', 'got_ack',
    'last_opcode', 'last_sequence', 'last_result', 'ack_age_ms',
    'mode', 'flags', 'diagnostic_flags', 'faults', 'board_faults',
    'status_age_ms', 'diagnostic_age_ms', 'fast_age_ms',
    'fast_valid', 'fast_speed_clipped', 'tx_count', 'tx_failed',
    'timeout_count', 'rejected_count', 'deadline_expired_count',
    'deadline_invalid_buffer_count', 'communication_blocked')
LAYOUT = struct.Struct('<35I')
GUARDS = (
    ('robot', 'g_robot_control.enable'), ('balance', 'g_balance.enable'),
    ('dual', 'g_dual_hold.enable'), ('wheel_test', 'g_wheel_test.enable'),
    ('wheel_zero', 'g_wheel_test.zero_request'),
    ('ch5', 'g_robot_control.remote_switch_on'), ('storage', 'g_lqr_record.storage_busy'))
REASONS = {
    0: ('none', 'No failure latched since initialization or the latest accepted active run.'),
    1: ('stop_request', 'Explicit FOC stop_request revoked the service.'),
    2: ('tx_deadline', 'Bounded hardware transmit deadline expired or canceled the batch.'),
    3: ('maintenance', 'Active control conflicted with a pending maintenance exchange.'),
    4: ('active_mask', 'A previously active node left the requested active group, including normal CH5/manual STOP.'),
    5: ('transport_epoch', 'Communication invalidation revoked the earlier node session.'),
    6: ('ack_timeout', 'Pending command exceeded its unchanged acknowledgment deadline.'),
    7: ('ack_rejected', 'Matching acknowledgment returned a rejection or driver fault.'),
    8: ('driver_health', 'Driver health, ownership or active-mode flags revoked the session.'),
    9: ('feedback', 'Required kinematic or health feedback was invalid or stale.'),
    10: ('session', 'STOP handshake/session readiness or inhibit gate denied active output.'),
    11: ('calibration_limit', 'Calibration, parameter readback or physical current limit was not usable.'),
    12: ('invalid_torque', 'Requested torque was absent or nonfinite.'),
    13: ('invalid_current', 'Torque conversion or current limit was invalid.'),
    14: ('tx_enqueue', 'Bounded transmit enqueue failed; this is not by itself a bus-off diagnosis.'),
    15: ('speed_clipped', 'Legal FAST speed reached an int16 endpoint; kinematics were unusable.')}
IDENTITY_BLOCKS = [('cpuid', 0xE000ED00, 1), ('device', 0x5C001000, 1),
                   ('uid', 0x1FF1E800, 3)]


def decode_failure(raw):
    if len(raw) != LAYOUT.size:
        raise RuntimeError('Expected exactly 140 bytes of MotorFocFailureDebug')
    values = dict(zip(FIELDS, LAYOUT.unpack(raw)))
    if values['valid'] not in (0, 1) or values['node_id'] > 8:
        raise RuntimeError('Invalid failure snapshot fields; check AXF and capture stability')
    name, explanation = REASONS.get(values['reason'],
        ('unknown', 'Unknown reason value; retain raw fields and use the matching firmware header.'))
    normalized = {name: None if name.endswith('_age_ms') and value == 0xFFFFFFFF else value
                  for name, value in values.items()}
    return dict(bytes=LAYOUT.size, fields=normalized, raw_fields=values,
                reason_name=name, reason_explanation=explanation,
                snapshot_sha256=hashlib.sha256(raw).hexdigest())


def local_image_info(axf, hex_path):
    """Bind the DWARF file to its HEX, including initialized RAM load bytes."""
    memory, base, eof = {}, 0, False
    for line in hex_path.read_text(encoding='ascii').splitlines():
        if not line.startswith(':'):
            raise RuntimeError('Invalid HEX record')
        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0]+5 or sum(record) & 255:
            raise RuntimeError('Invalid HEX size/checksum')
        count, offset, kind = record[0], int.from_bytes(record[1:3], 'big'), record[3]
        payload = record[4:4+count]
        if eof:
            raise RuntimeError('Unexpected data after HEX EOF')
        if kind == 0:
            for i, value in enumerate(payload):
                at = base+offset+i
                if at in memory and memory[at] != value:
                    raise RuntimeError('Conflicting HEX bytes')
                memory[at] = value
        elif kind in (2, 4) and count == 2:
            base = int.from_bytes(payload, 'big') << (4 if kind == 2 else 16)
        elif kind == 1 and count == 0:
            eof = True
        elif kind not in (3, 5):
            raise RuntimeError('Unsupported HEX record')
    if not eof or not memory or min(memory) != 0x08000000 or max(memory) >= FLASH:
        raise RuntimeError('Application HEX overlaps reserved recorder Flash or has no valid image')
    elf = axf.read_bytes()
    if len(elf) < 52 or elf[:6] != b'\x7fELF\x01\x01':
        raise RuntimeError('AXF must be a little-endian ELF32 ARM image')
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', elf)
    if header[2] != 40 or header[9] != 32:
        raise RuntimeError('Unsupported AXF machine/program-header layout')
    expected = {}
    for index in range(header[10]):
        kind, offset, _, address, count, _, _, _ = struct.unpack_from('<8I', elf, header[5]+index*32)
        if kind != 1 or count == 0:
            continue
        if not 0x08000000 <= address < address+count <= FLASH or offset+count > len(elf):
            raise RuntimeError('AXF load segment is outside application Flash')
        expected.update({address+i: value for i, value in enumerate(elf[offset:offset+count])})
    if expected != memory:
        raise RuntimeError('AXF and HEX load bytes differ; RAM symbols cannot be trusted')
    return dict(axf=str(axf.resolve()), axf_sha256=hashlib.sha256(elf).hexdigest(),
                hex=str(hex_path.resolve()), hex_sha256=hashlib.sha256(hex_path.read_bytes()).hexdigest(),
                application_bytes=len(memory), flash_first=hex(min(memory)), flash_last=hex(max(memory)))


def symbols(axf, gdb=GDB):
    expressions = ['p/x &g_motor_foc_failure', 'p sizeof(g_motor_foc_failure)']
    expressions += [f'p (unsigned long)&((MotorFocFailureDebug *)0)->{name}' for name in FIELDS]
    expressions += [f'p/x &{expression}' for _, expression in GUARDS]
    command = [gdb, '-nx', '-batch', str(axf)]
    for expression in expressions:
        command += ['-ex', expression]
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    values = [int(v, 0) for v in re.findall(r'\$\d+ = (0x[0-9a-fA-F]+|\d+)', result.stdout)]
    if (len(values) != len(expressions) or values[1] != LAYOUT.size or
            values[2:2+len(FIELDS)] != list(range(0, LAYOUT.size, 4))):
        raise RuntimeError('AXF failure fields/offsets do not match the 140-byte decoder')
    return dict(failure=values[0], guards=dict(zip((name for name, _ in GUARDS), values[2+len(FIELDS):])))


def target_identity(data, suffix=''):
    cpuid, = struct.unpack('<I', data['cpuid'+suffix])
    device, = struct.unpack('<I', data['device'+suffix])
    uid = list(struct.unpack('<3I', data['uid'+suffix]))
    if cpuid >> 24 != 0x41 or (cpuid >> 4) & 0xFFF != 0xC27 or device & 0xFFF != 0x483:
        raise RuntimeError('Connected target is not the expected Cortex-M7 STM32H723 device')
    return dict(cpuid=f'0x{cpuid:08x}', device_idcode=f'0x{device:08x}',
                device_id=f'0x{device & 0xfff:03x}', uid_words=[f'0x{word:08x}' for word in uid])


def read_stopped_snapshot(addresses, ocd):
    blocks = [(name, at, 1) for name, at in addresses['guards'].items()]
    blocks += [('failure', addresses['failure'], LAYOUT.size//4)]
    blocks += [('failure_after', addresses['failure'], LAYOUT.size//4)]
    blocks += [(name+'_after', at, 1) for name, at in addresses['guards'].items()]
    blocks += [(name+'_after', at, count) for name, at, count in IDENTITY_BLOCKS]
    data = read_blocks(blocks, ocd)  # Existing reader chunks every transfer to <=8 words.
    guards = {}
    for name in addresses['guards']:
        before, = struct.unpack('<I', data[name])
        after, = struct.unpack('<I', data[name+'_after'])
        if before or after:
            raise RuntimeError(f'Stop/disarm guard {name} is not zero; use CH5 OFF and wait for storage to finish')
        guards[name] = before
    if data['failure'] != data['failure_after']:
        raise RuntimeError('Failure snapshot changed while reading; retry after disarm')
    return data['failure'], guards, target_identity(data, '_after')


def collect(axf, hex_path, ocd, gdb=GDB):
    image = local_image_info(axf, hex_path)
    identity_before = target_identity(read_blocks(IDENTITY_BLOCKS, ocd))
    check_firmware(hex_path, ocd)  # Compare EVERY application HEX byte before using RAM addresses.
    addresses = symbols(axf, gdb)
    raw, guards, identity_after = read_stopped_snapshot(addresses, ocd)
    if identity_before != identity_after:
        raise RuntimeError('Target identity changed during the export')
    return dict(source='Stopped RAM failure snapshot; all guards zero and two byte-identical reads',
                image=image, full_application_hex_verified=True, target=identity_after,
                guards=guards, failure=decode_failure(raw),
                interpretation='A valid latched service/stop event is not necessarily a robot fault; normal CH5/manual STOP can record active_mask.',
                limitation='RAM only; reset/next accepted active run clears it. Does not identify a historical Flash failure after reset.')


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
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
