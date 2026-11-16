"""Streaming robot/legacy ANO decoder. No serial or GUI dependencies."""
import binascii
import math
import struct

FIELDS = (
    'pitch_rad', 'pitch_ref_rad', 'pitch_rate_rad_s',
    'position_m', 'position_ref_m', 'speed_m_s',
    'leg_angle_left_rad', 'leg_angle_right_rad',
    'leg_length_left_m', 'leg_length_right_m',
    'sync_left_nm', 'sync_right_nm', 'wheel_lqr_nm', 'leg_lqr_nm',
    'support_left_n', 'support_right_n', 'wheel_left_v', 'wheel_right_v',
    'joint_lf_v', 'joint_lr_v', 'joint_rf_v', 'joint_rr_v',
    'leg_angle_ref_left_rad', 'leg_angle_ref_right_rad',
)
META = ('sequence', 'tick_ms', 'validity', 'robot_status', 'startup_phase',
        'fault_reason', 'fault_count', 'output_flags')
LEGACY = ('imu_roll_deg', 'imu_pitch_deg', 'imu_yaw_deg', 'imu_temperature_c')
FRAME_BYTES = 136


def encode(meta, values):
    payload = struct.pack('<8I24f', *meta, *values)
    body = b'WL\x01\x01' + struct.pack('<H', len(payload)) + payload
    return body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.frames = self.crc_errors = self.discarded_bytes = 0
        self.legacy_sequence = 0

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] not in (b'WL', b'\xaa\xff'):
                positions = [p for p in (self.buffer.find(b'WL'), self.buffer.find(b'\xaa\xff')) if p >= 0]
                skip = min(positions) if positions else len(self.buffer)-1
                self.discarded_bytes += skip
                del self.buffer[:skip]
                continue
            robot = self.buffer[:2] == b'WL'
            header_size = 6 if robot else 4
            if len(self.buffer) < header_size:
                break
            if robot:
                size = struct.unpack_from('<H', self.buffer, 4)[0]
                valid_header = self.buffer[2:4] == b'\x01\x01' and size == 128
            else:
                size = self.buffer[3]
                valid_header = size <= 50
            if not valid_header:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            total = header_size + size + 2
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            if robot:
                good = binascii.crc_hqx(frame[:-2], 0xffff) == struct.unpack_from('<H', frame, total-2)[0]
            else:
                sc = ac = 0
                for value in frame[:-2]:
                    sc = (sc+value) & 255
                    ac = (ac+sc) & 255
                good = frame[-2:] == bytes((sc, ac))
            if not good:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:total]
            if robot:
                unpacked = struct.unpack('<8I24f', frame[6:-2])
                record = dict(zip(META, unpacked[:8]))
                record.update(zip(FIELDS, unpacked[8:]))
                record['protocol'] = 'robot24'
            elif frame[2] == 0xf1 and size == 16:
                record = dict(zip(LEGACY, (value/100 for value in struct.unpack('<4i', frame[4:-2]))))
                record.update(protocol='ano4', sequence=self.legacy_sequence)
                self.legacy_sequence = (self.legacy_sequence+1) & 0xffffffff
            elif frame[2] == 0xf2 and size == 4:
                record = {'protocol': 'ano_status', 'imu_status': struct.unpack('<I', frame[4:-2])[0]}
            else:
                continue
            self.frames += 1
            # Validity is explicit on the wire. JSON cannot represent NaN/Inf.
            record = {key: None if isinstance(value, float) and not math.isfinite(value) else value
                      for key, value in record.items()}
            records.append(record)
        return records
