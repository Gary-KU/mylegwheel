"""Export LQR launch-to-fall RAM/Flash records; never halt, reset or write the target."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import zlib

HEADER = struct.Struct('<12I16f4I')
ROW = struct.Struct('<6I27f6h4f')
ROW_V3 = struct.Struct('<6I27f6h12f4H21fI')
ROW_V4 = struct.Struct('<6I27f6h12f4H21fI8f')
ROW_V5 = struct.Struct(ROW_V4.format+'24f18H6I4f5I')
CONFIG_V3 = struct.Struct('<52f8i4I')
CONFIG_V5 = struct.Struct('<52f8i4I6f6i6I2I12f')
PARAMETERS_V3 = ('leg_target_m','leg_position_kp','leg_speed_kp','leg_speed_ki',
    'leg_max_speed_m_s','leg_filter_tau_s','leg_max_force_n','support_rate_n_s',
    'overlength_kp_n_per_m','overlength_margin_m','overlength_startup_s','overlength_transition_m',
    'length_error_deadband_m','length_speed_deadband_m_s','angle_kp_nm_per_rad',
    'angle_kd_nm_s_per_rad','angle_limit_nm','angle_target_rad','angle_reference_rate_rad_s',
    'leg_hold_ratio','sync_ratio','sync_position_kp','sync_speed_kp','sync_max_speed_rad_s',
    'sync_filter_tau_s','sync_limit_nm','sync_rate_nm_s','joint_back_emf_ratio',
    'shared_leg_Tp_limit_nm','joint_torque_limit_nm','load_ki_n_per_m_s','load_limit_n',
    'load_rate_n_s','angle_reference_mode','support_only','load_enable','body_pitch_stop_rad',
    'body_pitch_direction','wheel_radius_m','startup_ramp_s')
MAGIC, COMMIT, FLASH = 0x3152514c, 0xc0dec0de, 0x080c0000
REASONS = {1: 'controller stop (CH5/manual or isolated test timeout)', 2: 'large pitch trigger', 3: 'control fault', 4: 'buffer full'}
DIAGNOSTIC_BUILDS = {2026100712: 'left', 2026100713: 'right', 2026100714: 'left', 2026100715: 'right', 2026100716: 'left', 2026100717: 'right', 2026100718: 'left', 2026100719: 'right'}
VOLTAGE_PROBE_BUILDS = (2026100714, 2026100715, 2026100718, 2026100719)
FAULTS = {0: 'none', 1: 'leg length range', 2: 'feedback', 3: 'timing', 4: 'geometry',
          5: 'parameters', 6: 'CAN TX', 7: 'CAN TX disabled', 8: 'state', 9: 'unsafe leg posture/split',
          10: 'excessive body pitch', 11: 'uncommanded high yaw'}
OCD = 'D:/app/clion/clion+stm32/openocd-20230621/OpenOCD-20230621-0.12.0/bin/openocd.exe'
GDB = 'D:/app/clion/clion+stm32/gcc-arm-none-eabi-10.3-2021.10-win32/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gdb.exe'


def read_blocks(blocks, ocd):
    """Each block is (name, address, word count); transfers remain read-only."""
    commands = ['init', 'poll off']
    for name, address, count in blocks:
        for offset in range(0, count, 8):
            take = min(8, count-offset)
            commands += [f'echo READ_{name}_{offset}',
                         f'echo [read_memory {address+offset*4:#x} 32 {take}]']
    commands.append('shutdown')
    with tempfile.TemporaryDirectory(prefix='lqr_read_') as directory:
        config = Path(directory)/'read.cfg'
        config.write_text('\n'.join(commands)+'\n', encoding='ascii')
        result = subprocess.run([ocd, '-f', 'interface/cmsis-dap.cfg',
                                 '-c', 'if {[llength [info commands cmsis_dap_backend]]} {cmsis_dap_backend hid} else {cmsis-dap backend hid}',
                                 '-c', 'transport select swd',
                                 '-f', 'target/stm32h7x.cfg', '-c', 'adapter speed 500', '-f', str(config)],
                                capture_output=True, text=True, timeout=60)
    output = result.stdout+result.stderr
    if result.returncode:
        raise RuntimeError(output[-3000:])
    answer = {}
    for name, _, count in blocks:
        values = []
        for offset in range(0, count, 8):
            match = re.search(rf'READ_{re.escape(name)}_{offset}\s*\n([^\n]+)', output)
            if not match:
                raise RuntimeError(f'Missing read {name}; {output[-2000:]}')
            words = [int(word, 16) for word in match[1].split()]
            if len(words) != min(8, count-offset):
                raise RuntimeError(f'Incomplete read {name}')
            values.extend(words)
        answer[name] = struct.pack('<'+'I'*len(values), *values)
    return answer


def header_decode(raw, require_commit=True):
    values = HEADER.unpack(raw)
    version = values[1]
    row_bytes, capacity, configuration_bytes = (ROW_V5.size,500,CONFIG_V5.size) if version==5 else (ROW_V4.size,800,CONFIG_V3.size) if version==4 else (ROW_V3.size,896,CONFIG_V3.size) if version==3 else (ROW.size,1536,0)
    if (values[0] != MAGIC or version not in (1,2,3,4,5) or values[3] != row_bytes or values[4] != capacity or
            not 0 < values[5] <= capacity or (require_commit and values[31] != COMMIT)):
        raise RuntimeError('No valid LQR startup record header on this target')
    config_names = ('wheel_scale','leg_scale','wheel_limit_mv','joint_limit_mv','support_target_n',
                    'pitch_trim_deg','back_emf_ratio','fall_pitch_deg','supply_model_v',
                    'joint_driver_supply_v','left_wheel_nm_per_v','right_wheel_nm_per_v',
                    'LF_nm_per_v','LR_nm_per_v','RF_nm_per_v','RR_nm_per_v')
    configuration = dict(zip(config_names, values[12:28]))
    configuration['wheel_rate_scale'] = struct.unpack_from('<f',raw,112)[0] if version>=2 else 1.0
    return {'version': values[1], 'build_id': values[2], 'count': values[5], 'reason': values[6],
            'reason_text': REASONS.get(values[6], 'unknown'), 'trigger_index': values[7],
            'start_tick_ms': values[8], 'end_tick_ms': values[9], 'period_ms': values[10],
            'generation': values[11], 'configuration': configuration,
            'row_bytes': row_bytes, 'capacity': capacity, 'configuration_bytes': configuration_bytes,
            'payload_offset': HEADER.size+configuration_bytes,
            'recorded_startup_ok': bool(values[29]) if version>=2 else None,
            'crc32': values[30], 'commit': values[31],
            'control_mode': ('isolated leg voltage probe' if values[2] in VOLTAGE_PROBE_BUILDS
                else 'isolated leg PD' if values[2] in DIAGNOSTIC_BUILDS else 'full control'),
            'isolated_leg_side': DIAGNOSTIC_BUILDS.get(values[2]),
            'position_reference_semantics': ('unused by isolated leg diagnostic control'
                if values[2] in DIAGNOSTIC_BUILDS else 'true fixed parking goal, retained after stop; active moving reference before parking lock'
                if values[2]>=2026100710 else 'active LQR reference; may differ from true parking target if input was bounded')}


def configuration_decode(raw):
    v = (CONFIG_V5 if len(raw)==CONFIG_V5.size else CONFIG_V3).unpack(raw)
    answer = dict(zip(PARAMETERS_V3, v[:40]))
    for name, start in (('joint_encoder_reference_rad',40),('joint_model_reference_rad',44),
                        ('joint_encoder_ratio',48),('joint_encoder_direction',52),
                        ('joint_output_direction',56),('joint_motor_id',60)):
        answer[name] = list(v[start:start+4])
    if len(raw)==CONFIG_V5.size:
        answer.update(foc_kt_nm_per_a=list(v[64:70]),foc_direction=list(v[70:76]),
            foc_node_id=list(v[76:82]),current_backend_mask=v[82],motion_enable_mask=v[83],
            foc_current_limit_a=list(v[84:90]),minimum_iq_slew_a_per_s=list(v[90:96]))
    return answer


def record_crc(raw_header, raw_rows, raw_configuration=b''):
    return zlib.crc32(raw_rows,zlib.crc32(raw_configuration,zlib.crc32(raw_header[:120])))&0xffffffff


def rows_decode(raw, count, start_tick, version=2, configuration=None):
    records = []
    for index in range(count):
        layout = ROW_V5 if version==5 else ROW_V4 if version==4 else ROW_V3 if version==3 else ROW
        v = layout.unpack_from(raw, index*layout.size)
        record = {'index': index, 'tick_ms': v[0], 'elapsed_ms': (v[0]-start_tick)&0xffffffff,
                  'flags': v[1], 'phase': (v[1]>>8)&255, 'active': bool(v[1]&1),
                  'wheel_voltage_saturated': bool(v[1]&2), 'wheel_Nm_saturated': bool(v[1]&4),
                  'leg_Tp_saturated': bool(v[1]&8), 'joint_output_saturated': bool(v[1]&32),
                  'sync_priority_limited': bool(v[1]&64),
                  'parking_damping_active': bool(v[1]&128),
                  'position_locked': bool(v[1]&16), 'validity': v[2], 'control_status': v[3],
                  'fault_reason': v[4], 'fault_count': v[5],
                  'state': list(v[6:12]), 'reference': list(v[12:18]),
                  'wheel_components_nm': list(v[18:24]), 'wheel_requested_nm': v[24],
                  'left_wheel_torque_command_nm': v[25], 'leg_Tp_nm': v[26],
                  'leg_length_m': list(v[27:29]), 'leg_angle_rad': list(v[29:31]),
                  'support_command_n': list(v[31:33]), 'wheel_voltage_mv': list(v[33:35]),
                  'joint_voltage_mv': list(v[35:39]), 'body_pitch_deg': math.degrees(v[39]),
                  'body_rate_deg_s': math.degrees(v[40]), 'raw_wheel_speed_rad_s': list(v[41:43])}
        record['position_error_m'] = v[8]-v[14]
        record['fault_reason_text'] = FAULTS.get(v[4], 'unknown')
        record['new_fault_event'] = bool(index and v[5]!=records[-1]['fault_count'])
        if version>=3:
            record.update(joint_angle_rad=list(v[43:47]),joint_shaft_speed_rad_s=list(v[47:51]),
                joint_feedback_age_ms=list(v[51:55]),length_speed_m_s=list(v[55:57]),
                length_position_speed_m_s=list(v[57:59]),angle_speed_rad_s=list(v[59:61]),
                angle_pd_nm=list(v[61:63]),sync_nm=list(v[63:65]),lqr_pitch_nm=list(v[65:67]),
                total_pitch_nm=list(v[67:69]),joint_torque_command_nm=list(v[69:73]),
                angle_reference_rad=list(v[73:75]),support_feedforward_n=v[75],control_period_ms=v[76],
                joint_feedback_valid=[bool(v[2]&(1<<(16+i))) for i in range(4)],
                length_position_speed_valid=[bool(v[2]&(1<<(20+i))) for i in range(2)])
            if configuration:
                record['joint_model_speed_rad_s'] = [
                    record['joint_shaft_speed_rad_s'][i]*configuration['joint_encoder_direction'][i]/ratio
                    if math.isfinite(ratio) and ratio>0 else math.nan
                    for i,ratio in enumerate(configuration['joint_encoder_ratio'])]
        if version>=4:
            record.update(forward_target_m_s=v[77],yaw_target_rad_s=v[78],yaw_rate_rad_s=v[79],
                differential_applied_nm=v[80],wheel_requested_per_side_nm=list(v[81:83]),
                wheel_applied_per_side_nm=list(v[83:85]),
                mean_wheel_torque_command_nm=v[25],left_wheel_torque_command_nm=v[83],
                right_wheel_torque_command_nm=v[84],motion_status=(v[1]>>24)&15,
                motion_ready=bool(v[1]&(1<<17)),translating=bool(v[1]&(1<<18)),
                yaw_active=bool(v[1]&(1<<19)),neutral_ready=bool(v[1]&(1<<20)),
                yaw_enabled=bool(v[1]&(1<<21)),yaw_feedback_direction=-1 if v[1]&(1<<22) else 1)
        if version>=5:
            record.update(foc_requested_iq_a=list(v[85:91]),foc_applied_iq_a=list(v[91:97]),
                foc_actual_iq_a=list(v[97:103]),foc_current_limit_a=list(v[103:109]),
                foc_feedback_age_ms=list(v[109:115]),foc_flags=list(v[115:121]),foc_mode=list(v[121:127]),
                foc_faults=list(v[127:133]),support_requested_n=list(v[133:135]),support_limit_n=list(v[135:137]),
                support_saturated=[bool(x) for x in v[137:139]],feedback_detail=v[139],feedback_motor_id=v[140],feedback_ms=v[141])
            record['foc_driver_enabled']=[bool(x&1) for x in record['foc_flags']]
            record['combined_lean_deg']=record['parking_lean_deg']=math.degrees(record['reference'][0])
            record['parking_lean_learning']=bool(record['flags']&65536)
            record['motion_lean_learning']=bool(record['flags']&(1<<28))
            record['motion_stall_hold']=bool(record['flags']&(1<<29))
            record['motion_damping_active']=bool(record['flags']&(1<<23))
            record['motion_speed_error_m_s']=record['state'][3]-record['forward_target_m_s']
            record['motion_damping_command_nm']=(record['wheel_requested_nm']-sum(record['wheel_components_nm'])
                if record['active'] and record['phase']==3 and record['motion_damping_active'] else 0.0)
            record['yaw_integral_active']=bool(record['flags']&(1<<30))
            record['yaw_integral_blocked']=bool(record['flags']&(1<<31))
        records.append(record)
    return records


def symbols(axf):
    expressions = ('p/x &g_lqr_record', 'p sizeof(g_lqr_record)', 'p/x &g_lqr_record_data',
                   'p sizeof(LqrRecordHeader)', 'p sizeof(LqrRecordRow)', 'p/x &g_robot_control.enable')
    command = [GDB, '-nx', '-batch', str(axf)]
    for expression in expressions:
        command += ['-ex', expression]
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    values = [int(v, 0) for v in re.findall(r'\$\d+ = (0x[0-9a-f]+|\d+)', result.stdout)]
    if len(values) != 6 or values[1] != 64 or values[3] != HEADER.size or values[4] not in (ROW.size,ROW_V3.size,ROW_V4.size,ROW_V5.size):
        raise RuntimeError('AXF does not contain the matching LQR recorder layout')
    return values[0], values[2], values[5]


def check_firmware(hex_path, ocd):
    """Compare all HEX bytes, including initialized configuration, before using RAM addresses."""
    memory, base = {}, 0
    for line in hex_path.read_text().splitlines():
        r = bytes.fromhex(line[1:])
        if sum(r)&255:
            raise RuntimeError('HEX checksum mismatch')
        if r[3] == 4:
            base = int.from_bytes(r[4:6], 'big') << 16
        elif r[3] == 0:
            at = base+int.from_bytes(r[1:3], 'big')
            memory.update({at+i: value for i, value in enumerate(r[4:4+r[0]])})
    if not memory or min(memory) != 0x08000000 or max(memory) >= FLASH:
        raise RuntimeError('Firmware must remain outside reserved recorder Flash')
    first, last = min(memory)&~3, (max(memory)+4)&~3
    raw = read_blocks([('image', first, (last-first)//4)], ocd)['image']
    for at, expected in memory.items():
        if raw[at-first] != expected:
            raise RuntimeError('Connected target firmware differs from this HEX/AXF; use the correct target/files, or --flash-only for an already saved record')


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(v) for key, v in value.items()}
    if isinstance(value, list):
        return [finite_json(v) for v in value]
    return value


def active_run_summaries(records, trigger_index):
    """Keep separate launches separate, including mixed historical stop tails."""
    runs = []
    start = None
    for index in range(len(records)+1):
        active = index < len(records) and records[index]['active']
        if active and start is None:
            start = index
        if not active and start is not None:
            rows = records[start:index]
            def peak(values):
                return max((abs(v) for v in values if math.isfinite(v)), default=None)
            runs.append({
                'start_index': start, 'end_index': index-1, 'active_records': len(rows),
                'start_elapsed_ms': rows[0]['elapsed_ms'],
                'end_elapsed_ms': rows[-1]['elapsed_ms'],
                'observed_duration_ms': (rows[-1]['tick_ms']-rows[0]['tick_ms']) & 0xffffffff,
                'first_phase': rows[0]['phase'], 'last_phase': rows[-1]['phase'],
                'stop_observed': index < len(records),
                'starts_after_trigger': 0 <= trigger_index < start,
                'max_abs_pitch_deg': peak(r['body_pitch_deg'] for r in rows),
                'max_abs_speed_m_s': peak(r['state'][3] for r in rows),
                'max_abs_position_error_m': peak(r['position_error_m'] for r in rows)})
            start = None
    return runs


def export(header, raw_rows, output, source, raw_configuration=b''):
    if header['version']>=3:
        header['configuration'].update(configuration_decode(raw_configuration))
    records = rows_decode(raw_rows, header['count'], header['start_tick_ms'],header['version'],header['configuration'])
    if header['build_id'] in (2026110802,2026110803,2026110804,2026110805,2026110806,2026110807,2026110808,2026110809,2026110810,2026110811,2026110812,2026110813,2026110814,2026110815,2026110817,2026110818,2026110819,2026110820,2026110821,2026110822,2026110823,2026111201,2026111202,2026111203,2026111204,2026111205,2026111206,2026111207,2026111208,2026111211,2026111212,2026111213,2026111214,2026111215,2026111216,2026111217,2026111218,2026111219,2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['control_mode']='native LQR'
        header['native_lqr_semantics']='u=-K(x-reference), 1.47kg nominal DLQR; no row/derivative scaling, intermediate Tp clamp, adaptive lean or extra wheel damping; motor current caps remain'
    if header['build_id']==2026110810:
        header['native_lqr_semantics']='u=-K(x-reference), coupled nominal40mm body-COM-height plant; measured-current gravity-bias observer sets phi reference, no extra torque scaling/damping; motor caps remain'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110811:
        header['native_lqr_semantics']='u=-K(x-reference), coupled nominal40mm body-COM-height plant, 9cm leg target; gravity-bias observer disabled by default, full wheel/leg LQR and motor caps retained'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110812:
        header['native_lqr_semantics']='u=-K(x-reference), coupled nominal40mm body-COM-height plant, 9cm leg target; gravity-bias observer disabled by default; differential PD has original gains/cap and no extra virtual torque slew, driver current slew/caps retained'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110813:
        header['native_lqr_semantics']='0812 native LQR unchanged,9cm/2A; wheel3/7 voltage capability readback accepts6V, joint maximum remains5V; actual wheel voltage still bounded by driver PWM window'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110814:
        header['native_lqr_semantics']='0812 native LQR unchanged,9cm/2A; wheel3/7 voltage capability accepts7.2V, joint maximum remains5V; wheel candidate18kHz/0..89% window clamps actual vector near7.194V@14V'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110815:
        header['native_lqr_semantics']='0814 native DLQR, body angular-rate Q10->40; all other Q/R,9cm/2A,7.2V wheel capability and calibration unchanged; physical stability requires retest'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110817:
        header['native_lqr_semantics']='0815 native DLQR plus local neutral-leg PD Kp0.35,Kd0.011,cap0.12Nm from fixed9N loaded breakaway measurement;9cm/2A, wheel/sync and all LQR costs unchanged; physical stability requires retest'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110818:
        header['native_lqr_semantics']='0817 local neutral-leg PD raised to Kp0.60,Kd0.020,cap0.16Nm after measured0.12Nm cap was opposed by about0.025Nm shared Tp during rear fall;9cm/2A, wheel/sync and LQR costs unchanged; physical stability requires retest'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110819:
        header['native_lqr_semantics']='0818 plus remote-neutral startup parking origin follows measured x until pitch/rate/wheel/world-leg states remain quiet for300ms; normal locked parking,9cm/2A, wheel/sync and LQR gains unchanged; physical stability requires retest'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110820:
        header['native_lqr_semantics']='0819 startup origin capture capped at1s; fixed-position braking is then restored at the current location;9cm/2A and all gains unchanged; physical stability requires retest'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110821:
        header['native_lqr_semantics']='0820 plus bounded measured-joint-torque COM bias observer enabled only after parking-origin lock; phi target slew2deg/s, clamp10deg; native DLQR,9cm/2A unchanged; physical stability requires retest'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110822:
        header['native_lqr_semantics']='0821 COM trim limited to2deg and0.25deg/s; learning only near upright and low motion; native DLQR,9cm/2A and motor parameters unchanged'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026110823:
        header['native_lqr_semantics']='0822 unchanged except per-leg support feedforward 9.0N to7.1N for 1.16kg total mass;9cm/2A and motor parameters unchanged'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026111201:
        header['native_lqr_semantics']='0823 plus deterministic +0.5deg forward body trim at0.5deg/s; measured-torque COM observer default-off after wrong-sign field learning'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026111202:
        header['native_lqr_semantics']='1201 plus pose/support readiness dwell with zero native LQR output, followed by0.5s wheel/leg LQR ramp; local leg recovery remains active'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026111203:
        header['native_lqr_semantics']='1202 preparation keeps immediate bounded wheel body/rate balance while capturing initial world-leg tilt as theta reference; leg LQR ramps only after readiness dwell'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026111204:
        header['native_lqr_semantics']='1203 plus8.0N per-leg support feedforward for measured1.6kg total mass; current limits and LQR gains unchanged'
    if header['build_id']==2026111205:
        header['position_reference_semantics']='native LQR local position reference, constrained to +/-5cm around measured travel during drive, reversal, release and parking'
        header['native_lqr_semantics']='1204 plus native-LQR position reference constrained to +/-5cm around measured travel each cycle; prevents stale metre-scale tracking error after reversal/release; gains and current limits unchanged'
    if header['build_id']==2026111206:
        header['position_reference_semantics']='native LQR local position reference, constrained to +/-5cm around measured travel during drive, reversal, release and parking'
        header['native_lqr_semantics']='1205 plus9N/leg at24N/s,1s overlength ramp and fixed0.25s PREPARE before full wheel/ramping leg LQR;2A current limits unchanged'
    if header['build_id']==2026111207:
        header['position_reference_semantics']='native LQR local position reference, constrained to +/-5cm around measured travel during drive, reversal, release and parking'
        header['native_lqr_semantics']='FAILED experiment:1206 with common-wheel output inverted; field test reached40deg pitch within0.5s. Do not use this build; restored1206 polarity pending state-convention audit.'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id']==2026111208:
        header['position_reference_semantics']='1206 local +/-5cm position window retained'
        header['native_lqr_semantics']='1206 wheel/leg LQR polarity retained; neutral settled local-leg integral Ki0.8 Nm/(rad*s), cap0.08Nm, slew0.04Nm/s, antiwindup and OFF reset. angle_pd_nm includes this integral. Field stability not yet validated.'
        for row in records:
            row['adaptive_body_pitch_target_deg']=-math.degrees(row['reference'][4])
    if header['build_id'] in (2026111215,2026111216):
        header['native_lqr_semantics']='Complete coupled T/Tp feedback in all startup phases; no PREPARE wheel cap, hip-row deletion or forced common PD. Theta captured once then slewed independently across phases; true parking goal fixed from first cycle, one-shot release capture. Pose readiness only gates driving permission. Current caps, support/overlength law and CH5/fault protections retained. Physical stability unverified.'
        header['position_reference_semantics']='true fixed parking goal captured once at startup or drive release; bounded +/-5cm LQR input never rewrites goal; no startup following of measured travel'
        if header['build_id']==2026111216:
            header['native_lqr_semantics']+=' Native overlength support subtraction defaults OFF (Kp=0); original length-speed feedback,9N feedforward/24N-per-second ramp, axial cap and per-motor current limits retained. Positive runtime Kp explicitly re-enables soft unloading.'
    elif header['build_id']==2026111214:
        header['native_lqr_semantics']='1213 plus pose/reference-qualified PREPARE dwell before local-PD fade; theta reference slews continuously through RAMP/BALANCE rather than resetting. FAST/5ms IQ, fixed parking goal and continuous CH5 operation retained. Physical stability unverified.'
        header['position_reference_semantics']='true fixed parking goal when locked; bounded +/-5cm LQR position input; goal retained after stop'
    if header['build_id'] in (2026111217,2026111218,2026111219,2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['native_lqr_semantics']='Joint1.600kg DLQR, Q=[800,15,200,40,200,80], R=[100000,10000],5ms,40mm estimated COM height; complete coupled T/Tp in every startup phase, no row scaling or additional common pose PD. Model-consistent theta equilibrium and Tp feedforward accompany the applied phi trim. Leg-length velocity PI Kp30/Ki80 has bounded4.5N integral,3N/s rate and support-feedforward-ready gate. Motor2A caps and10A/s current slew retained; no run-duration cutoff. Physical standing, recovery and parking remain unverified.'
        header['position_reference_semantics']='true parking goal captured once at startup or drive release and retained after stop; parking LQR input bounded to +/-0.30m without rewriting goal; driving uses +/-0.05m local tracking window'
        header['native_lqr_design']={
            'total_mass_kg':1.6,'body_mass_kg':1.25,'non_body_mass_kg':.35,
            'body_com_height_m':.04,'body_com_height_estimated':True,
            'Q':[800,15,200,40,200,80],'R':[100000,10000],
            'sample_period_s':.005,'cubic_fit_sample_range_m':[.04,.15],
            'common_pose_pd_ratio':0,'physical_validation':'pending grounded trial'}
        header['height_control_build_defaults']={
            'velocity_kp_n_s_per_m':30,'velocity_ki_n_per_m':80,
            'integral_limit_n':4.5,'integral_rate_n_s':3,
            'support_feedforward_ready_error_n':.05,
            'integral_gate':'learn only after support feedforward is within0.05N of its target; OFF/fault resets; force/current bounds enforce antiwindup',
            'recording_limit':'V5 records configured speed Kp/Ki and resultant support commands; the integral state is not a separate row field'}
        header['equilibrium_reference_semantics']='m=body_mass/2, ml=leg_mass_per_side, h=estimated body COM height, L=mean measured leg length: theta_eq=m*h*phi_ref/((m+ml/2)*L), Tp_ff=-m*g*h*phi_ref, wheel feedforward=0. reference[0] records the once-captured startup theta slewing toward theta_eq; reference[4] is applied phi_ref. leg_Tp_nm includes joint-DLQR Tp plus Tp_ff.'
        header['runtime_stop_semantics']='CH5/manual OFF and control faults stop outputs; no running timer. Recorder capacity or the2s stopped logging tail does not stop a running controller.'
        if header['build_id'] in (2026111218,2026111219,2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
            header['recording_boundary_semantics']='Freeze the first run before a new activation enters its stopped observation tail; keep the first trigger reason and exclude all new-run rows. DQ commands are sampled after successful service enqueue; inactive or unsent slots are invalid.'
        if header['build_id'] in (2026111219,2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
            header['current_command_schedule_semantics']='Keep the 5ms absolute command phase through HAL tick jitter; send at most once per service call and skip expired slots after long gaps. No queued catch-up burst. ACK, STOP and hardware expiry protections remain.'
    if header['build_id']==2026111220:
        header['native_lqr_design']['Q']=[800,15,200,80,200,160]
        header['native_lqr_semantics']=header['native_lqr_semantics'].replace('Q=[800,15,200,40,200,80]','Q=[800,15,200,80,200,160]')
        header['parking_damping_candidate']='Full joint DARE redesign with speed/body-rate costs80/160; precise atan2f attitude conversion, unchanged fusion gains, and observed IMU DWT timing. Offline free/locked-hip/filter/delay screens are not hardware parking proof.'
    if header['build_id'] in (2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['imu_angle_semantics']='Precise atan2f roll/yaw, unchanged Mahony fusion gains and gyro input; observational DWT sample-path timing includes preemption.'
    if header['build_id'] in (2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['parking_revision']='Restore1219 Q40/80 after1220 rearward motion; stationary parking remains unresolved. Preserve precise attitude conversion and command-phase repairs.'
        header['fast_speed_guard_semantics']='FAST int16 speed endpoints retain telemetry but invalidate control kinematics and stop the active group; encoded wheel speed limits are not measured physical speed peaks.'
        header['output_failure_snapshot_semantics']='First output failure is latched in g_motor_foc_failure RAM before STOP invalidates session context; read with matching AXF before reset. V5 format unchanged; this snapshot is not persisted in the record.'
    if header['build_id']==2026111222:
        header['parking_revision']='1219 joint costs retained. Coherent remote/reference/goal/bias tail trace; default shadow gravity-bias estimation, active correction OFF. Stationary parking and equal-command fore/aft response still require physical validation.'
        header['gravity_bias_semantics']={
            'application_default':0,'shadow_default':1,
            'meaning':'Effective sagittal gravity/attitude bias estimate; not an independently measured physical COM.',
            'equilibrium':'theta=m*(h*phi-b_applied)/((m+ml/2)*L); Tp_ff=m*g*(b_applied-h*phi); b_applied=h*applied_bias_phi.',
            'learning':'Only neutral accepted control, BALANCE phase, quiet kinematics, support-ready, symmetric legs, unsaturated commands and fresh measured joint Iq; continuous dwell before learning/application. Motion freezes estimate, proposal and any accepted application.',
            'limits':'0.25deg/s and2deg maximum adaptive phi; manual0.5deg trim unchanged. OFF clears applied correction and dwell.',
            'recording':'V5 Flash format unchanged; bias/input/gate detail is in the matching128-byte balance tail ring.'}
    if header['build_id'] in (2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['native_lqr_semantics']='Startup uses the original six-state feedback; established BALANCE blends to jointly designed seven-state LQI, u=ff-K*error-Ki*z. Parking uses the unbounded true position error. Leg-length PI keeps its support-feedforward-ready gate; role-specific local/hard/remote current caps and10A/s current slew retained; no run-duration cutoff.'
        header['position_reference_semantics']='Ordinary driving integrates a local trajectory within +/-0.05m. Neutral braking and parking use the fixed true parking goal without clipping the position feedback. Integral qualification outside0.6m pauses accumulation only; it never clips the proportional error.'
        header['parking_revision']='Measured-stop motion governor and coupled seven-state LQI. Neutral braking and parking use the fixed true goal without position-error clipping. Physical validation pending.'
        header['normal_balance_servo_design']={
            'Q':[800,15,200,40,8000,80,100], 'R':[100000,10000],
            'state_order':['theta','dtheta','x_error','dx','phi','dphi','integral_x_error'],
            'sample_s':.005, 'startup':'Original six-state feedback until BALANCE; both complete servo rows blend over0.5s.',
            'integral':'Output uses z_k, then z_next=z_k+dt*bounded_true_error; bound2m*s and error update0.3m. Freeze outside0.6m parking/0.05m driving, invalid input, excessive motion/pose, or actuator saturation. OFF/fault clears.',
            'parking_position_error':'True x minus the fixed parking goal, without a30cm/60cm P-input cap.',
            'governor':'Raw neutral/reversal intent freezes the goal and brakes at0.8m/s2; measured speed below0.04m/s for100ms before releasing the latest direction, with0.4m/s2 restart.',
            'gravity_application_default':0,'shadow_default':1,
            'runtime_debug':'g_balance_servo and g_balance_motion_reference; existing128-byte tail stores actual references, true goal and output torques.',
            'limits':'Current/torque/driver health/FAST endpoint/fall/CH5 protections remain enforced.',
            'physical_validation':'Requires a new grounded trial; model results are not hardware parking proof.'}
    if header['build_id'] in (2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['parking_revision']='Measured-stop parking LQI with a conservative wheel-port drag design proxy; both full K/Ki rows blend over0.5s after governor enters PARKED. Driving, braking and startup retain1424 gains. Physical parking improvement requires a new trial.'
        header['native_lqr_semantics']+=' Measured-stop parking uses separately generated complete T/Tp and Ki rows with the same Q/R costs and a0.00025Nm*s/rad wheel-port design load; the load is not an identified friction measurement.'
        header['normal_balance_servo_design']['parking_schedule']={
            'design_wheel_port_drag_nm_s_per_rad':.00025,
            'wheel_port_rate':'dx/R-dtheta in the analytical plant; observer carrier-motion approximation remains',
            'Q':[800,15,200,40,8000,80,100], 'R':[100000,10000],
            'activation':'Established BALANCE, accepted neutral forward/yaw request, neutral ready, governor neither tracking nor braking. Drive-release enters PARKED after measured low-speed dwell; initial neutral startup is already PARKED.',
            'transition_s':.5,'integral_state':'One retained physical z state; both K and Ki blend. OFF/fault clears both blends and z.',
            'validation_limit':'Design proxy selected using uncertain-model and actual production-C current-slew screens; not a measured robot parameter or hardware proof.'}
        header['normal_balance_servo_design']['runtime_debug']='180-byte RAM rows at80ms, latest20.48s capacity: both legs angle/rate/length, z_next, output I torques, gates, governor phase, measured-Iq-derived Tp and parking blend. V5 persistent format unchanged.'
    if header['build_id'] in (2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['native_lqr_semantics']+=' Position-integral eligibility includes only float-roundoff tolerance at the window boundary; physical position, speed/pose, I bounds and actuator limits are unchanged.'
        header['normal_balance_servo_design']['position_gate_numerics']='4*FLT_EPSILON*max(1,abs(x),abs(goal)) comparison tolerance; no P-reference/goal changes or physical-limit relaxation.'
    if header['build_id'] in (2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['native_lqr_semantics']+=' User-requested3A local limits and STOP-synchronized driver IQ_LIMIT, requiring successful ACK plus exact parameter echo before torque. Driver voltage,10A/s slew and independent phase protection are unchanged.'
        header['trim_application_semantics']='Native startup_enable=1 holds manual trim at0 through PREPARE/RAMP, allowing the original5deg/250ms pose qualification at the initial neutral target. Only after BALANCE does the configured trim slew at0.5deg/s. startup_enable=0 retains the former immediate slew. OFF clears applied trim.'
        header['current_limit_semantics']={'local_limit_a':3.0,'fixed_remote_iq_limit_a':3.0,
            'joint_final_nm_ceiling':.7,'effective_limit':'min(local, verified driver IQ_LIMIT); written under full-group STOP and reapplied after driver restart',
            'phase_protection':'Independent driver phase-current trip is retained; this is a request limit, not proof of sustained3A shaft current.',
            'persistence':'Driver parameter3 is RAM; H7 restores/validates it after reboot. No driver calibration-journal write.'}
    if header['build_id'] in (2026111528,2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['native_lqr_semantics']=header['native_lqr_semantics'].replace('Measured-stop parking uses','Forward-neutral parking uses')
        header['parking_revision']='Accepted forward-neutral request starts complete parking K/Ki takeover immediately, including BRAKING with nonzero speed/reference or yaw. Full blend takes0.5s;3A limits and measured-stop reversal guard retained. Physical parking improvement requires a new trial.'
        header['normal_balance_servo_design']['parking_schedule']['activation']='Established BALANCE, TARGET_READY, neutral_ready and abs(requested_forward)<=1e-6m/s. First accepted forward-neutral tick starts the blend, independent of governor phase, measured/reference speed and yaw request. Nonzero forward request ramps back to driving gains.'
        header['normal_balance_servo_design']['parking_schedule']['reference_continuity']='Neutral freezes the existing goal once; velocity reference keeps its0.8m/s2 brake ramp. Same retained z state and full K/Ki transition; no goal recapture. Reversal still requires measured speed below0.04m/s for100ms.'
    if header['build_id'] in (2026111529,2026111530,2026111531,2026111533,2026111601,2026111602):
        header['native_lqr_semantics']=header['native_lqr_semantics'].replace('with the same Q/R costs','with unchanged Q and parking-specific R=[100000,40000] costs')
        header['parking_revision']='Bounded real FAST-history pairing retains10ms freshness/6ms same-leg skew. Forward-neutral parking starts immediately and blends0.5s; complete coupled parking K/Ki regenerated withR=[100000,40000] to reduce dependence on weak common-hip response.3A and support/startup/driving protections retained. Hardware parking improvement requires a new trial.'
        header['normal_balance_servo_design']['parking_schedule']['R']=[100000,40000]
        header['normal_balance_servo_design']['parking_schedule']['effort_allocation']='Hip effort cost increased jointly in seven-state DARE. Q/Qi, wheel-port design proxy, physical z, equilibrium references and support allocation are unchanged. Weak-hip response is a robustness hypothesis based on the1528 trace, not an identified actuator model.'
        header['feedback_pairing_semantics']={'freshness_ms':10,'max_same_leg_skew_ms':6,'rx_history_samples_per_joint':3,
            'selection':'Newest legal pair from bounded real FAST receive history; both latest nodes must still pass the existing health/freshness gates.',
            'timestamps':'Original local FIFO-dequeue timestamps; no extrapolation, retimestamping or hardware-sampling synchronization claim.',
            'invalidation':'Transport changes, bad FAST, clipping and unwrap reseeding must prevent reuse of pre-event history.',
            'limits':'Existing freshness, skew, CH5, driver, pose and3A current guards retained.'}
    if header['build_id'] in (2026111530,2026111531,2026111533,2026111601,2026111602):
        header['feedback_failure_diagnostics']={
            'ram_symbol':'g_motor_foc_feedback_failure',
            'source':'Original GetFeedback health snapshot selected by the observer that triggered the robot feedback fault; no later live-node reread.',
            'contents':'Online rejection bitmask; STATUS/DIAGNOSTIC/FAST counts and ages; epochs, health flags, faults and joint configuration checks.',
            'lifetime':'Retained across feedback recovery and STOP; cleared at initialization or the next accepted active FOC run. RAM only, lost on reset.',
            'reader':'scripts/read_feedback_failure.py',
            'control_change':'Diagnostics only; same1529 parking/driving gains, health thresholds and3A current limits.'}
    if header['build_id'] in (2026111531,2026111533,2026111601,2026111602):
        header['native_dynamic_hip_gain']={
            'default':1.5,'runtime_field':'g_balance.leg_scale','valid_range':[0.0,2.0],
            'formula':'Tp=leg_scale*(-Khip*error-Ki_hip*z)+equilibrium_Tp',
            'scope':'Both native six-state and LQI dynamic hip feedback; recorded hip integral term includes the actual multiplier.',
            'unscaled':'Wheel feedback, radial support and static gravity feedforward.',
            'hardware_validation':'User-requested grounded ratio trial; effect requires real robot observation.'}
        header['feedback_failure_diagnostics']['control_change']='Native dynamic hip feedback gain1.5. Original health thresholds,3A limits and1530 feedback diagnostics retained.'
    if header['build_id']==2026111533:
        header['single_leg_recovery']={'default_enabled':True,'runtime_field':'g_dual_hold.single_recovery_enable',
            'scope':'Native established balance only; extra restorative Tp on the dominant world-angle-error leg; no mirrored assist.',
            'limit_nm':0.08,'error_deadband_deg':4,'split_onset_deg':6,'body_fadeout_deg':[10,12],
            'debug_field':'g_dual_hold.single_recovery_torque_nm',
            'validation':'Candidate requires physical disturbance/release trial; implementation checks do not establish stability.'}
        header['feedback_failure_diagnostics']['control_change']='1533 bounded asymmetric recovery candidate; native common Tp gain1.5,3A and health protections retained.'
    if header['build_id']==2026111423:
        header['parking_revision']='1222 joint costs and offset-consistent equilibrium retained. Gravity-bias application and shadow estimation enabled by default; manual trim -4deg, parking integral OFF. Stationary parking still requires physical validation.'
        header['gravity_bias_semantics']={
            'application_default':1,'shadow_default':1,
            'manual_trim_default_deg':-4.0,
            'adaptive_phi_limit_deg':2.0,'adaptive_phi_rate_deg_s':0.25,
            'learning_dwell_ms':300,'application_dwell_ms':1500,'application_min_samples':200,
            'off_clears_applied_correction':True,'off_disables_observer':False,
            'meaning':'Effective sagittal gravity/attitude bias estimate; not an independently measured physical COM.',
            'equilibrium':'theta=m*(h*phi-b_applied)/((m+ml/2)*L); Tp_ff=m*g*(b_applied-h*phi); b_applied=h*applied_bias_phi.',
            'learning':'Original1222 gates retained: only neutral accepted control, BALANCE phase, quiet kinematics, support-ready, symmetric legs, unsaturated commands and fresh measured joint Iq; continuous dwell before learning/application. Motion freezes estimate, proposal and any accepted application.',
            'limits':'0.25deg/s and2deg maximum adaptive phi; manual -4deg trim. OFF clears applied correction and dwell but preserves the observer application enable setting.',
            'recording':'V5 Flash format unchanged; bias/input/gate detail is in the matching128-byte balance tail ring.'}
        header['equilibrium_reference_semantics']='m=body_mass/2, ml=leg_mass_per_side, h=estimated body COM height, L=mean measured leg length: theta_eq=m*(h*phi_ref-b_applied)/((m+ml/2)*L), Tp_ff=m*g*(b_applied-h*phi_ref), wheel feedforward=0; b_applied=h*applied_bias_phi. reference[0] records the once-captured startup theta slewing toward theta_eq; reference[4] is applied phi_ref. leg_Tp_nm includes joint-DLQR Tp plus Tp_ff.'
    if header['build_id']==2026111213:
        header['native_lqr_semantics']='1212 FAST/5ms command/fixed parking repairs retained. Local absolute leg recovery active in PREPARE, fades through RAMP, zero in BALANCE; full LQR common Tp and differential sync retained. No runtime cutoff; experimental integral OFF. Hardware standing outcome pending.'
        header['position_reference_semantics']='true fixed parking goal when locked; bounded +/-5cm LQR position input; actual goal retained after stop'
    if header['build_id']==2026111212:
        header['native_lqr_semantics']='200Hz FAST wheel+joint kinematics and5ms IQ command schedule; unchanged LQR gains/polarity. Finite1s launch capture, fixed true parking goal, bounded LQR position input, one-shot release capture. No run-duration cutoff; failed1208 integral OFF. Hardware stability pending.'
        header['position_reference_semantics']='true fixed parking goal when locked, retained in stopped tail; LQR position contribution uses bounded +/-5cm input, not the full recorded position error'
    if header['build_id']==2026111211:
        header['native_lqr_semantics']='Continuous CH5/manual control, no capture time cutoff;1206 LQR polarity and motor caps retained, failed1208 integral disabled.40ms onboard logging capacity does not stop motors. Rearward drift unresolved.'
        header['position_reference_semantics']='1206 local +/-5cm position window retained'
    if header['build_id']==2026111210:
        header['control_mode']='native LQR, supported short capture'
        header['position_reference_semantics']='1206 local +/-5cm position window retained'
        header['native_lqr_semantics']='1206 control polarity;1208 integral disabled. Capture-only image:1500ms fixed stop, CH5 OFF rearm, saved/unread trial blocks launch. Not a validated standing controller.'
        header['capture_limit_ms']=1500
    if header['build_id'] in (2026100720,2026100821,2026100822,2026100823,2026100824,2026100825,2026100826,2026100827,2026100828,2026100829,2026100830,2026100831,2026101001,2026101002,2026101003,2026101004,2026101005,2026101006,2026101007,2026101008,2026101009,2026101010,2026101011,2026101012,2026101013,2026101014,2026101015):
        for r in records:
            r['parking_damping_command_nm']=(r['wheel_requested_nm']-sum(r['wheel_components_nm'])
                if r['active'] and r['phase']==3 and r['parking_damping_active'] else 0.0)
    if header['build_id'] in (2026100821,2026100822,2026100823,2026100824,2026100825,2026100826,2026100827,2026100828,2026100829,2026100830,2026100831,2026101001,2026101002,2026101003,2026101004,2026101005,2026101006,2026101007,2026101008,2026101009,2026101010,2026101011,2026101012,2026101013,2026101014,2026101015):
        for r in records:
            r['parking_lean_deg']=math.degrees(r['reference'][0])
            r['parking_lean_learning']=bool(r['flags']&65536)
    if header['build_id'] in (2026100829,2026100830,2026100831,2026101001,2026101002,2026101003,2026101004,2026101005,2026101006,2026101007,2026101008,2026101009,2026101010,2026101011,2026101012,2026101013,2026101014,2026101015):
        header['lean_reference_semantics']='reference[0] is slow equilibrium plus bounded fast tracking lean; separate terms available only in RAM debug'
        for r in records:
            r['combined_lean_deg']=math.degrees(r['reference'][0])
    if header['version']==4 and header['build_id'] in (2026100827,2026100828,2026100829,2026100830,2026100831,2026101001,2026101002,2026101003,2026101004,2026101005,2026101006,2026101007,2026101008,2026101009,2026101010,2026101011,2026101012,2026101013,2026101014,2026101015):
        for r in records:
            r['yaw_integral_active']=bool(r['flags']&(1<<30))
            r['yaw_integral_blocked']=bool(r['flags']&(1<<31))
    if header['version']==4 and header['build_id'] in (2026100826,2026100827,2026100828,2026100829,2026100830,2026100831,2026101001,2026101002,2026101003,2026101004,2026101005,2026101006,2026101007,2026101008,2026101009,2026101010,2026101011,2026101012,2026101013,2026101014,2026101015):
        for r in records:
            r['motion_lean_learning']=bool(r['flags']&(1<<28))
            r['motion_stall_hold']=bool(r['flags']&(1<<29))
            r['motion_damping_active']=bool(r['flags']&(1<<23))
            r['motion_speed_error_m_s']=r['state'][3]-r['forward_target_m_s']
            r['motion_damping_command_nm']=(r['wheel_requested_nm']-sum(r['wheel_components_nm'])
                if r['active'] and r['phase']==3 and r['motion_damping_active'] else 0.0)
    if header['build_id'] in VOLTAGE_PROBE_BUILDS:
        for r in records:
            r['direction_probe_nm']=[r['total_pitch_nm'][s]-r['angle_pd_nm'][s]-r['sync_nm'][s]-r['lqr_pitch_nm'][s]
                                     for s in range(2)]
    active = [r for r in records if r['active']]
    if header['build_id']==2026111601:
        header.pop('normal_balance_servo_design',None)
        header['native_lqr_semantics']='Complete six-state LQR + bounded slow COM observation/application; native dynamic Tp row scaled1.5, static gravity feedforward unscaled; LQI disabled.3A and10A/s retained. Physical performance requires a grounded trial.'
        header['position_reference_semantics']='Fixed true parking goal; six-state parking input limited to+/-0.30m, driving input to+/-0.05m; no LQI position integral.'
        header['steady_correction_owner']='active gated COM observer; LQI default OFF, conflicting enable requests STOP'
        header['parking_revision']='Six-state LQR + active slow COM compensation, native dynamic hip gain1.5. Physical parking/COM accuracy not yet validated.'
        header['gravity_bias_semantics']={
            'application_default':1,'shadow_default':1,'manual_trim_default_deg':-4.0,
            'adaptive_phi_limit_deg':2.0,'adaptive_phi_rate_deg_s':.25,
            'learning_dwell_ms':300,'application_dwell_ms':1500,'application_min_samples':200,
            'meaning':'Effective sagittal gravity offset inferred from measured Iq/J; not an independent3D COM measurement.',
            'learning':'Quiet neutral BALANCE only; movement, saturation, torque loss or confidence loss freeze estimate and applied correction.',
            'equilibrium':'b_applied=h*applied_bias_phi; phi_ref=applied_bias_phi-manual_trim; same b_applied in theta/Tp equilibrium, no duplicate gravity feedforward.',
            'off_clears_applied_correction':True,'off_preserves_estimate':True,
            'recording':'V5 layout unchanged;180-byte balance ring records bias estimate, applied phi and gates.'}
        header['native_dynamic_hip_gain']['default']=1.5
        header['native_dynamic_hip_gain']['scope']='Native six-state feedback; LQI is disabled by default.'
        header['feedback_failure_diagnostics']['control_change']='Active gated COM compensation; LQI off;1.5 dynamic Tp,3A and feedback protections retained.'
    if header['build_id']==2026111602:
        header['normal_balance_servo_design']['position_integral_default_enabled']=False
        header['normal_balance_servo_design']['integral']='z_k=z_next=0, position integration and wheel/hip I torque OFF; existing feedback K schedule retained.'
        header['parking_revision']='1531 unchanged feedback/I/current/trim +0821 simple drift-to-theta reference adaptation. Not Iq-based COM estimation; physical validation pending.'
        header['simple_parking_compensation']={'default_enabled':True,'source_build':2026100821,
            'speed_filter_s':.3,'speed_deadband_m_s':.005,'gain_rad_per_m':.05,
            'rate_limit_deg_s':.2,'angle_limit_deg':2,'learning_speed_limit_m_s':.15,'settle_ms':1000,
            'reference':'Add learned angle to existing equilibrium theta; phi/Tp feedforward and fixed goal unchanged.',
            'retained':'1531 state feedback K, gain table values,1.5 dynamic hip gain,3A/10A/s, trim and protections; position-I accumulation/output OFF per user clarification.',
            'diagnostics':'g_balance.parking_lean_rad/filtered_speed/learning; frozen balance reference[0] contains applied angle.'}
    report = {'source': source, 'header': header, 'records': records,
              'limitations': 'Torque/support/voltage are commands, not measurements. The recorder does not control motor stopping; controller faults can independently zero outputs while post-stop logging continues.'}
    if active:
        report['summary'] = {'active_records': len(active),
            'max_abs_pitch_deg': max(abs(r['body_pitch_deg']) for r in active),
            'max_abs_left_torque_command_nm': max(abs(r['left_wheel_torque_command_nm']) for r in active),
            'wheel_voltage_saturated_records': sum(r['wheel_voltage_saturated'] for r in active),
            'sync_priority_limited_records': sum(r['sync_priority_limited'] for r in active),
            'parking_damping_active_records': sum(r['parking_damping_active'] for r in active),
            'last_phase': active[-1]['phase'], 'last_fault_reason': records[-1]['fault_reason'],
            'historical_fault_reason_at_start': records[0]['fault_reason'],
            'new_fault_events': [{'elapsed_ms':r['elapsed_ms'],'reason':r['fault_reason'],
                                  'reason_text':r['fault_reason_text']} for r in records if r['new_fault_event']]}
    summary=report.setdefault('summary',{})
    runs=active_run_summaries(records,header['trigger_index'])
    summary['active_run_count']=len(runs)
    summary['active_runs']=runs
    summary['multiple_control_runs']=len(runs)>1
    summary['restart_after_trigger']=any(r['starts_after_trigger'] for r in runs)
    if len(runs)>1:
        report['integrity_warnings']=['This historical record contains multiple control runs. Aggregate active metrics combine runs; use active_runs for each launch. The last active run may be truncated by an earlier trigger tail.']
    if header['build_id'] in (2026100821,2026100822,2026100823,2026100824,2026100825,2026100826,2026100827,2026100828,2026100829,2026100830,2026100831,2026101001,2026101002,2026101003,2026101004,2026101005,2026101006,2026101007,2026101008,2026101009,2026101010,2026101011,2026101012,2026101013,2026101014,2026101015):
        summary['parking_lean_learning_records']=sum(r['parking_lean_learning'] for r in active)
        summary['max_abs_parking_lean_deg']=max((abs(r['parking_lean_deg']) for r in records),default=0)
    if header['version']>=5:
        report['limitations'] += ' V5 Iq is driver FAST feedback, not a torque/load-cell measurement. Requested/applied Iq can retain cached values after STOP; use mode/flags and actual Iq. Samples are not simultaneous between independent CAN nodes.'
        summary['peak_abs_measured_iq_a']=[max((abs(r['foc_actual_iq_a'][i]) for r in records if math.isfinite(r['foc_actual_iq_a'][i])),default=None) for i in range(6)]
    stopped=[r for r in records if not r['active']]
    summary['inactive_records']=len(stopped)
    summary['first_inactive_elapsed_ms']=stopped[0]['elapsed_ms'] if stopped else None
    valid_pitch=[abs(r['body_pitch_deg']) for r in records if math.isfinite(r['body_pitch_deg'])]
    summary['max_abs_pitch_including_tail_deg']=max(valid_pitch) if valid_pitch else None
    trigger=header['trigger_index']
    if trigger<len(records):
        summary['trigger_elapsed_ms']=records[trigger]['elapsed_ms']
        summary['post_trigger_records']=len(records)-trigger-1
        summary['post_trigger_duration_ms']=(records[-1]['tick_ms']-records[trigger]['tick_ms'])&0xffffffff
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(finite_json(report), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    csv_path = output.with_suffix('.csv')
    with csv_path.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        details = []
        for joint in ('LF','LR','RF','RR'):
            details += [joint+'_'+name for name in ('angle_deg','shaft_speed_rad_s','model_speed_rad_s',
                       'feedback_age_ms','feedback_valid','torque_command_mNm')]
        for side in ('left','right'):
            details += [side+'_'+name for name in ('length_speed_mm_s','length_position_speed_mm_s',
                       'angle_speed_deg_s','PD_mNm','sync_mNm','LQR_Tp_mNm','total_Tp_mNm','angle_reference_deg')]
        details += ['support_feedforward_N','control_period_ms','left_direction_probe_mNm','right_direction_probe_mNm',
                    'parking_damping_active','parking_damping_command_mNm','parking_lean_deg','parking_lean_learning',
                    'forward_target_mm_s','yaw_target_rad_s','yaw_rate_rad_s','differential_applied_mNm',
                    'right_torque_command_mNm','left_requested_mNm','right_requested_mNm','motion_status','neutral_ready',
                    'motion_speed_error_mm_s','motion_damping_mNm','motion_lean_learning','motion_stall_hold']
        foc_details=[]
        for node in (1,2,3,5,6,7):
            foc_details += [f'ID{node}_'+name for name in ('requested_Iq_A','applied_Iq_A','actual_Iq_A','current_limit_A','FAST_age_ms','flags','mode','faults')]
        foc_details += ['left_support_requested_N','right_support_requested_N','left_support_limit_N','right_support_limit_N',
                        'left_support_saturated','right_support_saturated','feedback_detail','feedback_motor_id','feedback_ms']
        writer.writerow(('elapsed_ms','phase','active','left_torque_command_mNm','left_voltage_mv',
            'right_voltage_mv','body_pitch_deg','body_rate_deg_s','speed_mm_s','position_error_mm',
            'left_length_mm','right_length_mm','left_angle_deg','right_angle_deg','left_support_N',
            'right_support_N','leg_Tp_mNm','LF_voltage_mv','LR_voltage_mv','RF_voltage_mv','RR_voltage_mv',
            'raw_left_speed_rad_s','raw_right_speed_rad_s','wheel_voltage_saturated','position_locked',
            'validity','control_status','fault_reason','fault_count',
            'T_theta_mNm','T_dtheta_mNm','T_position_mNm','T_speed_mNm','T_pitch_mNm','T_body_rate_mNm',
            'leg_Tp_saturated','joint_output_saturated','fault_reason_text','new_fault_event','sync_priority_limited',*details,*foc_details))
        for r in records:
            extra = ['']*len(details)
            if header['version']>=3:
                extra = []
                for i in range(4):
                    extra += [math.degrees(r['joint_angle_rad'][i]),r['joint_shaft_speed_rad_s'][i],
                              r['joint_model_speed_rad_s'][i],r['joint_feedback_age_ms'][i],
                              int(r['joint_feedback_valid'][i]),1000*r['joint_torque_command_nm'][i]]
                for i in range(2):
                    extra += [1000*r['length_speed_m_s'][i],1000*r['length_position_speed_m_s'][i],
                              math.degrees(r['angle_speed_rad_s'][i]),1000*r['angle_pd_nm'][i],
                              1000*r['sync_nm'][i],1000*r['lqr_pitch_nm'][i],
                              1000*r['total_pitch_nm'][i],math.degrees(r['angle_reference_rad'][i])]
                extra += [r['support_feedforward_n'],r['control_period_ms'],
                          *[1000*v for v in r.get('direction_probe_nm',[0,0])],
                          int(r['parking_damping_active']),1000*r.get('parking_damping_command_nm',0),
                          r.get('parking_lean_deg',''),int(r['parking_lean_learning']) if 'parking_lean_learning' in r else '']
                extra += ([1000*r['forward_target_m_s'],r['yaw_target_rad_s'],r['yaw_rate_rad_s'],
                           1000*r['differential_applied_nm'],1000*r['right_wheel_torque_command_nm'],
                           *[1000*v for v in r['wheel_requested_per_side_nm']],r['motion_status'],int(r['neutral_ready'])]
                          if header['version']>=4 else ['']*9)
                extra += ([1000*r['motion_speed_error_m_s'],1000*r['motion_damping_command_nm'],
                           int(r['motion_lean_learning']),int(r['motion_stall_hold'])]
                          if header['version']>=5 or (header['version']==4 and header['build_id'] in (2026100826,2026100827,2026100828,2026100829,2026100830,2026100831,2026101001,2026101002,2026101003,2026101004,2026101005,2026101006,2026101007,2026101008,2026101009,2026101010,2026101011)) else ['']*4)
            foc_extra=['']*len(foc_details)
            if header['version']>=5:
                foc_extra=[]
                for i in range(6):
                    foc_extra += [r[name][i] for name in ('foc_requested_iq_a','foc_applied_iq_a','foc_actual_iq_a',
                        'foc_current_limit_a','foc_feedback_age_ms','foc_flags','foc_mode','foc_faults')]
                foc_extra += [*r['support_requested_n'],*r['support_limit_n'],*[int(x) for x in r['support_saturated']],
                              r['feedback_detail'],r['feedback_motor_id'],r['feedback_ms']]
            writer.writerow((r['elapsed_ms'],r['phase'],int(r['active']),1000*r['left_wheel_torque_command_nm'],
                *r['wheel_voltage_mv'],r['body_pitch_deg'],r['body_rate_deg_s'],1000*r['state'][3],
                1000*r['position_error_m'],*[1000*v for v in r['leg_length_m']],
                *[math.degrees(v) for v in r['leg_angle_rad']],*r['support_command_n'],1000*r['leg_Tp_nm'],
                *r['joint_voltage_mv'],*r['raw_wheel_speed_rad_s'],int(r['wheel_voltage_saturated']),
                int(r['position_locked']),r['validity'],r['control_status'],r['fault_reason'],r['fault_count'],
                *[1000*v for v in r['wheel_components_nm']],
                int(r['leg_Tp_saturated']),int(r['joint_output_saturated']),r['fault_reason_text'],
                int(r['new_fault_event']),int(r['sync_priority_limited']),*extra,*foc_extra))
    print(json.dumps(finite_json({k: v for k,v in report.items() if k != 'records'}), ensure_ascii=False, indent=2))
    print(f'JSON: {output}\nCSV: {csv_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axf', type=Path)
    parser.add_argument('--hex', type=Path, dest='hex_path')
    parser.add_argument('--flash-only', action='store_true', help='Read a committed Flash record, including after reboot; needs no AXF')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--openocd', default=OCD)
    args = parser.parse_args()
    if args.flash_only:
        first = read_blocks([('header', FLASH, HEADER.size//4)], args.openocd)['header']
        header = header_decode(first)
        config_blocks = [('configuration', FLASH+HEADER.size,header['configuration_bytes']//4)] if header['version']>=3 else []
        data = read_blocks([*config_blocks,('rows', FLASH+header['payload_offset'], header['count']*header['row_bytes']//4),
                            ('header_after', FLASH, HEADER.size//4)], args.openocd)
        if first != data['header_after']:
            raise RuntimeError('Flash save was in progress; retry after completion')
        raw_configuration = data.get('configuration',b'')
        if record_crc(first,data['rows'],raw_configuration) != header['crc32']:
            raise RuntimeError('Record CRC mismatch; incomplete/corrupted Flash save')
        export(header,data['rows'],args.output,'committed Flash, CRC verified',raw_configuration);return
    if not args.axf:
        parser.error('Pass --axf for RAM reading, or --flash-only for a completed Flash record')
    image = args.hex_path or args.axf.with_suffix('.hex')
    check_firmware(image,args.openocd)
    debug_at,data_at,control_at = symbols(args.axf)
    blocks = [('control',control_at,1),('debug',debug_at,16),('header',data_at,HEADER.size//4)]
    first = read_blocks(blocks,args.openocd)
    debug = struct.unpack('<13If2I',first['debug'])
    if struct.unpack('<I',first['control'])[0] or debug[9] or debug[1] not in (2,3,4) or not debug[5]:
        raise RuntimeError('RAM record is not frozen/disarmed, or rows only exist in Flash after reboot; use --flash-only after saving')
    header = header_decode(first['header'],require_commit=False)
    config_blocks = [('configuration',data_at+HEADER.size,header['configuration_bytes']//4)] if header['version']>=3 else []
    data = read_blocks([*config_blocks,('rows',data_at+header['payload_offset'],header['count']*header['row_bytes']//4),
                        *[(name+'_after',at,count) for name,at,count in blocks]],args.openocd)
    if any(first[name] != data[name+'_after'] for name,_,_ in blocks):
        raise RuntimeError('Record/control state changed during read; retry with CH5 OFF')
    export(header,data['rows'],args.output,'frozen RAM; matching firmware and stable headers checked',data.get('configuration',b''))


if __name__ == '__main__':
    main()
