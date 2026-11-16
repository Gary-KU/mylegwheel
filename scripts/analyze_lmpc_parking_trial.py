from pathlib import Path
import json
import struct
import math
from deploy_lmpc_parking_10801 import NEW,layout
ROOT=Path(__file__).resolve().parents[1]
FILE=sorted((ROOT/'diagnostics').glob('lmpc_10801_trial_*.json'))[-1]
r=json.loads(FILE.read_text());base=FILE.with_suffix('')
fields=['g_balance.parking_lean_enable','g_balance.parking_lean_gain_rad_per_m',
 'g_balance.parking_damping_gain_nm_per_m_s','g_balance.wheel_scale','g_balance.leg_scale',
 'g_balance.parking_correction_limit_m','g_balance.position_window_m','g_balance.back_emf_ratio',
 'g_dual_hold.angle_reference_mode','g_dual_hold.angle_target_rad','g_dual_hold.sync_ratio',
 'g_dual_hold.sync_limit_nm','g_dual_hold.sync_speed_kp','g_dual_hold.sync_rate_nm_s',
 'g_wheel_leg.leg[0].angle','g_wheel_leg.leg[1].angle','g_wheel_leg.body_pitch_rad',
 'g_wheel_leg.body_pitch_rate_rad_s','g_wheel_leg.stack_free_min_bytes']
objects=['g_balance','g_dual_hold','g_wheel_leg'];addr,_=layout(NEW,fields+objects)
values={}
for name in fields:
    parent=name.split('.')[0];blob=base.with_suffix('.'+parent+'.bin').read_bytes();offset=addr[name]-addr[parent]
    kind='<I' if name.endswith(('parking_lean_enable','angle_reference_mode','stack_free_min_bytes')) else '<f'
    values[name]=struct.unpack_from(kind,blob,offset)[0]
rows=r['rows'];prefix=r['startup']
gate_counts=dict(speed_over_025=sum(abs(t['state'][3])>.25 for t in rows),
    pitch_over_8deg=sum(abs(t['body_pitch_deg'])>8 for t in rows),
    body_rate_over_30deg_s=sum(abs(t['state'][5])>math.pi/6 for t in rows),
    wheel_limited=sum(t['wheel_limited'] for t in rows),tp_limited=sum(t['tp_limited'] for t in rows))
current_split=math.degrees(values['g_wheel_leg.leg[0].angle']-values['g_wheel_leg.leg[1].angle'])
startup_peak_joint=max(abs(v) for t in prefix for i,v in enumerate(t['voltage_mv']) if i not in (2,6))
start=prefix[0] if prefix else None
analysis=dict(source=str(FILE),current_parameters=values,sampled_gate_failures=gate_counts,
    retained_pose_split_deg=current_split,startup_peak_joint_voltage_mv=startup_peak_joint,
    latest_episode_start=start['tick_ms'] if start else None,
    latest_episode_recorded_duration_s=(rows[-1]['tick_ms']-start['tick_ms'])/1000 if start and rows else None,
    latest_episode_x_delta_m=rows[-1]['state'][2]-start['state'][2] if start and rows else None,
    max_solve_ms=r['solver']['max_cycles']/400000,
    limitations='Frozen ring samples every20ms; cannot prove health at every5ms or reconstruct missing middle states.')
FILE.with_suffix('.analysis.json').write_text(json.dumps(analysis,indent=2),encoding='utf-8')
print(json.dumps(analysis,indent=2))
