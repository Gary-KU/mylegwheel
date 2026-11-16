"""Read a frozen LMPC trial using its verified AXF layout; no target writes."""
from pathlib import Path
import csv
import json
import math
import struct
import time
from deploy_lmpc_parking_10801 import NEW,layout,safe_read,verify_image,snapshot,FIELDS

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'diagnostics'/('lmpc_parking_trial_'+time.strftime('%Y%m%d_%H%M%S'))
verify_image(NEW.with_suffix('.hex'))
before=snapshot(NEW,FIELDS)
if before['g_robot_control.enable'] or before['g_robot_control.remote_switch_on']:
    print(json.dumps(before,indent=2))
    raise SystemExit('CH5 is still ON. Close CH5 before reading a coherent frozen trial.')
names=['g_balance_trace','g_lmpc_startup_trace','g_lmpc','g_lmpc_control','g_robot_control',
       'g_balance','g_wheel_leg','g_dual_hold','g_wheel_output','g_joint_output']
addresses,sizes=layout(NEW,names,names)
if 'extern volatile ParkingLearningDebug' in (NEW.parents[2]/'App/Tasks/Inc/wheel_leg_task.h').read_text(encoding='utf-8'):
    extra,extra_sizes=layout(NEW,['g_parking_learning_debug'],['g_parking_learning_debug'])
    names+=['g_parking_learning_debug'];addresses.update(extra);sizes.update(extra_sizes)
raw=safe_read([(name,addresses[name],sizes[name]//4) for name in names])
after=snapshot(NEW,FIELDS)
heads=safe_read([('ring',addresses['g_balance_trace'],3),('startup',addresses['g_lmpc_startup_trace'],4)])
assert not after['g_robot_control.enable'] and not after['g_robot_control.remote_switch_on']
assert heads['ring']==raw['g_balance_trace'][:12] and heads['startup']==raw['g_lmpc_startup_trace'][:16]
for name,value in raw.items():OUT.with_suffix('.'+name+'.bin').write_bytes(value)
capacity=(sizes['g_balance_trace']-12)//64
assert sizes['g_balance_trace']==12+64*capacity and capacity in (256,512)
long_trace=capacity==512
next_row,count,sequence=struct.unpack('<3I',heads['ring']);assert next_row<capacity and count<=capacity
rows=[]
for k in range(count):
    index=(next_row-count+k)%capacity
    row=struct.unpack_from('<II6f4f2f2hf',raw['g_balance_trace'],12+64*index)
    x=list(row[2:8]);flags=row[1]
    rows.append(dict(tick_ms=row[0],flags=flags,phase=(flags>>8)&255,state=x,
        position_error_m=row[8],wheel_requested_nm=row[9],wheel_applied_nm=row[10],tp_nm=row[11],
        wheel_voltage_mv=list(row[14:16]),length_m=row[16],body_pitch_deg=-math.degrees(x[4]),
        mean_relative_leg_deg=90+math.degrees(x[0]+x[4]),wheel_limited=bool(flags&6),tp_limited=bool(flags&8)))
    if long_trace:
        rows[-1].update(parking_lean_rad=row[12],parking_filtered_speed_m_s=row[13],
            parking_learning=bool(flags&65536),parking_gate_mask=flags>>24)
def startup_row(buffer,offset):
    r=struct.unpack_from('<4I26f8h',buffer,offset)
    return dict(tick_ms=r[0],active=r[1],phase=r[2],fault=r[3],state=list(r[4:10]),
        reference=list(r[10:16]),leg_angle_rad=list(r[16:18]),angle_reference_rad=list(r[18:20]),
        tp_nm=r[20],local_pd_nm=list(r[21:23]),sync_nm=list(r[23:25]),support_n=list(r[25:27]),
        length_m=list(r[27:29]),wheel_nm=r[29],voltage_mv=list(r[30:38]))
prefix_count,episode,fault_valid,_=struct.unpack('<4I',heads['startup']);assert prefix_count<=128
prefix=[startup_row(raw['g_lmpc_startup_trace'],16+136*(k+1)) for k in range(prefix_count)]
fault=startup_row(raw['g_lmpc_startup_trace'],16) if fault_valid else None
d=struct.unpack('<7I4f3I16f2I',raw['g_lmpc'])
solver=dict(status=d[0],iterations=d[1],table=d[2],cycles=d[3],max_cycles=d[4],solves=d[5],
    failures=d[6],primal=d[7],dual=d[8],output_nm=list(d[9:11]),rho_mode=d[11],
    last_length_m=d[15],last_state=list(d[16:22]),last_reference=list(d[22:28]),
    last_limits_nm=list(d[28:30]),polish_reject=d[30],refinements=d[31])
c=struct.unpack('<8I5f',raw['g_lmpc_control'])
selector=dict(zip(['mode','fallback_events','solver_rejections','probe_attempts','good',
    'skipped','recoveries','last_solver_status','blend','selected_wheel','selected_tp','previous_wheel','previous_tp'],c))
summary={}
if rows:
    n=len(rows);tail=[r for r in rows if ((rows[-1]['tick_ms']-r['tick_ms'])&0xffffffff)<1000]
    summary=dict(duration_s=((rows[-1]['tick_ms']-rows[0]['tick_ms'])&0xffffffff)/1000,
        first_tick=rows[0]['tick_ms'],last_tick=rows[-1]['tick_ms'],
        first_x_m=rows[0]['state'][2],last_x_m=rows[-1]['state'][2],
        first_speed_m_s=rows[0]['state'][3],last_speed_m_s=rows[-1]['state'][3],
        peak_abs_speed_m_s=max(abs(r['state'][3]) for r in rows),
        peak_abs_parking_error_m=max(abs(r['position_error_m']) for r in rows),
        wheel_limited_rows=sum(r['wheel_limited'] for r in rows),tp_limited_rows=sum(r['tp_limited'] for r in rows),
        last_1s_mean_speed_m_s=sum(r['state'][3] for r in tail)/len(tail),
        last_1s_mean_pitch_deg=sum(r['body_pitch_deg'] for r in tail)/len(tail),
        last_1s_mean_relative_leg_deg=sum(r['mean_relative_leg_deg'] for r in tail)/len(tail))
learning=None
if 'g_parking_learning_debug' in raw:
    p=struct.unpack('<16I5f',raw['g_parking_learning_debug'])
    learning=dict(episode=p[0],tick_ms=p[1],gate_mask=p[2],settle_ms=p[3],samples=p[4],
        learning_samples=p[5],blocked_samples=p[6],gate_count=list(p[7:15]),input_limited_samples=p[15],
        filtered_speed_m_s=p[16],lean_rad=p[17],body_pitch_rad=p[18],body_rate_rad_s=p[19],leg_split_rad=p[20])
report=dict(axf=str(NEW),source='verified current LMPC image, CH5 OFF, stable frozen headers; no halt/reset/write',
    before=before,after=after,ring_header=[next_row,count,sequence],ring_capacity=capacity,startup_episode=episode,
    solver=solver,selector=selector,learning=learning,summary=summary,rows=rows,startup=prefix,startup_fault=fault)
OUT.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
with OUT.with_suffix('.csv').open('w',newline='',encoding='utf-8-sig') as stream:
    writer=csv.writer(stream);writer.writerow(['tick_ms','phase','x_m','speed_m_s','position_error_m',
        'body_pitch_deg','relative_leg_deg','T_nm','Tp_nm','left_mv','right_mv','wheel_limited','tp_limited',
        'parking_lean_rad','parking_filtered_speed_m_s','parking_learning','parking_gate_mask'])
    for r in rows:writer.writerow([r['tick_ms'],r['phase'],r['state'][2],r['state'][3],r['position_error_m'],
        r['body_pitch_deg'],r['mean_relative_leg_deg'],r['wheel_applied_nm'],r['tp_nm'],
        *r['wheel_voltage_mv'],r['wheel_limited'],r['tp_limited'],r.get('parking_lean_rad'),
        r.get('parking_filtered_speed_m_s'),r.get('parking_learning'),r.get('parking_gate_mask')])
print(json.dumps({k:v for k,v in report.items() if k not in ['rows','startup']},indent=2))
print('Startup rows',len(prefix),'ring rows',len(rows),'output',OUT.with_suffix('.json'))
