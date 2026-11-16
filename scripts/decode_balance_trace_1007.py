"""Decode 1007+ 132-byte RAM trace rows (not the older 64-byte format)."""
import argparse,json,struct
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('input',type=Path);a=parser.parse_args()
b=a.input.read_bytes();assert len(b)==12+256*132, 'Requires verified 1007 trace layout'
next_row,count,sequence=struct.unpack_from('<3I',b);assert next_row<256 and count<=256
rows=[]
for i in range(count):
 index=(next_row-count+i)%256;at=12+index*132
 prefix=struct.unpack_from('<2I12f2hf',b,at)
 v=struct.unpack_from('<14f4hI',b,at+64)
 names=['forward_target_m_s','forward_requested_m_s','yaw_target_rad_s','yaw_requested_rad_s','yaw_rate_rad_s','slow_lean_rad','fast_lean_rad','speed_correction_nm','differential_requested_nm','differential_applied_nm','left_raw_wheel_rad_s','right_raw_wheel_rad_s','left_leg_angle_rad','right_leg_angle_rad']
 row=dict(zip(names,v[:14]));row.update(tick_ms=prefix[0],active=bool(prefix[1]&1),flags=prefix[1],state=list(prefix[2:8]),wheel_voltage_mv=list(prefix[14:16]),joint_voltage_mv=list(v[14:18]),fault_reason=v[18]);rows.append(row)
out=a.input.with_suffix('.decoded.json');out.write_text(json.dumps(dict(layout=1007,row_bytes=132,sequence=sequence,rows=rows),indent=2),encoding='utf-8');print(out)
