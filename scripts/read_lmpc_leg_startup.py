"""Read the frozen 10706 trace and startup state; never halt/reset/write/flash."""
from pathlib import Path
import json
import re
import struct
import subprocess
ROOT=Path(__file__).resolve().parents[1]
OCD='D:/app/clion/clion+stm32/openocd-20230621/OpenOCD-20230621-0.12.0/bin/openocd.exe'
trace=0x20001ef8;rows_at=trace+12;row_bytes=64
cfg=ROOT/'build/read_lmpc_leg_startup.cfg';cfg.parent.mkdir(exist_ok=True)
commands=['init']
def read(name,address,count):
 for offset in range(0,count,8):
  commands.extend([f'echo READ_{name}_{offset}',f'echo [read_memory {address+offset*4:#x} 32 {min(8,count-offset)}]'])
read('remote_before',0x200003c8,8)
read('trace_before',trace,3)
read('debug',0x20005f84,32)
read('angle_mode',0x20000268,3)
read('angle_current_left',0x20006388,1)
read('angle_current_right',0x200063a0,1)
read('trace',rows_at,256*16)
read('trace_after',trace,3)
read('remote_after',0x200003c8,8)
commands.append('shutdown');cfg.write_text('\n'.join(commands)+'\n',encoding='ascii')
result=subprocess.run([OCD,'-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid',
 '-c','transport select swd','-c','adapter speed 1000','-f','target/stm32h7x.cfg','-f',str(cfg)],capture_output=True,text=True,check=True)
output=result.stdout+result.stderr
def words(name,count):
 values=[]
 for offset in range(0,count,8):
  match=re.search(rf'READ_{name}_{offset}\s*\n([^\n]+)',output);assert match,output[-2000:]
  values += [int(v,16) for v in match[1].split()]
 assert len(values)==count;return values
floats=lambda w:list(struct.unpack('<'+'f'*len(w),struct.pack('<'+'I'*len(w),*w)))
before=words('trace_before',3);after=words('trace_after',3)
rb=words('remote_before',8);ra=words('remote_after',8)
assert before==after and rb[4]==ra[4]==0,'Trace was active; read is not an atomic snapshot'
next_row,count,sequence=before
assert next_row<256 and count<=256
all_words=words('trace',256*16);rows=[]
for k in range(count):
 index=(next_row-count+k)%256;w=all_words[16*index:16*(index+1)]
 state=floats(w[2:8]);extra=floats(w[8:14]);voltage=struct.unpack('<hh',struct.pack('<I',w[14]))
 rows.append(dict(index=index,tick=w[0],flags=w[1],state=state,position_error=extra[0],
  wheel_requested=extra[1],wheel_applied=extra[2],tp=extra[3],voltage_mv=list(voltage),
  length=floats(w[15:16])[0],mean_angle_deg=(state[0]+state[4])*180/3.141592653589793+90,
  body_pitch_deg=-state[4]*180/3.141592653589793))
w=words('debug',32);mode=words('angle_mode',3)
report=dict(version=10706,scope='CH5 off, frozen trace; read only',header=before,
 remote=rb,solver=dict(status=w[0],iterations=w[1],solve_us=w[3]/400,max_us=w[4]/400,
 solves=w[5],failures=w[6],state=floats(w[16:22]),reference=floats(w[22:28])),
 angle_mode=mode[0],angle_target_rad=floats(mode[1:2])[0],angle_reference_rate=floats(mode[2:3])[0],
 current_leg_angles_rad=[floats(words(n,1))[0] for n in ['angle_current_left','angle_current_right']],rows=rows)
(ROOT/'diagnostics/lmpc_leg_startup_trace_20261007.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
print('Rows:',count)
for k in sorted(set([0,1,2,5,10,count//4,count//2,3*count//4,count-3,count-2,count-1])):
 if 0<=k<count:print(json.dumps(rows[k]))
