"""Read current 10707 task/core/actuator status and frozen logs. No writes or halt."""
from pathlib import Path
import json
import re
import struct
import subprocess
ROOT=Path(__file__).resolve().parents[1]
OCD='D:/app/clion/clion+stm32/openocd-20230621/OpenOCD-20230621-0.12.0/bin/openocd.exe'
reads=[('remote_before',0x200003c8,8),('tick_before',0x2001246c,1),('obs_before',0x2000a7f8,4),
 ('control',0x20000368,36),('telemetry',0x200003f8,14),('balance',0x2000010c,39),
 ('wheel_output',0x20000458,21),('lmpc',0x20005f84,32),('observation',0x2000a7f8,76),
 ('temperature',0x200001bc,16),('core_clock',0x2000002c,1),
 ('fault_registers',0xe000ed28,6),('debug_status',0xe000edf0,1),('active_exception',0xe000ed04,1),
 ('trace_header',0x20001ef8,3),('startup_header',0x20006004,4),
 ('trace_rows',0x20001f04,256*16),('startup_rows',0x20006014,129*34),
 ('trace_after',0x20001ef8,3),('startup_after',0x20006004,4),
 ('remote_after',0x200003c8,8),('tick_after',0x2001246c,1),('obs_after',0x2000a7f8,4)]
commands=['init']
for name,address,count in reads:
 for offset in range(0,count,8):commands += [f'echo READ_{name}_{offset}',f'echo [read_memory {address+offset*4:#x} 32 {min(8,count-offset)}]']
commands += ['shutdown'];cfg=ROOT/'build/read_current_lmpc_board.cfg';cfg.write_text('\n'.join(commands)+'\n',encoding='ascii')
r=subprocess.run([OCD,'-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid','-c','transport select swd',
 '-c','adapter speed 1000','-f','target/stm32h7x.cfg','-f',str(cfg)],capture_output=True,text=True,check=True)
output=r.stdout+r.stderr;values={}
for name,address,count in reads:
 words=[]
 for offset in range(0,count,8):
  match=re.search(rf'READ_{name}_{offset}\s*\n([^\n]+)',output);assert match,output[-1000:]
  words += [int(s,16) for s in match[1].split()]
 assert len(words)==count;values[name]=words
floats=lambda w:list(struct.unpack('<'+'f'*len(w),struct.pack('<'+'I'*len(w),*w)))
stable=values['trace_header']==values['trace_after'] and values['startup_header']==values['startup_after']
off=values['remote_before'][4]==values['remote_after'][4]==0
def startup_row(w):
 return dict(tick=w[0],active=w[1],phase=w[2],fault=w[3],state=floats(w[4:10]),reference=floats(w[10:16]),
  angle=floats(w[16:18]),angle_reference=floats(w[18:20]),tp=floats(w[20:21])[0],
  local_tp=floats(w[21:23]),sync_tp=floats(w[23:25]),force=floats(w[25:27]),
  length=floats(w[27:29]),wheel_torque=floats(w[29:30])[0],
  voltage_mv=list(struct.unpack('<8h',struct.pack('<4I',*w[30:34]))))
prefix=[];trace=[]
if stable and off:
 head=values['startup_header'];assert head[0]<=128
 w=values['startup_rows'];prefix=[startup_row(w[34*(k+1):34*(k+2)]) for k in range(head[0])]
 next_row,count,seq=values['trace_header'];assert next_row<256 and count<=256
 words=values['trace_rows']
 for k in range(count):
  i=(next_row-count+k)%256;w=words[16*i:16*(i+1)]
  trace.append(dict(tick=w[0],flags=w[1],state=floats(w[2:8]),position_error=floats(w[8:9])[0],
   wheel_requested=floats(w[9:10])[0],wheel_applied=floats(w[10:11])[0],tp=floats(w[11:12])[0],
   voltage_mv=list(struct.unpack('<hh',struct.pack('<I',w[14]))),length=floats(w[15:16])[0]))
c=values['control'];b=values['balance'];d=values['lmpc'];t=values['telemetry'];temp=values['temperature'];obs=values['observation'];clock=values['core_clock'][0]
report=dict(scope='Current 10707 layout; flash fragments matched prior to read; RAM/core only, no halt/reset/write/flash',
 coherent_frozen_logs=stable and off,task=dict(tick_before=values['tick_before'][0],tick_after=values['tick_after'][0],
  updates_before=values['obs_before'][0],updates_after=values['obs_after'][0],period_ms=obs[2],stack_free_min_bytes=obs[3],status=obs[4]),
 core=dict(CFSR=values['fault_registers'][0],HFSR=values['fault_registers'][1],DFSR=values['fault_registers'][2],
  DHCSR=values['debug_status'][0],active_exception=values['active_exception'][0]&511,clock_hz=clock),
 control=dict(enable=c[0],status=c[1],fault_count=c[2],fault_tick=c[3],fault_reason=c[4],
  dual_status=c[5],balance_status=c[6],fault_period_ms=c[10],startup_phase=c[16],
  ch5_raw=c[26],link_ok=c[27],switch_on=c[28],rearm_ready=c[29],remote_status=c[30],feedback_only=c[34]),
 telemetry=dict(enable=t[0],period_ms=t[1],ano_pair=t[3],baud=t[5],sent=t[9],drops=t[10],active_pair=t[13]),
 balance=dict(enable=b[0],status=b[1],wheel_scale=floats(b[2:3])[0],leg_scale=floats(b[3:4])[0],
  state=floats(b[7:13]),reference=floats(b[13:19]),torque=floats(b[20:22]),
  position_locked=b[35],parking_target=floats(b[37:38])[0],parking_error=floats(b[38:39])[0]),
 solver=dict(status=d[0],iterations=d[1],solve_us=d[3]*1e6/clock,max_us=d[4]*1e6/clock,
  solves=d[5],failures=d[6],primal=floats(d[7:8])[0],dual=floats(d[8:9])[0],
  state=floats(d[16:22]),reference=floats(d[22:28]),limits=floats(d[28:30]),reject=d[30],refinements=d[31]),
 temperature=dict(status=temp[0],valid=temp[1],latched=temp[2],temperature_c=floats(temp[12:13])[0],adc_error=temp[10]),
 startup_header=values['startup_header'],startup=prefix,
 startup_fault=startup_row(values['startup_rows'][:34]) if values['startup_header'][2] else None,
 trace_header=values['trace_header'],trace=trace,raw=values)
(ROOT/'diagnostics/current_board_lmpc_read_20261007.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k not in ('startup','trace','raw')},indent=2))
for name,rows in [('startup',prefix),('trace',trace)]:
 print(name,'rows',len(rows))
 for k in sorted(set([0,1,2,len(rows)//4,len(rows)//2,3*len(rows)//4,len(rows)-2,len(rows)-1])):
  if 0<=k<len(rows):print(json.dumps(rows[k]))
