from pathlib import Path
import json
import re
import shutil
import struct
import subprocess
ROOT=Path(__file__).resolve().parents[1]
source=ROOT.parent/'H723_LMPC';stage=ROOT/'build/lmpc_wheel_20261007'
ocd='D:/app/clion/clion+stm32/openocd-20230621/OpenOCD-20230621-0.12.0/bin/openocd.exe'
reads=[('debug',0x20005f84,32),('lo_first',0x200077f4,2),('hi_first',0x200068e8,2)]
commands=['init']
for name,address,count in reads:
 for offset in range(0,count,8):
  commands += [f'echo READ_{name}_{offset}',f'echo [read_memory {address+4*offset:#x} 32 {min(8,count-offset)}]']
commands+=['shutdown']
result=subprocess.run([ocd,'-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid',
 '-c','transport select swd','-c','adapter speed 1000','-f','target/stm32h7x.cfg',
 '-c','; '.join(commands)],capture_output=True,text=True,check=True)
output=result.stdout+result.stderr;words={}
for name,address,count in reads:
 values=[]
 for offset in range(0,count,8):
  match=re.search(rf'READ_{name}_{offset}\s*\n([^\n]+)',output);assert match,output
  values += [int(s,16) for s in match[1].split()]
 assert len(values)==count;words[name]=values
flt=lambda v:list(struct.unpack('<'+'f'*len(v),struct.pack('<'+'I'*len(v),*v)))
w=words['debug'];case=dict(version=10705,status=w[0],iterations=w[1],table_index=w[2],
 cycles=w[3],solve_us=w[3]/400,max_us=w[4]/400,solves=w[5],failures=w[6],
 primal=flt(w[7:8])[0],dual=flt(w[8:9])[0],rho_mode=w[11],budget_cycles=w[12],
 polished=w[13],rho=flt(w[14:15])[0],length=flt(w[15:16])[0],
 state=flt(w[16:22]),reference=flt(w[22:28]),limits=flt(w[28:30]),
 reject=w[30],refinements=w[31],lo_first=flt(words['lo_first']),hi_first=flt(words['hi_first']),raw=words,
 scope='Read-only RAM; N10 flash entry bytes previously matched 10705 AXF; no halt/reset/write/flash')
case['previous_nm']=[(a+b)*.5*limit for a,b,limit in zip(case['lo_first'],case['hi_first'],[.03,.02])]
(ROOT/'diagnostics/lmpc_wheel_failure_20261007.json').write_text(json.dumps(case,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in case.items() if k!='raw'},indent=2))
assert not stage.exists(),stage
shutil.copytree(source,stage,ignore=shutil.ignore_patterns('build','backups','__pycache__'))
(stage/'build').mkdir()
shutil.copy2(ROOT/'diagnostics/lmpc_wheel_failure_20261007.json',stage/'tests/data/n10_wheel_failure_20261007.json')
p=stage/'Control/LMPC/Src/lmpc.c';s=p.read_text(encoding='utf-8')
s=s.replace('static float hot_factor[LMPC_VARS][LMPC_VARS];',
 'static float hot_factor[LMPC_VARS][LMPC_VARS];\n/* Refinement repeatedly reads H; cache it in DTCM with the inverse. */\nstatic float hot_h[LMPC_VARS][LMPC_VARS];')
s=s.replace('static int polish(const LmpcTable *t,float rho,','static int polish(float rho,')
s=s.replace('t->H[i][j]','hot_h[i][j]').replace('polish(t,rho,','polish(rho,')
s=s.replace('memcpy(hot_factor,t->J,sizeof(hot_factor));',
 'memcpy(hot_h,t->H,sizeof(hot_h));\n        memcpy(hot_factor,t->J,sizeof(hot_factor));')
p.write_text(s,encoding='utf-8')
p=stage/'App/Tasks/Inc/wheel_leg_config.h';s=p.read_text(encoding='utf-8')
s=s.replace('#define BALANCE_DEFAULT_WHEEL_SCALE              0.7f',
 '/* LMPC may use the user-specified 0.030 Nm per-wheel budget. */\n#define BALANCE_DEFAULT_WHEEL_SCALE              1.0f')
p.write_text(s,encoding='utf-8')
p=stage/'App/Services/Src/robot_telemetry_service.c';s=p.read_text(encoding='utf-8').replace('10705','10706');p.write_text(s,encoding='utf-8')
p=stage/'tests/verify_integration.py';s=p.read_text(encoding='utf-8').replace('<=.021001f','<=.030001f')
s=s.replace('assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_balance.position_locked);',
 '''assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_balance.position_locked);
 assert(g_balance.wheel_scale==1.0f);
 /* Real task must reach the restored budget, and forward the signed voltage
    through the real CAN owner without raising the hard 0.030 Nm ceiling. */
 imu.roll_deg=-4.0f;unsigned reached_full=0;
 for(unsigned k=0;k<20;++k) {
  step();assert(g_robot_control.status==1 && g_lmpc.status==1);
  assert(fabsf(g_wheel_output.applied_torque_nm)<=.030001f);
  assert(fabsf(g_balance.torque_nm[1])<=.004001f);
  for(unsigned j=0;j<4;++j) assert(fabsf(g_dual_hold.joint_torque_nm[j])<=.680001f);
  if(fabsf(g_wheel_output.applied_torque_nm)>.0211f)reached_full=1;
  for(unsigned side=0;side<2;++side) {
   float expected=1000*g_wheel_output.applied_torque_nm/g_wheel_output.torque_ratio_nm_per_v[side]*.7f;
   assert(fabsf(g_wheel_output.torque_voltage_mv[side]-expected)<.01f);
  }
 }
 assert(reached_full);imu.roll_deg=0;step();''')
p.write_text(s,encoding='utf-8')
# Latest captured failure is independent of the previous release's case.
s=(stage/'tests/verify_lmpc_n10.py').read_text(encoding='utf-8')
s=s.replace('n10_replay.dll','wheel_replay.dll').replace('n20_timeout_20261007.json','n10_wheel_failure_20261007.json')
s=s.replace('lmpc_n10_replay_validation.json','lmpc_wheel_replay_validation.json')
(stage/'tests/verify_wheel_replay.py').write_text(s,encoding='utf-8')
print('Prepared wheel-budget restoration and repeated-H DTCM cache; board/project unchanged.')
