"""Deploy the tested 10803 actuator alignment, preserving old firmware and data."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import time
from deploy_lmpc_parking_10801 import snapshot,guard,verify_image,safe_read,layout,FIELDS,OCD,hex_extent
ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'build/lmpc_trace_10803'
TARGET=ROOT.parent/'H723_LMPC'
STAMP=time.strftime('%Y%m%d_%H%M%S')
PREFIX=ROOT/'diagnostics'/('h723_lmpc_trace_10803_'+STAMP)
BACKUP=ROOT/'backups'/('before_lmpc_10803_'+STAMP)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
FILES=['App/Tasks/Inc/wheel_leg_config.h','App/Tasks/Inc/wheel_leg_task.h',
 'App/Tasks/Src/wheel_leg_task.c','App/Services/Src/robot_telemetry_service.c',
 'tests/verify_long_trace.py',
 'docs/lmpc_trace_10803_20261008.txt','README.md']
for folder,name in [('keil','H723_LMPC'),('read-only','H723_READ_ONLY'),('lmpc-shadow','H723_LMPC_SHADOW')]:
 FILES += [f'build/{folder}/{name}{ext}' for ext in ['.hex','.axf','.htm']]
 FILES += [f'build/{folder}/build.log',f'build/{folder}/listings/{name}.map']
for name in FILES:
 if not (STAGE/name).is_file():raise RuntimeError(f'Missing candidate artifact: {name}')
# Preserve old symbols before installing a new canonical AXF.
for name in FILES:
 source=TARGET/name
 if source.is_file():
  saved=BACKUP/name;saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,saved)
  assert sha(saved)==sha(source)
OLD=BACKUP/'build/keil/H723_LMPC.axf'
NEW=STAGE/'build/keil/H723_LMPC.axf'
hex_extent(NEW.with_suffix('.hex'))
verify_image(OLD.with_suffix('.hex'))
before=snapshot(OLD,FIELDS);guard(before)
names=['g_balance_trace','g_lmpc_startup_trace','g_lmpc','g_lmpc_control','g_balance','g_dual_hold','g_robot_control','g_parking_learning_debug']
addresses,sizes=layout(OLD,names,names)
raw=safe_read([(n,addresses[n],sizes[n]//4) for n in names])
for name,data in raw.items():PREFIX.with_suffix('.'+name+'.bin').write_bytes(data)
oldlog=PREFIX.with_suffix('.preserved_flash.bin')
oldlog.write_bytes(safe_read([('log',0x080c0000,0x40000//4)])['log'])
after_backup=snapshot(OLD,FIELDS);guard(after_backup)
headers=safe_read([('ring',addresses['g_balance_trace'],3),('startup',addresses['g_lmpc_startup_trace'],4)])
assert raw['g_balance_trace'][:12]==headers['ring'] and raw['g_lmpc_startup_trace'][:16]==headers['startup']
rows=[]
for name in FILES:
 source=STAGE/name;dest=TARGET/name
 old_sha=sha(dest) if dest.is_file() else None
 if old_sha is not None:assert sha(BACKUP/name)==old_sha,'Project changed during preparation'
 dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
 assert sha(dest)==sha(source)
 rows.append(dict(path=name,before=old_sha,after=sha(dest)))
report=dict(firmware_id=10803,before=before,backup=str(BACKUP),candidate=str(STAGE),installed=rows,
 preserved_flash=str(oldlog),preserved_flash_sha256=sha(oldlog),motors_started_by_agent=False,
 verified=False,physical_stability_verified=False)
PREFIX.with_suffix('.prepared.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
guard(snapshot(OLD,FIELDS))
cfg=PREFIX.with_suffix('.program.cfg')
cfg.write_text('init\nreset halt\nprogram {'+NEW.with_suffix('.hex').as_posix()+'} verify\nreset run\nshutdown\n',encoding='ascii')
result=subprocess.run([OCD,'-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid',
 '-c','transport select swd','-c','adapter speed 1000','-f','target/stm32h7x.cfg','-f',str(cfg)],
 capture_output=True,text=True,timeout=90)
output=result.stdout+result.stderr;PREFIX.with_suffix('.program.log').write_text(output,encoding='utf-8')
if result.returncode or 'Verified OK' not in output:raise RuntimeError(output[-3000:])
print('Program verified; checking all firmware bytes and reserved Flash.',flush=True)
verify_image(NEW.with_suffix('.hex'))
log_after=PREFIX.with_suffix('.after_flash.bin')
log_after.write_bytes(safe_read([('log',0x080c0000,0x40000//4)])['log']);assert sha(log_after)==sha(oldlog)
fields=FIELDS+['g_balance.parking_lean_gain_rad_per_m','g_balance.back_emf_ratio',
 'g_dual_hold.sync_ratio','g_dual_hold.sync_limit_nm','g_dual_hold.sync_speed_kp','g_dual_hold.sync_rate_nm_s',
 'g_dual_hold.sync_recovery_priority_enable','g_dual_hold.torque_ratio_nm_per_v[0]',
 'g_dual_hold.torque_ratio_nm_per_v[1]','g_dual_hold.torque_ratio_nm_per_v[2]',
 'g_dual_hold.torque_ratio_nm_per_v[3]','SystemCoreClock','g_wheel_leg.period_ms','g_wheel_leg.stack_free_min_bytes']
addresses,_=layout(NEW,fields)
values=safe_read([(str(i),addresses[n],1) for i,n in enumerate(fields)])
import struct
after=snapshot(NEW,FIELDS)
try:
 guard(after);outputs_off=True
except RuntimeError:
 outputs_off=False
expected={'g_balance.parking_lean_gain_rad_per_m':.05,'g_balance.back_emf_ratio':.3,
 'g_dual_hold.sync_ratio':.65,'g_dual_hold.sync_limit_nm':.3,'g_dual_hold.sync_speed_kp':.06,
 'g_dual_hold.sync_rate_nm_s':.4,**{f'g_dual_hold.torque_ratio_nm_per_v[{i}]':.04 for i in range(4)}}
for i,name in enumerate(fields):
 if name in expected:
  value=struct.unpack('<f',values[str(i)])[0];assert abs(value-expected[name])<1e-6,(name,value)
 elif name not in FIELDS:value=struct.unpack('<I',values[str(i)])[0]
 else:continue
 after[name]=value
assert after['g_dual_hold.sync_recovery_priority_enable']==1 and after['g_robot_control.feedback_only']==0
report.update(verified=True,full_firmware_readback_matched=True,reserved_flash_preserved=True,after=after,post_flash_off_check_passed=outputs_off)
PREFIX.with_suffix('.flashed.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
(ROOT/'build/lmpc_current_deployment.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(dict(verified=True,firmware_id=10803,after=after,report=str(PREFIX.with_suffix('.flashed.json'))),indent=2))
