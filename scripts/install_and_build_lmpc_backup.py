from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1];stage=ROOT/'build/lmpc_backup_20261007';target=ROOT.parent/'H723_LMPC'
subprocess.run([sys.executable,str(stage/'scripts/apply_backup_overlay.py')],check=True)
def build(profile):
 result=subprocess.run([sys.executable,str(target/'scripts/build_keil.py'),'--profile',profile],capture_output=True,text=True)
 assert result.returncode==0,result.stdout+result.stderr;return result.stdout.strip()
with ThreadPoolExecutor(max_workers=3) as pool:
 for result in pool.map(build,['default','read-only','lmpc-shadow']):print(result)
for suffix in ['hex','axf']:
 source=target/f'build/keil/H723_LMPC.{suffix}';alias=target/f'build/keil/H723_LMPC_LQR_BACKUP.{suffix}'
 assert source.stat().st_size>0;shutil.copy2(source,alias);assert source.read_bytes()==alias.read_bytes()
manifest=json.loads((stage/'backup_overlay.json').read_text())
for name in manifest['files']:assert (target/name).read_bytes()==(stage/name).read_bytes(),name
report=dict(version=10708,files=manifest['files'],qp_states_using_backup=[3,4],invalid_inputs_still_stop=True,
 probe_period_ms=20,recovery_good_probes=10,return_blend_ms=100,
 wheel_limit_nm=.03,virtual_tp_limit_nm=.02,joint_limit_nm=.68,default_ano_pair=28,
 board_flashed_by_agent=False,actual_new_target_timing_measured=False,physical_stability_verified=False,
 model_validation=json.loads((stage/'build/backup_model_validation.json').read_text()),
 firmware_sha256=hashlib.sha256((target/'build/keil/H723_LMPC_LQR_BACKUP.hex').read_bytes()).hexdigest())
(ROOT/'diagnostics/lmpc_backup_release_20261007.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Verified installed source and aliases; no flash, reset, memory write or motor enable performed.')
print('Firmware:',target/'build/keil/H723_LMPC_LQR_BACKUP.hex')
