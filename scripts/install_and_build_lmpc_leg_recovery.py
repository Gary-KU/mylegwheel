from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
stage=ROOT/'build/lmpc_leg_recovery_20261007';target=ROOT.parent/'H723_LMPC'
subprocess.run([sys.executable,str(stage/'scripts/apply_leg_recovery_overlay.py')],check=True)
def build(profile):
 result=subprocess.run([sys.executable,str(target/'scripts/build_keil.py'),'--profile',profile],capture_output=True,text=True)
 assert result.returncode==0,result.stdout+result.stderr;return result.stdout.strip()
with ThreadPoolExecutor(max_workers=3) as pool:
 for result in pool.map(build,['default','read-only','lmpc-shadow']):print(result)
for suffix in ['hex','axf']:
 source=target/f'build/keil/H723_LMPC.{suffix}';alias=target/f'build/keil/H723_LMPC_LEG_RECOVERY.{suffix}'
 assert source.stat().st_size>0;shutil.copy2(source,alias);assert source.read_bytes()==alias.read_bytes()
manifest=json.loads((stage/'leg_recovery_overlay.json').read_text())
for name in manifest['files']:assert (target/name).read_bytes()==(stage/name).read_bytes(),name
report=dict(version=10707,files=manifest['files'],wheel_limit_nm=.03,virtual_tp_limit_nm=.02,joint_limit_nm=.68,
 region_theta_deg=15,region_phi_deg=15,region_relative_leg_deg=20,
 first_640ms_captured=True,default_ano_pair=27,board_flashed_by_agent=False,
 physical_leg_folding_resolved=False,actual_new_target_timing_measured=False,
 old_live_record=json.loads((stage/'tests/data/leg_rearward_10706.json').read_text()),
 firmware_sha256=hashlib.sha256((target/'build/keil/H723_LMPC_LEG_RECOVERY.hex').read_bytes()).hexdigest())
(ROOT/'diagnostics/lmpc_leg_recovery_release_20261007.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Verified installation/aliases; no device flash, reset, write or motor enable.')
print('Firmware:',target/'build/keil/H723_LMPC_LEG_RECOVERY.hex')
