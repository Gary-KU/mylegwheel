"""Install the already validated N10 overlay, back up old files and build all profiles."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
stage=ROOT/'build/lmpc_n10_20261007'
target=ROOT.parent/'H723_LMPC'
subprocess.run([sys.executable,str(stage/'scripts/apply_n10_overlay.py')],check=True)
def build(profile):
 result=subprocess.run([sys.executable,str(target/'scripts/build_keil.py'),'--profile',profile],
  capture_output=True,text=True)
 assert result.returncode==0,result.stdout+result.stderr
 return result.stdout.strip()
with ThreadPoolExecutor(max_workers=3) as pool:
 for result in pool.map(build,['default','read-only','lmpc-shadow']):print(result)
for suffix in ['hex','axf']:
 original=target/f'build/keil/H723_LMPC.{suffix}'
 alias=target/f'build/keil/H723_LMPC_N10.{suffix}'
 assert original.stat().st_size>0
 shutil.copy2(original,alias)
 assert alias.read_bytes()==original.read_bytes()
records=json.loads((stage/'n10_overlay.json').read_text())
for name in records['files']:
 assert (target/name).read_bytes()==(stage/name).read_bytes(),name
report=dict(version=10705,prediction_steps=10,control_steps=10,Ts=.005,
 target=str(target),files=records['files'],board_flashed_by_agent=False,
 actual_target_timing_measured=False,
 firmware_sha256=hashlib.sha256((target/'build/keil/H723_LMPC_N10.hex').read_bytes()).hexdigest(),
 host_validation=[json.loads((stage/'build'/name).read_text()) for name in
  ['lmpc_host_validation.json','lmpc_startup_validation.json','lmpc_n10_replay_validation.json']])
(ROOT/'diagnostics/lmpc_n10_release_20261007.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Verified installation and aliases. No board flash/reset/enable operation performed.')
print('Firmware:',target/'build/keil/H723_LMPC_N10.hex')
