from pathlib import Path
import hashlib
import json
root=Path(__file__).resolve().parents[1]
stage=root/'build/lmpc_thermal_20261007'
target=root.parent/'H723_LMPC'
baseline=root/'build/lmpc_perf_20261007'
files=[];hashes={}
for p in sorted(stage.rglob('*')):
    relative=p.relative_to(stage)
    if not p.is_file() or relative.parts[0] in ('build','backups') or '__pycache__' in relative.parts:
        continue
    dest=target/relative
    if dest.exists() and p.read_bytes()==dest.read_bytes():continue
    if dest.exists():
        old=baseline/relative
        assert old.exists() and old.read_bytes()==dest.read_bytes(),f'Intervening change since performance baseline: {dest}'
    key=relative.as_posix();files.append(key)
    hashes[key]=hashlib.sha256(dest.read_bytes()).hexdigest() if dest.exists() else None
(stage/'thermal_overlay.json').write_text(json.dumps({'files':files,'baseline':hashes},indent=2),encoding='utf-8')
installer=(baseline/'scripts/apply_perf_overlay.py').read_text(encoding='utf-8')
installer=installer.replace('perf_overlay.json','thermal_overlay.json').replace('before_perf_','before_thermal_')
installer=installer.replace('zero-output profiling changes','temperature acquisition and protection changes')
installer=installer.replace('performance overlay','temperature overlay')
(stage/'scripts/apply_thermal_overlay.py').write_text(installer,encoding='utf-8')
print(json.dumps({'files':files},indent=2))
