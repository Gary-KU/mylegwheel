"""Save the flashed 10708 release in H723_LMPC and archive its Shadow entry."""
from pathlib import Path
from datetime import datetime
import hashlib
import json
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'build/lmpc_backup_20261007'
TARGET = (ROOT.parent / 'H723_LMPC').resolve()
assert TARGET == Path('D:/CODE/FOC_WHEEL/foc-wheel-legged-robot-master/damiao/H723_LMPC').resolve()
manifest = json.loads((STAGE / 'backup_overlay.json').read_text())
flash = json.loads((ROOT / 'diagnostics/flash_10708_record_20261007.json').read_text(encoding='utf-8-sig'))

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

hex_path = STAGE / 'build/keil/H723_LMPC.hex'
assert digest(hex_path).upper() == flash['sha256']
for relative in manifest['files']:
    assert (STAGE / relative).is_file(), relative
    destination = (TARGET / relative).resolve()
    assert destination.is_relative_to(TARGET)
    expected = manifest['baseline'][relative]
    if expected is None:
        assert not destination.exists(), f'New file already exists: {destination}'
    else:
        assert destination.is_file() and digest(destination) == expected, f'Intervening edit: {destination}'
shadow = TARGET / 'MDK-ARM/H723_LMPC_Shadow.uvprojx'
assert shadow.is_file()
assert {p.name for p in (TARGET / 'MDK-ARM').glob('*.uvprojx')} == {
    'H723_LMPC.uvprojx', 'H723_LMPC_Shadow.uvprojx'}
print('Preflight passed: reviewed 10708 sources, exact flashed HEX, two identified Keil entries.', flush=True)
if '--apply' not in sys.argv:
    sys.exit(0)

backup = TARGET / 'backups' / ('before_10708_single_project_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
assert backup.resolve().is_relative_to(TARGET)
for relative in manifest['files']:
    old = TARGET / relative
    if old.exists():
        saved = backup / relative
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(old, saved)
saved_shadow = backup / 'MDK-ARM' / shadow.name
saved_shadow.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(shadow, saved_shadow)
for old in (TARGET / 'build/keil').glob('*'):
    if old.is_file() and old.suffix.lower() in ('.hex', '.axf'):
        saved = backup / 'build/keil' / old.name
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(old, saved)
for relative in manifest['files']:
    destination = TARGET / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(STAGE / relative, destination)
for script in ('check_project.py', 'build_keil.py'):
    subprocess.run([sys.executable, str(TARGET / 'scripts' / script)], check=True)
assert digest(TARGET / 'build/keil/H723_LMPC.hex') == digest(hex_path), 'Rebuilt flash content differs from flashed 10708!'
for suffix in ('hex', 'axf'):
    shutil.copy2(TARGET / f'build/keil/H723_LMPC.{suffix}', TARGET / f'build/keil/H723_LMPC_LQR_BACKUP.{suffix}')
for relative in manifest['files']:
    assert (TARGET / relative).read_bytes() == (STAGE / relative).read_bytes(), relative

# Remove only the reviewed Shadow entry after the main rebuild has succeeded;
# its byte-identical recoverable copy lives outside the active MDK-ARM directory.
assert digest(shadow) == digest(saved_shadow)
shadow.unlink()
assert [p.name for p in (TARGET / 'MDK-ARM').glob('*.uvprojx')] == ['H723_LMPC.uvprojx']
readme = TARGET / 'README.md'
text = readme.read_text(encoding='utf-8')
text = text.replace('打开 `MDK-ARM/H723_LMPC_Shadow.uvprojx`，或执行 `python scripts/build_keil.py --profile lmpc-shadow`。',
                    'MDK-ARM 仅保留完整控制工程 `H723_LMPC.uvprojx`。关闭电机的影子效率测试仍可执行 `python scripts/build_keil.py --profile lmpc-shadow`；旧 Shadow 工程入口已归档到 backups。')
text = text.replace('编号10708，用户自行烧录；尚未实机验证保底切换与停车效果。',
                    '编号10708，已于2026-10-07烧录并通过Flash校验，关闭CH5时确认控制未使能；尚未实机验证保底切换与停车效果。')
readme.write_text(text, encoding='utf-8')
doc = TARGET / 'docs/lmpc_backup.md'
text = doc.read_text(encoding='utf-8').replace('未烧录或开启电机，未测本版板上耗时或真实停车稳定性。',
    '已于2026-10-07烧录10708并通过Flash校验，确认CH5关闭时控制未使能；未开启电机，未测本版求解耗时或真实停车稳定性。')
text = text.replace('用户自行烧录 build/keil/H723_LMPC_LQR_BACKUP.hex，配合同名AXF；编号10708。',
    '唯一Keil入口为 MDK-ARM/H723_LMPC.uvprojx。烧录 build/keil/H723_LMPC.hex，配合同名AXF；编号10708，Flash内容与2026-10-07已烧录固件完全一致。')
doc.write_text(text, encoding='utf-8')
report = dict(version=10708, project=str(TARGET / 'MDK-ARM/H723_LMPC.uvprojx'),
              backup=str(backup), hex_sha256=digest(TARGET / 'build/keil/H723_LMPC.hex'),
              identical_to_flashed_hex=True, build_passed=True, active_keil_projects=1,
              hardware_access_during_save=False)
(ROOT / 'diagnostics/saved_10708_single_project.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report, indent=2))
