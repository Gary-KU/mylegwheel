from pathlib import Path
import hashlib
import json
root=Path(__file__).resolve().parents[1]
stage=root/'build/lmpc_startup_fix_20261007';target=root.parent/'H723_LMPC'
baseline=root/'build/lmpc_temperature_fix_20261007'
p=stage/'README.md';text=p.read_text(encoding='utf-8')
text=text.split('\n## 2026-10-07 上挡超时修正（10704）\n')[0]
text=text.replace('3 ms','4 ms')
text=text.replace('影子固件编号 10703/求解耗时','影子固件编号 10704/求解耗时')
text=text.replace('影子编号 10703，默认','影子编号 10704，默认')
text=text.replace('通常跑满 160 次迭代','修正版可能提前收敛，实际迭代/状态以读数为准')
text+='\n## 2026-10-07 上挡超时修正（10704）\n\n上挡后故障已只读确认为 LMPC 3 ms 超时。新增三档数值罚参数、约束工作集修正和完整 KKT 检查，保持模型、权重、力矩/变化率限制及失败停机。完整求解截止改为 4 ms，5 ms 控制周期保留。48 组启动工况全部通过独立完整轨迹对照；捕获工况在主机 16 次迭代后收敛。旧版板上效率数据是历史记录，本修正版仍待实机复测。用户自行烧录 H723_LMPC_STARTUP_FIX.hex，完整/影子编号均为 10704。详见 [启动修正与验证](docs/lmpc_startup_fix.md)。\n'
p.write_text(text,encoding='utf-8')
p=stage/'docs/cpu_temperature.md';text=p.read_text(encoding='utf-8')
text=text.replace('温度修正版：完整控制编号 **20261007**，影子编号 **10703**','当前启动修正版：完整控制/影子编号均为 **10704**')
if '后续 LMPC 上挡超时修正保持本温度保护' not in text:
    text+='\n后续 LMPC 上挡超时修正保持本温度保护，详见 [启动修正](lmpc_startup_fix.md)。\n'
p.write_text(text,encoding='utf-8')
files=[];hashes={}
intended=['Control/LMPC/Inc/lmpc.h','Control/LMPC/Inc/lmpc_tables.h',
 'Control/LMPC/Src/lmpc.c','Control/LMPC/Src/lmpc_tables.c','scripts/generate_lmpc.py',
 'models/lmpc_tables.npz','App/Tasks/Src/wheel_leg_task.c','App/Services/Src/robot_telemetry_service.c',
 'tests/verify_lmpc.py','tests/verify_lmpc_startup.py','tests/verify_integration.py',
 'tests/integration_fixture.c.in','tests/verify_shadow.py','tests/data/startup_failure_20261007.json',
 'README.md','docs/cpu_temperature.md','docs/lmpc_startup_fix.md']
for p in [stage/n for n in sorted(intended)]:
    relative=p.relative_to(stage)
    if not p.is_file() or relative.parts[0] in ('build','backups') or '__pycache__' in relative.parts:
        continue
    dest=target/relative
    if dest.exists() and p.read_bytes()==dest.read_bytes():continue
    if dest.exists():
        old=baseline/relative
        assert old.exists() and old.read_bytes()==dest.read_bytes(),f'Intervening change: {dest}'
    key=relative.as_posix();files.append(key)
    hashes[key]=hashlib.sha256(dest.read_bytes()).hexdigest() if dest.exists() else None
(stage/'startup_fix_overlay.json').write_text(json.dumps({'files':files,'baseline':hashes},indent=2),encoding='utf-8')
installer=(baseline/'scripts/apply_temperature_fix_overlay.py').read_text(encoding='utf-8')
installer=installer.replace('temperature_fix_overlay.json','startup_fix_overlay.json').replace('before_temperature_fix_','before_startup_fix_')
installer=installer.replace('H723 12-bit temperature calibration fix','reviewed LMPC startup convergence fix')
installer=installer.replace('temperature calibration fix','LMPC startup fix')
(stage/'scripts/apply_startup_fix_overlay.py').write_text(installer,encoding='utf-8')
print(json.dumps({'files':files},indent=2))
