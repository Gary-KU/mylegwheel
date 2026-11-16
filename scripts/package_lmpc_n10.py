from pathlib import Path
import hashlib
import json
import shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
stage=ROOT/'build/lmpc_n10_20261007';target=ROOT.parent/'H723_LMPC'
baseline=ROOT/'build/lmpc_startup_fix_20261007'
old=np.load(target/'models/lmpc_tables.npz');new=np.load(stage/'models/lmpc_tables.npz')
for key in ['A','B','P','lengths','limit','step']:
    assert np.array_equal(old[key],new[key]),f'Unexpected physical model change: {key}'
assert new['H'].shape==(11,20,20) and new['C'].shape==(11,40,20)
p=stage/'README.md';s=p.read_text(encoding='utf-8')
s=s.replace('预测和控制时域均 20 步，40 个决策变量','预测和控制时域均 10 步（50 ms），20 个决策变量、40 个约束行')
s=s.replace('固件编号 20261007 | 上次 LMPC','固件编号 10705 | 上次 LMPC')
s=s.replace('22 组显示影子固件编号 10704','22 组显示影子固件编号 10705')
s=s.replace('影子编号 10704，默认','影子编号 10705，默认')
s=s.replace('6.4 KB DTCM 热矩阵缓存','1.6 KB DTCM 热矩阵缓存（当前 10 步版）')
s=s.replace('这是一份完成编译和离线验证的移植版，已完成 400 MHz 影子效率测量：',
 '当前 10705 版尚待板上耗时与动力复测。此前 20 步旧版的 400 MHz 影子效率历史记录：')
s=s.replace('## 2026-10-07 求解效率修改','## 2026-10-07 求解效率修改（历史 20 步版）')
s=s.split('\n## 2026-10-07 10 步修正（10705）\n')[0]
s+='\n## 2026-10-07 10 步修正（10705）\n\n再次只读确认 10704 求解超时：16 次 ADMM 后约 4.1269 ms，超过 4 ms 截止。当前缩短为 10/10 步，采样仍 5 ms、预测 50 ms；决策变量 40→20，约束行 80→40。保留物理模型、Q/R、DARE 终端权重、力矩与变化率限制、失败停机和过温保护。完整固件使用 `build/keil/H723_LMPC_N10.hex`，编号 10705，由用户自行烧录。已通过 48 组旧启动工况、24 组最新故障/邻近工况、64 次暖启动序列及 8 组线性闭环验证；未烧录，未测新版本板上耗时。详见 [10 步版本说明](docs/lmpc_n10.md)。\n'
p.write_text(s,encoding='utf-8')
p=stage/'docs/cpu_temperature.md';s=p.read_text(encoding='utf-8').replace('当前启动修正版：完整控制/影子编号均为 **10704**','当前 10 步版：完整控制/影子编号均为 **10705**')
p.write_text(s,encoding='utf-8')
p=stage/'docs/lmpc_startup_fix.md';s=p.read_text(encoding='utf-8')
if not s.startswith('> 历史版本'):
    s='> 历史版本 10704；当前 10705 已缩短预测时域，见 [10 步版本](lmpc_n10.md)。本文的 20 步参数和验证结果仅适用于 10704。\n\n'+s
p.write_text(s,encoding='utf-8')
live=json.loads((stage/'build/lmpc_n10_replay_validation.json').read_text())
exact=next(r for r in live['cases'] if r['scale']==1 and r['length']==live['live_timeout']['length'] and any(r['applied_nm']))
doc=f'''# LMPC 10 步版（10705）

## 故障证据

10704 在运行期间触发 ROBOT_FAULT_LMPC（9），状态 4：16 次 ADMM、约束修正到第 16 次时超时，DWT 为 1,650,760 周期，400 MHz 下约 4.1269 ms，超过 4 ms 求解截止。记录累计 454 次求解、6 次失败，最大约 4.273 ms。不是用尽 160 次迭代。只读读取未暂停、复位、改写或烧录主控。

当时腿长均值 78.254 mm；theta 0.19139 rad，dtheta -0.97745 rad/s，phi 0.11708 rad，dphi 0.30360 rad/s。输入限幅轮 ±0.021、Tp ±0.00312 N·m；上次实际请求约 +0.021/-0.00308 N·m。完整状态、参考、首步变化率上下界保存在 `tests/data/n20_timeout_20261007.json`；这次输入直接来自调试记录。

## 当前修改

预测/控制时域由 20/20 改成 10/10；采样保持 5 ms，预测 100→50 ms，变量 40→20，约束行 80→40，单次稠密矩阵乘法乘加数量为原来的 1/4。工作集 Cholesky 最大维度也减半；这只说明计算规模减少，不能推导实际板上耗时。模型表与工作空间均按维数生成，不只截短循环。

物理 A/B、DARE P、Q/R、MVRate、腿长表、Nm 限幅和每步变化率逐项保留。终端 Q+P 放在第 10 步；缩短时域会改变约束下的控制结果，需要动力复测。中档 rho 选择阈值从 gradient_max/rho>50 调整为 >25，解决缩短时域后发现的一组启动迭代用尽；三档 rho、完整 KKT 检查和 160 次迭代上限保留。暖启动移位、约束索引、矩阵/链工作区均使用 LMPC_VARS/LMPC_CONSTRAINTS，并有编译期四路展开维数检查。

完整求解截止仍 4 ms，任务周期 5 ms；失败拒绝输出、补发零、需关闭 CH5 重新解锁。所有支撑/同步、ADC3 温度保护以及影子/只读零输出路径保留。软件 Nm 请求由电压模式 F103 实现，不是电流反馈实测力矩。

## 验证与限制

最新故障及邻近 24 组 QP 全部收敛，并与独立双精度求解器对照完整 20 维轨迹、KKT、幅值和非零已施加输入的变化率限制。精确故障输入在主机 C 中 {exact['iterations']} 次 ADMM、{exact['refinements']} 次最终工作集修正后收敛，最大完整轨迹差 {exact['horizon_error_nm']:.3g} N·m。另通过 48 组旧启动扫描、64 次暖启动/腿长表切换序列、33 组独立 QP 和 8 组小扰动线性闭环，以及任务/CAN/遥控故障、截止失败、温度锁止、匿名帧和影子零输出测试。

主机测试证明计算正确性与所测线性工况，不证明实机稳定性或板上耗时。此版未由代理烧录。

## 用户自行烧录与效率查看

- 完整输出：`build/keil/H723_LMPC_N10.hex`，AXF 同名；与当前 `H723_LMPC.hex/.axf` 内容一致。仅更新 H723，不更换 F103。
- 零输出影子：`build/lmpc-shadow/H723_LMPC_SHADOW.hex`；只读：`build/read-only/H723_READ_ONLY.hex`。
- 固件编号 10705；PA10、115200、50 Hz，匿名前两个 S32 通道比例都为 0.01。
- Watch 将 `g_robot_telemetry.ch5_select=0`，`ano_pair=22` 显示编号/求解 µs；23 显示求解 µs/ADMM 迭代数；24 显示状态/输入模式。4=超时，3=迭代用尽，1=收敛。ADMM 次数不含工作集修正。
- DWT `g_lmpc.cycles/max_cycles` 除以 400 得 µs。`polish_reject_reason=1` 是约束修正遇截止，`polish_refinements` 看修正次数；`g_wheel_leg.period_ms/stack_free_min_bytes` 检查周期/栈余量。

旧版 `H723_LMPC_STARTUP_FIX.hex` 是 10704 历史文件，勿用它验证本版。
'''
(stage/'docs/lmpc_n10.md').write_text(doc,encoding='utf-8')
intended=['Control/LMPC/Inc/lmpc.h','Control/LMPC/Inc/lmpc_tables.h','Control/LMPC/Src/lmpc.c',
 'Control/LMPC/Src/lmpc_tables.c','scripts/generate_lmpc.py','models/lmpc_tables.npz','models/lmpc_parameters.json',
 'App/Tasks/Src/wheel_leg_task.c','App/Services/Src/robot_telemetry_service.c',
 'tests/verify_lmpc.py','tests/verify_lmpc_startup.py','tests/verify_lmpc_n10.py','tests/data/n20_timeout_20261007.json',
 'README.md','docs/cpu_temperature.md','docs/lmpc_startup_fix.md','docs/lmpc_n10.md']
files=[];hashes={}
for relative in intended:
 p=stage/relative;dest=target/relative
 assert p.is_file(),p
 if dest.exists() and p.read_bytes()==dest.read_bytes():continue
 if dest.exists():
  old=baseline/relative
  assert old.exists() and old.read_bytes()==dest.read_bytes(),f'Intervening edit: {dest}'
 files.append(relative);hashes[relative]=hashlib.sha256(dest.read_bytes()).hexdigest() if dest.exists() else None
(stage/'n10_overlay.json').write_text(json.dumps(dict(files=files,baseline=hashes),indent=2),encoding='utf-8')
installer=(baseline/'scripts/apply_startup_fix_overlay.py').read_text(encoding='utf-8')
installer=installer.replace('startup_fix_overlay.json','n10_overlay.json').replace('before_startup_fix_','before_n10_')
installer=installer.replace('reviewed reviewed LMPC startup convergence fix','reviewed LMPC N10 fix').replace('reviewed LMPC startup fix','reviewed LMPC N10 fix')
(stage/'scripts/apply_n10_overlay.py').write_text(installer,encoding='utf-8')
print(json.dumps(dict(files=files,exact_replay=exact),indent=2))
