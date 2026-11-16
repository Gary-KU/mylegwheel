from pathlib import Path
import hashlib
import json
ROOT=Path(__file__).resolve().parents[1]
stage=ROOT/'build/lmpc_wheel_20261007';target=ROOT.parent/'H723_LMPC'
baseline=ROOT/'build/lmpc_n10_20261007'
p=stage/'README.md';s=p.read_text(encoding='utf-8')
s=s.replace('默认为 .7/.2，正常时实际 QP 上限为轮 ±.021、Tp ±.004 N·m',
 '默认为 1.0/.2，正常时实际 QP 上限为轮 ±.030、Tp ±.004 N·m')
s=s.replace('固件编号 10705 | 上次 LMPC','固件编号 10706 | 上次 LMPC')
s=s.replace('22 组显示影子固件编号 10705','22 组显示影子固件编号 10706')
s=s.replace('影子编号 10705，默认','影子编号 10706，默认')
s=s.replace('当前 10705 版尚待板上耗时与动力复测','当前 10706 版尚待板上耗时与动力复测')
s+='\n## 2026-10-07 轮子输出与求解缓存修正（10706）\n\n轮子默认比例由 .7 恢复到 1.0，QP 可用软件力矩请求预算由 ±.021 恢复为用户指定的 ±.030 N·m。10/10 步、5 ms 周期、Tp ±.004 和各关节 ±.680 N·m 保留。约束修正使用 DTCM 中的 H 缓存，减少重复 Flash 读取；大误差突变时重置数值暖启动、保留已施加输入和变化率约束，并对大输入回零工况选用中档 rho。完整文件为 `build/keil/H723_LMPC_WHEEL_FULL.hex`，编号 10706，用户自行烧录。详见 [轮子修正版](docs/lmpc_wheel_output.md)。\n'
p.write_text(s,encoding='utf-8')
p=stage/'docs/cpu_temperature.md';s=p.read_text(encoding='utf-8').replace('当前 10 步版：完整控制/影子编号均为 **10705**',
 '当前轮子修正版：完整控制/影子编号均为 **10706**');p.write_text(s,encoding='utf-8')
p=stage/'docs/lmpc_n10.md';s=p.read_text(encoding='utf-8')
if not s.startswith('> 历史版本'):
 s='> 历史版本 10705。当前轮子可用请求预算与求解缓存已更新，见 [10706 轮子修正版](lmpc_wheel_output.md)。本文参数和实测记录适用于 10705。\n\n'+s
p.write_text(s,encoding='utf-8')
record=json.loads((stage/'build/lmpc_wheel_replay_validation.json').read_text())
exact=next(r for r in record['cases'] if r['scale']==1 and r['length']==record['live_timeout']['length'] and any(r['applied_nm']))
doc=f'''# 轮子可用预算与求解缓存修正（10706）

10705 的只读调试记录显示：wheel_scale=.7，所以 LMPC 可用轮力矩请求仅 ±.021 N·m；另有 LMPC 状态 4 的超时记录：48 次迭代、4.161065 ms，超过 4 ms 截止，累计 1157 次求解、2 次失败。读数时 CH5/控制已经关闭，不能据零输出判断之前绿灯时的真实输出。新版本之前仅采样比对了 Lmpc_Solve 入口 32 字节 Flash 与 10705 AXF，未暂停/复位/改写/烧录主控。

## 修改

- wheel_scale=.7→1.0，恢复原先用户要求的 ±.030 N·m 软件请求预算，约增加 42.9%；每步变化仍 ±.003 N·m，不放大已优化结果，不增加硬上限。10/10 步、5 ms 采样、4 ms 截止、Q/R 和物理模型保持。
- 用额外 1600 字节 DTCM 缓存当前腿长表的 H；约束修正中重复矩阵读由 Flash 改为 RAM。冷启动或腿长表切换时重新复制，暖启动使用对应缓存。
- 梯度幅度中档 rho 门限 25→20，解决本次相邻工况发现的弱罚参数问题。已施加任一输入超过两步变化量时，至少选择中档 rho，解决大轮输出回到零误差时仍受变化率约束的 QP。
- 大梯度突变时丢弃旧数值暖启动，保留真实已施加输入、幅值/变化率边界。接受仍需原残差/KKT 检查；超时/失败仍补发零并要求 CH5 重解锁。
- 支撑、同步、Tp ±.004 N·m、各关节 ±.680 N·m、温度保护、只读/影子零输出保留。

12 V 配置、Nm/V 标定和旧供电换算 ratio=.7 保留。恢复 .030 N·m 预算后，静止时左右轮满请求分别约 2.1875/2.0792 V（原 .021 预算约 1.5313/1.4554 V）。这是 H723 电压模式的软件请求；未采电流，不能当作实测轮力矩。实际电池/驱动电压与 Nm/V 标定仍需实机核对。

## 验证

主机重放最新真实输入及 23 个邻近 QP，并对完整轨迹、KKT、幅值/非零已施加输入的变化率约束独立核对；精确输入 {exact['iterations']} 次 ADMM 后收敛，最终修正 {exact['refinements']} 次。另以恢复后的 .030 轮预算重复 24 组和 64 次暖启动序列，均通过。

实际任务/CAN 集成测试证明：4° 机身输入能请求超过旧 .021 预算并保持不超过 .030，各关节限幅保留；随即回零再持续 50 周期不产生旧暖启动导致的迭代用尽。电压换算被独立核对。另通过 48 组旧启动、前一真实超时的 24 组/64 次暖启动、33 个 QP、8 组小扰动线性闭环、遥控/反馈/CAN/截止失败、温度锁止、匿名帧及影子零输出验证。

新增缓存和数值启动策略未在此板上测时，动力平衡未验证。主机收敛与模型测试不代表真机已经能平衡。

## 使用

完整固件：`build/keil/H723_LMPC_WHEEL_FULL.hex`；同名 AXF 用于调试。编号 10706，用户自行烧录 H723。`H723_LMPC.hex/.axf` 是相同内容的通用输出。旧 N10/STARTUP_FIX 别名仍是历史版本。

匿名 PA10、115200、50 Hz、仅前两个 S32 通道，比例均 .01。无需新增通道：ano_pair=6 为轮 T / 虚拟 Tp（mN·m）；7 为左右轮指令电压（V）；22 为编号/求解 µs；23 为 µs/ADMM 次数；24 为状态/模式（1 收敛、3 迭代用尽、4 超时）。完整控制默认仍 20 位移误差/速度；手动选择数据时将 ch5_select=0。

Watch 看 g_lmpc 的 cycles/max_cycles（除以400得 µs）、status、polish_reject_reason，g_wheel_output 的 requested/applied_torque_nm、torque_voltage_mv、voltage_mv 和 saturated。关闭时输出零是停机行为，last_fault 保留故障记录。
'''
(stage/'docs/lmpc_wheel_output.md').write_text(doc,encoding='utf-8')
intended=['Control/LMPC/Src/lmpc.c','App/Tasks/Inc/wheel_leg_config.h',
 'App/Services/Src/robot_telemetry_service.c','tests/verify_integration.py','tests/verify_lmpc_startup.py',
 'tests/verify_wheel_replay.py','tests/data/n10_wheel_failure_20261007.json',
 'README.md','docs/cpu_temperature.md','docs/lmpc_n10.md','docs/lmpc_wheel_output.md']
files=[];hashes={}
for name in intended:
 p=stage/name;dest=target/name
 assert p.is_file()
 if dest.exists() and p.read_bytes()==dest.read_bytes():continue
 if dest.exists():
  assert (baseline/name).exists() and (baseline/name).read_bytes()==dest.read_bytes(),f'Intervening change: {dest}'
 files.append(name);hashes[name]=hashlib.sha256(dest.read_bytes()).hexdigest() if dest.exists() else None
(stage/'wheel_overlay.json').write_text(json.dumps(dict(files=files,baseline=hashes),indent=2),encoding='utf-8')
installer=(baseline/'scripts/apply_n10_overlay.py').read_text(encoding='utf-8')
installer=installer.replace('n10_overlay.json','wheel_overlay.json').replace('before_n10_','before_wheel_')
installer=installer.replace('reviewed LMPC N10 fix','reviewed wheel-budget and solver-cache fix')
(stage/'scripts/apply_wheel_overlay.py').write_text(installer,encoding='utf-8')
print(json.dumps(dict(files=files,exact_replay=exact),indent=2))
