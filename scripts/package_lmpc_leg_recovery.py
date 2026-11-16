from pathlib import Path
import hashlib
import json
ROOT=Path(__file__).resolve().parents[1]
stage=ROOT/'build/lmpc_leg_recovery_20261007';target=ROOT.parent/'H723_LMPC';baseline=ROOT/'build/lmpc_wheel_20261007'
p=stage/'README.md';s=p.read_text(encoding='utf-8')
s=s.replace('默认为 1.0/.2，正常时实际 QP 上限为轮 ±.030、Tp ±.004 N·m',
 '默认为 1.0/1.0，正常时实际 QP 上限为轮 ±.030、Tp ±.020 N·m')
s=s.replace('20（本工程默认）','20').replace('固件编号 10706 | 上次 LMPC','固件编号 10707 | 上次 LMPC')
s=s.replace('当前 10706 版尚待板上耗时与动力复测','当前 10707 版尚待板上耗时与动力复测')
s=s.replace('组号修改只存 RAM，重启恢复默认 20','组号修改只存 RAM，当前完整固件重启恢复默认 27（局部 PD/LMPC Tp）')
s+='\n## 2026-10-07 两腿后摆诊断与恢复（10707）\n\n10706 实机捕获 1562 次求解、0 次失败、最大 2.28235 ms；但两腿平均角约 105→156°、轮请求始终 +.030、LMPC Tp 始终 -.004 N·m，说明求解成功不代表实机可控。10707 恢复完整 Tp ±.020 请求预算（0.5 s 渐入），加入每腿 theta ±15°、phi ±15°、相对腿角 90°±20° 的线性工作范围停机。增加最初 640 ms 的 5 ms 启动记录和独立故障帧，默认匿名 27 组显示平均局部 PD/公共 LMPC Tp（mN·m）。完整文件 `build/keil/H723_LMPC_LEG_RECOVERY.hex`，编号 10707，用户自行烧录；尚未实机确认后摆消失。详见 [后摆修正版](docs/lmpc_leg_recovery.md)。\n'
p.write_text(s,encoding='utf-8')
p=stage/'docs/cpu_temperature.md';s=p.read_text(encoding='utf-8').replace('当前轮子修正版：完整控制/影子编号均为 **10706**',
 '当前摆腿恢复版：完整控制/影子编号均为 **10707**');p.write_text(s,encoding='utf-8')
p=stage/'docs/lmpc_wheel_output.md';s=p.read_text(encoding='utf-8')
if not s.startswith('> 历史版本'):
 s='> 历史版本 10706。当前虚拟 Tp 预算和姿态停机已更新，见 [10707 摆腿恢复版](lmpc_leg_recovery.md)。本文参数与验证范围适用于 10706。\n\n'+s
p.write_text(s,encoding='utf-8')
doc='''# 两腿共同后摆诊断与恢复（10707）

## 实机证据与未确定之处

用户确认仅 CH5 上挡使能时两条腿一起向后摆，腿长没有明显塌缩。10706 入口 Flash 片段匹配对应 AXF；CH5 中挡后只读采集冻结的 50 Hz 轨迹，未暂停/复位/写内存/烧录。1562 次求解没有失败、最大耗时 2.28235 ms；关闭后 solver status=0 是正常重置。过去的超时不是此次持续后摆的故障证据。

保存的最后 5.12 s 中，平均模型腿角约 105.49→155.84°，theta 0.326→1.049 rad，机身 pitch 约 +3.16→-5.75°。轮软件请求持续 +.030 N·m、公共 Tp 持续 -.004 N·m。相对停车目标的位置误差增至 .634 m。完整捕获在 tests/data/leg_rearward_10706.json。

这些数据不能证明 Tp 极性需要反转。输入已经饱和且姿态远离线性模型，实际方向、局部 PD、轴向支撑和轮子运动的作用无法由公共 Tp 单独推断。旧滚动记录只保留最后 5.12 s，最初使能帧已被覆盖。本版不改变电机/编码器符号、B 矩阵或参考坐标。

## 本版修改

- leg_scale .20→1.0，使 LMPC 可用 Tp 预算由 ±.004 恢复到原有虚拟上限 ±.020 N·m；启动仍用 .5 s 渐入，首个 5 ms 周期 ±.0002，随后逐步增加。轮仍 ±.030 N·m，各关节支撑/PD/同步合成后仍限 ±.680 N·m。
- 控制周期 5 ms、10/10 步、4 ms 求解截止、Q/R、物理参数和既有 DTCM 矩阵缓存保留。不是放大已求出的命令；实际更大 Tp 的新板上耗时尚待复测。
- 完整平衡输出前检查每条腿：|theta|≤15°、|phi|≤15°、模型相对腿角距90°≤20°。任一越界均拒绝输出、发送零并锁止，需关闭 CH5 再解锁。新增故障 12=ROBOT_FAULT_MODEL_REGION，balance status=4。这是本线性控制器的诊断工作范围，不是机械关节限位，也不是已验证的可恢复范围。纯独立支撑、零输出影子/只读的已有依赖处理保留。
- 新 g_lmpc_startup_trace 保留每次使能最初 128 帧×5 ms（640 ms）以及独立故障帧。记录真实输入状态、优化参考、两腿角/局部参考、公共 Tp、两腿局部 PD、同步、支撑力、腿长、轮请求与八槽电压。关机冻结，新的 OFF→ON 才开始新一组。不会不断覆盖最初的动作。故障帧保留停机前阶段和退出姿态，输出字段反映停机后的零指令。
- 缓冲 17560 字节；ARM6 完整 ZI=84272 字节，静态最大栈链2528字节（另有未知函数指针项），任务栈4096字节。影子构建不链接此启动缓存；新的实机栈余量仍待测。

## 匿名上位机

当前完整固件默认 ano_pair=27，只使用原来的前两个 S32 通道：通道1=两腿平均局部角度 PD 力矩，通道2=公共 LMPC Tp，显示单位 mN·m，两个显示比例均 .01。例如通道2的20表示 .020 N·m。局部项由实际总摆腿请求减公共 Tp 与同步项得到。robot24 的128字节payload、版本、CRC以及原有组均保留；没有增加串口链路或有效通道。

原有6组为轮T/LMPC Tp（mN·m），3组为两腿角（°），8组为左右支撑力（N），14组为两腿总摆腿请求（mN·m），22组编号/求解µs，23组耗时/迭代，24组求解状态/模式。默认PA10、115200、50Hz。读取只反馈固件时CH5自动选组仍保留；完整控制的自动只读选择不会覆盖27组。

## 验证与使用

实际任务/CAN 主机测试证明 .5 s 渐入、首帧局部参考捕获、Tp可超过旧 .004但不越 .020、轮/关节硬上限、前128帧冻结、故障帧、正负机身/单腿越界当周期零输出、故障后上挡不能自动重启以及OFF→ON恢复。捕获到的156°相对腿姿态现在会停机。另通过既有任务/遥控/反馈/CAN/求解截止失败、温度锁止、匿名通道/CRC/固定robot24、影子零输出测试以及完整/影子/只读编译。

完整文件 build/keil/H723_LMPC_LEG_RECOVERY.hex，同名AXF；编号10707，用户自行烧录H723。通用 H723_LMPC.hex/.axf 内容相同；WHEEL_FULL/N10/STARTUP_FIX 别名仍是旧文件。

尚未烧录本版，也未确认真机后摆已消失。若上挡后变红，保留主控供电并关闭CH5，读取last_fault和g_lmpc_startup_trace来区分姿态越界、反馈故障与求解失败；勿将12号工作范围停机误判为MPC超时。若刚使能就因12号停机，应先核对初始模型theta/phi与机械姿态，再决定下一步，不通过改符号或扩大工作范围掩盖问题。
'''
(stage/'docs/lmpc_leg_recovery.md').write_text(doc,encoding='utf-8')
intended=['App/Tasks/Inc/wheel_leg_config.h','App/Tasks/Inc/wheel_leg_task.h','App/Tasks/Src/wheel_leg_task.c',
 'App/Services/Src/robot_telemetry_service.c','App/Services/Inc/robot_telemetry_config.h',
 'App/Protocol/Inc/robot_telemetry.h','App/Protocol/Src/robot_telemetry.c',
 'tests/verify_integration.py','tests/verify_telemetry.py','tests/verify_leg_recovery.py','tests/data/leg_rearward_10706.json',
 'README.md','docs/cpu_temperature.md','docs/lmpc_wheel_output.md','docs/lmpc_leg_recovery.md']
files=[];hashes={}
for name in intended:
 p=stage/name;dest=target/name;assert p.is_file()
 if dest.exists() and p.read_bytes()==dest.read_bytes():continue
 if dest.exists():assert (baseline/name).exists() and (baseline/name).read_bytes()==dest.read_bytes(),f'Intervening edit: {dest}'
 files.append(name);hashes[name]=hashlib.sha256(dest.read_bytes()).hexdigest() if dest.exists() else None
(stage/'leg_recovery_overlay.json').write_text(json.dumps(dict(files=files,baseline=hashes),indent=2),encoding='utf-8')
installer=(baseline/'scripts/apply_wheel_overlay.py').read_text(encoding='utf-8')
installer=installer.replace('wheel_overlay.json','leg_recovery_overlay.json').replace('before_wheel_','before_leg_recovery_')
installer=installer.replace('reviewed wheel-budget and solver-cache fix','reviewed leg-budget, model-region and startup-trace fix')
(stage/'scripts/apply_leg_recovery_overlay.py').write_text(installer,encoding='utf-8')
print(json.dumps(dict(files=files),indent=2))
