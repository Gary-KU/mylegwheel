from pathlib import Path
import hashlib
import json
ROOT=Path(__file__).resolve().parents[1]
stage=ROOT/'build/lmpc_backup_20261007';target=ROOT.parent/'H723_LMPC';baseline=ROOT/'build/lmpc_leg_recovery_20261007'
p=stage/'README.md';s=p.read_text(encoding='utf-8')
s=s.replace('QP 非收敛或 DWT 计时达到 4 ms 就拒绝命令、停止并要求重新解锁',
 'QP 迭代用尽或 DWT 达到4 ms 时拒绝该QP候选，切到受限LQR继续控制；无效输入和真实反馈/姿态/温度故障仍停机')
s=s.replace('固件编号 10707 | 上次 LMPC','固件编号 10708 | 上次 LMPC')
s=s.replace('当前 10707 版尚待板上耗时与动力复测','当前 10708 版尚待板上耗时与动力复测')
s=s.replace('当前完整固件重启恢复默认 27（局部 PD/LMPC Tp）',
 '当前完整固件重启恢复默认28（控制模式/累计保底切换次数）')
s+='\n## 2026-10-07 求解失败保底控制（10708）\n\n仅QP状态3/4改为本周期切到受限1450g LQR，不清停车目标、不关整机使能。LQR持续用当前反馈计算，按原力矩与变化率限幅；每20ms冷探测一次LMPC，连续10次成功后用100ms平滑返回。遥控/反馈/CAN/姿态/温度异常、无效输入仍停机。默认匿名28组为模式0关闭/1LMPC/2LQR/3过渡，以及累计切换次数。完整固件 `build/keil/H723_LMPC_LQR_BACKUP.hex`，编号10708，用户自行烧录；尚未实机验证保底切换与停车效果。见 [保底控制说明](docs/lmpc_backup.md)。\n'
p.write_text(s,encoding='utf-8')
p=stage/'docs/cpu_temperature.md';s=p.read_text(encoding='utf-8').replace('当前摆腿恢复版：完整控制/影子编号均为 **10707**',
 '当前求解失败保底版：完整控制/影子编号均为 **10708**');p.write_text(s,encoding='utf-8')
p=stage/'docs/lmpc_leg_recovery.md';s=p.read_text(encoding='utf-8')
if not s.startswith('> 历史版本'):
 s='> 历史版本10707。当前数值求解失败会切到LQR保底；姿态等保护仍停机，见 [10708保底控制](lmpc_backup.md)。本文默认通道和求解失败行为适用于10707。\n\n'+s
p.write_text(s,encoding='utf-8')
doc='''# LMPC 求解失败切到 LQR 保底（10708）

## 触发与输出

10707 板上读取确认：控制任务5ms、硬故障寄存器0、结温44.6°C；6569次求解中5次失败，最后4.168905ms超过4ms求解截止。停车目标固定0.5468897m，最后状态0.7440748m、前进速度0.0656174m/s。旧行为把求解超时当成整机停机，清零后不能继续主动平衡或刹停。捕获在 tests/data/parking_timeout_10707.json；此事件并不证明全部停车不稳都由超时造成。

本版只有数值返回状态3（迭代用尽）/4（超时）可以使用保底。当周期丢弃失败QP输出，调用当前1450g LQR、相同六状态/参考和已确认符号，重新计算轮/腿请求。不保持超时前旧力矩，也不执行未收敛的QP。停车目标、局部腿角PD、支撑和同步不因算法切换重新捕获。

LQR使用现有1.15倍拟合K；轮比例由当前轮限幅比例得到，腿原始LQR增益最多按旧0.20比例且随启动渐入，不把MPC的leg_scale=1直接作用到原始LQR。最终两控制器/过渡统一限幅：轮±.030、虚拟Tp±.020 N·m及本次更小的启动/Watch限幅；每样本轮变化≤.003、Tp≤.002 N·m。新的硬限幅下降优先于旧请求。关节支撑/PD/同步合成后仍各±.680 N·m，电压硬限压不变。Nm是电压模式请求估计，不是电流实测力矩。

## 探测与恢复

- 模式0=关闭，1=LMPC，2=LQR保底，3=返回LMPC的混合过渡。
- 保底每5ms持续计算新LQR请求；LMPC每20ms探测一次。探测时冷启动数值工作空间，并以最近已接受的软件请求作为首步变化率锚点，避免把20ms前的预测计划只移一帧。若一直数值失败，则在反馈/姿态有效时持续LQR，不因失败次数自动断电。
- 连续10次探测成功（至少约200ms）才进入过渡；随后100ms从LQR平滑混合到LMPC。混合后的指令仍再次施加幅值和变化率限制。过渡中再次失败就返回LQR、重计恢复成功次数。
- 原4ms求解截止、5ms任务周期不变，为快速保底与输出保留约1ms。新的实际板上总周期/栈余量仍需复测；本版不保证任何CPU硬异常或不返回的代码能被此选择器处理。
- 无效状态/模型/数值（包括求解状态2）、遥控丢失/关闭、反馈失效、CAN发送失败、姿态工作范围越界、过温/温度失效仍停机并保留原解锁规则。CPU硬故障不属于可恢复QP返回。

g_lmpc_control是只读诊断镜像；实际模式、输入锚点和恢复计数在私有状态中。可看mode/fallback_events/solver_rejections/probe_attempts/consecutive_good/skipped_probes/recoveries/last_solver_status/blend/selected_nm/previous_nm。镜像Watch改值不会选择实际输出控制器。停止时模式/指令清零，累计事件与最后求解状态保留。

执行器成功接受请求后维护选择器输入锚点；即便下游发生电压/关节饱和、需要重置QP数值暖启动，也不把上一条请求误当成0。输入锚点仍是请求估计，实际饱和后的电流力矩未测量。

## 串口

完整固件默认匿名28组：通道1控制模式（0/1/2/3），通道2累计进入LQR保底次数。PA10、115200、50Hz，只有前两个S32有效，比例均.01。例如原始200显示2=LQR。27组仍为平均局部PD/当前平衡控制器Tp（mN·m），后者在保底时就是LQR请求；24组完整固件为上次QP状态/控制模式。QP状态4且模式2表示QP超时、LQR仍运行，不代表整机关闭。

影子固件24组仍是求解状态/测试输入模式，默认26组温度；28组影子显示0/0，不代表电机可发力。原robot24 payload128字节、版本、CRC和原ANO组保留；两个新字段仅在任务采样结构内部用于28组，不增加无线链路或有效通道。

## 验证与使用

实际任务/CAN的真实求解截止注入验证：一次超时当周期保持使能、连续1秒数值失败不掉控制、每20ms探测、LQR随当前正反机身反馈改变、目标保持、连续成功计数/再失败归零、混合返回/混合失败再次保底、全程幅值/变化率/关节/电压限幅。保底中的遥控、反馈、CAN、只读、姿态与温度异常均确实发送零并停机。

另用真实C求解器的主机1次迭代构建验证状态3也进入保底；捕获的真实停车失败状态与独立矩阵乘法LQR对照；非法输入/镜像篡改验证停机；8组不同腿长、正反小扰动的离散模型在永久强制数值拒绝下收敛。通过既有腿恢复/启动记录、热保护、匿名CRC/robot24以及影子零输出检查。

ARM6完整Code126724、RO96368、RW2056、ZI84344字节，静态最大栈链2528字节（有未知函数指针项），任务栈4096；三构建均通过。未烧录或开启电机，未测本版板上耗时或真实停车稳定性。此修改解决数值拒绝时突然失去控制，不能证明模型/停车参数已匹配真机。

用户自行烧录 build/keil/H723_LMPC_LQR_BACKUP.hex，配合同名AXF；编号10708。通用H723_LMPC.hex/.axf内容相同；LEG_RECOVERY/WHEEL_FULL/N10等旧别名仍是历史文件。仅更新H723，不需要为本次选择器修改F103。
'''
(stage/'docs/lmpc_backup.md').write_text(doc,encoding='utf-8')
intended=['Control/LMPC/Inc/lmpc_control.h','Control/LMPC/Src/lmpc_control.c','MDK-ARM/H723_LMPC.uvprojx',
 'App/Tasks/Src/wheel_leg_task.c','App/Services/Src/robot_telemetry_service.c','App/Services/Inc/robot_telemetry_config.h',
 'App/Protocol/Inc/robot_telemetry.h','App/Protocol/Src/robot_telemetry.c','tests/verify_integration.py',
 'tests/verify_shadow.py','tests/verify_temperature.py','tests/verify_telemetry.py','tests/verify_backup.py',
 'tests/verify_backup_model.py','tests/data/parking_timeout_10707.json','README.md','docs/cpu_temperature.md',
 'docs/lmpc_leg_recovery.md','docs/lmpc_backup.md']
files=[];hashes={}
for name in intended:
 p=stage/name;dest=target/name;assert p.is_file()
 if dest.exists() and p.read_bytes()==dest.read_bytes():continue
 if dest.exists():assert (baseline/name).exists() and (baseline/name).read_bytes()==dest.read_bytes(),f'Intervening edit: {dest}'
 files.append(name);hashes[name]=hashlib.sha256(dest.read_bytes()).hexdigest() if dest.exists() else None
(stage/'backup_overlay.json').write_text(json.dumps(dict(files=files,baseline=hashes),indent=2),encoding='utf-8')
installer=(baseline/'scripts/apply_leg_recovery_overlay.py').read_text(encoding='utf-8')
installer=installer.replace('leg_recovery_overlay.json','backup_overlay.json').replace('before_leg_recovery_','before_lqr_backup_')
installer=installer.replace('reviewed leg-budget, model-region and startup-trace fix','reviewed bounded LQR fallback and recovery selector')
(stage/'scripts/apply_backup_overlay.py').write_text(installer,encoding='utf-8')
print(json.dumps(dict(files=files),indent=2))
