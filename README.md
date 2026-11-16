# MyLegwheel 工程

源码与性能整理（2026-11-07）：已分离观测、调度与控制职责；LQR 使用共享局部增益内核，两个接口分别通过 55,240 组生产 ARM 机器码按位回归；电流查询改用标量原子快照；启动时启用 I-cache。栈余量每秒采样，`g_wheel_leg_runtime` 记录耗时。算法参数和时序门禁保持原值，未烧录；入口与限制见 [控制代码说明](docs/control_code_map.txt)。

2026-11-01 六路电流后端候选：节点1/2/5/6为4015关节（11极对、Kt0.23），节点3/7为2808轮（7极对、Kt0.06616仅估算）。已完成A1电流通信、FAST关节反馈、Nm/Kt力矩路径、参数读回/ACK/超时停机、H7 bus-off停机通信重建及两profile构建；**尚未烧录，轮Kt、实际动态响应和关节安装标定门禁尚未确认**。配置、Watch小电流脉冲与调试步骤见 [FOC_CURRENT_INTEGRATION.md](docs/FOC_CURRENT_INTEGRATION.md)，产物和SHA256见 `build/foc-current/manifest.json`。下文实机记录均为此前的旧电压工程历史。

当前实机（2026-10-08）：0826 已烧录并完整HEX读回校验，保留第23组584帧原始日志。CH5关闭、六路电机输出零、采集READY state0/count0；前后最大0.15m/s、yaw最大0.45rad/s。实际跟踪改善待测试。部署：diagnostics/lqr_remote_0826_20261008_162746.json。下文未烧录及旧硬件状态均为历史。

最新候选（2026-10-08，2026100826）：已修改并编译，尚未烧录。前后限速0.15m/s、加速度0.20m/s²；yaw限速0.45rad/s、加速度1.0rad/s²。行驶共享倾角改为速度误差有界学习，增加±3mNm行驶纠偏、堵转/饱和暂停积分；yaw增益和差动力矩适度提高、30ms滤波、回中即纠偏。七项主机测试和89源ARMCLANG构建通过，实机效果待测。候选及SHA256：build/remote_tracking_0826_20261008/manifest.json；详见docs/remote_tracking_0826_20261008.txt。最后已核对实机为0825，第23组已CRC备份且未重新准备采集；下文旧硬件状态均为历史。

当前实机（2026-10-08）：LQR 遥控版 2026100825 已烧录，完整 HEX 读回一致，旧 Flash 日志完整保留。CH2 前后方向已修正为 -1，CH1 右推 +1、右转 gyroZ 为负已实测，yaw 角速度闭环已启用。CH5 关闭、全部输出零、采集 READY（state=0/count=0）。前后与转向跟踪效果待完整 V4 测试，用户反馈直行保持稍好但跟踪仍差；刚才首轮仅有 RAM 末 5.1 秒，缺目标/yaw字段，不能据此调参。部署：diagnostics/lqr_remote_0825_20261008_160026.json；说明：docs/remote_motion_direction_20261008.txt。下文硬件状态均为历史。

Current hardware (2026-10-08): 10803 logging update flashed and fully read back. Reserved Flash preserved; CH5=1000, controls OFF, all six motor voltages zero. Control behavior and parameters remain10802. RAM trace now retains51.2s at100ms, including lean and learning gates. Host60s wrap/freeze/rearm test and three ARM builds passed. Receipt: MyLegwheel/diagnostics/h723_lmpc_trace_10803_20261008_153428.flashed.json. Earlier hardware status paragraphs below are historical.


当前实机（2026-10-08）：已按用户要求烧录独立 H723_LMPC 的10801停车候选，OpenOCD Verified OK、全部HEX字节读回一致，0x080C0000起256KiB旧LQR日志逐字节保留。第22组896帧CRC1880776527、旧固件与RAM数据已备份。烧录后CH5中挡1000/link1，所有控制使能与六电机电压0；400MHz、5ms任务、停车gain0.05/阻尼0.01/输入界限0.10核对通过。未启动电机，真实短距离停车效果待测。先运行3..5秒关CH5后保持供电，LMPC只保留RAM轨迹。部署记录：diagnostics/h723_lmpc_parking_10801_20261008_145503.flashed.json。下文LQR0823和候选未烧录状态均为历史。

LMPC 对照候选（2026-10-08，10801）：已在独立 H723_LMPC 工程安装跟随当前 LQR 反馈的约束预测优化，并加入固定停车输入界限、共享倾角学习与最终限幅前的轮速阻尼。独立冷/暖 QP、真实任务/CAN/停机/保底/温度/遥测与72条同条件停车模型对照通过，完整/只读/影子93源构建通过；源码和旧固件已备份、安装SHA256已校验。**未烧录，实机仍是下述LQR0823**；模型最大位移约4.7→3.4cm、持续停稳最慢时间约3.36→1.01s，不等于真实刹车距离。说明见独立工程 docs/lmpc_parking_10801_20261008.txt；安装记录为 diagnostics/lmpc_parking_10801_installed_20261008.json。

当前实机（2026-10-08，build2026100823）：恢复gain0.05、学习速度资格0.25m/s及运行50ms/尾段20ms日志版已烧录、完整读回和运行参数检查通过。旧第20组CRC2474501968逐字节保留、最新0822RAM尾段备份，READY state0/count0，输出关闭。保持这次上电，固定摆放单轮运行30..35s，关CH5等3s后读取；K与停机保护保持，效果待测。见 [docs/parking_lean_entry_20261008.txt](docs/parking_lean_entry_20261008.txt)。下文为历史状态。

待烧录候选（2026-10-08，build2026100823）：按用户偏好恢复第一版自动补偿gain=0.05，姿态/输出正常时允许轮速<=0.25m/s学习，角度/变化速度及停机保护保持。日志运行50ms、停机尾段20ms，测试32s运行+2s完整尾段通过；控制/记录/遥测/隔离诊断/工程/理想模型及89源编译通过。候选已打包但未烧录，实机仍0822。详见 [docs/parking_lean_entry_20261008.txt](docs/parking_lean_entry_20261008.txt)。下文为历史状态。

最新读取（2026-10-08，第19组0822）：Flash首17.9s无新故障/输出限幅，速度由约0.23m/s逐步降到末5s平均0.0446m/s，补偿参考-1.18deg。不同上电的最新RAM尾段平均轮速-0.00639m/s、pitch0.75deg，不能按相同tick拼接到Flash。数据CRC备份、采集重新READY；未改K或固件，摆放/操作及重启说明待用户确认。见 [diagnostics/parking_lean_slow_g19_analysis_20261008.json](diagnostics/parking_lean_slow_g19_analysis_20261008.json)。下文为历史状态。

当前实机（2026-10-08，build2026100822）：学习增益0.025rad/m减半版已烧录、完整读回匹配、运行参数核对，旧第18组和最新0821RAM尾段完整备份保留，采集READY state0/count0，输出关闭。K与其余控制参数保持。保持此次上电，固定前侧配重跑一轮约12s，关闭CH5等3s后读取；实机效果待测。见 [docs/parking_lean_slow_20261008.txt](docs/parking_lean_slow_20261008.txt)。下文为历史状态。

优化候选（2026-10-08，build2026100822）：只将停车倾角学习默认增益0.05减半至0.025rad/m，K/方向/其他控制参数保持。控制/CAN/日志/回归/理想模型与89源编译通过，尚未烧录；最新检查CH5仍开启，更新脚本在只读检查处退出。实机仍为0821，需先关闭CH5保存尾段后更新。见 [docs/parking_lean_slow_20261008.txt](docs/parking_lean_slow_20261008.txt)。下文为历史状态。

最新读取（2026-10-08，第18组0821）：补偿参考实际最大-0.246度，LQR/局部PD目标一致、速率限制正常；第一轮Flash前17.9s无新故障/输出限幅，约9s后出现多次速度/姿态突变，操作和配重尚待用户确认。更晚RAM属于故障后又运行的尾段，平均轮速-0.00425m/s，不能拼接到首轮；日志只保留首轮。数据已CRC备份并重新READY，未修改K或固件。分析见 [diagnostics/parking_lean_g18_analysis_20261008.json](diagnostics/parking_lean_g18_analysis_20261008.json)。下文为历史状态。

遥控预留接口（2026-10-08）：已新增CH2前后速度/CH1右转目标构造器，默认关闭，仅生成带限速和变化率的目标；未接入电机或LQR、未再次烧录。保留当前0821配重测试版。见 [docs/remote_motion_entry_20261008.txt](docs/remote_motion_entry_20261008.txt)。

当前实机（2026-10-08，build2026100821）：K保持，新增低速固定停车世界腿倾角积分，最大±2度、0.2度/秒；LQR与局部PD目标联动。控制/CAN/记录/回归、理想模型与88源编译通过，已烧录完整读回，旧第17组完整保留。烧录后曾提前开启CH5，随后已关闭并完成准备，**现在采集READY state0/count0**。保持这次上电，固定前侧配重只跑一轮10..12s，关闭CH5等3s再读取。见 [docs/parking_lean_20261008.txt](docs/parking_lean_20261008.txt)。下文为历史状态。

最新读取：720实机Flash仍是旧第16组，已恢复最新RAM5.1秒并确认本轮手动CH5停机，旧故障9不属最近尾段。机身约0.8度，但轮轴仍平均向前0.094m/s；新增停车阻尼确实执行（均值-0.945mNm），未完全消除前移。旧Flash已备份，采集**现在READY state0/count0**；保持这次上电再跑一轮10..12秒。未继续改控制参数。分析见 [diagnostics/parking_damping_latest_RAM_analysis_20261008.json](diagnostics/parking_damping_latest_RAM_analysis_20261008.json)，步骤见 [docs/parking_damping_20261007.txt](docs/parking_damping_20261007.txt)。下文为历史状态。

当前实机（2026-10-07，build2026100720）：完整LQR增加小幅停车轮速阻尼0.01Nm/(m/s)，上限1.5mNm/轮，其余稳定基线参数保持；控制/CAN/日志/回归和88源文件编译通过，已烧录并完整读回匹配。用户反馈启动走一段后能停稳，随后没电去充电。**采集准备前已经运行，新一轮未记录进Flash，当前仍是旧第16组；充电后须先保持CH5关闭并重新准备采集。**不凭旧数据解释这次效果。见 [docs/parking_damping_20261007.txt](docs/parking_damping_20261007.txt)。下文为历史状态。

最新实测（2026-10-07，第16组，0711完整控制）：用户确认站立稳定但缓慢前移，曾手动转向。18.346秒完整记录无保护故障；末段俯仰约4.59度、轮速均值+0.062m/s、零速目标有效，但仍有+7.08mNm/轮请求。记录已CRC备份、采集重新准备。本次未改变固件；小幅低速停车阻尼只完成候选模型核算，尚未实施。见 [docs/stable_forward_drift_g16_20261007.txt](docs/stable_forward_drift_g16_20261007.txt)。

当前实机（2026-10-07 22:01）：已按用户要求从718单腿电压诊断恢复**0711完整LQR基线**，轮子/双腿支撑/同步及正常CH5控制恢复。此次未修改K、方向或完整控制参数，尚未新增前倾/后退修复。烧录及完整读回匹配，第15组数据逐字节保留并备份，采集READY。见 [docs/restore_full_lqr_after_leg_probe_20261007.txt](docs/restore_full_lqr_after_leg_probe_20261007.txt)。下文为历史状态。

排查重点更正（2026-10-07）：用户已确认腿的实际运动方向均正确，不再重复方向试验或反转标定。接地腿角下降不能由单个虚拟摆角请求的正负判断原因，后续回到完整控制的共同LQR/同步分配、支撑与实际驱动强度。板上目前仍为下述718隔离诊断固件，尚未恢复完整平衡。见 [docs/leg_direction_confirmed_20261007.txt](docs/leg_direction_confirmed_20261007.txt)。

最新隔离诊断（2026-10-07，build2026100718）：按用户指定2V、最多2秒，左腿几何比例定向输出，CH5中/下立即停；通过90度后保持原方向，运行速率界限4rad/s，原70..110度等保护保留，可能提前停止。完整控制K和增益未改。真实CAN/比例/隔离/停止/日志与回归通过，88源文件编译、烧录和完整读回匹配。第14组1V记录显示20ms内角度82.9->85度后被原1rad/s门限截停。测试须架住机身、左腿离地。详见 [docs/leg_voltage_probe_2v2s_20261007.txt](docs/leg_voltage_probe_2v2s_20261007.txt)。下文为历史版本。

最新实机诊断（2026-10-07，build2026100714）：按用户最后指定1V，左腿独立几何比例短脉冲，较大一路电压请求约1000mV、另一关节保持纯摆角J^T比例，最多300ms自动停止；到达90度可提前停止，其他电机/LQR/支撑/同步0。须架住机身、左腿卸载后只测一轮。实际CAN幅值/比例/隔离/停机/2秒记录与完整控制回归通过，88源文件编译并烧录、完整读回匹配，采集READY。旧第13组记录保留；旧版实际峰值只有26/29mV。步骤见 [docs/leg_voltage_probe_1v_20261007.txt](docs/leg_voltage_probe_1v_20261007.txt)。下文为历史版本。

最新诊断修改（2026-10-07，左腿build2026100712）：第12组0711测试仍后退并开叉，新增左腿隔离纠偏版的启动至停止、停机后2秒Flash记录和诊断模式标记。仅选中腿局部PD，关节指令±200mV、最多3秒；轮子/另一腿/LQR/支撑/同步关闭，须架住机身、卸载测试。左右隔离/真实任务记录/完整控制回归通过，左腿88源文件编译完成，**已烧录并完整读回校验，采集READY**。旧第12组记录逐字节保留，六电机输出关闭。完整LQR参数未改。固件与步骤见 [docs/leg_pd_record_20261007.txt](docs/leg_pd_record_20261007.txt)，本轮数据见 [docs/sync_priority_g12_analysis_20261007.txt](docs/sync_priority_g12_analysis_20261007.txt)。下文为历史版本。

最新共同回正优先分配（2026-10-07，build2026100711）：对照ESP原版共享LQR+差动同步结构，新增H7分配层；完整中性控制两腿同侧偏差时缩放同步，完全介入（两腿均偏离目标≥5°）至少保留各腿50%的PD+LQR回正指令，解除后平滑恢复。K与原增益/限幅、停车0.1m输入界限及停机后2秒采集不变；日志bit6/JSON/CSV记录介入。控制/Flash/遥测/单腿profile回归和88源文件构建通过，已烧录与完整读回，新enable1/输出0/采集就绪已核对。实机效果待测。参考差异、固件和步骤见 [docs/sync_recovery_priority_20261007.txt](docs/sync_recovery_priority_20261007.txt)。下文为历史版本。

最新停车位置输入对照（2026-10-07，build2026100710）：真实停车目标保持，将给LQR的位置误差输入限制在±0.10m；其余0709控制参数、保护和停机后2秒采集保持。日志补齐真实停车目标及停机尾段目标保留，避免把实际误差显示为限幅后的0.1m。控制/记录器/遥测回归、工程布局和88源文件编译通过；已烧录、完整读回及运行参数核对，旧记录完整保留，采集器就绪。实机改善待复测，步骤见 [docs/parking_input_bound_20261007.txt](docs/parking_input_bound_20261007.txt)。下文为历史版本。

最新共同前倾力度对照（2026-10-07，build2026100709）：腿部LQR比例0.20→0.30，未限幅时共同纠偏指令提高50%，0.12Nm上限/0.5秒渐入保持；轮、同步、支撑、标定/方向及停机保护不变，停机后2秒采集保留。控制/记录回归、旧记录固定状态重算与完整编译通过；已烧录、完整读回校验及leg_scale=0.30确认，采集器已准备，实机效果待测。见 [docs/common_pitch_gain_20261007.txt](docs/common_pitch_gain_20261007.txt)。下文为历史版本。

最新停机尾段采集（2026-10-07，build2026100708）：按用户选择保留所有停机保护和0707控制参数，故障/停机后继续以50Hz观察约2秒再冻结保存，保留四关节及姿态/轮速变化；总容量仍约17.9秒。C默认尾段/Flash/Python解码与控制回归、88源文件编译通过；烧录/完整读回及tail_ms=2000核对通过，采集器已准备。每次只跑一轮，结束关闭CH5后保持供电至少3秒，state3后通知读取。见 [docs/post_stop_tail_20261007.txt](docs/post_stop_tail_20261007.txt)。下文为先前版本记录。

最新同步响应对照（2026-10-07，build2026100707）：generation7显示约3.9秒开叉保护停机，同步±0.15Nm仍未拉住。同步小误差增益/差动速度阻尼提高到1.95倍，每腿上限0.195Nm、变化速度0.4Nm/s；轮/LQR/支撑/Nm-V标定及停机保护保留。控制、记录器、遥测回归与88源文件编译通过；烧录、完整读回及运行默认参数核对通过，旧记录完整备份，下一轮记录已准备。实机改善待测试。固件与步骤见 [docs/sync_response_20261007.txt](docs/sync_response_20261007.txt)。下文为先前版本记录。

详细动作采集（2026-10-07）：已补齐四关节角度/轴速度/反馈年龄与有效位、每腿伸缩/摆角速度、PD/同步/LQR/总摆角力矩、关节请求力矩及启动参数快照。控制参数保持上一版。记录v3每条288字节、50Hz、约17.9秒，兼容旧v1/v2记录；控制/记录器/串口回归及完整编译、烧录和读回校验通过。实机generation6的223条新记录CRC与新增字段导出已验证，最后故障9停机，当前新记录保留。见 [docs/detailed_motion_record_20261007.txt](docs/detailed_motion_record_20261007.txt)。

机身低头恢复对照（2026-10-07，最新已烧录）：共享腿LQR软上限0.020→0.120 N·m，保留腿比例0.2与其他增益；增加映射后每电机±0.68 N·m请求上限，原6.9V限压仍生效。完整平衡新增机身倾角±35°/40ms停机（故障10），防止两腿差角不大时继续低头。控制/Flash回归与全量构建通过，烧录和完整读回校验完成、输出全零，generation4原记录完整保留，下一轮采集已准备；实际恢复效果待验证。见 [docs/pitch_recovery_20261007.txt](docs/pitch_recovery_20261007.txt)。

开叉对照更新（2026-10-07）：已编译并烧录校验，同步每腿软上限0.10→0.15 N·m，原小误差增益与变化速度保持；完整平衡新增逐腿/左右分离姿态停机保护（故障9），避免平均状态掩盖严重开叉后继续驱动。轮角速度项0.5及支撑参数保持作对照；控制回归、记录器和完整H7构建通过。重启后输出为零，旧generation3记录完整保留，记录器已准备下一轮；实机效果待验证。固件与说明见 [docs/leg_sync_cap_pose_guard_20261007.txt](docs/leg_sync_cap_pose_guard_20261007.txt)。

主动停车角速度反馈对照（2026-10-07）：实机轮比例0.5仍出现往返，RAM补读的5.1秒中停车误差峰值约0.794m、111/256条轮输出限幅。已烧录对照版：保留轮比例0.5，将轮dtheta/dphi贡献乘`g_balance.wheel_rate_scale=0.5`，其余反馈及腿部控制保持。记录器改为成功启动第一拍开始采集，兼容并绕过旧的一拍失败启动记录。模型/控制回归/全量编译及烧录校验通过，真实刹车改善待复测。详见 [docs/braking_rate_compare_20261007.txt](docs/braking_rate_compare_20261007.txt)。

LQR 启动至倒下记录（2026-10-07）：新增50Hz、最多30.7秒的独立记录，从CH5启动第一拍采集，在大倾角、故障或停机时冻结；全部执行关闭后保存到预留的主控Flash，完成保存后可断电。冻结记录保留到显式rearm，附带参数快照和CRC。JSON/CSV读取及操作见 [docs/lqr_startup_flash_record_20261007.txt](docs/lqr_startup_flash_record_20261007.txt)。未烧录，主机模拟Flash、控制回归和完整编译通过；硬件保存时长/掉电行为待验证。

LQR 启动倒下诊断（2026-10-07）：当前源码匿名默认组22=左轮力矩指令（mN·m）/机身倾角（°），新增组23=左轮力矩/左轮电压、组24=启动阶段/最后故障码，两个S32倍率0.01；原分组保留。冻结记录可另导出CSV查看倒下前约5秒的力矩、电压、姿态、速度及停车误差。此次保留K、构建前已为1.0的轮比例和原12V/9V供电配置，未烧录（先前0.7说明沿用旧记录，已按备份更正）。固件与步骤见 [docs/lqr_left_wheel_diag_20261007.txt](docs/lqr_left_wheel_diag_20261007.txt)。

Keil 工程目录整理（2026-10-07）：完整 LQR 控制入口是 `MDK-ARM/MyLegwheel.uvprojx`，输出 `build/keil/MyLegwheel.hex/.axf`。已将左右单腿 PD 测试工程及旧启动汇编列表共 5 个文件归档到 `backups/mdk_arm_cleanup_20261007.zip`，主工程、Watch/窗口设置、启动汇编、链接及 RTE/调试配置保留。昨天最后恢复烧录的存档是 `build/original_full_ano_20261006/MyLegwheel_original_full_ano.hex/.axf`；当前源码轮比例为 0.7，昨天该存档记录为 0.8，两者需区分。整理未改控制源码、未重新编译或烧录，工程路径检查通过。恢复测试工程时将 ZIP 内容解压回 `MDK-ARM`；也可运行 `python scripts/create_leg_pd_keil_projects.py` 按当前主工程重新生成。

当前已按用户选择恢复最初完整控制并烧录（2026-10-06）：停车3cm修正和后来增加的0.03Nm软件截断均撤回，恢复原停车和6V轮输出路径，K/腿部保留。匿名新增默认组21：里程计速度（mm/s）/右轮电压指令（mV），两S32倍率0.01；组20为停车误差/速度，旧组保留。完整镜像Verified OK，主控实际F1帧及UART发送核对通过。约5秒RAM记录保留，闭环震动/往返仍待排查。详见 [docs/original_full_ano_20261006.txt](docs/original_full_ano_20261006.txt)。以下停车修正试验记录已由本次回退替代。

当前轮子震动/摔倒对照（2026-10-06）：已补每轮请求±0.03Nm限幅及约5秒冻结RAM记录。用户确认安静很多但仍前后倒；实机记录显示停车误差最多约1m、速度0.51m/s。现在已烧录停车参考修正版：保留原停车目标，只将进入LQR的远距离位置误差限至±3cm，K和腿部参数保留，完整镜像Verified OK，效果待短测。流程、限制和数据见 [docs/wheel_parking_oscillation_20261006.txt](docs/wheel_parking_oscillation_20261006.txt)。

当前已恢复并烧录完整控制（2026-10-06，用户要求“直接搞全功能”）：标准 `MyLegwheel.uvprojx` 默认 `feedback_only=0`，轮子平衡、双腿支撑/PD/同步和停车控制恢复，保留腿LQR比例0.20及±0.020Nm限制。CH5中/下停、上启，持续运行，不再3秒停止；串口组19为机身pitch/Tp。OpenOCD完整镜像Verified OK，重启读取无故障、CH5中挡、双腿80/64mm、输出零。只读恢复改为专用 `--profile read-only`，早前“默认只读”记录已由此替代。过程及固件路径见 [docs/full_control_restore_20261006.txt](docs/full_control_restore_20261006.txt)。

当前下一步（2026-10-06）：用户已确认速度修正版驱动的左右腿与机身反馈均正确。标准主控仍默认只读；另提供 `leg-pd-right` / `leg-pd-left` 专用主控构建，只使选中腿局部角度PD运行最多3秒，虚拟腿指令力矩±0.005Nm、每关节指令±200mV，其余腿/轮/LQR/支撑/同步输出为零。该专用版本CH5中/下停、上启，串口固定选中腿角度/角速度；不再通过CH5换数据对象。构建、输出隔离及停机回归通过，未自动烧录，实机输出纠偏方向仍待验证。步骤见 [docs/leg_pd_isolation.txt](docs/leg_pd_isolation.txt)。下列旧记录中的只读状态指标准构建。

当前自抖排查与速度链路修正（2026-10-06）：用户明确放下/手扶时腿板自行持续往返，Tp 随之变化，恢复控制仍暂停。查到 F103 角度发当前 shaft_angle，速度却经角度 Kalman 后再差分；六个驱动统一改为当前连续 shaft_angle 的5拍差分、按真实时间算 RPM、启动/失效重新建历史，C 估算器5Hz处理延迟16.317→2.500ms（不含 CAN/任务）。启动静止不再产生假速度。六个 F103 编译、反馈/启停回归通过，未烧录且未证明实机自抖已消失；主控保持只读 CH5 切组。包 build/feedback_speed_fix_20261006.zip 含ID1/2/3/5/6/7及只读H723，保留标定页/ID页，不做整片擦除。详见 docs/f103_feedback_speed_fix.txt。

关节猛摆后的低力度诊断（2026-10-06）：腿部 LQR 比例 1.0→0.20，比例/0.5 s 渐入后共同 Tp 限幅为 ±0.020 N·m；原始遥测 Tp 范围 −2000..2000，0.01 倍率显示 ±20 mN·m。保留并行启动、轮子控制、局部 PD、同步和支撑设置；该限制不等于总摆腿/电机力矩上限。交替 ±20° 且运行时比例 1 的主机重放、启停/故障归零验证通过；实机方向与稳定性仍待确认，旧满比例版本暂停使用。

启动阻塞修复（2026-10-06）：反馈有效后腿部 LQR 从首拍开始 0.5 s 渐入，与支撑同时运行；五项静态就绪条件改为诊断，不再等待 4.5 s 支撑升满或先恢复姿态。阶段 0→2→3，轮子 LQR 首拍运行；K、支撑升速、配平及限压保留当前源码设置。控制回归覆盖五位全缺失的 10°/95 mm 分腿姿态，505 ms 完成渐入；反馈/发送故障和 CH5 重新解锁通过，尚未验证实机恢复。以下旧准备阶段记录已由本次流程替代。

匿名两通道采集（先前控制版记录，已由只读诊断组 16 替代）：默认组 15 为启动阶段/已接入的腿部 LQR Tp；组 12 看姿态和支撑诊断，组 14 看左右合成摆角力矩。两个 S32 通道倍率均为 0.01，原始阶段值开启后 200，约 0.5 s 后 300。
默认仅 USART1 TX/RX SWAP 后的 PA10 发送（PA10 接模块 RX），暂定 115200、8N1；模块波特率仍需匹配。
配置、分组和验证见 [docs/robot_telemetry.md](docs/robot_telemetry.md)。本次未自动烧录。

LQR 质量更新（2026-10-06）：整机总重 1450 g，其他部件合计 350 g，机身 1100 g；每侧 LQR 使用 550 g 机身。已重算并更新 K，数值对照、控制回归和 H7 编译通过，尚未烧录。轮/腿各自的质量暂按旧几何比例分配，参数来源、估算说明和复现步骤见 [docs/lqr_mass_update.md](docs/lqr_mass_update.md)。

手动配平对照（2026-10-05）：针对持续前溜，将 `BALANCE_DEFAULT_PITCH_TRIM_DEG` 从 0 改为 −0.2°，机身目标略后倾；正常平衡后约 0.4 s 渐入，两腿局部角度参考相应变为 90.2°。K、轮/腿比例及输出上限未随此次配平调整。正负配平及当前默认值的主机控制回归通过；此为试验值，尚未验证实机效果，可设回 0 作对照。该功能不移动实体重心、不修改 IMU 零点，也不自动估计载荷。

滚动启动修正（2026-10-05）：移除准备阶段“每只轮轴速度不超过 1 rad/s”的条件，避免车溜走时持续等待先停车、腿部 LQR 无法接入。保留对齐、腿长、机身/腿部姿态及运动、支撑和反馈检查，准备确认 0.2 s 与渐入 0.5 s 不变。正反向约 5.2 cm/s 的模拟滚动均可进入正常平衡并捕获停车位置；控制回归和 Keil 编译通过，实机尚未验证。若 `position_locked` 一直为 0，仍需核对启动阶段、窗口设置和故障状态，不能仅凭这个标志确定阻塞原因。

固定停车位置（当前默认，2026-10-05）：遥控只使用 CH5 开关，前后速度目标为零。`g_balance.position_window_m=0` 在进入正常平衡（`startup_phase=3`）时捕获当前位置，此后保持目标，不随溜车移动；准备/渐入阶段保留 ±0.10 m 窗口。只读 `position_locked=1` 表示停车目标已捕获。停机、故障和单测接管时清除锁定，重启重新捕获。设窗口为 0.10 可恢复之前的移动窗口。主机控制回归与 Keil 编译通过，实际停车效果需烧录确认；轮子里程计打滑会影响地面位置。参数与观察项见 [docs/tuning.md](docs/tuning.md#前走后回退与电池前后偏重)。

当前参数保留用户源码的 `g_balance.leg_scale=0.9`，轮子比例 0.8、局部摆角 PD、左右同步、支撑和限压均未随停车修改增加。以下旧记录的 0.22/0.25 腿部比例已由当前 0.9 替代。`g_balance.pitch_trim_deg` 当前试验默认 −0.2°（机身略后倾），允许 ±3°，协调机身 LQR 倾角参考和两腿局部目标角度，在正常平衡后按 0.5°/s 渐入；它是手动姿态配平，不修改 IMU 原始零点。

可选承重补偿（2026-10-05）：新增 `g_dual_hold.load_enable`，默认 0。启用后按持续腿长误差慢慢修正每腿支撑，默认范围 ±2 N、变化速率上限 0.25 N/s；输出饱和或姿态不满足条件时冻结学习，停机/故障/关闭时清零。它不测量重量、触地、电流或 CC，总控制进入正常平衡后才学习。当前 16 V 供电匹配与左轮异常耗电仍需处理，先保持关闭；参数和观察项见 [docs/tuning.md](docs/tuning.md#可选的慢积分承重补偿)。

调参入口（2026-10-05）：默认值集中在 `App/Tasks/Inc/wheel_leg_config.h`，按支撑、虚拟腿转动、左右同步、轮子平衡等功能分组；Keil Watch 对应表与操作顺序见 [docs/tuning.md](docs/tuning.md)，可复制表达式见 [docs/tuning_watch.txt](docs/tuning_watch.txt)。现有 Watch 名称、增益、控制输出和故障处理保留。关节 F103 构建也读取集中配置的母线/电压上限；当前仍是主控/轮驱动 12 V、关节驱动 9 V，尚未适配约 16 V 电池。

支撑与虚拟腿转动力度（当前默认，2026-10-05）：承重试用前先作小幅提高，每腿支撑前馈从 8.5 增到 9.0 N（约 +6%），仍按 2 N/s 渐升。局部虚拟腿摆角保持 KP/KD 从 0.1/0.01 增到 0.11/0.011，局部力矩上限从 0.020 增到 0.022 N·m；腿部 LQR 比例从 0.20 增到 0.22（均约 +10%）。三个局部参数集中在 wheel_leg_config.h，运行时分别对应 g_dual_hold.angle_kp_nm_per_rad、angle_kd_nm_s_per_rad、angle_limit_nm；支撑为 g_dual_hold.feedforward_target_n，LQR 为 g_balance.leg_scale。为保留超长腿卸载，overlength_kp_n_per_m 从 250 增到 265 N/m，5 mm 起点、2 mm 平滑过渡和 0.5 s 渐入保留；目标 0.07 m、实际 0.11 m 的静态限幅前力约 -0.41 N，support_only=1 仍归零，不主动拉腿。轮子 0.8、关节 6900 mV/轮子 6000 mV 限压、同步比例 0.30 和原启停/提示保持。主机验证 9 N 支撑、力矩方向/限幅、超长卸载、启动、遥控闭锁及故障归零；0.07/0.11 m 正反 5° 倾斜下，LQR Tp 从约 0.00522 增至 0.00575 N·m。此为模型指令变化，承重能力和实机稳定性尚未验证。以下旧调参记录中的 8.5 N、0.20 腿 LQR 和 0.020 N·m 局部上限由本次参数替代。

启动状态提示（2026-10-05）：使用现有 PA7 板载 RGB LED 和 PB15 蜂鸣器。提示由低优先级灯光任务每 20 ms 读取控制状态，不增加启动超时、不改变电机控制。原上电旋律保留；正常停机和进入平衡后无状态蜂鸣。

| 提示 | 含义 | 蜂鸣 |
|---|---|---|
| 黄灯每组闪 1 次 | 遥控未解锁或失联；检查遥控连接，先中/下档再上档 | 每组 1 声 |
| 蓝灯每组闪 2 次 | 腿部准备或渐入尚未完成，仍有准备控制输出 | 每组 2 声 |
| 红灯每组闪 3 次 | 控制反馈/参数/发送故障，或 IMU 异常/超时 | 每组 3 声 |
| 绿灯呼吸 | 总控制开启且 startup_phase=3 | 静音 |
| 微亮绿灯 | IMU 正常、总控制关闭 | 静音 |
| 蓝灯连续慢闪 | IMU 上电初始化，最多保留 5 s，随后未正常采样转红色故障 | 仅原上电旋律 |

前三种灯每 2 s 重复一组；状态稳定 1 s 后开始蜂鸣，每 3 s 重复一组，状态恢复立即停止。故障优先于遥控等待和准备提示。Watch 看 `g_board_feedback.status`（0 停机、1 遥控等待、2 准备、3 故障、4 平衡、5 初始化），`ready_missing_mask` 显示准备阶段缺少的条件（1 轮子前后间距、2 腿长、4 姿态、8 速度、16 支撑前馈；可组合），`frequency_hz` 为当前请求的蜂鸣频率。设 `g_board_feedback.buzzer_enable=0` 关闭状态蜂鸣，上电旋律保留。具体故障继续看 `g_robot_control.last_fault`、`remote_status` 和 `imu_status`。主机验证提示次数/时序、优先级、准备条件位图、恢复/停机静音、手动静音和时钟回绕；硬件灯色/声音待烧录实测。

遥控启动开关（2026-10-05）：默认 `g_robot_control.remote_enable=1`，顺序 CH1～CH5 的第 5 通道（`channel[4]`）接管整机使能。按实机截图，上/中/下档原始值分别为 **306/1000/1694**，上档才开启总控制；中、下档同时停止关节、轮子和独立轮子单测。`g_sbus_debug.state.frame.channel[4]` 保留 SBUS 无符号原始值，最终正负百分比看 **`g_robot_control.remote_ch5_percent`**，上述三档对应 **+100/0/-100**。配置 `remote_on_raw/remote_center_raw/remote_off_raw` 表示三个档位中心，`remote_tolerance_raw=50` 表示各档容差；开启范围是 256～356，中档 950～1050 或下档 1644～1744 可确认失能解锁，其他过渡值停止且保留此前解锁。百分比限幅本身不能决定开启。上电、失联、故障或手动停止后先拨回中/下档，再拨上档；保持上档不会自动重启。SBUS 超过 100 ms 未收到有效数据、frame_lost、failsafe 或 UART/DMA 接收异常会停止。Watch 的 remote_ch5_raw/percent/link_ok/switch_on/rearm_ready/status 显示遥控状态，status 0/1/2/3/4/5 分别是 Watch 模式、无有效链路、失能档、等待回到失能档、遥控开启和参数错误。需使用原调试操作时扶稳停机，再设 `remote_enable=0`；默认遥控模式下 Watch 写 enable=1 或独立测试开关不能绕过失能档。前四通道保持原输入处理，此处只绑定 CH5 启停。主机验证实测档位、百分比、容差边界、首拍轮响应、失联/故障闭锁和停止归零；实机联动待烧录验证。新主控 HEX/AXF 烧录后生效，F103 无需更新。

中低力度腿部对称反馈（2026-10-05）：根据用户试用反馈，将同步从 25% 小幅提高到初版 30% 的强度，`g_dual_hold.sync_ratio=0.30`，额外同步力矩每腿最多 0.03 N·m。`sync_rate_nm_s=0.05`，每个 5 ms 周期最多变化 0.00025 N·m，从零到上限约 0.6 s。外环 12、内环 0.04、角差滤波与长度缩放保持原值。这是叠加在局部摆角保持和腿部 LQR 上的额外项，0.03 不是总腿力矩上限。轮子立即接入、8.5 N 支撑及原关节限压保持原值。正反开叉、不同腿长、分离角速度、首拍/重启渐变、故障归零和轮均值输出独立性主机检查通过；实机恢复效果与稳定性未验证。扶稳停机后，现有支持同步字段的固件可在 Watch 设置 ratio=0.30、rate=0.05，再开启总开关；烧录本次新 H723 HEX/AXF 后成为重启默认值。关闭同步作对照时停机设 ratio=0 再开启。

启动轮子立即接入（2026-10-05）：修正准备阶段只留速度阻尼、轮子静止且机身倾斜时没有平衡输出的问题。总开关开启且反馈有效后，轮子六状态 LQR 从第一拍按 `wheel_scale=0.8` 输出，不受腿部对齐门槛或 `startup_blend=0` 屏蔽；正常轮电压上限仍为 6000 mV。`startup_phase=1/2/3` 现在分别表示腿部准备、腿部 LQR 渐入、正常组合平衡。腿部 LQR 和轮反电动势补偿仍按准备条件与 0.5 s 渐入执行。轮位移参考从第一次有效轮控制开始保持，准备/渐入切换不重捕获，以保留溜车时的位置反馈。额外直接轮阻尼 `startup_wheel_damping_nm_s_per_rad` 改为默认 0，避免叠加改变现有 LQR 动力学；该项仍可手动诊断，`startup_wheel_limit_nm` 为 0.012 N·m，两项允许 0～0.02。若手动启用，额外阻尼按 `1-startup_blend` 退出，`startup_wheel_damping_torque_nm` 显示该项，轮总指令为六项 LQR 贡献加该项。主机回归在腿部准备、零轮速、机身前后倾斜 1° 的首拍得到左右约 -1149/+1092 mV 及其反向；姿态、位置参考连续性、限幅、故障及单测独占检查通过，尚未实机验证。烧录新的 H723 HEX 并加载同次 AXF 后生效，准备/渐入期间仍需扶稳机身。

STM32H723VGT6 轮腿机器人固件，使用 STM32 HAL、FreeRTOS 和 Keil ARM Compiler 6。当前运行 IMU、USB 遥测、灯光蜂鸣器、SBUS、CAN 反馈和一个 5 ms 轮腿观察任务。原关节点动、正方形跟踪、腿长保持实验和独立 VMC 演示任务已移除。

当前已接入双腿腿长保持、共享六状态 LQR 和 CAN 电压执行层，实机平衡稳定性尚未验证。控制入口为 `App/Tasks/Src/wheel_leg_task.c`，Watch 为 `g_wheel_leg`、`g_dual_hold` 和 `g_balance`；独立双腿纯计算示例已移到 `tests/support/wheel_leg/Src/wheel_leg_control.c`，不参与固件。`g_dual_hold.enable` 是执行总开关，`g_balance.enable` 加入 LQR；两者默认均为 0，反馈或发送异常会关闭执行，需要手动重新使能。

整机默认由上述 CH5 控制；切到 `remote_enable=0` 后，原 Watch 调试只需 `g_robot_control.enable`：设为 1 即开启轮子 LQR、双腿定长支撑和摆角准备，达到准备条件后渐入腿部 LQR；设为 0 同时停止关节和轮子。默认 `startup_enable=1`，`startup_phase=1/2/3` 分别是腿部准备、腿部 LQR 渐入和正常平衡；准备和渐入期间扶稳机身，进入 3 后再逐渐减少扶持。启动仍默认关闭，支撑前馈默认每腿 9.0 N，腿长目标 0.07 m。`status=1` 仅表示指令已入队，不表示已准备好；2/3/4/5 分别是反馈、参数、发送失败和编译时禁止发送，异常会清掉总开关，反馈恢复也不会自动重启；手动重新设 1 才重启。在整机模式中手动清掉 dual 或 balance 的 enable 也会停止整机。轮子单测或零点采样具有优先权，会关闭整机总开关，status=6；不要同时开启两个测试。原 dual/balance 开关仍可用于分项排查，未使用整机总开关时保持原来的行为。参数继续在原 Watch 对象内调整，无需分别打开腿和轮子。主机验证单开关同时输出/归零、异常闭锁、分项停止和单测独占；重新烧录并加载同次 AXF 后使用新变量，实机组合运行尚未验证。

参数集中在 `App/Tasks/Inc/wheel_leg_config.h`。实测机头下俯时原始 roll 减小，已选 `BODY_PITCH_AXIS=0`，观察量 `body_pitch_rad=-roll×π/180`，模型 `phi=-body_pitch_rad`。轮半径为 0.026 m，左轮方向 -1、右轮 +1。完整 LQR 当前启动 `wheel_scale=0.8`、`leg_scale=0.9`、轮电压限幅 6000 mV、轮补偿默认关闭；轮子单测为 scale=1、6000 mV 上限和原来的中心补偿逻辑。轮电压已移植 ESP32 的左右轮独立力矩换算和 2804 反电动势补偿，默认供电 12 V，轮电压硬上限 6000 mV。四个关节也已移植 ESP32 的独立力矩换算和 4010 反电动势补偿；当前承重参数将关节限压设为本地 F103 硬上限 6900 mV，关节补偿仍关闭。当前轮位移按直驱且忽略定子随连杆转动估算，俯仰角速度采用平面运动近似。驱动接收 mV，LQR 输出为 N·m，两者不能直接等同。无力或零输出的排查见 `docs/control_code_map.txt`。

腿部输出：先关闭 `g_wheel_test.enable`；`g_dual_hold.enable=1` 开启双腿腿长保持，另设 `g_balance.enable=1` 加入抗倾斜的前后摆腿力矩和轮子 LQR 输出。关节系数在 `g_dual_hold.torque_ratio_nm_per_v[4]`，按 LF/LR/RF/RR 排列，默认 0.0316/0.0317/0.0299/0.0321 N·m/V；12 V 配置下，同样请求 0.1 N·m，静止关节的限幅前请求约为 2.22/2.21/2.34/2.18 V，最终电压受 6900 mV 上限约束。`g_joint_output` 显示力矩电压、补偿电压、限幅前请求、最终电压及饱和标志。轮子和关节共用 `g_wheel_output.supply_voltage_v` 配置的供电模型，尚未接入电池 ADC。LQR 保留当前 MATLAB 增益，左右腿角度同步串级控制已补入，见下述 2026-10-05 更新。

左右腿前后开叉同步控制（2026-10-05）：共享 LQR 只读取左右均值，不能单独纠正一前一后的摆角偏差。新增参考 ESP32 `legAnglePID` 的角度差/角速度差串级控制，外环比例 12、内环比例 0.04，目标相对角速度上限 20 rad/s，外环误差滤波时间常数 0.004 s；5 ms 本地采样采用按时间计算的滤波系数。输出按每侧腿长/0.07 m 缩放，并按本工程 X 向前、Y 向下及 Jacobian 转换纠偏符号；左侧角度较大时，左侧模型摆角力矩为负、右侧为正。未直接照搬 ESP32 VMC 的输出符号或其 1 N·m 上限。初版 ratio=1 的同步分量每腿限幅 0.1 N·m，并以 0.2 N·m/s 渐变，首拍最多 0.001 N·m；当前小力度设置见顶部更新；最终仍受关节 6900 mV 限压。

同步当前默认 `g_dual_hold.sync_ratio=0.30`，强度为初版的 30%，额外同步力矩每腿最多 0.03 N·m，以 0.05 N·m/s 渐变。初版默认 1 曾因实机站不稳撤回为 0，现按用户要求尝试小力度纠偏。同步不依赖 wheel_scale、leg_scale、leg_hold_ratio 或局部参考模式。`sync_angle_difference_rad`、`sync_speed_difference_rad_s` 和 `sync_torque_nm[2]` 分别显示左减右摆角、角速度及实际同步分量；`pitch_torque_nm` 是局部摆角保持、LQR 和同步三项之和。停机、反馈故障、参数错误、CAN 发送失败和单测接管均清掉同步状态。

主机测试覆盖左右开叉的两个方向、不同腿长、角速度阻尼、输出渐变、整机模式、轮子/IMU 无效但输出均为零的定长模式、参数/反馈/CAN 故障及单测独占。对实际编码的 CAN 电压解码并经 Jacobian 还原摆角力矩，在固定腿长的理想角惯量/阻尼模型中，3 秒后摆角差由无同步时约 0.24 rad 降至约 0.000033 rad；该模型不包含真实摩擦、地面约束或完整机器人动力学。用户随后反馈开启同步后站不稳，该实机结果优先于理想模型，当时关闭了同步默认值；先改为 10% 强度再试，收到反馈后提高到 25%，随后小幅提高到 30%。测试保留 ratio=0 的撤回行为和手动 ratio=1 的算法检查，并增加小力度默认的方向、限幅、渐变与故障检查。上述理想模型数值对应强同步，不代表当前小力度设置的实机恢复速度。此前不稳根因尚未确认。

关闭同步作对照：扶稳机身，先设 `g_robot_control.enable=0`，将 `g_dual_hold.sync_ratio=0` 后再开启总开关。当前新固件重启默认恢复 0.30；早期固件重启默认可能是 1 或 0，应以同次 AXF 的 Watch 值为准。小力度默认烧录本次 H723 HEX 并加载对应 AXF，F103 无需更新。总开关若自动变 0，读取 last_fault。实机稳定性仍待验证。

启动准备后渐入平衡（2026-10-05）：用户确认两侧腿角会向 90° 靠近，但还未对齐就失稳，因此本次针对过早接入共享 LQR 的启动时序进行修改，并非确认了实机全部失稳根因。整机 `startup_enable=1` 时先进入 `ROBOT_STARTUP_PREPARE`，轮子 LQR 立即输出，LQR 摆腿力矩和轮补偿为零；腿长控制、8.5 N 前馈渐升以及原 0.1/0.01、0.02 N·m 局部摆角保持仍运行。准备期两侧参考从当前角度逐步移向共同目标，旧 capture 模式在准备/渐入期间也临时走共同目标；若后续配置 `leg_hold_ratio=0`，局部保持会在平衡渐入期间逐渐退出。初版强同步保持关闭；当前仅启用顶部说明的小力度同步。

准备放行条件：模型中的前后差 `abs(wheel_forward_gap_m)<=0.003 m`；每腿距当前腿长目标不超过 0.01 m，且左右腿长差不超过 0.01 m；各腿距共同摆角目标不超过 5°，机身俯仰绝对值不超过 5°；腿长速度不超过 0.02 m/s、摆角速度不超过 0.2 rad/s、机身俯仰角速度不超过 0.1 rad/s；不再要求轮子先停，轮反馈仍须有效；支撑前馈距目标不超过 0.1 N。完整 LQR 所需反馈仍须有效，上述条件连续满足 0.2 s 后进入 `ROBOT_STARTUP_RAMP`，腿部 LQR 及轮补偿以同一个 `startup_blend` 在 0.5 s 内从 0 到配置值；轮子 LQR 始终按配置的 wheel_scale 输出。轮位移参考从第一次有效控制开始保持，准备时的轮子移动会形成位置反馈，准备/渐入切换不重新捕获。进入 `ROBOT_STARTUP_BALANCE` 后不再因普通平衡运动切回静态准备，避免频繁接入/撤出轮控制。

Watch：`startup_phase` 为枚举，0 停机、1 准备、2 渐入、3 平衡；`startup_blend` 为 0～1；`startup_elapsed_s` 显示启动经过时间，`startup_settle_s` 显示连续满足条件时间。`wheel_forward_m[2]` 为每侧 `length*cos(angle)`，`wheel_forward_gap_m` 为左减右，仅在总开关开启且腿反馈有效时解读。`startup_ready_mask` 的 1/2/4/8/16 位分别表示前后差、腿长、姿态、速度、前馈到位；31 表示全部到位。若一直停在准备，继续扶稳并查看哪个条件未满足，不会按固定延时强行进入平衡。没有新增触地或手扶检测，准备完成不等于实机稳定保证。暂停后设 `startup_enable=0` 可恢复旧的立即接入 LQR 行为，用于明确对照；再设为 1 并重新开关才能重新执行准备。三项 LQR/轮补偿全部为零的纯定长模式继续绕过准备和无用传感器依赖；运行中从纯定长恢复 LQR 保留原直接恢复语义，需要重新开关总开关才执行本启动流程。

测试覆盖实测约 95 mm 错位时轮子 LQR 立即工作而腿部 LQR 和轮补偿仍受门槛控制、零轮速倾斜首拍非零轮指令、可选准备阶段正反滚动阻尼及限幅、腿姿参考渐移、前馈等待、短暂对齐不放行、腿长/机身倾斜/运动分别阻止放行、连续准备计时、位移参考持续保持、可选阻尼退出及腿部 LQR 和轮补偿同步渐入、局部保持的平滑退出、正常平衡不反复准备、故障/手动退出/单测独占及纯定长/旧流程对照。主机测试使用固定或注入的姿态，证明输出和阶段逻辑，不证明真实腿已恢复对齐。烧录新 H723 HEX 并加载同次 AXF，F103 不用更新；双轮接地、扶稳但不要向上提机身，开启总开关，待 `startup_phase=3` 且实机姿态稳定后再逐渐减少扶持。本次尚未实机烧录或验证。

承重测试参数：`target_m=0.07`、`feedforward_target_n=8.5`（每腿 N）、`feedforward_rate_n_s=2`、`max_force_n=15`、`voltage_limit_mv=6900`。用户估计整机约 2 kg，按竖直腿、均匀承重且把全重计入 VMC 的初始估计是每腿约 9.8 N；收到力度偏大的反馈后，当前将前馈降到 8.5 N，约减小 13%。整机质量包含轮腿部件，实际经 VMC 承担的负载及力矩电压标定还需实测。主机计算 12 V 模型下，0.07 m 腿姿提供 9.8 N 需约 5.7～6.0 V，0.11 m 则需约 7.2～7.6 V，超出驱动 6.9 V 上限，所以仍用短腿承重。当前前馈从零以 2 N/s 增加，约 4.25 s 到 8.5 N；运行中降低目标前馈也按此速率渐降。执行仍默认关闭。先双轮接地、扶稳机身，待前馈升起后逐渐放手。当前没有触地检测，抬离地之前关闭 dual；承重前馈不能自动识别离地。若腿持续上顶，在 Watch 每次降低 0.5 N；开始下沉则加回 0.5 N，观察腿长是否稳定在目标附近。

腿长控制当前用 `position_kp=5`、`speed_kp=10`、`speed_ki=0`、`max_speed_m_s=0.04`。提高速度比例到 35 后用户反馈刚开启就快速震动且更严重，因此已退回 10；静止速度参考纠偏上限恢复为 0.4 N/腿。支撑前馈仍为 8.5 N，不通过加大前馈处理抖动。`g_dual_hold.force_requested_n[2]` 显示限幅前轴向力，`force_limit_n[2]` 显示当前姿态和电压限制下的纯轴向力上限，`force_saturated[2]` 指示该限幅，`force_n[2]` 显示实际用于 VMC 的力；最终混合摆角力矩后的电压限幅仍看 `g_joint_output.saturated`。需要回到轻载排查时，先关闭 dual，设 `feedforward_target_n=0`、`voltage_limit_mv=800`；此诊断设置不能用于整机承重。补偿仍默认关闭，低通只平滑目标腿长速度，测得的腿长速度立即参与制动。重新烧录并加载同次 AXF 后使用新增 Watch 字段。主机验证前馈渐升、短腿支撑、长腿轴向限幅、退出清零及控制回归，实机稳定性尚未验证。

摆角保持参数仍为 `angle_kp_nm_per_rad=0.1`、`angle_kd_nm_s_per_rad=0.01`、`angle_limit_nm=0.02`。2026-10-04 根据左右腿初始姿态不一致、手扶俯仰不固定的反馈，默认改为 `angle_reference_mode=LEG_ANGLE_NEUTRAL`（值 1），两腿使用同一个相对机身的 90° 目标 `angle_target_rad=1.5707963`。首拍各自沿用当前姿态，随后每侧参考以 `angle_reference_rate_rad_s=0.2`（约 11.5°/s）逐步移到共同目标，避免位置参考突跳；腿长控制和支撑前馈保持原值。balance 切换不重新捕获，PD 仍与 LQR Tp 按 `leg_hold_ratio` 混合，默认 1；设为 0 会关闭这项局部摆角纠偏。`angle_reference_mode=LEG_ANGLE_CAPTURE`（值 0）保留旧的独立启动姿态保持用于对照。`angle_reference_rad[2]` 显示逐渐移动的参考，`angle_error_rad[2]` 显示各自当前误差。该共同目标修复的是启动参考保持不一致，尚未验证实机姿态收敛或自由平衡。

balance 开启后关节归零修复（历史 captured 模式验证）：旧路径撤掉 dual 摆角保持，直接换成 LQR Tp。主机用静止机身、0.11 m 腿长、偏离捕获角 0.1 rad 的反馈复现四关节从 `75/76/-79/-75 mV` 变成全零：此时 LQR Tp 约为 -0.00000143137 N·m，换算不足 1 mV，被整数指令截成零。当前 `g_balance.leg_hold_ratio=1` 默认保留摆角保持并叠加 LQR Tp；设为 0 可恢复纯 LQR 摆腿控制，范围 0～1。这是支撑下诊断设置，并非 ESP32 原始控制律，自由平衡需另行验证。Watch 中 `g_dual_hold.angle_hold_torque_nm[2]` 和 `lqr_pitch_torque_nm[2]` 显示实际施加的两项，之和为 `pitch_torque_nm[2]`；`force_n[2]` 显示腿长轴向力。新 AXF 的结构布局已改变，烧录新 HEX 后须重新加载同次 AXF。主机回归验证保持连续、比例混合、纯 LQR、腿长输出和异常归零；尚未读取实机 RAM 或验证实机动作。

数据不刷新排查：先确认 Keil 已 Run，并开启 View → Periodic Window Update，查看 `g_wheel_leg.updates` 是否持续增加。若仍停住，暂停后查看 PC/调用栈是否进入 HardFault_Handler、Error_Handler 或 FreeRTOS 断言。ARM 链接器报告轮腿任务的控制调用链约 1.9 KB；原任务只分配 512 个 word（2048 字节），浮点上下文保存还需要额外空间。当前改为 1024 个 word（4096 字节），FreeRTOS 堆从 15 KB 增至 20 KB 以容纳任务栈和原有队列。`g_wheel_leg.stack_free_min_bytes` 显示该任务历史最小剩余栈字节数，任务首次采样前为 0。此修改消除了已发现的栈余量风险；尚未取得实机故障现场，不能据此确认本次停住由栈溢出造成。

完整 LQR 轮抖排查：实机反馈是轮子单测正常、接地且机身有支撑时开启完整平衡后轮子抖。`g_balance.back_emf_ratio`（0～1，默认 0）单独控制完整平衡的轮补偿；`g_dual_hold.back_emf_ratio` 仍只管关节。`g_balance.wheel_torque_component_nm[6]` 显示 theta、dtheta、x、dx、phi、dphi 各项对轮力矩的贡献，六项之和加 `startup_wheel_damping_torque_nm` 为 `torque_nm[0]`，正常平衡阶段额外阻尼为零。先在补偿为零时观察是否仍抖，以及速度项 `[1]/[3]/[5]` 是否快速变号，再检查输入量与反馈方向。此版本提供排查手段，尚未确认或解决实机轮抖根因。支撑约束会改变整机动力学；悬空时应使用轮子单测，参考 ESP32 的触地检测在离地时会关闭轮子，当前工程没有触地检测。

左右启动姿态一致性（2026-10-04）：旧版将左右初始虚拟腿摆角分别作为固定保持目标，LQR 又只读取两侧状态均值。主机重放左右相对竖直偏 ±0.12 rad、腿长分别为 0.07/0.09 m、零机身俯仰，旧局部保持输出为零且共享 LQR 几乎看不到反向摆角偏差。新默认中两侧参考会缓慢移到同一个相对机身 90° 的目标，纠偏力矩分别向相应目标，继续使用原 0.02 N·m 上限和角速度阻尼。±5° 机身俯仰和 balance 切换均不重捕获两个不同参考；机身俯仰仍由 IMU 输入 LQR，未将手扶姿态自动采成零位。主机检查参考移动速率、方向、目标修改渐变、长度支撑独立性、异常参数停机和退出清零通过；固定反馈重放只证明参考/输出行为，不表示实机腿已移动到目标。下载新主控 HEX 并加载同次 AXF（Watch 结构已增加字段）；已有 6 V 轮驱动无需重复更新。两轮接地、扶稳开启总开关，等待支撑前馈升到目标且两侧角度/腿长实际稳定后再逐渐减少扶持。

轮电压上限 6 V（2026-10-04）：按用户要求，主控默认 `g_balance.wheel_limit_mv=6000`，参数允许的硬上限 `WHEEL_VOLTAGE_LIMIT_MV=6000`；轮子单测默认上限也为 6000 mV。设置 5000/6000 mV 可正常启用，超过 6000 mV 仍触发参数故障并清零全部输出，恢复参数后需要手动重启总开关。轮比例 0.8、摆腿比例 0.2、每腿支撑 8.5 N 和关节 6900 mV 限压保持原值。左右 F103 构建脚本读取主控配置，生成 6 V 轮配置并在 setTargetVotage 中对 CAN 电压明确限幅到 ±6 V；NaN 归零，原 100 ms 通信超时停机仍保留。驱动母线模型从原 9 V 统一为主控配置的 12 V，实际供电应与该配置匹配；未接电池 ADC，6 V 指令不是实测电压或恒定输出。新固件为 build/keil/MyLegwheel.hex、build/f103_left_wheel/left_wheel.hex（ID3）、build/f103_right_wheel/right_wheel.hex（ID7），三端一起更新并加载对应 AXF。轮驱动更新保留标定/ID 页，不做整片擦除；上电的原轮电角度对齐流程保留。主机验证主控边界、正反 CAN/FOC 限幅及故障归零，三份固件已构建，尚未烧录或实机验证。

单独增加轮子力度（历史 0.4→0.8 调整，下述 3000 mV 已被新的 6000 mV 设置替代）：原来的 `g_balance.scale` 已拆成 `wheel_scale` 和 `leg_scale`，均为 0～1。2026-10-04 根据腿支撑有力、轮子仍无力的反馈，将默认轮比例从 0.4 增至 0.8，轮限压从 1500 增至 3000 mV；腿比例仍为 0.1，支撑前馈、腿长和摆角保持参数保持原值，轮补偿仍为 0。相同状态下，限幅前轮电压请求为原来的两倍；这不是实测力矩翻倍。主机重放 0.11 m 竖直腿、静止、前倾 1°，左右轮电压由约 -574/+546 mV 增至 -1149/+1092 mV；正反 10° 均受 3000 mV 限幅，CAN 编码、关节输出独立性和故障归零回归通过。烧录前扶稳并关闭执行，烧录新 HEX 后加载同次 AXF；或停机时在 Watch 设置 wheel_scale=0.8、wheel_limit_mv=3000、back_emf_ratio=0，再开启总开关。若之前为纯腿调试把 wheel_scale 设为 0，必须恢复非零才有轮力矩。启动仍默认关闭，实机力度和平衡稳定性待验证。若轮指令已达到 3000 mV 但仍无力，检查电源限流及带载电压；此前已有电源限流导致左轮无力的记录。

虚拟杆摆腿力度（2026-10-04）：用户反馈烧录线牵着时稳定、松开后向前倒，并要求增大摆腿力矩。默认 `g_balance.leg_scale` 从 0.1 提到 0.2，使相同状态下的 LQR 摆腿分量翻倍；轮比例 0.8、当时轮限压 3000 mV（现为 6000 mV）、每腿支撑 8.5 N、腿长增益、摆角保持及其混合比例保持原值，关节硬限压仍为 6900 mV。主机重放 0.07 m 竖直腿、静止前倾 5°，LQR Tp 从约 0.002612 增至 0.005224 N·m；该数值不含摆角保持，也不代表实测力矩。0.07/0.11 m 正反倾斜、轮输出/轴向支撑独立性、关节换算/限压和故障停机回归通过。可扶稳停机后在 Watch 设 `g_balance.leg_scale=0.2`，再开启总开关；重启后要保留新默认值则烧录新 HEX 并加载同次 AXF。此调整尚未实机验证，不能确认前倒由摆腿力度引起；若松开时总开关自动归零，先读 last_fault，若出现持续抖动则停机。

单独测试轮子的倾斜响应：烧录新固件后，在 Keil Watch 中添加 `g_wheel_test`，只设置 `g_wheel_test.enable=1` 即可，设为 `0` 停止。此模式只依赖 IMU，以固定 0.11 m 虚拟竖直腿计算 LQR 轮输出，不依赖关节或轮编码器反馈；四个腿电机电压始终为零。它会关闭腿长保持和完整平衡开关，独占 CAN 指令输出。观察 `status`（1 为指令已入队）和 `voltage_mv[2]`、`voltage_mv[6]`（左右轮 mV）。

轮速有效时，自动加入与转向一致的反电动势补偿；轮速缺失或超时，只关闭该侧补偿，倾斜测试仍可输出。`g_wheel_output` 可观察左右轮力矩电压、补偿电压和限幅前请求电压。左右轮力矩/电压比直接沿用 ESP32 的 0.0096、0.0101 N·m/V，供电按 `supply_voltage_v=12` 计算（未接入电池 ADC 自动测量）。轮子单测默认角度死区为 ±0.15°、角速度死区为 ±0.5°/s；两者均在范围内时输出真零，反电动势补偿也归零，离开范围后逐步恢复补偿。零偏为 0 时，默认静止前倾 1° 约输出 1.22/1.16 V，前倾 10° 达到 6 V 限幅。

校准单测零位：把机身保持在你选择的竖直姿态，设置 `g_wheel_test.zero_request=1`，静止保持约 0.5 秒；`zero_status=2` 表示完成，角度零位和陀螺零偏已更新。采集期间所有电机输出为零，移动或 IMU 无效会重新采集。原来已开启单测则完成后继续输出，原来关闭则继续保持关闭。校准值保留到重启，不写入 Flash；完整平衡模式不使用此单测零位和死区。

上电开叉的排查结果：用户确认未开启 enable 时，扶住机身、腿不承重就不动，承重才开叉；这符合停机时没有主动支撑力、重力压开腿的表现。按用户选择，继续保留上电全部停机，未增加自动支撑。启动时双轮接地，扶稳并摆正腿姿，再设置 `g_robot_control.enable=1`；约 4～5 秒内扶住机身，观察 `g_dual_hold.feedforward_n` 升至目标且腿长稳定后逐渐放手。dual 首拍捕获当前摆角以避免位置参考突跳，随后默认逐渐向共同的 90° 目标纠偏；启动前仍应扶稳机身并摆正腿姿。总开关置 0 后失去支撑，需要重新扶住。

四个关节 F103 的构建脚本另已移除每次上电的固定磁场找零和电机开机音，改为仅加载 Flash 中已有的有效电角度零位；此更新解决主动启动动作，不能阻止停机时承重下塌。新关节固件已生成在 `build/f103_left_front/left_front.hex`、`build/f103_left_rear/left_rear.hex`、`build/f103_right_front/right_front.hex`、`build/f103_right_rear/right_rear.hex`，本次没有下载到实机。更新时保留 `0x08007800` 电机标定页和 `0x08007C00` ID 页，不做整片擦除。无有效标定时驱动保持关闭；旧按键长按擦除标定并重启的操作不再能自动找零，需要单独标定流程恢复有效零位。共享原始 F103 工程与轮驱动构建未修改。

当前接地站立默认 `g_dual_hold.support_only=1`：腿长轴向力指令限制在 0 到当前电压/姿态允许的上限之间。高于目标时只减小支撑，最低降为 0，由重力自然压回，不再主动向下拉。`force_requested_n[2]` 可以为负，实际 `force_n[2]` 不会为负；`force_saturated[2]` 同时指示下限和上限限幅。电机电压的正负仍由关节坐标和摆角力矩决定，不等同于轴向下拉。`support_only=0` 保留有符号轴向力用于明确的非承重诊断，不是站立推荐设置；积分在零力下限也做抗饱和，退出仍立即清零。

腿长的小幅震荡处理：速度比例保留 10，支撑前馈保留每腿 8.5 N。`length_error_deadband_m=0.001` 忽略目标附近 1 mm 的位置误差，`length_speed_deadband_m_s=0.003` 忽略 3 mm/s 内的速度微扰；区间外连续减去阈值，保持方向且不额外增加速度滤波延迟。`length_error_control_m[2]`、`length_speed_control_m_s[2]` 显示处理后的控制量，原始反馈保持可观察。卸力刚度 `overlength_kp_n_per_m` 从 300 降至 250，超长阈值仍为目标以上 5 mm；`overlength_transition_m=0.002` 让卸力项在随后 2 mm 内平滑介入，再接上较软的线性段。位置/速度死区和过渡宽度都可设 0 恢复对应原路径。主机注入 0.2 mm 位置微扰、2 mm/s 速度微扰，腿长力指令的峰峰波动从 0.042 N 降至 0，支撑仍为 8.5 N；这不表示真实电机电压零噪声或实机震荡已解决。

`overlength_startup_s=0.5` 仍将卸力项在启用后的半秒内从零逐渐增加，`overlength_blend` 显示进度，退出时清零；设 startup 为 0 可绕过渐入用于对照。它不改变支撑前馈原有的 2 N/s 渐升，切换 balance 不重新开始渐入。主机重放目标 0.07 m、实际 0.11 m、零速度、全支撑前馈，限幅前总力请求约 -0.4 N，站立模式实际轴向力为 0。首拍负请求也同样归零，不再主动缩腿。此项不替代触地检测或保证严格定长；仍需接地、扶稳启动。此前 300/35 参数在理想质量模型中表现更好，但实机震动更严重；该历史对照继续保留在测试中，实际默认为 265/10，且不允许负轴向力。

速度核对：`g_wheel_leg.length_speed_m_s[2]` 是经 Jacobian 换算的 CAN 速度，`length_speed_from_position_m_s[2]` 是连续有效腿长样本的有限差分；后者只用于观察，未替代控制速度，且受位置量化影响。`length_speed_position_valid[2]` 为 1 才表示差分有效。停机扶住机身，缓慢压短一侧腿时，该侧两项应总体为负，缓慢拉长应总体为正。F103 源码中的速度来自滤波角度的 5 ms 差分，位置反馈则是实时轴角；滤波滞后、速度噪声或方向不一致都需要实测核对，不能只继续提高速度增益。

用户确认失效时总开关自动归零，说明控制触发了停机。`g_robot_control.last_fault` 现在保留停机瞬间的 `reason`、`leg_length_m[2]`、`leg_valid[2]`、`feedback_detail`、`motor_id`、`feedback_ms`、`dual_status`、`balance_status`、`period_ms`、`tick_ms` 和 `count`；后续反馈恢复或手动停机不会覆盖，下一次控制故障或重启才改变。reason：1 腿长超出 LQR 的 0.04～0.14 m 范围，2 反馈异常，3 控制周期异常，4 几何异常，5 参数异常，6 CAN 发送失败，7 发送未启用，8 其他状态异常。实际腿长只在相应 `leg_valid=1` 时有意义。不能仅凭升高就认定此次实机故障是超范围；重新烧录新 H723 HEX、加载同次 AXF 后，看该记录确认。停机不自动重启，停机后失去支撑也可能被重力压开；总开关仍默认关闭。

分步调试仍使用同一个总开关：先停机，把 `g_balance.wheel_scale=0`、`leg_scale=0`、`back_emf_ratio=0`，再开 `g_robot_control.enable=1`。三个都为 0 时自动使用纯定长/摆角保持路径，轮指令和 LQR 摆腿输出归零，支撑参数保持不变；此路径不使用轮子和 IMU 状态，因此其反馈异常不会单独触发双腿停机。四关节反馈、5 ms 控制周期、电压/参数、几何和原 0.04～0.14 m 每腿范围检查仍有效。任何 LQR 比例或轮补偿恢复非零时，完整轮/IMU 反馈检查重新生效；重新进入 LQR 时捕获当前轮位移参考，保持腿长和摆角控制状态。先在双轮接地、手只防倒而不向上提机身的条件下调稳腿长，再逐步恢复轮子比例，例如 0.1、0.2；腿 LQR 比例暂时保持 0。每次改参数前扶住并停机，观察是否持续升高、下沉或触发 last_fault；不能以增大前馈代替腿长控制。

稳定一段时间后倒下：用户确认总开关自动变 0，并描述腿慢慢伸长或开叉，尚未读到具体停机原因，也未确认是否已经恢复轮比例。本次按之前比例均为 0 的测试设置检查，主机复现了旧完整路径因未使用的轮/IMU 反馈失效而停止双腿的问题，已按上述纯定长路径修复；腿部满意的增益、前馈和卸力参数未调整。这是一个已复现的代码耦合问题，尚不能认定它就是此次实机倒下的根因。`last_fault.reason` 改为枚举，加载同次 AXF 后调试器可显示 `ROBOT_FAULT_LENGTH_RANGE`、`ROBOT_FAULT_FEEDBACK` 等名称。再次停机时先确认原因和停机时两腿长度；停机后失去支撑也会导致腿被重力压开，不能把该动作直接当成停机前的原因。

## 目录与职责

| 目录 | 内容 |
| --- | --- |
| `Core/Inc`、`Core/Src` | 入口、时钟、系统、异常中断和 HAL 时基；沿用 STM32 生成文件命名 |
| `App/Tasks/Inc`、`App/Tasks/Src` | IMU、CAN 接收、5 ms 轮腿观察任务与集中配置 |
| `App/Services/Inc`、`App/Services/Src` | 电机指令发送、关节标定、灯光蜂鸣器和 SBUS 接收服务 |
| `App/Protocol/Inc`、`App/Protocol/Src` | 匿名上位机 ANO V7 与 SBUS 流式解码 |
| `BSP/Peripherals/Inc`、`BSP/Peripherals/Src` | SPI、定时器、FDCAN 和 UART5 SBUS 板级初始化 |
| `BSP/BMI088/Inc`、`BSP/BMI088/Src` | BMI088 寄存器、传感器驱动和平台适配 |
| `BSP/DWT/Inc`、`BSP/DWT/Src` | Cortex-M DWT 计时工具 |
| `Algorithms/Attitude/Inc`、`Algorithms/Attitude/Src` | Mahony 姿态算法 |
| `Algorithms/PID/Inc`、`Algorithms/PID/Src` | IMU 温控 PID |
| `Control/WheelLeg/Inc`、`Control/WheelLeg/Src` | 当前固件共用的腿部几何/雅可比与 LQR |
| `tests/support/wheel_leg` | 主机测试用轨迹、独立计算示例和旧 MATLAB 兼容包装，不参与固件 |
| `USB_DEVICE` | USB 设备应用与 HAL 适配，保留 STM32 目录惯例 |
| `Drivers`、`Middlewares` | HAL、CMSIS、USB 协议栈和 FreeRTOS 依赖，保留上游结构 |
| `MDK-ARM` | Keil 工程、调试选项、启动汇编、链接配置和 RTE 配置 |
| `scripts`、`tests`、`docs` | 构建与结构检查脚本、主机测试、使用与移植说明 |
| `build` | 自动生成的固件、目标文件、日志和链接映射 |
| `backups` | 整理前的本地源码快照、旧输出与迁移记录 |

## 命名与放置规则

自定义 C 源文件和头文件采用小写下划线命名，例如 `imu_task.c`、`bmi088_driver.h`；同一模块的 `.c` 放在 `Src`，`.h` 放在 `Inc`。头文件保护宏采用大写下划线、以 `_H` 结束，例如 `IMU_TASK_H`。平台外围初始化文件采用 `bsp_` 前缀。对外函数接口保持现有名称，STM32 生成文件与第三方文件保持各自命名约定。

新增模块时同时更新 Keil 的文件分组和头文件搜索路径。目标文件、固件和临时数据只放在 `build`，不放入源码目录。`MyLegwheel.ioc` 保持根目录位置。该 `.ioc` 目前只记录基础系统配置，并非全部手写外设和应用配置的完整描述；后续使用 CubeMX 重新生成时，需要核对应用初始化调用、工程分组、搜索路径和链接配置。

## 编译

在 Keil 打开 `MDK-ARM/MyLegwheel.uvprojx`，选择 `MyLegwheel` 目标进行 Rebuild。输出固件为 `build/keil/MyLegwheel.hex`，映射文件放在 `build/keil/listings`。

也可以使用 Python 3.9 或更高版本，通过命令行完整编译：

```powershell
python scripts/build_keil.py --keil-root D:/app/Keil_v5
```

脚本使用 Keil 自带 ARM Compiler 6 和工程指定的 ARM CMSIS 5.7.0 pack，重新编译全部工程及 RTE DSP 编译单元，不复用旧目标文件。工具安装路径可通过 `--keil-root` 或 `KEIL_ROOT` 指定；省略时尝试常见安装位置。构建日志为 `build/keil/build.log`。

`MDK-ARM/MyLegwheel.sct` 是工程使用的链接文件：普通数据和栈放在 DTCM，WS2812B 的 `.led_dma` 与 SBUS 的 `.sbus_dma` 缓冲区放在 AXI SRAM。不要改回把所有数据都放在 DTCM 的自动链接布局，否则 DMA1 无法访问这些缓冲区。

## 检查与测试

```powershell
python scripts/check_project_layout.py
python tests/verify_board_feedback.py
python tests/verify_usb_imu.py
python tests/verify_imu_telemetry.py
python tests/verify_sbus.py
python tests/verify_sbus_sticks.py
python tests/verify_can_feedback.py
python tests/verify_wheel_leg_control.py
python tests/verify_lqr_kernel.py
python tests/verify_lqr_regeneration.py
python tests/verify_foc_scalar_snapshot.py
python tests/verify_wheel_leg_runtime.py
python tests/verify_f103_joint_boot.py
python tests/verify_f103_wheel_voltage.py
python tests/verify_leg_trajectory.py
python tests/verify_wheel_leg_math.py --source D:/MATLAB/Robot/WHEEL_LEG/MyProc
```

ARM 浮点回归使用真实生产编译参数，在 Cortex-M7 指令仿真中对照优化前内核；依赖只需安装到本地构建目录：

```powershell
python -m pip install --target build/arm-check-deps unicorn pyelftools
python tests/verify_lqr_arm.py --deps-dir build/arm-check-deps --require-bitwise
```

结构检查使用 Python 标准库。主机逻辑测试需要 PATH 中的主机 `gcc`，轮腿测试还需要对应 MATLAB 源文件。主机测试与编译不能替代开发板上的灯光、音效、USB 连接和运动稳定性验证。

VS Code 的 C 配置使用 `arm-none-eabi-gcc` 解析 Cortex-M7 代码；固件构建任务调用 ARM Compiler 6。硬件下载和调试仍通过 Keil 工程执行。编辑器配置不再保留将 MCU 固件当作 Windows 可执行程序启动的调试项。

当前安装的 C/C++ Runner 会在其配置或活动目录变化时重新生成 `launch.json`。它的单文件运行功能不适用于此 MCU 工程；若重新出现旧目录调试项，可通过命令面板中的 `C/C++ Runner: Toggle Extension State` 关闭该扩展的配置生成，使用本工程提供的构建任务与 Keil 调试。

## 使用说明与备份

- USB 数据通道与连接方法：`docs/usb_imu.txt`。
- PD2 SBUS 接线、DMA 接收、通道读取与失控处理：`docs/sbus.txt`。
- 自研 FOC 的 CAN 接线、全局 Watch 变量与关节标定采集：`docs/can_calibration.txt`。
- 轮腿算法接口、单位与验证方法：`docs/wheel_leg_porting.txt`。
- 当前控制入口、参数和整理范围：`docs/control_code_map.txt`。
- 移除未用 PID、分离测试辅助代码前的备份：`backups/before_unused_cleanup_20261005.zip`。
- 移除实验任务前的完整工作版本：`backups/pre_lqr_cleanup_20261002.zip`。
- 本次代码精简前的源码与工程配置：`backups/cleanup_before_20261002_01.zip`。
- 上次目录整理前的源码与工程配置：`backups/layout_before_20261001.zip`。
- 整理前的编译产物：`backups/legacy_keil_20261001`。
- 文件迁移明细：`backups/layout_migration_20261001.json`。

`build`、`backups` 和个人 Keil 窗口状态已列入 `.gitignore`。当前目录没有 Git 仓库，源码快照用于保留这次整理前的文件状态；无需运行迁移记录脚本进行日常构建。
