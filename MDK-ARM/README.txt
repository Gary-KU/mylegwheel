Keil 工程入口（2026-10-07 整理）

打开 MyLegwheel.uvprojx：完整轮腿 LQR 控制工程。
编译输出：../build/keil/MyLegwheel.hex 和 MyLegwheel.axf。
控制入口：../App/Tasks/Src/wheel_leg_task.c。
LQR 系数和计算：../Control/WheelLeg/Src/lqr_k.c。
集中参数：../App/Tasks/Inc/wheel_leg_config.h。

昨天（2026-10-06）最后恢复烧录的完整 LQR 存档：
../build/original_full_ano_20261006/MyLegwheel_original_full_ano.hex
../build/original_full_ano_20261006/MyLegwheel_original_full_ano.axf
当前源码轮比例 0.7；该历史存档记录为 0.8，请区分当前编译和历史镜像。
本次只整理文件，没有修改控制源码、重新编译或烧录。

2026-10-07 后续新增LQR启动/倒下记录：主控Flash最后256KB现已预留给日志，
启动记录固件和流程见 ../docs/lqr_startup_flash_record_20261007.txt。
归档测试项目是旧文件；需要单腿工程时请运行下述生成脚本，包含最新模块注册。

保留文件：
MyLegwheel.uvprojx / .uvoptx：主工程及调试选项。
MyLegwheel.uvguix.Gary：个人窗口布局和 Watch 设置。
startup_stm32h723xx.s / MyLegwheel.sct：启动汇编和链接配置。
RTE / DebugConfig / EventRecorderStub.scvd：Keil 配置。

已归档：左右单腿 PD 测试工程各一对 uvprojx/uvoptx，及旧 .lst 列表。
归档路径：../backups/mdk_arm_cleanup_20261007.zip。
校验清单：../backups/mdk_arm_cleanup_20261007_manifest.json。
需恢复原测试工程时，将 ZIP 内容解压回本目录。
需按当前源码重建测试工程时，在工程根目录运行：
python scripts/create_leg_pd_keil_projects.py
命令行专用构建仍支持：
python scripts/build_keil.py --profile leg-pd-right
python scripts/build_keil.py --profile leg-pd-left

2026-10-09: 已烧录 0827，原地 yaw 有界积分、正常转向保留前后速度补偿。
固件：../build/remote_yaw_pi_0827_20261009/MyLegwheel_remote_yaw_pi_0827.hex
说明：../docs/remote_yaw_pi_0827_20261009.txt

2026-10-09: 已烧录并读回校验 0828，反打减速与跟随增强。
固件：../build/tracking_brake_0828_20261009/MyLegwheel_tracking_brake_0828.hex
说明：../docs/tracking_brake_0828_20261009.txt

2026-10-09: 已烧录并读回校验 0829，独立快速姿态修正、行驶超速纠偏及更强速度补偿。
固件：../build/tracking_transient_0829_20261009/MyLegwheel_tracking_transient_0829.hex
说明：../docs/tracking_transient_0829_20261009.txt

2026-10-09: 已烧录并读回校验 0830，最高前后0.35m/s、yaw0.70rad/s，增强加减速与纠偏。
固件：../build/speed_0830_20261009/MyLegwheel_speed_0830.hex
说明：../docs/speed_0830_20261009.txt

2026-10-09: 已烧录并读回校验0831，0.4m/s、制动过渡、随速度限制yaw，采集READY。
固件：../build/parking_0831_20261009/MyLegwheel_parking_0831.hex

2026-10-10: 已烧录并全量读回校验1001，居中起步零速恢复，前后0.4m/s，原地yaw1.5rad/s。
固件：../build/startup_101001_20261010/MyLegwheel_startup_101001.hex
说明：../docs/startup_101001_20261010.txt
烧录记录：../diagnostics/lqr_remote_101001_20261010_122836.json

2026-10-10: 已烧录并全量读回校验1002，原地yaw2.2rad/s，增强转向PI和前后速度纠偏。
固件：../build/yaw_tracking_1002_20261010/MyLegwheel_yaw_tracking_1002.hex
烧录记录：../diagnostics/lqr_remote_1002_20261010_123624.json

2026-10-10: 已烧录并全量读回校验1003，原地yaw上限3.0rad/s，前后目标0.4m/s。
固件：../build/yaw_tracking_1003_20261010/MyLegwheel_yaw_tracking_1003.hex
烧录记录：../diagnostics/lqr_remote_1003_20261010_124356.json

2026-10-10: 已烧录并全量读回校验1004，高速转向保留快速前后倾角纠偏。
固件：../build/turn_tracking_1004_20261010/MyLegwheel_turn_tracking_1004.hex
烧录记录：../diagnostics/lqr_remote_1004_20261010_125135.json
最终核对时用户已开始测试，采集RUNNING/count571，未重新准备采集。

2026-10-10: 已烧录并全量读回校验1005，超过0.6m/s仍保留有界快速倾角纠偏。
固件：../build/overspeed_1005_20261010/MyLegwheel_overspeed_1005.hex
烧录记录：../diagnostics/lqr_remote_1005_20261010_130000.json
最终核对时采集RUNNING/count477，保留用户已开始的测试。

2026-10-10: 已烧录并全量读回校验1006，低速松杆漂移满足条件2秒后恢复缓慢重心补偿。
固件：../build/coast_1006_20261010/MyLegwheel_coast_1006.hex
烧录记录：../diagnostics/lqr_remote_1006_20261010_130642.json
最终核对时采集RUNNING/count42，保留用户已开始的测试。

2026-10-10: 已烧录并全量读回校验1007，增强有界快速倾角响应，RAM故障前记录扩展为132字节/行。
固件：../build/response_1007_20261010/MyLegwheel_response_1007.hex
烧录记录：../diagnostics/lqr_remote_1007_20261010_212049.json
最终采集READY/count0；新RAM记录请用scripts/decode_balance_trace_1007.py解码。

2026-10-10: 已烧录并全量读回校验1008，遥控共同轮力矩0.012Nm起步，2秒升至0.030Nm并持续限幅；yaw差动力矩独立。
固件：../build/output_1008_20261010/MyLegwheel_output_1008.hex
烧录记录：../diagnostics/lqr_remote_1008_20261010_213612.json
最终采集READY/count0。左轮异常尚待排查。

2026-10-10: 已烧录并全量读回校验1009，启动0.5秒临时力矩渐入，之后恢复正常平衡输出；新增无指令高速yaw保护。
固件：../build/startup_1009_20261010/MyLegwheel_startup_1009.hex
烧录记录：../diagnostics/lqr_remote_1009_20261010_214514.json
最终采集READY/count0。

2026-10-10: Rolled back board to ORIGINAL frozen 1005 HEX; full firmware readback matched.
Firmware: ../build/overspeed_1005_20261010/MyLegwheel_overspeed_1005.hex
Deployment: ../diagnostics/lqr_rollback_1005_20261010_215229.json
Workspace sources still 1009: rebuilding/downloading current project will NOT preserve this rollback.
User started tests during verification; new Flash record saved and CRC verified; no rearm/reset after test began.

2026-10-10: Built 1010 startup leg-reference correction; NOT FLASHED. Board remains original 1005.
Candidate: ../build/startup_reference_1010_20261010/MyLegwheel_startup_reference_1010.hex
Details: ../docs/startup_reference_1010_20261010.txt

2026-10-11: 1010 flashed and full image verified. User performed two starts during verification.
Flash generation39 first start, RAM second start: both stopped by fault11.
Data: ../diagnostics/spin_review_20261011_094400/
Old Flash record backed up before programming. Recorder is saved, not armed for another trial.

2026-10-11: 1011 left feedback sign trial compiled, NOT FLASHED; board remains 1010.
Left feedback +1; left voltage -1 unchanged.
Candidate: ../build/left_feedback_1011_20261011/MyLegwheel_left_feedback_1011.hex

2026-10-11: 1011 flashed; complete readback verified, previous Flash retained, logger READY/count0, CH5 OFF and all outputs zero.
Report: ../diagnostics/lqr_remote_1011_20261011_094924.json
Feedback sign is an experimental hypothesis; historical firmware also used -1, so root cause remains unconfirmed.

2026-10-11: Board reverted to original 0823 for controlled standing comparison. Full firmware readback passed; old record preserved; final logger READY/count0, CH5 OFF, all outputs zero.
Firmware: ../build/parking_lean_entry_20261008/MyLegwheel_parking_lean_entry.hex
Report: ../diagnostics/lqr_baseline_0823_20261011_095506.json
Workspace source remains 1011; do NOT rebuild/download workspace to reproduce this 0823 baseline. No joystick driving/yaw in 0823. RAM trace is old 64-byte layout, Flash V3.

2026-10-11: User found broken left-wheel wire. Pause control tuning; board remains original 0823 for post-repair comparison. Workspace 1011 feedback-sign flip is unvalidated and should not be treated as a fix. See ../diagnostics/left_wheel_wire_discovery_20261011.json

2026-10-11: User repaired left motor PHASE wire. Original 1005 remote firmware restored and complete image readback matched. Forward 0.4m/s, yaw 3rad/s; original left feedback -1 and voltage -1.
Report: ../diagnostics/lqr_rollback_1005_20261011_100412.json
Logger armed successfully; user started during final verification, final RUNNING/count25. Do not reset/rearm this trial.
Workspace source remains experimental 1011; rebuilding/downloading the current project does NOT reproduce 1005. Use frozen ../build/overspeed_1005_20261010/MyLegwheel_overspeed_1005.hex

2026-10-11: Workspace restored to original 1005 baseline and modified as 1012: left torque-voltage component +5% (0.0096/1.05 Nm/V command parameter), original feedback -1, right unchanged. Four regression suites passed, 89-source build passed. NOT FLASHED; board remains original 1005.
Candidate: ../build/left_output_1012_20261011/MyLegwheel_left_output_1012.hex
Manifest: ../build/left_output_1012_20261011/manifest.json
The previous experimental 1011 source is preserved in ../backups/before_left_output_1012_20261011_101603.

2026-10-11: 1012 left torque-voltage +5% firmware flashed, complete image readback matched, old Flash preserved. Final CH5 OFF, all outputs zero, recorder READY/count0. Workspace source matches 1012.
Report: ../diagnostics/lqr_left_output_1012_20261011_102417.json

2026-10-11: Built 1013, NOT FLASHED. Stronger established-motion velocity damping0.120/0.030/0.100, original startup0.075/0.016/0.060 preserved; conditional braking split sync slew doubled. Left +5%, yaw3 and forward0.4, K and stop guards unchanged. Four suites and 89-source build passed. Board remains1012.
Candidate: ../build/tracking_1013_20261011/MyLegwheel_tracking_1013.hex
Details: ../docs/tracking_1013_20261011.txt

2026-10-11: 1013 flashed, complete image readback matched; prior Flash preserved. Final recorder READY/count0, CH5 OFF and all outputs zero. Workspace matches1013.
Report: ../diagnostics/lqr_tracking_1013_20261011_103958.json

2026-10-11: Built1014, NOT FLASHED. Established remote tracking/braking uses25% original wheel velocity component; other wheel terms and leg LQR unchanged. Startup recovery/parking retain original component. Effective remote velocity gain changes; K table unchanged. 89-source build and four suites passed. Board remains1013.
Candidate: ../build/tracking_1014_20261011/MyLegwheel_tracking_1014.hex
Details: ../docs/tracking_1014_20261011.txt

2026-10-11: 1014 flashed and complete image readback matched. Old record preserved on PC. User trial occurred and was saved during final verification; backed up and CRC verified. Final logger SAVED/count163; no further reset/rearm. Workspace matches1014.
Report: ../diagnostics/lqr_tracking_1014_20261011_105329.json

2026-10-11: Reverted failed1014 allocation to original1013 firmware/source. Full board image verified; recompiled default Keil HEX/AXF exactly match frozen1013 hashes. Original wheel velocity term restored. User started another trial during verification; new Flash saved/count170, CRC verified and backed up; no subsequent rearm/reset.
Report: ../diagnostics/lqr_rollback_1013_20261011_105840.json

2026-10-11: Built1015, NOT FLASHED. Neutral startup restores fixed parking after ramp; moving zero-speed recovery opt-in/off by default. Full LQR terms preserved, no1014 scale. Five suites including both recovery modes and89-source build passed. Board remains1013.
Candidate: ../build/startup_1015_20261011/MyLegwheel_startup_1015.hex
Details: ../docs/startup_1015_20261011.txt
