#ifndef WHEEL_LEG_CONFIG_H
#define WHEEL_LEG_CONFIG_H

/* Dedicated bench images only. Standard builds retain full-control defaults. */
#ifndef ROBOT_LEG_PD_TEST_SIDE
#define ROBOT_LEG_PD_TEST_SIDE (-1)
#endif
#if ROBOT_LEG_PD_TEST_SIDE < -1 || ROBOT_LEG_PD_TEST_SIDE > 1
#error Invalid ROBOT_LEG_PD_TEST_SIDE
#endif
#ifndef ROBOT_LEG_VOLTAGE_PROBE
#define ROBOT_LEG_VOLTAGE_PROBE 0
#endif
#if ROBOT_LEG_VOLTAGE_PROBE && ROBOT_LEG_PD_TEST_SIDE < 0
#error Voltage probe requires an isolated leg side
#endif
#define LEG_VOLTAGE_PROBE_PEAK_MV                2000.0f
#define LEG_VOLTAGE_PROBE_MAX_TP_NM              0.60f
#define LEG_VOLTAGE_PROBE_DURATION_MS            2000U
#define LEG_VOLTAGE_PROBE_RUNNING_SPEED_RAD_S    4.0f

/* Isolated, unloaded angle-PD diagnostics. The earlier 0.04/0.004/0.005 Nm
 * preset sent only 26/29 mV in generation13 despite its 200 mV ceiling.
 * Increase proportional authority and torque budget together; no stiction
 * kick, support, sync, wheel owner, or change to full-control parameters. */
#define LEG_PD_TEST_ANGLE_KP_NM_PER_RAD           0.30f
#define LEG_PD_TEST_ANGLE_KD_NM_S_PER_RAD         0.012f
#define LEG_PD_TEST_ANGLE_LIMIT_NM               0.050f
#define LEG_PD_TEST_VOLTAGE_LIMIT_MV             600.0f
#define LEG_PD_TEST_REFERENCE_RATE_RAD_S         0.10f

/* Standard firmware uses the native six-state LQR. Historical control
 * regressions can explicitly select 0; isolated diagnostics retain theirs. */
#ifndef ROBOT_NATIVE_LQR
#if ROBOT_LEG_PD_TEST_SIDE >= 0 || (defined(ROBOT_LEGS_ONLY_CALIBRATION) && ROBOT_LEGS_ONLY_CALIBRATION)
#define ROBOT_NATIVE_LQR 0
#else
#define ROBOT_NATIVE_LQR 1
#endif
#endif

/* 调参默认值：修改后重新编译 H7；Watch 对应项见 docs/tuning.md。
 * 这些宏提供上电初值、固定边界和编译期开关；可调项运行时读取 Watch 对应字段。
 * 运行时修改 Watch 不会写回源码或 Flash，重新上电后恢复源码中的初值。 */

/* 1. 腿长与支撑：力 N，长度 m，速度 m/s。
 * TARGET 是虚拟腿目标长度；它与实际结构/编码器零点不匹配时，可能持续触发卸力。
 * SUPPORT_FORCE 是每腿轴向前馈，不是最终输出，也不是电源显示的输入电流。
 * 近竖直时1.6kg约需每腿7.85N；腿倾斜、动态运动与传动损耗会改变需求。 */
#define LEG_DEFAULT_TARGET_M                     0.07f
#define LEG_DEFAULT_SUPPORT_FORCE_N              9.0f /* 1.6kg: 7.85N/leg static plus 15% rise/friction reserve. */
#if ROBOT_NATIVE_LQR
#define LEG_DEFAULT_SUPPORT_RATE_N_S             24.0f /* 9N support established in 0.375s. */
#else
#define LEG_DEFAULT_SUPPORT_RATE_N_S             2.0f
#endif
#define LEG_DEFAULT_MAX_FORCE_N                  15.0f
#define LEG_DEFAULT_SUPPORT_ONLY                 1U
#define LEG_DEFAULT_POSITION_KP                  5.0f
#if ROBOT_NATIVE_LQR
/* 速度PI输出轴向力：KP单位N*s/m，KI单位N/m（积分时乘dt）。
 * 旧KP10/KI0只能静态卸0.4N，无法消除9N前馈与真实承重的差。
 * 积分只学习有界力偏置，不代替姿态LQR，也不突破电流/力矩限幅。 */
#define LEG_DEFAULT_SPEED_KP                     30.0f
#define LEG_DEFAULT_SPEED_KI                     80.0f
#else
#define LEG_DEFAULT_SPEED_KP                     10.0f
#define LEG_DEFAULT_SPEED_KI                     0.0f
#endif
#define LEG_HEIGHT_INTEGRAL_LIMIT_N              4.5f
#define LEG_HEIGHT_INTEGRAL_RATE_N_S              3.0f
#define LEG_HEIGHT_INTEGRAL_SUPPORT_READY_N      0.05f
#define LEG_DEFAULT_MAX_SPEED_M_S                0.04f
#define LEG_DEFAULT_FILTER_TAU_S                 0.02f
#define LEG_DEFAULT_LENGTH_ERROR_DEADBAND_M      0.001f
#define LEG_DEFAULT_LENGTH_SPEED_DEADBAND_M_S    0.003f
/* 可选过长卸力：KP=0关闭，KP>0时在超过target+margin后扣减轴向支撑。
 * native默认关闭：此前103/98mm腿长使9N前馈被扣到约1.4/2.65N，妨碍承重。
 * 关闭本项仍保留腿长速度反馈、15N轴向上限和逐关节电流/力矩限幅。
 * 若台架需要恢复软收腿，可在Watch给g_dual_hold.overlength_kp_n_per_m赋正值；
 * 旧值为265N/m。MARGIN调大更晚卸力，TRANSITION平滑阈值，STARTUP只控制渐入。 */
#if ROBOT_NATIVE_LQR
#define LEG_DEFAULT_OVERLENGTH_KP                 0.0f
#else
#define LEG_DEFAULT_OVERLENGTH_KP                 265.0f
#endif
#define LEG_DEFAULT_OVERLENGTH_MARGIN_M           0.005f
#define LEG_DEFAULT_OVERLENGTH_TRANSITION_M       0.002f
#define LEG_DEFAULT_OVERLENGTH_STARTUP_S          1.0f

/* 慢积分承重补偿：供电/限流尚未确认，默认关闭。只用于接地站立。
 * 腿长误差积分为附加力，每腿最多 +/-2 N，变化不超过 0.25 N/s。 */
#define LEG_DEFAULT_LOAD_ENABLE                  0U
#define LEG_DEFAULT_LOAD_KI_N_PER_M_S             20.0f
#define LEG_DEFAULT_LOAD_LIMIT_N                 2.0f
#define LEG_DEFAULT_LOAD_RATE_N_S                0.25f
/* Learning envelope only: ordinary startup readiness and fall guards retain
 * their existing limits. The supported test settled at an 8.7 deg leg error. */
#define LEG_LOAD_MAX_LEG_ERROR_RAD                0.174532925f /* 10 deg */

/* 2. 虚拟腿转动：局部角度PD与腿部LQR是两个独立分量。
 * ANGLE_TARGET为机身坐标中的虚拟腿角，pi/2表示中立；不是AS5600原始角度。
 * REFERENCE_RATE限制参考的移动速度，单位rad/s；不是测量滤波时间常数。
 * native默认LEG_HOLD_RATIO=0，因此下列局部PD增益不会额外叠加到共同LQR上。 */
#define LEG_DEFAULT_ANGLE_REFERENCE_MODE          LEG_ANGLE_NEUTRAL
#define LEG_DEFAULT_ANGLE_TARGET_RAD              1.5707963267948966f
#define LEG_DEFAULT_ANGLE_REFERENCE_RATE_RAD_S    0.2f
#if ROBOT_NATIVE_LQR
/* Historical fixed9N-support probes found negative-direction breakaway at
 * left0.08/right0.12N.m; positive-direction breakaway was not established.
 * These values cannot certify positive-Tp recovery in the grounded robot.
 * The historical local-PD constants remain available, but native common
 * hold ratio0 leaves complete joint LQR in charge of common swing. */
#define LEG_DEFAULT_ANGLE_KP_NM_PER_RAD            0.60f
#define LEG_DEFAULT_ANGLE_KD_NM_S_PER_RAD          0.020f
#define LEG_DEFAULT_ANGLE_LIMIT_NM                 0.16f
/* Experimental1208 compensation worsened rearward motion on hardware.
 * Retained for diagnosis, explicitly disabled in all production profiles. */
#define LEG_ANGLE_INTEGRAL_ENABLED                 0U
#define LEG_ANGLE_KI_NM_PER_RAD_S                  0.80f
#define LEG_ANGLE_I_LIMIT_NM                       0.080f
#define LEG_ANGLE_I_RATE_NM_S                      0.040f
#define LEG_ANGLE_I_DEADBAND_RAD                   0.008726646f
#define LEG_ANGLE_I_MAX_ERROR_RAD                  0.34906585f
#define LEG_ANGLE_I_MAX_RATE_RAD_S                 0.50f
#define LEG_ANGLE_I_MAX_BODY_PITCH_RAD              0.174532925f
#else
#define LEG_DEFAULT_ANGLE_KP_NM_PER_RAD            0.11f
#define LEG_DEFAULT_ANGLE_KD_NM_S_PER_RAD          0.011f
#define LEG_DEFAULT_ANGLE_LIMIT_NM                0.022f
#endif
#if ROBOT_NATIVE_LQR
#define BALANCE_DEFAULT_LEG_SCALE                 1.50f /* Native dynamic Tp gain; axial support and equilibrium feedforward stay unscaled. */
#else
#define BALANCE_DEFAULT_LEG_SCALE                 0.40f
#endif
/* User-requested stronger leg LQR: scale 0.30 -> 0.40 on 2026-11-08.
 * Retain the 0.12 Nm virtual-leg cap and the 2 A motor current limits.
 * Supported bench trials do not establish free-standing ground stability. */
#if ROBOT_NATIVE_LQR
#define BALANCE_LEG_LQR_TORQUE_LIMIT_NM            0.0f /* No intermediate Tp clamp; motor 2A caps remain. */
#else
#define BALANCE_LEG_LQR_TORQUE_LIMIT_NM            0.120f
#endif
#if ROBOT_NATIVE_LQR
/* Full LQR owns common swing from the first valid cycle in every startup
 * phase. Differential synchronization and axial support remain separate. */
#define BALANCE_DEFAULT_LEG_HOLD_RATIO            0.0f
#else
#define BALANCE_DEFAULT_LEG_HOLD_RATIO            1.0f
#endif

/* 3. 左右摆角同步：ratio=0 关闭；每腿原始同步上限 = ratio * limit。
 * 共同回正优先分配可进一步缩小实际同步量，参见 sync_priority_scale。 */
#define LEG_DEFAULT_SYNC_RATIO                   0.65f
#define LEG_DEFAULT_SYNC_POSITION_KP             12.0f
#define LEG_DEFAULT_SYNC_SPEED_KP                0.06f
#define LEG_DEFAULT_SYNC_MAX_SPEED_RAD_S          20.0f
#define LEG_DEFAULT_SYNC_FILTER_TAU_S             0.004f
/* Generation 7 split trial: nearly double the small-error/damping gain,
 * per-leg cap 0.15 -> 0.195 Nm and slew 0.2 -> 0.4 Nm/s. Signs unchanged.
 * Staying below 0.2 Nm also avoids roundoff at the parameter guard boundary.
 * Command checks do not establish physical torque or standing stability. */
#define LEG_DEFAULT_SYNC_LIMIT_NM                0.3f
#define LEG_DEFAULT_SYNC_RATE_NM_S               0.4f /* Legacy mode only; native has driver current slew. */
/* Faster differential recovery only during established deceleration with split. */
#define LEG_SYNC_DECEL_RATE_SCALE                2.0f
#define LEG_SYNC_DECEL_SPLIT_RAD                 0.13962634f /* 8 deg */
#define LEG_SYNC_DECEL_SPEED_ERROR_M_S           0.10f
/* H7 allocation layer over ESP32-style differential sync, not an ESP feature.
 * Full priority at >=5deg same-sided neutral errors retains at least half
 * of each restoring PD+LQR command. Opposite-sided errors keep original sync. */
#define LEG_DEFAULT_SYNC_RECOVERY_PRIORITY       1U
#define LEG_SYNC_RECOVERY_RESERVE_RATIO           0.5f
#define LEG_SYNC_RECOVERY_BLEND_RAD               0.087266463f
/* Full balance guard checks individual body-relative legs, not their mean.
 * A grossly invalid initial pose is rejected immediately; during a run it
 * must persist for 40 ms. This stops outputs, it does not recover a fall. */
#define ROBOT_POSE_MAX_LEG_ERROR_RAD             1.134464014f /* 65 deg from vertical. */
#define ROBOT_POSE_MAX_SPLIT_RAD                 1.047197551f /* 60 deg left/right. */
#define ROBOT_POSE_CONFIRM_MS                    40U
/* Independent of leg split: both legs can stay aligned while the body falls. */
#define ROBOT_BODY_PITCH_LIMIT_RAD               0.610865238f /* 35 deg either direction. */
#define ROBOT_BODY_PITCH_CONFIRM_MS              40U
/* 4. 轮子平衡：整机 LQR 比例与单独倾斜测试比例分开。 */
#if ROBOT_NATIVE_LQR
#define BALANCE_DEFAULT_WHEEL_SCALE              1.0f
#define BALANCE_DEFAULT_WHEEL_RATE_SCALE         1.0f
#else
#define BALANCE_DEFAULT_WHEEL_SCALE              0.5f
#define BALANCE_DEFAULT_WHEEL_RATE_SCALE         0.5f /* Wheel dtheta/dphi terms only. */
#endif
#define BALANCE_DEFAULT_WHEEL_LIMIT_MV            6000.0f
#define BALANCE_DEFAULT_BACK_EMF_RATIO            0.3f
/* CH5-only standing: 0 captures position on entering normal balance and
 * holds that target. >0 selects the previous moving target window.
 * Preparation still uses a 10 cm window until fixed parking can engage. */
#define BALANCE_DEFAULT_POSITION_WINDOW_M        0.0f
/* Generation10 common-lean comparison: keep the true parking goal, but
 * bound distant position error in both LQR inputs to +/-0.10m.
 * 0 restores unbounded fixed parking. No measurement filtering/delay. */
#define BALANCE_DEFAULT_PARKING_CORRECTION_M      0.10f
/* Small per-wheel physical velocity damping in normal fixed parking.
 * No integral/filter or change to the coupled six LQR contributions. */
#define BALANCE_DEFAULT_PARKING_DAMPING_NM_PER_M_S 0.010f
#define BALANCE_PARKING_DAMPING_MAX_GAIN           0.010f
#define BALANCE_PARKING_DAMPING_LIMIT_NM           0.0015f
/* Slow velocity integral shifts world-leg inclination, not IMU zero.
 * Positive forward velocity learns a negative theta reference. */
/* Preserve servo feedback K rows; let0821 drift adaptation own slow trim. */
#ifndef BALANCE_NATIVE_POSITION_INTEGRAL_ENABLE
#define BALANCE_NATIVE_POSITION_INTEGRAL_ENABLE   0U
#endif
/* 0821 drift-to-world-leg-reference migration; other1531 parameters unchanged. */
#ifndef BALANCE_NATIVE_DRIFT_COMPENSATION_ENABLE
#define BALANCE_NATIVE_DRIFT_COMPENSATION_ENABLE  1U
#endif
#define BALANCE_NATIVE_DRIFT_MAX_SPEED_M_S        0.15f /* Original0821 learning envelope. */
#define BALANCE_DEFAULT_PARKING_LEAN_ENABLE       1U
#define BALANCE_DEFAULT_PARKING_LEAN_GAIN_RAD_M   0.05f
#define BALANCE_PARKING_LEAN_MAX_GAIN_RAD_M       0.05f
#define BALANCE_PARKING_LEAN_LIMIT_RAD            0.034906585f /* 2 deg */
#define BALANCE_PARKING_LEAN_RATE_RAD_S           0.003490659f /* 0.2 deg/s */
#define BALANCE_PARKING_LEAN_FILTER_S             0.30f
#define BALANCE_PARKING_LEAN_SPEED_DEADBAND_M_S   0.005f
#define BALANCE_PARKING_LEAN_MAX_SPEED_M_S        0.25f /* Learning gate only; output slew stays capped. */
#define BALANCE_PARKING_LEAN_SETTLE_MS            1000U
#define BALANCE_PARKING_LEAN_POSE_RAD             0.13962634f /* 8 deg */
#define BALANCE_PARKING_LEAN_BODY_RATE_RAD_S      0.52359878f /* 30 deg/s */
/* Joystick motion: calibrated CH2 forward, CH1 right-positive yaw. */
#define ROBOT_MOTION_DEFAULT_ENABLE              1U
#define ROBOT_MOTION_DEFAULT_FORWARD_M_S         0.40f
#define ROBOT_MOTION_DEFAULT_YAW_RAD_S           3.00f
#define ROBOT_MOTION_DEFAULT_ACCEL_M_S2          0.80f
#define ROBOT_MOTION_DEFAULT_YAW_ACCEL_RAD_S2    4.00f
#define ROBOT_MOTION_NEUTRAL_MS                  200U
#define ROBOT_MOTION_TRACK_WINDOW_M              0.05f
/* A setpoint crossing zero is not a stopped robot. The native reference
 * governor brakes to zero, then requires measured low-speed dwell before
 * releasing a pending direction. Its own ramp also bounds restart after wait. */
#ifndef BALANCE_MOTION_REFERENCE_ENABLE
#define BALANCE_MOTION_REFERENCE_ENABLE          1U
#endif
#define BALANCE_MOTION_ACCELERATION_M_S2         0.40f
#define BALANCE_MOTION_BRAKE_ACCELERATION_M_S2   0.80f
#define BALANCE_MOTION_STOP_SPEED_M_S            0.040f
#define BALANCE_MOTION_STOP_DWELL_S              0.10f
/* Coupled offset-free servo. Startup retains the original six-state feedback;
 * established balance blends both complete LQI rows together. */
#ifndef BALANCE_NATIVE_SERVO_ENABLE
#define BALANCE_NATIVE_SERVO_ENABLE              1U
#endif
#define BALANCE_SERVO_BLEND_S                    0.50f
#define BALANCE_PARKING_BLEND_S                  0.50f /* Full K/Ki blend, starting on accepted forward-neutral intent. */
#define BALANCE_SERVO_POSITION_WINDOW_M          0.60f /* Integral qualification only; parking P error is not clipped. */
#define BALANCE_SERVO_INTEGRAL_LIMIT_M_S         2.0f
#define BALANCE_SERVO_INTEGRAL_RATE_M            0.30f
/* Original six-state controller compatibility path only. The default servo
 * feeds the actual parking goal without a position-error cap. */
#define BALANCE_NATIVE_PARKING_WINDOW_M           0.30f
/* Slow parking correction along the existing two-output LQR position column.
 * The true parking goal stays fixed. Correction is retained while driving,
 * frozen outside qualified neutral parking, and cleared on OFF/fault. */
#define BALANCE_NATIVE_PARKING_INTEGRAL_ENABLE    0U /* Candidate: enable only after model/task screens. */
#define BALANCE_NATIVE_PARKING_INTEGRAL_GAIN_S    0.20f
#define BALANCE_NATIVE_PARKING_INTEGRAL_LEAK_S    0.20f
#define BALANCE_NATIVE_PARKING_INTEGRAL_LIMIT_M   0.50f
#define BALANCE_NATIVE_PARKING_INTEGRAL_RATE_M_S  0.10f
#define BALANCE_NATIVE_PARKING_INTEGRAL_BAND_M    0.01f
#define BALANCE_NATIVE_PARKING_INTEGRAL_DWELL_MS  300U
#define BALANCE_NATIVE_PITCH_TRIM_LIMIT_DEG       7.0f
/* Keep gravity estimation in shadow; the servo is the sole correction owner. */
#define BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE       0U /* Shadow only; LQI owns steady correction. */
#define ROBOT_MOTION_DEFAULT_YAW_ENABLE          1U /* Right turn: gyro Z negative, measured 2026-10-08. */
#define ROBOT_MOTION_YAW_FEEDBACK_DIRECTION      (-1)
#define ROBOT_MOTION_YAW_KP_NM_PER_RAD_S          0.012f
#define ROBOT_MOTION_YAW_TORQUE_LIMIT_NM          0.012f
#define ROBOT_MOTION_YAW_TORQUE_RATE_NM_S         0.045f
#define ROBOT_MOTION_YAW_KI_NM_PER_RAD           0.016f
#define ROBOT_MOTION_YAW_INTEGRAL_LIMIT_NM       0.008f
#define ROBOT_MOTION_YAW_FILTER_S               0.03f
#define ROBOT_MOTION_YAW_RATE_DEADBAND_RAD_S      0.02f
#define ROBOT_MOTION_SPEED_DAMPING_NM_PER_M_S    0.120f
#define ROBOT_MOTION_SPEED_DAMPING_LIMIT_NM      0.030f
#define ROBOT_MOTION_SPEED_DAMPING_RATE_NM_S     0.100f
/* Preserve the already tested neutral startup recovery response. */
#define ROBOT_MOTION_STARTUP_DAMPING_NM_PER_M_S  0.075f
#define ROBOT_MOTION_STARTUP_DAMPING_LIMIT_NM    0.016f
#define ROBOT_MOTION_STARTUP_DAMPING_RATE_NM_S   0.060f
/* Neutral startup captures a fixed parking goal after the leg ramp, as in
 * the original standing controller. Moving zero-speed recovery is opt-in. */
#ifndef ROBOT_MOTION_STARTUP_RECOVERY_ENABLE
#define ROBOT_MOTION_STARTUP_RECOVERY_ENABLE    0U
#endif
#define ROBOT_MOTION_LEAN_MAX_SPEED_M_S        0.60f
#define ROBOT_MOTION_FAST_LEAN_GAIN_RAD_M_S     0.08f
#define ROBOT_MOTION_FAST_LEAN_LIMIT_RAD        0.017453293f /* 1 deg */
#define ROBOT_MOTION_FAST_LEAN_RATE_RAD_S       0.06981317f /* 4 deg/s */
#define ROBOT_MOTION_LEAN_GAIN_RAD_M            0.10f
#define ROBOT_MOTION_LEAN_RATE_RAD_S            0.008726646f /* 0.5 deg/s, same 2 deg cap. */
#define ROBOT_MOTION_LEAN_SETTLE_MS             250U
#define ROBOT_MOTION_LEAN_MAX_YAW_RAD_S         1.00f /* Slow equilibrium learning only; fast correction stays available in turns. */
#define ROBOT_MOTION_STALL_CONFIRM_MS           500U /* Pauses learning only; never changes motor ownership. */
#define BALANCE_STARTUP_POSITION_WINDOW_M        0.1f
/* Manual body-pitch equilibrium trim; positive = forward, negative = backward.
 * The measured-torque observer remains opt-in; trim is a posture request,
 * not a measurement of the assembled centre of mass. */
#define BALANCE_DEFAULT_PITCH_TRIM_DEG            -4.0f
#define BALANCE_PITCH_TRIM_RATE_DEG_S             0.5f
#define WHEEL_TEST_DEFAULT_SCALE                  1.0f
#define WHEEL_TEST_DEFAULT_LEG_LENGTH_M           0.11f
#define WHEEL_TEST_DEFAULT_PITCH_DEADBAND_DEG     0.15f
#define WHEEL_TEST_DEFAULT_RATE_DEADBAND_RAD_S    0.00872664626f

/* 5. 供电与电机换算：软件配置，不是电池电压/电流测量。
 * H7 换算模型、F103 PWM 母线模型是不同参数，换电源需配套核对。
 * 改硬限压/驱动母线须重建并烧录对应 F103 固件。
 * 当前电池约 16 V；以下保留此前 12/9 V 配置，尚未适配电池。 */
#define WHEEL_SUPPLY_VOLTAGE_V                   12.0f
#define JOINT_DRIVER_SUPPLY_VOLTAGE_V            9.0f
#define WHEEL_VOLTAGE_LIMIT_MV                   6000.0f
/* User-specified per-wheel requested torque ceiling, before Nm->voltage.
 * Empirical calibration does not provide a measured-current torque limit. */
#define WHEEL_TORQUE_LIMIT_NM                    0.030f
/* Original full-control output restored at user's request. This optional
 * requested-Nm ceiling needs motor-specific Nm/V calibration before reuse. */
#ifndef WHEEL_REQUEST_TORQUE_LIMIT_ENABLED
#define WHEEL_REQUEST_TORQUE_LIMIT_ENABLED       0U
#endif
#define JOINT_VOLTAGE_LIMIT_MV                   6900.0f
#define JOINT_TORQUE_LIMIT_NM                    0.700f /* J^T fallback ceiling; verified3A*Kt=.69Nm is tighter. */
#define JOINT_DEFAULT_VOLTAGE_LIMIT_MV           6900.0f
#define JOINT_DEFAULT_BACK_EMF_RATIO             0.0f
/* 1012 output comparison: 0.0096 / 1.05 gives 5% more left torque voltage.
 * This is command scaling, not a new measured motor calibration. */
#define WHEEL_LEFT_TORQUE_RATIO                  0.009142857f
#define WHEEL_RIGHT_TORQUE_RATIO                 0.0101f
/* 关节顺序：左前 LF、左后 LR、右前 RF、右后 RR。 */
#define JOINT_LF_TORQUE_RATIO                    0.0400f
#define JOINT_LR_TORQUE_RATIO                    0.0400f
#define JOINT_RF_TORQUE_RATIO                    0.0400f
#define JOINT_RR_TORQUE_RATIO                    0.0400f

/* 6. 启动渐入：原生 LQR 先用局部腿环恢复近竖直姿态，再渐入轮/腿 LQR。 */
#ifndef ROBOT_DEFAULT_FEEDBACK_ONLY
#define ROBOT_DEFAULT_FEEDBACK_ONLY              0U /* Full control requested; CH5 OFF->ON still required. */
#endif
#define ROBOT_DEFAULT_STARTUP_ENABLE             1U
#define ROBOT_STARTUP_WHEEL_DAMPING_NM_S_PER_RAD  0.001f
#define ROBOT_STARTUP_WHEEL_TORQUE_LIMIT_NM       0.012f
#define ROBOT_STARTUP_GAP_M                      0.003f
#define ROBOT_STARTUP_LENGTH_ERROR_M             0.01f
#define ROBOT_STARTUP_ANGLE_ERROR_RAD            0.087266463f
#define ROBOT_STARTUP_LENGTH_SPEED_M_S           0.02f
#define ROBOT_STARTUP_ANGLE_SPEED_RAD_S           0.2f
#define ROBOT_STARTUP_BODY_RATE_RAD_S             0.1f
/* No wheel-speed readiness gate: full balance must be able to arrest rolling. */
#define ROBOT_STARTUP_SUPPORT_ERROR_N             0.1f
/* Native readiness gates driving permission on pose/reference arrival.
 * Complete coupled LQR feedback stays active in every startup phase. */
#define ROBOT_STARTUP_RAMP_S                      0.5f
#ifndef ROBOT_CAPTURE_BUILD
#define ROBOT_CAPTURE_BUILD                      0
#endif
/* Supported data collection only; every CH5 launch stops after1.5s.
 * The duration is compiled in and cannot be removed through Watch. */
#define ROBOT_CAPTURE_LIMIT_MS                   1500U
#define ROBOT_NATIVE_PREPARE_SETTLE_S              0.25f

/* 7. 遥控解锁：CH5 上=306 开启，中=1000/下=1694 关闭。
 * 上电/故障后须先经过关闭挡；不是 raw 大于阈值就开启。 */
#define ROBOT_DEFAULT_REMOTE_ENABLE              1U
#define ROBOT_REMOTE_CHANNEL_INDEX              4U
#define ROBOT_REMOTE_DEFAULT_ON_RAW              306U
#define ROBOT_REMOTE_DEFAULT_CENTER_RAW          1000U
#define ROBOT_REMOTE_DEFAULT_OFF_RAW             1694U
#define ROBOT_REMOTE_DEFAULT_TOLERANCE_RAW        50U

/* 8. 安装与方向：几何/极性与控制增益分开设置。
 * 编码器方向用于统一前进速度符号；输出方向用于匹配电机安装极性。 */
#define WHEEL_LEFT_CAN_ID                        3U
#define WHEEL_RIGHT_CAN_ID                       7U
#define WHEEL_RADIUS_M                          0.026f
#define LEFT_LEG_BRANCH                         LEG_BRANCH_DOWN
#define RIGHT_LEG_BRANCH                        LEG_BRANCH_DOWN
/* IMU：机头向下时 raw roll 减小；0=roll，1=pitch。 */
#define BODY_PITCH_AXIS                         0U
#define BODY_PITCH_DIRECTION                    (-1.0f)
#define WHEEL_LEFT_DIRECTION                    (-1)
#define WHEEL_RIGHT_DIRECTION                   1
#define WHEEL_LEFT_VOLTAGE_DIRECTION             (-1)
#define WHEEL_RIGHT_VOLTAGE_DIRECTION            1
/* Preserve the established wheel/body feedback polarity.  A global output
 * inversion destabilized attitude in build1207; the old ESP32 output sign
 * cannot be transferred without checking its complete state convention. */

#endif
