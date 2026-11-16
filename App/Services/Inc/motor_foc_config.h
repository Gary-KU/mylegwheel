#ifndef MOTOR_FOC_CONFIG_H
#define MOTOR_FOC_CONFIG_H

/* Keil 正式目标通过编译宏选择 A1 电流控制；主机测试及旧电压目标可关闭此分支。 */
#ifndef MYLEGWHEEL_A1_CURRENT
#define MYLEGWHEEL_A1_CURRENT 0
#endif
#ifndef MOTOR_FOC_NODE_MASK
#define MOTOR_FOC_NODE_MASK 0x77U /* bit(ID-1): joints1/2/5/6, wheels3/7. */
#endif
/* Full-vehicle mode; leg calibration remains available as an explicit profile. */
#ifndef ROBOT_LEGS_ONLY_CALIBRATION
#define ROBOT_LEGS_ONLY_CALIBRATION 0
#endif
#ifndef MOTOR_FOC_OUTPUT_ENABLE_MASK
#define MOTOR_FOC_OUTPUT_ENABLE_MASK 0x77U /* Joints 1/2/5/6 and wheels 3/7. */
#endif
#if ROBOT_LEGS_ONLY_CALIBRATION && (MOTOR_FOC_OUTPUT_ENABLE_MASK & 0x44U)
#error Leg calibration must inhibit wheel motion
#endif
#define MOTOR_FOC_STATUS_TIMEOUT_MS 40U
#define MOTOR_FOC_ANGLE_TIMEOUT_MS 70U /* page2 is only sent every 30 ms. */
#define MOTOR_FOC_FAST_TIMEOUT_MS 10U /* 0x700+ID: angle+speed at 200 Hz. */
#define MOTOR_FOC_ACK_TIMEOUT_MS 40U
#define MOTOR_FOC_COMMAND_PERIOD_MS 5U /* Match the 5ms DLQR/robot task and FAST feedback. */
#define MOTOR_FOC_STOP_PERIOD_MS 100U
#ifndef MOTOR_FOC_CURRENT_HARD_LIMIT_A
#define MOTOR_FOC_CURRENT_HARD_LIMIT_A 3.0f /* User-requested wheel limit. */
#endif
#ifndef MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A
#define MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A 3.0f /* User-requested joint limit. */
#endif
/* Readback describes the driver's capability, not permission to request it.
 * A1 4015/2808 drivers advertise up to 3.3 A; transmitted Iq stays bounded
 * by min(role hard limit, local calibration limit, verified remote limit).
 */
#define MOTOR_FOC_REMOTE_IQ_LIMIT_MAX_A 3.3f
/* Wheel2808 candidate permits7.2V; its18kHz/0..89% sampling profile
 * still clamps the physical vector near7.194V at14V. Joint drivers remain5V. */
#define MOTOR_FOC_WHEEL_VOLTAGE_LIMIT_MAX_V 7.2f
/* The supported driver alignment lasts 4 s. Keep its 50 ms refresh and
 * allow completion plus transport margin; a timeout still sends STOP.
 */
#define MOTOR_FOC_ALIGN_TIMEOUT_MS 5000U
/* Explicit calibration journal save includes Flash work and a board reboot. */
#define MOTOR_FOC_CALIBRATION_TIMEOUT_MS 1000U

/* Kt 填轴端力矩 / 固件 Iq，单位 Nm/A，不能沿用旧 Nm/V 系数或用电源电流代替 Iq。
 * 零值表示未标定；运行还要求 calibration_confirmed=1，方向须按同一编码器约定核对。 */
#define MOTOR_FOC_DEFAULT_KT_NM_PER_A 0.0f
#define MOTOR_FOC_WHEEL_ESTIMATED_KT_NM_PER_A 0.07643f /* Historical 9.55/KV125 estimate; not measured Nm/FOC-Iq. */
#define MOTOR_FOC_WHEEL_KV_RPM_PER_V 125.0f
#define MOTOR_FOC_WHEEL_LINE_RESISTANCE_OHM 9.0f
#define MOTOR_FOC_WHEEL_PHASE_RESISTANCE_OHM 4.5f
#define MOTOR_FOC_WHEEL_LINE_INDUCTANCE_H .0034f
#define MOTOR_FOC_WHEEL_DATASHEET_MAX_CURRENT_A 3.3f
#define MOTOR_FOC_WHEEL_DATASHEET_MAX_TORQUE_NM .1f
#define MOTOR_FOC_DEFAULT_CALIBRATION_CONFIRMED 0U
#ifndef MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A
#define MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A 3.0f
#endif
/* Restore the explicitly requested limit under STOP after every driver reboot.
 * A matching ACK and exact parameter echo are required before active output.
 * This RAM parameter does not change the driver's independent phase trip. */
#ifndef MOTOR_FOC_FIXED_IQ_LIMIT_A
#define MOTOR_FOC_FIXED_IQ_LIMIT_A 3.0f
#endif
/* This robot only: signed current pulses and supported ground closed-loop
 * commissioning completed 2026-11-08. This is not free-standing balance proof.
 * Clear affected bits after motor/driver changes before recommissioning. */
#ifndef MOTOR_FOC_COMMISSIONED_RESPONSE_MASK
#define MOTOR_FOC_COMMISSIONED_RESPONSE_MASK 0x77U
#endif
/* Fixed execution setting, restored under STOP after each driver restart.
 * This is not a LQR tuning knob. Zero is reserved for legacy host fixtures. */
#ifndef MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S
#define MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S 10.0f
#endif
#define MOTOR_FOC_COMMISSIONED_MIN_IQ_SLEW_A_PER_S \
    (MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S>0 ? MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S : 1.0f)

#define MOTOR_FOC_JOINT_KT_NM_PER_A 0.23f /* User-confirmed 4015 joints1/2/5/6. */
/* Wheels3/7: 2808, 7 pole pairs. Kt above is only an estimate from KV;
 * user supplied a0.1N*m maximum-torque figure, not a measured0.07643N*m/A.
 * Software torque readouts must not be treated as measured shaft torque. */
#define MOTOR_FOC_DEFAULT_DIRECTIONS {-1, -1, -1, 1, 1, 1, 1, 1}

/* A1 mechanical angle is wrapped. Wheels use fresh speed feedback, while
 * joints require an independently confirmed absolute turn/zero reference.
 * Setting a joint bit alone cannot silently reuse an old multi-turn zero. */
#define MOTOR_FOC_JOINT_NODE_MASK 0x33U /* LF=1, LR=2, RF=5, RR=6 */
/* This robot's commissioned joints: user-calibrated mechanical references,
 * with valid FAST frames and 0..5 ms receive ages checked through DAPLink.
 * Bits are motor ID - 1, not indices into g_joint_calibration.
 * Init takes each turn anchor from that joint's encoder_reference_rad;
 * keep angle calibration in joint_calibration_config.h only.
 * These approvals do not replace live fault/freshness/pair-skew checks or
 * confirm motor-output dynamics. Clear the affected bit before recommissioning.
 */
#ifndef MOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK
#define MOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK 0x33U
#endif
#ifndef MOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK
#define MOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK 0x33U
#endif
#if MOTOR_FOC_NODE_MASK & ~0xffU
#error Motor FOC mask supports IDs 1 through 8 in this robot
#endif
#if (MOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK | MOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK) & ~MOTOR_FOC_JOINT_NODE_MASK
#error Joint feedback confirmations must refer to configured joint nodes only
#endif
#endif
