#ifndef WHEEL_LEG_TASK_H
#define WHEEL_LEG_TASK_H

#include <stdint.h>
#include "leg_model.h"
#include "robot_motion_command.h"

typedef enum {
    WHEEL_LEG_WAIT_FEEDBACK = 0,
    WHEEL_LEG_BAD_GEOMETRY,
    WHEEL_LEG_BAD_TIMING,
    WHEEL_LEG_OBSERVE,        /* All observation channels valid. */
    WHEEL_LEG_OBSERVE_PARTIAL /* Some channels valid; check individual flags. */
} WheelLegStatus;

typedef enum {
    WHEEL_LEG_FEEDBACK_OK = 0,
    WHEEL_LEG_FEEDBACK_IMU,
    WHEEL_LEG_FEEDBACK_MOTOR,
    WHEEL_LEG_FEEDBACK_STALE,
    WHEEL_LEG_FEEDBACK_CALIBRATION,
    WHEEL_LEG_FEEDBACK_NONFINITE,
    WHEEL_LEG_FEEDBACK_SKEW
} WheelLegFeedbackDetail;

typedef struct {
    uint32_t updates;
    uint32_t tick_ms;
    uint32_t period_ms;
    uint32_t stack_free_min_bytes; /* Task high-water mark; 0 until task samples it. */
    uint32_t status;
    float body_pitch_rad;    /* 机身前倾为正；LQR 的 phi 使用相反符号。 */
    float gyro_rad_s[3];     /* Raw sensor axes; no assumed Euler-rate mapping. */
    LegModel leg[2];
    float length_speed_m_s[2];
    float length_speed_from_position_m_s[2]; /* Unfiltered finite difference; observation only. */
    uint32_t length_speed_position_valid[2]; /* Requires two consecutive valid leg samples. */
    float angle_speed_rad_s[2];
    float wheel_speed_rad_s[2]; /* Raw shaft speed, before direction/ratio. */
    uint32_t feedback_detail;   /* WheelLegFeedbackDetail */
    uint32_t feedback_motor_id; /* First failing CAN ID; 0 for IMU/skew/no fault. */
    uint32_t feedback_ms;       /* Failing motor's age, or cross-motor skew. */
    uint32_t imu_valid;
    uint32_t leg_valid[2];      /* left/right; invalid leg data is zeroed. */
    uint32_t wheel_valid[2];    /* Independent of leg and IMU availability. */
    float body_pitch_rate_rad_s; /* Planar approximation: signed gyro X. */
    float state[2][6];          /* 左/右；theta、dtheta、x、dx、phi、dphi；rad、m、s。 */
    float reference[2][6];
    float lqr_torque_nm[2][2];  /* T, Tp; calculation only, never actuated. */
    uint32_t state_valid[2];    /* Preview valid; wheel carrier motion omitted. */
    uint32_t lqr_valid[2];      /* Also requires length within gain-fit range. */
    float joint_speed_rad_s[2][2]; /* 左/右，再后/前；已标定的模型关节角速度。 */
    /* Diagnostic snapshots, LF/LR/RF/RR; never used by actuator control. */
    float joint_angle_rad[4], joint_shaft_speed_rad_s[4];
    uint32_t joint_feedback_age_ms[4], joint_feedback_valid[4];
} WheelLegDebug;

extern volatile WheelLegDebug g_wheel_leg;
extern volatile float g_wheel_leg_reference[2][6];

typedef enum {
    ROBOT_FAULT_NONE = 0, ROBOT_FAULT_LENGTH_RANGE, ROBOT_FAULT_FEEDBACK,
    ROBOT_FAULT_TIMING, ROBOT_FAULT_GEOMETRY, ROBOT_FAULT_PARAMETERS,
    ROBOT_FAULT_TX, ROBOT_FAULT_TX_DISABLED, ROBOT_FAULT_STATE,
    ROBOT_FAULT_LEG_POSE, /* 9: individual leg posture / left-right split. */
    ROBOT_FAULT_BODY_PITCH /* 10: full-balance body pitch outside recovery envelope. */
} RobotControlFaultReason;
typedef struct {
    uint32_t count, tick_ms; /* Latched until next fault or reset. */
    RobotControlFaultReason reason; /* Debugger can show the symbolic fault name. */
    uint32_t dual_status, balance_status;
    uint32_t feedback_detail, motor_id, feedback_ms, period_ms;
    float leg_length_m[2]; /* Read only where leg_valid is 1. */
    uint32_t leg_valid[2];
} RobotControlFaultDebug;
typedef enum {
    ROBOT_STARTUP_OFF = 0, ROBOT_STARTUP_PREPARE,
    ROBOT_STARTUP_RAMP, ROBOT_STARTUP_BALANCE
} RobotStartupPhase;
enum {
    ROBOT_READY_GAP = 1U, ROBOT_READY_LENGTH = 2U, ROBOT_READY_POSE = 4U,
    ROBOT_READY_SPEED = 8U, ROBOT_READY_SUPPORT = 16U, ROBOT_READY_ALL = 31U
};
typedef struct {
    uint32_t enable, status; /* 0 off, 1 queued, 2 feedback, 3 parameters, 4 TX,
                             * 5 TX disabled, 6 wheel-test/zeroing ownership. */
    RobotControlFaultDebug last_fault;
    uint32_t startup_enable; /* 0: immediate zero-reference LQR; 1: capture/slew theta, qualify motion readiness. */
    RobotStartupPhase startup_phase; /* Read-only: off, prepare, ramp, balance. */
    uint32_t startup_ready_mask; /* POSE参与行驶许可检查；不用于关闭native平衡输出，其他位仅诊断。 */
    float startup_elapsed_s, startup_settle_s, startup_blend; /* Native readiness progress; full LQR stays active. */
    float wheel_forward_m[2], wheel_forward_gap_m; /* Body-relative L*cos(angle), left minus right. */
    uint32_t remote_enable, remote_on_raw; /* 1: CH5 owns all enables; on_raw is upper detent, NOT a threshold. */
    uint32_t remote_ch5_raw, remote_link_ok, remote_switch_on, remote_rearm_ready;
    uint32_t remote_status; /* 0 Watch, 1 link lost, 2 off, 3 wait off/rearm, 4 on, 5 parameters. */
    uint32_t remote_center_raw, remote_off_raw, remote_tolerance_raw;
    uint32_t feedback_only; /* 1 blocks all control/test outputs; sensor telemetry still runs. */
    float remote_ch5_percent; /* Processed signed CH5: upper +100, center 0, lower -100. */
} RobotControlDebug;
/* 总开关同时控制双腿保持和完整 LQR，默认关闭。
 * CH5 默认接管使能；上电、断连或故障后须先到中/下挡，再拨上挡重新解锁。
 * remote_enable=0 时改用 Watch 手动启停，供台架调试。 */
extern volatile RobotControlDebug g_robot_control;

typedef enum {
    LEG_ANGLE_CAPTURE = 0, /* Legacy independent initial-pose hold. */
    LEG_ANGLE_NEUTRAL = 1  /* Slew both references toward a common body-relative target. */
} LegAngleReferenceMode;
typedef struct {
    uint32_t enable, status; /* 0 off, 1 queued, 2 feedback, 3 config/polarity, 4 TX,
                             * 5 TX disabled, 7 unsafe full-balance leg posture,
                             * 8 excessive full-balance body pitch. */
    float target_m, position_kp, speed_kp, speed_ki;
    float max_speed_m_s, filter_tau_s, max_force_n;
    float feedforward_target_n, feedforward_rate_n_s;
    float overlength_kp_n_per_m, overlength_margin_m; /* KP=0关闭过长卸力；native默认0，可Watch调整。 */
    float overlength_startup_s; /* Ramp restoring stiffness after arming; 0 bypasses ramp. */
    float overlength_transition_m; /* Smooth onset over this width; 0 restores linear onset. */
    float length_error_deadband_m, length_speed_deadband_m_s;
    uint32_t support_only; /* 1: axial force >=0 for standing; 0: signed bench control. */
    float torque_ratio_nm_per_v[4], voltage_limit_mv; /* ESP32 LF/LR/RF/RR calibration. */
    float back_emf_ratio; /* 0..1; joint compensation only. Defaults 0 for bench diagnosis. */
    float angle_kp_nm_per_rad, angle_kd_nm_s_per_rad, angle_limit_nm;
    LegAngleReferenceMode angle_reference_mode;
    float angle_target_rad, angle_reference_rate_rad_s; /* Common body-relative target/rate; rad and rad/s. */
    int32_t direction[4];    /* LF, LR, RF, RR; 0 unknown blocks ALL output. */
    float feedforward_n, error_m[2], force_n[2];
    float length_error_control_m[2], length_speed_control_m_s[2]; /* After continuous deadbands. */
    float overlength_force_n[2]; /* 过长主动扣减量，非正；KP=0时为0，不表示没有关节限幅。 */
    float overlength_blend; /* 0..1 startup stiffness blend; resets on disarm. */
    float force_requested_n[2], force_limit_n[2]; /* 轴向限幅前请求，以及纯轴向可用输出上限。 */
    uint32_t force_saturated[2]; /* Axial clamp active; separate from final joint voltage clamp. */
    float joint_torque_nm[4]; /* LF, LR, RF, RR, model coordinates. */
    float angle_reference_rad[2], angle_error_rad[2], pitch_torque_nm[2];
    float angle_hold_torque_nm[2], lqr_pitch_torque_nm[2]; /* Applied terms; pitch also includes sync_torque_nm. */
    /* Capture first for a bumpless start, then approach the common target in
     * NEUTRAL mode. CAPTURE retains each initial pose. Balance toggles do not
     * recapture. Balance leg_hold_ratio blends the PD alongside LQR Tp. */
    int16_t voltage_mv[8];
    /* Angle-difference cascade runs with dual hold, independently of LQR
     * scales, local reference mode and balance leg_hold_ratio. ratio=0 off. */
    float sync_ratio, sync_position_kp, sync_speed_kp, sync_max_speed_rad_s;
    float sync_filter_tau_s, sync_limit_nm, sync_rate_nm_s;
    float sync_angle_difference_rad, sync_speed_difference_rad_s;
    float sync_filtered_error_rad, sync_target_speed_rad_s;
    float sync_torque_nm[2]; /* Applied model-angle torque; included in pitch_torque_nm. */
    /* Optional slow length-error integral; no weight/current/contact sensor. */
    uint32_t load_enable; /* 0 clears correction; 1 grounded standing only. */
    float load_ki_n_per_m_s, load_limit_n, load_rate_n_s;
    float load_force_n[2]; /* Correction used for this cycle, before axial clamp. */
    uint32_t load_frozen[2]; /* Enabled but gated/headroom-limited this cycle. */
    uint32_t sync_recovery_priority_enable; /* 0 legacy mix, 1 full-neutral recovery allocation. */
    float sync_raw_torque_nm[2]; /* Original slew-limited differential command. */
    float sync_priority_scale; /* 0..1 common scale, applied to BOTH sync commands. */
    uint32_t sync_priority_active; /* Allocation/release currently weakens nonzero sync. */
} DualHoldDebug;
extern volatile DualHoldDebug g_dual_hold;
/* 高度速度PI实际使用的力偏置，单位N；只读诊断，OFF/故障清零。 */
extern volatile float g_leg_height_integral_n[2];
/* Per-leg loaded-angle bias torque, Nm. Cleared on disarm/fault; no Flash save. */
extern volatile float g_leg_angle_integral_nm[2];
typedef struct {
    uint32_t enable, status; /* 0 off, 1 calculated, 2 feedback/range, 3 parameters. Requires dual hold. */
    float wheel_scale, leg_scale; /* Legacy T/Tp scales; native leg_scale is dynamic Tp gain0..2, wheel remains full. */
    float leg_hold_ratio; /* 0..1 local angle PD during balance; default 1. */
    float wheel_limit_mv, back_emf_ratio; /* Full-balance wheel EMF: 0..1, default 0. */
    float state[6], reference[6]; /* Shared state; x target locked in parking or window-limited. */
    float leg_length_m, torque_nm[2]; /* T per wheel, Tp per leg; scaled experimental outputs. */
    float wheel_torque_component_nm[6]; /* LQR contributions: theta,dtheta,x,dx,phi,dphi. */
    uint32_t saturated;
    float startup_wheel_damping_nm_s_per_rad, startup_wheel_limit_nm; /* 0..0.02 each; master startup only. */
    float startup_wheel_damping_torque_nm; /* Added to six LQR terms; fades out during RAMP. */
    float position_window_m; /* 0: park at normal-balance entry; >0: moving target window. */
    float pitch_trim_deg; /* Writable; native +/-7 deg forward-positive trim, not IMU zero. */
    float pitch_trim_applied_deg; /* Read-only; slewed after preparation, zeroed on stop/fault. */
    uint32_t position_locked; /* Read-only: normal-balance parking target captured once. */
    float parking_correction_limit_m; /* 0 disables; limits error in LQR, never moves true parking goal. */
    float parking_target_m, parking_error_m; /* Read-only true goal and actual x minus true goal. */
    float wheel_rate_scale; /* 0..1 multiplier of wheel dtheta/dphi contributions only. */
    float parking_damping_gain_nm_per_m_s; /* 0..0.01; 0 gives original full-control output. */
    float parking_damping_torque_nm; /* Per-wheel additive physical velocity damping, max1.5mNm. */
    uint32_t parking_damping_active; /* Normal fixed parking, wheel owner and gain enabled. */
    uint32_t parking_lean_enable; /* Shared equilibrium adaptation in parking and straight driving. */
    float parking_lean_gain_rad_per_m; /* 0..0.05 parking gain; driving gain fixed in config. */
    float parking_lean_rad; /* World-leg theta reference, +/-2deg; retained while learning pauses. */
    float parking_lean_filtered_speed_m_s;
    uint32_t parking_lean_learning; /* Parking learning; separate motion_lean_learning flag for driving. */
    float motion_speed_error_m_s, motion_damping_torque_nm;
    uint32_t startup_recovery; /* Initial neutral zero-speed recovery before parking capture. */
    uint32_t motion_braking; /* Neutral request; wait for low measured speed before position lock. */
    float motion_lean_rad; /* Fast +/-1deg transient term, added to slow +/-2deg equilibrium. */
    uint32_t motion_damping_active, motion_lean_learning, motion_stall_hold;
} BalanceDebug;
extern volatile BalanceDebug g_balance;
typedef struct {
    uint32_t enable, learning, gate_mask, settle_ms;
    float correction_m; /* LQR position-error correction; never the true goal. */
} BalanceParkingIntegralDebug;
extern volatile BalanceParkingIntegralDebug g_balance_parking_integral;
typedef struct {
    uint32_t enable, phase; /* 0 parked, 1 tracking, 2 braking; read actual speed separately. */
    float velocity_m_s, goal_m, quiet_s;
} BalanceMotionReferenceDebug;
extern volatile BalanceMotionReferenceDebug g_balance_motion_reference;
typedef struct {
    uint32_t enable, active, learning, gate_mask;
    float blend, integral_m_s, wheel_integral_nm, leg_integral_nm;
    float parking_blend; /* 0 driving LQI, 1 jointly redesigned neutral-parking LQI; OFF resets. */
} BalanceServoDebug;
extern volatile BalanceServoDebug g_balance_servo;
/* 当前模型平衡点及重力摆矩前馈，只读；实际theta参考仍按启动速率平滑移动。 */
extern volatile float g_balance_equilibrium_theta_rad,g_balance_equilibrium_tp_nm;
typedef struct {
    RobotMotionConfig config;
    uint32_t yaw_enable; /* Set after checking right-turn gyro sign. */
    int32_t yaw_feedback_direction; /* Sensor Z -> right-positive yaw, +/-1. */
    float yaw_kp_nm_per_rad_s, yaw_torque_limit_nm, yaw_torque_rate_nm_s;
    RobotMotionCommand command; /* CH2 velocity, CH1 right-positive yaw rate. */
    uint32_t neutral_ready, translating, yaw_active;
    float measured_yaw_rate_rad_s, yaw_error_rad_s;
    float differential_requested_nm, differential_applied_nm;
    float filtered_yaw_rate_rad_s;
    float yaw_integral_nm; /* Bounded correction; cleared on release/reversal/stop. */
    uint32_t yaw_integral_blocked; /* Cap, slew or wheel headroom blocks growth. */
} RobotMotionDebug;
extern volatile RobotMotionDebug g_robot_motion;
typedef struct {
    uint32_t enable, status; /* 0 off, 1 queued, 2 IMU/timing, 3 parameters, 4 TX, 5 TX disabled, 6 zeroing. */
    float leg_length_m, scale, wheel_limit_mv;
    uint32_t zero_request, zero_status, zero_samples; /* request=1 captures 500 ms; status 0 idle, 1 sampling, 2 ready, 3 waiting still/IMU. */
    float pitch_zero_deg, pitch_rate_bias_rad_s;
    float pitch_deadband_deg, rate_deadband_rad_s;
    float pitch_error_deg, pitch_rate_error_rad_s;
    uint32_t centered; /* Angle/rate inside deadbands: true zero voltage, including compensation. */
    float state[6]; /* Virtual vertical legs; theta=body pitch, phi=-body pitch; x/dx=0. */
    float torque_nm;
    int16_t voltage_mv[8]; /* Only wheel IDs 3/7 can be nonzero. */
    uint32_t saturated;
} WheelTestDebug;
/* Single-switch IMU/LQR wheel response test. No leg or encoder feedback needed.
 * Takes output ownership and disables dual hold/balance until explicitly rearmed. */
extern volatile WheelTestDebug g_wheel_test;
typedef struct {
    float torque_ratio_nm_per_v[2], supply_voltage_v; /* Supply model shared by wheels AND joints. */
    float effective_supply_voltage_v, output_ratio, back_emf_blend;
    float torque_voltage_mv[2], back_emf_mv[2], requested_mv[2];
    float speed_rad_s[2]; /* Forward-positive, encoder mounting signs applied. */
    uint32_t speed_valid[2]; /* Missing/stale speed disables only that side's compensation. */
    int16_t voltage_mv[2]; /* Actual left/right commands after clamp and actuator signs. */
    uint32_t saturated;
    float requested_torque_nm, applied_torque_nm; /* Before/after common requested-Nm ceiling. */
    uint32_t torque_saturated; /* Requested Nm ceiling, separately from voltage/EMF. */
    float requested_per_side_nm[2], applied_per_side_nm[2];
    float requested_differential_nm, applied_differential_nm;
    float requested_iq_a[2]; /* A1 target after Nm/Kt; voltage fields stay zero. */
    uint32_t current_backend[2];
} WheelOutputDebug;
/* Shared by full balance and the single-switch wheel test; no extra enable. */
extern volatile WheelOutputDebug g_wheel_output;

typedef struct {
    uint32_t enable, reset_request, learning, samples, status;
    float estimated_offset_m, phi_reference_rad, body_pitch_target_deg;
    float actual_pitch_torque_nm, nominal_height_m;
    /* Default shadow estimates never enter the reference/torque path.
     * enable=1 explicitly requests bounded application after quiet dwell.
     * gate bits: estimator, BALANCE, parked, quiet, symmetry, support,
     * unsaturated, measured-Iq, remote-neutral, dwell, confident, dynamics. */
    uint32_t shadow_enable, gate_mask, confidence_ms;
    float applied_offset_m, applied_phi_rad;
} BalanceBiasDebug;
extern volatile BalanceBiasDebug g_balance_bias;

typedef struct {
    float effective_supply_voltage_v, output_ratio;
    float torque_mv_per_nm[4], torque_voltage_mv[4], back_emf_mv[4], requested_mv[4];
    float speed_rad_s[4]; /* Shaft rad/s expressed in the torque command coordinate. */
    int16_t voltage_mv[4]; /* LF/LR/RF/RR, after clamp and actuator signs. */
    uint32_t saturated;
} JointOutputDebug;
/* Diagnostics only; enabled by dual hold, using its calibration/voltage limit.
 * Invalid joint speed disarms all outputs; no extra switch is required. */
extern volatile JointOutputDebug g_joint_output;

/* Latest 20.48 s of samples at 80 ms, frozen on disarm; RAM only, never motor commands.
 * Read after CH5 OFF. Sequential live debugger reads are not atomic. */
typedef struct {
    uint32_t tick_ms, flags; /* bit0 active, bit1 wheel voltage/general saturation,
                            * bit2 wheel Nm saturation, bit3 Tp at cap,
                            * bit4 position locked; phase in bits8..15. */
    float state[6];
    float position_error_m, wheel_requested_nm, wheel_applied_nm, leg_tp_nm;
    float motion_component_nm[2]; /* wheel dtheta and dphi contributions. */
    int16_t wheel_voltage_mv[2];
    float leg_length_m;
    /* The original 64-byte prefix above remains unchanged. Diagnostic inputs
     * are the same SBUS snapshot consumed by motion_update, before gating. */
    uint16_t raw_ch2; /* UINT16_MAX when the consumed sticks were invalid. */
    uint8_t motion_status, input_flags;
    uint32_t source_frame_count, sbus_age_ms; /* UINT32_MAX age: no source frame. */
    float normalized_forward, requested_forward_m_s, slewed_forward_m_s;
    float reference[4]; /* Actual theta, x, dx, phi references: indices 0,2,3,4. */
    float parking_goal_m, bias_offset_m, bias_applied_phi_rad;
    uint32_t bias_gate_flags; /* Published observer gates, never recomputed here. */
    float requested_yaw_rad_s, slewed_yaw_rad_s;
    /* The original 128-byte prefix stays unchanged. The integral is captured
     * after z_k was consumed and z_(k+1) was prepared; the two I torque terms
     * below are the actual terms consumed by this tick's output calculation. */
    float servo_integral_next_m_s, servo_blend;
    float servo_wheel_integral_nm, servo_leg_integral_nm;
    uint32_t servo_motion_flags; /* gate mask bits0..4; flags below. */
    float leg_angle_rad[2], leg_angle_speed_rad_s[2], leg_lengths_m[2];
    float measured_leg_tp_nm; /* Iq/Jacobian estimate, not a load-cell measurement. */
    float parking_blend; /* Actual mix of complete parking K/Ki rows this tick. */
} WheelBalanceTraceRow;
enum {
    WHEEL_TRACE_INPUT_VALID=1U, WHEEL_TRACE_INPUT_CALIBRATED=2U,
    WHEEL_TRACE_REMOTE_ENABLED=4U, WHEEL_TRACE_CH5_ON=8U,
    WHEEL_TRACE_CONTROLLER_ON=16U, WHEEL_TRACE_NORMAL_BALANCE=32U,
    WHEEL_TRACE_NEUTRAL_READY=64U, WHEEL_TRACE_COMMAND_READY=128U
};
enum {
    WHEEL_TRACE_SERVO_GATE_MASK=31U,
    WHEEL_TRACE_SERVO_ENABLED=1U<<5, WHEEL_TRACE_SERVO_ACTIVE=1U<<6,
    WHEEL_TRACE_SERVO_LEARNING=1U<<7,
    WHEEL_TRACE_MOTION_PHASE_SHIFT=8U, WHEEL_TRACE_MOTION_PHASE_MASK=3U<<8,
    WHEEL_TRACE_MOTION_ENABLED=1U<<10, WHEEL_TRACE_JOINT_SATURATED=1U<<11,
    WHEEL_TRACE_SUPPORT_LEFT_SATURATED=1U<<12, WHEEL_TRACE_SUPPORT_RIGHT_SATURATED=1U<<13
};
#define WHEEL_BALANCE_TRACE_ROWS 256U
#define WHEEL_BALANCE_TRACE_PERIOD_MS 80U
typedef struct {
    uint32_t next, count, sequence;
    WheelBalanceTraceRow rows[WHEEL_BALANCE_TRACE_ROWS];
} WheelBalanceTraceDebug;
extern volatile WheelBalanceTraceDebug g_balance_trace;
void WheelLeg_Update(void);
void WheelLeg_NotifyCanTransportFault(void);
uint32_t WheelLeg_CanCommunicationStopped(void);
void WheelLeg_Task_Create(void);

#endif
