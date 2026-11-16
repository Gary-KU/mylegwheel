/* 5 ms 控制入口：采集反馈 -> 状态估计 -> 共享 LQR -> 双腿/双轮统一输出。
 * 轮子单测与完整控制互斥；g_* 同时提供运行参数和 Watch 诊断快照。 */
#include "wheel_leg_task.h"
#include "wheel_leg_config.h"
#include "leg_height_control.h"
#include "wheel_leg_observer.h"
#include "can_task.h"
#include "imu_task.h"
#include "joint_calibration.h"
#include "lqr_k.h"
#include "motor_command.h"
#include "sbus_receiver.h"
#include "robot_telemetry_service.h"
#include "lqr_startup_record.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#if ROBOT_NATIVE_LQR
#include "balance_bias_observer.h"
#include "balance_equilibrium.h"
#include "balance_motion_reference.h"
#include "balance_servo_gains.h"
#include "balance_parking_gains.h"
static BalanceBiasState balance_bias;
static BalanceMotionReferenceState balance_motion_reference;
static float balance_bias_applied_phi;
static uint32_t balance_bias_quiet_ms;
#endif
/* The gravity estimate stays in shadow while the coupled servo owns steady
 * correction. Manual pitch trim remains a rate-limited posture request. */
volatile BalanceBiasDebug g_balance_bias={
    .enable=ROBOT_NATIVE_LQR ? BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE : 0U,
    .shadow_enable=1U};
volatile BalanceParkingIntegralDebug g_balance_parking_integral={
    .enable=ROBOT_NATIVE_LQR ? BALANCE_NATIVE_PARKING_INTEGRAL_ENABLE : 0U};
volatile BalanceMotionReferenceDebug g_balance_motion_reference={
    .enable=ROBOT_NATIVE_LQR ? BALANCE_MOTION_REFERENCE_ENABLE : 0U};
volatile BalanceServoDebug g_balance_servo={
    .enable=ROBOT_NATIVE_LQR ? BALANCE_NATIVE_SERVO_ENABLE : 0U};
volatile float g_leg_height_integral_n[2];
volatile float g_balance_equilibrium_theta_rad,g_balance_equilibrium_tp_nm;

#define DEG_TO_RAD 0.017453292519943295f

/* Optional one-shot commissioning timer; zero at boot disables it.
 * Enforced in the control task, independently of debugger/host latency. */
volatile uint32_t g_robot_bench_limit_ms=ROBOT_CAPTURE_BUILD ? ROBOT_CAPTURE_LIMIT_MS : 0U;
volatile uint32_t g_robot_bench_elapsed_ms;
static uint32_t robot_bench_start_ms, robot_bench_running;
volatile WheelLegDebug g_wheel_leg;
volatile float g_wheel_leg_reference[2][6];
volatile RobotControlDebug g_robot_control = {
#if ROBOT_LEG_PD_TEST_SIDE >= 0
    .feedback_only = 0U,
#else
    .feedback_only = ROBOT_DEFAULT_FEEDBACK_ONLY,
#endif
    .startup_enable = ROBOT_DEFAULT_STARTUP_ENABLE,
    .remote_enable = ROBOT_DEFAULT_REMOTE_ENABLE,
    .remote_on_raw = ROBOT_REMOTE_DEFAULT_ON_RAW,
    .remote_center_raw = ROBOT_REMOTE_DEFAULT_CENTER_RAW,
    .remote_off_raw = ROBOT_REMOTE_DEFAULT_OFF_RAW,
    .remote_tolerance_raw = ROBOT_REMOTE_DEFAULT_TOLERANCE_RAW
};
static uint32_t robot_control_active;
static uint32_t remote_armed, remote_off_seen;
static RobotStartupPhase robot_startup_phase;
static float robot_startup_elapsed, robot_startup_settle, robot_startup_blend;
static float balance_pitch_trim;
volatile float g_leg_angle_integral_nm[2];
#if ROBOT_NATIVE_LQR
static float native_startup_theta_reference;
#endif
static float balance_parking_lean, balance_parking_speed;
static float balance_motion_lean;
static uint32_t balance_parking_settle_ms;
static uint32_t balance_motion_stall_ms;
static float balance_motion_damping;
static uint32_t balance_drive_running, balance_braking, balance_stop_settle_ms, balance_startup_recovery;
static uint32_t motion_neutral_ms, motion_armed, motion_yaw_braking;
static float motion_turn_applied;
static uint32_t balance_active, balance_position_locked;
static float balance_origin;
static volatile uint32_t can_transport_fault_pending;
#if ROBOT_LEG_VOLTAGE_PROBE
static float leg_voltage_probe_direction;
#endif

static float clamp_symmetric(float x, float limit)
{
    return x > limit ? limit : x < -limit ? -limit : x;
}

/* Subtract the band outside its edges: no force jump when crossing zero. */
static float continuous_deadband(float value, float band)
{
    return value>band ? value-band : value<-band ? value+band : 0.0f;
}

/* Positive-part spring with a continuous slope at the onset and transition. */
static float smooth_extension(float extension, float width)
{
    if (extension<=0) return 0;
    if (width>0 && extension<width) return extension*extension/(2.0f*width);
    return extension-.5f*width;
}

volatile DualHoldDebug g_dual_hold = {
    .target_m = LEG_DEFAULT_TARGET_M,
    .position_kp = LEG_DEFAULT_POSITION_KP, .speed_kp = LEG_DEFAULT_SPEED_KP,
    .speed_ki = LEG_DEFAULT_SPEED_KI,
    .max_speed_m_s = LEG_DEFAULT_MAX_SPEED_M_S, .filter_tau_s = LEG_DEFAULT_FILTER_TAU_S,
    .max_force_n = LEG_DEFAULT_MAX_FORCE_N,
    .feedforward_target_n = LEG_DEFAULT_SUPPORT_FORCE_N,
    .feedforward_rate_n_s = LEG_DEFAULT_SUPPORT_RATE_N_S,
    .overlength_kp_n_per_m = LEG_DEFAULT_OVERLENGTH_KP,
    .overlength_margin_m = LEG_DEFAULT_OVERLENGTH_MARGIN_M,
    .overlength_startup_s = LEG_DEFAULT_OVERLENGTH_STARTUP_S,
    .overlength_transition_m = LEG_DEFAULT_OVERLENGTH_TRANSITION_M,
    .length_error_deadband_m = LEG_DEFAULT_LENGTH_ERROR_DEADBAND_M,
    .length_speed_deadband_m_s = LEG_DEFAULT_LENGTH_SPEED_DEADBAND_M_S,
    .support_only = LEG_DEFAULT_SUPPORT_ONLY,
    .torque_ratio_nm_per_v = {JOINT_LF_TORQUE_RATIO, JOINT_LR_TORQUE_RATIO,
                             JOINT_RF_TORQUE_RATIO, JOINT_RR_TORQUE_RATIO},
    .voltage_limit_mv = JOINT_DEFAULT_VOLTAGE_LIMIT_MV,
    .back_emf_ratio = JOINT_DEFAULT_BACK_EMF_RATIO,
    .angle_kp_nm_per_rad = LEG_DEFAULT_ANGLE_KP_NM_PER_RAD,
    .angle_kd_nm_s_per_rad = LEG_DEFAULT_ANGLE_KD_NM_S_PER_RAD,
    .angle_limit_nm = LEG_DEFAULT_ANGLE_LIMIT_NM,
    .angle_reference_mode = LEG_DEFAULT_ANGLE_REFERENCE_MODE,
    .angle_target_rad = LEG_DEFAULT_ANGLE_TARGET_RAD,
    .angle_reference_rate_rad_s = LEG_DEFAULT_ANGLE_REFERENCE_RATE_RAD_S,
    .sync_ratio = LEG_DEFAULT_SYNC_RATIO,
    .sync_position_kp = LEG_DEFAULT_SYNC_POSITION_KP,
    .sync_speed_kp = LEG_DEFAULT_SYNC_SPEED_KP,
    .sync_max_speed_rad_s = LEG_DEFAULT_SYNC_MAX_SPEED_RAD_S,
    .sync_filter_tau_s = LEG_DEFAULT_SYNC_FILTER_TAU_S,
    .sync_limit_nm = LEG_DEFAULT_SYNC_LIMIT_NM,
    .sync_rate_nm_s = LEG_DEFAULT_SYNC_RATE_NM_S,
    .direction = {-1, -1, 1, 1},
    .load_enable = LEG_DEFAULT_LOAD_ENABLE,
    .load_ki_n_per_m_s = LEG_DEFAULT_LOAD_KI_N_PER_M_S,
    .load_limit_n = LEG_DEFAULT_LOAD_LIMIT_N,
    .load_rate_n_s = LEG_DEFAULT_LOAD_RATE_N_S,
    .sync_recovery_priority_enable = ROBOT_NATIVE_LQR ? 0U : LEG_DEFAULT_SYNC_RECOVERY_PRIORITY,
    .sync_priority_scale = 1.0f
};

volatile BalanceDebug g_balance = {
    .wheel_scale = BALANCE_DEFAULT_WHEEL_SCALE,
    .leg_scale = BALANCE_DEFAULT_LEG_SCALE,
    .leg_hold_ratio = BALANCE_DEFAULT_LEG_HOLD_RATIO,
    .wheel_limit_mv = BALANCE_DEFAULT_WHEEL_LIMIT_MV,
    .wheel_rate_scale = BALANCE_DEFAULT_WHEEL_RATE_SCALE,
    .pitch_trim_deg = BALANCE_DEFAULT_PITCH_TRIM_DEG,
#if ROBOT_NATIVE_LQR
    .parking_lean_enable = BALANCE_NATIVE_DRIFT_COMPENSATION_ENABLE,
    .parking_lean_gain_rad_per_m = BALANCE_DEFAULT_PARKING_LEAN_GAIN_RAD_M,
#endif
#if !ROBOT_NATIVE_LQR
    .back_emf_ratio = BALANCE_DEFAULT_BACK_EMF_RATIO,
    .startup_wheel_damping_nm_s_per_rad = ROBOT_STARTUP_WHEEL_DAMPING_NM_S_PER_RAD,
    .startup_wheel_limit_nm = ROBOT_STARTUP_WHEEL_TORQUE_LIMIT_NM,
    .position_window_m = BALANCE_DEFAULT_POSITION_WINDOW_M,
    .parking_correction_limit_m = BALANCE_DEFAULT_PARKING_CORRECTION_M,
    .parking_damping_gain_nm_per_m_s = BALANCE_DEFAULT_PARKING_DAMPING_NM_PER_M_S,
    .parking_lean_enable = BALANCE_DEFAULT_PARKING_LEAN_ENABLE,
    .parking_lean_gain_rad_per_m = BALANCE_DEFAULT_PARKING_LEAN_GAIN_RAD_M
#endif
};

static void balance_reference_clear(void)
{
#if ROBOT_NATIVE_LQR
    BalanceBias_ResetMotion(&balance_bias);
    BalanceMotionReference_Reset(&balance_motion_reference);
    g_balance_motion_reference.phase=0U;
    g_balance_motion_reference.velocity_m_s=g_balance_motion_reference.goal_m=0;
    g_balance_motion_reference.quiet_s=0;
    g_balance_servo.active=g_balance_servo.learning=g_balance_servo.gate_mask=0U;
    g_balance_servo.blend=g_balance_servo.integral_m_s=0;
    g_balance_servo.parking_blend=0;
    g_balance_servo.wheel_integral_nm=g_balance_servo.leg_integral_nm=0;
    balance_bias_applied_phi=0;
    balance_bias_quiet_ms=0U;
    g_balance_bias.applied_offset_m=g_balance_bias.applied_phi_rad=0;
    g_balance_bias.gate_mask=g_balance_bias.confidence_ms=0U;
    native_startup_theta_reference=0;
    g_balance_parking_integral.correction_m=0;
    g_balance_parking_integral.learning=g_balance_parking_integral.gate_mask=0U;
    g_balance_parking_integral.settle_ms=0U;
    if(g_balance_bias.reset_request && !g_robot_control.enable) {
        balance_bias=(BalanceBiasState){0};g_balance_bias.reset_request=0U;
    }
    g_balance_bias.learning=0U;
#endif
    balance_drive_running=balance_braking=balance_stop_settle_ms=0U;
    g_balance.motion_braking=0;balance_startup_recovery=0;g_balance.startup_recovery=0;
    balance_pitch_trim=0;
    g_leg_angle_integral_nm[0]=g_leg_angle_integral_nm[1]=0;
    balance_parking_lean=balance_parking_speed=0;
    balance_parking_settle_ms=0;
    balance_motion_stall_ms=0;balance_motion_damping=0;
    g_balance.motion_speed_error_m_s=g_balance.motion_damping_torque_nm=0;
    g_balance.motion_damping_active=g_balance.motion_lean_learning=g_balance.motion_stall_hold=0U;
    g_balance.parking_lean_rad=g_balance.parking_lean_filtered_speed_m_s=0;
    balance_motion_lean=0;g_balance.motion_lean_rad=0;
    g_balance.parking_lean_learning=0U;
    balance_active=balance_position_locked=0;
    balance_origin=0;
    g_balance_equilibrium_theta_rad=g_balance_equilibrium_tp_nm=0;
    g_balance.pitch_trim_applied_deg=0;
    g_balance.position_locked=0U;
    g_balance.parking_target_m=g_balance.parking_error_m=0;
    g_balance.parking_damping_torque_nm=0;g_balance.parking_damping_active=0U;
}

volatile WheelTestDebug g_wheel_test = {
    .leg_length_m = WHEEL_TEST_DEFAULT_LEG_LENGTH_M, .scale = WHEEL_TEST_DEFAULT_SCALE,
    .wheel_limit_mv = WHEEL_VOLTAGE_LIMIT_MV,
    .pitch_deadband_deg = WHEEL_TEST_DEFAULT_PITCH_DEADBAND_DEG,
    .rate_deadband_rad_s = WHEEL_TEST_DEFAULT_RATE_DEADBAND_RAD_S
};

volatile WheelOutputDebug g_wheel_output = {
    .torque_ratio_nm_per_v = {WHEEL_LEFT_TORQUE_RATIO, WHEEL_RIGHT_TORQUE_RATIO},
    .supply_voltage_v = WHEEL_SUPPLY_VOLTAGE_V
};

volatile RobotMotionDebug g_robot_motion = {
    .config={.enable=ROBOT_MOTION_DEFAULT_ENABLE,
        .max_forward_m_s=ROBOT_MOTION_DEFAULT_FORWARD_M_S,
        .max_turn_rad_s=ROBOT_MOTION_DEFAULT_YAW_RAD_S,
        .acceleration_m_s2=ROBOT_MOTION_DEFAULT_ACCEL_M_S2,
        .turn_acceleration_rad_s2=ROBOT_MOTION_DEFAULT_YAW_ACCEL_RAD_S2},
    .yaw_enable=ROBOT_MOTION_DEFAULT_YAW_ENABLE,
    .yaw_feedback_direction=ROBOT_MOTION_YAW_FEEDBACK_DIRECTION,
    .yaw_kp_nm_per_rad_s=ROBOT_MOTION_YAW_KP_NM_PER_RAD_S,
    .yaw_torque_limit_nm=ROBOT_MOTION_YAW_TORQUE_LIMIT_NM,
    .yaw_torque_rate_nm_s=ROBOT_MOTION_YAW_TORQUE_RATE_NM_S
};

/* Diagnostic-only copy of the input actually consumed by this control tick.
 * Retain it through motion_clear so rejected/held inputs stay observable. */
static struct {
    uint16_t raw_ch2;
    uint8_t flags;
    uint32_t source_frame_count, age_ms;
    float normalized_forward;
} motion_input_trace;

static void motion_clear(RobotMotionStatus status)
{
    const RobotMotionCommand zero={.status=status};
    g_robot_motion.command=zero;
    g_robot_motion.neutral_ready=g_robot_motion.translating=g_robot_motion.yaw_active=0U;
    g_robot_motion.measured_yaw_rate_rad_s=g_robot_motion.yaw_error_rad_s=0;
    g_robot_motion.filtered_yaw_rate_rad_s=0;
    g_robot_motion.yaw_integral_nm=0;g_robot_motion.yaw_integral_blocked=0U;
    g_robot_motion.differential_requested_nm=g_robot_motion.differential_applied_nm=0;
    motion_neutral_ms=motion_armed=motion_yaw_braking=0U;motion_turn_applied=0;
}

static void motion_update(const WheelLegDebug *state)
{
    const RobotMotionDebug c=g_robot_motion;
    SbusSticks sticks={0};
    (void)SBUS_GetSticks(&sticks);
    RobotMotionPermission permission={
        g_robot_control.remote_enable==1U,
        g_robot_control.remote_switch_on!=0U,
        g_robot_control.enable && g_robot_control.status==1U &&
            g_balance.enable && g_balance.wheel_scale>0 && g_balance.leg_scale>0 &&
            !g_wheel_test.enable && !g_robot_control.feedback_only && !g_lqr_record.storage_busy &&
            state->imu_valid,
        g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE
    };
    motion_input_trace.raw_ch2=sticks.valid ? sticks.raw_ch2 : UINT16_MAX;
    motion_input_trace.normalized_forward=sticks.valid ? sticks.forward : NAN;
    motion_input_trace.source_frame_count=sticks.frame_count;
    motion_input_trace.age_ms=sticks.frame_count ?
        (uint32_t)(HAL_GetTick()-sticks.received_ms) : UINT32_MAX;
    motion_input_trace.flags=(sticks.valid?WHEEL_TRACE_INPUT_VALID:0U) |
        (sticks.calibrated?WHEEL_TRACE_INPUT_CALIBRATED:0U) |
        (permission.remote_enabled?WHEEL_TRACE_REMOTE_ENABLED:0U) |
        (permission.ch5_on?WHEEL_TRACE_CH5_ON:0U) |
        (permission.controller_on?WHEEL_TRACE_CONTROLLER_ON:0U) |
        (permission.normal_balance?WHEEL_TRACE_NORMAL_BALANCE:0U);
    if(c.yaw_enable>1U || (c.yaw_feedback_direction!=1 && c.yaw_feedback_direction!=-1) ||
       !isfinite(c.yaw_kp_nm_per_rad_s) || c.yaw_kp_nm_per_rad_s<0 || c.yaw_kp_nm_per_rad_s>ROBOT_MOTION_YAW_KP_NM_PER_RAD_S ||
       !isfinite(c.yaw_torque_limit_nm) || c.yaw_torque_limit_nm<0 || c.yaw_torque_limit_nm>ROBOT_MOTION_YAW_TORQUE_LIMIT_NM ||
       !isfinite(c.yaw_torque_rate_nm_s) || c.yaw_torque_rate_nm_s<=0 || c.yaw_torque_rate_nm_s>ROBOT_MOTION_YAW_TORQUE_RATE_NM_S) {
        motion_clear(ROBOT_MOTION_BAD_PARAMETERS);return;
    }
    RobotMotionCommand next=c.command;
    RobotMotion_Update(&c.config,&sticks,&permission,state->tick_ms,state->period_ms,&next);
    if(next.status!=ROBOT_MOTION_TARGET_READY) { motion_clear(next.status);return; }
    /* 总开关解锁后，摇杆还须连续回中一段时间，避免带杆启动行驶。 */
    if(!motion_armed) {
        if(fabsf(sticks.forward)<=.05f && fabsf(sticks.yaw)<=.05f) {
            motion_neutral_ms+=state->period_ms;
            if(motion_neutral_ms>=ROBOT_MOTION_NEUTRAL_MS) motion_armed=1U;
        } else motion_neutral_ms=0U;
        if(!motion_armed) {
            const RobotMotionCommand zero={.status=ROBOT_MOTION_NOT_READY};
            g_robot_motion.command=zero;return;
        }
    }
    /* Limit commanded yaw using both requested and measured translation.
     * Re-slew from the previous yaw target, never reset the forward ramp. */
    float travel=fmaxf(fabsf(next.requested_forward_m_s),
        fabsf(.5f*(state->state[0][3]+state->state[1][3])));
    float yaw_blend=clamp_symmetric(travel/.4f,1.0f);
    float yaw_limit=fminf(c.config.max_turn_rad_s,
        ROBOT_MOTION_DEFAULT_YAW_RAD_S-(ROBOT_MOTION_DEFAULT_YAW_RAD_S-.55f)*yaw_blend);
    next.requested_turn_rad_s=sticks.yaw*yaw_limit;
    next.turn_rad_s=c.command.turn_rad_s+clamp_symmetric(
        next.requested_turn_rad_s-c.command.turn_rad_s,
        c.config.turn_acceleration_rad_s2*state->period_ms*.001f);
    if(!c.yaw_enable) next.turn_rad_s=next.requested_turn_rad_s=0;
    g_robot_motion.command=next;g_robot_motion.neutral_ready=1U;
    g_robot_motion.translating=fabsf(next.forward_m_s)>1e-6f ||
                              fabsf(next.requested_forward_m_s)>1e-6f;
    float yaw=c.yaw_feedback_direction*state->gyro_rad_s[2];
    if(!isfinite(yaw)) { motion_clear(ROBOT_MOTION_REMOTE_INVALID);return; }
    g_robot_motion.measured_yaw_rate_rad_s=yaw;
    float dt=state->period_ms*.001f;
    float filtered=c.filtered_yaw_rate_rad_s;
    if(!isfinite(filtered)) filtered=yaw;
    filtered+=dt/(ROBOT_MOTION_YAW_FILTER_S+dt)*(yaw-filtered);
    g_robot_motion.filtered_yaw_rate_rad_s=filtered;
    unsigned commanded=g_robot_motion.translating || fabsf(next.turn_rad_s)>1e-6f ||
                       fabsf(next.requested_turn_rad_s)>1e-6f;
    if(c.yaw_enable && commanded) motion_yaw_braking=1U;
    else if(fabsf(filtered)<=ROBOT_MOTION_YAW_RATE_DEADBAND_RAD_S) motion_yaw_braking=0U;
    /* Also reject uncommanded yaw after neutral arming, before any stick movement. */
    unsigned yaw_active=c.yaw_enable && (commanded || motion_yaw_braking ||
        fabsf(filtered)>ROBOT_MOTION_YAW_RATE_DEADBAND_RAD_S);
    float error=next.turn_rad_s-filtered;
    float shaped=fabsf(error)<=ROBOT_MOTION_YAW_RATE_DEADBAND_RAD_S ? 0 :
        error-copysignf(ROBOT_MOTION_YAW_RATE_DEADBAND_RAD_S,error);
    float integral=c.yaw_integral_nm;
    unsigned turn_requested=fabsf(next.requested_turn_rad_s)>1e-6f;
    unsigned released=fabsf(c.command.requested_turn_rad_s)>1e-6f && !turn_requested;
    unsigned reversed=next.requested_turn_rad_s*c.command.requested_turn_rad_s<0;
    /* Drop stored drive torque when the user releases/reverses or feedback
     * crosses the target. Slewing the final command keeps the change bounded. */
    if(!isfinite(integral) || !yaw_active || released || reversed ||
       (!turn_requested && !g_robot_motion.translating) || integral*shaped<0)
        integral=0;
    float proportional=c.yaw_kp_nm_per_rad_s*shaped;
    float total=proportional+integral;
    unsigned blocked=(total*shaped>0 && fabsf(total)>=c.yaw_torque_limit_nm) ||
        ((total-motion_turn_applied)*shaped>1e-8f &&
         fabsf(total-motion_turn_applied)>c.yaw_torque_rate_nm_s*dt) ||
        ((c.differential_requested_nm-c.differential_applied_nm)*shaped>1e-8f);
    /* 用上一周期实际分配的转向力矩做抗饱和：平衡输出占满余量时暂停积分。 */
    if(yaw_active && (turn_requested || g_robot_motion.translating) &&
       !released && !reversed && !blocked)
        integral=clamp_symmetric(integral+ROBOT_MOTION_YAW_KI_NM_PER_RAD*shaped*dt,
                                ROBOT_MOTION_YAW_INTEGRAL_LIMIT_NM);
    g_robot_motion.yaw_integral_nm=integral;
    g_robot_motion.yaw_integral_blocked=yaw_active && blocked;
    float request=yaw_active ? clamp_symmetric(proportional+integral,c.yaw_torque_limit_nm) : 0;
    if(!yaw_active) motion_turn_applied=0;
    else motion_turn_applied+=clamp_symmetric(request-motion_turn_applied,
                                              c.yaw_torque_rate_nm_s*state->period_ms*.001f);
    motion_turn_applied=clamp_symmetric(motion_turn_applied,c.yaw_torque_limit_nm);
    g_robot_motion.yaw_active=yaw_active;g_robot_motion.yaw_error_rad_s=yaw_active?error:0;
    g_robot_motion.differential_requested_nm=motion_turn_applied;
}

volatile JointOutputDebug g_joint_output;

static void joint_output_clear(void)
{
    const JointOutputDebug zero={0};
    g_joint_output=zero;
}

/* ESP32 ADC_Task fallback and common voltage-dependent output ratio.
 * The H723 uses a configured supply value rather than an unconnected ADC. */
static int supply_model(float supply, float *effective, float *ratio)
{
    if (!isfinite(supply) || supply<0) return 0;
    *effective=supply<8 ? 12 : supply;
    *ratio=(12-*effective)/10.0f+0.7f;
    return isfinite(*ratio) && *ratio>0;
}

static int joint_output_prepare(const WheelLegDebug *state, const DualHoldDebug *c,
                                JointOutputDebug *out)
{
    unsigned legacy=0U;
    for(unsigned i=0;i<4;++i) legacy|=!MotorCommand_UsesCurrent((uint8_t)g_joint_calibration[i].motor_id);
    if(legacy) {
        if (!supply_model(g_wheel_output.supply_voltage_v,
                          &out->effective_supply_voltage_v,&out->output_ratio)) return 0;
    } else out->output_ratio=1.0f;
    for (unsigned i=0;i<4;++i) {
        const JointCalibration *cal=&g_joint_calibration[i];
        unsigned current=MotorCommand_UsesCurrent((uint8_t)cal->motor_id);
        if (!isfinite(cal->ratio) || cal->ratio<=0 ||
            (cal->direction!=1 && cal->direction!=-1)) return 0;
        if(current) {
            if(MotorCommand_CurrentTorqueLimitNm((uint8_t)cal->motor_id)<=0) return 0;
            out->torque_mv_per_nm[i]=0;
        } else {
            if(!isfinite(c->torque_ratio_nm_per_v[i]) || c->torque_ratio_nm_per_v[i]<=0) return 0;
            out->torque_mv_per_nm[i]=1000.0f*out->output_ratio/c->torque_ratio_nm_per_v[i];
        }
        /* State rates are model joint rates, rear/front. Undo encoder
         * calibration to get shaft speed, then express it in the same
         * coordinate as requested torque. Final voltage sign is applied once. */
        float speed=state->joint_speed_rad_s[i/2][1-i%2]*cal->ratio*
                    (float)cal->direction*(float)c->direction[i];
        float s=fabsf(speed);
        float emf=0;
        /* A disabled feedforward term must be true zero, even at rates that
         * would overflow the empirical polynomial. Speed validity still holds. */
        if (!current && c->back_emf_ratio>0)
            emf=c->back_emf_ratio*(0.00008f*s*s*s-0.0035f*s*s+0.2322f*s);
        out->speed_rad_s[i]=speed;
        out->back_emf_mv[i]=1000.0f*(speed<0 ? -emf : emf);
        if (!isfinite(out->torque_mv_per_nm[i]) || (!current && out->torque_mv_per_nm[i]<=0) ||
            !isfinite(speed) || !isfinite(out->back_emf_mv[i])) return 0;
    }
    return 1;
}

static void wheel_output_clear(void)
{
    g_wheel_output.effective_supply_voltage_v=0;
    g_wheel_output.output_ratio=0; g_wheel_output.back_emf_blend=0;
    g_wheel_output.saturated=0U;
    g_wheel_output.requested_torque_nm=g_wheel_output.applied_torque_nm=0;
    g_wheel_output.torque_saturated=0U;
    g_wheel_output.requested_differential_nm=g_wheel_output.applied_differential_nm=0;
    for (unsigned side=0;side<2;++side) {
        g_wheel_output.torque_voltage_mv[side]=0;
        g_wheel_output.requested_per_side_nm[side]=g_wheel_output.applied_per_side_nm[side]=0;
        g_wheel_output.back_emf_mv[side]=0; g_wheel_output.requested_mv[side]=0;
        g_wheel_output.speed_rad_s[side]=0; g_wheel_output.speed_valid[side]=0U;
        g_wheel_output.voltage_mv[side]=0;
        g_wheel_output.requested_iq_a[side]=0;g_wheel_output.current_backend[side]=0U;
    }
}

static int wheel_output_pair(const WheelLegDebug *state, float torque, float differential, float limit,
                        float emf_blend, int16_t voltage[8])
{
    const WheelOutputDebug c=g_wheel_output;
    const int encoder_direction[2]={WHEEL_LEFT_DIRECTION,WHEEL_RIGHT_DIRECTION};
    const int voltage_direction[2]={WHEEL_LEFT_VOLTAGE_DIRECTION,WHEEL_RIGHT_VOLTAGE_DIRECTION};
    const unsigned id[2]={WHEEL_LEFT_CAN_ID,WHEEL_RIGHT_CAN_ID};
    WheelOutputDebug out={0};
    unsigned legacy_wheel=!MotorCommand_UsesCurrent((uint8_t)id[0]) || !MotorCommand_UsesCurrent((uint8_t)id[1]);
    wheel_output_clear();
    if (!isfinite(torque) || !isfinite(differential) || fabsf(differential)>ROBOT_MOTION_YAW_TORQUE_LIMIT_NM+1e-7f ||
        (legacy_wheel && (!isfinite(limit) || limit<=0 || limit>WHEEL_VOLTAGE_LIMIT_MV ||
        !isfinite(c.supply_voltage_v) || c.supply_voltage_v<0 ||
        !isfinite(emf_blend) || emf_blend<0 || emf_blend>1))) return 0;
    out.back_emf_blend=legacy_wheel?emf_blend:0;
    out.requested_torque_nm=torque;
    out.applied_torque_nm=WHEEL_REQUEST_TORQUE_LIMIT_ENABLED ?
        clamp_symmetric(torque,WHEEL_TORQUE_LIMIT_NM) : torque;
    out.torque_saturated=WHEEL_REQUEST_TORQUE_LIMIT_ENABLED && fabsf(torque)>WHEEL_TORQUE_LIMIT_NM;
    for(unsigned side=0;side<2;++side) if(MotorCommand_UsesCurrent((uint8_t)id[side])) {
        float cap=MotorCommand_CurrentTorqueLimitNm((uint8_t)id[side]);
        if(!isfinite(cap) || cap<=0) return 0;
        if(fabsf(out.applied_torque_nm)>cap) {
            out.applied_torque_nm=clamp_symmetric(out.applied_torque_nm,cap);out.torque_saturated=1U;
        }
    }
    out.saturated=out.torque_saturated;
    torque=out.applied_torque_nm;
    if(!MotorCommand_UsesCurrent((uint8_t)id[0]) || !MotorCommand_UsesCurrent((uint8_t)id[1])) {
        if (!supply_model(c.supply_voltage_v,&out.effective_supply_voltage_v,&out.output_ratio)) return 0;
    } else out.output_ratio=1.0f;
    for (unsigned side=0;side<2;++side) {
        if(MotorCommand_UsesCurrent((uint8_t)id[side])) {
            out.current_backend[side]=1U;
            out.speed_valid[side]=state->wheel_valid[side] && isfinite(state->wheel_speed_rad_s[side]);
            if(out.speed_valid[side]) out.speed_rad_s[side]=encoder_direction[side]*state->wheel_speed_rad_s[side];
            continue; /* 电流后端由 F103 电流 PI 处理电压与反电动势，无需 H7 再补偿。 */
        }
        if (!isfinite(c.torque_ratio_nm_per_v[side]) || c.torque_ratio_nm_per_v[side]<=0) return 0;
        out.torque_voltage_mv[side]=1000.0f*(torque/c.torque_ratio_nm_per_v[side]*out.output_ratio);
        out.speed_valid[side]=state->wheel_valid[side] && isfinite(state->wheel_speed_rad_s[side]);
        if (out.speed_valid[side]) {
            float speed=encoder_direction[side]*state->wheel_speed_rad_s[side];
            float s=fabsf(speed);
            out.speed_rad_s[side]=speed;
            if (emf_blend>0) {
                /* ESP32 Motor_CalcRevVolt2804, evaluated at absolute rad/s. */
                float emf=0.000004f*s*s*s-0.0003f*s*s+0.0266f*s;
                out.back_emf_mv[side]=emf_blend*1000.0f*(speed<0 ? -emf : emf);
            }
        }
        out.requested_mv[side]=out.torque_voltage_mv[side]+out.back_emf_mv[side];
        if (!isfinite(out.torque_voltage_mv[side]) || !isfinite(out.back_emf_mv[side]) ||
            !isfinite(out.requested_mv[side])) return 0;
        if (fabsf(out.requested_mv[side])>limit) out.saturated=1U;
        out.voltage_mv[side]=(int16_t)(voltage_direction[side]*clamp_symmetric(out.requested_mv[side],limit));
    }
    /* 先满足共同平衡力矩，再用两侧都可用的电压/力矩余量分配差动转向。
     * 平衡请求已饱和时转向归零，避免转向挤占维持平衡的输出。 */
    float headroom=ROBOT_MOTION_YAW_TORQUE_LIMIT_NM;
    if(WHEEL_REQUEST_TORQUE_LIMIT_ENABLED)
        headroom=fminf(headroom,fmaxf(0,WHEEL_TORQUE_LIMIT_NM-fabsf(out.applied_torque_nm)));
    for(unsigned side=0;side<2;++side) {
        if(out.current_backend[side]) {
            headroom=fminf(headroom,fmaxf(0,MotorCommand_CurrentTorqueLimitNm((uint8_t)id[side])-fabsf(torque)));
            continue;
        }
        float gain=1000.0f*out.output_ratio/c.torque_ratio_nm_per_v[side];
        headroom=fminf(headroom,fmaxf(0,(limit-fabsf(out.requested_mv[side]))/gain));
    }
    out.requested_differential_nm=differential;
    out.applied_differential_nm=clamp_symmetric(differential,headroom);
    if(fabsf(differential)>headroom) out.saturated=1U;
    for(unsigned side=0;side<2;++side) {
        float direction=side?-1.0f:1.0f;
        out.requested_per_side_nm[side]=out.requested_torque_nm+direction*differential;
        out.applied_per_side_nm[side]=out.applied_torque_nm+direction*out.applied_differential_nm;
        if(out.current_backend[side]) {
            out.requested_iq_a[side]=MotorCommand_CurrentForTorque((uint8_t)id[side],out.applied_per_side_nm[side]);
        } else if(out.applied_differential_nm!=0) {
            out.torque_voltage_mv[side]+=direction*out.applied_differential_nm*
                                         (1000.0f*out.output_ratio/c.torque_ratio_nm_per_v[side]);
            out.requested_mv[side]=out.torque_voltage_mv[side]+out.back_emf_mv[side];
            out.voltage_mv[side]=(int16_t)(voltage_direction[side]*clamp_symmetric(out.requested_mv[side],limit));
        }
    }
    /* Publish/apply only after both sides pass numeric validation. */
    g_wheel_output.effective_supply_voltage_v=out.effective_supply_voltage_v;
    g_wheel_output.output_ratio=out.output_ratio; g_wheel_output.saturated=out.saturated;
    g_wheel_output.back_emf_blend=out.back_emf_blend;
    g_wheel_output.requested_torque_nm=out.requested_torque_nm;
    g_wheel_output.applied_torque_nm=out.applied_torque_nm;
    g_wheel_output.torque_saturated=out.torque_saturated;
    g_wheel_output.requested_differential_nm=out.requested_differential_nm;
    g_wheel_output.applied_differential_nm=out.applied_differential_nm;
    for (unsigned side=0;side<2;++side) {
        voltage[id[side]-1U]=out.voltage_mv[side];
        g_wheel_output.requested_per_side_nm[side]=out.requested_per_side_nm[side];
        g_wheel_output.applied_per_side_nm[side]=out.applied_per_side_nm[side];
        g_wheel_output.torque_voltage_mv[side]=out.torque_voltage_mv[side];
        g_wheel_output.back_emf_mv[side]=out.back_emf_mv[side];
        g_wheel_output.requested_mv[side]=out.requested_mv[side];
        g_wheel_output.speed_rad_s[side]=out.speed_rad_s[side];
        g_wheel_output.speed_valid[side]=out.speed_valid[side];
        g_wheel_output.voltage_mv[side]=out.voltage_mv[side];
        g_wheel_output.requested_iq_a[side]=out.requested_iq_a[side];
        g_wheel_output.current_backend[side]=out.current_backend[side];
    }
    return 1;
}

static int wheel_output(const WheelLegDebug *state,float torque,float limit,
                        float emf_blend,int16_t voltage[8])
{
    return wheel_output_pair(state,torque,0,limit,emf_blend,voltage);
}

/* Validate Watch parameters separately from control calculations. */
#if !ROBOT_NATIVE_LQR
static int balance_parameters_valid(const BalanceDebug *config)
{
    const BalanceDebug *c = config;
    if (!isfinite(c->wheel_scale) || c->wheel_scale < 0 || c->wheel_scale > 1 ||
        !isfinite(c->wheel_rate_scale) || c->wheel_rate_scale < 0 || c->wheel_rate_scale > 1 ||
        !isfinite(c->leg_scale) || c->leg_scale < 0 || c->leg_scale > 1 ||
        !isfinite(c->leg_hold_ratio) || c->leg_hold_ratio < 0 || c->leg_hold_ratio > 1 ||
        !isfinite(c->wheel_limit_mv) || c->wheel_limit_mv <= 0 ||
        c->wheel_limit_mv > WHEEL_VOLTAGE_LIMIT_MV ||
        !isfinite(c->back_emf_ratio) || c->back_emf_ratio<0 || c->back_emf_ratio>1 ||
        !isfinite(c->startup_wheel_damping_nm_s_per_rad) || c->startup_wheel_damping_nm_s_per_rad<0 ||
        c->startup_wheel_damping_nm_s_per_rad>0.02f ||
        !isfinite(c->startup_wheel_limit_nm) || c->startup_wheel_limit_nm<0 ||
        c->startup_wheel_limit_nm>0.02f ||
        !isfinite(c->position_window_m) || c->position_window_m<0 || c->position_window_m>0.5f ||
        !isfinite(c->parking_correction_limit_m) || c->parking_correction_limit_m<0 ||
        c->parking_correction_limit_m>0.1f ||
        !isfinite(c->pitch_trim_deg) || fabsf(c->pitch_trim_deg)>10.0f ||
        !isfinite(c->parking_damping_gain_nm_per_m_s) || c->parking_damping_gain_nm_per_m_s<0 ||
        c->parking_damping_gain_nm_per_m_s>BALANCE_PARKING_DAMPING_MAX_GAIN ||
        c->parking_lean_enable>1U || !isfinite(c->parking_lean_gain_rad_per_m) ||
        c->parking_lean_gain_rad_per_m<0 ||
        c->parking_lean_gain_rad_per_m>BALANCE_PARKING_LEAN_MAX_GAIN_RAD_M) return 0;
    return 1;
}
#endif

static int dual_parameters_valid(const DualHoldDebug *config)
{
    const DualHoldDebug *c = config;
    if (c->support_only>1U || c->load_enable>1U || c->sync_recovery_priority_enable>1U) return 0;
    if (c->angle_reference_mode!=LEG_ANGLE_CAPTURE && c->angle_reference_mode!=LEG_ANGLE_NEUTRAL) return 0;
    const float params[] = {c->target_m,c->position_kp,c->speed_kp,c->speed_ki,c->max_speed_m_s,
        c->filter_tau_s,c->max_force_n,c->feedforward_target_n,c->feedforward_rate_n_s,c->voltage_limit_mv,c->back_emf_ratio,
        c->overlength_kp_n_per_m,c->overlength_margin_m,c->overlength_startup_s,
        c->overlength_transition_m,c->length_error_deadband_m,c->length_speed_deadband_m_s,
        c->angle_kp_nm_per_rad,c->angle_kd_nm_s_per_rad,c->angle_limit_nm,
        c->angle_target_rad,c->angle_reference_rate_rad_s,
        c->sync_ratio,c->sync_position_kp,c->sync_speed_kp,c->sync_max_speed_rad_s,
        c->sync_filter_tau_s,c->sync_limit_nm,c->sync_rate_nm_s,
        c->load_ki_n_per_m_s,c->load_limit_n,c->load_rate_n_s};
    for (unsigned i = 0; i < sizeof(params)/sizeof(params[0]); ++i)
        if (!isfinite(params[i]) || params[i] < 0) return 0;
    for (unsigned i = 0; i < 4; ++i) if (c->direction[i] != 1 && c->direction[i] != -1) return 0;
    if (c->target_m < .04f || c->target_m > .14f || c->max_speed_m_s <= 0 || c->max_force_n <= 0 ||
        c->feedforward_target_n > c->max_force_n || c->feedforward_rate_n_s <= 0 ||
        c->overlength_margin_m > .02f || c->overlength_startup_s>5.0f ||
        c->overlength_transition_m>.01f || c->length_error_deadband_m>.005f ||
        c->length_speed_deadband_m_s>.02f ||
        c->angle_target_rad<=0 || c->angle_target_rad>=3.141592653589793f ||
        c->angle_reference_rate_rad_s<=0 || c->angle_reference_rate_rad_s>1.0f ||
        c->sync_ratio>1 || c->sync_position_kp>50 || c->sync_speed_kp>0.2f ||
        c->sync_max_speed_rad_s<=0 || c->sync_max_speed_rad_s>20 ||
        c->sync_filter_tau_s>0.1f || c->sync_limit_nm>0.3f ||
        c->sync_ratio*c->sync_limit_nm>0.2f ||
        c->sync_rate_nm_s<=0 || c->sync_rate_nm_s>2 ||
        c->voltage_limit_mv <= 0 || c->voltage_limit_mv > JOINT_VOLTAGE_LIMIT_MV || c->back_emf_ratio>1) return 0;
    if (c->load_ki_n_per_m_s>100 || c->load_limit_n>5 || c->load_rate_n_s>1) return 0;
    if (c->load_enable && (!c->support_only || c->speed_ki!=0 ||
        c->load_ki_n_per_m_s<=0 || c->load_limit_n<=0 || c->load_rate_n_s<=0)) return 0;
    return 1;
}

#if ROBOT_NATIVE_LQR
static uint32_t measured_leg_pitch(const WheelLegDebug *state,float *mean_tp)
{
    float torque[4];*mean_tp=0;
    for(unsigned i=0;i<4;++i) {
        if(!MotorCommand_MeasuredTorqueNm((uint8_t)g_joint_calibration[i].motor_id,&torque[i]))return 0U;
        torque[i]*=g_joint_calibration[i].ratio;
    }
    for(unsigned side=0;side<2;++side) {
        const float *j=state->leg[side].jacobian;
        float det=j[0]*j[3]-j[1]*j[2];
        if(!isfinite(det) || fabsf(det)<1e-7f)return 0U;
        *mean_tp+=.5f*(j[0]*torque[2*side]-j[1]*torque[2*side+1])/det;
    }
    return isfinite(*mean_tp);
}
/* 共同矢状面控制：左右状态取均值，差动腿角误差交给后面的同步控制。
 * x = [theta, dtheta, travel, speed, phi, dphi]，单位为 rad、rad/s、m、m/s。
 * theta 是世界竖直方向的虚拟腿倾角；phi = -body_pitch，符号在 observer 中统一。
 * 输出 u[0] 为每侧共同轮矩 T，u[1] 为每侧共同虚拟腿摆矩 Tp，单位均为 N*m。
 * 两行 -K(x-reference) 必须一起保留；实际关节电流限幅在 VMC/CAN 适配层执行。 */
static void balance_update(const WheelLegDebug *state, uint32_t dual_active)
{
    const BalanceDebug c=g_balance;
    float x[6]={0}, reference[6]={0}, u[2]={0}, K[12]={0};
    float length=0, velocity=0;
    float integral_gain[2]={0};
    uint32_t status=0;
    uint32_t motion_braking=0U;
    if(!c.enable || !dual_active) goto publish_native;
    status=2U;
    if(!state->period_ms || state->period_ms>10U) goto publish_native;
    for(unsigned side=0;side<2;++side)
        if(!state->state_valid[side] || !state->lqr_valid[side]) goto publish_native;
    for(unsigned i=0;i<6;++i) x[i]=.5f*state->state[0][i]+.5f*state->state[1][i];
    if(!balance_active) {
        /* 一次启动只捕获一次停车位置；后续被推走/自行滑动必须产生位置误差。
         * OFF、失联或故障会清掉 balance_active，下一次启动重新捕获。 */
        balance_origin=x[2];balance_position_locked=1U;
        balance_drive_running=0U;
        native_startup_theta_reference=robot_control_active && g_robot_control.startup_enable ? x[0] : 0;
    }
    if(g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY)
        velocity=g_robot_motion.command.forward_m_s;
    if(!isfinite(velocity) || g_balance_motion_reference.enable>1U ||
       g_balance_servo.enable>1U || !isfinite(g_balance_servo.integral_m_s) ||
       !isfinite(g_balance_servo.blend) || !isfinite(g_balance_servo.parking_blend) ||
       g_balance_servo.parking_blend<0 || g_balance_servo.parking_blend>1) {
        status=3U;goto publish_native;
    }
    unsigned driving=fabsf(velocity)>1e-6f;
    /* Driving retains the local integrated reference. Parking instead keeps
     * a true fixed goal: bounding the LQR input must not rewrite that goal.
     * Otherwise position_locked can remain true while the goal follows every
     * uncommanded displacement beyond5cm. */
    if(g_balance_motion_reference.enable) {
        const BalanceMotionReferenceConfig mc={
            .max_speed_m_s=ROBOT_MOTION_DEFAULT_FORWARD_M_S,
            .acceleration_m_s2=BALANCE_MOTION_ACCELERATION_M_S2,
            .brake_acceleration_m_s2=BALANCE_MOTION_BRAKE_ACCELERATION_M_S2,
            .stop_speed_m_s=BALANCE_MOTION_STOP_SPEED_M_S,
            .stop_dwell_s=BALANCE_MOTION_STOP_DWELL_S,
            .tracking_window_m=ROBOT_MOTION_TRACK_WINDOW_M};
        BalanceMotionReferenceOutput mo;
        float requested=g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY ?
            g_robot_motion.command.requested_forward_m_s : 0;
        /* Rejected input clears velocity immediately, as RobotMotion_Update
         * promises. Preserve the true parking goal even during PREPARE. */
        if(!balance_motion_reference.initialized ||
           g_robot_motion.command.status!=ROBOT_MOTION_TARGET_READY) {
            BalanceMotionReference_Reset(&balance_motion_reference);
            balance_motion_reference.initialized=1U;
            balance_motion_reference.goal_m=balance_origin;
        }
        if(!BalanceMotionReference_Update(&balance_motion_reference,&mc,x[2],x[3],
                requested,velocity,state->period_ms*.001f,&mo)) {
            status=3U;goto publish_native;
        }
        velocity=mo.velocity_m_s;balance_origin=mo.goal_m;
        balance_position_locked=mo.position_locked;motion_braking=mo.braking;
        driving=fabsf(velocity)>1e-6f;
        g_balance_motion_reference.phase=balance_motion_reference.phase;
        g_balance_motion_reference.velocity_m_s=velocity;
        g_balance_motion_reference.goal_m=balance_origin;
        g_balance_motion_reference.quiet_s=balance_motion_reference.quiet_s;
    } else if(driving) {
        BalanceMotionReference_Reset(&balance_motion_reference);
        if(!balance_drive_running) balance_origin=x[2];
        balance_origin+=velocity*state->period_ms*.001f;
        balance_origin=x[2]+clamp_symmetric(balance_origin-x[2],ROBOT_MOTION_TRACK_WINDOW_M);
        balance_position_locked=0U;
    } else if(balance_drive_running) {
        /* Capture once as the slewed velocity request reaches zero. Measured
         * coasting after this frame changes the error, never the fixed goal. */
        balance_origin=x[2];balance_position_locked=1U;
    } else balance_position_locked=1U;
    balance_drive_running=driving;
    /* Neutral braking/parking feeds the true fixed goal to the controller.
     * Only ordinary driving retains the local trajectory window. In
     * particular, no30cm cap hides continued uncommanded displacement. */
    float position_window=driving ? ROBOT_MOTION_TRACK_WINDOW_M :
        (g_balance_servo.enable ? BALANCE_SERVO_POSITION_WINDOW_M : BALANCE_NATIVE_PARKING_WINDOW_M);
    unsigned servo_tracking=g_balance_motion_reference.enable ?
        balance_motion_reference.phase==BALANCE_MOTION_TRACKING : driving;
    reference[2]=g_balance_servo.enable && !servo_tracking ? balance_origin :
        x[2]+clamp_symmetric(balance_origin-x[2],position_window);
    reference[3]=velocity;
    length=.5f*(state->leg[0].length+state->leg[1].length);
    if(!isfinite(c.leg_scale) || c.leg_scale<0 || c.leg_scale>2.0f ||
       !isfinite(c.pitch_trim_deg) || fabsf(c.pitch_trim_deg)>BALANCE_NATIVE_PITCH_TRIM_LIMIT_DEG ||
       c.parking_lean_enable>1U || !isfinite(c.parking_lean_gain_rad_per_m) ||
       c.parking_lean_gain_rad_per_m<0 || c.parking_lean_gain_rad_per_m>BALANCE_PARKING_LEAN_MAX_GAIN_RAD_M ||
       g_balance_parking_integral.enable>1U) {
        status=3U;goto publish_native;
    }
    /* Qualify the initial pose at zero manual trim. Moving the equilibrium
     * during PREPARE can outrun a leg near the5deg readiness boundary before
     * its reference slew and250ms dwell complete. Full T/Tp remain active;
     * the configured trim starts at the original rate only after BALANCE. */
    if(robot_control_active && g_robot_control.startup_enable &&
       g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE)
        balance_pitch_trim=0;
    else balance_pitch_trim+=clamp_symmetric(c.pitch_trim_deg-balance_pitch_trim,
        BALANCE_PITCH_TRIM_RATE_DEG_S*state->period_ms*.001f);
    float actual_tp=NAN;
    uint32_t measured=measured_leg_pitch(state,&actual_tp);
    /* 重力偏置估计依赖 Kt/方向/几何正确，actual_tp 是由 Iq
     * 推算的力矩，不是独立力传感器的测量；静止条件满足也不代表已完成标定。 */
    if(g_balance_servo.enable && (g_balance_bias.enable || g_balance_parking_integral.enable)) {
        status=3U;goto publish_native; /* Exactly one owner of steady correction. */
    }
    uint32_t estimator=g_balance_bias.enable==1U || g_balance_bias.shadow_enable==1U;
    uint32_t gates=estimator?1U:0U;
    if(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE) gates|=2U;
    if(balance_position_locked && fabsf(velocity)<.01f) gates|=4U;
    if(fabsf(state->body_pitch_rad)<.139626340f && fabsf(x[5])<.12f &&
       fabsf(x[0])<.174532925f && fabsf(x[1])<.15f && fabsf(x[3])<.04f) gates|=8U;
    if(fabsf(state->leg[0].angle-state->leg[1].angle)<.087266463f) gates|=16U;
    if(fabsf(g_dual_hold.feedforward_n-g_dual_hold.feedforward_target_n)<.05f) gates|=32U;
    if(!g_wheel_output.saturated && !g_wheel_output.torque_saturated && !g_joint_output.saturated) gates|=64U;
    if(measured) gates|=128U;
    if(g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY && g_robot_motion.neutral_ready &&
       fabsf(g_robot_motion.command.forward_axis)<.05f && fabsf(g_robot_motion.command.turn_axis)<.05f &&
       fabsf(g_robot_motion.command.requested_forward_m_s)<.01f &&
       fabsf(g_robot_motion.command.requested_turn_rad_s)<.02f &&
       fabsf(g_robot_motion.command.turn_rad_s)<.02f) gates|=256U;
    if((gates&511U)==511U) {
        if(balance_bias_quiet_ms<1500U) balance_bias_quiet_ms+=state->period_ms;
    } else balance_bias_quiet_ms=0U;
    if(balance_bias_quiet_ms>=300U) gates|=512U;
    if(balance_bias_quiet_ms>=1500U) gates|=1024U;
    if(measured && estimator) {
        g_balance_bias.learning=BalanceBias_Update(&balance_bias,x,length,actual_tp,
            state->period_ms*.001f,(gates&1023U)==1023U);
        if(balance_bias.initialized && fabsf(balance_bias.acceleration[0])<.5f &&
           fabsf(balance_bias.acceleration[1])<3 && fabsf(balance_bias.acceleration[2])<3) gates|=2048U;
        else {
            balance_bias_quiet_ms=0U;
            gates&=~(512U|1024U);
        }
        g_balance_bias.status=g_balance_bias.learning?1U:2U;
    } else {
        if(estimator) BalanceBias_ResetMotion(&balance_bias);
        else balance_bias=(BalanceBiasState){0};
        g_balance_bias.learning=0U;
        g_balance_bias.status=measured?0U:3U;
    }
    /* Keep observation and application separate. Motion, clipping, missing
     * feedback or lost confidence freezes BOTH estimate and application.
     * Boot enable never bypasses the quiet-data and confidence gates. */
    if(g_balance_bias.enable!=1U) balance_bias_applied_phi=0;
    else if(g_balance_bias.learning && (gates&4095U)==4095U && balance_bias.samples>=200U)
        balance_bias_applied_phi+=clamp_symmetric(balance_bias.phi_reference_rad-balance_bias_applied_phi,
            .004363323f*state->period_ms*.001f);
    float applied_offset=LQR_BODY_COM_HEIGHT_M*balance_bias_applied_phi;
    g_balance_bias.gate_mask=gates;
    g_balance_bias.confidence_ms=balance_bias_quiet_ms;
    g_balance_bias.applied_phi_rad=balance_bias_applied_phi;
    g_balance_bias.applied_offset_m=applied_offset;
    reference[4]=balance_bias_applied_phi-balance_pitch_trim*DEG_TO_RAD;
    float equilibrium_theta=0,equilibrium_tp=0;
    if(!BalanceEquilibrium_CalculateWithOffset(length,reference[4],applied_offset,&equilibrium_theta,&equilibrium_tp)) {
        status=3U;goto publish_native;
    }
    g_balance_equilibrium_theta_rad=equilibrium_theta;
    g_balance_equilibrium_tp_nm=equilibrium_tp;
    if(robot_control_active && g_robot_control.startup_enable) {
        /* 初值只捕获一次，随后独立移向模型一致的平衡倾角。
         * 非零机身配平时目标不再强制为0，避免固定停车点处仍有恒定重力残差。 */
        if(balance_active)
            native_startup_theta_reference+=clamp_symmetric(equilibrium_theta-native_startup_theta_reference,
                LEG_DEFAULT_ANGLE_REFERENCE_RATE_RAD_S*state->period_ms*.001f);
    } else native_startup_theta_reference=equilibrium_theta;
    reference[0]=native_startup_theta_reference;
    /* Original0821 simple drift compensation, added to the existing theta
     * equilibrium only. Keep the1531 K/Ki, phi trim, Tp feedforward and true
     * parking goal untouched. This is not the Iq-based COM observer. */
    g_balance.parking_lean_learning=0U;
    unsigned lean_mode=BALANCE_NATIVE_DRIFT_COMPENSATION_ENABLE &&
        c.parking_lean_enable && c.parking_lean_gain_rad_per_m>0 &&
        c.position_window_m==0 && c.wheel_scale>0 && c.leg_scale>0 &&
        g_dual_hold.angle_reference_mode==LEG_ANGLE_NEUTRAL &&
        fabsf(g_dual_hold.angle_target_rad-1.57079632679f)<.0001f &&
        (!robot_control_active || !g_robot_control.startup_enable || robot_startup_phase==ROBOT_STARTUP_BALANCE) &&
        g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY &&
        g_robot_motion.neutral_ready && !driving && !motion_braking && balance_position_locked;
    if(!lean_mode) {
        balance_parking_lean=balance_parking_speed=0;
        balance_parking_settle_ms=0U;
    } else {
        float dt=state->period_ms*.001f;
        balance_parking_speed+=dt/(BALANCE_PARKING_LEAN_FILTER_S+dt)*(x[3]-balance_parking_speed);
        unsigned healthy=fabsf(x[3])<=BALANCE_NATIVE_DRIFT_MAX_SPEED_M_S &&
            fabsf(state->body_pitch_rad)<=BALANCE_PARKING_LEAN_POSE_RAD &&
            fabsf(x[5])<=BALANCE_PARKING_LEAN_BODY_RATE_RAD_S &&
            fabsf(state->leg[0].angle-state->leg[1].angle)<=BALANCE_PARKING_LEAN_POSE_RAD &&
            fabsf(g_robot_motion.command.requested_forward_m_s)<1e-6f &&
            fabsf(g_robot_motion.command.requested_turn_rad_s)<.02f &&
            fabsf(g_robot_motion.command.turn_rad_s)<.02f &&
            !g_wheel_output.saturated && !g_wheel_output.torque_saturated && !g_joint_output.saturated;
        if(!healthy) balance_parking_settle_ms=0;
        else {
            if(balance_parking_settle_ms<BALANCE_PARKING_LEAN_SETTLE_MS)
                balance_parking_settle_ms+=state->period_ms;
            if(balance_parking_settle_ms>=BALANCE_PARKING_LEAN_SETTLE_MS) {
                float speed=continuous_deadband(balance_parking_speed,BALANCE_PARKING_LEAN_SPEED_DEADBAND_M_S);
                float rate=clamp_symmetric(-c.parking_lean_gain_rad_per_m*speed,BALANCE_PARKING_LEAN_RATE_RAD_S);
                balance_parking_lean=clamp_symmetric(balance_parking_lean+rate*dt,BALANCE_PARKING_LEAN_LIMIT_RAD);
                g_balance.parking_lean_learning=1U;
            }
        }
    }
    reference[0]+=balance_parking_lean;
    g_balance_bias.samples=balance_bias.samples;
    g_balance_bias.estimated_offset_m=balance_bias.offset_m;
    g_balance_bias.phi_reference_rad=reference[4];
    g_balance_bias.body_pitch_target_deg=-reference[4]/DEG_TO_RAD;
    g_balance_bias.actual_pitch_torque_nm=measured?actual_tp:NAN;
    g_balance_bias.nominal_height_m=LQR_BODY_COM_HEIGHT_M;
    /* Finite-memory integration of the real parking error beyond the 30cm cap.
     * Adding one bounded error state to the existing position column keeps
     * both T and Tp coupled. It is not a standalone wheel-speed damper.
     * The trace stores this corrected reference and the unchanged true goal,
     * so correction = x-ref_x-clamp(x-goal,window) can be reconstructed. */
    uint32_t parking_gates=0U;
    if(g_balance_parking_integral.enable==1U && g_balance_bias.enable==0U) parking_gates|=1U;
    if(!robot_control_active || !g_robot_control.startup_enable ||
       g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE) parking_gates|=2U;
    if(!driving && g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY &&
       g_robot_motion.neutral_ready && fabsf(g_robot_motion.command.forward_axis)<.05f &&
       fabsf(g_robot_motion.command.turn_axis)<.05f &&
       fabsf(g_robot_motion.command.requested_forward_m_s)<.01f &&
       fabsf(g_robot_motion.command.requested_turn_rad_s)<.02f &&
       fabsf(g_robot_motion.command.turn_rad_s)<.02f) parking_gates|=4U;
    if(fabsf(x[3])<.35f && fabsf(x[1])<.35f && fabsf(x[5])<.35f &&
       fabsf(x[0]-reference[0])<.139626340f && fabsf(x[4])<.261799388f &&
       fabsf(x[4]-reference[4])<.174532925f &&
       fabsf(state->leg[0].angle-state->leg[1].angle)<.087266463f) parking_gates|=8U;
    if(fabsf(g_dual_hold.feedforward_n-g_dual_hold.feedforward_target_n)<.05f) parking_gates|=16U;
    if(!g_wheel_output.saturated && !g_wheel_output.torque_saturated &&
       !g_joint_output.saturated && !g_dual_hold.force_saturated[0] &&
       !g_dual_hold.force_saturated[1]) parking_gates|=32U;
    if(parking_gates==63U) {
        if(g_balance_parking_integral.settle_ms<BALANCE_NATIVE_PARKING_INTEGRAL_DWELL_MS)
            g_balance_parking_integral.settle_ms+=state->period_ms;
    } else g_balance_parking_integral.settle_ms=0U;
    g_balance_parking_integral.gate_mask=parking_gates;
    g_balance_parking_integral.learning=parking_gates==63U &&
        g_balance_parking_integral.settle_ms>=BALANCE_NATIVE_PARKING_INTEGRAL_DWELL_MS;
    if(g_balance_parking_integral.enable==0U) g_balance_parking_integral.correction_m=0;
    else if(g_balance_parking_integral.learning) {
        float dt=state->period_ms*.001f;
        float delta=(BALANCE_NATIVE_PARKING_INTEGRAL_GAIN_S*
            continuous_deadband(x[2]-balance_origin,BALANCE_NATIVE_PARKING_INTEGRAL_BAND_M)-
            BALANCE_NATIVE_PARKING_INTEGRAL_LEAK_S*g_balance_parking_integral.correction_m)*dt;
        g_balance_parking_integral.correction_m=clamp_symmetric(
            g_balance_parking_integral.correction_m+clamp_symmetric(delta,
                BALANCE_NATIVE_PARKING_INTEGRAL_RATE_M_S*dt),
            BALANCE_NATIVE_PARKING_INTEGRAL_LIMIT_M);
    }
    reference[2]-=g_balance_parking_integral.correction_m;
    if(!lqr_control_with_gain(length,x,reference,u,K)) goto publish_native;
    g_balance_servo.active=g_balance_servo.learning=g_balance_servo.gate_mask=0U;
    g_balance_servo.wheel_integral_nm=g_balance_servo.leg_integral_nm=0;
    unsigned established=!robot_control_active || !g_robot_control.startup_enable ||
        g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE;
    if(!g_balance_servo.enable) {
        g_balance_servo.blend=g_balance_servo.integral_m_s=0;
        g_balance_servo.parking_blend=0;
    } else if(established) {
        float servo_gain[12];
        if(!BalanceServo_Gain(length,servo_gain,integral_gain)) {
            status=3U;goto publish_native;
        }
        /* Accepted forward-neutral intent starts the coupled parking blend
         * immediately, including while coasting, BRAKING or turning. The
         * governor still owns its velocity ramp, fixed goal and measured-stop
         * dwell before releasing a reversed direction. Use its neutral epsilon
         * so a small intentional forward speed keeps driving gains. Keep the
         * same physical integral state and blend both full K rows AND Ki;
         * no isolated wheel-speed torque is added. */
        unsigned neutral=g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY &&
            g_robot_motion.neutral_ready &&
            fabsf(g_robot_motion.command.requested_forward_m_s)<=BALANCE_MOTION_REFERENCE_EPSILON_M_S;
        float target=neutral?1.0f:0.0f;
        g_balance_servo.parking_blend+=clamp_symmetric(target-g_balance_servo.parking_blend,
            state->period_ms*.001f/BALANCE_PARKING_BLEND_S);
        float parking_gain[12],parking_i[2];
        if(!BalanceParking_Gain(length,parking_gain,parking_i)) {
            status=3U;goto publish_native;
        }
        float parking_blend=g_balance_servo.parking_blend;
        for(unsigned i=0;i<12;++i)
            servo_gain[i]+=parking_blend*(parking_gain[i]-servo_gain[i]);
        for(unsigned row=0;row<2;++row)
            integral_gain[row]+=parking_blend*(parking_i[row]-integral_gain[row]);
        g_balance_servo.blend=fminf(1.0f,g_balance_servo.blend+
            state->period_ms*.001f/BALANCE_SERVO_BLEND_S);
        float blend=g_balance_servo.blend;
        for(unsigned i=0;i<12;++i)K[i]+=blend*(servo_gain[i]-K[i]);
        /* Conditional integration freezes on clipped position input or actuator
         * limits. No leakage: a learned constant correction must not decay
         * while it is holding the parking point. The same integral acts on
         * both outputs through the jointly designed Ki vector. */
        uint32_t sg=1U;
        if(g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY)sg|=2U;
        if(blend>=1.0f)sg|=4U;
        /* Forming goal=x+window and subtracting x again can straddle the
         * nominal boundary by a few float ULPs at metre-scale odometry.
         * Only the I eligibility comparison gets this numerical tolerance;
         * the physical window, true goal and proportional error stay exact. */
        float position_roundoff=4.0f*FLT_EPSILON*
            fmaxf(1.0f,fmaxf(fabsf(x[2]),fabsf(balance_origin)));
        if(fabsf(x[2]-balance_origin)<=position_window+position_roundoff &&
           fabsf(x[3])<.60f && fabsf(x[1])<.60f && fabsf(x[5])<.60f &&
           fabsf(x[4])<.261799388f && fabsf(x[0]-reference[0])<.261799388f &&
           fabsf(state->leg[0].angle-state->leg[1].angle)<.087266463f)sg|=8U;
        if(!g_wheel_output.saturated && !g_wheel_output.torque_saturated &&
           !g_joint_output.saturated && !g_dual_hold.force_saturated[0] &&
           !g_dual_hold.force_saturated[1])sg|=16U;
        g_balance_servo.gate_mask=sg;
        g_balance_servo.learning=BALANCE_NATIVE_POSITION_INTEGRAL_ENABLE && sg==31U;
        if(!BALANCE_NATIVE_POSITION_INTEGRAL_ENABLE) g_balance_servo.integral_m_s=0;
        u[0]=u[1]=0;
        for(unsigned i=0;i<6;++i)for(unsigned row=0;row<2;++row)
            u[row]-=K[2*i+row]*(x[i]-reference[i]);
        for(unsigned row=0;row<2;++row) {
            integral_gain[row]*=BALANCE_NATIVE_POSITION_INTEGRAL_ENABLE ? blend : 0;
            u[row]-=integral_gain[row]*g_balance_servo.integral_m_s;
        }
        g_balance_servo.wheel_integral_nm=-integral_gain[0]*g_balance_servo.integral_m_s;
        g_balance_servo.leg_integral_nm=-c.leg_scale*integral_gain[1]*g_balance_servo.integral_m_s;
        /* DARE timing: output uses z_k; only then prepare z_(k+1). */
        if(g_balance_servo.learning) {
            float error=clamp_symmetric(x[2]-balance_origin,BALANCE_SERVO_INTEGRAL_RATE_M);
            g_balance_servo.integral_m_s=clamp_symmetric(g_balance_servo.integral_m_s+
                error*state->period_ms*.001f,BALANCE_SERVO_INTEGRAL_LIMIT_M_S);
        }
        g_balance_servo.active=1U;
    }
    /* Explicit native swing authority: scale state/I feedback together,
     * preserving the wheel row, radial support and static gravity balance. */
    u[1]=c.leg_scale*u[1]+equilibrium_tp;
    /* 姿态未就绪时仍保留完整 T/Tp；就绪标志只控制摇杆行驶指令的放行。
     * 曾经只保留轮端部分反馈、把 Tp 置零，会破坏原耦合控制器的闭环结构。
     * 这里的 torque_nm 是请求值；判断执行能力还要看下游应用值和实际 Iq。 */
    status=1U;
publish_native:
    if(status!=1U) {
        u[0]=u[1]=0;
        balance_reference_clear();
    } else {
        balance_active=1U;
        if(!g_balance_motion_reference.enable)
            balance_position_locked=fabsf(velocity)<=1e-6f;
    }
    g_balance.status=status;
    g_balance.leg_length_m=length;
    g_balance.torque_nm[0]=u[0];g_balance.torque_nm[1]=u[1];
    g_balance.saturated=0U;
    g_balance.position_locked=balance_position_locked;
    g_balance.parking_target_m=balance_origin;
    g_balance.parking_error_m=status==1U ? x[2]-balance_origin : 0;
    g_balance.motion_speed_error_m_s=status==1U ? x[3]-reference[3] : 0;
    g_balance.motion_braking=status==1U && (motion_braking ||
        (fabsf(velocity)<=1e-6f && fabsf(x[3])>.02f));
    g_balance.startup_recovery=0U;
    g_balance.startup_wheel_damping_torque_nm=0;
    g_balance.parking_damping_torque_nm=0;g_balance.parking_damping_active=0U;
    g_balance.motion_damping_torque_nm=0;g_balance.motion_damping_active=0U;
    g_balance.parking_lean_rad=balance_parking_lean;g_balance.motion_lean_rad=0;
    g_balance.parking_lean_filtered_speed_m_s=balance_parking_speed;
    g_balance.motion_lean_learning=g_balance.motion_stall_hold=0U;
    g_balance.pitch_trim_applied_deg=status==1U ? -reference[4]/DEG_TO_RAD : 0;
    for(unsigned i=0;i<6;++i) {
        g_balance.state[i]=x[i];g_balance.reference[i]=reference[i];
        g_balance.wheel_torque_component_nm[i]=status==1U ?
            -K[2*i]*(x[i]-reference[i]) : 0;
    }
}
#else
/* Historical scaled/compensated controller, explicitly selected by regressions. */
static void balance_update(const WheelLegDebug *state, uint32_t dual_active)
{
    const BalanceDebug c = g_balance;
    float x[6] = {0}, reference[6] = {0}, u[2] = {0}, length = 0;
    float component[6]={0};
    float K[12];
    float startup_damping=0;
    float parking_damping=0;
    uint32_t parking_damping_active=0U;
    uint32_t status = 0U;
    uint32_t lqr_running=0U;
    uint32_t leg_lqr_limited=0U;
    float startup_blend=robot_control_active ? robot_startup_blend : 1.0f;
    if (!c.enable || !dual_active) goto publish;
    status = 3U;
    if (robot_control_active && g_robot_control.startup_enable>1U) goto publish;
    if (!balance_parameters_valid(&c)) goto publish;
    /* Trim assumes a world-vertical virtual leg (theta reference zero).
     * Reject conflicting independent local pose requests instead of making
     * the local PD fight the LQR equilibrium reference. */
    if ((c.pitch_trim_deg!=0 || balance_pitch_trim!=0) &&
        (g_dual_hold.angle_reference_mode!=LEG_ANGLE_NEUTRAL ||
         fabsf(g_dual_hold.angle_target_rad-1.5707963267948966f)>0.0001f)) goto publish;
    status = 2U;
    if (c.wheel_scale==0.0f && c.leg_scale==0.0f && c.back_emf_ratio==0.0f) {
        /* Pure length/pose support has no requested wheel or LQR torque.
         * Like dual-only hold, it does not use wheel odometry or IMU state.
         * Retain the existing per-leg envelope and validity checks. */
        for (unsigned side=0;side<2;++side) {
            if (!state->leg_valid[side] || state->leg[side].length<LQR_LEG_MIN_M ||
                state->leg[side].length>LQR_LEG_MAX_M) goto publish;
        }
        length=.5f*(state->leg[0].length+state->leg[1].length);
        status=1U;
        goto publish;
    }
    for (unsigned side=0;side<2;++side) {
        if (!state->state_valid[side] || !state->lqr_valid[side]) goto publish;
    }
    /* 左右六维状态取均值，形成共同的矢状面 LQR 输入。
     * 轮里程仍假设直接驱动且无滑移，未计入轮架转动。 */
    for (unsigned i=0;i<6;++i) x[i] = .5f*state->state[0][i] + .5f*state->state[1][i];
    if (!balance_active) {
        balance_origin=x[2];
        balance_startup_recovery=ROBOT_MOTION_STARTUP_RECOVERY_ENABLE && robot_control_active &&
            g_robot_control.remote_enable && g_robot_control.startup_enable && c.parking_lean_enable;
    }
    unsigned normal_balance=!robot_control_active || robot_startup_phase==ROBOT_STARTUP_BALANCE;
    unsigned translating=normal_balance &&
        g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY && g_robot_motion.translating;
    if(translating!=balance_drive_running) balance_parking_settle_ms=0;
    unsigned parking_context=c.position_window_m==0 && normal_balance;
    if(!parking_context || !c.wheel_scale || !c.leg_scale) {
        balance_braking=balance_stop_settle_ms=0;
    } else if(translating) {
        balance_startup_recovery=0;balance_braking=0;balance_stop_settle_ms=0;
    } else if(balance_drive_running) {
        balance_braking=1;balance_stop_settle_ms=0;
    }
    if(balance_startup_recovery && parking_context && !translating) balance_braking=1;
    if(balance_braking) {
        if(fabsf(x[3])<.02f && fabsf(state->body_pitch_rad)<BALANCE_PARKING_LEAN_POSE_RAD &&
           fabsf(x[5])<BALANCE_PARKING_LEAN_BODY_RATE_RAD_S)
            balance_stop_settle_ms+=state->period_ms;
        else balance_stop_settle_ms=0;
        if(balance_stop_settle_ms>=200U &&
           (!balance_startup_recovery || g_robot_control.startup_elapsed_s>=2.0f || !g_robot_control.startup_enable)) {
            balance_braking=0;balance_startup_recovery=0;
        }
    }
    g_balance.motion_braking=balance_braking;g_balance.startup_recovery=balance_startup_recovery;
    unsigned tracking=translating || balance_braking;
    unsigned fixed_parking=parking_context && !tracking;
    if (translating) {
        if (!balance_drive_running) balance_origin=x[2];
        balance_origin+=g_robot_motion.command.forward_m_s*state->period_ms*.001f;
        balance_origin=x[2]+clamp_symmetric(balance_origin-x[2],ROBOT_MOTION_TRACK_WINDOW_M);
        reference[3]=g_robot_motion.command.forward_m_s;
    } else if (balance_drive_running) {
        balance_origin=x[2];
        balance_position_locked=0U;
    }
    if(balance_braking) { balance_origin=x[2];balance_position_locked=0;reference[3]=0; }
    balance_drive_running=translating;
    if (fixed_parking && !balance_position_locked) {
        /* 首次进入定点停车时锁定当前位置；行驶/刹车退出后重新捕获。
         * 单独改变倾角补偿或俯仰微调不会重置已锁定的停车点。 */
        balance_origin=x[2];
        balance_position_locked=1U;
    } else if (!fixed_parking) balance_position_locked=0U;
    float window=c.position_window_m;
    if (!normal_balance && window==0) window=BALANCE_STARTUP_POSITION_WINDOW_M;
    if (!translating && window>0 && fabsf(balance_origin-x[2])>window)
        balance_origin=x[2]+clamp_symmetric(balance_origin-x[2],window);
    reference[2]=balance_origin;
    if (fixed_parking && c.parking_correction_limit_m>0) {
        /* A distant parking request must not consume the whole small wheel
         * torque budget. Preserve the goal; bound only its input to LQR.
         * Once within this band, the original exact parking feedback resumes. */
        reference[2]=x[2]-clamp_symmetric(x[2]-balance_origin,c.parking_correction_limit_m);
    }
    if (!balance_active) balance_pitch_trim=0;
    if (!robot_control_active || robot_startup_phase==ROBOT_STARTUP_BALANCE)
        balance_pitch_trim+=clamp_symmetric(c.pitch_trim_deg-balance_pitch_trim,
            BALANCE_PITCH_TRIM_RATE_DEG_S*state->period_ms*.001f);
    else balance_pitch_trim=0;
    reference[4]=-balance_pitch_trim*DEG_TO_RAD;
    /* 慢积分修正世界系虚拟腿倾角，LQR 与局部摆角 PD 共用该补偿。
     * 行驶时学习实际速度与目标速度之差，并将学到的平衡点保留到停车。 */
    unsigned learn_mode=c.parking_lean_enable && c.parking_lean_gain_rad_per_m>0 &&
        parking_context && c.wheel_scale>0 && c.leg_scale>0 &&
        g_dual_hold.angle_reference_mode==LEG_ANGLE_NEUTRAL &&
        fabsf(g_dual_hold.angle_target_rad-1.5707963267948966f)<0.0001f;
    g_balance.parking_lean_learning=0U;
    g_balance.motion_lean_learning=0U;
    g_balance.motion_speed_error_m_s=tracking ? x[3]-reference[3] : 0;
    unsigned stalled=translating && fabsf(reference[3])>.04f && fabsf(x[3])<.005f;
    if(stalled) {
        if(balance_motion_stall_ms<ROBOT_MOTION_STALL_CONFIRM_MS)
            balance_motion_stall_ms+=state->period_ms;
    } else balance_motion_stall_ms=0;
    g_balance.motion_stall_hold=balance_motion_stall_ms>=ROBOT_MOTION_STALL_CONFIRM_MS;
    if (!learn_mode) {
        balance_parking_lean=balance_parking_speed=0;
        balance_parking_settle_ms=0;
    } else {
        float dt=state->period_ms*.001f;
        balance_parking_speed += dt/(BALANCE_PARKING_LEAN_FILTER_S+dt)*
            (x[3]-balance_parking_speed);
        unsigned yaw_in_range=translating ?
            fabsf(g_robot_motion.measured_yaw_rate_rad_s)<ROBOT_MOTION_LEAN_MAX_YAW_RAD_S :
            (fabsf(g_robot_motion.command.turn_rad_s)<.02f &&
             fabsf(g_robot_motion.command.requested_turn_rad_s)<.02f &&
             fabsf(g_robot_motion.measured_yaw_rate_rad_s)<.15f);
        /* 饱和标记来自上一周期输出；姿态、转向或堵转条件不满足时暂停学习。 */
        unsigned healthy=(!balance_braking || balance_startup_recovery) && yaw_in_range && !g_balance.motion_stall_hold &&
            fabsf(x[3])<=((translating || balance_startup_recovery) ? ROBOT_MOTION_LEAN_MAX_SPEED_M_S : BALANCE_PARKING_LEAN_MAX_SPEED_M_S) &&
            fabsf(state->body_pitch_rad)<=BALANCE_PARKING_LEAN_POSE_RAD &&
            fabsf(x[5])<=BALANCE_PARKING_LEAN_BODY_RATE_RAD_S &&
            fabsf(state->leg[0].angle-state->leg[1].angle)<=BALANCE_PARKING_LEAN_POSE_RAD &&
            !g_wheel_output.saturated && !g_wheel_output.torque_saturated &&
            !g_joint_output.saturated;
        if (!healthy) balance_parking_settle_ms=0;
        else {
            uint32_t settle=(translating || balance_startup_recovery) ? ROBOT_MOTION_LEAN_SETTLE_MS : BALANCE_PARKING_LEAN_SETTLE_MS;
            if (balance_parking_settle_ms<settle)
                balance_parking_settle_ms+=state->period_ms;
            if (balance_parking_settle_ms>=settle) {
                float speed=balance_parking_speed-(translating ? reference[3] : 0);
                speed=fabsf(speed)<=BALANCE_PARKING_LEAN_SPEED_DEADBAND_M_S ? 0 :
                    speed-copysignf(BALANCE_PARKING_LEAN_SPEED_DEADBAND_M_S,speed);
                float gain=(translating || balance_startup_recovery) ? ROBOT_MOTION_LEAN_GAIN_RAD_M : c.parking_lean_gain_rad_per_m;
                float max_rate=(translating || balance_startup_recovery) ? ROBOT_MOTION_LEAN_RATE_RAD_S : BALANCE_PARKING_LEAN_RATE_RAD_S;
                float rate=clamp_symmetric(-gain*speed,max_rate);
                /* Clamping the integrator itself prevents hidden windup. */
                balance_parking_lean=clamp_symmetric(balance_parking_lean+rate*dt,
                                                    BALANCE_PARKING_LEAN_LIMIT_RAD);
                g_balance.parking_lean_learning=!translating;
                g_balance.motion_lean_learning=translating;
            }
        }
    }
    /* Bounded proportional lean handles transient tracking; the separate
     * slow integral retains the learned equilibrium. Both LQR and leg PD
     * receive the SAME combined reference. No second hidden integrator. */
    /* Fast proportional correction must remain available during turning.
     * Only slow equilibrium learning uses the yaw gate: it must not learn
     * a turning load as a persistent bias. The slow speed gate must not
     * withdraw fast braking correction at overspeed. Pose, actuator, amplitude
     * and slew bounds still constrain this non-integrating fast term. */
    unsigned fast_healthy=learn_mode && tracking &&
        fabsf(state->body_pitch_rad)<=BALANCE_PARKING_LEAN_POSE_RAD &&
        fabsf(x[5])<=BALANCE_PARKING_LEAN_BODY_RATE_RAD_S &&
        fabsf(state->leg[0].angle-state->leg[1].angle)<=BALANCE_PARKING_LEAN_POSE_RAD &&
        !g_joint_output.saturated && !g_wheel_output.saturated;
    float fast_target=fast_healthy ? -clamp_symmetric(
        ROBOT_MOTION_FAST_LEAN_GAIN_RAD_M_S*continuous_deadband(x[3]-reference[3],.005f),
        ROBOT_MOTION_FAST_LEAN_LIMIT_RAD) : 0;
    if(!learn_mode) balance_motion_lean=0;
    else balance_motion_lean+=clamp_symmetric(fast_target-balance_motion_lean,
        ROBOT_MOTION_FAST_LEAN_RATE_RAD_S*state->period_ms*.001f);
    g_balance.motion_lean_rad=balance_motion_lean;
    reference[0]=balance_parking_lean+balance_motion_lean;
    g_balance.parking_lean_rad=balance_parking_lean;
    g_balance.parking_lean_filtered_speed_m_s=balance_parking_speed;
    length=.5f*(state->leg[0].length+state->leg[1].length);
    /* 同一次增益计算供总力矩和轮力矩分项复用，避免重复计算腿长多项式。 */
    if (!lqr_control_with_gain(length,x,reference,u,K)) goto publish;
    /* Wheel balance starts immediately; leg LQR ramps concurrently with
     * support. Static pose/support diagnostics never suppress recovery. */
    u[1]*=c.leg_scale*startup_blend;
    /* Bound the shared virtual-leg command independently of the final
     * per-motor J^T torque and voltage ceilings, including Watch changes. */
    leg_lqr_limited=fabsf(u[1])>BALANCE_LEG_LQR_TORQUE_LIMIT_NM;
    u[1]=clamp_symmetric(u[1],BALANCE_LEG_LQR_TORQUE_LIMIT_NM);
    u[0]=0;
    for(unsigned i=0;i<6;++i) {
        float rate_scale=(i==1U || i==5U) ? c.wheel_rate_scale : 1.0f;
        component[i]=-K[2*i]*(x[i]-reference[i])*c.wheel_scale*rate_scale;
        u[0]+=component[i];
    }
    if (robot_control_active && startup_blend<1.0f && c.wheel_scale>0.0f) {
        /* Oppose common rolling in physical forward-positive coordinates.
         * Unlike the LQR speed term this is direct damping, independent of
         * split-pose tilt. Preserve relative wheel motion for leg alignment.
         * No position spring: do not pull back to the initial split pose. */
        float speed=x[3]/WHEEL_RADIUS_M;
        startup_damping=clamp_symmetric(-c.startup_wheel_damping_nm_s_per_rad*c.wheel_scale*speed,
                                     c.startup_wheel_limit_nm)*(1.0f-startup_blend);
        u[0]+=startup_damping;
    }
    if(fixed_parking && c.wheel_scale>0 && c.parking_damping_gain_nm_per_m_s>0) {
        parking_damping=-clamp_symmetric(c.parking_damping_gain_nm_per_m_s*x[3],
                                        BALANCE_PARKING_DAMPING_LIMIT_NM);
        parking_damping_active=1U;
        u[0]+=parking_damping;
    }
    /* Immediate bounded velocity-error damping while the slower shared pose
     * correction converges. Do not use the desired speed as damping toward zero. */
    g_balance.motion_damping_active=(tracking || fabsf(balance_motion_damping)>1e-8f) && c.wheel_scale>0;
    if(g_balance.motion_damping_active) {
        float gain=balance_startup_recovery ? ROBOT_MOTION_STARTUP_DAMPING_NM_PER_M_S : ROBOT_MOTION_SPEED_DAMPING_NM_PER_M_S;
        float limit=balance_startup_recovery ? ROBOT_MOTION_STARTUP_DAMPING_LIMIT_NM : ROBOT_MOTION_SPEED_DAMPING_LIMIT_NM;
        float rate=balance_startup_recovery ? ROBOT_MOTION_STARTUP_DAMPING_RATE_NM_S : ROBOT_MOTION_SPEED_DAMPING_RATE_NM_S;
        float request=tracking ? -clamp_symmetric(gain*
            continuous_deadband(x[3]-reference[3],.005f),limit) : 0;
        if(request*balance_motion_damping<0) balance_motion_damping=0;
        balance_motion_damping+=clamp_symmetric(request-balance_motion_damping,
            rate*state->period_ms*.001f);
        u[0]+=balance_motion_damping;
    } else balance_motion_damping=0;
    g_balance.motion_damping_torque_nm=balance_motion_damping;
    status=1U;
    /* Reset references when real LQR is disabled, not during normal motion. */
    lqr_running=c.wheel_scale>0.0f || startup_blend>0.0f;
publish:
    /* Re-latch wheel displacement origin when real LQR is enabled again. */
    balance_active=status==1U && lqr_running;
    if (!balance_active) balance_reference_clear();
    else g_balance.pitch_trim_applied_deg=balance_pitch_trim;
    g_balance.position_locked=balance_position_locked;
    g_balance.parking_target_m=balance_active ? balance_origin : 0;
    g_balance.parking_error_m=balance_active ? x[2]-balance_origin : 0;
    if (status!=1U) { u[0]=u[1]=startup_damping=parking_damping=0;parking_damping_active=0U; }
    g_balance.status=status;
    g_balance.leg_length_m=length;
    g_balance.torque_nm[0]=u[0];g_balance.torque_nm[1]=u[1];
    g_balance.startup_wheel_damping_torque_nm=startup_damping;
    g_balance.parking_damping_torque_nm=parking_damping;
    g_balance.parking_damping_active=parking_damping_active;
    g_balance.saturated=status==1U ? leg_lqr_limited : 0U;
    for(unsigned i=0;i<6;++i) {
        g_balance.state[i]=x[i];g_balance.reference[i]=reference[i];
        g_balance.wheel_torque_component_nm[i]=component[i];
    }
}

#endif

/* 轴向支撑 + 局部摆角 PD + 腿 LQR + 左右同步，经 J^T 映射为关节力矩。
 * 电压后端在此换算 mV；电流后端向电机服务提交轴端力矩 Nm。 */
static uint32_t dual_hold(const WheelLegDebug *state, uint32_t output_owned)
{
    static uint32_t active, stops;
    static float integral[2], filtered[2], feedforward, angle_reference[2];
    static float overlength_elapsed;
    static uint32_t pose_bad_elapsed_ms;
    static uint32_t body_bad_elapsed_ms;
    static float sync_error_filtered, sync_applied[2];
    static float sync_priority_scale=1.0f;
    static float load_force[2];
    const DualHoldDebug c = g_dual_hold;
    float pitch_trim=g_balance.enable && g_balance.status==1U ?
        g_balance.pitch_trim_applied_deg*DEG_TO_RAD : 0;
    float parking_lean=g_balance.enable && g_balance.status==1U ? g_balance.parking_lean_rad+g_balance.motion_lean_rad : 0;
    if(ROBOT_NATIVE_LQR && g_balance.enable && g_balance.status==1U)
        parking_lean=g_balance_equilibrium_theta_rad+g_balance.parking_lean_rad;
    float angle_target=c.angle_target_rad-pitch_trim+parking_lean;
    JointOutputDebug joint_out={0};
    float dt=0;
    int16_t voltage[8] = {0};
    float shaft_torque_nm[8]={0};
    uint32_t active_motor_mask=0U;
    float error[2] = {0}, force[2] = {0}, torque[4] = {0};
    float error_control[2]={0}, speed_control[2]={0};
    float angle_error[2]={0}, pitch[2]={0};
    float angle_hold[2]={0}, lqr_pitch[2]={0};
    float force_requested[2]={0}, force_bound[2]={0};
    float overlength_force[2]={0};
    float overlength_blend=0;
    float sync_difference=0, sync_speed=0, sync_target_speed=0;
    float sync_used[2]={0}, sync_previous[2]={sync_applied[0],sync_applied[1]};
    uint32_t sync_priority_active=0U;
    uint32_t force_saturated[2]={0};
    float load_applied[2]={0};
    uint32_t load_frozen[2]={0};
    uint32_t status = 0U;
    if (output_owned) goto stop;
    if (!c.enable && !active && !stops) { wheel_output_clear(); joint_output_clear(); return 0U; }
    if (!c.enable) goto stop;
    status = 3U;
    if (!dual_parameters_valid(&c)) goto stop;
    status = 2U;
    if (!state->leg_valid[0] || !state->leg_valid[1] || !state->period_ms || state->period_ms > 10U) goto stop;
    /* Startup uses the same feedback and posture protection as a running
     * controller below. Legacy static-readiness thresholds are diagnostic;
     * hand-held setup motion must not add a separate startup fault latch. */
    status = 5U;
    if (!MOTOR_COMMAND_TX_ENABLED) goto stop;
    if (g_balance.enable && g_balance.status!=1U) {
        status=g_balance.status==3U?3U:2U;
        goto stop;
    }
    status=3U;
    if (!joint_output_prepare(state,&c,&joint_out)) goto stop;
    /* A shared mean can look upright with one leg near 0 and the other pi.
     * Check both physical legs before issuing this cycle's motor commands.
     * Pure axial/PD bench operation and wheel-only ownership retain their
     * own dependencies. A stopped controller clears the confirmation time. */
    if (g_balance.enable && (ROBOT_NATIVE_LQR || g_balance.wheel_scale>0 || g_balance.leg_scale>0 ||
                             g_balance.back_emf_ratio>0)) {
        unsigned bad=fabsf(state->leg[0].angle-1.57079632679f)>ROBOT_POSE_MAX_LEG_ERROR_RAD ||
                     fabsf(state->leg[1].angle-1.57079632679f)>ROBOT_POSE_MAX_LEG_ERROR_RAD ||
                     fabsf(remainderf(state->leg[0].angle-state->leg[1].angle,
                                      6.28318530718f))>ROBOT_POSE_MAX_SPLIT_RAD;
        if (bad) {
            pose_bad_elapsed_ms+=state->period_ms;
            if (!active || pose_bad_elapsed_ms>=ROBOT_POSE_CONFIRM_MS) { status=7U;goto stop; }
        } else pose_bad_elapsed_ms=0U;
        if (state->imu_valid && fabsf(state->body_pitch_rad)>ROBOT_BODY_PITCH_LIMIT_RAD) {
            body_bad_elapsed_ms+=state->period_ms;
            if (!active || body_bad_elapsed_ms>=ROBOT_BODY_PITCH_CONFIRM_MS) { status=8U;goto stop; }
        } else body_bad_elapsed_ms=0U;
    } else pose_bad_elapsed_ms=body_bad_elapsed_ms=0U;
    dt = state->period_ms * .001f;
    if (!active) overlength_elapsed=0;
    overlength_elapsed=fminf(overlength_elapsed+dt,5.0f);
    overlength_blend=c.overlength_startup_s>0 ?
        fminf(overlength_elapsed/c.overlength_startup_s,1.0f) : 1.0f;
    feedforward += clamp_symmetric(c.feedforward_target_n - feedforward, c.feedforward_rate_n_s * dt);
    for (unsigned side=0;side<2;++side) {
        if (!active || !c.load_enable) load_force[side]=0;
        load_applied[side]=clamp_symmetric(load_force[side],c.load_limit_n);
        load_force[side]=load_applied[side];
    }
    /* 共同 LQR 的均值看不到两腿反向偏差，须用左右摆角差/角速度差单独同步。
     * 此处力矩正向对应模型摆角增大：左减右为正，应给左负、右正纠偏力矩；
     * 幅值按各侧腿长缩放，不能直接照搬其他坐标定义下的 VMC 符号。 */
    sync_difference=remainderf(state->leg[0].angle-state->leg[1].angle,6.28318530718f);
    sync_speed=state->angle_speed_rad_s[0]-state->angle_speed_rad_s[1];
    if (!isfinite(sync_difference) || !isfinite(sync_speed)) { status=3U; goto stop; }
    float sync_rate=c.sync_rate_nm_s;
    /* The g3 reversal split grew faster than the ordinary sync slew could
     * recover. Increase slew, not torque limits, only after the leg ramp and
     * while a velocity error calls for deceleration of existing motion. */
    if(!ROBOT_NATIVE_LQR && robot_control_active && robot_startup_phase==ROBOT_STARTUP_BALANCE &&
       !g_balance.startup_recovery && g_balance.motion_damping_active &&
       g_balance.state[3]*g_balance.motion_speed_error_m_s>0 &&
       fabsf(g_balance.motion_speed_error_m_s)>LEG_SYNC_DECEL_SPEED_ERROR_M_S &&
       fabsf(sync_difference)>LEG_SYNC_DECEL_SPLIT_RAD)
        sync_rate=fminf(2.0f,c.sync_rate_nm_s*LEG_SYNC_DECEL_RATE_SCALE);
    if (!active) { sync_error_filtered=sync_applied[0]=sync_applied[1]=0;sync_priority_scale=1; }
    if (c.sync_ratio==0) { sync_error_filtered=sync_applied[0]=sync_applied[1]=0;sync_priority_scale=1; }
    else {
        sync_error_filtered+=dt/(c.sync_filter_tau_s+dt)*(-sync_difference-sync_error_filtered);
        sync_target_speed=clamp_symmetric(c.sync_position_kp*sync_error_filtered,c.sync_max_speed_rad_s);
        float correction=c.sync_speed_kp*(sync_target_speed-sync_speed);
        if (!isfinite(correction)) { status=3U; goto stop; }
        for(unsigned side=0;side<2;++side) {
            float request=(side?-1.0f:1.0f)*correction*(state->leg[side].length/0.07f);
            request=c.sync_ratio*clamp_symmetric(request,c.sync_limit_nm);
            /* At 0.4Nm/s a full differential reversal took up to0.975s.
             * Both 0810 and the reversal regression show the retained torque
             * can then accelerate the new split. Native control uses the
             * bounded, fresh PD request; the driver still enforces its
             * current slew and per-motor hard limit. Legacy tests retain
             * their existing virtual torque ramp. */
            if (ROBOT_NATIVE_LQR) sync_applied[side]=request;
            else sync_applied[side]+=clamp_symmetric(request-sync_applied[side],sync_rate*dt);
            /* A reduced runtime limit takes effect immediately. */
            sync_applied[side]=clamp_symmetric(sync_applied[side],c.sync_ratio*c.sync_limit_nm);
        }
    }
    for (unsigned side = 0; side < 2; ++side) {
        /* The shared LQR mean cannot see opposing left/right swing errors.
         * Capture on the first frame, then slew each reference to the common
         * neutral target. Keep local angle damping, limited PD torque, and
         * length state continuous through balance toggles. */
        if (!active) angle_reference[side]=state->leg[side].angle;
        else if (c.angle_reference_mode==LEG_ANGLE_NEUTRAL ||
                 (robot_control_active && (robot_startup_phase==ROBOT_STARTUP_PREPARE ||
                                          robot_startup_phase==ROBOT_STARTUP_RAMP)))
            angle_reference[side]+=clamp_symmetric(angle_target-angle_reference[side],
                                                 c.angle_reference_rate_rad_s*dt);
        angle_error[side]=remainderf(angle_reference[side]-state->leg[side].angle,6.28318530718f);
        float tp=c.angle_kp_nm_per_rad*angle_error[side]-
                 c.angle_kd_nm_s_per_rad*state->angle_speed_rad_s[side];
        if (!isfinite(tp)) { status=3U; goto stop; }
#if ROBOT_NATIVE_LQR && LEG_ANGLE_INTEGRAL_ENABLED
        float bias=clamp_symmetric(g_leg_angle_integral_nm[side],
                                  fminf(LEG_ANGLE_I_LIMIT_NM,c.angle_limit_nm));
        if (!active || !g_balance.enable || g_balance.leg_hold_ratio<=0) bias=0;
        /* Learn only the slow loaded posture error, after startup and with
         * neutral sticks. Fast swing, turns and large body tilt freeze it.
         * Retained bias compensates load during motion; OFF/fault clears it. */
        unsigned learn=active && g_balance.enable && g_balance.leg_hold_ratio>0 &&
            robot_control_active && robot_startup_phase==ROBOT_STARTUP_BALANCE &&
            fabsf(g_robot_motion.command.forward_m_s)<.01f &&
            fabsf(g_robot_motion.command.turn_rad_s)<.10f &&
            fabsf(state->body_pitch_rad)<LEG_ANGLE_I_MAX_BODY_PITCH_RAD &&
            fabsf(angle_error[side])<LEG_ANGLE_I_MAX_ERROR_RAD &&
            fabsf(state->angle_speed_rad_s[side])<LEG_ANGLE_I_MAX_RATE_RAD_S;
        if (learn) {
            float delta=clamp_symmetric(LEG_ANGLE_KI_NM_PER_RAD_S*
                continuous_deadband(angle_error[side],LEG_ANGLE_I_DEADBAND_RAD),
                LEG_ANGLE_I_RATE_NM_S)*dt;
            float candidate=clamp_symmetric(bias+delta,
                fminf(LEG_ANGLE_I_LIMIT_NM,c.angle_limit_nm));
            /* Both the local torque cap and previous joint-current clamp
             * block further windup. An increment that unwinds is allowed. */
            if ((!g_joint_output.saturated || delta*bias<0) &&
                (fabsf(tp+candidate)<=c.angle_limit_nm || delta*(tp+candidate)<0))
                bias=candidate;
        }
        g_leg_angle_integral_nm[side]=bias;
        tp+=bias;
#endif
        float hold_ratio=g_balance.enable ? g_balance.leg_hold_ratio : 1.0f;
        /* Native startup uses the same complete LQR in every phase. Do not
         * silently introduce a second common-angle controller during rise. */
        if (!ROBOT_NATIVE_LQR && robot_control_active && (robot_startup_phase==ROBOT_STARTUP_PREPARE ||
                                     robot_startup_phase==ROBOT_STARTUP_RAMP))
            hold_ratio=1.0f+(hold_ratio-1.0f)*robot_startup_blend;
        angle_hold[side]=hold_ratio*clamp_symmetric(tp,c.angle_limit_nm);
        lqr_pitch[side]=g_balance.enable ? g_balance.torque_nm[1] : 0;
        pitch[side]=angle_hold[side]+lqr_pitch[side];
    }
    /* 两腿同向偏离中立目标时，差动同步可能抵消其中一侧的共同回正力矩。
     * 对两侧同步量使用同一缩放系数，给 PD+LQR 保留回正余量；
     * 原同步滤波、变化率、限幅及腿长比例仍在缩放前生效。 */
    float desired_scale=1;
    float neutral_error[2]={remainderf(angle_target-state->leg[0].angle,6.28318530718f),
                            remainderf(angle_target-state->leg[1].angle,6.28318530718f)};
    if(!ROBOT_NATIVE_LQR && c.sync_recovery_priority_enable && c.angle_reference_mode==LEG_ANGLE_NEUTRAL &&
       g_balance.enable && g_balance.leg_scale>0 && g_balance.status==1U &&
       neutral_error[0]*neutral_error[1]>0 &&
       pitch[0]*neutral_error[0]>0 && pitch[1]*neutral_error[1]>0) {
        float budget_scale=1;
        for(unsigned side=0;side<2;++side) {
            if(sync_applied[side]*pitch[side]<0)
                budget_scale=fminf(budget_scale,
                    (1-LEG_SYNC_RECOVERY_RESERVE_RATIO)*fabsf(pitch[side])/fabsf(sync_applied[side]));
        }
        float blend=fminf(1,fminf(fabsf(neutral_error[0]),fabsf(neutral_error[1]))/
                               LEG_SYNC_RECOVERY_BLEND_RAD);
        desired_scale=1-blend*(1-budget_scale);
    }
    if(desired_scale<sync_priority_scale) sync_priority_scale=desired_scale;
    else {
        /* Restore authority gently. Deduct movement already consumed by the
         * raw slew, so scale release plus raw movement remains within its
         * existing per-cycle step. Applying a smaller recovery budget is
         * immediate, like a reduced runtime torque limit. */
        float raw_delta=fmaxf(fabsf(sync_applied[0]-sync_previous[0]),
                             fabsf(sync_applied[1]-sync_previous[1]));
        float raw_max=fmaxf(fabsf(sync_applied[0]),fabsf(sync_applied[1]));
        float release=fmaxf(0,sync_rate*dt-raw_delta)/fmaxf(raw_max,1e-6f);
        sync_priority_scale+=fminf(desired_scale-sync_priority_scale,release);
    }
    sync_priority_scale=fmaxf(0,fminf(1,sync_priority_scale));
    sync_priority_active=sync_priority_scale<.999999f &&
        fmaxf(fabsf(sync_applied[0]),fabsf(sync_applied[1]))>1e-6f;
#if ROBOT_LEG_VOLTAGE_PROBE
    /* A pure virtual swing probe, normalized through the same J^T and
     * voltage calibration as normal control. One selected joint requests
     * 2 V; its partner retains the geometry ratio, with zero axial force.
     * Keep this separate from the recorded PD/sync/LQR components. */
    const unsigned probe_side=ROBOT_LEG_PD_TEST_SIDE;
    float probe_gain=0;
    for(unsigned joint=0;joint<2;++joint) {
        float gain=fabsf(state->leg[probe_side].jacobian[3-joint])*
                   joint_out.torque_mv_per_nm[probe_side*2+joint];
        if(!isfinite(gain)) {status=3U;goto stop;}
        probe_gain=fmaxf(probe_gain,gain);
    }
    if(probe_gain<1e-3f || fabsf(leg_voltage_probe_direction)!=1) {status=3U;goto stop;}
    pitch[probe_side]=leg_voltage_probe_direction*fminf(LEG_VOLTAGE_PROBE_PEAK_MV/probe_gain,
                                                     LEG_VOLTAGE_PROBE_MAX_TP_NM);
#endif
    for(unsigned side=0;side<2;++side) {
        sync_used[side]=sync_priority_scale*sync_applied[side];
        pitch[side]+=sync_used[side];
#if ROBOT_LEG_PD_TEST_SIDE >= 0
        if (side != ROBOT_LEG_PD_TEST_SIDE) angle_hold[side]=pitch[side]=0;
#endif
        error[side] = c.target_m - state->leg[side].length;
        error_control[side]=continuous_deadband(error[side],c.length_error_deadband_m);
        float target = c.position_kp * error_control[side];
        if (!isfinite(target)) { status = 3U; goto stop; }
        float desired_speed=clamp_symmetric(target,c.max_speed_m_s);
        /* Smooth the reference, not the whole error: measured velocity must
         * oppose current motion immediately, without another 20 ms delay. */
        if (!active) filtered[side]=0;
        filtered[side]+=dt/(c.filter_tau_s+dt)*(desired_speed-filtered[side]);
        /* Ignore small signed speed noise without adding measurement delay.
         * Outside the band, preserve the measured direction and immediate damping. */
        speed_control[side]=continuous_deadband(state->length_speed_m_s[side],
                                              c.length_speed_deadband_m_s);
        float speed_error=filtered[side]-speed_control[side];
        float next;
        if (ROBOT_NATIVE_LQR) {
            /* 等前馈建立后再学习承重偏置，避免把起立初始的重力落差积分进去。
             * 力偏置限制在+/-4.5N、每秒最多变化3N，目标速度仍限制为0.04m/s。 */
            unsigned ready=fabsf(feedforward-c.feedforward_target_n)<=LEG_HEIGHT_INTEGRAL_SUPPORT_READY_N;
            next=leg_height_integral_candidate(integral[side],speed_error,c.speed_ki,dt,
                LEG_HEIGHT_INTEGRAL_LIMIT_N,LEG_HEIGHT_INTEGRAL_RATE_N_S,ready);
        } else next=c.speed_ki==0 ? 0 : integral[side]+c.speed_ki*speed_error*dt;
        /* 轴向力由四项相加：腿长速度反馈 + 支撑前馈 + 过长反压 + 慢速承重补偿。
         * overlength_force <= 0：超过 target+margin 后主动卸力，允许腿收回。
         * native默认KP=0关闭过长卸力，保留速度反馈在前馈附近调节承重。
         * 手动启用265N/m时，超出阈值10mm约扣2.65N；前馈9N不等于最终9N。
         * 调参时同时看实际腿长、force_requested、force_limit，区分主动减力和限幅。
         * 启动1s渐入只平滑反压建立过程，不是电机运行时限。 */
        overlength_force[side]=-overlength_blend*c.overlength_kp_n_per_m*
            smooth_extension(-error[side]-c.overlength_margin_m,c.overlength_transition_m);
        float base = c.speed_kp * speed_error + feedforward + overlength_force[side] + load_applied[side];
        if (!isfinite(next) || !isfinite(base) || !isfinite(base+next)) { status = 3U; goto stop; }
        if (!ROBOT_NATIVE_LQR) next = clamp_symmetric(next,c.max_force_n);
        /* 先按纯轴向分量的可用输出限制支撑力，保留两关节的几何比例。
         * 加入摆角力矩后仍需逐关节限幅，因此这不是最终混合输出的保证。 */
        float force_limit = c.max_force_n;
        for (unsigned joint=0;joint<2;++joint) {
            unsigned index=side*2+joint;
            float gain=fabsf(state->leg[side].jacobian[1-joint])*joint_out.torque_mv_per_nm[index];
            if (!isfinite(gain)) { status=3U; goto stop; }
            if(MotorCommand_UsesCurrent((uint8_t)g_joint_calibration[index].motor_id)) {
                float jac=fabsf(state->leg[side].jacobian[1-joint]);
                float cap=MotorCommand_CurrentTorqueLimitNm((uint8_t)g_joint_calibration[index].motor_id)*g_joint_calibration[index].ratio;
                if(jac>1e-6f) force_limit=fminf(force_limit,cap/jac);
            } else if (gain>1e-6f) force_limit=fminf(force_limit,c.voltage_limit_mv/gain);
        }
        float sum = base+next;
        /* support_only 只把轴向力下限设为0，避免主动拉低机身；它不会关闭
         * 摆角控制，也不会保证有足够的地面支撑力。真正力矩仍受电流上限约束。 */
        float force_floor=c.support_only ? 0.0f : -force_limit;
        if (ROBOT_NATIVE_LQR) {
            /* 先检查纯轴向限幅，再按本拍Tp预测J^T混合输出。
             * 只拒绝加深饱和的积分增量；允许卸力/退积分帮助饱和关节恢复。 */
            float accepted=c.speed_ki==0 ? 0 :
                leg_height_integral_accept(integral[side],next,base,force_floor,force_limit);
            if (!isfinite(accepted)) { status=3U;goto stop; }
            float previous_force=fmaxf(force_floor,fminf(base+integral[side],force_limit));
            float candidate_force=fmaxf(force_floor,fminf(base+accepted,force_limit));
            unsigned joint_allows_integral=1U;
            for(unsigned joint=0;joint<2;++joint) {
                unsigned index=side*2+joint;
                unsigned id=g_joint_calibration[index].motor_id;
                unsigned current=MotorCommand_UsesCurrent((uint8_t)id);
                float cap=JOINT_TORQUE_LIMIT_NM;
                if(current) cap=fminf(cap,MotorCommand_CurrentTorqueLimitNm((uint8_t)id)*g_joint_calibration[index].ratio);
                float jf=state->leg[side].jacobian[1-joint],jt=state->leg[side].jacobian[3-joint];
                float old_tau=jf*previous_force+jt*pitch[side];
                float new_tau=jf*candidate_force+jt*pitch[side];
                float previous_violation=fmaxf(0,fabsf(old_tau)-cap);
                float candidate_violation=fmaxf(0,fabsf(new_tau)-cap);
                if(!current && joint_out.torque_mv_per_nm[index]>1e-6f) {
                    float gain=joint_out.torque_mv_per_nm[index],emf=joint_out.back_emf_mv[index];
                    float old_mv=clamp_symmetric(old_tau,cap)*gain+emf;
                    float new_mv=clamp_symmetric(new_tau,cap)*gain+emf;
                    previous_violation=fmaxf(previous_violation,(fabsf(old_mv)-c.voltage_limit_mv)/gain);
                    candidate_violation=fmaxf(candidate_violation,(fabsf(new_mv)-c.voltage_limit_mv)/gain);
                }
                joint_allows_integral &= candidate_violation<=previous_violation+1e-7f;
            }
            if(c.speed_ki==0 || joint_allows_integral)
                integral[side]=accepted;
        } else {
            /* 历史控制路径保留原积分语义。 */
            if (c.speed_ki == 0 || (sum>=force_floor && sum<=force_limit) ||
                (sum > force_limit && speed_error < 0) || (sum < force_floor && speed_error > 0)) integral[side] = next;
        }
        g_leg_height_integral_n[side]=integral[side];
        force_requested[side]=base+integral[side];
        force_bound[side]=force_limit;
        force_saturated[side]=force_requested[side]>force_limit || force_requested[side]<force_floor;
        force[side] = fmaxf(force_floor,fminf(force_requested[side],force_limit));
        for (unsigned joint = 0; joint < 2; ++joint) {
            /* VMC：关节力矩 = J^T * [轴向力F, 虚拟腿摆矩Tp]。
             * 数组按前/后存放，雅可比列按后/前排列，故这里反向取列。
             * F用N、J长度项用m，乘积得到N*m；Tp本身已是N*m，不再乘腿长。 */
            unsigned index = side*2+joint;
            float requested_torque=state->leg[side].jacobian[1-joint]*force[side] +
                                   state->leg[side].jacobian[3-joint]*pitch[side];
            if (!isfinite(requested_torque)) { status=3U;goto stop; }
            unsigned current=MotorCommand_UsesCurrent((uint8_t)g_joint_calibration[index].motor_id);
            float cap=JOINT_TORQUE_LIMIT_NM;
            if(current) cap=fminf(cap,MotorCommand_CurrentTorqueLimitNm((uint8_t)g_joint_calibration[index].motor_id)*g_joint_calibration[index].ratio);
            torque[index]=clamp_symmetric(requested_torque,cap);
            if (fabsf(requested_torque)>cap) {
                joint_out.saturated=1U;g_balance.saturated=1U;
            }
            if(current) continue;
            joint_out.torque_voltage_mv[index]=torque[index]*joint_out.torque_mv_per_nm[index];
            float mv=joint_out.torque_voltage_mv[index]+joint_out.back_emf_mv[index];
            if (!isfinite(mv)) { status = 3U; goto stop; }
            joint_out.requested_mv[index]=mv;
            joint_out.voltage_mv[index]=(int16_t)(c.direction[index]*clamp_symmetric(mv,c.voltage_limit_mv));
            voltage[g_joint_calibration[index].motor_id-1U]=joint_out.voltage_mv[index];
            if (fabsf(mv)>c.voltage_limit_mv) {
                joint_out.saturated=1U; g_balance.saturated=1U;
            }
        }
    }
    if (g_balance.enable) {
        if (!wheel_output_pair(state,g_balance.torque_nm[0],
                           g_robot_motion.differential_requested_nm,
                          g_balance.wheel_limit_mv,
                          g_balance.back_emf_ratio*(robot_control_active ? robot_startup_blend : 1.0f),voltage)) {
            status=3U;goto stop;
        }
        g_robot_motion.differential_applied_nm=g_wheel_output.applied_differential_nm;
        if (g_wheel_output.saturated) g_balance.saturated=1U;
    } else wheel_output_clear();
    status = 1U;
    active = 1U;
    g_joint_output=joint_out;
    goto send;
stop:
    wheel_output_clear();
    joint_output_clear();
    g_dual_hold.enable = 0U;
    g_balance.enable = 0U;
    g_balance.torque_nm[0]=g_balance.torque_nm[1]=0;
    g_balance.startup_wheel_damping_torque_nm=0;
    for(unsigned i=0;i<6;++i) g_balance.wheel_torque_component_nm[i]=0;
    integral[0] = integral[1] = filtered[0] = filtered[1] = feedforward = 0;
    g_leg_height_integral_n[0]=g_leg_height_integral_n[1]=0;
    load_force[0]=load_force[1]=load_applied[0]=load_applied[1]=0;
    balance_reference_clear();
    overlength_elapsed=0;overlength_blend=0;
    pose_bad_elapsed_ms=0U;
    body_bad_elapsed_ms=0U;
    angle_reference[0]=angle_reference[1]=0;
    g_leg_angle_integral_nm[0]=g_leg_angle_integral_nm[1]=0;
    angle_error[0]=angle_error[1]=pitch[0]=pitch[1]=0;
    angle_hold[0]=angle_hold[1]=lqr_pitch[0]=lqr_pitch[1]=0;
    sync_error_filtered=sync_applied[0]=sync_applied[1]=0;
    sync_used[0]=sync_used[1]=0;sync_priority_scale=1;sync_priority_active=0U;
    sync_difference=sync_speed=sync_target_speed=0;
    for (unsigned i=0;i<8;++i) voltage[i]=0;
    for (unsigned i=0;i<4;++i) torque[i]=0;
    force[0]=force[1]=0;
    error_control[0]=error_control[1]=speed_control[0]=speed_control[1]=0;
    overlength_force[0]=overlength_force[1]=0;
    force_requested[0]=force_requested[1]=force_bound[0]=force_bound[1]=0;
    force_saturated[0]=force_saturated[1]=0U;
    /* 停机后连续三个控制周期尝试发送零输出，发送失败会重新开始计数。 */
    if (active || c.enable) stops=3U;
    active=0U;
send:
    if(active) {
        for(unsigned i=0;i<4;++i) {
            unsigned id=g_joint_calibration[i].motor_id;
            /* 模型关节力矩按传动比换算为轴端力矩；安装方向由电机服务处理。 */
            shaft_torque_nm[id-1U]=torque[i]/g_joint_calibration[i].ratio;
            active_motor_mask|=1U<<(id-1U);
        }
        if(g_balance.enable) {
            shaft_torque_nm[WHEEL_LEFT_CAN_ID-1U]=g_wheel_output.applied_per_side_nm[0];
            shaft_torque_nm[WHEEL_RIGHT_CAN_ID-1U]=g_wheel_output.applied_per_side_nm[1];
            active_motor_mask|=(1U<<(WHEEL_LEFT_CAN_ID-1U))|(1U<<(WHEEL_RIGHT_CAN_ID-1U));
        }
    }
    if (output_owned) {
        /* Wheel-only test owns CAN. Reset leg controller without a second
         * command that would overwrite the wheel voltage in this cycle. */
        stops=0U;
    } else if (MotorCommand_SendOutputs(voltage,shaft_torque_nm,active_motor_mask)!=2U && MOTOR_COMMAND_TX_ENABLED) {
        status=4U;g_dual_hold.enable=0U;g_balance.enable=0U;stops=3U;
        wheel_output_clear();
        joint_output_clear();
        sync_error_filtered=sync_applied[0]=sync_applied[1]=0;
        sync_used[0]=sync_used[1]=0;sync_priority_scale=1;sync_priority_active=0U;
        sync_difference=sync_speed=sync_target_speed=0;
        load_force[0]=load_force[1]=load_applied[0]=load_applied[1]=0;
        balance_reference_clear();
    } else if(stops) --stops;
    /* 输出成功后才更新下一周期的承重慢积分；先预测加入补偿后的关节混合输出，
     * 同时检查轴向上限与摆角/反电动势占用的余量。外部电源限流无法由此识别。 */
    if (!ROBOT_NATIVE_LQR && c.load_enable && status==1U) {
        unsigned ready=state->imu_valid &&
            fabsf(state->body_pitch_rad-pitch_trim)<=ROBOT_STARTUP_ANGLE_ERROR_RAD &&
            fabsf(state->body_pitch_rate_rad_s)<=ROBOT_STARTUP_BODY_RATE_RAD_S &&
            fabsf(feedforward-c.feedforward_target_n)<=ROBOT_STARTUP_SUPPORT_ERROR_N &&
            overlength_blend>=1.0f &&
            (!robot_control_active || robot_startup_phase==ROBOT_STARTUP_BALANCE);
        for (unsigned side=0;side<2;++side) {
            load_frozen[side]=1U;
            if (!ready || joint_out.saturated || g_wheel_output.saturated || force_saturated[side] ||
                fabsf(state->leg[side].angle-angle_target)>LEG_LOAD_MAX_LEG_ERROR_RAD ||
                fabsf(state->angle_speed_rad_s[side])>ROBOT_STARTUP_ANGLE_SPEED_RAD_S ||
                fabsf(state->length_speed_m_s[side])>ROBOT_STARTUP_LENGTH_SPEED_M_S) continue;
            float rate=clamp_symmetric(c.load_ki_n_per_m_s*error_control[side],c.load_rate_n_s);
            float candidate=clamp_symmetric(load_force[side]+rate*dt,c.load_limit_n);
            float candidate_force=force_requested[side]+candidate-load_applied[side];
            if (!isfinite(candidate_force) || candidate_force<0 || candidate_force>force_bound[side]) continue;
            unsigned voltage_ok=1U;
            for (unsigned joint=0;joint<2;++joint) {
                unsigned index=side*2+joint;
                if(MotorCommand_UsesCurrent((uint8_t)g_joint_calibration[index].motor_id)) {
                    float candidate_torque=state->leg[side].jacobian[1-joint]*candidate_force+
                                           state->leg[side].jacobian[3-joint]*pitch[side];
                    float cap=MotorCommand_CurrentTorqueLimitNm((uint8_t)g_joint_calibration[index].motor_id)*g_joint_calibration[index].ratio;
                    voltage_ok &= isfinite(candidate_torque) && fabsf(candidate_torque)<=fminf(cap,JOINT_TORQUE_LIMIT_NM);
                    continue;
                }
                float mv=(state->leg[side].jacobian[1-joint]*candidate_force+
                          state->leg[side].jacobian[3-joint]*pitch[side])*
                         joint_out.torque_mv_per_nm[index]+joint_out.back_emf_mv[index];
                voltage_ok &= isfinite(mv) && fabsf(mv)<=c.voltage_limit_mv;
            }
            if (voltage_ok) {
                load_force[side]=candidate;
                load_frozen[side]=0U;
            }
        }
    }
    if (status || c.enable || g_dual_hold.status==1U) g_dual_hold.status=status;
    g_dual_hold.feedforward_n=feedforward;
    g_dual_hold.overlength_blend=overlength_blend;
    g_dual_hold.sync_angle_difference_rad=sync_difference;
    g_dual_hold.sync_speed_difference_rad_s=sync_speed;
    g_dual_hold.sync_filtered_error_rad=sync_error_filtered;
    g_dual_hold.sync_target_speed_rad_s=sync_target_speed;
    g_dual_hold.sync_priority_scale=sync_priority_scale;
    g_dual_hold.sync_priority_active=sync_priority_active;
    for(unsigned i=0;i<2;++i) {
        g_dual_hold.error_m[i]=error[i];g_dual_hold.force_n[i]=force[i];
        g_dual_hold.length_error_control_m[i]=error_control[i];
        g_dual_hold.length_speed_control_m_s[i]=speed_control[i];
        g_dual_hold.overlength_force_n[i]=overlength_force[i];
        g_dual_hold.force_requested_n[i]=force_requested[i];
        g_dual_hold.force_limit_n[i]=force_bound[i];
        g_dual_hold.force_saturated[i]=force_saturated[i];
        g_dual_hold.angle_reference_rad[i]=angle_reference[i];
        g_dual_hold.angle_error_rad[i]=angle_error[i];g_dual_hold.pitch_torque_nm[i]=pitch[i];
        g_dual_hold.angle_hold_torque_nm[i]=angle_hold[i];
        g_dual_hold.lqr_pitch_torque_nm[i]=lqr_pitch[i];
        g_dual_hold.sync_raw_torque_nm[i]=sync_applied[i];
        g_dual_hold.sync_torque_nm[i]=sync_used[i];
        g_dual_hold.load_force_n[i]=load_applied[i];
        g_dual_hold.load_frozen[i]=load_frozen[i];
    }
    for(unsigned i=0;i<4;++i) g_dual_hold.joint_torque_nm[i]=torque[i];
    for(unsigned i=0;i<8;++i) g_dual_hold.voltage_mv[i]=voltage[i];
    return 1U;
}

static uint32_t wheel_test_output(const WheelLegDebug *state)
{
    static uint32_t active, stops;
    static uint32_t zero_count, zero_elapsed;
    static float zero_pitch, zero_rate, zero_min, zero_max;
    const WheelTestDebug c = g_wheel_test;
    uint32_t status = 0U, saturated = 0U, centered = 0U;
    float x[6] = {0}, reference[6] = {0}, u[2] = {0};
    float pitch_error = 0, rate_error = 0;
    int16_t voltage[8] = {0};
    float shaft_torque_nm[8]={0};
    uint32_t active_motor_mask=0U;
    if (!c.zero_request) zero_count=zero_elapsed=0U;
    if (!c.enable && !c.zero_request && !active && !stops) return 0U;
    dual_hold(state,1U);
    if (c.zero_request>1U) { status=3U; goto stop; }
    if (c.zero_request==1U) {
        /* Capture the user's chosen upright pose; wheels/joints stay at true
         * zero throughout. Restart the window on movement or invalid IMU. */
        wheel_output_clear(); active=0U;
        status=MOTOR_COMMAND_TX_ENABLED ? 6U : 5U;
        if (!MOTOR_COMMAND_TX_ENABLED) goto stop;
        float pitch=state->body_pitch_rad/DEG_TO_RAD;
        float rate=state->body_pitch_rate_rad_s;
        if (!state->imu_valid || !state->period_ms || state->period_ms>10U ||
            !isfinite(pitch) || !isfinite(rate) || fabsf(rate)>0.05f) {
            zero_count=zero_elapsed=0U; g_wheel_test.zero_status=3U;
        } else {
            if (!zero_count) zero_min=zero_max=pitch;
            zero_min=fminf(zero_min,pitch); zero_max=fmaxf(zero_max,pitch);
            if (zero_max-zero_min>0.2f) {
                zero_count=zero_elapsed=0U; g_wheel_test.zero_status=3U;
            } else {
                ++zero_count; zero_elapsed+=state->period_ms;
                if (zero_count==1U) { zero_pitch=pitch; zero_rate=rate; }
                else {
                    zero_pitch+=(pitch-zero_pitch)/(float)zero_count;
                    zero_rate+=(rate-zero_rate)/(float)zero_count;
                }
                g_wheel_test.zero_status=1U;
                if (zero_elapsed>=500U) {
                    g_wheel_test.pitch_zero_deg=zero_pitch;
                    g_wheel_test.pitch_rate_bias_rad_s=zero_rate;
                    g_wheel_test.zero_request=0U;
                    g_wheel_test.zero_status=2U;
                    status=c.enable ? 6U : 0U;
                }
            }
        }
        g_wheel_test.zero_samples=zero_count;
        goto send;
    }
    if (!c.enable) goto stop;
    status = 3U;
    if (!isfinite(c.leg_length_m) || c.leg_length_m < LQR_LEG_MIN_M ||
        c.leg_length_m > LQR_LEG_MAX_M || !isfinite(c.scale) ||
        c.scale <= 0 || c.scale > 1 || !isfinite(c.wheel_limit_mv) ||
        c.wheel_limit_mv <= 0 || c.wheel_limit_mv > WHEEL_VOLTAGE_LIMIT_MV ||
        c.zero_request>1U || !isfinite(c.pitch_zero_deg) || fabsf(c.pitch_zero_deg)>180 ||
        !isfinite(c.pitch_rate_bias_rad_s) || !isfinite(c.pitch_deadband_deg) ||
        c.pitch_deadband_deg<0 || c.pitch_deadband_deg>5 || !isfinite(c.rate_deadband_rad_s) ||
        c.rate_deadband_rad_s<0 || c.rate_deadband_rad_s>0.1f) goto stop;
    status = 2U;
    if (!state->imu_valid || !state->period_ms || state->period_ms > 10U) goto stop;
    /* Wheel response test: virtual legs stay vertical relative to the body.
     * Ignore joint geometry, wheel odometry and leg swing torque completely. */
    pitch_error=state->body_pitch_rad/DEG_TO_RAD-c.pitch_zero_deg;
    rate_error=state->body_pitch_rate_rad_s-c.pitch_rate_bias_rad_s;
    /* Continuous deadbands: remove the threshold, avoiding an output step at
     * their boundary. This applies to the wheel response test only. */
    x[0]=(pitch_error-clamp_symmetric(pitch_error,c.pitch_deadband_deg))*DEG_TO_RAD;
    x[1]=rate_error-clamp_symmetric(rate_error,c.rate_deadband_rad_s);
    x[4] = -x[0]; x[5] = -x[1];
    status = 3U;
    if (!lqr_control(c.leg_length_m,x,reference,u)) goto stop;
    u[0] *= c.scale;
    centered=fabsf(pitch_error)<=c.pitch_deadband_deg && fabsf(rate_error)<=c.rate_deadband_rad_s;
    if (centered) wheel_output_clear();
    else {
        /* Restore compensation smoothly outside the center, so a spinning
         * wheel does not introduce a voltage step at the deadband boundary. */
        float blend=fminf(1.0f,fmaxf(fabsf(x[0])/fmaxf(c.pitch_deadband_deg*DEG_TO_RAD,0.002617994f),
                                    fabsf(x[1])/fmaxf(c.rate_deadband_rad_s,0.00872664626f)));
        if (!wheel_output(state,u[0],c.wheel_limit_mv,blend,voltage)) goto stop;
        saturated = g_wheel_output.saturated;
    }
    status = 5U;
    if (!MOTOR_COMMAND_TX_ENABLED) goto stop;
    status = 1U; active = 1U;
    goto send;
stop:
    wheel_output_clear();
    g_wheel_test.enable = 0U;
    g_wheel_test.zero_request=0U; zero_count=zero_elapsed=0U;
    if (active || c.enable) stops = 3U;
    active = 0U;
    u[0] = 0;
    saturated = 0U;
    for (unsigned i=0;i<8;++i) voltage[i]=0;
send:
    if(active) {
        shaft_torque_nm[WHEEL_LEFT_CAN_ID-1U]=g_wheel_output.applied_per_side_nm[0];
        shaft_torque_nm[WHEEL_RIGHT_CAN_ID-1U]=g_wheel_output.applied_per_side_nm[1];
        active_motor_mask=(1U<<(WHEEL_LEFT_CAN_ID-1U))|(1U<<(WHEEL_RIGHT_CAN_ID-1U));
    }
    if (MotorCommand_SendOutputs(voltage,shaft_torque_nm,active_motor_mask)!=2U && MOTOR_COMMAND_TX_ENABLED) {
        status=4U; g_wheel_test.enable=0U; g_wheel_test.zero_request=0U;
        zero_count=zero_elapsed=0U; active=0U; stops=3U;
        u[0]=0; saturated=0U; wheel_output_clear();
        for (unsigned i=0;i<8;++i) voltage[i]=0;
    } else if (stops) --stops;
    if (status || c.enable || c.zero_request || g_wheel_test.status==1U ||
        g_wheel_test.status==6U) g_wheel_test.status=status;
    g_wheel_test.torque_nm=u[0];
    g_wheel_test.centered=centered;
    g_wheel_test.pitch_error_deg=pitch_error;
    g_wheel_test.pitch_rate_error_rad_s=rate_error;
    g_wheel_test.saturated=saturated;
    for (unsigned i=0;i<6;++i) g_wheel_test.state[i]=x[i];
    for (unsigned i=0;i<8;++i) g_wheel_test.voltage_mv[i]=voltage[i];
    return 1U;
}

static void robot_control_stop(uint32_t status)
{
    motion_clear(ROBOT_MOTION_OFF);
    balance_reference_clear();
    g_robot_control.enable=0U;
    g_robot_control.status=status;
    g_dual_hold.enable=0U;
    g_balance.enable=0U;
    robot_control_active=0U;
    robot_startup_phase=ROBOT_STARTUP_OFF;
    robot_startup_elapsed=robot_startup_settle=robot_startup_blend=0;
    g_robot_control.startup_phase=ROBOT_STARTUP_OFF;
    g_robot_control.startup_elapsed_s=g_robot_control.startup_settle_s=g_robot_control.startup_blend=0;
    g_robot_control.startup_ready_mask=0U;
    g_balance.startup_wheel_damping_torque_nm=0;
    remote_armed=remote_off_seen=0U;
    g_robot_control.remote_rearm_ready=0U;
    if (g_robot_control.remote_enable==1U && g_robot_control.remote_switch_on)
        g_robot_control.remote_status=3U;
}

/* 通信故障通知只置位，由控制任务在下周期统一停机并清理模式状态。 */
void WheelLeg_NotifyCanTransportFault(void)
{
    can_transport_fault_pending=1U;
}

uint32_t WheelLeg_CanCommunicationStopped(void)
{
    return !can_transport_fault_pending && !g_robot_control.enable && !g_dual_hold.enable &&
           !g_balance.enable && !g_wheel_test.enable && !g_wheel_test.zero_request;
}

static void robot_remote_update(void)
{
    SbusState remote;
    uint32_t mode=g_robot_control.remote_enable;
    uint32_t on=g_robot_control.remote_on_raw, center=g_robot_control.remote_center_raw;
    uint32_t off=g_robot_control.remote_off_raw, tolerance=g_robot_control.remote_tolerance_raw;
    g_robot_control.remote_link_ok=g_robot_control.remote_switch_on=0U;
    g_robot_control.remote_ch5_percent=0;
    if (mode==0U) {
        remote_armed=remote_off_seen=0U;
        g_robot_control.remote_rearm_ready=0U;
        g_robot_control.remote_status=0U;
        return;
    }
    int32_t on_span=on<=2047U && center<=2047U ? (int32_t)on-(int32_t)center : 0;
    int32_t off_span=off<=2047U && center<=2047U ? (int32_t)off-(int32_t)center : 0;
    if (mode>1U || on>2047U || center>2047U || off>2047U ||
        tolerance==0U || tolerance>100U || on_span*off_span>=0 ||
        abs(on_span)<=(int32_t)(2U*tolerance) || abs(off_span)<=(int32_t)(2U*tolerance)) {
        robot_control_stop(3U);
        g_wheel_test.enable=g_wheel_test.zero_request=0U;
        g_robot_control.remote_status=5U;
        return;
    }
    /* 读取一致的遥控快照，并拒绝超时、失控、丢帧及 UART/DMA 故障。
     * Watch 镜像仅供观察，不能作为控制输入。 */
    uint32_t valid=SBUS_GetState(&remote);
    uint32_t raw=remote.frame.channel[ROBOT_REMOTE_CHANNEL_INDEX];
    g_robot_control.remote_ch5_raw=raw;
    if (!valid || raw>2047U) {
        robot_control_stop(0U);
        g_wheel_test.enable=g_wheel_test.zero_request=0U;
        g_robot_control.remote_status=1U;
        return;
    }
    g_robot_control.remote_link_ok=1U;
    int32_t delta=(int32_t)raw-(int32_t)center;
    float percent=delta*on_span>=0 ? 100.0f*(float)delta/(float)on_span :
                                  -100.0f*(float)delta/(float)off_span;
    g_robot_control.remote_ch5_percent=clamp_symmetric(percent,100.0f);
    uint32_t upper=abs((int32_t)raw-(int32_t)on)<=(int32_t)tolerance;
    uint32_t off_detent=abs(delta)<=(int32_t)tolerance ||
                        abs((int32_t)raw-(int32_t)off)<=(int32_t)tolerance;
    if (!upper) {
        /* 只有落入上挡容差才允许开启；中/下挡确认关闭并准备再次解锁。
         * 挡位间过渡值也停机，但保留此前已见过关闭挡的记录。 */
        uint32_t off_ready=remote_off_seen || off_detent;
        robot_control_stop(0U);
        g_wheel_test.enable=g_wheel_test.zero_request=0U;
        remote_off_seen=off_ready;
        g_robot_control.remote_rearm_ready=off_ready;
        g_robot_control.remote_status=2U;
        return;
    }
    g_robot_control.remote_switch_on=1U;
    if (remote_armed && !g_robot_control.enable) {
        /* Manual stop, just like a fault, must not be overwritten by upper. */
        robot_control_stop(g_robot_control.status);
    }
    if (!remote_armed && remote_off_seen) {
        remote_armed=1U;remote_off_seen=0U;
        g_robot_control.remote_rearm_ready=0U;
        g_wheel_test.enable=g_wheel_test.zero_request=0U;
        g_robot_control.enable=1U;
    } else if (!remote_armed) {
        robot_control_stop(g_robot_control.status);
        g_wheel_test.enable=g_wheel_test.zero_request=0U;
        g_robot_control.remote_status=3U;
        return;
    }
    g_robot_control.remote_status=4U;
}

static void robot_startup_update(const WheelLegDebug *state)
{
    /* Native模式的状态机只描述就绪进度：
     * PREPARE：姿态/参考连续达标0.25s；RAMP：再等待0.5s；BALANCE：允许摇杆行驶。
     * 三个阶段都执行完整LQR；支撑力和theta参考各自平滑变化，不按状态删除反馈。
     * 普通镜像没有按运行时长自动停机；CH5、反馈故障及倾倒保护仍能停机。 */
    uint32_t ready=0U;
    float forward[2]={0}, gap=0;
    float dt=state->period_ms*.001f;
    if (state->leg_valid[0] && state->leg_valid[1]) {
        float body_target=0,leg_target=g_dual_hold.angle_target_rad;
#if ROBOT_NATIVE_LQR
        /* 就绪判定与真正执行的平衡点一致。配平角非零时，机身和局部腿角
         * 都不应再强制接近0/90度，否则合法目标也可能永远停在PREPARE。 */
        if(g_balance.enable && g_balance.status==1U) {
            body_target=g_balance.pitch_trim_applied_deg*DEG_TO_RAD;
            leg_target-=body_target;
            leg_target+=g_balance_equilibrium_theta_rad;
        }
#endif
        unsigned length_ok=fabsf(state->leg[0].length-state->leg[1].length)<=ROBOT_STARTUP_LENGTH_ERROR_M;
        unsigned pose_ok=state->imu_valid &&
            fabsf(state->body_pitch_rad-body_target)<=ROBOT_STARTUP_ANGLE_ERROR_RAD;
        unsigned speed_ok=state->imu_valid &&
            fabsf(state->body_pitch_rate_rad_s)<=ROBOT_STARTUP_BODY_RATE_RAD_S;
        for(unsigned side=0;side<2;++side) {
            forward[side]=state->leg[side].length*cosf(state->leg[side].angle);
            length_ok &= fabsf(state->leg[side].length-g_dual_hold.target_m)<=ROBOT_STARTUP_LENGTH_ERROR_M;
            pose_ok &= fabsf(state->leg[side].angle-leg_target)<=ROBOT_STARTUP_ANGLE_ERROR_RAD;
            speed_ok &= fabsf(state->angle_speed_rad_s[side])<=ROBOT_STARTUP_ANGLE_SPEED_RAD_S &&
                        fabsf(state->length_speed_m_s[side])<=ROBOT_STARTUP_LENGTH_SPEED_M_S &&
                        state->wheel_valid[side];
        }
        /* Pose readiness controls the native PREPARE handover below. Length,
         * support, velocity and wheel-offset bits remain diagnostics. */
        gap=forward[0]-forward[1];
        if (isfinite(gap) && fabsf(gap)<=ROBOT_STARTUP_GAP_M) ready|=ROBOT_READY_GAP;
        if (length_ok) ready|=ROBOT_READY_LENGTH;
        if (pose_ok) ready|=ROBOT_READY_POSE;
        if (speed_ok) ready|=ROBOT_READY_SPEED;
        if (isfinite(g_dual_hold.feedforward_n) && isfinite(g_dual_hold.feedforward_target_n) &&
            fabsf(g_dual_hold.feedforward_n-g_dual_hold.feedforward_target_n)<=ROBOT_STARTUP_SUPPORT_ERROR_N)
            ready|=ROBOT_READY_SUPPORT;
    }
    g_robot_control.wheel_forward_m[0]=forward[0];g_robot_control.wheel_forward_m[1]=forward[1];
    g_robot_control.wheel_forward_gap_m=gap;
    g_robot_control.startup_ready_mask=ready;
    /* Pure support does not need unused wheel/IMU feedback or a balance ramp.
     * Its original dependency checks remain in balance_update. On restoring
     * real LQR, active preview handling still captures a fresh odometry origin. */
    unsigned output_requested=g_balance.wheel_scale!=0.0f || g_balance.leg_scale!=0.0f || g_balance.back_emf_ratio!=0.0f;
    if (!g_robot_control.startup_enable || !output_requested) {
        robot_startup_phase=ROBOT_STARTUP_BALANCE;
        robot_startup_blend=1.0f;
    } else {
        if (!robot_control_active) {
#if ROBOT_NATIVE_LQR
            robot_startup_phase=ROBOT_STARTUP_PREPARE;
#else
            robot_startup_phase=ROBOT_STARTUP_RAMP;
#endif
            robot_startup_elapsed=robot_startup_settle=robot_startup_blend=0;
        }
        if (state->period_ms && state->period_ms<=10U &&
            state->state_valid[0] && state->state_valid[1] &&
            state->lqr_valid[0] && state->lqr_valid[1]) {
            robot_startup_elapsed=fminf(robot_startup_elapsed+dt,3600.0f);
            if (robot_startup_phase==ROBOT_STARTUP_PREPARE) {
#if ROBOT_NATIVE_LQR
                /* Readiness gates motion permission, not balancing output.
                 * Both full LQR rows remain active while pose/reference
                 * preparation and its continuous dwell are incomplete. */
                float pitch_trim=g_balance.enable && g_balance.status==1U ?
                    g_balance.pitch_trim_applied_deg*DEG_TO_RAD : 0;
                float parking_lean=g_balance.enable && g_balance.status==1U ?
                    g_balance.parking_lean_rad+g_balance.motion_lean_rad : 0;
                if(g_balance.enable && g_balance.status==1U)
                    parking_lean=g_balance_equilibrium_theta_rad+g_balance.parking_lean_rad;
                float target=g_dual_hold.angle_target_rad-pitch_trim+parking_lean;
                float tolerance=fmaxf(1e-5f,g_dual_hold.angle_reference_rate_rad_s*dt);
                unsigned references_ready=isfinite(target);
                for(unsigned side=0;side<2;++side) {
                    float reference=g_dual_hold.angle_reference_rad[side];
                    references_ready &= isfinite(reference) &&
                        fabsf(remainderf(reference-target,6.28318530718f))<=tolerance;
                }
                unsigned prepared=(ready&ROBOT_READY_POSE) && references_ready;
                robot_startup_settle=prepared ?
                    fminf(robot_startup_settle+dt,ROBOT_NATIVE_PREPARE_SETTLE_S) : 0;
                if(robot_startup_settle>=ROBOT_NATIVE_PREPARE_SETTLE_S)
                    robot_startup_phase=ROBOT_STARTUP_RAMP;
#else
                robot_startup_phase=ROBOT_STARTUP_RAMP;
#endif
            } else if (robot_startup_phase==ROBOT_STARTUP_RAMP) {
                /* Pose-qualified preparation has completed. This dwell
                 * delays motion permission without changing feedback gains;
                 * invalid feedback still
                 * faults through balance_update/robot_control_finish. */
                robot_startup_blend=fminf(1.0f,robot_startup_blend+dt/ROBOT_STARTUP_RAMP_S);
                if (robot_startup_blend>=1.0f) robot_startup_phase=ROBOT_STARTUP_BALANCE;
            }
        }
    }
    g_robot_control.startup_phase=robot_startup_phase;
    g_robot_control.startup_elapsed_s=robot_startup_elapsed;
    g_robot_control.startup_settle_s=robot_startup_settle;
    g_robot_control.startup_blend=robot_startup_blend;
}

static uint32_t robot_control_prepare(const WheelLegDebug *state)
{
    uint32_t enable=g_robot_control.enable;
    /* Keep the existing wheel-only/zeroing path as the exclusive CAN owner. */
    if (g_wheel_test.enable || g_wheel_test.zero_request) {
        if (enable || robot_control_active) robot_control_stop(6U);
        return 0U;
    }
    if (enable>1U) { robot_control_stop(3U); return 0U; }
    if (!enable) {
        if (robot_control_active) robot_control_stop(0U);
        return 0U;
    }
    /* Honour an individual stop while combined output is active; never
     * overwrite that stop with an automatic re-enable on the next cycle. */
    if (robot_control_active && (!g_dual_hold.enable || !g_balance.enable)) {
        robot_control_stop(0U);
        return 0U;
    }
    g_dual_hold.enable=1U;
    g_balance.enable=1U;
    robot_startup_update(state);
    robot_control_active=1U;
    return 1U;
}

/* 在清理控制状态前锁存故障现场；正常停机和后续反馈恢复不覆盖此记录。 */
static void robot_control_record_fault(const WheelLegDebug *state, uint32_t status)
{
    uint32_t reason=ROBOT_FAULT_STATE;
    if (status==7U) reason=ROBOT_FAULT_LEG_POSE;
    else if (status==8U) reason=ROBOT_FAULT_BODY_PITCH;
    else if (status==3U) reason=ROBOT_FAULT_PARAMETERS;
    else if (status==4U) reason=ROBOT_FAULT_TX;
    else if (status==5U) reason=ROBOT_FAULT_TX_DISABLED;
    else if (!state->period_ms || state->period_ms>10U) reason=ROBOT_FAULT_TIMING;
    else if (state->leg_valid[0] && state->leg_valid[1] &&
             (state->leg[0].length<LQR_LEG_MIN_M || state->leg[0].length>LQR_LEG_MAX_M ||
              state->leg[1].length<LQR_LEG_MIN_M || state->leg[1].length>LQR_LEG_MAX_M))
        reason=ROBOT_FAULT_LENGTH_RANGE;
    else if (state->feedback_detail!=WHEEL_LEG_FEEDBACK_OK) reason=ROBOT_FAULT_FEEDBACK;
    else if (!state->leg_valid[0] || !state->leg_valid[1]) reason=ROBOT_FAULT_GEOMETRY;
    /* Preserve the exact failed observer snapshot before STOP or a newer RX
     * page can replace it. Passive offline observations never latch a fault. */
    if (reason==ROBOT_FAULT_FEEDBACK) WheelLeg_LatchFeedbackFailure(state);
    ++g_robot_control.last_fault.count;
    g_robot_control.last_fault.tick_ms=state->tick_ms;
    g_robot_control.last_fault.reason=reason;
    g_robot_control.last_fault.dual_status=g_dual_hold.status;
    g_robot_control.last_fault.balance_status=g_balance.status;
    g_robot_control.last_fault.feedback_detail=state->feedback_detail;
    g_robot_control.last_fault.motor_id=state->feedback_motor_id;
    g_robot_control.last_fault.feedback_ms=state->feedback_ms;
    g_robot_control.last_fault.period_ms=state->period_ms;
    for (unsigned side=0;side<2;++side) {
        g_robot_control.last_fault.leg_length_m[side]=state->leg[side].length;
        g_robot_control.last_fault.leg_valid[side]=state->leg_valid[side];
    }
}

static void robot_control_finish(uint32_t requested, const WheelLegDebug *state)
{
    if (!requested) return;
    if (g_dual_hold.enable && g_balance.enable &&
        g_dual_hold.status==1U && g_balance.status==1U) {
        g_robot_control.status=1U;
    } else {
        uint32_t status=g_dual_hold.status;
        robot_control_record_fault(state,status);
        robot_control_stop(status>=2U && status<=5U ? status : 2U);
    }
}

typedef char WheelBalanceTraceRowSizeCheck[sizeof(WheelBalanceTraceRow)==180U?1:-1];
typedef char WheelBalanceTracePrefixCheck[offsetof(WheelBalanceTraceRow,raw_ch2)==64U?1:-1];
typedef char WheelBalanceTraceServoPrefixCheck[offsetof(WheelBalanceTraceRow,servo_integral_next_m_s)==128U?1:-1];
volatile WheelBalanceTraceDebug g_balance_trace;
static void balance_trace_capture(const WheelLegDebug *state)
{
    static uint32_t was_active, last_tick;
    uint32_t active=g_robot_control.enable && g_robot_control.status==1U &&
                    g_dual_hold.status==1U && g_balance.status==1U;
    if (!active) { was_active=0U; return; }
    if (!was_active) {
        g_balance_trace.next=g_balance_trace.count=0U;
        last_tick=state->tick_ms-WHEEL_BALANCE_TRACE_PERIOD_MS;
    }
    was_active=1U;
    if ((uint32_t)(state->tick_ms-last_tick)<WHEEL_BALANCE_TRACE_PERIOD_MS) return;
    last_tick=state->tick_ms;
    WheelBalanceTraceRow row={0};
    row.tick_ms=state->tick_ms;
    row.flags=1U | (g_wheel_output.saturated?2U:0U) |
              (g_wheel_output.torque_saturated?4U:0U) |
              (BALANCE_LEG_LQR_TORQUE_LIMIT_NM>0 && fabsf(g_balance.torque_nm[1])>=BALANCE_LEG_LQR_TORQUE_LIMIT_NM?8U:0U) |
              (g_balance.position_locked?16U:0U) |
              ((uint32_t)g_robot_control.startup_phase<<8);
    for (unsigned i=0;i<6;++i) row.state[i]=g_balance.state[i];
    row.position_error_m=g_balance.position_locked ? g_balance.parking_error_m :
                         g_balance.state[2]-g_balance.reference[2];
    row.wheel_requested_nm=g_wheel_output.requested_torque_nm;
    row.wheel_applied_nm=g_wheel_output.applied_torque_nm;
    row.leg_tp_nm=g_balance.torque_nm[1];
    row.motion_component_nm[0]=g_balance.wheel_torque_component_nm[1];
    row.motion_component_nm[1]=g_balance.wheel_torque_component_nm[5];
    for (unsigned side=0;side<2;++side) row.wheel_voltage_mv[side]=g_wheel_output.voltage_mv[side];
    row.leg_length_m=g_balance.leg_length_m;
    row.raw_ch2=motion_input_trace.raw_ch2;
    row.motion_status=(uint8_t)g_robot_motion.command.status;
    row.input_flags=motion_input_trace.flags |
        (g_robot_motion.neutral_ready?WHEEL_TRACE_NEUTRAL_READY:0U) |
        (g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY?WHEEL_TRACE_COMMAND_READY:0U);
    row.source_frame_count=motion_input_trace.source_frame_count;
    row.sbus_age_ms=motion_input_trace.age_ms;
    row.normalized_forward=motion_input_trace.normalized_forward;
    row.requested_forward_m_s=g_robot_motion.command.requested_forward_m_s;
    row.slewed_forward_m_s=g_robot_motion.command.forward_m_s;
    const unsigned reference_indices[4]={0U,2U,3U,4U};
    for(unsigned i=0;i<4;++i) row.reference[i]=g_balance.reference[reference_indices[i]];
    row.parking_goal_m=g_balance.parking_target_m;
    row.bias_offset_m=g_balance_bias.estimated_offset_m;
    row.bias_applied_phi_rad=g_balance_bias.applied_phi_rad;
    row.bias_gate_flags=g_balance_bias.gate_mask;
    row.requested_yaw_rad_s=g_robot_motion.command.requested_turn_rad_s;
    row.slewed_yaw_rad_s=g_robot_motion.command.turn_rad_s;
    row.servo_integral_next_m_s=g_balance_servo.integral_m_s;
    row.servo_blend=g_balance_servo.blend;
    row.servo_wheel_integral_nm=g_balance_servo.wheel_integral_nm;
    row.servo_leg_integral_nm=g_balance_servo.leg_integral_nm;
    row.servo_motion_flags=(g_balance_servo.gate_mask&WHEEL_TRACE_SERVO_GATE_MASK) |
        (g_balance_servo.enable?WHEEL_TRACE_SERVO_ENABLED:0U) |
        (g_balance_servo.active?WHEEL_TRACE_SERVO_ACTIVE:0U) |
        (g_balance_servo.learning?WHEEL_TRACE_SERVO_LEARNING:0U) |
        ((g_balance_motion_reference.phase<<WHEEL_TRACE_MOTION_PHASE_SHIFT)&WHEEL_TRACE_MOTION_PHASE_MASK) |
        (g_balance_motion_reference.enable?WHEEL_TRACE_MOTION_ENABLED:0U) |
        (g_joint_output.saturated?WHEEL_TRACE_JOINT_SATURATED:0U) |
        (g_dual_hold.force_saturated[0]?WHEEL_TRACE_SUPPORT_LEFT_SATURATED:0U) |
        (g_dual_hold.force_saturated[1]?WHEEL_TRACE_SUPPORT_RIGHT_SATURATED:0U);
    for(unsigned side=0;side<2;++side) {
        row.leg_angle_rad[side]=state->leg_valid[side]?state->leg[side].angle:NAN;
        row.leg_angle_speed_rad_s[side]=state->leg_valid[side]?state->angle_speed_rad_s[side]:NAN;
        row.leg_lengths_m[side]=state->leg_valid[side]?state->leg[side].length:NAN;
    }
    row.measured_leg_tp_nm=g_balance_bias.actual_pitch_torque_nm;
    row.parking_blend=g_balance_servo.parking_blend;
    unsigned next=g_balance_trace.next;
    g_balance_trace.rows[next]=row;
    g_balance_trace.next=(next+1U)%WHEEL_BALANCE_TRACE_ROWS;
    if (g_balance_trace.count<WHEEL_BALANCE_TRACE_ROWS) ++g_balance_trace.count;
    ++g_balance_trace.sequence;
}

#if ROBOT_LEG_PD_TEST_SIDE >= 0
/* Isolate the actual angle-PD/J^T/voltage path. No support, sync, LQR or
 * wheel owner can run in this image, including through Watch enables. */
static void leg_pd_test_update(const WheelLegDebug *state)
{
    static uint32_t active, started_ms;
    const unsigned side=ROBOT_LEG_PD_TEST_SIDE;
    g_wheel_test.enable=g_wheel_test.zero_request=0U;
    g_balance.enable=0U;
    /* Advertise the disabled components accurately in this profile's log. */
    g_balance.wheel_scale=g_balance.leg_scale=g_balance.back_emf_ratio=0;
    balance_update(state,0U);
    g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
    g_dual_hold.feedforward_target_n=g_dual_hold.overlength_kp_n_per_m=0;
    g_dual_hold.sync_ratio=0;
    g_dual_hold.load_enable=0U;
    g_dual_hold.back_emf_ratio=0;
    g_dual_hold.angle_reference_mode=LEG_ANGLE_NEUTRAL;
    g_dual_hold.angle_target_rad=1.5707963267948966f;
    g_dual_hold.angle_reference_rate_rad_s=LEG_PD_TEST_REFERENCE_RATE_RAD_S;
    g_dual_hold.angle_kp_nm_per_rad=LEG_PD_TEST_ANGLE_KP_NM_PER_RAD;
    g_dual_hold.angle_kd_nm_s_per_rad=LEG_PD_TEST_ANGLE_KD_NM_S_PER_RAD;
    g_dual_hold.angle_limit_nm=LEG_PD_TEST_ANGLE_LIMIT_NM;
    g_dual_hold.voltage_limit_mv=LEG_PD_TEST_VOLTAGE_LIMIT_MV;
#if ROBOT_LEG_VOLTAGE_PROBE
    g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
    g_dual_hold.angle_limit_nm=LEG_VOLTAGE_PROBE_MAX_TP_NM;
    g_dual_hold.voltage_limit_mv=LEG_VOLTAGE_PROBE_PEAK_MV;
    const uint32_t duration_ms=LEG_VOLTAGE_PROBE_DURATION_MS;
#else
    const uint32_t duration_ms=3000U;
#endif
    if (g_robot_control.enable && !g_robot_control.feedback_only) {
        float error=state->leg[side].angle-1.5707963267948966f;
#if ROBOT_LEG_VOLTAGE_PROBE
        float speed_limit=active?LEG_VOLTAGE_PROBE_RUNNING_SPEED_RAD_S:1.0f;
#else
        float speed_limit=1.0f;
#endif
        unsigned valid=state->leg_valid[0] && state->leg_valid[1] &&
            state->period_ms && state->period_ms<=10U &&
            isfinite(error) && fabsf(error)<=(active?20.0f:15.0f)*DEG_TO_RAD &&
            isfinite(state->angle_speed_rad_s[side]) &&
            fabsf(state->angle_speed_rad_s[side])<=speed_limit;
        for (unsigned i=0;i<2;++i)
            valid &= state->leg[i].length>=LQR_LEG_MIN_M &&
                     state->leg[i].length<=LQR_LEG_MAX_M;
#if ROBOT_LEG_VOLTAGE_PROBE
        /* Choose direction once from the initial neutral error. This
         * fixed-direction, operator-stopped probe may cross90degrees;
         * keep the running pose envelope and hard2second timeout. */
        if(!active && fabsf(error)<2.0f*DEG_TO_RAD) valid=0;
#endif
        if (!valid || (active && (uint32_t)(state->tick_ms-started_ms)>=duration_ms)) {
            if (!valid) robot_control_record_fault(state,2U);
            robot_control_stop(valid?0U:2U);
        } else {
            if (!active) {
                started_ms=state->tick_ms;
#if ROBOT_LEG_VOLTAGE_PROBE
                leg_voltage_probe_direction=error<0?1.0f:-1.0f;
#endif
            }
            active=1U;
        }
    }
    g_dual_hold.enable=g_robot_control.enable && !g_robot_control.feedback_only;
    dual_hold(state,0U);
    if (g_dual_hold.enable && g_dual_hold.status==1U) g_robot_control.status=1U;
    else {
        if (g_robot_control.enable) {
            uint32_t status=g_dual_hold.status;
            robot_control_record_fault(state,status);
            robot_control_stop(status);
            dual_hold(state,0U);
        }
        active=0U;
#if ROBOT_LEG_VOLTAGE_PROBE
        leg_voltage_probe_direction=0;
#endif
    }
}
#endif

void WheelLeg_Update(void)
{
    if(can_transport_fault_pending) {
        robot_control_stop(4U);
        g_wheel_test.enable=g_wheel_test.zero_request=0U;
        can_transport_fault_pending=0U;
    }
    MotorCommand_BeginCycle();
    WheelLegDebug out;
    WheelLeg_Observe(&out);
#if ROBOT_LEG_PD_TEST_SIDE >= 0
    /* The test always requires the physical remote OFF->ON sequence. */
    g_robot_control.remote_enable=1U;
#endif
    robot_remote_update();
#if ROBOT_CAPTURE_BUILD
    /* No independent owners or unrecorded launches in this short-run image.
     * Preserve the last trial until exported/rearmed explicitly. */
    g_wheel_test.enable=g_wheel_test.zero_request=0U;
    g_robot_bench_limit_ms=ROBOT_CAPTURE_LIMIT_MS;
    if(g_robot_control.enable && !robot_control_active &&
       g_lqr_record.state!=LQR_RECORD_READY)
        robot_control_stop(0U);
#endif
    if (!g_robot_bench_limit_ms || !g_robot_control.enable) robot_bench_running=0U;
    else {
        uint32_t now=HAL_GetTick();
        if (!robot_bench_running) {
            robot_bench_start_ms=now;robot_bench_running=1U;
        }
        g_robot_bench_elapsed_ms=now-robot_bench_start_ms;
        if (g_robot_bench_limit_ms>60000U || g_robot_bench_elapsed_ms>=g_robot_bench_limit_ms) {
#if ROBOT_CAPTURE_BUILD
            /* Stop and require a fresh physical OFF->ON. The saved trial
             * must also be exported/rearmed before another launch. */
            robot_control_stop(0U);
            robot_bench_running=0U;
#else
            g_robot_control.feedback_only=1U;
            g_robot_bench_limit_ms=0U;robot_bench_running=0U;
#endif
        }
    }
    if (g_robot_control.feedback_only || g_lqr_record.storage_busy) {
        /* Keep feedback acquisition alive, but stop every actuator owner,
         * including independent wheel/zero tests and manual Watch enables.
         * Leaving this mode while CH5 is high still requires OFF/rearm. */
        robot_control_stop(0U);
        g_wheel_test.enable=g_wheel_test.zero_request=0U;
    }
#if ROBOT_LEGS_ONLY_CALIBRATION
    /* Physical CH5 owns both legs; no wheel/LQR/independent wheel owner.
     * MotorFoc also rejects wheel pulses and energizing maintenance. */
    g_balance.enable=0U;
    g_balance.torque_nm[0]=g_balance.torque_nm[1]=0;
    g_wheel_test.enable=g_wheel_test.zero_request=0U;
    g_dual_hold.enable=g_robot_control.enable && !g_robot_control.feedback_only;
    dual_hold(&out,0U);
    if (g_dual_hold.enable && g_dual_hold.status==1U) g_robot_control.status=1U;
    else if (g_robot_control.enable) {
        uint32_t status=g_dual_hold.status;
        robot_control_record_fault(&out,status);
        robot_control_stop(status);
        dual_hold(&out,0U);
    }
    g_wheel_leg=out;
    RobotTelemetry_Capture(&out);
    MotorCommand_EndCycle();
    return;
#endif
#if ROBOT_LEG_PD_TEST_SIDE >= 0
    leg_pd_test_update(&out);
    /* Only an accepted, actively isolated PD test consumes READY. Keep the
     * observer running after its 3-second timeout and physical CH5 stop. */
    if(g_robot_control.enable && g_robot_control.status==1U && g_dual_hold.status==1U)
        LqrStartupRecord_Begin(&out);
    LqrStartupRecord_Capture(&out);
    g_wheel_leg=out;
    RobotTelemetry_Capture(&out);
    MotorCommand_EndCycle();
    return;
#endif
    /* Capture the requested start before validation can reject it. This
     * observes failure only; it does not enable outputs or weaken guards. */
    if(g_robot_control.enable && !g_robot_control.feedback_only && !g_lqr_record.storage_busy)
        LqrStartupRecord_Begin(&out);
    uint32_t robot_requested=robot_control_prepare(&out);
    motion_update(&out);
    if (wheel_test_output(&out)) balance_update(&out,0U);
    else {
        balance_update(&out,g_dual_hold.enable);
        dual_hold(&out,0U);
    }
    robot_control_finish(robot_requested,&out);
    balance_trace_capture(&out);
    LqrStartupRecord_Capture(&out);
    g_wheel_leg = out;
    RobotTelemetry_Capture(&out);
    MotorCommand_EndCycle();
}
