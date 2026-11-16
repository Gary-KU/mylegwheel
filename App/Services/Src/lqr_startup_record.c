#include "lqr_startup_record.h"
#include "wheel_leg_config.h"
#include "joint_calibration.h"
#include "main.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
#if MYLEGWHEEL_A1_CURRENT
#include "motor_foc.h"
#include "FreeRTOS.h"
#include "task.h"
#endif
/* 固定布局与 Flash/主机解码共用；结构大小变化必须同步升级记录格式。 */
typedef char RecordHeaderSize[sizeof(LqrRecordHeader)==128 ? 1:-1];
typedef char RecordRowSize[sizeof(LqrRecordRow)==512 ? 1:-1];
typedef char RecordConfigurationSize[sizeof(LqrRecordConfiguration)==384 ? 1:-1];
typedef char RecordFlashFits[sizeof(LqrRecordData)<=LQR_RECORD_FLASH_BYTES ? 1:-1];
volatile LqrRecordDebug g_lqr_record={.enable=1U,.auto_save=1U,.fall_pitch_deg=35.0f,
                                    .fall_confirm_ms=40U,.tail_ms=LQR_RECORD_TAIL_MS};
volatile LqrRecordData g_lqr_record_data __attribute__((section(".bss.lqr_record"),aligned(32)));
static uint32_t initialized,last_tick,start_fault,fall_since,fall_tracking;
static uint32_t triggered,trigger_tick,trigger_reason,off_since,off_tracking;
static uint32_t tail_stopped;
static uint32_t parking_goal_valid;
static float parking_goal;

uint32_t LqrStartupRecord_CRC32(uint32_t crc,const void *bytes,uint32_t size)
{
    const uint8_t *p=bytes;
    for(uint32_t i=0;i<size;++i) {
        crc^=p[i];
        for(unsigned bit=0;bit<8;++bit) crc=(crc>>1)^((crc&1U)?0xedb88320U:0U);
    }
    return crc;
}
static unsigned foc_outputs_off(void)
{
#if MYLEGWHEEL_A1_CURRENT
    if (!MotorFoc_TransportMotionStopped()) return 0U;
    uint32_t now=HAL_GetTick(),ready=1U;
    taskENTER_CRITICAL();
    for (unsigned slot=0;slot<8;++slot) if (MOTOR_FOC_NODE_MASK&(1U<<slot)) {
        const volatile MotorFocNode *n=&g_motor_foc.node[slot];
        if (!n->status_count || (uint32_t)(now-n->status_ms)>MOTOR_FOC_STATUS_TIMEOUT_MS || (n->flags&1U)) ready=0U;
    }
    taskEXIT_CRITICAL();
    return ready;
#else
    return 1U;
#endif
}
static unsigned outputs_off(void)
{
    return foc_outputs_off() && !g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable &&
           !g_wheel_test.enable && !g_wheel_test.zero_request &&
           (!g_robot_control.remote_enable || !g_robot_control.remote_switch_on);
}
void LqrStartupRecord_Init(void)
{
    if(initialized) return;
    LqrRecordHeader saved;
    if(LqrRecordStorage_Load(&saved)) {
        g_lqr_record_data.header=saved;
        g_lqr_record.count=saved.count;g_lqr_record.reason=saved.reason;
        g_lqr_record.trigger_index=saved.trigger_index;
        g_lqr_record.state=LQR_RECORD_SAVED;g_lqr_record.flash_present=1U;
        /* Rows stay in Flash after reboot; the reader must not use empty RAM. */
        g_lqr_record.ram_valid=0U;
        /* V1 could retain a one-row failed attempt without any output. Keep
         * the old Flash file, but allow a successful run to record over it. */
        if(saved.version<=2U && saved.reason==LQR_RECORD_FAULT && saved.count==1U && !saved.recorded_startup_ok) {
            g_lqr_record.state=LQR_RECORD_READY;g_lqr_record.count=g_lqr_record.reason=0U;
            g_lqr_record.trigger_index=UINT32_MAX;
        }
    }
    initialized=1U;
}
void LqrStartupRecord_Begin(const WheelLegDebug *state)
{
    if(!initialized || !state || !g_lqr_record.enable ||
       g_lqr_record.state!=LQR_RECORD_READY || g_lqr_record.storage_busy) return;
    LqrRecordHeader h={0};
    h.magic=LQR_RECORD_MAGIC;h.version=LQR_RECORD_VERSION;h.build_id=LQR_RECORD_BUILD_ID;
    h.row_bytes=sizeof(LqrRecordRow);h.capacity=LQR_RECORD_ROWS;
    h.trigger_index=UINT32_MAX;h.start_tick_ms=state->tick_ms;
    h.period_ms=LQR_RECORD_PERIOD_MS;h.generation=g_lqr_record_data.header.generation+1U;
    h.wheel_scale=g_balance.wheel_scale;h.leg_scale=g_balance.leg_scale;
    h.wheel_limit_mv=g_balance.wheel_limit_mv;h.joint_limit_mv=g_dual_hold.voltage_limit_mv;
    h.support_target_n=g_dual_hold.feedforward_target_n;h.pitch_trim_deg=g_balance.pitch_trim_deg;
    h.back_emf_ratio=g_balance.back_emf_ratio;h.fall_pitch_deg=g_lqr_record.fall_pitch_deg;
    h.supply_model_v=g_wheel_output.supply_voltage_v;h.joint_driver_supply_v=JOINT_DRIVER_SUPPLY_VOLTAGE_V;
    h.wheel_rate_scale=g_balance.wheel_rate_scale;h.recorded_startup_ok=0U;
    for(unsigned i=0;i<2;++i) h.wheel_nm_per_v[i]=g_wheel_output.torque_ratio_nm_per_v[i];
    for(unsigned i=0;i<4;++i) h.joint_nm_per_v[i]=g_dual_hold.torque_ratio_nm_per_v[i];
    /* 开始时保存参数与标定快照，离线分析使用本轮配置，而非读取时的 Watch 值。 */
    const DualHoldDebug c=g_dual_hold;
    const float parameters[40]={c.target_m,c.position_kp,c.speed_kp,c.speed_ki,
        c.max_speed_m_s,c.filter_tau_s,c.max_force_n,c.feedforward_rate_n_s,
        c.overlength_kp_n_per_m,c.overlength_margin_m,c.overlength_startup_s,c.overlength_transition_m,
        c.length_error_deadband_m,c.length_speed_deadband_m_s,
        c.angle_kp_nm_per_rad,c.angle_kd_nm_s_per_rad,c.angle_limit_nm,c.angle_target_rad,
        c.angle_reference_rate_rad_s,g_balance.leg_hold_ratio,
        c.sync_ratio,c.sync_position_kp,c.sync_speed_kp,c.sync_max_speed_rad_s,
        c.sync_filter_tau_s,c.sync_limit_nm,c.sync_rate_nm_s,c.back_emf_ratio,
        BALANCE_LEG_LQR_TORQUE_LIMIT_NM,JOINT_TORQUE_LIMIT_NM,
        c.load_ki_n_per_m_s,c.load_limit_n,c.load_rate_n_s,
        (float)c.angle_reference_mode,(float)c.support_only,(float)c.load_enable,
        ROBOT_BODY_PITCH_LIMIT_RAD,BODY_PITCH_DIRECTION,WHEEL_RADIUS_M,ROBOT_STARTUP_RAMP_S};
    for(unsigned i=0;i<40;++i) g_lqr_record_data.configuration.parameters[i]=parameters[i];
    for(unsigned i=0;i<4;++i) {
        g_lqr_record_data.configuration.encoder_reference_rad[i]=g_joint_calibration[i].encoder_reference_rad;
        g_lqr_record_data.configuration.model_reference_rad[i]=g_joint_calibration[i].model_reference_rad;
        g_lqr_record_data.configuration.encoder_ratio[i]=g_joint_calibration[i].ratio;
        g_lqr_record_data.configuration.encoder_direction[i]=g_joint_calibration[i].direction;
        g_lqr_record_data.configuration.output_direction[i]=c.direction[i];
        g_lqr_record_data.configuration.joint_motor_id[i]=g_joint_calibration[i].motor_id;
    }
    const unsigned node_ids[6]={1,2,3,5,6,7};
    g_lqr_record_data.configuration.current_backend_mask=MYLEGWHEEL_A1_CURRENT?MOTOR_FOC_NODE_MASK:0U;
    g_lqr_record_data.configuration.motion_enable_mask=MOTOR_FOC_OUTPUT_ENABLE_MASK;
    for (unsigned i=0;i<6;++i) {
        g_lqr_record_data.configuration.foc_node_id[i]=node_ids[i];
#if MYLEGWHEEL_A1_CURRENT
        const volatile MotorFocCalibration *cal=&g_motor_foc.calibration[node_ids[i]-1U];
        g_lqr_record_data.configuration.foc_kt_nm_per_a[i]=cal->kt_nm_per_a;
        g_lqr_record_data.configuration.foc_direction[i]=cal->direction;
        g_lqr_record_data.configuration.foc_current_limit_a[i]=cal->current_limit_a;
        g_lqr_record_data.configuration.minimum_iq_slew_a_per_s[i]=cal->minimum_iq_slew_a_per_s;
#else
        g_lqr_record_data.configuration.foc_kt_nm_per_a[i]=0;
        g_lqr_record_data.configuration.foc_direction[i]=0;
        g_lqr_record_data.configuration.foc_current_limit_a[i]=0;
        g_lqr_record_data.configuration.minimum_iq_slew_a_per_s[i]=0;
#endif
    }
    g_lqr_record_data.header=h;g_lqr_record.count=g_lqr_record.reason=0U;
    g_lqr_record.trigger_index=UINT32_MAX;g_lqr_record.ram_valid=1U;
    g_lqr_record.flash_error=0U;g_lqr_record.save_request=0U;
    start_fault=g_robot_control.last_fault.count;last_tick=state->tick_ms-LQR_RECORD_PERIOD_MS;
    fall_tracking=triggered=trigger_reason=off_tracking=tail_stopped=0U;g_lqr_record.state=LQR_RECORD_RUNNING;
    parking_goal_valid=0U;parking_goal=0;
}
static void freeze(uint32_t reason)
{
    g_lqr_record.reason=reason;g_lqr_record_data.header.reason=reason;
    g_lqr_record_data.header.count=g_lqr_record.count;
    g_lqr_record_data.header.trigger_index=g_lqr_record.trigger_index;
    g_lqr_record.state=LQR_RECORD_FROZEN;
    if(g_lqr_record.auto_save) g_lqr_record.save_request=1U;
}
void LqrStartupRecord_Capture(const WheelLegDebug *state)
{
    if(!state || g_lqr_record.state!=LQR_RECORD_RUNNING) return;
    unsigned active=g_robot_control.enable!=0U;
    /* A fresh launch during the observation tail belongs to another run.
     * Keep the first run and its stopped rows, excluding the new launch even
     * between logging ticks. Only the recorder freezes; control is untouched.
     * A fall trigger may still be active until STOP, so wait until an actual
     * inactive frame has been observed before treating activation as restart. */
    if(triggered && tail_stopped && active) { freeze(trigger_reason);return; }
    if(!active) tail_stopped=1U;
    if (active && g_robot_control.status==1U && g_dual_hold.status==1U)
        g_lqr_record_data.header.recorded_startup_ok=1U;
    unsigned fault=g_robot_control.last_fault.count!=start_fault;
    /* Include the FIRST stop/fault frame immediately. Continuing to treat
     * every stopped frame as urgent would consume the tail at task rate. */
    unsigned urgent=!triggered && (!active || fault);
    uint32_t sample_period=triggered ? LQR_RECORD_TAIL_PERIOD_MS : LQR_RECORD_PERIOD_MS;
    if(!urgent && (uint32_t)(state->tick_ms-last_tick)<sample_period) return;
    if(g_lqr_record.count>=LQR_RECORD_ROWS) { freeze(LQR_RECORD_FULL);return; }
    last_tick=state->tick_ms;
    LqrRecordRow row={0};
    row.tick_ms=state->tick_ms;
    row.flags=(active?1U:0U) | (g_wheel_output.saturated?2U:0U) |
        (g_wheel_output.torque_saturated?4U:0U) |
        (BALANCE_LEG_LQR_TORQUE_LIMIT_NM>0 && fabsf(g_balance.torque_nm[1])>=BALANCE_LEG_LQR_TORQUE_LIMIT_NM?8U:0U) |
        (g_balance.position_locked?16U:0U) | (g_joint_output.saturated?32U:0U) |
        ((active && g_dual_hold.sync_priority_active)?64U:0U) |
        ((active && g_balance.parking_damping_active)?128U:0U) |
        ((active && g_balance.parking_lean_learning)?65536U:0U) |
        ((active && g_balance.motion_damping_active)?(1U<<23):0U) |
        ((active && g_balance.motion_lean_learning)?(1U<<28):0U) |
        ((active && g_balance.motion_stall_hold)?(1U<<29):0U) |
        ((uint32_t)g_robot_control.startup_phase<<8);
    row.validity=(state->imu_valid?1U:0U) | (state->leg_valid[0]?2U:0U) |
        (state->leg_valid[1]?4U:0U) | (state->wheel_valid[0]?8U:0U) |
        (state->wheel_valid[1]?16U:0U) | (state->state_valid[0]?32U:0U) |
        (state->state_valid[1]?64U:0U);
    row.control_status=g_robot_control.status;row.fault_reason=g_robot_control.last_fault.reason;
    row.fault_count=g_robot_control.last_fault.count;
    for(unsigned i=0;i<6;++i) {
        row.state[i]=(state->state_valid[0] && state->state_valid[1]) ?
            .5f*(state->state[0][i]+state->state[1][i]):NAN;
        row.reference[i]=g_balance.reference[i];
        row.wheel_component_nm[i]=g_balance.wheel_torque_component_nm[i];
    }
    /* The controller's reference[2] may be shifted to bound its input.
     * Preserve the physical parking goal instead, including after stop
     * clears the controller's lock/reference. Never feed this back to control. */
    if(active) {
        parking_goal_valid=g_balance.position_locked;
        if(parking_goal_valid) parking_goal=g_balance.parking_target_m;
    }
    if(parking_goal_valid) row.reference[2]=parking_goal;
    row.wheel_requested_nm=g_wheel_output.requested_torque_nm;
    row.wheel_command_nm=g_wheel_output.applied_torque_nm;row.leg_tp_nm=g_balance.torque_nm[1];
    for(unsigned i=0;i<2;++i) {
        row.leg_length_m[i]=state->leg_valid[i]?state->leg[i].length:NAN;
        row.leg_angle_rad[i]=state->leg_valid[i]?state->leg[i].angle:NAN;
        row.wheel_speed_rad_s[i]=state->wheel_valid[i]?state->wheel_speed_rad_s[i]:NAN;
        row.support_n[i]=g_dual_hold.force_n[i];row.wheel_voltage_mv[i]=g_wheel_output.voltage_mv[i];
    }
    for(unsigned i=0;i<4;++i) row.joint_voltage_mv[i]=g_joint_output.voltage_mv[i];
    for(unsigned i=0;i<4;++i) {
        if(state->joint_feedback_valid[i]) row.validity|=1U<<(16U+i);
        row.joint_angle_rad[i]=state->joint_feedback_valid[i]?state->joint_angle_rad[i]:NAN;
        row.joint_shaft_speed_rad_s[i]=state->joint_feedback_valid[i]?state->joint_shaft_speed_rad_s[i]:NAN;
        row.joint_feedback_age_ms[i]=(uint16_t)(state->joint_feedback_age_ms[i]>65535U?
                                              65535U:state->joint_feedback_age_ms[i]);
        row.joint_torque_command_nm[i]=g_dual_hold.joint_torque_nm[i];
    }
    for(unsigned i=0;i<2;++i) {
        row.length_speed_m_s[i]=state->leg_valid[i]?state->length_speed_m_s[i]:NAN;
        row.angle_speed_rad_s[i]=state->leg_valid[i]?state->angle_speed_rad_s[i]:NAN;
        if(state->length_speed_position_valid[i]) row.validity|=1U<<(20U+i);
        row.length_position_speed_m_s[i]=state->length_speed_position_valid[i]?
                                        state->length_speed_from_position_m_s[i]:NAN;
        row.angle_pd_nm[i]=g_dual_hold.angle_hold_torque_nm[i];
        row.sync_nm[i]=g_dual_hold.sync_torque_nm[i];
        row.lqr_pitch_nm[i]=g_dual_hold.lqr_pitch_torque_nm[i];
        row.total_pitch_nm[i]=g_dual_hold.pitch_torque_nm[i];
        row.angle_reference_rad[i]=g_dual_hold.angle_reference_rad[i];
    }
    row.support_feedforward_n=g_dual_hold.feedforward_n;row.control_period_ms=state->period_ms;
    row.forward_target_m_s=g_robot_motion.command.forward_m_s;
    row.yaw_target_rad_s=g_robot_motion.command.turn_rad_s;
    row.yaw_rate_rad_s=state->imu_valid ? g_robot_motion.yaw_feedback_direction*state->gyro_rad_s[2] : NAN;
    row.differential_applied_nm=g_wheel_output.applied_differential_nm;
    row.flags|=(g_robot_motion.command.status==ROBOT_MOTION_TARGET_READY ? 1U<<17 : 0U) |
        (g_robot_motion.translating ? 1U<<18 : 0U) | (g_robot_motion.yaw_active ? 1U<<19 : 0U) |
        (g_robot_motion.neutral_ready ? 1U<<20 : 0U) | (g_robot_motion.yaw_enable ? 1U<<21 : 0U) |
        (g_robot_motion.yaw_feedback_direction<0 ? 1U<<22 : 0U) |
        (fabsf(g_robot_motion.yaw_integral_nm)>1e-8f ? 1U<<30 : 0U) |
        (g_robot_motion.yaw_integral_blocked ? 1U<<31 : 0U) |
        ((uint32_t)g_robot_motion.command.status<<24);
    for(unsigned i=0;i<2;++i) {
        row.wheel_requested_per_side_nm[i]=g_wheel_output.requested_per_side_nm[i];
        row.wheel_applied_per_side_nm[i]=g_wheel_output.applied_per_side_nm[i];
    }
    row.body_pitch_rad=state->imu_valid?state->body_pitch_rad:NAN;
    row.body_rate_rad_s=state->imu_valid?state->body_pitch_rate_rad_s:NAN;
    for (unsigned i=0;i<2;++i) {
        row.support_requested_n[i]=g_dual_hold.force_requested_n[i];
        row.support_limit_n[i]=g_dual_hold.force_limit_n[i];
        row.support_saturated[i]=g_dual_hold.force_saturated[i];
    }
    row.feedback_detail=state->feedback_detail;row.feedback_motor_id=state->feedback_motor_id;row.feedback_ms=state->feedback_ms;
    for (unsigned i=0;i<6;++i) row.foc_actual_iq_a[i]=NAN;
#if MYLEGWHEEL_A1_CURRENT
    const unsigned node_ids[6]={1,2,3,5,6,7};
    taskENTER_CRITICAL();
    uint32_t sample_now=HAL_GetTick();
    for (unsigned i=0;i<6;++i) {
        unsigned slot=node_ids[i]-1U;const volatile MotorFocNode *n=&g_motor_foc.node[slot];
        uint32_t age=sample_now-n->fast_ms;
        row.foc_requested_iq_a[i]=n->requested_iq_a;row.foc_applied_iq_a[i]=n->applied_iq_a;
        row.foc_current_limit_a[i]=g_motor_foc.calibration[slot].current_limit_a;
        row.foc_feedback_age_ms[i]=(uint16_t)(age>65535U?65535U:age);
        row.foc_flags[i]=(uint16_t)n->fast_flags;row.foc_mode[i]=(uint16_t)n->mode;
        row.foc_faults[i]=n->faults|n->board_faults;
        if (n->fast_valid && n->fast_seen && n->epoch==g_motor_foc.transport_epoch && age<=MOTOR_FOC_FAST_TIMEOUT_MS)
            row.foc_actual_iq_a[i]=n->fast_iq_a;
    }
    taskEXIT_CRITICAL();
#endif
    uint32_t index=g_lqr_record.count;
    g_lqr_record_data.rows[index]=row;g_lqr_record.count=index+1U;
    g_lqr_record_data.header.count=index+1U;g_lqr_record_data.header.end_tick_ms=state->tick_ms;
    /* 连续超阈值才触发跌倒记录；此处只决定日志尾段，电机停机由控制层负责。 */
    if(!triggered && isfinite(row.body_pitch_rad) && isfinite(g_lqr_record.fall_pitch_deg) &&
       g_lqr_record.fall_pitch_deg>=10.0f && g_lqr_record.fall_pitch_deg<=80.0f &&
       fabsf(row.body_pitch_rad)*57.295779513f>=g_lqr_record.fall_pitch_deg) {
        if(!fall_tracking) { fall_since=state->tick_ms;fall_tracking=1U; }
        if((uint32_t)(state->tick_ms-fall_since)>=g_lqr_record.fall_confirm_ms) {
            triggered=1U;trigger_tick=state->tick_ms;trigger_reason=LQR_RECORD_FALL;
            g_lqr_record.trigger_index=index;
        }
    } else if(!triggered) fall_tracking=0U;
    if(!triggered && (fault || !active)) {
        triggered=1U;trigger_tick=state->tick_ms;
        trigger_reason=fault?LQR_RECORD_FAULT:LQR_RECORD_STOP;
        g_lqr_record.trigger_index=index;
    }
    /* Logging continues independently of motor ownership. Observer fields
     * retain the physical motion while actuator command fields become zero.
     * Poll cannot save until this tail freezes AND all outputs/CH5 are OFF. */
    if(triggered && (uint32_t)(state->tick_ms-trigger_tick)>=g_lqr_record.tail_ms)
        freeze(trigger_reason);
    else if(g_lqr_record.count==LQR_RECORD_ROWS) freeze(LQR_RECORD_FULL);
}
void LqrStartupRecord_Poll(void)
{
    if(!initialized) return;
    uint32_t now=HAL_GetTick();
    if(!outputs_off()) { off_tracking=0U;return; }
    if(!off_tracking) { off_since=now;off_tracking=1U;return; }
    if((uint32_t)(now-off_since)<100U) return; /* Allow repeated CAN zero frames first. */
    if(g_lqr_record.rearm_request && !g_lqr_record.storage_busy &&
       g_lqr_record.state!=LQR_RECORD_RUNNING) {
        g_lqr_record.rearm_request=g_lqr_record.save_request=0U;
        g_lqr_record.count=g_lqr_record.reason=0U;g_lqr_record.ram_valid=0U;
        g_lqr_record.state=LQR_RECORD_READY;return;
    }
    if((g_lqr_record.state!=LQR_RECORD_FROZEN && g_lqr_record.state!=LQR_RECORD_SAVE_FAILED) || !g_lqr_record.save_request ||
       !g_lqr_record.ram_valid) return;
    g_lqr_record.storage_busy=1U;g_lqr_record.save_request=0U;
    /* CRC 覆盖头部前段、配置和有效行；crc32/commit 自身不计入。 */
    uint32_t crc=LqrStartupRecord_CRC32(UINT32_MAX,(const void *)&g_lqr_record_data.header,
                                      offsetof(LqrRecordHeader,crc32));
    crc=LqrStartupRecord_CRC32(crc,(const void *)&g_lqr_record_data.configuration,
                              sizeof(LqrRecordConfiguration));
    crc=LqrStartupRecord_CRC32(crc,(const void *)g_lqr_record_data.rows,
                              g_lqr_record.count*sizeof(LqrRecordRow));
    g_lqr_record_data.header.crc32=crc^UINT32_MAX;
    g_lqr_record_data.header.commit=LQR_RECORD_COMMIT;
    uint32_t error=0;
    if(LqrRecordStorage_Save(&g_lqr_record_data,&error)) {
        g_lqr_record.flash_present=1U;g_lqr_record.state=LQR_RECORD_SAVED;
        ++g_lqr_record.save_count;
    } else {
        g_lqr_record.state=LQR_RECORD_SAVE_FAILED;g_lqr_record.flash_error=error;
        g_lqr_record.flash_present=0U;
    }
    g_lqr_record.storage_busy=0U;
}
