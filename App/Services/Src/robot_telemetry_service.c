#include "robot_telemetry_service.h"
#include "robot_telemetry.h"
#include "robot_telemetry_config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "imu_task.h"
#include "bsp_telemetry_uart.h"
#include <math.h>
#include <string.h>
volatile RobotTelemetryDebug g_robot_telemetry={.enable=1U,
    .period_ms=ROBOT_TELEMETRY_DEFAULT_PERIOD_MS,
    .ano_pair=ROBOT_TELEMETRY_DEFAULT_ANO_PAIR,
    .ch5_select=ROBOT_TELEMETRY_DEFAULT_CH5_SELECT,
    .route=ROBOT_TELEMETRY_DEFAULT_ROUTE,
    .uart_baudrate=ROBOT_TELEMETRY_DEFAULT_UART_BAUD};
static QueueHandle_t frame_queue;
static uint32_t sequence,last_capture,last_fault_count,started;
static uint32_t last_status_tick;
static uint32_t diagnostic_pair_select(void)
{
    if (g_robot_control.feedback_only!=1U || g_robot_telemetry.ch5_select!=1U ||
        g_robot_control.remote_enable!=1U || !g_robot_control.remote_link_ok) return UINT32_MAX;
    uint32_t raw=g_robot_control.remote_ch5_raw;
    uint32_t centers[3]={g_robot_control.remote_on_raw,g_robot_control.remote_center_raw,g_robot_control.remote_off_raw};
    uint32_t tolerance=g_robot_control.remote_tolerance_raw;
    if (raw>2047U || tolerance>100U) return UINT32_MAX;
    for (unsigned i=0;i<3;++i) {
        if (centers[i]>2047U) return UINT32_MAX;
        uint32_t delta=raw>centers[i] ? raw-centers[i] : centers[i]-raw;
        if (delta<=tolerance) return 16U+i;
    }
    return UINT32_MAX;
}
void RobotTelemetry_Init(void)
{
    if (!frame_queue) frame_queue=xQueueCreate(1,sizeof(RobotTelemetrySample));
    if (!frame_queue) Error_Handler();
}
void RobotTelemetry_Capture(const WheelLegDebug *state)
{
    if (!state || !frame_queue || !g_robot_telemetry.enable) { started=0;return; }
    uint32_t period=g_robot_telemetry.period_ms;
    if (period<10U || period>1000U) return;
    unsigned urgent=g_robot_control.last_fault.count!=last_fault_count;
    if (started && !urgent && (uint32_t)(state->tick_ms-last_capture)<period) return;
    last_capture=state->tick_ms;started=1;
    g_robot_telemetry.sample_tick_ms=state->tick_ms;
    last_fault_count=g_robot_control.last_fault.count;
    RobotTelemetrySample sample={0};
    sample.ano_pair_override=diagnostic_pair_select();
    sample.meta[0]=sequence++;sample.meta[1]=state->tick_ms;
    sample.meta[2]=(state->imu_valid ? 1U:0U) |
        (state->leg_valid[0] ? 2U:0U) | (state->leg_valid[1] ? 4U:0U) |
        (state->wheel_valid[0] ? 8U:0U) | (state->wheel_valid[1] ? 16U:0U) |
        (state->state_valid[0] ? 32U:0U) | (state->state_valid[1] ? 64U:0U);
    sample.meta[3]=g_robot_control.status;
    sample.meta[4]=(uint32_t)g_robot_control.startup_phase;
    sample.meta[5]=(uint32_t)g_robot_control.last_fault.reason;
    sample.meta[6]=last_fault_count;
    sample.meta[7]=(g_robot_control.enable ? 1U:0U) |
        (g_joint_output.saturated ? 2U:0U) | (g_wheel_output.saturated ? 4U:0U) |
        (g_balance.position_locked ? 8U:0U) | (g_dual_hold.enable ? 16U:0U) |
        (g_balance.enable ? 32U:0U) |
        ((g_robot_control.startup_ready_mask&31U)<<8);
    /* 未通过有效性检查的观测保留 NaN，避免把缺失反馈显示成真实零值。 */
    for (unsigned i=0;i<ROBOT_TELEMETRY_CHANNELS;++i) sample.value[i]=NAN;
    if (state->imu_valid) {
        sample.value[0]=state->body_pitch_rad;
        sample.value[2]=state->body_pitch_rate_rad_s;
    }
    if (g_balance.enable && g_balance.status==1U) {
        sample.value[1]=-g_balance.reference[4];
        sample.value[4]=g_balance.position_locked ? g_balance.parking_target_m : g_balance.reference[2];
    }
    if (state->state_valid[0] && state->state_valid[1]) {
        sample.value[3]=.5f*(state->state[0][2]+state->state[1][2]);
        sample.value[5]=.5f*(state->state[0][3]+state->state[1][3]);
    }
    for (unsigned side=0;side<2;++side) {
        sample.leg_angle_rate_rad_s[side]=NAN;
        if (state->leg_valid[side]) {
            sample.value[6+side]=state->leg[side].angle;
            sample.value[8+side]=state->leg[side].length;
            sample.leg_angle_rate_rad_s[side]=state->angle_speed_rad_s[side];
        }
        sample.value[10+side]=g_dual_hold.sync_torque_nm[side];
        sample.value[14+side]=g_dual_hold.force_n[side];
        sample.value[16+side]=g_wheel_output.voltage_mv[side]*.001f;
        if (g_dual_hold.enable && g_dual_hold.status==1U) {
            sample.value[22+side]=g_dual_hold.angle_reference_rad[side];
            sample.swing_command_nm[side]=g_dual_hold.pitch_torque_nm[side];
        }
    }
    sample.value[12]=g_balance.torque_nm[0];sample.value[13]=g_balance.torque_nm[1];
    sample.wheel_torque_command_nm=g_wheel_output.applied_torque_nm;
    for (unsigned i=0;i<4;++i) sample.value[18+i]=g_joint_output.voltage_mv[i]*.001f;
    /* 控制任务只覆盖最新快照，实际编码/发送由通信任务 Poll 完成，避免等待串口。 */
    if (uxQueueMessagesWaiting(frame_queue)) ++g_robot_telemetry.queue_overwrites;
    (void)xQueueOverwrite(frame_queue,&sample);
    ++g_robot_telemetry.produced;
}
__weak uint8_t RobotTelemetry_TransportSend(const uint8_t *frame, uint16_t length)
{
    uint32_t route=g_robot_telemetry.route;
    uint8_t accepted=0;
    if (route<1U || route>3U) return 0;
    if (route&1U) accepted=CDC_Transmit_HS((uint8_t *)frame,length)==USBD_OK;
    if (route&2U) accepted|=BSP_TelemetryUART_Send(frame,length,g_robot_telemetry.uart_baudrate);
    return accepted;
}
void RobotTelemetry_Poll(void)
{
    RobotTelemetrySample sample;
    uint8_t frame[ROBOT_TELEMETRY_FRAME_BYTES];
    if (!frame_queue) return;
    if (!g_robot_telemetry.enable) {
        (void)xQueueReceive(frame_queue,&sample,0);g_robot_telemetry.data_valid=0;return;
    }
    uint32_t now=HAL_GetTick();
    if (g_robot_telemetry.format==0U && (uint32_t)(now-last_status_tick)>=1000U) {
        uint32_t status=imu_status ? imu_status :
            (!g_robot_telemetry.data_valid ||
             (uint32_t)(now-g_robot_telemetry.sample_tick_ms)>100U ? 512U:0U);
        uint16_t length=RobotTelemetry_EncodeAnoStatus(frame,sizeof(frame),status);
        last_status_tick=now;
        if (RobotTelemetry_TransportSend(frame,length)) ++g_robot_telemetry.sent;
        else ++g_robot_telemetry.transport_drops;
        return;
    }
    if (xQueueReceive(frame_queue,&sample,0)!=pdPASS) return;
    uint16_t length=0;
    uint32_t pair=sample.ano_pair_override<ROBOT_TELEMETRY_ANO_PAIRS ?
        sample.ano_pair_override : g_robot_telemetry.ano_pair;
    g_robot_telemetry.active_pair=pair;
    if (g_robot_telemetry.format==0U)
        length=RobotTelemetry_EncodeAnoPair(frame,sizeof(frame),&sample,pair);
    else if (g_robot_telemetry.format==1U)
        length=RobotTelemetry_Encode(frame,sizeof(frame),&sample);
    g_robot_telemetry.data_valid=length!=0U;
    if (!length && g_robot_telemetry.format==0U && pair<ROBOT_TELEMETRY_ANO_PAIRS) {
        /* Keep the ANO plot/link alive while the selected data is unavailable.
         * Both channels are placeholders; preserve data_valid=0 and F2=512. */
        memset(&sample,0,sizeof(sample));
        length=RobotTelemetry_EncodeAnoPair(frame,sizeof(frame),&sample,pair);
    }
    if (!length) return;
    if (RobotTelemetry_TransportSend(frame,length)) ++g_robot_telemetry.sent;
    else ++g_robot_telemetry.transport_drops;
}
