/* 反馈采集与 bus-off 通信恢复；恢复须先确认所有输出停止，随后由任务重建通信。 */
#include "can_task.h"
#include "bsp_fdcan.h"
#include "joint_calibration.h"
#include "motor_foc_config.h"
#if MYLEGWHEEL_A1_CURRENT
#include "motor_foc.h"
#include "motor_command.h"
#include "bsp_fdcan_deadline.h"
#endif
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define CAN_STATUS_PERIOD_MS 10U
#define CAN_RX_BATCH_LIMIT 16U
#define CAN_RECOVERY_QUIET_MS 200U
#define CAN_RECOVERY_RETRY_MS 1000U
#define CAN_RECOVERY_WAIT_MS 1000U
#define CAN_RX_INTERRUPTS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | \
                           FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
#define CAN_ERROR_INTERRUPTS (FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | \
                              FDCAN_IT_ERROR_WARNING | FDCAN_IT_ARB_PROTOCOL_ERROR | \
                              FDCAN_IT_DATA_PROTOCOL_ERROR)

volatile MotorCanDebug g_motor_can;
static TaskHandle_t can_rx_task_handle;
/* 丢帧或总线错误时递增；只有同一 epoch 内收到的反馈才能重新标为在线。 */
static volatile uint32_t receive_epoch;
static uint32_t motor_epoch[MOTOR_CAN_COUNT];
#if MYLEGWHEEL_A1_CURRENT
static uint32_t recovery_quiet_valid, recovery_quiet_ms, recovery_retry_ms;
static uint32_t recovery_hardware_ms, recovery_fault_notified;
#endif

uint32_t CAN_CommunicationBlocked(void)
{
    return g_motor_can.recovery_state != 0U;
}

void CAN_Task_Init(void)
{
    uint32_t i;
    memset((void *)&g_motor_can, 0, sizeof(g_motor_can));
    receive_epoch = 0U;
    memset(motor_epoch, 0, sizeof(motor_epoch));
#if MYLEGWHEEL_A1_CURRENT
    recovery_quiet_valid = recovery_quiet_ms = recovery_retry_ms = 0U;
    recovery_hardware_ms = 0U;
    recovery_fault_notified = 0U;
    MotorFoc_Init();
#endif
    for (i = 0U; i < MOTOR_CAN_COUNT; ++i) {
        g_motor_can.motor[i].motor_id = i + 1U;
    }
    /* A1 status/diagnostic/ACK/parameter/trace/FAST and legacy feedback all
     * use standard data IDs. Validate their protocol and DLC in the task. */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ConfigInterruptLines(&hfdcan1, CAN_RX_INTERRUPTS | CAN_ERROR_INTERRUPTS,
                                     FDCAN_INTERRUPT_LINE0) != HAL_OK) {
        Error_Handler();
    }
}

static void CAN_WakeFromISR(void)
{
    BaseType_t higher_priority_woken = pdFALSE;
    if (can_rx_task_handle != NULL) {
        vTaskNotifyGiveFromISR(can_rx_task_handle, &higher_priority_woken);
    }
    portYIELD_FROM_ISR(higher_priority_woken);
}

static void CAN_InvalidateFromISR(void)
{
    uint32_t i;
    ++receive_epoch;
#if MYLEGWHEEL_A1_CURRENT
    MotorFoc_InvalidateFromISR();
#endif
    for (i = 0U; i < MOTOR_CAN_COUNT; ++i) g_motor_can.motor[i].online = 0U;
}

#if MYLEGWHEEL_A1_CURRENT
static void CAN_RequestRecovery(void)
{
    if (g_motor_can.recovery_state == 0U) {
        /* Called from ISR or a task critical section. The TX gate must close
         * before the lower-priority control task can queue another target. */
        g_motor_can.recovery_state = 1U;
        g_motor_can.recovery_started_ms = HAL_GetTick();
        ++g_motor_can.recovery_count;
        recovery_quiet_valid = 0U;
        recovery_fault_notified = 0U;
        recovery_retry_ms = HAL_GetTick();
    }
}

static void CAN_RecoveryFailure(void)
{
    ++g_motor_can.recovery_failures;
    g_motor_can.last_hal_error = hfdcan1.ErrorCode;
    /* A HAL ERROR state is not repaired by forging READY. Keep TX inhibited
     * for an explicit reset; this path never repeatedly deinitializes CAN. */
    g_motor_can.recovery_state = 3U;
}

static uint32_t CAN_DiscardRx(void)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t bytes[64];
    for (uint32_t i = 0U; i < CAN_RX_BATCH_LIMIT; ++i) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) return 1U;
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, bytes) != HAL_OK) {
            ++g_motor_can.rx_read_failures;
            g_motor_can.last_hal_error = hfdcan1.ErrorCode;
            return 0U;
        }
        ++g_motor_can.recovery_discarded_frames;
    }
    return HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U;
}

/* Return nonzero while ordinary RX decoding must remain suspended. The CAN
 * task owns HAL Stop/Start; the control task only revokes motion intent. */
static uint32_t CAN_ServiceRecovery(void)
{
    uint32_t now = HAL_GetTick();
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    uint32_t state = g_motor_can.recovery_state;
    if (state == 0U) return 0U;
    if (!recovery_fault_notified) {
        recovery_fault_notified = 1U;
        MotorCommand_OnTransportFault();
    }
    FdcanDeadline_AbortAll();
    if (state == 3U) return 1U;
    /* 等待控制、维护和调试输出全部退出；通信恢复不继承故障前的使能请求。 */
    if (!MotorCommand_CommunicationStopped()) {
        recovery_quiet_valid = 0U;
        return 1U;
    }
    if (state == 1U) {
        if (!recovery_quiet_valid) {
            recovery_quiet_valid = 1U;
            recovery_quiet_ms = now;
        }
        if ((uint32_t)(now - recovery_quiet_ms) < CAN_RECOVERY_QUIET_MS ||
            (int32_t)(now - recovery_retry_ms) < 0) return 1U;
        /* TXBCR is a request, not its completion. Never release INIT with
         * an old current frame still pending in the hardware FIFO. */
        if (hfdcan1.Instance->TXBRP & 0xffU) {
            if ((uint32_t)(now - recovery_quiet_ms) >= CAN_RECOVERY_WAIT_MS) {
                ++g_motor_can.recovery_tx_cancel_timeout;
                CAN_RecoveryFailure();
            }
            return 1U;
        }
        ++g_motor_can.recovery_attempts;
        recovery_retry_ms = now + CAN_RECOVERY_RETRY_MS;
        if (HAL_FDCAN_DeactivateNotification(&hfdcan1,
                CAN_RX_INTERRUPTS | CAN_ERROR_INTERRUPTS) != HAL_OK) {
            CAN_RecoveryFailure(); return 1U;
        }
        /* In this HAL GetRxMessage requires BUSY. INIT does not empty FIFO;
         * discard while bus-off has stopped reception, before Stop=>READY. */
        if (hfdcan1.State == HAL_FDCAN_STATE_BUSY) {
            (void)CAN_DiscardRx();
            if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) {
                CAN_RecoveryFailure(); return 1U;
            }
        }
        if (hfdcan1.State != HAL_FDCAN_STATE_READY) {
            CAN_RecoveryFailure(); return 1U;
        }
        if (hfdcan1.Instance->TXBRP & 0xffU) {
            ++g_motor_can.recovery_tx_cancel_timeout;
            CAN_RecoveryFailure(); return 1U;
        }
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, CAN_RX_INTERRUPTS | CAN_ERROR_INTERRUPTS);
        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
            CAN_RecoveryFailure(); return 1U;
        }
        recovery_hardware_ms = now;
        g_motor_can.recovery_state = 2U;
        return 1U;
    }
    /* Start merely clears INIT. M_CAN then needs the hardware bus-off
     * recovery sequence (129 occurrences of 11 recessive bits). No command
     * may be queued until both hardware indications have actually cleared. */
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) != HAL_OK ||
        protocol.BusOff || (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT)) {
        if ((uint32_t)(now - recovery_hardware_ms) >= CAN_RECOVERY_WAIT_MS) {
            ++g_motor_can.recovery_failures;
            g_motor_can.recovery_state = 1U;
            recovery_quiet_valid = 0U;
        }
        return 1U;
    }
    /* Bound each drain to FIFO capacity. On a continuously busy bus defer
     * another bounded batch; never spin above IMU/control task priorities. */
    if (!CAN_DiscardRx()) {
        if ((uint32_t)(now - recovery_hardware_ms) >= CAN_RECOVERY_WAIT_MS) {
            ++g_motor_can.recovery_failures;
            g_motor_can.recovery_state = 1U;
            recovery_quiet_valid = 0U;
        }
        return 1U;
    }
    taskENTER_CRITICAL();
    __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, CAN_RX_INTERRUPTS | CAN_ERROR_INTERRUPTS);
    CAN_InvalidateFromISR();
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            CAN_RX_INTERRUPTS | CAN_ERROR_INTERRUPTS, 0U) != HAL_OK) {
        CAN_RecoveryFailure();
    } else {
        g_motor_can.bus_off = 0U;
        g_motor_can.recovery_completed_ms = now;
        g_motor_can.recovery_state = 0U;
    }
    taskEXIT_CRITICAL();
    return CAN_CommunicationBlocked();
}
#endif

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *can, uint32_t events)
{
    if (can != &hfdcan1) return;
    ++g_motor_can.rx_irq_count;
    g_motor_can.last_irq_ms = HAL_GetTick();
    if (events & FDCAN_IT_RX_FIFO0_FULL) ++g_motor_can.fifo_full_count;
    if (events & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        ++g_motor_can.fifo_lost_count;
        CAN_InvalidateFromISR();
    }
    /* 中断只记录事件并唤醒任务；浮点解码和 FIFO 读取留给接收任务。 */
    CAN_WakeFromISR();
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *can, uint32_t events)
{
    if (can != &hfdcan1) return;
    ++g_motor_can.error_irq_count;
    g_motor_can.last_error_events = events;
#if MYLEGWHEEL_A1_CURRENT
    if (events & FDCAN_IT_BUS_OFF) CAN_RequestRecovery();
#endif
    CAN_InvalidateFromISR();
    CAN_WakeFromISR();
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *can)
{
    if (can != &hfdcan1) return;
    ++g_motor_can.hal_error_count;
    g_motor_can.last_hal_error = can->ErrorCode;
    /* HAL_IRQHandler ORs faults into ErrorCode and calls this callback on
     * every subsequent IRQ while it remains nonzero, including good RX.
     * Preserve the event in diagnostics, then consume the software latch.
     * The HAL has already cleared the hardware IR flags; new faults still
     * trigger invalidation. Never suppress the fault or mark old data valid.
     */
    can->ErrorCode = HAL_FDCAN_ERROR_NONE;
    CAN_InvalidateFromISR();
    CAN_WakeFromISR();
}

static uint8_t CAN_DataLength(uint32_t dlc)
{
    /* Use HAL constants: newer H7 HAL uses unshifted DLC values, unlike
     * older releases that expose the register-positioned representation.
     */
    switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    case FDCAN_DLC_BYTES_8: return 8U;
    default: return 0xFFU;
    }
}

static void CAN_ProcessFeedback(const FDCAN_RxHeaderTypeDef *header,
                                const uint8_t *data, uint32_t epoch)
{
    MotorFeedback feedback = {0};
    uint8_t length = CAN_DataLength(header->DataLength);
    uint32_t raw_angle;
    uint32_t raw_speed;
    uint32_t slot;
    uint32_t now = HAL_GetTick();
    uint32_t i;

    ++g_motor_can.received_frames;
    g_motor_can.last_frame.identifier = header->Identifier;
    g_motor_can.last_frame.id_type = header->IdType;
    g_motor_can.last_frame.frame_type = header->RxFrameType;
    g_motor_can.last_frame.fd_format = header->FDFormat;
    g_motor_can.last_frame.length = length;
    g_motor_can.last_frame.received_ms = now;
    for (i = 0U; i < 8U; ++i) {
        g_motor_can.last_frame.data[i] = i < length && length <= 8U ? data[i] : 0U;
    }

    if (header->IdType != FDCAN_STANDARD_ID ||
        header->RxFrameType != FDCAN_DATA_FRAME ||
        header->FDFormat != FDCAN_CLASSIC_CAN || length > 8U) {
        ++g_motor_can.malformed_frames;
        return;
    }
    if (header->Identifier < 0x101U || header->Identifier > 0x108U) {
#if MYLEGWHEEL_A1_CURRENT
        taskENTER_CRITICAL();
        uint32_t accepted = length == 8U && epoch == receive_epoch &&
            !CAN_CommunicationBlocked() && MotorFoc_OnFrame(header->Identifier,data,now);
        taskEXIT_CRITICAL();
        if(accepted) {
            ++g_motor_can.decoded_frames;
            return;
        }
#endif
        ++g_motor_can.unknown_frames;
        return;
    }
    if (length < 6U) {
        ++g_motor_can.malformed_frames;
        return;
    }
    slot = header->Identifier - 0x101U;
#if MYLEGWHEEL_A1_CURRENT
    if(MOTOR_FOC_NODE_MASK&(1U<<slot)) {++g_motor_can.unknown_frames;return;}
#endif
    raw_angle = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    raw_speed = (uint32_t)data[4] | ((uint32_t)data[5] << 8);
    /* Explicit signed little-endian conversion: no unaligned pointer casts,
     * aliasing violations, or implementation-defined unsigned-to-signed cast.
     */
    feedback.raw_angle_mrad = (int32_t)(raw_angle & 0x7FFFFFFFU);
    if (raw_angle & 0x80000000U) feedback.raw_angle_mrad += INT32_MIN;
    feedback.raw_speed_rpm_x10 = (int32_t)(raw_speed & 0x7FFFU) -
                                ((raw_speed & 0x8000U) ? 32768 : 0);
    feedback.motor_id = slot + 1U;
    feedback.angle_rad = (float)feedback.raw_angle_mrad * 0.001f;
    feedback.angle_deg = feedback.angle_rad * 57.2957795131f;
    feedback.speed_rpm = (float)feedback.raw_speed_rpm_x10 * 0.1f;
    feedback.speed_rad_s = feedback.speed_rpm * 0.10471975512f;
    JointCalibration_Apply(&feedback);
    feedback.last_rx_ms = now;
    feedback.online = 1U;

    taskENTER_CRITICAL();
    /* 读取 FIFO 后若恰好发生故障，epoch 已变化，该帧不能覆盖失效标记。 */
    if (epoch == receive_epoch && !CAN_CommunicationBlocked()) {
        feedback.rx_count = g_motor_can.motor[slot].rx_count + 1U;
        g_motor_can.motor[slot] = feedback;
        motor_epoch[slot] = epoch;
        ++g_motor_can.decoded_frames;
    }
    taskEXIT_CRITICAL();
}

static void CAN_UpdateStatus(void)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef errors = {0};
    uint32_t now = HAL_GetTick();
    uint32_t i;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol) == HAL_OK) {
        g_motor_can.bus_off = protocol.BusOff;
#if MYLEGWHEEL_A1_CURRENT
        if (protocol.BusOff) {
            taskENTER_CRITICAL();
            if (!CAN_CommunicationBlocked()) { CAN_RequestRecovery(); CAN_InvalidateFromISR(); }
            taskEXIT_CRITICAL();
        }
#endif
        g_motor_can.error_passive = protocol.ErrorPassive;
        g_motor_can.error_warning = protocol.Warning;
        if (protocol.LastErrorCode != 7U) g_motor_can.last_error_code = protocol.LastErrorCode;
    }
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &errors) == HAL_OK) {
        g_motor_can.tx_error_counter = errors.TxErrorCnt;
        g_motor_can.rx_error_counter = errors.RxErrorCnt;
    }
    taskENTER_CRITICAL();
    for (i = 0U; i < MOTOR_CAN_COUNT; ++i) {
        g_motor_can.motor[i].age_ms = now - g_motor_can.motor[i].last_rx_ms;
        g_motor_can.motor[i].online = g_motor_can.motor[i].rx_count != 0U &&
            g_motor_can.motor[i].age_ms <= MOTOR_CAN_TIMEOUT_MS &&
            motor_epoch[i] == receive_epoch && !g_motor_can.bus_off && !CAN_CommunicationBlocked();
    }
    ++g_motor_can.task_iterations;
    taskEXIT_CRITICAL();
}

static void CAN_Rx_Task(void *argument)
{
    FDCAN_RxHeaderTypeDef header;
    /* HAL copies according to the DLC. A 64-byte scratch buffer also keeps
     * malformed/FD frames from overflowing a classic-CAN 8-byte buffer.
     */
    uint8_t data[64];
    uint32_t status_tick = HAL_GetTick();
    (void)argument;
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
        CAN_RX_INTERRUPTS | CAN_ERROR_INTERRUPTS, 0U) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        g_motor_can.last_hal_error = hfdcan1.ErrorCode;
        Error_Handler();
    }
    g_motor_can.started = 1U;
    for (;;) {
        uint32_t batch;
        uint8_t read_failed = 0U;
#if MYLEGWHEEL_A1_CURRENT
        if (CAN_ServiceRecovery()) {
            /* Keep telemetry/error counters current without decoding any
             * FIFO content as a fresh rotor sample during recovery. */
            CAN_UpdateStatus();
            status_tick = HAL_GetTick();
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1U));
            continue;
        }
#endif
        for (batch = 0U; batch < CAN_RX_BATCH_LIMIT; ++batch) {
            uint32_t epoch;
            if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) break;
            epoch = receive_epoch;
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
                ++g_motor_can.rx_read_failures;
                g_motor_can.last_hal_error = hfdcan1.ErrorCode;
                read_failed = 1U;
                break;
            }
            CAN_ProcessFeedback(&header, data, epoch);
        }
        if ((uint32_t)(HAL_GetTick() - status_tick) >= CAN_STATUS_PERIOD_MS ||
            g_motor_can.task_iterations == 0U) {
            CAN_UpdateStatus();
            status_tick = HAL_GetTick();
        }
        if (read_failed) {
            /* A persistent HAL failure must not spin above the IMU task. */
            vTaskDelay(pdMS_TO_TICKS(1U));
        } else if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) {
            /* Notification wakes immediately; 10 ms is the idle status/timeout
             * refresh period, not the motor feedback acquisition period.
             */
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_STATUS_PERIOD_MS));
        }
    }
}

void CAN_Task_Create(void)
{
    if (xTaskCreate(CAN_Rx_Task, "can_rx", 384U, NULL, 5U, &can_rx_task_handle) != pdPASS) {
        Error_Handler();
    }
}

uint8_t MotorCAN_GetFeedback(uint8_t motor_id, MotorFeedback *feedback)
{
    return MotorCAN_GetFeedbackWithHealth(motor_id,feedback,NULL);
}

uint8_t MotorCAN_GetFeedbackWithHealth(uint8_t motor_id, MotorFeedback *feedback,
                                     MotorFeedbackHealth *health)
{
    uint32_t slot;
    if(health) memset(health,0,sizeof(*health));
    if (feedback == NULL) return 0U;
    if (motor_id < 1U || motor_id > MOTOR_CAN_COUNT) {
        memset(feedback, 0, sizeof(*feedback));
        return 0U;
    }
    slot = motor_id - 1U;
#if MYLEGWHEEL_A1_CURRENT
    if(MOTOR_FOC_NODE_MASK&(1U<<slot)) {
        uint8_t online=MotorFoc_GetFeedbackWithHealth(motor_id,feedback,health);
        uint32_t blocked=CAN_CommunicationBlocked();
        if(health) {
            health->communication_blocked=blocked;health->recovery_state=g_motor_can.recovery_state;
            if(blocked) health->reject_mask|=MOTOR_FEEDBACK_REJECT_COMMUNICATION;
        }
        if (blocked) { feedback->online = 0U; online = 0U; }
        taskENTER_CRITICAL();g_motor_can.motor[slot]=*feedback;taskEXIT_CRITICAL();
        return online;
    }
#endif
    taskENTER_CRITICAL();
    *feedback = g_motor_can.motor[slot];
    feedback->age_ms = HAL_GetTick() - feedback->last_rx_ms;
    feedback->online = feedback->rx_count != 0U &&
        feedback->age_ms <= MOTOR_CAN_TIMEOUT_MS &&
        motor_epoch[slot] == receive_epoch && !g_motor_can.bus_off && !CAN_CommunicationBlocked();
    taskEXIT_CRITICAL();
    return feedback->online;
}
