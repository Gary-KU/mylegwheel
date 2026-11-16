#include "sbus_receiver.h"
#include "sbus_calibration_config.h"
#include "bsp_sbus.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

#define SBUS_DMA_BYTES 64U
#define SBUS_RX_QUEUE_DEPTH 8U
#define SBUS_RECOVERY_PERIOD_MS 20U

typedef struct {
    uint32_t received_ms;
    uint32_t epoch;
    uint16_t length;
    uint8_t bytes[SBUS_DMA_BYTES];
} SbusRxChunk;

typedef struct {
    SbusState state;
    uint32_t epoch;
} SbusSnapshot;

/* DMA1 cannot access DTCM. Dedicated AXI SRAM, owned cache lines, scatter
 * placement at 0x24000400; CPU only reads while DMA is running.
 */
static uint8_t sbus_dma_buffer[SBUS_DMA_BYTES]
    __attribute__((section(".sbus_dma"), aligned(32)));
static QueueHandle_t rx_queue;
static QueueHandle_t state_queue;
static SbusDecoder decoder;
static volatile uint32_t transport_epoch;
static volatile uint8_t rx_active;
static volatile uint8_t recovery_pending;
static uint16_t dma_read_position;
static volatile SbusDiagnostics counters;
static uint32_t decoder_last_rx_ms;
static uint8_t decoder_has_timestamp;
volatile SbusDebug g_sbus_debug;
/* CH1/CH2 use the measured motion preset when selected. Unused CH3/CH4
 * retain preview spans and do not enable any height/auxiliary actuator.
 */
volatile SbusAxisCalibration g_sbus_axis_calibration[SBUS_STICK_COUNT] = {
#if SBUS_USE_MEASURED_MOTION_PRESET
    {SBUS_CH1_MEASURED_MIN, SBUS_CH1_MEASURED_CENTER, SBUS_CH1_MEASURED_MAX, SBUS_STICK_DEADBAND, 1},
    {SBUS_CH2_MEASURED_MIN, SBUS_CH2_MEASURED_CENTER, SBUS_CH2_MEASURED_MAX, SBUS_STICK_DEADBAND, SBUS_CH2_FORWARD_DIRECTION},
#else
    {0U, 1008U, 2047U, SBUS_STICK_DEADBAND, 1},
    {0U, 1017U, 2047U, SBUS_STICK_DEADBAND, 1},
#endif
    {0U, 1032U, 2047U, SBUS_STICK_DEADBAND, 1},
    {0U, 998U, 2047U, SBUS_STICK_DEADBAND, 1}
};
static SbusStickCalibration stick_calibration={.required_axes=SBUS_MOTION_CALIBRATION_MASK};
static uint8_t calibrated_axes=SBUS_USE_MEASURED_MOTION_PRESET ? SBUS_MOTION_CALIBRATION_MASK : 0U;

static void SBUS_ClearStickDebug(void)
{
    g_sbus_debug.sticks.yaw = 0.0f;
    g_sbus_debug.sticks.forward = 0.0f;
    g_sbus_debug.sticks.height = 0.0f;
    g_sbus_debug.sticks.auxiliary = 0.0f;
    g_sbus_debug.sticks.valid = 0U;
    g_sbus_debug.sticks.raw_ch2 = UINT16_MAX;
}

void SBUS_Init(void)
{
    rx_queue = xQueueCreate(SBUS_RX_QUEUE_DEPTH, sizeof(SbusRxChunk));
    state_queue = xQueueCreate(1U, sizeof(SbusSnapshot));
    if (rx_queue == NULL || state_queue == NULL) Error_Handler();
    g_sbus_debug.state.frame.failsafe = 1U;
    SBUS_DecoderInit(&decoder);
    BSP_SBUS_Init();
}

static void SBUS_TransportFaultFromISR(void)
{
    rx_active = 0U;
    recovery_pending = 1U;
    /* 故障换代后，队列里尚未处理的旧字节块和旧快照也必须失效。 */
    ++transport_epoch;
    g_sbus_debug.state.link_ok = 0U;
    SBUS_ClearStickDebug();
    g_sbus_debug.rx_active = 0U;
    g_sbus_debug.recovery_pending = 1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart5) return;
    ++counters.uart_errors;
    g_sbus_debug.last_uart_error = uart->ErrorCode;
    SBUS_TransportFaultFromISR();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    SbusRxChunk chunk;
    BaseType_t higher_priority_woken = pdFALSE;
    uint16_t write_position;
    uint16_t first;
    uint16_t available;
    /* Callback Size can describe a delayed HT/TC event. Read the current
     * NDTR instead so out-of-order/duplicate IDLE/HT/TC events cannot replay
     * bytes. Keep both HT and TC enabled for continuous input.
     */
    (void)size;
    if (uart != &huart5) return;
    ++g_sbus_debug.rx_events;
    if (!rx_active) return;
    write_position = (uint16_t)(SBUS_DMA_BYTES - __HAL_DMA_GET_COUNTER(uart->hdmarx));
    if (write_position == SBUS_DMA_BYTES) write_position = 0U;
    g_sbus_debug.dma_write_position = write_position;
    available = (uint16_t)((write_position + SBUS_DMA_BYTES - dma_read_position) % SBUS_DMA_BYTES);
    if (available == 0U) return;
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_InvalidateDCache_by_Addr((uint32_t *)sbus_dma_buffer, sizeof(sbus_dma_buffer));
    }
    chunk.received_ms = HAL_GetTick();
    g_sbus_debug.last_event_ms = chunk.received_ms;
    g_sbus_debug.rx_bytes += available;
    chunk.epoch = transport_epoch;
    chunk.length = available;
    first = (uint16_t)(SBUS_DMA_BYTES - dma_read_position);
    if (first > available) first = available;
    memcpy(chunk.bytes, sbus_dma_buffer + dma_read_position, first);
    if (available > first) memcpy(chunk.bytes + first, sbus_dma_buffer, available - first);
    dma_read_position = write_position;
    if (xQueueSendFromISR(rx_queue, &chunk, &higher_priority_woken) != pdPASS) {
        ++counters.queue_overflows;
        SBUS_TransportFaultFromISR();
    }
    portYIELD_FROM_ISR(higher_priority_woken);
}

static uint8_t SBUS_StartReceive(void)
{
    uint8_t started;
    rx_active = 0U;
    /* A fault may have aborted DMA asynchronously inside HAL's IRQ handler.
     * Synchronous abort here brings UART + DMA back to a known state.
     */
    if (HAL_UART_AbortReceive(&huart5) != HAL_OK) return 0U;
    dma_read_position = 0U;
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sbus_dma_buffer, sizeof(sbus_dma_buffer));
    }
    /* HAL enables PE/EIE before DMAR. An error interrupt in that window can
     * abort the DMA, then let startup return with inconsistent state. Keep
     * UART/DMA IRQs masked until HAL setup and SW publication are complete.
     * The blocking abort above remains outside the critical section.
     */
    taskENTER_CRITICAL();
    started = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_dma_buffer, SBUS_DMA_BYTES) == HAL_OK &&
        BSP_SBUS_IsReceiving();
    rx_active = started;
    taskEXIT_CRITICAL();
    return started;
}

static void SBUS_CheckTransport(void)
{
    taskENTER_CRITICAL();
    if (rx_active && !BSP_SBUS_IsReceiving()) {
        ++counters.receiver_stalls;
        SBUS_TransportFaultFromISR();
    }
    taskEXIT_CRITICAL();
}

static void SBUS_Recover(void)
{
    taskENTER_CRITICAL();
    rx_active = 0U;
    recovery_pending = 0U;
    /* Only task context resets the queue. IRQ producers are masked here. */
    (void)xQueueReset(rx_queue);
    taskEXIT_CRITICAL();
    decoder.used = 0U;
    decoder_has_timestamp = 0U;
    if (stick_calibration.stage == SBUS_CAL_CENTER) {
        SBUS_StickCalibrationFeed(&stick_calibration, NULL, NULL, HAL_GetTick(), 0U);
    }
    if (!SBUS_StartReceive()) {
        taskENTER_CRITICAL();
        ++counters.restart_failures;
        ++transport_epoch;
        recovery_pending = 1U;
        taskEXIT_CRITICAL();
    }
}

static void SBUS_ProcessChunk(const SbusRxChunk *chunk)
{
    uint32_t index;
    SbusFrame frame;
    if (chunk->epoch != transport_epoch) return;
    g_sbus_debug.last_rx_length = chunk->length;
    memset((void *)g_sbus_debug.last_rx_data, 0, sizeof(g_sbus_debug.last_rx_data));
    memcpy((void *)g_sbus_debug.last_rx_data, chunk->bytes,
        chunk->length < sizeof(g_sbus_debug.last_rx_data) ? chunk->length : sizeof(g_sbus_debug.last_rx_data));
    /* 收包间隔过长时丢弃半帧，避免重连后的新字节与断链前残留拼接。 */
    if (decoder_has_timestamp &&
        (uint32_t)(chunk->received_ms - decoder_last_rx_ms) > SBUS_TIMEOUT_MS) {
        decoder.used = 0U;
    }
    decoder_last_rx_ms = chunk->received_ms;
    decoder_has_timestamp = 1U;
    for (index = 0U; index < chunk->length; ++index) {
        uint32_t bad_before = decoder.bad_frames;
        if (chunk->epoch != transport_epoch) return;
        if (SBUS_DecoderFeed(&decoder, chunk->bytes[index], &frame)) {
            SbusSnapshot snapshot;
            snapshot.state.frame = frame;
            snapshot.state.received_ms = chunk->received_ms;
            snapshot.state.link_ok = !frame.frame_lost && !frame.failsafe;
            snapshot.epoch = chunk->epoch;
            if ((uint32_t)(HAL_GetTick() - chunk->received_ms) <= SBUS_TIMEOUT_MS) {
                SbusAxisCalibration calibration[SBUS_STICK_COUNT];
                uint32_t command, axis;
                taskENTER_CRITICAL();
                command = g_sbus_debug.calibration_command;
                if (snapshot.epoch == transport_epoch && (command != 0U ||
                    stick_calibration.stage == SBUS_CAL_RANGE || stick_calibration.stage == SBUS_CAL_CENTER)) {
                    g_sbus_debug.calibration_command = 0U;
                    for (axis = 0U; axis < SBUS_STICK_COUNT; ++axis) calibration[axis] = g_sbus_axis_calibration[axis];
                    SBUS_StickCalibrationFeed(&stick_calibration, calibration, &frame,
                                              chunk->received_ms, command <= 3U ? (uint8_t)command : 0U);
                    if (stick_calibration.stage == SBUS_CAL_DONE && stick_calibration.calibrated) {
                        for (axis = 0U; axis < SBUS_STICK_COUNT; ++axis) g_sbus_axis_calibration[axis] = calibration[axis];
                        calibrated_axes |= SBUS_MOTION_CALIBRATION_MASK;
                    }
                }
                taskEXIT_CRITICAL();
            }
            taskENTER_CRITICAL();
            if (snapshot.epoch == transport_epoch) {
                snapshot.state.frame_count = ++counters.valid_frames;
                (void)xQueueOverwrite(state_queue, &snapshot);
            }
            taskEXIT_CRITICAL();
        }
        if (decoder.bad_frames != bad_before) {
            taskENTER_CRITICAL();
            counters.bad_frames += decoder.bad_frames - bad_before;
            taskEXIT_CRITICAL();
        }
    }
}

static void SBUS_DebugUpdate(void)
{
    SbusState state;
    SbusSticks sticks;
    (void)SBUS_GetState(&state);
    (void)SBUS_GetSticks(&sticks);
    taskENTER_CRITICAL();
    /* Recheck after taking the lock: a UART fault may have occurred between
     * SBUS_GetState() and this debug publication.
     */
    if (recovery_pending || !rx_active || !BSP_SBUS_IsReceiving()) {
        state.link_ok = 0U;
        memset(&sticks, 0, sizeof(sticks));
    }
    g_sbus_debug.state = state;
    g_sbus_debug.diagnostics = counters;
    g_sbus_debug.rx_active = rx_active;
    g_sbus_debug.recovery_pending = recovery_pending;
    g_sbus_debug.rx_hardware_active = BSP_SBUS_IsReceiving();
    g_sbus_debug.sticks = sticks;
    g_sbus_debug.stick_calibration = stick_calibration;
    ++g_sbus_debug.task_iterations;
    taskEXIT_CRITICAL();
}

static void SBUS_Task(void *argument)
{
    SbusRxChunk chunk;
    (void)argument;
    recovery_pending = 1U;
    for (;;) {
        SBUS_CheckTransport();
        if (recovery_pending) {
            SBUS_Recover();
            if (recovery_pending) {
                SBUS_DebugUpdate();
                vTaskDelay(pdMS_TO_TICKS(SBUS_RECOVERY_PERIOD_MS));
                continue;
            }
        }
        SBUS_DebugUpdate();
        if (xQueueReceive(rx_queue, &chunk, pdMS_TO_TICKS(SBUS_RECOVERY_PERIOD_MS)) == pdPASS) {
            SBUS_ProcessChunk(&chunk);
        }
    }
}

void SBUS_Task_Create(void)
{
    /* IMU priority 4, SBUS 3, USB telemetry 2, feedback 1. */
    if (xTaskCreate(SBUS_Task, "sbus_rx", 384U, NULL, 3U, NULL) != pdPASS) Error_Handler();
}

uint8_t SBUS_GetState(SbusState *state)
{
    SbusSnapshot snapshot;
    if (state == NULL) return 0U;
    taskENTER_CRITICAL();
    if (state_queue == NULL || xQueuePeek(state_queue, &snapshot, 0U) != pdPASS) {
        memset(state, 0, sizeof(*state));
        state->frame.failsafe = 1U;
    } else {
        *state = snapshot.state;
        /* 取用时重新检查硬件、代次和帧龄，历史有效帧不能维持当前解锁条件。 */
        state->link_ok = snapshot.state.link_ok && rx_active && BSP_SBUS_IsReceiving() &&
            snapshot.epoch == transport_epoch &&
            (uint32_t)(HAL_GetTick() - snapshot.state.received_ms) <= SBUS_TIMEOUT_MS;
    }
    taskEXIT_CRITICAL();
    return state->link_ok;
}

void SBUS_GetDiagnostics(SbusDiagnostics *diagnostics)
{
    if (diagnostics == NULL) return;
    taskENTER_CRITICAL();
    *diagnostics = counters;
    taskEXIT_CRITICAL();
}

uint8_t SBUS_SetAxisCalibration(uint8_t channel, const SbusAxisCalibration *calibration)
{
    if (channel < 1U || channel > SBUS_STICK_COUNT || !SBUS_AxisCalibrationValid(calibration)) return 0U;
    taskENTER_CRITICAL();
    g_sbus_axis_calibration[channel - 1U] = *calibration;
    calibrated_axes |= (uint8_t)(1U << (channel - 1U));
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t SBUS_GetSticks(SbusSticks *sticks)
{
    SbusState state;
    SbusAxisCalibration calibration[SBUS_STICK_COUNT];
    uint32_t axis;
    uint8_t calibration_busy, link_ok;
    if (sticks == NULL) return 0U;
    memset(sticks, 0, sizeof(*sticks));
    sticks->raw_ch2 = UINT16_MAX;
    taskENTER_CRITICAL();
    link_ok = SBUS_GetState(&state);
    calibration_busy = stick_calibration.stage == SBUS_CAL_RANGE ||
        stick_calibration.stage == SBUS_CAL_CENTER || stick_calibration.stage == SBUS_CAL_FAILED;
    /* 行驶只要求配置掩码中的轴完成标定；标定进行中或失败时不发布有效摇杆。 */
    sticks->calibrated = (calibrated_axes & SBUS_MOTION_CALIBRATION_MASK) == SBUS_MOTION_CALIBRATION_MASK;
    for (axis = 0U; axis < SBUS_STICK_COUNT; ++axis) calibration[axis] = g_sbus_axis_calibration[axis];
    taskEXIT_CRITICAL();
    sticks->received_ms = state.received_ms;
    sticks->frame_count = state.frame_count;
    if (!link_ok || calibration_busy) return 0U;
    for (axis = 0U; axis < SBUS_STICK_COUNT; ++axis) {
        if (!SBUS_AxisCalibrationValid(&calibration[axis])) return 0U;
    }
    sticks->yaw = SBUS_NormalizeAxis(state.frame.channel[0], &calibration[0]);
    sticks->forward = SBUS_NormalizeAxis(state.frame.channel[1], &calibration[1]);
    sticks->height = SBUS_NormalizeAxis(state.frame.channel[2], &calibration[2]);
    sticks->auxiliary = SBUS_NormalizeAxis(state.frame.channel[3], &calibration[3]);
    sticks->raw_ch2 = state.frame.channel[1];
    sticks->valid = 1U;
    return 1U;
}
