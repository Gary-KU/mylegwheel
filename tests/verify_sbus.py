"""Run the actual SBUS decoder and DMA receiver with host HAL/RTOS mocks."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
runtime = r'''
#ifndef TEST_RUNTIME_H
#define TEST_RUNTIME_H
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *QueueHandle_t;
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmarx; uint32_t ErrorCode; } UART_HandleTypeDef;
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t CCR; } TestSCB;
extern TestSCB test_scb;
extern unsigned critical_depth;
#define SCB (&test_scb)
#define SCB_CCR_DC_Msk 1U
#define HAL_OK 0
#define HAL_ERROR 1
#define pdPASS 1
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (assert(critical_depth > 0), --critical_depth)
#define portYIELD_FROM_ISR(woken) ((void)(woken))
QueueHandle_t xQueueCreate(unsigned depth, unsigned size);
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken);
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t timeout);
BaseType_t xQueuePeek(QueueHandle_t queue, void *data, TickType_t timeout);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *data);
BaseType_t xQueueReset(QueueHandle_t queue);
BaseType_t xTaskCreate(void (*task)(void *), const char *name, unsigned stack,
                       void *argument, unsigned priority, void *handle);
void vTaskDelay(TickType_t timeout);
void Error_Handler(void);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size);
void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size);
void SCB_CleanInvalidateDCache_by_Addr(uint32_t *address, int32_t size);
#endif
'''

harness = r'''
#include "test_runtime.h"
#include "sbus_receiver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
TestSCB test_scb;
unsigned critical_depth;
static uint32_t now_ms;
static unsigned abort_calls, start_calls, invalidate_calls, clean_calls, bsp_calls;
static int start_fail, abort_fail, task_mode, task_waits;
static uint8_t hardware_receiving;
static TickType_t last_delay;
static jmp_buf task_escape;
static DMA_HandleTypeDef test_dma;
UART_HandleTypeDef huart5 = {&test_dma, 0};
static void (*created_task)(void *);
typedef struct {
    unsigned depth, size, head, count;
    uint8_t data[2048];
} TestQueue;

QueueHandle_t xQueueCreate(unsigned depth, unsigned size) {
    TestQueue *q = calloc(1, sizeof(*q));
    assert(q && depth * size <= sizeof(q->data));
    q->depth = depth; q->size = size; return q;
}
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *woken) {
    TestQueue *q = queue;
    if (q->count == q->depth) return pdFALSE;
    memcpy(q->data + ((q->head + q->count) % q->depth) * q->size, data, q->size);
    ++q->count; *woken = pdTRUE; return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *data, TickType_t timeout) {
    TestQueue *q = queue;
    if (q->count == 0) {
        if (task_mode) {
            assert(timeout == 20U);
            now_ms += timeout;
            if (++task_waits == 2) longjmp(task_escape, 1);
        }
        return pdFALSE;
    }
    memcpy(data, q->data + q->head * q->size, q->size);
    q->head = (q->head + 1) % q->depth; --q->count; return pdPASS;
}
BaseType_t xQueuePeek(QueueHandle_t queue, void *data, TickType_t timeout) {
    TestQueue *q = queue; assert(timeout == 0);
    if (q->count == 0) return pdFALSE;
    memcpy(data, q->data + q->head * q->size, q->size); return pdPASS;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *data) {
    TestQueue *q = queue; assert(q->depth == 1);
    memcpy(q->data, data, q->size); q->head = 0; q->count = 1; return pdPASS;
}
BaseType_t xQueueReset(QueueHandle_t queue) {
    TestQueue *q = queue; q->count = 0; q->head = 0; return pdPASS;
}
BaseType_t xTaskCreate(void (*task)(void *), const char *name, unsigned stack,
                       void *argument, unsigned priority, void *handle) {
    assert(!strcmp(name, "sbus_rx") && stack == 384 && priority == 3);
    assert(argument == NULL && handle == NULL); created_task = task; return pdPASS;
}
void vTaskDelay(TickType_t timeout) {
    last_delay = timeout;
    if (task_mode) longjmp(task_escape, 1);
}
void Error_Handler(void) { assert(!"Unexpected Error_Handler"); abort(); }
void BSP_SBUS_Init(void) { ++bsp_calls; }
uint8_t BSP_SBUS_IsReceiving(void) { return hardware_receiving; }
uint32_t HAL_GetTick(void) { return now_ms; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) {
    assert(uart == &huart5 && critical_depth == 0); ++abort_calls;
    if (!abort_fail) hardware_receiving = 0;
    return abort_fail ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size) {
    assert(uart == &huart5 && size == 64 && ((uintptr_t)buffer % 32) == 0);
    assert(critical_depth > 0); /* HAL setup must not be preempted by UART/DMA IRQ. */
    ++start_calls; test_dma.remaining = size; hardware_receiving = !start_fail;
    return start_fail ? HAL_ERROR : HAL_OK;
}
void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size) {
    assert((uintptr_t)address % 32 == 0 && size == 64); ++invalidate_calls;
}
void SCB_CleanInvalidateDCache_by_Addr(uint32_t *address, int32_t size) {
    assert((uintptr_t)address % 32 == 0 && size == 64); ++clean_calls;
}

#include "RECEIVER_SOURCE"

static unsigned hardware_position;
static uint32_t seed = 0x7826a4e3U;
static uint32_t random_value(void) {
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed;
}
/* Independent reference packing, one bit at a time (production uses words). */
static void pack_frame(const SbusFrame *frame, uint8_t footer, uint8_t *bytes) {
    unsigned ch, bit; memset(bytes, 0, 25); bytes[0] = 0x0f;
    for (ch = 0; ch < 16; ++ch) for (bit = 0; bit < 11; ++bit) {
        unsigned index = ch * 11 + bit;
        if (frame->channel[ch] & (1U << bit)) bytes[1 + index / 8] |= 1U << (index % 8);
    }
    bytes[23] = frame->channel17 | (frame->channel18 << 1) |
        (frame->frame_lost << 2) | (frame->failsafe << 3);
    bytes[24] = footer;
}
static SbusFrame frame_values(unsigned base) {
    SbusFrame frame = {0}; unsigned i;
    for (i = 0; i < 16; ++i) frame.channel[i] = (uint16_t)((base + i * 119U) & 2047U);
    return frame;
}
static void assert_frame(const SbusFrame *actual, const SbusFrame *expected) {
    assert(!memcmp(actual->channel, expected->channel, sizeof(actual->channel)));
    assert(actual->channel17 == expected->channel17 && actual->channel18 == expected->channel18);
    assert(actual->frame_lost == expected->frame_lost && actual->failsafe == expected->failsafe);
}
static unsigned feed_bytes(SbusDecoder *d, const uint8_t *bytes, unsigned length, SbusFrame *frame) {
    unsigned count = 0, i;
    for (i = 0; i < length; ++i) count += SBUS_DecoderFeed(d, bytes[i], frame);
    return count;
}
static void test_decoder(void) {
    static const uint8_t footers[] = {0, 4, 0x14, 0x24, 0x34};
    SbusDecoder d; SbusFrame input, output; uint8_t bytes[25]; unsigned n, i;
    SBUS_DecoderInit(&d);
    for (n = 0; n < 5000; ++n) {
        input = frame_values(0);
        for (i = 0; i < 16; ++i) input.channel[i] = random_value() & 2047U;
        if (n == 0) for (i = 0; i < 16; ++i) input.channel[i] = i % 2 ? 2047 : 0;
        input.channel17 = n & 1; input.channel18 = (n >> 1) & 1;
        input.frame_lost = (n >> 2) & 1; input.failsafe = (n >> 3) & 1;
        pack_frame(&input, footers[n % 5], bytes);
        assert(feed_bytes(&d, bytes, 7, &output) == 0);
        assert(feed_bytes(&d, bytes + 7, 18, &output) == 1);
        assert_frame(&output, &input);
    }
    input = frame_values(257); pack_frame(&input, 0, bytes);
    for (i = 0; i < 256; ++i) {
        if (i == 0 || i == 4 || i == 0x14 || i == 0x24 || i == 0x34) continue;
        SBUS_DecoderInit(&d); bytes[24] = (uint8_t)i;
        assert(feed_bytes(&d, bytes, 25, &output) == 0 && d.bad_frames == 1);
    }
    SBUS_DecoderInit(&d); bytes[24] = 0; bytes[23] = 0x10;
    assert(feed_bytes(&d, bytes, 25, &output) == 0 && d.bad_frames == 1);
    SBUS_DecoderInit(&d); bytes[23] = 0;
    for (i = 0; i < 101; ++i) assert(!SBUS_DecoderFeed(&d, 0x66, &output));
    /* Truncated packet and a spurious header must resynchronize. */
    assert(feed_bytes(&d, bytes, 9, &output) == 0);
    feed_bytes(&d, bytes, 25, &output);
    assert(feed_bytes(&d, bytes, 25, &output) == 1);
    assert_frame(&output, &input);
    assert(d.bad_frames > 0);
}
static void write_dma(const uint8_t *bytes, unsigned length, int boundaries) {
    unsigned i;
    for (i = 0; i < length; ++i) {
        sbus_dma_buffer[hardware_position] = bytes[i];
        hardware_position = (hardware_position + 1) % SBUS_DMA_BYTES;
        test_dma.remaining = hardware_position ? SBUS_DMA_BYTES - hardware_position : 0;
        if (boundaries && (hardware_position == 0 || hardware_position == 32)) {
            HAL_UARTEx_RxEventCallback(&huart5, hardware_position ? 32 : 64);
        }
    }
}
static void idle_event(void) { HAL_UARTEx_RxEventCallback(&huart5, (uint16_t)hardware_position); }
static void pump(void) {
    SbusRxChunk chunk;
    while (xQueueReceive(rx_queue, &chunk, 0) == pdPASS) SBUS_ProcessChunk(&chunk);
}
static void recover(void) {
    SBUS_Recover(); hardware_position = 0;
    assert(rx_active && !recovery_pending && ((TestQueue *)rx_queue)->count == 0);
}
static void send_frame(const SbusFrame *frame) {
    uint8_t bytes[25]; pack_frame(frame, 0, bytes);
    write_dma(bytes, 25, 1); idle_event(); pump();
}
static void expect_state(const SbusFrame *frame, int valid) {
    SbusState state; assert(SBUS_GetState(&state) == valid && state.link_ok == valid);
    assert_frame(&state.frame, frame);
}
static void test_receiver(void) {
    SbusState state; SbusDiagnostics diag; SbusFrame a = frame_values(172), b = frame_values(992);
    uint8_t bytes[25]; unsigned count, starts;
    UART_HandleTypeDef other_uart = {NULL, 0};
    assert(!SBUS_GetState(NULL)); assert(!SBUS_GetState(&state) && state.frame.failsafe);
    SBUS_Init(); SBUS_Task_Create(); assert(bsp_calls == 1 && created_task);
    assert(!SBUS_GetState(&state) && state.frame.failsafe);
    recover(); assert(abort_calls == 1 && start_calls == 1);
    now_ms = 10; send_frame(&a); expect_state(&a, 1);
    SBUS_DebugUpdate();
    assert_frame((const SbusFrame *)&g_sbus_debug.state.frame, &a);
    assert(g_sbus_debug.state.link_ok && g_sbus_debug.state.received_ms == 10);
    assert(g_sbus_debug.rx_active && !g_sbus_debug.recovery_pending);
    assert(g_sbus_debug.rx_bytes == 25 && g_sbus_debug.rx_events == 1);
    assert(g_sbus_debug.dma_write_position == 25 && g_sbus_debug.last_event_ms == 10);
    assert(g_sbus_debug.diagnostics.valid_frames == 1 && g_sbus_debug.task_iterations == 1);
    assert(state_queue && invalidate_calls == 0 && clean_calls == 0);
    /* Reach HT at 32, then IDLE: IDLE must not replay the same 7 bytes. */
    now_ms = 20; pack_frame(&b, 0, bytes); write_dma(bytes, 7, 1);
    count = ((TestQueue *)rx_queue)->count; assert(count == 1); idle_event();
    assert(((TestQueue *)rx_queue)->count == count);
    write_dma(bytes + 7, 18, 1); idle_event(); pump(); expect_state(&b, 1);
    /* A delayed callback's Size must not rewind a current cursor at 50. */
    HAL_UARTEx_RxEventCallback(&huart5, 32);
    assert(((TestQueue *)rx_queue)->count == 0);
    now_ms = 30; send_frame(&a); expect_state(&a, 1); /* TC wrap at 64 */
    /* Delayed HT and IDLE together; NDTR represents all currently arrived bytes. */
    write_dma(bytes, 20, 0); HAL_UARTEx_RxEventCallback(&huart5, 32);
    count = ((TestQueue *)rx_queue)->count; idle_event();
    assert(count == 1 && ((TestQueue *)rx_queue)->count == count);
    write_dma(bytes + 20, 5, 1); idle_event(); pump(); expect_state(&b, 1);
    assert(start_calls == 1); /* No DMA restart after ordinary packet/IDLE events. */

    now_ms = 100; a.channel17 = 1; a.channel18 = 1; send_frame(&a); expect_state(&a, 1);
    a.frame_lost = 1; send_frame(&a); expect_state(&a, 0);
    a.frame_lost = 0; a.failsafe = 1; send_frame(&a); expect_state(&a, 0);
    a.failsafe = 0; send_frame(&a); expect_state(&a, 1);
    assert(SBUS_GetState(&state) && state.received_ms == 100);
    now_ms = 200; expect_state(&a, 1); now_ms = 201; expect_state(&a, 0);
    SBUS_DebugUpdate(); assert(!g_sbus_debug.state.link_ok);
    now_ms = UINT32_MAX - 25U; send_frame(&a); now_ms = 10; expect_state(&a, 1);
    now_ms = 75; expect_state(&a, 0); /* Clock wrapping and exact age boundary. */

    /* A late task must not date an old queued frame with the processing time. */
    now_ms = 1000; pack_frame(&b, 0, bytes); write_dma(bytes, 25, 1); idle_event();
    now_ms = 1101; pump(); expect_state(&b, 0);
    assert(!SBUS_GetState(&state) && state.received_ms == 1000);
    /* Disconnection between packet fragments must discard the old half-frame. */
    recover(); now_ms = 1200; write_dma(bytes, 10, 1); idle_event(); pump();
    assert(decoder.used == 10); now_ms = 1301;
    write_dma(bytes + 10, 15, 1); idle_event(); pump();
    assert(!SBUS_GetState(&state)); send_frame(&b); expect_state(&b, 1);

    HAL_UARTEx_RxEventCallback(&other_uart, 25); HAL_UART_ErrorCallback(&other_uart);
    expect_state(&b, 1);
    SBUS_DebugUpdate(); assert(g_sbus_debug.state.link_ok);
    huart5.ErrorCode = 4U;
    HAL_UART_ErrorCallback(&huart5); expect_state(&b, 0);
    assert(!g_sbus_debug.state.link_ok && !g_sbus_debug.rx_active);
    assert(g_sbus_debug.recovery_pending && g_sbus_debug.last_uart_error == 4);
    assert(recovery_pending && !rx_active);
    count = ((TestQueue *)rx_queue)->count; write_dma(bytes, 4, 0); idle_event();
    assert(((TestQueue *)rx_queue)->count == count);
    /* Even a queued packet from before the fault cannot restore link_ok. */
    { SbusRxChunk old = {0}; old.epoch = transport_epoch - 1; old.length = 25;
      memcpy(old.bytes, bytes, 25); SBUS_ProcessChunk(&old); expect_state(&b, 0); }
    test_scb.CCR = SCB_CCR_DC_Msk; recover(); assert(clean_calls == 1);
    send_frame(&b); expect_state(&b, 1); assert(invalidate_calls > 0);

    /* Receiver fault on queue overflow, then flush+restart and require a new frame. */
    for (count = 0; count < SBUS_RX_QUEUE_DEPTH + 1; ++count) {
        write_dma(bytes, 25, 1); idle_event();
    }
    expect_state(&b, 0); assert(recovery_pending && !rx_active);
    SBUS_GetDiagnostics(&diag); assert(diag.queue_overflows == 1 && diag.uart_errors == 1);
    recover(); expect_state(&b, 0); send_frame(&b); expect_state(&b, 1);
    bytes[24] = 0x7f; write_dma(bytes, 25, 1); idle_event(); pump();
    SBUS_GetDiagnostics(&diag); assert(diag.bad_frames >= 1 && diag.valid_frames >= 10);
    SBUS_GetDiagnostics(NULL);

    /* Start and abort failures keep the old state invalid, retry with backoff. */
    HAL_UART_ErrorCallback(&huart5); start_fail = 1; SBUS_Recover();
    assert(recovery_pending && !rx_active); expect_state(&b, 0);
    starts = start_calls; start_fail = 0; abort_fail = 1; SBUS_Recover();
    assert(start_calls == starts && recovery_pending && !rx_active);
    SBUS_GetDiagnostics(&diag); assert(diag.restart_failures == 2);
    abort_fail = 0; recover(); send_frame(&b); expect_state(&b, 1);

    /* Real-board failure: SW active=1, hardware DMAR/EN=0, no callback. */
    hardware_receiving = 0; assert(rx_active && !recovery_pending);
    expect_state(&b, 0); SBUS_CheckTransport();
    assert(recovery_pending && !rx_active && counters.receiver_stalls == 1);
    SBUS_DebugUpdate(); assert(!g_sbus_debug.rx_hardware_active);
    recover(); send_frame(&b); expect_state(&b, 1); SBUS_DebugUpdate();
    assert(g_sbus_debug.rx_hardware_active && g_sbus_debug.diagnostics.receiver_stalls == 1);
    assert(g_sbus_debug.last_rx_length > 0);

    /* Signed processed values preserve raw data, zero on neutral and loss. */
    {
        SbusSticks sticks;
        assert(sizeof(SbusSticks)==28); /* raw CH2 uses the old tail padding. */
        SbusAxisCalibration saved = g_sbus_axis_calibration[0];
        for (unsigned axis = 0; axis < 4; ++axis) b.channel[axis] = g_sbus_axis_calibration[axis].center;
        send_frame(&b); assert(SBUS_GetSticks(&sticks) && sticks.calibrated);
        assert(sticks.raw_ch2==b.channel[1]);
        g_sbus_debug.state.frame.channel[1]=17;g_sbus_debug.sticks.raw_ch2=18;
        assert(SBUS_GetSticks(&sticks) && sticks.raw_ch2==b.channel[1]);
        assert(g_sbus_axis_calibration[0].minimum==325 && g_sbus_axis_calibration[0].center==1011);
        assert(g_sbus_axis_calibration[1].minimum==317 && g_sbus_axis_calibration[1].center==1007);
        /* Only the two used motion axes must be swept; unused axes stay still. */
        g_sbus_debug.calibration_command=1;
        b.channel[0]=325;b.channel[1]=317;send_frame(&b);
        b.channel[0]=1706;b.channel[1]=1698;send_frame(&b);
        b.channel[0]=1011;b.channel[1]=1007;g_sbus_debug.calibration_command=2;
        for(unsigned n=0;n<101;++n){now_ms+=7;send_frame(&b);}
        assert(stick_calibration.stage==SBUS_CAL_DONE);
        send_frame(&b); assert(SBUS_GetSticks(&sticks) && sticks.calibrated);
        assert(sticks.yaw == 0 && sticks.forward == 0 && sticks.height == 0 && sticks.auxiliary == 0);
        b.channel[0] -= 40; b.channel[1] += 40;
        send_frame(&b); assert(SBUS_GetSticks(&sticks) && sticks.yaw < 0 && sticks.forward < 0);
        assert(sticks.raw_ch2==b.channel[1] && sticks.frame_count>0 && sticks.received_ms==now_ms);
        expect_state(&b, 1);
        b.failsafe = 1; send_frame(&b); assert(!SBUS_GetSticks(&sticks) && sticks.yaw == 0 && sticks.forward == 0);
        assert(sticks.raw_ch2==UINT16_MAX);
        b.failsafe = 0; send_frame(&b);
        now_ms += 101; assert(!SBUS_GetSticks(&sticks) && sticks.yaw == 0);
        assert(sticks.raw_ch2==UINT16_MAX);
        send_frame(&b); HAL_UART_ErrorCallback(&huart5);
        assert(!SBUS_GetSticks(&sticks) && !g_sbus_debug.sticks.valid && g_sbus_debug.sticks.yaw == 0);
        assert(sticks.raw_ch2==UINT16_MAX && g_sbus_debug.sticks.raw_ch2==UINT16_MAX);
        recover(); send_frame(&b);
        g_sbus_debug.calibration_command = 1; send_frame(&b);
        assert(!SBUS_GetSticks(&sticks) && stick_calibration.stage == SBUS_CAL_RANGE);
        assert(sticks.raw_ch2==UINT16_MAX);
        g_sbus_debug.calibration_command = 3; send_frame(&b); assert(SBUS_GetSticks(&sticks));
        assert(!SBUS_SetAxisCalibration(0, &saved) && !SBUS_SetAxisCalibration(5, &saved));
        for (unsigned axis = 0; axis < 4; ++axis) {
            SbusAxisCalibration measured = g_sbus_axis_calibration[axis];
            measured.minimum = 280 + axis * 10; measured.maximum = 1690 + axis * 10;
            assert(SBUS_SetAxisCalibration(axis + 1, &measured));
        }
        assert(SBUS_GetSticks(&sticks) && sticks.calibrated);
        g_sbus_axis_calibration[0].center = g_sbus_axis_calibration[0].minimum;
        assert(!SBUS_GetSticks(&sticks) && sticks.yaw == 0);
        assert(sticks.raw_ch2==UINT16_MAX);
        assert(SBUS_SetAxisCalibration(1, &saved));
        assert(!SBUS_GetSticks(NULL));
    }

    /* Run the actual infinite task up to its blocking wait / retry delay. */
    task_mode = 1; task_waits = 0;
    if (setjmp(task_escape) == 0) created_task(NULL);
    assert(task_waits == 2 && last_delay == 0);
    start_fail = 1;
    if (setjmp(task_escape) == 0) created_task(NULL);
    assert(last_delay == 20 && !rx_active && recovery_pending);
    assert(!g_sbus_debug.state.link_ok && !g_sbus_debug.rx_active);
    assert(g_sbus_debug.recovery_pending && g_sbus_debug.diagnostics.restart_failures == 3);
    task_mode = 0; start_fail = 0;
    assert(critical_depth == 0);
    free(rx_queue); free(state_queue);
}
int main(void) {
    test_decoder(); test_receiver();
    puts("PASS: 5000 packed SBUS frames, all channel bits/flags, footer checks and resync");
    puts("PASS: actual circular DMA receiver: wrap, HT/TC/IDLE duplicates, timestamps,");
    puts("      timeout/tick wrap, failsafe, UART/queue faults, cache maintenance and retry");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="sbus-test-") as temp:
    folder = Path(temp)
    (folder / "test_runtime.h").write_text(runtime, encoding="utf-8")
    for name in ("stm32h7xx_hal.h", "main.h", "FreeRTOS.h", "task.h", "queue.h"):
        (folder / name).write_text('#include "test_runtime.h"\n', encoding="utf-8")
    source = harness.replace("RECEIVER_SOURCE", (root / "App/Services/Src/sbus_receiver.c").as_posix())
    (folder / "test_sbus.c").write_text(source, encoding="utf-8")
    output = folder / "test_sbus.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(folder), "-I", str(root / "App/Services/Inc"),
                    "-I", str(root / "App/Protocol/Inc"), "-I", str(root / "BSP/Peripherals/Inc"),
                    str(folder / "test_sbus.c"), str(root / "App/Protocol/Src/sbus_decoder.c"),
                    str(root / "App/Protocol/Src/sbus_sticks.c"),
                    "-o", str(output)], check=True)
    subprocess.run([str(output)], check=True)
