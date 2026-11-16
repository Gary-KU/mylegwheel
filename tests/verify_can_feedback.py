"""Exercise the actual receive-only CAN task with host HAL/RTOS mocks."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
hal = (root / "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h").read_text()
device = (root / "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h723xx.h").read_text()
source = (root / "App/Tasks/Src/can_task.c").read_text()
constants = sorted(set(re.findall(r"\bFDCAN_[A-Z][A-Z0-9_]+\b", source)))
constants.extend(("FDCAN_EXTENDED_ID", "FDCAN_REMOTE_FRAME", "FDCAN_FD_CAN", "HAL_FDCAN_ERROR_NONE"))
defines = []
seen = set()
for name in constants:
    if name in seen:
        continue
    seen.add(name)
    match = re.search(r"^#define\s+" + name + r"\s+([^\r\n]+)", hal + "\n" + device, re.M)
    assert match, name
    defines.append("#define " + name + " " + match[1])
    value = match[1].split("/*")[0]
    constants.extend(re.findall(r"\bFDCAN_[A-Za-z0-9_]+\b", value))

runtime = r'''
#ifndef TEST_RUNTIME_H
#define TEST_RUNTIME_H
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t ErrorCode; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t BusOff, ErrorPassive, Warning, LastErrorCode; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
extern FDCAN_HandleTypeDef hfdcan1;
extern unsigned critical_depth;
#define HAL_OK 0
#define HAL_ERROR 1
#define pdPASS 1
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (assert(critical_depth), --critical_depth)
#define portYIELD_FROM_ISR(x) ((void)(x))
uint32_t HAL_GetTick(void);
void Error_Handler(void);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *);
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
void vTaskDelay(TickType_t);
BaseType_t xTaskCreate(void (*)(void *), const char *, unsigned, void *, unsigned, TaskHandle_t *);
#endif
'''

harness = r'''
#include "test_runtime.h"
#include "can_task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <setjmp.h>
FDCAN_HandleTypeDef hfdcan1;
unsigned critical_depth;
static uint32_t now_ms, notices, notification_mask, line_mask, queued, reads;
static unsigned started, delays, waits, fail_read;
static FDCAN_ProtocolStatusTypeDef protocol;
static FDCAN_ErrorCountersTypeDef errors;
static jmp_buf escape;
static void (*created_task)(void *);
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[64];
uint32_t HAL_GetTick(void) { return now_ms; }
void Error_Handler(void) { assert(!"unexpected fatal error"); abort(); }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    assert(h == &hfdcan1 && a == FDCAN_ACCEPT_IN_RX_FIFO0 && b == FDCAN_REJECT);
    assert(c == FDCAN_REJECT_REMOTE && d == FDCAN_REJECT_REMOTE); return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *h, uint32_t mask, uint32_t line) {
    assert(h == &hfdcan1 && line == FDCAN_INTERRUPT_LINE0); line_mask = mask; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t mask, uint32_t tx) {
    assert(h == &hfdcan1 && tx == 0); notification_mask = mask; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) {
    assert(h == &hfdcan1); ++started; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *p) {
    assert(h == &hfdcan1); *p = protocol; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h, FDCAN_ErrorCountersTypeDef *p) {
    assert(h == &hfdcan1); *p = errors; return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo) {
    assert(h == &hfdcan1 && fifo == FDCAN_RX_FIFO0); return queued;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo, FDCAN_RxHeaderTypeDef *p, uint8_t *data) {
    assert(h == &hfdcan1 && fifo == FDCAN_RX_FIFO0 && queued); ++reads;
    if (fail_read) return HAL_ERROR;
    --queued; *p = rx_header; memcpy(data, rx_data, sizeof(rx_data)); ++now_ms; return HAL_OK;
}
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t *wake) {
    assert(handle == (void *)1); ++notices; *wake = pdTRUE;
}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t timeout) {
    assert(clear == pdTRUE && timeout == 10 && critical_depth == 0);
    ++waits; longjmp(escape, 1);
}
void vTaskDelay(TickType_t timeout) {
    assert(timeout == 1 && critical_depth == 0); ++delays; longjmp(escape, 1);
}
BaseType_t xTaskCreate(void (*task)(void *), const char *name, unsigned stack, void *arg, unsigned priority, TaskHandle_t *handle) {
    assert(!strcmp(name, "can_rx") && stack == 384 && priority == 5 && arg == NULL);
    created_task = task; *handle = (void *)1; return pdPASS;
}
#include "CAN_SOURCE"

static FDCAN_RxHeaderTypeDef header(uint32_t id) {
    FDCAN_RxHeaderTypeDef h = {id, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8, FDCAN_CLASSIC_CAN};
    return h;
}
static void encode(int32_t angle, int32_t speed, uint8_t *data) {
    uint32_t a = (uint32_t)angle; uint16_t s = (uint16_t)speed;
    data[0] = a; data[1] = a >> 8; data[2] = a >> 16; data[3] = a >> 24;
    data[4] = s; data[5] = s >> 8; data[6] = 0xab; data[7] = 0xcd;
}
static void check_units(const MotorFeedback *f, int32_t a, int32_t s) {
    assert(f->raw_angle_mrad == a && f->raw_speed_rpm_x10 == s);
    assert(fabs(f->angle_rad - a * 0.001) <= 0.000001 * fmax(1, fabs(a * 0.001)));
    assert(fabs(f->angle_deg - a * 0.001 * 180 / 3.141592653589793) <= 0.000001 * fmax(1, fabs(a * 0.001 * 180 / 3.141592653589793)));
    assert(fabs(f->speed_rpm - s * 0.1) < 0.0004);
    assert(fabs(f->speed_rad_s - s * 0.1 * 3.141592653589793 / 30) < 0.00006);
}
int main(void) {
    uint8_t data[8]; MotorFeedback f; FDCAN_RxHeaderTypeDef h;
    uint32_t rng = 0x54831297U, previous;
    CAN_Task_Init();
    assert(line_mask == (CAN_RX_INTERRUPTS | CAN_ERROR_INTERRUPTS));
    for (unsigned i = 0; i < 8; ++i) assert(g_motor_can.motor[i].motor_id == i + 1 && !g_motor_can.motor[i].online);
    assert(!MotorCAN_GetFeedback(0, &f) && f.motor_id == 0);
    assert(!MotorCAN_GetFeedback(9, &f) && !MotorCAN_GetFeedback(1, NULL));
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(notices == 0); CAN_Task_Create();
    /* Replay measured vertical and forward poses through the actual CAN decoder.
       One-milliradian wire quantization gives <0.03 degree reference error. */
    const unsigned joint_ids[] = {1, 2, 5, 6};
    const int32_t vertical[] = {4040, 3042, 5954, 5851};
    const int32_t forward[] = {4611, 2533, 5395, 5421};
    const int32_t joint_dir[] = {-1, 1, 1, 1};
    for (unsigned j = 0; j < 4; ++j) {
        h = header(0x100 + joint_ids[j]); encode(vertical[j], 0, data);
        CAN_ProcessFeedback(&h, data, receive_epoch);
        assert(MotorCAN_GetFeedback(joint_ids[j], &f) && f.joint_calibrated);
        assert(fabsf(f.joint_angle_deg - 90.0f) < 0.03f);
        encode(forward[j], 1000, data); CAN_ProcessFeedback(&h, data, receive_epoch);
        assert(MotorCAN_GetFeedback(joint_ids[j], &f));
        assert(f.joint_angle_deg > 50.0f && f.joint_angle_deg < 70.0f);
        assert(fabsf(f.joint_speed_rad_s - joint_dir[j] * 10.4719755f) < 0.0001f);
    }
    h = header(0x103); encode(1000, 1000, data); CAN_ProcessFeedback(&h, data, receive_epoch);
    assert(MotorCAN_GetFeedback(3, &f) && !f.joint_calibrated);
    assert(f.joint_angle_rad == 0.0f && f.joint_speed_rad_s == 0.0f);
    for (unsigned i = 0; i < 5000; ++i) {
        rng = rng * 1664525U + 1013904223U;
        int32_t a = (int32_t)(rng & 0x7fffffffU) + ((rng & 0x80000000U) ? INT32_MIN : 0);
        int32_t s = (int32_t)(rng & 0x7fffU) - ((rng & 0x8000U) ? 32768 : 0);
        unsigned id = i % 8 + 1; h = header(0x100 + id); encode(a, s, data);
        ++now_ms; CAN_ProcessFeedback(&h, data, receive_epoch);
        assert(MotorCAN_GetFeedback(id, &f) && f.motor_id == id); check_units(&f, a, s);
    }
    const int32_t angles[] = {INT32_MIN, INT32_MAX, -1, 0, 1, -3142, 3142};
    const int32_t speeds[] = {-32768, 32767, -1, 0, 1, -6000, 6000};
    h = header(0x101);
    for (unsigned i = 0; i < 7; ++i) {
        encode(angles[i], speeds[i], data); CAN_ProcessFeedback(&h, data, receive_epoch);
        assert(MotorCAN_GetFeedback(1, &f)); check_units(&f, angles[i], speeds[i]);
    }
    h.DataLength = FDCAN_DLC_BYTES_6; CAN_ProcessFeedback(&h, data, receive_epoch);
    assert(g_motor_can.last_frame.data[6] == 0 && g_motor_can.last_frame.data[7] == 0);
    h.DataLength = FDCAN_DLC_BYTES_7; CAN_ProcessFeedback(&h, data, receive_epoch);
    assert(g_motor_can.last_frame.data[6] == 0xab && g_motor_can.last_frame.data[7] == 0);
    previous = g_motor_can.decoded_frames;
    h.DataLength = FDCAN_DLC_BYTES_5; CAN_ProcessFeedback(&h, data, receive_epoch);
    h.DataLength = 0xffffffffU; CAN_ProcessFeedback(&h, data, receive_epoch);
    h = header(0x101); h.IdType = FDCAN_EXTENDED_ID; CAN_ProcessFeedback(&h, data, receive_epoch);
    h = header(0x101); h.RxFrameType = FDCAN_REMOTE_FRAME; CAN_ProcessFeedback(&h, data, receive_epoch);
    h = header(0x101); h.FDFormat = FDCAN_FD_CAN; CAN_ProcessFeedback(&h, data, receive_epoch);
    h = header(0x100); CAN_ProcessFeedback(&h, data, receive_epoch);
    h = header(0x109); CAN_ProcessFeedback(&h, data, receive_epoch);
    assert(g_motor_can.decoded_frames == previous && g_motor_can.malformed_frames == 5 && g_motor_can.unknown_frames == 2);
    now_ms += 100; CAN_UpdateStatus(); assert(MotorCAN_GetFeedback(1, &f) && f.age_ms == 100);
    ++now_ms; CAN_UpdateStatus(); assert(!MotorCAN_GetFeedback(1, &f) && !g_motor_can.motor[0].online);
    now_ms = UINT32_MAX - 49; h = header(0x101); CAN_ProcessFeedback(&h, data, receive_epoch);
    now_ms = 50; CAN_UpdateStatus(); assert(MotorCAN_GetFeedback(1, &f) && f.age_ms == 100);
    now_ms = 51; assert(!MotorCAN_GetFeedback(1, &f));
    FDCAN_HandleTypeDef other = {9}; previous = receive_epoch;
    HAL_FDCAN_RxFifo0Callback(&other, CAN_RX_INTERRUPTS);
    HAL_FDCAN_ErrorStatusCallback(&other, CAN_ERROR_INTERRUPTS); HAL_FDCAN_ErrorCallback(&other);
    assert(receive_epoch == previous && notices == 0);
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO0_FULL);
    assert(g_motor_can.fifo_lost_count == 1 && g_motor_can.fifo_full_count == 1 && notices == 1);
    CAN_ProcessFeedback(&h, data, previous); assert(!MotorCAN_GetFeedback(1, &f));
    CAN_ProcessFeedback(&h, data, receive_epoch); assert(MotorCAN_GetFeedback(1, &f));
    HAL_FDCAN_ErrorStatusCallback(&hfdcan1, FDCAN_IT_BUS_OFF); assert(!MotorCAN_GetFeedback(1, &f));
    CAN_ProcessFeedback(&h, data, receive_epoch); protocol.BusOff = 1; protocol.LastErrorCode = 3;
    errors.TxErrorCnt = 255; errors.RxErrorCnt = 128; CAN_UpdateStatus();
    assert(!MotorCAN_GetFeedback(1, &f) && g_motor_can.tx_error_counter == 255 && g_motor_can.last_error_code == 3);
    protocol.BusOff = 0; protocol.LastErrorCode = 7; CAN_UpdateStatus(); assert(g_motor_can.last_error_code == 3);
    hfdcan1.ErrorCode = 123; HAL_FDCAN_ErrorCallback(&hfdcan1);
    assert(g_motor_can.last_hal_error == 123 && !MotorCAN_GetFeedback(1, &f));
    assert(hfdcan1.ErrorCode == HAL_FDCAN_ERROR_NONE);
    /* Mirror the real HAL IRQ's sticky-code callback condition: a good RX
     * after the handled error must not cause another error invalidation.
     */
    uint32_t handled_errors = g_motor_can.hal_error_count;
    CAN_ProcessFeedback(&h, data, receive_epoch);
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    if (hfdcan1.ErrorCode != HAL_FDCAN_ERROR_NONE) HAL_FDCAN_ErrorCallback(&hfdcan1);
    assert(g_motor_can.hal_error_count == handled_errors && MotorCAN_GetFeedback(1, &f));
    hfdcan1.ErrorCode = 456; HAL_FDCAN_ErrorCallback(&hfdcan1);
    assert(g_motor_can.hal_error_count == handled_errors + 1 &&
           g_motor_can.last_hal_error == 456 && !MotorCAN_GetFeedback(1, &f));
    CAN_Task_Init(); memset(&protocol, 0, sizeof(protocol)); memset(&errors, 0, sizeof(errors));
    queued = 35; rx_header = header(0x105); encode(-1234, -567, rx_data);
    if (!setjmp(escape)) created_task(NULL);
    assert(queued == 0 && reads == 35 && waits == 1 && started == 1 && g_motor_can.started);
    assert(notification_mask == line_mask && g_motor_can.motor[4].rx_count == 35 && MotorCAN_GetFeedback(5, &f));
    check_units(&f, -1234, -567);
    queued = 1; fail_read = 1;
    if (!setjmp(escape)) created_task(NULL);
    assert(g_motor_can.rx_read_failures == 1 && delays == 1 && reads == 36);
    assert(critical_depth == 0);
    puts("PASS: 5000 signed CAN frames, units/IDs/DLC validation, timeout/wrap, fault invalidation, ISR notification, FIFO task drain and failure backoff");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="can_feedback_") as directory:
    work = Path(directory)
    # Replay the recorded poses with their recorded calibration, independently
    # of later user changes to the production configuration header.
    calibration_fixture = (root / "App/Services/Inc/joint_calibration_config.h").read_text()
    fixture_values = {"JOINT_REFERENCE_DEG": "90.0f"}
    for prefix, motor_id, reference, direction in (
        ("LF", 1, "4.040360107f", "(-1)"),
        ("LR", 2, "3.041800175f", "1"),
        ("RF", 5, "5.953520279f", "1"),
        ("RR", 6, "5.851000290f", "1"),
    ):
        for suffix, value in (("CAN_ID", str(motor_id) + "U"),
                              ("ENCODER_REFERENCE_RAD", reference),
                              ("DIRECTION", direction), ("RATIO", "1.0f"),
                              ("TRIM_DEG", "0.0f")):
            fixture_values["JOINT_" + prefix + "_" + suffix] = value
    for name, value in fixture_values.items():
        calibration_fixture, replaced = re.subn(
            r"(^#define\s+" + name + r"\s+)[^\r\n]+",
            lambda match: match[1] + value, calibration_fixture, flags=re.M)
        assert replaced == 1, name
    (work / "joint_calibration_config.h").write_text(calibration_fixture)
    (work / "test_runtime.h").write_text(runtime.replace("#endif", "\n".join(defines) + "\n#endif"))
    for name in ("bsp_fdcan.h", "main.h", "FreeRTOS.h", "task.h"):
        (work / name).write_text('#include "test_runtime.h"\n')
    c_file = work / "test_can.c"
    c_file.write_text(harness.replace("CAN_SOURCE", (root / "App/Tasks/Src/can_task.c").as_posix()))
    exe = work / "test_can.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-I", str(work),
                    "-I", str(root / "App/Tasks/Inc"), "-I", str(root / "App/Services/Inc"),
                    str(c_file), str(root / "App/Services/Src/joint_calibration.c"),
                    "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

# Ensure the receive task is wired into the firmware and contains no TX API.
main = (root / "Core/Src/main.c").read_text(encoding="utf-8")
assert "CAN_Task_Init();" in main and "CAN_Task_Create();" in main
assert "HAL_FDCAN_AddMessageToTx" not in source and "HAL_FDCAN_EnableTx" not in source
irq = (root / "Core/Src/stm32h7xx_it.c").read_text(encoding="utf-8")
assert "void FDCAN1_IT0_IRQHandler(void)" in irq and "HAL_FDCAN_IRQHandler(&hfdcan1);" in irq
print("PASS: receive-only firmware startup and FDCAN1 IRQ integration")
