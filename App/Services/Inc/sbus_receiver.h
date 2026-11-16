#ifndef SBUS_RECEIVER_H
#define SBUS_RECEIVER_H

#include "sbus_decoder.h"
#include "sbus_sticks.h"

#define SBUS_TIMEOUT_MS 100U

typedef struct {
    SbusFrame frame;
    uint32_t received_ms; /* Timestamp captured in the DMA/IDLE ISR. */
    uint32_t frame_count;
    uint8_t link_ok;
} SbusState;

typedef struct {
    uint32_t valid_frames;
    uint32_t bad_frames;
    uint32_t uart_errors;
    uint32_t queue_overflows;
    uint32_t restart_failures;
    uint32_t receiver_stalls; /* SW active but UART/DMA stopped. */
} SbusDiagnostics;

typedef struct {
    SbusState state;
    SbusDiagnostics diagnostics;
    uint32_t task_iterations;
    uint32_t rx_events;
    uint32_t rx_bytes;
    uint32_t last_event_ms;
    uint32_t last_uart_error;
    uint16_t dma_write_position;
    uint8_t rx_active;
    uint8_t recovery_pending;
    uint8_t rx_hardware_active;
    uint16_t last_rx_length;
    uint8_t last_rx_data[32]; /* First bytes of latest task-processed DMA chunk. */
    SbusSticks sticks; /* Signed processed values; original channel[] retained. */
    SbusStickCalibration stick_calibration;
    uint32_t calibration_command; /* Watch: 1=range, 2=neutral, 3=cancel. */
} SbusDebug;

/* Keil Watch: add g_sbus_debug and expand state.frame.channel[0..15].
 * Observation only: debugger reads of this multi-field mirror are not an
 * atomic snapshot. Control tasks must use SBUS_GetState() instead.
 * State/diagnostics refresh on RX processing and each 20 ms idle timeout;
 * ISR event counters and transport-fault invalidation update immediately.
 */
extern volatile SbusDebug g_sbus_debug;
extern volatile SbusAxisCalibration g_sbus_axis_calibration[SBUS_STICK_COUNT];

void SBUS_Init(void);
void SBUS_Task_Create(void);
/* Task context only, nonblocking, coherent latest snapshot. Returns 1 only
 * if fresh, without frame_lost/failsafe or a subsequent transport fault.
 * On return 0, retained channel values MUST NOT drive motors.
 */
uint8_t SBUS_GetState(SbusState *state);
void SBUS_GetDiagnostics(SbusDiagnostics *diagnostics);
/* Task context only. Returns 0 and zero commands on loss/fault, invalid
 * calibration, or while calibrating. Check calibrated before motor control.
 */
uint8_t SBUS_GetSticks(SbusSticks *sticks);
uint8_t SBUS_SetAxisCalibration(uint8_t channel, const SbusAxisCalibration *calibration);

#endif /* SBUS_RECEIVER_H */
