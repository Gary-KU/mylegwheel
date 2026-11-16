#ifndef ROBOT_TELEMETRY_SERVICE_H
#define ROBOT_TELEMETRY_SERVICE_H
#include "wheel_leg_task.h"
typedef struct {
    uint32_t enable, period_ms; /* Default on, 20 ms. Allowed period: 10..1000 ms. */
    uint32_t format, ano_pair; /* 0: ANO pair (default); 1: robot24. Pair 0..21. */
    uint32_t route, uart_baudrate; /* 2 PA10 UART (default), 1 USB, 3 both. */
    uint32_t data_valid; /* Read-only: current selected pair/frame could be encoded. */
    uint32_t sample_tick_ms; /* Read-only: last coherent control snapshot. */
    uint32_t produced, sent, transport_drops, queue_overwrites;
    uint32_t ch5_select, active_pair; /* Select only in feedback-only mode; active_pair is read-only. */
} RobotTelemetryDebug;
extern volatile RobotTelemetryDebug g_robot_telemetry;
void RobotTelemetry_Init(void);
/* Single producer: call after the complete control/output cycle. No I/O waits. */
void RobotTelemetry_Capture(const WheelLegDebug *state);
/* Single consumer: call from the lower-priority communication task. */
void RobotTelemetry_Poll(void);
/* Nonblocking transport contract: copy accepted data to transport-owned storage
 * before returning 1. Return 0 on busy/disconnected/error. Default is USB CDC.
 * A future UART DMA adapter must own its DMA buffer until completion. */
uint8_t RobotTelemetry_TransportSend(const uint8_t *frame, uint16_t length);
#endif
