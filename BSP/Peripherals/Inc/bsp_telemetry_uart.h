#ifndef BSP_TELEMETRY_UART_H
#define BSP_TELEMETRY_UART_H
#include <stdint.h>
typedef struct {
    uint32_t status, baudrate, accepted, busy_drops, errors;
} TelemetryUartDebug;
extern volatile TelemetryUartDebug g_telemetry_uart;
/* USART1 TX only, PA10/AF7 with TX/RX SWAP. PA10 -> module RX.
 * Lazy setup: USB-only mode does not configure PA9/PA10 or USART1.
 * Accepted data is copied to persistent storage until the TX IRQ completes. */
uint8_t BSP_TelemetryUART_Send(const uint8_t *bytes, uint16_t length, uint32_t baudrate);
#endif
