#ifndef BSP_SBUS_H
#define BSP_SBUS_H

#include "stm32h7xx_hal.h"

/* CtrBoard-H7 P6 includes Q1 inversion: PD2 receives normal UART polarity.
 * Use 1 only for an inverted SBUS signal connected directly to bare PD2.
 */
#ifndef SBUS_RX_INVERTED
#define SBUS_RX_INVERTED 0U
#endif

extern UART_HandleTypeDef huart5;
void BSP_SBUS_Init(void);
/* Check the actual UART request + circular DMA enable bits, not a SW flag. */
uint8_t BSP_SBUS_IsReceiving(void);

#endif /* BSP_SBUS_H */
