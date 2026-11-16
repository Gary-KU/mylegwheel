#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct {
    uint32_t kernel_hz;
    uint32_t apb_hz;
    uint32_t clock_divider;
    uint32_t tq_hz;
    uint32_t bitrate;
    uint32_t ccfg;
    uint32_t valid;
} FdcanClockDebug;

/* HAL clock-tree frequencies plus the actual CCU divider; Keil Watch. */
extern volatile FdcanClockDebug g_fdcan_clock;

void MX_FDCAN1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FDCAN_H */
