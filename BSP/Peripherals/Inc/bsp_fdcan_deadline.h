#ifndef BSP_FDCAN_DEADLINE_H
#define BSP_FDCAN_DEADLINE_H
#include "bsp_fdcan.h"

/* 允许仲裁重发，但待发目标寿命最多 15 ms；TIM1 HAL 时基独立于控制任务执行取消。
 * ISR 只标记超时，epoch 更新及会话撤销留到任务处理。 */
#define FDCAN_TX_DEADLINE_MS 15U
typedef struct {
    uint32_t tracked_mask, expired, expired_count, aborted_mask;
    uint32_t submitted_count, invalid_buffer_count;
} FdcanDeadlineDebug;
extern volatile FdcanDeadlineDebug g_fdcan_deadline;
void FdcanDeadline_Init(void);
HAL_StatusTypeDef Fdcan_SendBounded(const FDCAN_TxHeaderTypeDef *header,const uint8_t data[8]);
void FdcanDeadline_TickFromISR(uint32_t now_ms);
uint32_t FdcanDeadline_TakeExpired(void);
void FdcanDeadline_AbortAll(void);
#endif
