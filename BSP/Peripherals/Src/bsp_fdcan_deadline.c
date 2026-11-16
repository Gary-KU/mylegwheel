#include "bsp_fdcan_deadline.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can_task.h"
#include <string.h>

volatile FdcanDeadlineDebug g_fdcan_deadline;
static volatile uint32_t initialized;
static uint32_t submitted_ms[8];

void FdcanDeadline_Init(void)
{
    taskENTER_CRITICAL();
    initialized=0U;
    memset((void *)&g_fdcan_deadline,0,sizeof(g_fdcan_deadline));
    memset(submitted_ms,0,sizeof(submitted_ms));
    initialized=1U;
    taskEXIT_CRITICAL();
}

HAL_StatusTypeDef Fdcan_SendBounded(const FDCAN_TxHeaderTypeDef *header,const uint8_t data[8])
{
    HAL_StatusTypeDef result;
    taskENTER_CRITICAL();
    if(!initialized || g_fdcan_deadline.expired || CAN_CommunicationBlocked()) {
        taskEXIT_CRITICAL();return HAL_ERROR;
    }
    result=HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,header,data);
    if(result==HAL_OK) {
        uint32_t bit=HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
        if(!bit || (bit&(bit-1U)) || (bit&~0xffU)) {
            hfdcan1.Instance->TXBCR=0xffU;
            ++g_fdcan_deadline.invalid_buffer_count;
            g_fdcan_deadline.expired=1U;result=HAL_ERROR;
        } else {
            unsigned slot=0U;
            while((1U<<slot)!=bit) ++slot;
            /* 入队与登记时间戳必须处于同一临界区，防止 ISR 用旧时间戳取消复用槽中的新帧。 */
            submitted_ms[slot]=HAL_GetTick();
            g_fdcan_deadline.tracked_mask|=bit;
            ++g_fdcan_deadline.submitted_count;
        }
    }
    taskEXIT_CRITICAL();return result;
}

void FdcanDeadline_TickFromISR(uint32_t now)
{
    if(!initialized) return;
    uint32_t pending=g_fdcan_deadline.tracked_mask&hfdcan1.Instance->TXBRP;
    g_fdcan_deadline.tracked_mask=pending; /* Completed/aborted buffers are free. */
    if(pending && CAN_CommunicationBlocked()) {
        hfdcan1.Instance->TXBCR=pending;
        g_fdcan_deadline.aborted_mask=pending;
        g_fdcan_deadline.expired=1U;g_fdcan_deadline.tracked_mask=0U;
        return;
    }
    for(unsigned slot=0;slot<8;++slot) {
        if((pending&(1U<<slot)) && (uint32_t)(now-submitted_ms[slot])>=FDCAN_TX_DEADLINE_MS) {
            /* ISR 只做整数与寄存器操作；一帧超时便取消整批，避免另一侧稍后收到过期目标。 */
            hfdcan1.Instance->TXBCR=pending;
            g_fdcan_deadline.aborted_mask=pending;
            ++g_fdcan_deadline.expired_count;
            g_fdcan_deadline.expired=1U;
            g_fdcan_deadline.tracked_mask=0U;
            break;
        }
    }
}

uint32_t FdcanDeadline_TakeExpired(void)
{
    taskENTER_CRITICAL();
    uint32_t expired=g_fdcan_deadline.expired;
    g_fdcan_deadline.expired=0U;
    taskEXIT_CRITICAL();return expired;
}

void FdcanDeadline_AbortAll(void)
{
    taskENTER_CRITICAL();
    hfdcan1.Instance->TXBCR=0xffU;
    g_fdcan_deadline.tracked_mask=0U;
    taskEXIT_CRITICAL();
}
