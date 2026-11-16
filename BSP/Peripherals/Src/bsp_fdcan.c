/* FDCAN1 : PD0=RX, PD1=TX (AF9), 1Mbps 经典CAN(兼容 STM32F103 bxCAN) */

#include "bsp_fdcan.h"

FDCAN_HandleTypeDef hfdcan1;
volatile FdcanClockDebug g_fdcan_clock;

void MX_FDCAN1_Init(void)
{
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;      /* 经典CAN, 不是FD */
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  /* A1 允许硬件重发以承受六节点仲裁；独立 1 ms HAL 时基限制每个待发帧的寿命。 */
#if MYLEGWHEEL_A1_CURRENT
  hfdcan1.Init.AutoRetransmission = ENABLE;
#else
  hfdcan1.Init.AutoRetransmission = DISABLE;
#endif
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

  /* 1Mbps @ 50MHz: 1 + 39 + 10 = 50 tq = 1us; sample point 80%. */
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 10;
  hfdcan1.Init.NominalTimeSeg1 = 39;
  hfdcan1.Init.NominalTimeSeg2 = 10;

  /* 经典CAN下数据相位不用, 填最小合法值 */
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;

  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 16;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* CCU 分频独立于位时序预分频；启动前按实际时钟树验算，避免错误波特率干扰总线。 */
  g_fdcan_clock.kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  g_fdcan_clock.apb_hz = HAL_RCC_GetPCLK1Freq();
  g_fdcan_clock.ccfg = FDCAN_CCU->CCFG;
  g_fdcan_clock.clock_divider = (g_fdcan_clock.ccfg & FDCANCCU_CCFG_CDIV) >> FDCANCCU_CCFG_CDIV_Pos;
  g_fdcan_clock.clock_divider = g_fdcan_clock.clock_divider == 0U ? 1U :
                               g_fdcan_clock.clock_divider * 2U;
  g_fdcan_clock.tq_hz = g_fdcan_clock.kernel_hz / g_fdcan_clock.clock_divider;
  g_fdcan_clock.bitrate = g_fdcan_clock.tq_hz / (hfdcan1.Init.NominalPrescaler *
      (1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2));
  g_fdcan_clock.valid = g_fdcan_clock.kernel_hz == 50000000U &&
      g_fdcan_clock.clock_divider == 1U &&
      (g_fdcan_clock.ccfg & FDCANCCU_CCFG_BCC) != 0U &&
      g_fdcan_clock.tq_hz <= g_fdcan_clock.apb_hz &&
      g_fdcan_clock.bitrate == 1000000U;
  if (!g_fdcan_clock.valid) Error_Handler();
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (fdcanHandle->Instance == FDCAN1)
  {
    /* HSE 24MHz -> PLL1Q 50MHz, below APB1 64MHz; CCU DIV1. */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PD0 -> FDCAN1_RX, PD1 -> FDCAN1_TX, AF9 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    /* RX/error callbacks only notify the priority-5 receive task. */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
}
