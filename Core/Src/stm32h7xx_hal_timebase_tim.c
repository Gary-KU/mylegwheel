/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#if MYLEGWHEEL_A1_CURRENT
#include "bsp_fdcan_deadline.h"
#endif
#include "stm32h7xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim1;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM1 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock;

  uint32_t              uwPrescalerValue;
  uint32_t              pFLatency;
/*Configure the TIM1 IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, TickPriority ,0U);

  /* Enable the TIM1 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    uwTickPrio = TickPriority;
    }
  else
  {
    return HAL_ERROR;
  }

  /* Enable TIM1 clock */
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* TIMPRE=0: APB DIV1/2 -> HCLK, otherwise 2*PCLK.
     TIMPRE=1: APB DIV1/2/4 -> HCLK, otherwise 4*PCLK.
     In this board's APB2 DIV1 configuration, TIM1 is 64MHz, not 128MHz. */
  if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0U)
  {
    uwTimclock = (clkconfig.APB2CLKDivider == RCC_APB2_DIV1 ||
                  clkconfig.APB2CLKDivider == RCC_APB2_DIV2) ?
                 HAL_RCC_GetHCLKFreq() : 2U * HAL_RCC_GetPCLK2Freq();
  }
  else
  {
    uwTimclock = (clkconfig.APB2CLKDivider == RCC_APB2_DIV1 ||
                  clkconfig.APB2CLKDivider == RCC_APB2_DIV2 ||
                  clkconfig.APB2CLKDivider == RCC_APB2_DIV4) ?
                 HAL_RCC_GetHCLKFreq() : 4U * HAL_RCC_GetPCLK2Freq();
  }

  /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM1 */
  htim1.Instance = TIM1;

  /* Initialize TIMx peripheral as follow:

  + Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim1.Init.Period = (1000000U / 1000U) - 1U;
  htim1.Init.Prescaler = uwPrescalerValue;
  htim1.Init.ClockDivision = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;

  if(HAL_TIM_Base_Init(&htim1) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&htim1);
  }

  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM1 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM1 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM1 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM1 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

/**
  * @brief  Period elapsed callback in non-blocking mode (1ms HAL 时基 tick).
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
#if MYLEGWHEEL_A1_CURRENT
    FdcanDeadline_TickFromISR(HAL_GetTick());
#endif
  }
}
