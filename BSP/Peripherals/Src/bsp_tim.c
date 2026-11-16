/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM3 CH4 PWM 初始化（IMU 加热电阻驱动）
  *          PB1 = TIM3_CH4 (AF2)
  ******************************************************************************
  */
#include "bsp_tim.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;

void MX_TIM12_Init(void)
{
  TIM_OC_InitTypeDef config = {0};
  htim12.Instance = TIM12;
  /* 64 MHz /64 = 1 MHz counter; ARR is changed for each note. */
  htim12.Init.Prescaler = 64U - 1U;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000U - 1U;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) Error_Handler();
  config.OCMode = TIM_OCMODE_PWM1;
  config.Pulse = 0U;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &config, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  HAL_TIM_MspPostInit(&htim12);
}

void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if (tim_pwmHandle->Instance == TIM12)
  {
    __HAL_RCC_TIM12_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (timHandle->Instance == TIM3)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB1 -> TIM3_CH4 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM12)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);
  }
}
