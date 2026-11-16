/**
  ******************************************************************************
  * @file    bsp_tim.h
  * @brief   TIM3 CH4 PWM（IMU 加热）接口
  ******************************************************************************
  */
#ifndef BSP_TIM_H
#define BSP_TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;

void MX_TIM3_Init(void);
void MX_TIM12_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIM_H */
