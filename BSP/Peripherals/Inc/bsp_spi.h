/**
  ******************************************************************************
  * @file    bsp_spi.h
  * @brief   SPI2 (BMI088 陀螺仪) 接口
  ******************************************************************************
  */
#ifndef BSP_SPI_H
#define BSP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi2;

void MX_SPI2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
