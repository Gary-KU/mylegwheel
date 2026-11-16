#include "bsp_sbus.h"
#include "main.h"
#include "stm32h7xx_it.h"

UART_HandleTypeDef huart5;
static DMA_HandleTypeDef sbus_rx_dma;

uint8_t BSP_SBUS_IsReceiving(void)
{
    return huart5.RxState == HAL_UART_STATE_BUSY_RX &&
        huart5.ReceptionType == HAL_UART_RECEPTION_TOIDLE &&
        (UART5->CR3 & USART_CR3_DMAR) != 0U &&
        (DMA1_Stream1->CR & DMA_SxCR_EN) != 0U;
}

void BSP_SBUS_Init(void)
{
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 100000U;
    /* HAL counts the parity bit in WordLength: 9B + even = 8 data bits. */
    huart5.Init.WordLength = UART_WORDLENGTH_9B;
    huart5.Init.StopBits = UART_STOPBITS_2;
    huart5.Init.Parity = UART_PARITY_EVEN;
    huart5.Init.Mode = UART_MODE_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
    huart5.AdvancedInit.RxPinLevelInvert = SBUS_RX_INVERTED ?
        UART_ADVFEATURE_RXINV_ENABLE : UART_ADVFEATURE_RXINV_DISABLE;
    if (HAL_UART_Init(&huart5) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    if (uart->Instance != UART5) return;
    clock.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    clock.Usart234578ClockSelection = RCC_UART5CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* Stream0 is occupied by the WS2812 SPI1 output. */
    sbus_rx_dma.Instance = DMA1_Stream1;
    sbus_rx_dma.Init.Request = DMA_REQUEST_UART5_RX;
    sbus_rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    sbus_rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    sbus_rx_dma.Init.MemInc = DMA_MINC_ENABLE;
    sbus_rx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    sbus_rx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    sbus_rx_dma.Init.Mode = DMA_CIRCULAR;
    sbus_rx_dma.Init.Priority = DMA_PRIORITY_HIGH;
    sbus_rx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&sbus_rx_dma) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(uart, hdmarx, sbus_rx_dma);
    /* Equal preemption priority serializes both RX producers. Priority 6
     * permits FreeRTOS FromISR APIs (configured syscall threshold is 5).
     */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(UART5_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&sbus_rx_dma);
}

void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
}
