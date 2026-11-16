#include "bsp_telemetry_uart.h"
#include "main.h"
#include <string.h>
volatile TelemetryUartDebug g_telemetry_uart;
static UART_HandleTypeDef telemetry_uart;
static uint8_t tx_buffer[136];
static uint32_t configured;
static uint8_t configure(uint32_t baudrate)
{
    RCC_PeriphCLKInitTypeDef clock={0};
    GPIO_InitTypeDef gpio={0};
    if (configured && HAL_UART_DeInit(&telemetry_uart)!=HAL_OK) return 0;
    configured=0;
    clock.PeriphClockSelection=RCC_PERIPHCLK_USART1;
    clock.Usart16ClockSelection=RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK) return 0;
    __HAL_RCC_USART1_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin=GPIO_PIN_10;gpio.Mode=GPIO_MODE_AF_PP;gpio.Pull=GPIO_PULLUP;
    gpio.Speed=GPIO_SPEED_FREQ_HIGH;gpio.Alternate=GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA,&gpio);
    telemetry_uart.Instance=USART1;
    telemetry_uart.Init.BaudRate=baudrate;
    telemetry_uart.Init.WordLength=UART_WORDLENGTH_8B;
    telemetry_uart.Init.StopBits=UART_STOPBITS_1;
    telemetry_uart.Init.Parity=UART_PARITY_NONE;
    telemetry_uart.Init.Mode=UART_MODE_TX;
    telemetry_uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    telemetry_uart.Init.OverSampling=UART_OVERSAMPLING_16;
    telemetry_uart.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
    telemetry_uart.Init.ClockPrescaler=UART_PRESCALER_DIV1;
    /* 开启 TX/RX 引脚交换后，USART1 的发送信号从 PA10 引出。 */
    telemetry_uart.AdvancedInit.AdvFeatureInit=UART_ADVFEATURE_SWAP_INIT;
    telemetry_uart.AdvancedInit.Swap=UART_ADVFEATURE_SWAP_ENABLE;
    /* Shared HAL_UART_MspInit currently configures UART5 only. USART1's clock
     * and GPIO are deliberately configured here without changing SBUS. */
    if (HAL_UART_Init(&telemetry_uart)!=HAL_OK ||
        HAL_UARTEx_SetTxFifoThreshold(&telemetry_uart,UART_TXFIFO_THRESHOLD_1_8)!=HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&telemetry_uart)!=HAL_OK) return 0;
    HAL_NVIC_SetPriority(USART1_IRQn,7U,0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    configured=1;g_telemetry_uart.baudrate=baudrate;
    return 1;
}
uint8_t BSP_TelemetryUART_Send(const uint8_t *bytes, uint16_t length, uint32_t baudrate)
{
    if (!bytes || !length || length>sizeof(tx_buffer) ||
        (baudrate!=9600U && baudrate!=19200U && baudrate!=38400U && baudrate!=57600U &&
         baudrate!=115200U && baudrate!=230400U && baudrate!=460800U && baudrate!=921600U)) {
        g_telemetry_uart.status=3;++g_telemetry_uart.errors;return 0;
    }
    if (configured && telemetry_uart.gState!=HAL_UART_STATE_READY) {
        g_telemetry_uart.status=2;++g_telemetry_uart.busy_drops;return 0;
    }
    if ((!configured || g_telemetry_uart.baudrate!=baudrate) && !configure(baudrate)) {
        g_telemetry_uart.status=3;++g_telemetry_uart.errors;return 0;
    }
    /* 中断发送会在函数返回后继续读取数据，先复制到静态缓冲；忙时丢帧不覆盖。 */
    memcpy(tx_buffer,bytes,length);
    if (HAL_UART_Transmit_IT(&telemetry_uart,tx_buffer,length)!=HAL_OK) {
        g_telemetry_uart.status=3;++g_telemetry_uart.errors;return 0;
    }
    g_telemetry_uart.status=1;++g_telemetry_uart.accepted;return 1;
}
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&telemetry_uart);
}
