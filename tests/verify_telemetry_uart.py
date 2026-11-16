"""Actual PA10/SWAP UART driver with mocked HAL, including async buffer lifetime."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
mock=r'''
#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
typedef struct {uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,
 OverSampling,OneBitSampling,ClockPrescaler;} UART_InitTypeDef;
typedef struct {uint32_t AdvFeatureInit,Swap;} UART_AdvFeatureInitTypeDef;
typedef struct {void *Instance;UART_InitTypeDef Init;UART_AdvFeatureInitTypeDef AdvancedInit;
 uint32_t gState;} UART_HandleTypeDef;
typedef struct {uint32_t PeriphClockSelection,Usart16ClockSelection;} RCC_PeriphCLKInitTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
#define HAL_OK 0
#define USART1 ((void*)0x1000)
#define GPIOA ((void*)0x2000)
#define RCC_PERIPHCLK_USART1 1
#define RCC_USART16CLKSOURCE_D2PCLK2 2
#define GPIO_PIN_10 1024
#define GPIO_MODE_AF_PP 3
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_HIGH 2
#define GPIO_AF7_USART1 7
#define UART_WORDLENGTH_8B 8
#define UART_STOPBITS_1 1
#define UART_PARITY_NONE 0
#define UART_MODE_TX 8
#define UART_HWCONTROL_NONE 0
#define UART_OVERSAMPLING_16 16
#define UART_ONE_BIT_SAMPLE_DISABLE 0
#define UART_PRESCALER_DIV1 1
#define UART_ADVFEATURE_SWAP_INIT 8
#define UART_ADVFEATURE_SWAP_ENABLE 0x8000
#define UART_TXFIFO_THRESHOLD_1_8 1
#define USART1_IRQn 37
#define HAL_UART_STATE_READY 0x20
#define HAL_UART_STATE_BUSY_TX 0x21
void mock_enable_clock(unsigned);
#define __HAL_RCC_USART1_CLK_ENABLE() mock_enable_clock(1)
#define __HAL_RCC_GPIOA_CLK_ENABLE() mock_enable_clock(2)
int HAL_UART_Init(UART_HandleTypeDef*);
int HAL_UART_DeInit(UART_HandleTypeDef*);
int HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef*);
void HAL_GPIO_Init(void*,GPIO_InitTypeDef*);
int HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef*,unsigned);
int HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef*);
void HAL_NVIC_SetPriority(unsigned,unsigned,unsigned);
void HAL_NVIC_EnableIRQ(unsigned);
int HAL_UART_Transmit_IT(UART_HandleTypeDef*,uint8_t*,uint16_t);
void HAL_UART_IRQHandler(UART_HandleTypeDef*);
#endif
'''
harness=r'''
#include "main.h"
#include "bsp_telemetry_uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
void USART1_IRQHandler(void);
static UART_HandleTypeDef *handle;
static uint8_t *pending;
static unsigned length,inits,clock_mask,gpios,irq,fail_clock,fail_tx;
void mock_enable_clock(unsigned which) {clock_mask|=which;}
int HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c) {
 assert(c->PeriphClockSelection==1 && c->Usart16ClockSelection==2);return fail_clock?1:HAL_OK;
}
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *g) {
 assert(port==GPIOA && g->Pin==GPIO_PIN_10 && g->Mode==GPIO_MODE_AF_PP && g->Alternate==7 && g->Pull==GPIO_PULLUP);
 ++gpios;
}
int HAL_UART_Init(UART_HandleTypeDef *h) {
 assert(h->Instance==USART1 && h->Init.WordLength==8 && h->Init.StopBits==1 && h->Init.Parity==0 && h->Init.Mode==UART_MODE_TX);
 assert(h->AdvancedInit.AdvFeatureInit==UART_ADVFEATURE_SWAP_INIT && h->AdvancedInit.Swap==UART_ADVFEATURE_SWAP_ENABLE);
 assert(clock_mask==3);handle=h;h->gState=HAL_UART_STATE_READY;++inits;return HAL_OK;
}
int HAL_UART_DeInit(UART_HandleTypeDef *h) {assert(h==handle && h->gState==HAL_UART_STATE_READY);return HAL_OK;}
int HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *h,unsigned threshold) {assert(h==handle && threshold==1);return HAL_OK;}
int HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *h) {assert(h==handle);return HAL_OK;}
void HAL_NVIC_SetPriority(unsigned n,unsigned p,unsigned sub) {assert(n==37 && p==7 && !sub);}
void HAL_NVIC_EnableIRQ(unsigned n) {assert(n==37);++irq;}
int HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *data,uint16_t size) {
 assert(h==handle && h->gState==HAL_UART_STATE_READY);if(fail_tx)return 1;
 pending=data;length=size;h->gState=HAL_UART_STATE_BUSY_TX;return HAL_OK;
}
void HAL_UART_IRQHandler(UART_HandleTypeDef *h) {assert(h==handle);h->gState=HAL_UART_STATE_READY;}
int main(void) {
 uint8_t first[]={1,2,3,4},second[]={9,8,7};
 assert(!BSP_TelemetryUART_Send(first,4,12345));assert(!inits && !gpios);
 assert(!BSP_TelemetryUART_Send(NULL,4,115200));assert(!BSP_TelemetryUART_Send(first,0,115200));
 assert(BSP_TelemetryUART_Send(first,4,115200));assert(inits==1 && gpios==1 && irq==1);
 assert(pending!=first && length==4 && !memcmp(pending,first,4));
 memset(first,0,4);assert(pending[0]==1 && pending[3]==4); /* Caller storage can disappear. */
 assert(!BSP_TelemetryUART_Send(second,3,115200));assert(pending[0]==1 && length==4);
 assert(!BSP_TelemetryUART_Send(second,3,9600));assert(inits==1 && g_telemetry_uart.baudrate==115200);
 USART1_IRQHandler();assert(BSP_TelemetryUART_Send(second,3,9600));assert(inits==2 && handle->Init.BaudRate==9600);
 memset(second,0,3);assert(pending[0]==9 && pending[2]==7);
 USART1_IRQHandler();fail_tx=1;assert(!BSP_TelemetryUART_Send(first,4,9600));fail_tx=0;
 assert(BSP_TelemetryUART_Send(first,4,9600));USART1_IRQHandler();
 fail_clock=1;assert(!BSP_TelemetryUART_Send(first,4,57600));assert(g_telemetry_uart.status==3);
 fail_clock=0;assert(BSP_TelemetryUART_Send(first,4,57600));assert(g_telemetry_uart.baudrate==57600);
 assert(g_telemetry_uart.busy_drops==2 && g_telemetry_uart.errors>=4);
 puts("PASS: USART1 PA10/AF7 TX-RX swap, TX-only 8N1, lazy init, persistent IRQ buffer, busy/no-overwrite, baud changes and nonfatal errors");
}
'''
with tempfile.TemporaryDirectory(prefix='telemetry_uart_') as folder:
    tmp=Path(folder);(tmp/'main.h').write_text(mock);(tmp/'test.c').write_text(harness)
    executable=tmp/'test.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-O2','-I',str(tmp),
                    '-I',str(root/'BSP/Peripherals/Inc'),str(tmp/'test.c'),
                    str(root/'BSP/Peripherals/Src/bsp_telemetry_uart.c'),'-o',str(executable)],check=True)
    subprocess.run([str(executable)],check=True)
