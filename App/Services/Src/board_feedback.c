#include "board_feedback.h"
#include "main.h"
#include "bsp_tim.h"
#include "imu_task.h"
#include "wheel_leg_task.h"
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* HSI/8 = 8 MHz. Ten SPI bits per LED bit: 800 kbit/s.
 * 0 = 1110000000 (375 ns high); 1 = 1111110000 (750 ns high).
 * GRB, MSB first. Each frame includes 320 us low before and after data.
 * Only MOSI is routed to PA7; no SCK/NSS/MISO pins are needed.
 */
#define LED_RESET_BYTES 320U
#define LED_DATA_BYTES  30U
#define LED_FRAME_BYTES (LED_RESET_BYTES * 2U + LED_DATA_BYTES)
#define LED_BUFFER_BYTES ((LED_FRAME_BYTES + 31U) & ~31U)
#define FEEDBACK_PERIOD_MS 20U
#define IMU_STALE_MS 100U

static SPI_HandleTypeDef led_spi;
static DMA_HandleTypeDef led_dma;
/* DMA1 cannot access the default DTCM at 0x20000000. The project scatter
 * file places this dedicated, cache-line-aligned section in AXI SRAM.
 */
static uint8_t led_frame[LED_BUFFER_BYTES]
    __attribute__((section(".led_dma"), aligned(32)));
static TickType_t led_started;
static uint32_t buzzer_counter_hz;
volatile BoardFeedbackDebug g_board_feedback = {.buzzer_enable = 1U};

typedef struct {
    uint32_t remote_enable, remote_status, enable, status, phase, ready;
    uint32_t imu_status, imu_fresh;
} FeedbackInputs;

/* Read only the decision fields, coherently with the higher-priority control
 * task. GPIO/DMA/PWM work stays outside this short critical section. */
static void Feedback_ReadInputs(FeedbackInputs *input)
{
    taskENTER_CRITICAL();
    TickType_t now=xTaskGetTickCount(); /* Cannot precede a newer IMU timestamp. */
    input->remote_enable=g_robot_control.remote_enable;
    input->remote_status=g_robot_control.remote_status;
    input->enable=g_robot_control.enable;
    input->status=g_robot_control.status;
    input->phase=g_robot_control.startup_phase;
    input->ready=g_robot_control.startup_ready_mask;
    input->imu_status=imu_status;
    input->imu_fresh=(TickType_t)(now-(TickType_t)imu_last_sample_tick)<=pdMS_TO_TICKS(IMU_STALE_MS);
    taskEXIT_CRITICAL();
}

static BoardFeedbackStatus Feedback_Classify(const FeedbackInputs *input, uint32_t boot_ms)
{
    if ((input->status>=2U && input->status<=5U) ||
        (input->remote_enable && input->remote_status==5U)) return BOARD_FEEDBACK_FAULT;
    if (input->imu_status==0xFFU && boot_ms<5000U) return BOARD_FEEDBACK_BOOTING;
    if (input->imu_status!=0U || !input->imu_fresh) return BOARD_FEEDBACK_FAULT;
    if (input->remote_enable && (input->remote_status==1U || input->remote_status==3U))
        return BOARD_FEEDBACK_REMOTE_WAIT;
    if (input->enable && (input->phase==ROBOT_STARTUP_PREPARE || input->phase==ROBOT_STARTUP_RAMP))
        return BOARD_FEEDBACK_PREPARING;
    if (input->enable && input->phase==ROBOT_STARTUP_BALANCE) return BOARD_FEEDBACK_BALANCING;
    return BOARD_FEEDBACK_IDLE;
}

/* Repeating groups with a long gap distinguish counts without watching a
 * debugger. Pending preparation is a warning, not a new stop/timeout rule. */
static void Feedback_Pattern(BoardFeedbackStatus status, uint32_t state_ms, uint32_t buzzer_enable,
                             uint8_t *red, uint8_t *green, uint8_t *blue, uint16_t *frequency)
{
    *red=*green=*blue=0U;
    *frequency=0U;
    if (status>=BOARD_FEEDBACK_REMOTE_WAIT && status<=BOARD_FEEDBACK_FAULT) {
        uint32_t count=(uint32_t)status;
        uint32_t led_ms=state_ms%2000U;
        uint32_t lit=led_ms<count*240U && led_ms%240U<120U;
        if (lit && status==BOARD_FEEDBACK_REMOTE_WAIT) {*red=24U;*green=16U;}
        else if (lit && status==BOARD_FEEDBACK_PREPARING) *blue=24U;
        else if (lit) *red=32U;
        /* Wait a second before the first warning, then repeat every 3 s.
         * OFF, balance and recovery immediately silence the warning. */
        if (buzzer_enable && state_ms>=1000U) {
            uint32_t beep_ms=(state_ms-1000U)%3000U;
            if (beep_ms<count*240U && beep_ms%240U<120U)
                *frequency=status==BOARD_FEEDBACK_FAULT ? 1047U : 2093U;
        }
    } else if (status==BOARD_FEEDBACK_BALANCING) {
        uint32_t phase=state_ms%2000U;
        if (phase>1000U) phase=2000U-phase;
        *green=(uint8_t)(3U+phase*29U/1000U);
    } else if (status==BOARD_FEEDBACK_BOOTING) {
        *blue=state_ms%500U<250U ? 24U : 0U;
    } else {
        *green=3U; /* Sensor healthy, control intentionally off. */
    }
}

typedef struct {
    uint16_t frequency_hz;
    uint16_t duration_ms;
} StartupNote;

/* Short ascending startup cue; tune this table for a specific DJI model.
 * A passive buzzer reproduces pitches/rhythm, not a drone motor's timbre.
 * frequency=0 is a rest. This sequence is played once per power-on.
 */
static const StartupNote startup_notes[] = {
    {1047U, 140U}, {0U, 40U},
    {1319U, 140U}, {0U, 40U},
    {1568U, 260U}, {0U, 0U}
};

static void LED_Encode(uint8_t red, uint8_t green, uint8_t blue)
{
    uint32_t grb = ((uint32_t)green << 16) | ((uint32_t)red << 8) | blue;
    uint32_t output_bit = LED_RESET_BYTES * 8U;
    int32_t bit;
    memset(led_frame, 0, sizeof(led_frame));
    for (bit = 23; bit >= 0; --bit) {
        uint16_t symbol = (grb & (1UL << bit)) ? 0x3F0U : 0x380U;
        int32_t subbit;
        for (subbit = 9; subbit >= 0; --subbit, ++output_bit) {
            if (symbol & (1U << subbit)) {
                led_frame[output_bit / 8U] |= (uint8_t)(0x80U >> (output_bit % 8U));
            }
        }
    }
}

static void LED_Set(uint8_t red, uint8_t green, uint8_t blue, TickType_t now)
{
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&led_spi);
    if (state != HAL_SPI_STATE_READY) {
        /* Do not overwrite a frame in flight. Recover a failed DMA/EOT
         * transaction rather than leaving the status task stuck forever.
         */
        if ((TickType_t)(now - led_started) < pdMS_TO_TICKS(FEEDBACK_PERIOD_MS)) return;
        if (HAL_SPI_Abort(&led_spi) != HAL_OK) return;
    }
    LED_Encode(red, green, blue);
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanDCache_by_Addr((uint32_t *)led_frame, sizeof(led_frame));
    }
    led_started = now;
    (void)HAL_SPI_Transmit_DMA(&led_spi, led_frame, LED_FRAME_BYTES);
}

static void Buzzer_SetFrequency(uint16_t frequency_hz)
{
    uint32_t period;
    /* Reload with the channel disabled: avoids a click/partial pulse and
     * guarantees zero duty during rests and after the final note.
     */
    (void)HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    if (frequency_hz == 0U) return;
    period = (buzzer_counter_hz + frequency_hz / 2U) / frequency_hz;
    if (period < 2U) period = 2U;
    if (period > 65536U) period = 65536U;
    __HAL_TIM_SET_AUTORELOAD(&htim12, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, period / 2U);
    __HAL_TIM_SET_COUNTER(&htim12, 0U);
    htim12.Instance->EGR = TIM_EGR_UG;
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void BoardFeedback_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clocks = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    /* Same CLKP selection as BMI088 SPI2; leave its kernel clock unchanged. */
    clocks.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    clocks.Spi123ClockSelection = RCC_SPI123CLKSOURCE_CLKP;
    if (HAL_RCCEx_PeriphCLKConfig(&clocks) != HAL_OK) Error_Handler();
    gpio.Pin = RUN_LED_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(RUN_LED_GPIO_Port, &gpio);

    led_dma.Instance = DMA1_Stream0;
    led_dma.Init.Request = DMA_REQUEST_SPI1_TX;
    led_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    led_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    led_dma.Init.MemInc = DMA_MINC_ENABLE;
    led_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    led_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    led_dma.Init.Mode = DMA_NORMAL;
    led_dma.Init.Priority = DMA_PRIORITY_LOW;
    led_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&led_dma) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&led_spi, hdmatx, led_dma);

    led_spi.Instance = SPI1;
    led_spi.Init.Mode = SPI_MODE_MASTER;
    led_spi.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    led_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    led_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    led_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    led_spi.Init.NSS = SPI_NSS_SOFT;
    led_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    led_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    led_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    led_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    led_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    led_spi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    led_spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    led_spi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    led_spi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    led_spi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    led_spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    led_spi.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&led_spi) != HAL_OK) Error_Handler();
    /* STM32H7 DMA completion enables SPI EOT: both IRQs are required. */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(SPI1_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    MX_TIM12_Init();
    buzzer_counter_hz = HAL_RCC_GetPCLK1Freq() / (htim12.Init.Prescaler + 1U);
    /* Current clock tree: APB1 /1, TIMPRE=0, TIM12 clock = PCLK1 = 64 MHz. */
    if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != 0U) buzzer_counter_hz *= 2U;
    Buzzer_SetFrequency(0U);
    LED_Set(0U, 0U, 20U, 0U);
}

static void BoardFeedback_Task(void *argument)
{
    TickType_t last = xTaskGetTickCount();
    TickType_t boot_tick = last, note_tick = last;
    TickType_t state_tick = last;
    BoardFeedbackStatus shown=BOARD_FEEDBACK_BOOTING;
    uint32_t note_index = 0U;
    uint16_t applied_frequency=startup_notes[0].frequency_hz;
    (void)argument;
    Buzzer_SetFrequency(applied_frequency);
    IMU_Task_Start();
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        uint32_t elapsed_ms = (uint32_t)(now - boot_tick) * portTICK_PERIOD_MS;
        FeedbackInputs input;
        Feedback_ReadInputs(&input);
        BoardFeedbackStatus status=Feedback_Classify(&input,elapsed_ms);
        uint8_t red = 0U, green = 0U, blue = 0U;
        uint16_t frequency=0U;

        if (status!=shown) {shown=status;state_tick=now;}
        g_board_feedback.status=status;
        g_board_feedback.ready_missing_mask=input.enable && input.phase==ROBOT_STARTUP_PREPARE ?
                                             ROBOT_READY_ALL & ~input.ready : 0U;
        g_board_feedback.flash_count=status>=BOARD_FEEDBACK_REMOTE_WAIT && status<=BOARD_FEEDBACK_FAULT ?
                                     (uint32_t)status : 0U;
        Feedback_Pattern(status,(uint32_t)(now-state_tick)*portTICK_PERIOD_MS,
                         g_board_feedback.buzzer_enable,&red,&green,&blue,&frequency);

        /* Catch up after preemption, so task delays cannot stretch the melody. */
        while (startup_notes[note_index].duration_ms != 0U &&
               (TickType_t)(now - note_tick) >= pdMS_TO_TICKS(startup_notes[note_index].duration_ms)) {
            note_tick += pdMS_TO_TICKS(startup_notes[note_index].duration_ms);
            ++note_index;
            applied_frequency=startup_notes[note_index].frequency_hz;
            Buzzer_SetFrequency(applied_frequency);
        }
        if (startup_notes[note_index].duration_ms==0U && frequency!=applied_frequency) {
            applied_frequency=frequency;
            Buzzer_SetFrequency(applied_frequency);
        }
        g_board_feedback.frequency_hz=applied_frequency;
        LED_Set(red, green, blue, now);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(FEEDBACK_PERIOD_MS));
    }
}

void BoardFeedback_Task_Create(void)
{
    if (xTaskCreate(BoardFeedback_Task, "BoardFeedback", 256U, NULL, 1U, NULL) != pdPASS) Error_Handler();
}

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&led_dma);
}

void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&led_spi);
}
