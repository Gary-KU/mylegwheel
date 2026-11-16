/* BMI088 sampling and USB CDC / ANO V7 telemetry. */
#include "imu_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ano_v7.h"
#include "robot_telemetry_service.h"
#include "lqr_startup_record.h"
#include "bmi088_driver.h"
#include "mahony_ahrs.h"
#include "pid.h"
#include "bsp_tim.h"
#include "bsp_spi.h"
#include <math.h>

#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk) && \
    defined(CoreDebug_DEMCR_TRCENA_Msk)
#define IMU_HAS_DWT 1
#else
#define IMU_HAS_DWT 0
#endif
volatile ImuRuntimeDebug g_imu_runtime;

static uint32_t imu_runtime_start(void)
{
#if IMU_HAS_DWT
    g_imu_runtime.clock_hz=SystemCoreClock;
    g_imu_runtime.available=SystemCoreClock!=0U &&
        (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
        (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
#ifdef DWT_CTRL_NOCYCCNT_Msk
    if(DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) g_imu_runtime.available=0U;
#endif
    return g_imu_runtime.available ? DWT->CYCCNT : 0U;
#else
    return 0U;
#endif
}

static void imu_runtime_init(void)
{
    const ImuRuntimeDebug zero={0};g_imu_runtime=zero;
#if IMU_HAS_DWT
    g_imu_runtime.clock_hz=SystemCoreClock;
#ifdef DWT_CTRL_NOCYCCNT_Msk
    if(DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) return;
#endif
    /* Share the cycle counter with other tasks; never reset CYCCNT. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    (void)imu_runtime_start();
#endif
}

static void imu_runtime_finish(uint32_t started)
{
#if IMU_HAS_DWT
    const uint32_t ended=imu_runtime_start();
    if(!g_imu_runtime.available) return;
    const uint32_t elapsed=ended-started; /* Supports one CYCCNT wrap. */
    g_imu_runtime.last_cycles=elapsed;
    if(elapsed>g_imu_runtime.max_cycles) g_imu_runtime.max_cycles=elapsed;
    ++g_imu_runtime.sample_count;
    if((uint64_t)elapsed*1000U>g_imu_runtime.clock_hz) ++g_imu_runtime.budget_overruns;
#else
    (void)started;
#endif
}

static float gyro[3], accel[3], temp;
static float roll, pitch, yaw;
static pid_type_def Temperature_PID;
static const float RefTemp = 40.0f;
static const float Temperature_PID_Para[3] = {1600.0f, 50.0f, 40.0f};
/* 0=healthy; BMI088 init code; 0x100=invalid read; 0x101=invalid attitude. */
volatile uint32_t imu_status = 0xFFU;
volatile uint32_t imu_last_sample_tick;

static QueueHandle_t imu_sample_queue;
static TaskHandle_t imu_task_handle;

uint8_t IMU_GetSample(ImuSample *sample)
{
    ImuSample latest;
    if (sample == NULL) return 0U;
    *sample = (ImuSample){0};
    /* 整包读取保持姿态、角速度和时间戳来自同一采样；异常后不沿用上次正常姿态。 */
    if (imu_sample_queue == NULL ||
        xQueuePeek(imu_sample_queue, &latest, 0) != pdPASS ||
        latest.status != 0U || imu_status != 0U ||
        (TickType_t)(xTaskGetTickCount() - latest.tick) > pdMS_TO_TICKS(10)) return 0U;
    *sample = latest;
    return 1U;
}

void IMU_Task_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* Set CS/power output levels before switching pin modes. */
    HAL_GPIO_WritePin(GPIOC, ACC_CS_Pin | GYRO_CS_Pin | POWER_5V_Pin, GPIO_PIN_SET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = ACC_CS_Pin | GYRO_CS_Pin | POWER_5V_Pin;
    HAL_GPIO_Init(GPIOC, &gpio);
    MX_SPI2_Init();
    MX_TIM3_Init();
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    PID_init(&Temperature_PID, PID_POSITION, Temperature_PID_Para, 2000.0f, 200.0f);
    /* Sensor initialization/retries run in the task so USB can enumerate. */
}

static void IMU_Task(void *argument)
{
    TickType_t last;
    uint8_t ready = 0, mahony_first = 1;
    uint32_t temp_cnt = 0;
    ImuSample sample = {0};
    (void)argument;
    imu_runtime_init();
    /* The feedback task starts the first note, then releases initialization.
     * Sampling remains gated by successful BMI088_init() below.
     */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (;;) {
        if (!ready) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            imu_status = BMI088_init();
            sample.status = imu_status;
            sample.tick = xTaskGetTickCount();
            xQueueOverwrite(imu_sample_queue, &sample);
            if (imu_status != 0U) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            Mahony_Init(1000.0f);
            mahony_first = 1;
            ready = 1;
            last = xTaskGetTickCount();
        }
        const uint32_t runtime_started=imu_runtime_start();
        /* 芯片 ID 校验失败时驱动不会写回 gyro；预填 NaN 可识别失败，避免误用旧值。 */
        gyro[0] = gyro[1] = gyro[2] = NAN;
        BMI088_read(gyro, accel, &temp);
        if (!isfinite(gyro[0]) || !isfinite(gyro[1]) || !isfinite(gyro[2]) ||
            !isfinite(accel[0]) || !isfinite(accel[1]) || !isfinite(accel[2]) ||
            !isfinite(temp) || temp < -40.0f || temp > 85.0f) {
            imu_status = 0x100U;
            sample.status = imu_status;
            sample.tick = xTaskGetTickCount();
            xQueueOverwrite(imu_sample_queue, &sample);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            ready = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        /* 初始化或故障重试后用加速度重建初始倾角；无磁力计，航向从零开始积分。 */
        if (mahony_first) {
            MahonyAHRSinit(accel[0], accel[1], accel[2], 0, 0, 0);
            mahony_first = 0;
        }
        Mahony_update(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0, 0, 0);
        Mahony_computeAngles();
        roll = getRoll(); pitch = getPitch(); yaw = getYaw();
        if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw)) {
            imu_status = 0x101U;
            sample.status = imu_status;
            sample.tick = xTaskGetTickCount();
            xQueueOverwrite(imu_sample_queue, &sample);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            ready = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        /* 1 kHz 采样中每 10 拍更新加热 PWM；仅加热，温控更新时若超过 45°C 则关闭。 */
        if (++temp_cnt % 10U == 0U) {
            int32_t pwm = 0;
            if (temp <= 45.0f) {
                PID_calc(&Temperature_PID, temp, RefTemp);
                pwm = (int32_t)Temperature_PID.out;
                if (pwm < 0) pwm = 0;
                if (pwm > 2000) pwm = 2000;
            }
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)pwm);
        }
        sample.attitude_deg[0] = roll;
        sample.attitude_deg[1] = pitch;
        sample.attitude_deg[2] = yaw;
        for (uint32_t i = 0U; i < 3U; ++i) sample.gyro_rad_s[i] = gyro[i];
        sample.temperature = temp;
        sample.tick = xTaskGetTickCount();
        sample.status = 0U;
        imu_last_sample_tick = (uint32_t)sample.tick;
        imu_status = 0U;
        xQueueOverwrite(imu_sample_queue, &sample);
        imu_runtime_finish(runtime_started);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}

static void Comm_Task(void *argument)
{
    TickType_t last = xTaskGetTickCount();
    TickType_t last_status = last;
    ImuSample sample;
    (void)argument;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        LqrStartupRecord_Poll();
        /* F2 channel 1: status code (0=OK); sent separately at 1 Hz. */
        if (g_robot_telemetry.enable) {
            RobotTelemetry_Poll();
        } else if ((TickType_t)(now-last_status) >= pdMS_TO_TICKS(1000)) {
            uint32_t status = imu_status;
            uint8_t payload[4] = {(uint8_t)status, (uint8_t)(status>>8),
                                  (uint8_t)(status>>16), (uint8_t)(status>>24)};
            ano_v7_send(0xF2, payload, sizeof(payload));
            last_status = now;
        } else if (xQueuePeek(imu_sample_queue, &sample, 0) == pdPASS &&
                   sample.status == 0U &&
                   (TickType_t)(now-sample.tick) <= pdMS_TO_TICKS(50)) {
            /* Mahony getters already return degrees; transmit degrees *100.
             * F1: roll,pitch,yaw,temp, four little-endian signed int32.
             */
            ano_v7_send_data((int32_t)(sample.attitude_deg[0]*100.0f),
                             (int32_t)(sample.attitude_deg[1]*100.0f),
                             (int32_t)(sample.attitude_deg[2]*100.0f),
                             (int32_t)(sample.temperature*100.0f));
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

void IMU_Task_Create(void)
{
    RobotTelemetry_Init();
    /* 单元素队列配合覆盖写入，控制与通信各自查看最新整包，避免积压历史采样。 */
    imu_sample_queue = xQueueCreate(1, sizeof(ImuSample));
    if (imu_sample_queue == NULL) Error_Handler();
    if (xTaskCreate(IMU_Task, "IMU_Task", 512, NULL, 4, &imu_task_handle) != pdPASS) Error_Handler();
    if (xTaskCreate(Comm_Task, "Comm_Task", 512, NULL, 2, NULL) != pdPASS) Error_Handler();
}

void IMU_Task_Start(void)
{
    if (imu_task_handle != NULL) xTaskNotifyGive(imu_task_handle);
}
