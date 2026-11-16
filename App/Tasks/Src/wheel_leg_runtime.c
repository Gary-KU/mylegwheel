#include "wheel_leg_runtime.h"
#include "wheel_leg_task.h"
#include "lqr_startup_record.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk) && \
    defined(CoreDebug_DEMCR_TRCENA_Msk)
#define WHEEL_LEG_HAS_DWT 1
#else
#define WHEEL_LEG_HAS_DWT 0
#endif

volatile WheelLegRuntimeDebug g_wheel_leg_runtime;

static void runtime_counter_init(void)
{
    const WheelLegRuntimeDebug zero = {0};
    g_wheel_leg_runtime = zero;
#if WHEEL_LEG_HAS_DWT
    g_wheel_leg_runtime.core_clock_hz = SystemCoreClock;
#ifdef DWT_CTRL_NOCYCCNT_Msk
    if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) return;
#endif
    /* 与姿态等 DWT 用户共享计数器，只开使能位，绝不重置 CYCCNT。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_wheel_leg_runtime.available = SystemCoreClock != 0U &&
        (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
        (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
#endif
}

static uint32_t runtime_counter_read(void)
{
#if WHEEL_LEG_HAS_DWT
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

#if WHEEL_LEG_HAS_DWT
static void runtime_record_cycles(uint32_t elapsed, uint32_t clock_hz)
{
    if (!clock_hz) return;
    uint32_t us;
    if (clock_hz % 1000000U == 0U)
        us = elapsed / (clock_hz / 1000000U);
    else us = (uint32_t)(((uint64_t)elapsed * 1000000U) / clock_hz);
    g_wheel_leg_runtime.core_clock_hz = clock_hz;
    g_wheel_leg_runtime.last_cycles = elapsed;
    g_wheel_leg_runtime.last_us = us;
    if (elapsed > g_wheel_leg_runtime.max_cycles) g_wheel_leg_runtime.max_cycles = elapsed;
    if (us > g_wheel_leg_runtime.max_us) g_wheel_leg_runtime.max_us = us;
    ++g_wheel_leg_runtime.samples;
    /* 严格超过 5 ms 才计超预算；直接比较周期数，避免微秒取整掩盖超时。 */
    if ((uint64_t)elapsed * 1000U > (uint64_t)clock_hz * WHEEL_LEG_CONTROL_PERIOD_MS)
        ++g_wheel_leg_runtime.budget_overruns;
}
#endif

static void runtime_timing_finish(uint32_t started)
{
#if WHEEL_LEG_HAS_DWT
    uint32_t clock_hz = SystemCoreClock;
    g_wheel_leg_runtime.core_clock_hz = clock_hz;
    g_wheel_leg_runtime.available = clock_hz != 0U &&
        (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
        (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
#ifdef DWT_CTRL_NOCYCCNT_Msk
    if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) g_wheel_leg_runtime.available = 0U;
#endif
    if (g_wheel_leg_runtime.available) {
        /* uint32 差值允许 CYCCNT 单次回绕；统计写回与下一次休眠不计入。 */
        uint32_t elapsed = runtime_counter_read() - started;
        runtime_record_cycles(elapsed, clock_hz);
    }
#else
    (void)started;
#endif
}

static void WheelLeg_Task(void *argument)
{
    TickType_t last = xTaskGetTickCount();
    TickType_t last_stack_sample = last;
    uint32_t stack_sampled = 0U;
    (void)argument;
    runtime_counter_init();
    for (;;) {
        uint32_t started = runtime_counter_read();
        WheelLeg_Update();
        TickType_t now = xTaskGetTickCount();
        /* 栈扫描只用于诊断：首拍保留可见值，随后每秒更新一次。 */
        if (!stack_sampled || (TickType_t)(now - last_stack_sample) >=
            pdMS_TO_TICKS(WHEEL_LEG_STACK_SAMPLE_PERIOD_MS)) {
            g_wheel_leg.stack_free_min_bytes =
                (uint32_t)uxTaskGetStackHighWaterMark(NULL) * (uint32_t)sizeof(StackType_t);
            last_stack_sample = now;
            stack_sampled = 1U;
        }
        now = xTaskGetTickCount();
        if ((TickType_t)(now - last) > pdMS_TO_TICKS(WHEEL_LEG_CONTROL_PERIOD_MS)) last = now;
        runtime_timing_finish(started);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(WHEEL_LEG_CONTROL_PERIOD_MS));
    }
}

void WheelLeg_Task_Create(void)
{
    LqrStartupRecord_Init();
    /* 保留 1024 word 栈，预留浮点上下文空间；结合链接报告与运行时栈高水位确认余量。 */
    if (xTaskCreate(WheelLeg_Task, "wheel_leg", 1024U, NULL, 4U, NULL) != pdPASS) Error_Handler();
}
