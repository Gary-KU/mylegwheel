#ifndef WHEEL_LEG_RUNTIME_H
#define WHEEL_LEG_RUNTIME_H

#include <stdint.h>

#define WHEEL_LEG_CONTROL_PERIOD_MS 5U
#define WHEEL_LEG_STACK_SAMPLE_PERIOD_MS 1000U

/* 迭代经过时间包含中断/抢占及栈诊断，不是任务独占 CPU 时间；不包含休眠。
 * available=0 表示 DWT 或核心时钟不可用，此时耗时字段不产生新样本。 */
typedef struct {
    uint32_t available, core_clock_hz;
    uint32_t samples, last_cycles, max_cycles;
    uint32_t last_us, max_us, budget_overruns;
} WheelLegRuntimeDebug;

extern volatile WheelLegRuntimeDebug g_wheel_leg_runtime;

#endif
