"""Host regression for task scheduling and nonintrusive DWT diagnostics."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = r'''
#ifndef RUNTIME_STUB_H
#define RUNTIME_STUB_H
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef struct { uint32_t CTRL, CYCCNT; } FakeDwt;
typedef struct { uint32_t DEMCR; } FakeCoreDebug;
extern FakeDwt fake_dwt;
extern FakeCoreDebug fake_core;
extern uint32_t SystemCoreClock;
#if TEST_DWT
#define DWT (&fake_dwt)
#define CoreDebug (&fake_core)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define DWT_CTRL_NOCYCCNT_Msk (1U << 25)
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#endif
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdPASS 1
typedef struct { uint32_t before, stack_free_min_bytes, after; } WheelLegDebug;
extern volatile WheelLegDebug g_wheel_leg;
void WheelLeg_Update(void);
void WheelLeg_Task_Create(void);
void LqrStartupRecord_Init(void);
TickType_t xTaskGetTickCount(void);
unsigned uxTaskGetStackHighWaterMark(void *task);
void vTaskDelayUntil(TickType_t *last, TickType_t period);
int xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
                void *argument, unsigned priority, void *handle);
void Error_Handler(void);
#endif
'''
HARNESS = r'''
#include "runtime_stub.h"
#include "wheel_leg_runtime.h"
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "wheel_leg_runtime.c"

FakeDwt fake_dwt;
FakeCoreDebug fake_core;
uint32_t SystemCoreClock = 400000000U;
volatile WheelLegDebug g_wheel_leg;
static jmp_buf task_exit;
static void (*created_task)(void *);
static unsigned create_ok=1U, create_calls, record_inits, errors;
static uint32_t now, work_cycles, stack_cycles, work_ticks, first_counter;
static unsigned update_calls, stack_calls, delays, stop_after, disable_counter;
static uint32_t scan_ticks[8], delay_anchors[512];

void LqrStartupRecord_Init(void) { ++record_inits; }
int xTaskCreate(void (*fn)(void *), const char *name, unsigned stack,
                void *argument, unsigned priority, void *handle) {
    assert(fn && !strcmp(name,"wheel_leg") && stack==1024U);
    assert(!argument && priority==4U && !handle);
    created_task=fn; ++create_calls; return create_ok;
}
void Error_Handler(void) { ++errors; }
TickType_t xTaskGetTickCount(void) { return now; }
void WheelLeg_Update(void) {
    if (!update_calls) first_counter=fake_dwt.CYCCNT;
    ++update_calls; now+=work_ticks; fake_dwt.CYCCNT+=work_cycles;
    if (disable_counter) fake_dwt.CTRL&=~1U;
}
unsigned uxTaskGetStackHighWaterMark(void *task) {
    assert(!task && stack_calls<8U);
    scan_ticks[stack_calls]=now; ++stack_calls;
    fake_dwt.CYCCNT+=stack_cycles;
    return 512U-stack_calls;
}
void vTaskDelayUntil(TickType_t *last, TickType_t period) {
    assert(period==5U && delays<512U);
    delay_anchors[delays++]=*last;
    TickType_t next=*last+period;
    if ((int32_t)(next-now)>0) {
        uint32_t advance=next-now;
        fake_dwt.CYCCNT+=(uint32_t)((uint64_t)advance*SystemCoreClock/1000U);
        now=next;
    }
    *last=next;
    if (delays==stop_after) longjmp(task_exit,1);
}
static void reset(void) {
    now=0U;work_cycles=40000U;stack_cycles=8000U;work_ticks=0U;
    update_calls=stack_calls=delays=disable_counter=0U;
    fake_dwt.CTRL=0x10U; fake_dwt.CYCCNT=0xfedcba98U;fake_core.DEMCR=0x20U;
    SystemCoreClock=400000000U;
    g_wheel_leg.before=0x1234U;g_wheel_leg.after=0x5678U;
    g_wheel_leg.stack_free_min_bytes=0U;
}
static void run(unsigned count) {
    stop_after=count;
    if (!setjmp(task_exit)) created_task(NULL);
    assert(update_calls==count && delays==count);
    assert(g_wheel_leg.before==0x1234U && g_wheel_leg.after==0x5678U);
}
int main(void) {
    assert(WHEEL_LEG_CONTROL_PERIOD_MS==5U && WHEEL_LEG_STACK_SAMPLE_PERIOD_MS==1000U);
    WheelLeg_Task_Create();
    assert(create_calls==1U && record_inits==1U && errors==0U);
    create_ok=0U;WheelLeg_Task_Create();assert(errors==1U && record_inits==2U);
    create_ok=1U;

    reset();uint32_t initial=fake_dwt.CYCCNT;
    run(202U);
    assert(first_counter==initial); /* Task startup must preserve the shared counter. */
    assert(stack_calls==2U && scan_ticks[0]==0U && scan_ticks[1]==1000U);
    assert(g_wheel_leg.stack_free_min_bytes==510U*sizeof(StackType_t));
    assert(delay_anchors[0]==0U && delay_anchors[201]==1005U);
#if TEST_DWT
    assert(g_wheel_leg_runtime.available==1U && g_wheel_leg_runtime.samples==202U);
    assert(g_wheel_leg_runtime.core_clock_hz==400000000U);
    assert(g_wheel_leg_runtime.last_cycles==40000U && g_wheel_leg_runtime.last_us==100U);
    assert(g_wheel_leg_runtime.max_cycles==48000U && g_wheel_leg_runtime.max_us==120U);
    assert(g_wheel_leg_runtime.budget_overruns==0U);
    assert(fake_dwt.CTRL==(0x10U|1U) && fake_core.DEMCR==(0x20U|(1U<<24)));
#else
    assert(g_wheel_leg_runtime.available==0U && g_wheel_leg_runtime.samples==0U);
    assert(fake_dwt.CTRL==0x10U && fake_core.DEMCR==0x20U);
#endif

    reset();now=100U;work_ticks=7U;run(2U);
    assert(delay_anchors[0]==107U && delay_anchors[1]==119U); /* Original >5ms re-anchor. */
    reset();now=100U;work_ticks=5U;run(1U);
    assert(delay_anchors[0]==100U); /* At exactly 5ms the old anchor remains. */
    reset();now=UINT32_MAX-3U;run(202U);
    assert(stack_calls==2U && (uint32_t)(scan_ticks[1]-scan_ticks[0])==1000U);

#if TEST_DWT
    reset();fake_dwt.CYCCNT=UINT32_MAX-199U;work_cycles=400U;stack_cycles=0U;run(1U);
    assert(first_counter==UINT32_MAX-199U && g_wheel_leg_runtime.last_cycles==400U);
    assert(g_wheel_leg_runtime.last_us==1U && g_wheel_leg_runtime.samples==1U);

    reset();work_cycles=2000001U;stack_cycles=0U;run(1U);
    assert(g_wheel_leg_runtime.last_us==5000U && g_wheel_leg_runtime.budget_overruns==1U);
    runtime_counter_init();
    runtime_record_cycles(2000000U,400000000U);
    assert(g_wheel_leg_runtime.budget_overruns==0U);
    runtime_record_cycles(2000001U,400000000U);
    assert(g_wheel_leg_runtime.budget_overruns==1U);
    runtime_record_cycles(48000001U,48000001U); /* Non-integral-MHz conversion path. */
    assert(g_wheel_leg_runtime.last_us==1000000U);
    unsigned samples=g_wheel_leg_runtime.samples;
    runtime_record_cycles(100U,0U);assert(g_wheel_leg_runtime.samples==samples);

    reset();SystemCoreClock=0U;run(1U);
    assert(!g_wheel_leg_runtime.available && !g_wheel_leg_runtime.samples);
    reset();fake_dwt.CTRL=(1U<<25)|1U;fake_core.DEMCR=1U<<24;run(1U);
    assert(!g_wheel_leg_runtime.available && !g_wheel_leg_runtime.samples);
    reset();disable_counter=1U;run(1U);
    assert(!g_wheel_leg_runtime.available && !g_wheel_leg_runtime.samples);
#endif
    puts(TEST_DWT ? "DWT scheduling/diagnostic regression passed" : "No-DWT fallback regression passed");
    return 0;
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="wheel_leg_runtime_") as temp:
        work = Path(temp)
        (work / "runtime_stub.h").write_text(RUNTIME, encoding="utf-8")
        for header in ("wheel_leg_task.h", "lqr_startup_record.h", "main.h", "FreeRTOS.h", "task.h"):
            (work / header).write_text('#include "runtime_stub.h"\n', encoding="utf-8")
        (work / "harness.c").write_text(HARNESS, encoding="utf-8")
        for dwt in (1, 0):
            binary = work / f"runtime_{dwt}.exe"
            subprocess.run([
                "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", f"-DTEST_DWT={dwt}",
                f"-I{work}", f"-I{ROOT / 'App/Tasks/Inc'}", f"-I{ROOT / 'App/Tasks/Src'}",
                str(work / "harness.c"), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
