"""Run actual IMU/communication task bodies with sensor and RTOS mocks."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "App/Tasks/Src/imu_task.c").read_text(encoding="utf-8")


def function(name):
    start = re.search(r"(?:static (?:void|uint32_t)|uint8_t) " + name + r"\(", source).start()
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


header = (root / "App/Tasks/Inc/imu_task.h").read_text(encoding="utf-8")
sample_type = re.search(r"typedef struct \{.*?\} ImuSample;", header, re.S)[0]
runtime_type = re.search(r"typedef struct \{[^}]*\} ImuRuntimeDebug;", header, re.S)[0]
dwt_guard = source[source.index('#if defined(DWT)'):source.index('volatile ImuRuntimeDebug')]
harness = r'''
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <setjmp.h>
#include <stdio.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define pdPASS 1
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
#define TIM_CHANNEL_4 4
SAMPLE_TYPE
RUNTIME_TYPE
#if TEST_IMU_DWT
static struct {uint32_t CTRL,CYCCNT;} fake_dwt;
static struct {uint32_t DEMCR;} fake_core_debug;
#define DWT (&fake_dwt)
#define CoreDebug (&fake_core_debug)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define DWT_CTRL_NOCYCCNT_Msk (1U<<25)
#define CoreDebug_DEMCR_TRCENA_Msk (1U<<24)
static uint32_t SystemCoreClock=550000000U;
#endif
DWT_GUARD
volatile ImuRuntimeDebug g_imu_runtime;
static uint32_t runtime_work_cycles=100U;
static unsigned iterations_until_sleep=1U;
static void fake_cycles(uint32_t cycles) {
#if TEST_IMU_DWT
    DWT->CYCCNT+=cycles;
#else
    (void)cycles;
#endif
}
RUNTIME_FUNCTIONS
static float gyro[3], accel[3], temp, roll, pitch, yaw;
static float RefTemp = 40;
static struct { float out; } Temperature_PID;
static uint32_t imu_status, imu_last_sample_tick;
static struct { uint32_t enable; } g_robot_telemetry;
static unsigned telemetry_polls;
static void RobotTelemetry_Poll(void) { ++telemetry_polls; }
static void LqrStartupRecord_Poll(void) {}
static void *imu_sample_queue = (void *)1;
static int htim3;
static ImuSample queued;
static int mode, peek_ok, frames, status_frames, notifications, updates;
static int32_t data[4];
static uint32_t received_status, clock_calls, pwm, delay;
static TickType_t first_tick, now_tick;
static jmp_buf task_end;
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((void)(timer), (void)(channel), pwm = (value))
static TickType_t xTaskGetTickCount(void) {
    return clock_calls++ == 0 ? first_tick : now_tick;
}
static unsigned ulTaskNotifyTake(int clear, uint32_t timeout) {
    assert(clear == pdTRUE && timeout == portMAX_DELAY);
    ++notifications; return 1;
}
static void vTaskDelay(TickType_t ticks) {
    delay = ticks; fake_cycles(3000000U); longjmp(task_end, 1);
}
static void vTaskDelayUntil(TickType_t *last, TickType_t ticks) {
    *last += ticks; delay = ticks; fake_cycles(3000000U);
    if(--iterations_until_sleep==0U) longjmp(task_end, 1);
    now_tick+=ticks;
}
static int xQueueOverwrite(void *queue, const ImuSample *sample) {
    (void)queue; queued = *sample; fake_cycles(17U); return pdPASS;
}
static int xQueuePeek(void *queue, ImuSample *sample, int timeout) {
    (void)queue; assert(timeout == 0); *sample = queued; return peek_ok;
}
static uint32_t BMI088_init(void) { fake_cycles(1000000U);return mode == 4 ? 7 : 0; }
static void BMI088_read(float *g, float *a, float *temperature) {
    fake_cycles(runtime_work_cycles);
    g[0] = 1; g[1] = 2; g[2] = 3;
    a[0] = 0; a[1] = 0; a[2] = 9.8f;
    *temperature = 37.75f;
    if (mode == 1) g[2] = NAN;
    if (mode == 2) a[0] = NAN;
}
static void Mahony_Init(float rate) { assert(rate == 1000); }
static void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz) {
    (void)ax; (void)ay; assert(az == 9.8f && mx == 0 && my == 0 && mz == 0);
}
static void Mahony_update(float gx, float gy, float gz, float ax, float ay,
                          float az, float mx, float my, float mz) {
    assert(gx == 1 && gy == 2 && gz == 3);
    (void)ax; (void)ay; (void)az; (void)mx; (void)my; (void)mz; ++updates;fake_cycles(100U);
}
static void Mahony_computeAngles(void) {fake_cycles(200U);}
static float getRoll(void) { return mode == 3 ? NAN : 12.5f; }
static float getPitch(void) { return -7.25f; }
static float getYaw(void) { return 90.25f; }
static void PID_calc(void *pid, float temperature, float ref) {
    (void)pid; (void)temperature; (void)ref;fake_cycles(1000U);
}
static void ano_v7_send_data(int32_t a, int32_t b, int32_t c, int32_t d) {
    data[0] = a; data[1] = b; data[2] = c; data[3] = d; ++frames;
}
static void ano_v7_send(uint8_t id, const uint8_t *payload, unsigned size) {
    assert(id == 0xF2 && size == 4);
    received_status = payload[0] | ((uint32_t)payload[1] << 8) |
        ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
    ++status_frames;
}
IMU_TASK
COMM_TASK
GET_SAMPLE
static void sample_once(int failure) {
    mode = failure; clock_calls = 0; first_tick = now_tick = 100;
    pwm = 100; notifications = updates = 0;iterations_until_sleep=1;
    if (!setjmp(task_end)) IMU_Task(NULL);
    assert(notifications == 1);
    assert(queued.tick == 100);
    if (failure) {
        assert(queued.status == (failure == 4 ? 7U : failure == 3 ? 257U : 256U));
        assert(imu_status == queued.status && pwm == 0 && delay == 1000);
        assert(updates == (failure == 3 ? 1 : 0));
    } else {
        assert(queued.status == 0 && imu_status == 0 && imu_last_sample_tick == 100);
        assert(queued.attitude_deg[0] == 12.5f && queued.attitude_deg[1] == -7.25f);
        assert(queued.attitude_deg[2] == 90.25f && queued.temperature == 37.75f);
        assert(queued.gyro_rad_s[0] == 1 && queued.gyro_rad_s[1] == 2 && queued.gyro_rad_s[2] == 3);
        assert(updates == 1 && delay == 1);
    }
#if TEST_IMU_DWT
    assert(g_imu_runtime.available && g_imu_runtime.clock_hz==SystemCoreClock);
    assert(g_imu_runtime.sample_count==(failure?0U:1U));
    assert(g_imu_runtime.last_cycles==(failure?0U:runtime_work_cycles+317U));
#else
    assert(!g_imu_runtime.available && !g_imu_runtime.sample_count && !g_imu_runtime.last_cycles);
#endif
}
static void communicate(TickType_t start, TickType_t now, int expected_frames) {
    first_tick = start; now_tick = now; clock_calls = 0; frames = status_frames = 0;iterations_until_sleep=1;
    if (!setjmp(task_end)) Comm_Task(NULL);
    assert(frames == expected_frames && delay == 10);
    if (frames) {
        assert(data[0] == 1250 && data[1] == -725 && data[2] == 9025 && data[3] == 3775);
    }
}
static void test_runtime(void) {
    mode=0;clock_calls=0;first_tick=now_tick=100;updates=0;iterations_until_sleep=10;
    runtime_work_cycles=100;
    if(!setjmp(task_end)) IMU_Task(NULL);
    assert(updates==10 && queued.tick==109 && queued.status==0);
#if TEST_IMU_DWT
    /* Actual task includes publication and its tenth-sample heater work,
     * excludes init and all simulated3M-cycle sleeps, and never resets DWT. */
    assert(g_imu_runtime.sample_count==10 && g_imu_runtime.last_cycles==1417 &&
           g_imu_runtime.max_cycles==1417 && !g_imu_runtime.budget_overruns);
    runtime_work_cycles=SystemCoreClock/1000U-317U;sample_once(0);
    assert(g_imu_runtime.last_cycles==SystemCoreClock/1000U && !g_imu_runtime.budget_overruns);
    ++runtime_work_cycles;sample_once(0);assert(g_imu_runtime.budget_overruns==1);
    DWT->CYCCNT=UINT32_MAX-50U;imu_runtime_init();assert(DWT->CYCCNT==UINT32_MAX-50U);
    uint32_t started=imu_runtime_start();fake_cycles(80);imu_runtime_finish(started);
    assert(g_imu_runtime.last_cycles==80 && g_imu_runtime.sample_count==1);
    fake_cycles(100);started=imu_runtime_start();fake_cycles(40);imu_runtime_finish(started);
    assert(g_imu_runtime.last_cycles==40 && g_imu_runtime.max_cycles==80 && g_imu_runtime.sample_count==2);
    uint32_t saved_clock=SystemCoreClock;SystemCoreClock=0;imu_runtime_finish(0);
    assert(!g_imu_runtime.available && g_imu_runtime.sample_count==2);SystemCoreClock=saved_clock;
    DWT->CTRL=0;imu_runtime_finish(0);assert(!g_imu_runtime.available && g_imu_runtime.sample_count==2);
    DWT->CTRL=DWT_CTRL_CYCCNTENA_Msk;CoreDebug->DEMCR=0;imu_runtime_finish(0);
    assert(!g_imu_runtime.available && g_imu_runtime.sample_count==2);
    DWT->CTRL=DWT_CTRL_NOCYCCNT_Msk;started=DWT->CYCCNT;imu_runtime_init();
    assert(!g_imu_runtime.available && DWT->CYCCNT==started && !g_imu_runtime.sample_count);
    DWT->CTRL=0;imu_runtime_init();assert(g_imu_runtime.available && DWT->CYCCNT==started);
    puts("IMU runtime passed: real task read/fusion/queue/heater scope, strict1ms budget, wrap/max, disabled clock/DWT and shared-counter preservation");
#else
    assert(!g_imu_runtime.available && !g_imu_runtime.clock_hz && !g_imu_runtime.sample_count);
    puts("IMU runtime passed: absent DWT remains unavailable and sampling continues unchanged");
#endif
    runtime_work_cycles=100;
}
int main(void) {
    peek_ok = pdPASS;
    g_robot_telemetry.enable=1;
    communicate(100,100,0);assert(telemetry_polls==1 && status_frames==0);
    g_robot_telemetry.enable=0;
    sample_once(0); communicate(100, 100, 1);
    communicate(100, 150, 1); communicate(100, 151, 0);
    peek_ok = 0; communicate(100, 100, 0); peek_ok = pdPASS;
    for (int failure = 1; failure <= 4; ++failure) {
        sample_once(failure); communicate(100, 100, 0);
    }
    communicate(100, 1100, 0);
    assert(status_frames == 1 && received_status == 7);
    sample_once(0);
    ImuSample sample;
    now_tick=110;assert(IMU_GetSample(&sample) && sample.gyro_rad_s[1]==2);
    now_tick=111;assert(!IMU_GetSample(&sample) && sample.gyro_rad_s[1]==0);
    now_tick=100;imu_status=1;assert(!IMU_GetSample(&sample));imu_status=0;
    peek_ok=0;assert(!IMU_GetSample(&sample));peek_ok=pdPASS;
    imu_sample_queue=NULL;assert(!IMU_GetSample(&sample));imu_sample_queue=(void *)1;
    assert(!IMU_GetSample(NULL));
    queued.tick = UINT32_MAX - 5;
    now_tick=4;assert(IMU_GetSample(&sample));
    communicate(UINT32_MAX - 5, 4, 1);
    test_runtime();
    puts("IMU telemetry passed: solved angles, scaling, temperature, invalid/stale data, status and tick wrap");
}
'''
harness = harness.replace("SAMPLE_TYPE", sample_type)
harness = harness.replace("RUNTIME_TYPE", runtime_type)
harness = harness.replace("DWT_GUARD", dwt_guard)
harness = harness.replace("RUNTIME_FUNCTIONS", '\n'.join(function(name) for name in
    ('imu_runtime_start','imu_runtime_init','imu_runtime_finish')))
harness = harness.replace("IMU_TASK", function("IMU_Task"))
harness = harness.replace("COMM_TASK", function("Comm_Task"))
harness = harness.replace("GET_SAMPLE", function("IMU_GetSample"))
with tempfile.TemporaryDirectory(prefix="imu_telemetry_") as folder:
    c_file = Path(folder) / "check.c"
    executable = Path(folder) / "check.exe"
    c_file.write_text(harness, encoding="utf-8")
    for has_dwt in (0,1):
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        f"-DTEST_IMU_DWT={has_dwt}", str(c_file), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
