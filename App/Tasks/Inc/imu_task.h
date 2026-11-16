#ifndef IMU_TASK_H
#define IMU_TASK_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float attitude_deg[3]; /* roll, pitch, yaw */
    float gyro_rad_s[3];   /* sensor X, Y, Z */
    float temperature;
    uint32_t tick;
    uint32_t status;
} ImuSample;
/* Nonblocking coherent sample; returns 0 on invalid/stale (>10 ms) data. */
uint8_t IMU_GetSample(ImuSample *sample);
/* Initialize SPI/GPIO and heater (initial duty=0). */
void IMU_Task_Init(void);
/* 1kHz IMU sampling. Default USB telemetry uses coherent robot ANO pairs
 * at 50Hz; see g_robot_telemetry and docs/robot_telemetry.md.
 * Setting g_robot_telemetry.enable=0 restores the legacy 100Hz IMU F1:
 * roll,pitch,yaw (degrees*100), temperature (C*100), plus 1Hz F2 status.
 */
void IMU_Task_Create(void);
/* Release initialization after the status task has started the startup cue. */
void IMU_Task_Start(void);
extern volatile uint32_t imu_status;
/* FreeRTOS tick of the last valid, fully processed sample. */
extern volatile uint32_t imu_last_sample_tick;
/* Valid-sample elapsed time from BMI088 read through fusion, heater update
 * and coherent queue publication. Includes preemption; excludes sleep,
 * sensor startup/retries and final diagnostic stores. available=0 means DWT/clock
 * timing is unavailable. Over-budget counts are observational only. */
typedef struct {
    uint32_t available, clock_hz, sample_count;
    uint32_t last_cycles, max_cycles, budget_overruns;
} ImuRuntimeDebug;
extern volatile ImuRuntimeDebug g_imu_runtime;
#ifdef __cplusplus
}
#endif
#endif
