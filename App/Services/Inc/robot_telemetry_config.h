#ifndef ROBOT_TELEMETRY_CONFIG_H
#define ROBOT_TELEMETRY_CONFIG_H
/* Defaults persist after rebuilding/flashing; Watch edits last until reset. */
#define ROBOT_TELEMETRY_DEFAULT_ROUTE         2U /* PA10 TX/SWAP; 1 USB, 3 both */
#define ROBOT_TELEMETRY_DEFAULT_UART_BAUD     115200U /* Match the wireless module */
#define ROBOT_TELEMETRY_DEFAULT_PERIOD_MS     20U /* At 9600 baud use >=50 ms */
#if defined(ROBOT_LEG_PD_TEST_SIDE) && ROBOT_LEG_PD_TEST_SIDE >= 0
#define ROBOT_TELEMETRY_DEFAULT_ANO_PAIR      (16U + ROBOT_LEG_PD_TEST_SIDE)
#elif defined(ROBOT_TELEMETRY_READ_ONLY_BUILD)
#define ROBOT_TELEMETRY_DEFAULT_ANO_PAIR      16U /* Read-only recovery image. */
#else
#define ROBOT_TELEMETRY_DEFAULT_ANO_PAIR      22U /* Left-wheel torque command mN*m / body pitch degrees. */
#endif
#define ROBOT_TELEMETRY_DEFAULT_CH5_SELECT    1U /* Read-only mode: upper left, center right, lower IMU. */
#endif
