#ifndef SBUS_CALIBRATION_CONFIG_H
#define SBUS_CALIBRATION_CONFIG_H
/* Measured with outputs OFF on 2026-10-08. Only CH1/CH2 drive the robot.
 * CH3/CH4 remain preview values, with no height/auxiliary actuator owner. */
#ifndef SBUS_USE_MEASURED_MOTION_PRESET
#define SBUS_USE_MEASURED_MOTION_PRESET 1
#endif
#define SBUS_MOTION_CALIBRATION_MASK 0x03U
#define SBUS_CH1_MEASURED_MIN 325U
#define SBUS_CH1_MEASURED_CENTER 1011U
#define SBUS_CH1_MEASURED_MAX 1706U
#define SBUS_CH2_MEASURED_MIN 317U
#define SBUS_CH2_MEASURED_CENTER 1007U
#define SBUS_CH2_MEASURED_MAX 1698U
/* Physical driving test: CH2 forward push decreases the channel value. */
#define SBUS_CH2_FORWARD_DIRECTION (-1)
#endif
