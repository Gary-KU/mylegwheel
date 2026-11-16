#ifndef SBUS_STICKS_H
#define SBUS_STICKS_H

#include "sbus_decoder.h"

#define SBUS_STICK_COUNT 4U
#define SBUS_STICK_DEADBAND 30U
#define SBUS_CENTER_WINDOW_MS 600U
#define SBUS_CENTER_MIN_SAMPLES 50U

typedef struct {
    uint16_t minimum;
    uint16_t center;
    uint16_t maximum;
    uint16_t deadband;
    int8_t direction; /* +1 or -1. */
} SbusAxisCalibration;

typedef struct {
    float yaw;       /* CH1: right positive. */
    float forward;   /* CH2: forward positive. */
    float height;    /* CH3: raise positive. */
    float auxiliary; /* CH4: right positive, currently unused. */
    uint32_t received_ms;
    uint32_t frame_count;
    uint8_t valid;
    uint8_t calibrated; /* Required motion axes have measured endpoints/center. */
    uint16_t raw_ch2; /* Same coherent frame as forward; UINT16_MAX when invalid. */
} SbusSticks;

enum {
    SBUS_CAL_IDLE = 0,
    SBUS_CAL_RANGE = 1,
    SBUS_CAL_CENTER = 2,
    SBUS_CAL_DONE = 3,
    SBUS_CAL_FAILED = 4
};

typedef struct {
    uint32_t stage;
    uint32_t samples;
    uint32_t center_samples;
    uint32_t center_started_ms;
    uint32_t last_sample_ms;
    uint32_t center_sum[SBUS_STICK_COUNT];
    uint16_t observed_min[SBUS_STICK_COUNT];
    uint16_t observed_max[SBUS_STICK_COUNT];
    uint16_t center_min[SBUS_STICK_COUNT];
    uint16_t center_max[SBUS_STICK_COUNT];
    uint16_t measured_center[SBUS_STICK_COUNT];
    uint8_t calibrated;
    uint8_t required_axes; /* 0: all four (legacy API); otherwise bit mask. */
} SbusStickCalibration;

uint8_t SBUS_AxisCalibrationValid(const SbusAxisCalibration *calibration);
float SBUS_NormalizeAxis(uint16_t raw, const SbusAxisCalibration *calibration);
/* Explicit commands: 1=begin ranges, 2=finish ranges and collect neutral,
 * 3=cancel. Feed only fresh frames without failsafe/frame_lost.
 * Direction/deadband are retained on successful calibration.
 */
void SBUS_StickCalibrationFeed(SbusStickCalibration *state,
    SbusAxisCalibration calibration[SBUS_STICK_COUNT], const SbusFrame *frame,
    uint32_t received_ms, uint8_t command);

#endif /* SBUS_STICKS_H */
