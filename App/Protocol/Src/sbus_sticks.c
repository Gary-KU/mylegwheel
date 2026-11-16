#include "sbus_sticks.h"
#include <string.h>

uint8_t SBUS_AxisCalibrationValid(const SbusAxisCalibration *c)
{
    return c != NULL && c->maximum <= 2047U && c->minimum < c->center &&
        c->center < c->maximum &&
        (uint32_t)(c->center - c->minimum) > c->deadband &&
        (uint32_t)(c->maximum - c->center) > c->deadband &&
        (c->direction == 1 || c->direction == -1);
}

float SBUS_NormalizeAxis(uint16_t raw, const SbusAxisCalibration *c)
{
    int32_t delta;
    float value;
    if (!SBUS_AxisCalibrationValid(c) || raw > 2047U) return 0.0f;
    /* 中点两侧分别缩放：兼容不对称行程，并让死区边缘连续映射到零。 */
    delta = (int32_t)raw - (int32_t)c->center;
    if (delta > (int32_t)c->deadband) {
        value = (float)(delta - (int32_t)c->deadband) /
            (float)(c->maximum - c->center - c->deadband);
    } else if (delta < -(int32_t)c->deadband) {
        value = (float)(delta + (int32_t)c->deadband) /
            (float)(c->center - c->minimum - c->deadband);
    } else {
        return 0.0f;
    }
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    return value * (float)c->direction;
}

static void SBUS_ResetCenterWindow(SbusStickCalibration *s, uint32_t now)
{
    uint32_t i;
    s->center_started_ms = now;
    s->center_samples = 0U;
    for (i = 0U; i < SBUS_STICK_COUNT; ++i) {
        s->center_sum[i] = 0U;
        s->center_min[i] = 2047U;
        s->center_max[i] = 0U;
    }
}

void SBUS_StickCalibrationFeed(SbusStickCalibration *s,
    SbusAxisCalibration calibration[SBUS_STICK_COUNT], const SbusFrame *frame,
    uint32_t now, uint8_t command)
{
    uint32_t i;
    SbusAxisCalibration candidate[SBUS_STICK_COUNT];
    uint8_t required=s->required_axes ? s->required_axes : 0x0FU;
    if (command == 3U) {
        s->stage = s->calibrated ? SBUS_CAL_DONE : SBUS_CAL_IDLE;
        return;
    }
    if (command == 1U) {
        s->stage = SBUS_CAL_RANGE;
        s->samples = 0U;
        s->center_samples = 0U;
        for (i = 0U; i < SBUS_STICK_COUNT; ++i) {
            s->observed_min[i] = 2047U;
            s->observed_max[i] = 0U;
        }
    } else if (command == 2U && s->stage == SBUS_CAL_RANGE) {
        if (s->samples < 2U) { s->stage = SBUS_CAL_FAILED; return; }
        s->stage = SBUS_CAL_CENTER;
        SBUS_ResetCenterWindow(s, now);
    }
    if (frame == NULL || frame->failsafe || frame->frame_lost) {
        if (s->stage == SBUS_CAL_CENTER) SBUS_ResetCenterWindow(s, now);
        return;
    }
    if (s->stage == SBUS_CAL_RANGE) {
        for (i = 0U; i < SBUS_STICK_COUNT; ++i) {
            uint16_t raw = frame->channel[i];
            if (raw < s->observed_min[i]) s->observed_min[i] = raw;
            if (raw > s->observed_max[i]) s->observed_max[i] = raw;
        }
        ++s->samples;
    } else if (s->stage == SBUS_CAL_CENTER) {
        if ((uint32_t)(now - s->last_sample_ms) > 100U) SBUS_ResetCenterWindow(s, now);
        /* Restart the window if any stick moved beyond normal center jitter.
         * Never continuously track center during normal driving.
         */
        for (i = 0U; i < SBUS_STICK_COUNT; ++i) {
            uint16_t raw = frame->channel[i];
            uint16_t low = raw < s->center_min[i] ? raw : s->center_min[i];
            uint16_t high = raw > s->center_max[i] ? raw : s->center_max[i];
            if ((required & (1U<<i)) && (uint32_t)(high - low) > 20U) {
                SBUS_ResetCenterWindow(s, now);
                break;
            }
        }
        for (i = 0U; i < SBUS_STICK_COUNT; ++i) {
            uint16_t raw = frame->channel[i];
            if (raw < s->center_min[i]) s->center_min[i] = raw;
            if (raw > s->center_max[i]) s->center_max[i] = raw;
            s->center_sum[i] += raw;
        }
        ++s->center_samples;
        if (s->center_samples >= SBUS_CENTER_MIN_SAMPLES &&
            (uint32_t)(now - s->center_started_ms) >= SBUS_CENTER_WINDOW_MS) {
            for (i = 0U; i < SBUS_STICK_COUNT; ++i) {
                candidate[i] = calibration[i];
                if (!(required & (1U<<i))) continue;
                candidate[i].minimum = s->observed_min[i];
                candidate[i].maximum = s->observed_max[i];
                candidate[i].center = (uint16_t)((s->center_sum[i] + s->center_samples / 2U) /
                                                s->center_samples);
                s->measured_center[i] = candidate[i].center;
                /* Require substantial travel on both sides; incomplete range
                 * sweeps or a stick held at its end must not become zero. */
                if (!SBUS_AxisCalibrationValid(&candidate[i]) ||
                    (uint32_t)(candidate[i].center - candidate[i].minimum) < 100U ||
                    (uint32_t)(candidate[i].maximum - candidate[i].center) < 100U) {
                    s->stage = SBUS_CAL_FAILED;
                    s->last_sample_ms = now;
                    return;
                }
            }
            /* 所有必需轴都验证通过后再一起生效，失败时保留原标定。 */
            memcpy(calibration, candidate, sizeof(candidate));
            s->calibrated = 1U;
            s->stage = SBUS_CAL_DONE;
        }
    }
    s->last_sample_ms = now;
}
