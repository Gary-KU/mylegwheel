#ifndef BALANCE_MOTION_REFERENCE_H
#define BALANCE_MOTION_REFERENCE_H

#include <math.h>
#include <stddef.h>

/* This module shapes references only. The caller retains permission, feedback,
 * current/torque and fall checks, and must Reset on OFF or permission loss.
 * A return value of zero is an invalid-input/state failure: the caller must
 * use its existing stop/fault path, rather than apply the cleared output. */
typedef struct {
    float max_speed_m_s;
    float acceleration_m_s2;
    float brake_acceleration_m_s2;
    float stop_speed_m_s;
    float stop_dwell_s;
    float tracking_window_m;
} BalanceMotionReferenceConfig;

typedef enum {
    BALANCE_MOTION_PARKED = 0,
    BALANCE_MOTION_TRACKING = 1,
    BALANCE_MOTION_BRAKING = 2
} BalanceMotionReferencePhase;

typedef struct {
    unsigned initialized;
    BalanceMotionReferencePhase phase;
    int drive_sign;
    float velocity_m_s;
    float goal_m;
    float quiet_s;
} BalanceMotionReferenceState;

typedef struct {
    float velocity_m_s;
    float goal_m;
    unsigned position_locked;
    unsigned braking;
} BalanceMotionReferenceOutput;

#define BALANCE_MOTION_REFERENCE_EPSILON_M_S 0.000001f
#define BALANCE_MOTION_REFERENCE_MAX_DT_S 0.010f

static inline void BalanceMotionReference_Reset(BalanceMotionReferenceState *s)
{
    if (s) *s = (BalanceMotionReferenceState){0};
}

static inline float BalanceMotionReference_Clamp(float x, float limit)
{
    return x > limit ? limit : x < -limit ? -limit : x;
}

static inline int BalanceMotionReference_Sign(float x)
{
    return x > BALANCE_MOTION_REFERENCE_EPSILON_M_S ? 1 :
           x < -BALANCE_MOTION_REFERENCE_EPSILON_M_S ? -1 : 0;
}

static inline int BalanceMotionReference_ValidConfig(
    const BalanceMotionReferenceConfig *c)
{
    return c && isfinite(c->max_speed_m_s) && c->max_speed_m_s > 0 &&
        c->max_speed_m_s <= 5.0f &&
        isfinite(c->acceleration_m_s2) && c->acceleration_m_s2 > 0 &&
        c->acceleration_m_s2 <= 20.0f &&
        isfinite(c->brake_acceleration_m_s2) && c->brake_acceleration_m_s2 > 0 &&
        c->brake_acceleration_m_s2 <= 20.0f &&
        isfinite(c->stop_speed_m_s) && c->stop_speed_m_s > 0 &&
        c->stop_speed_m_s < c->max_speed_m_s &&
        isfinite(c->stop_dwell_s) && c->stop_dwell_s > 0 &&
        c->stop_dwell_s <= 5.0f &&
        isfinite(c->tracking_window_m) && c->tracking_window_m > 0 &&
        c->tracking_window_m <= 5.0f;
}

static inline int BalanceMotionReference_Fail(BalanceMotionReferenceState *s,
                                             BalanceMotionReferenceOutput *out)
{
    BalanceMotionReference_Reset(s);
    if (out) *out = (BalanceMotionReferenceOutput){0};
    return 0;
}

/* requested is the accepted joystick intent BEFORE its upstream slew;
 * slewed is that upstream ramp's current value. Keeping both matters: neutral
 * intent starts braking immediately even if the upstream ramp is nonzero.
 * measured_speed is actual feedback, not either commanded velocity.
 *
 * The position goal is captured once at initialization. A fresh drive from
 * PARKED may capture the current position; TRACKING integrates the accepted
 * velocity inside its local window. Neutral/reversal freezes the existing
 * goal for the entire brake. Completing a reversal never recaptures it.
 * A subsequent TRACKING update resumes the ordinary local-window policy.
 *
 * Once braking starts, no new direction is released until the internal
 * velocity reference is zero AND measured speed remains below the stop
 * threshold for the dwell. Every update uses the latest requested intent;
 * changing one's mind cannot bypass braking or replay an old queued turn.
 * The upstream ramp may have crossed zero during the wait, so our own ramp
 * also bounds the reference when tracking resumes. */
static inline int BalanceMotionReference_Update(
    BalanceMotionReferenceState *s, const BalanceMotionReferenceConfig *c,
    float position_m, float measured_speed_m_s, float requested_m_s,
    float slewed_m_s, float dt_s, BalanceMotionReferenceOutput *out)
{
    if (!s || !out || !BalanceMotionReference_ValidConfig(c) ||
        !isfinite(position_m) || !isfinite(measured_speed_m_s) ||
        !isfinite(requested_m_s) || !isfinite(slewed_m_s) ||
        !isfinite(dt_s) || dt_s <= 0 || dt_s > BALANCE_MOTION_REFERENCE_MAX_DT_S ||
        fabsf(requested_m_s) > c->max_speed_m_s + BALANCE_MOTION_REFERENCE_EPSILON_M_S ||
        fabsf(slewed_m_s) > c->max_speed_m_s + BALANCE_MOTION_REFERENCE_EPSILON_M_S)
        return BalanceMotionReference_Fail(s, out);

    if (s->initialized > 1U ||
        (s->initialized &&
         (s->phase < BALANCE_MOTION_PARKED || s->phase > BALANCE_MOTION_BRAKING ||
          s->drive_sign < -1 || s->drive_sign > 1 ||
          !isfinite(s->velocity_m_s) || !isfinite(s->goal_m) ||
          !isfinite(s->quiet_s) || s->quiet_s < 0 ||
          fabsf(s->velocity_m_s) > c->max_speed_m_s + BALANCE_MOTION_REFERENCE_EPSILON_M_S ||
          (s->phase == BALANCE_MOTION_PARKED &&
           (s->velocity_m_s != 0 || s->drive_sign != 0)) ||
          (s->phase == BALANCE_MOTION_TRACKING &&
           (s->drive_sign == 0 || s->drive_sign * s->velocity_m_s < 0)))))
        return BalanceMotionReference_Fail(s, out);

    if (!s->initialized) {
        *s = (BalanceMotionReferenceState){
            .initialized = 1U, .phase = BALANCE_MOTION_PARKED, .goal_m = position_m
        };
    }
    requested_m_s = BalanceMotionReference_Clamp(requested_m_s, c->max_speed_m_s);
    slewed_m_s = BalanceMotionReference_Clamp(slewed_m_s, c->max_speed_m_s);
    int intent = BalanceMotionReference_Sign(requested_m_s);

    if (s->phase == BALANCE_MOTION_TRACKING &&
        (!intent || intent != s->drive_sign)) {
        s->phase = BALANCE_MOTION_BRAKING;
        s->quiet_s = 0;
    } else if (s->phase == BALANCE_MOTION_PARKED && intent) {
        s->quiet_s = 0;
        if (fabsf(measured_speed_m_s) > c->stop_speed_m_s)
            s->phase = BALANCE_MOTION_BRAKING;
        else {
            s->phase = BALANCE_MOTION_TRACKING;
            s->drive_sign = intent;
            s->goal_m = position_m;
        }
    }

    if (s->phase == BALANCE_MOTION_BRAKING) {
        s->velocity_m_s += BalanceMotionReference_Clamp(-s->velocity_m_s,
                                                       c->brake_acceleration_m_s2 * dt_s);
        if (fabsf(s->velocity_m_s) <= BALANCE_MOTION_REFERENCE_EPSILON_M_S)
            s->velocity_m_s = 0;
        if (s->velocity_m_s == 0 && fabsf(measured_speed_m_s) <= c->stop_speed_m_s)
            s->quiet_s = fminf(c->stop_dwell_s, s->quiet_s + dt_s);
        else s->quiet_s = 0;

        if (s->quiet_s + 0.000001f >= c->stop_dwell_s) {
            s->quiet_s = 0;
            s->drive_sign = intent;
            s->phase = intent ? BALANCE_MOTION_TRACKING : BALANCE_MOTION_PARKED;
            /* This tick stays at zero and keeps the frozen goal. A large
             * pending upstream ramp must not jump through on completion. */
        }
    } else if (s->phase == BALANCE_MOTION_TRACKING) {
        float target = intent * slewed_m_s > 0 ?
            intent * fminf(fabsf(slewed_m_s), fabsf(requested_m_s)) : 0;
        s->velocity_m_s += BalanceMotionReference_Clamp(target - s->velocity_m_s,
                                                       c->acceleration_m_s2 * dt_s);
        if (fabsf(s->velocity_m_s) <= BALANCE_MOTION_REFERENCE_EPSILON_M_S)
            s->velocity_m_s = 0;
        s->goal_m += s->velocity_m_s * dt_s;
        s->goal_m = position_m + BalanceMotionReference_Clamp(s->goal_m - position_m,
                                                            c->tracking_window_m);
    }

    if (!isfinite(s->velocity_m_s) || !isfinite(s->goal_m))
        return BalanceMotionReference_Fail(s, out);
    *out = (BalanceMotionReferenceOutput){
        .velocity_m_s = s->velocity_m_s,
        .goal_m = s->goal_m,
        .position_locked = s->phase != BALANCE_MOTION_TRACKING && s->velocity_m_s == 0,
        .braking = s->phase == BALANCE_MOTION_BRAKING
    };
    return 1;
}

#endif
