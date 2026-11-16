#ifndef LEG_HEIGHT_CONTROL_H
#define LEG_HEIGHT_CONTROL_H

#include <math.h>

/* Speed-loop integral is a force bias, in N. Its Ki has units N/m because
 * the integrated speed error is a distance. The caller owns reset on STOP,
 * the proportional term, force/current limits and mixed J^T anti-windup. */
static inline float leg_height_integral_candidate(
    float previous_n, float speed_error_m_s, float ki_n_per_m, float dt_s,
    float limit_n, float rate_n_s, unsigned learn_ready)
{
    if (!isfinite(previous_n) || !isfinite(speed_error_m_s) ||
        !isfinite(ki_n_per_m) || !isfinite(dt_s) || !isfinite(limit_n) ||
        !isfinite(rate_n_s) || ki_n_per_m < 0.0f || dt_s <= 0.0f ||
        limit_n < 0.0f || rate_n_s < 0.0f)
        return NAN;
    if (ki_n_per_m == 0.0f || limit_n == 0.0f) return 0.0f;

    float bounded = fmaxf(-limit_n, fminf(limit_n, previous_n));
    /* A temporarily unavailable support ramp freezes an already learned
     * bias. It must not silently erase the equilibrium compensation. */
    if (!learn_ready) return bounded;
    float rate = ki_n_per_m * speed_error_m_s;
    if (!isfinite(rate)) return NAN;
    rate = fmaxf(-rate_n_s, fminf(rate_n_s, rate));
    return fmaxf(-limit_n, fminf(limit_n, bounded + rate * dt_s));
}

/* Conditional integration against the present axial limits. Evaluate both
 * requests using the same P/feedforward base so a changing base cannot be
 * mistaken for integral unwinding. Current/pitch allocation needs the same
 * comparison on the predicted mixed joint torques in the caller. */
static inline float leg_height_integral_accept(
    float previous_n, float candidate_n, float base_force_n,
    float floor_n, float ceiling_n)
{
    if (!isfinite(previous_n) || !isfinite(candidate_n) ||
        !isfinite(base_force_n) || !isfinite(floor_n) ||
        !isfinite(ceiling_n) || floor_n > ceiling_n)
        return NAN;
    float previous_force = base_force_n + previous_n;
    float candidate_force = base_force_n + candidate_n;
    if (!isfinite(previous_force) || !isfinite(candidate_force)) return NAN;
    float previous_limited = fmaxf(floor_n, fminf(ceiling_n, previous_force));
    float candidate_limited = fmaxf(floor_n, fminf(ceiling_n, candidate_force));
    return fabsf(candidate_force-candidate_limited) <=
           fabsf(previous_force-previous_limited) ? candidate_n : previous_n;
}

#endif
