#ifndef LEG_TRAJECTORY_H
#define LEG_TRAJECTORY_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float x_m, y_m; /* Rear hip origin, X forward/Y down. */
    float rear_rad, front_rad;
    float dx_dphase, dy_dphase; /* Multiply by phase rate for m/s. */
} LegTrajectoryPoint;
/* Return 1 on success. Nearest equivalent angles preserve the supplied
 * assembly reference; selected candidate must match the existing MATLAB FK.
 * No actuator transmission, positioning controller or automatic activation.
 */
int leg_inverse(float x_m, float y_m, float rear_reference_rad,
                float front_reference_rad, float angles_rad[2]);
/* phase = elapsed_seconds / seconds_per_edge. Repeats every four edges.
 * Quintic interpolation: zero velocity/acceleration at each corner.
 * Invalid/unreachable samples return 0 and clear the result.
 */
int leg_square_sample(float center_x_m, float center_y_m, float side_m,
                      float phase, float rear_reference_rad,
                      float front_reference_rad, LegTrajectoryPoint *out);
#ifdef __cplusplus
}
#endif
#endif
