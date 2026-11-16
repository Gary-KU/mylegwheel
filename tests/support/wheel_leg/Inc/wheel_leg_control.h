#ifndef WHEEL_LEG_CONTROL_H
#define WHEEL_LEG_CONTROL_H

#include "leg_model.h"

typedef struct {
    LegModel leg[2];              /* left, right; one geometry solve per cycle */
    float state[2][6];            /* theta, dtheta, x, dx, phi, dphi in LQR coordinates */
    float reference[2][6];
    float support_force_n[2];     /* length controller output, provided by caller */
    float vmc_pitch_direction;    /* +1/-1: LQR Tp to model angle generalized torque */
} WheelLegControlInput;

typedef struct {
    float wheel_torque_nm[2];
    float pitch_torque_nm[2];
    float joint_torque_nm[2][2];   /* left/right, rear/front */
} WheelLegControlOutput;

/* Pure LQR -> VMC calculation, no RTOS/CAN, allocation or persistent state.
 * Input state and torque signs must match the fitted model. Not motor commands.
 * Returns 0 and clears all outputs if either leg/input is invalid.
 */
int WheelLeg_Compute(const WheelLegControlInput *input, WheelLegControlOutput *output);

#endif
