#include "wheel_leg_control.h"
#include "lqr_k.h"
#include <math.h>

int WheelLeg_Compute(const WheelLegControlInput *input, WheelLegControlOutput *output)
{
    WheelLegControlOutput result = {0};
    if (!output) return 0;
    *output = result;
    if (!input || (input->vmc_pitch_direction != 1.0f &&
                   input->vmc_pitch_direction != -1.0f)) return 0;

    for (unsigned side = 0U; side < 2U; ++side) {
        const LegModel *leg = &input->leg[side];
        float u[2], force = input->support_force_n[side];
        if (!isfinite(force) || !isfinite(leg->angle) ||
            !lqr_control(leg->length, input->state[side], input->reference[side], u)) return 0;
        result.wheel_torque_nm[side] = u[0];
        result.pitch_torque_nm[side] = u[1];
        for (unsigned joint = 0U; joint < 2U; ++joint) {
            float j_length = leg->jacobian[joint];
            float j_angle = leg->jacobian[joint + 2U];
            float torque = j_length * force + j_angle * u[1] * input->vmc_pitch_direction;
            if (!isfinite(j_length) || !isfinite(j_angle) || !isfinite(torque)) return 0;
            result.joint_torque_nm[side][joint] = torque;
        }
    }
    *output = result;
    return 1;
}
