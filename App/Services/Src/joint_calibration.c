#include "joint_calibration.h"
#include "joint_calibration_config.h"

#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.2957795131f

/* Order: left front, left rear, right front, right rear.
 * Mean of 25 stationary observations. Direction comes from the separately
 * observed forward sweep, which decreases model phi in this coordinate frame.
 */
const JointCalibration g_joint_calibration[JOINT_CALIBRATION_COUNT] = {
    {JOINT_LF_CAN_ID, JOINT_LEFT_FRONT, JOINT_LF_ENCODER_REFERENCE_RAD,
     (JOINT_REFERENCE_DEG + JOINT_LF_TRIM_DEG) * DEG_TO_RAD, JOINT_LF_RATIO, JOINT_LF_DIRECTION},
    {JOINT_LR_CAN_ID, JOINT_LEFT_REAR, JOINT_LR_ENCODER_REFERENCE_RAD,
     (JOINT_REFERENCE_DEG + JOINT_LR_TRIM_DEG) * DEG_TO_RAD, JOINT_LR_RATIO, JOINT_LR_DIRECTION},
    {JOINT_RF_CAN_ID, JOINT_RIGHT_FRONT, JOINT_RF_ENCODER_REFERENCE_RAD,
     (JOINT_REFERENCE_DEG + JOINT_RF_TRIM_DEG) * DEG_TO_RAD, JOINT_RF_RATIO, JOINT_RF_DIRECTION},
    {JOINT_RR_CAN_ID, JOINT_RIGHT_REAR, JOINT_RR_ENCODER_REFERENCE_RAD,
     (JOINT_REFERENCE_DEG + JOINT_RR_TRIM_DEG) * DEG_TO_RAD, JOINT_RR_RATIO, JOINT_RR_DIRECTION}
};

void JointCalibration_Apply(MotorFeedback *feedback)
{
    uint32_t i;
    if (feedback == 0) return;
    feedback->joint_angle_rad = 0.0f;
    feedback->joint_angle_deg = 0.0f;
    feedback->joint_speed_rad_s = 0.0f;
    feedback->joint_calibrated = 0U;
    for (i = 0U; i < JOINT_CALIBRATION_COUNT; ++i) {
        const JointCalibration *cal = &g_joint_calibration[i];
        if (feedback->motor_id != cal->motor_id) continue;
        if (!(cal->ratio > 0.0f)) return;
        /* 以实测编码器参考为零差，按安装方向和传动比映射到五连杆模型角。
         * 速度沿用相同方向/传动比，角度参考偏移不参与速度换算。 */
        feedback->joint_angle_rad = cal->model_reference_rad +
            (float)cal->direction * (feedback->angle_rad - cal->encoder_reference_rad) / cal->ratio;
        feedback->joint_angle_deg = feedback->joint_angle_rad * RAD_TO_DEG;
        feedback->joint_speed_rad_s = (float)cal->direction * feedback->speed_rad_s / cal->ratio;
        feedback->joint_calibrated = 1U;
        return;
    }
}
