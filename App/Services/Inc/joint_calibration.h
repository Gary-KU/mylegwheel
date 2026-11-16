#ifndef JOINT_CALIBRATION_H
#define JOINT_CALIBRATION_H

#include "can_task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JOINT_CALIBRATION_COUNT 4U

typedef enum {
    JOINT_LEFT_FRONT = 0,
    JOINT_LEFT_REAR,
    JOINT_RIGHT_FRONT,
    JOINT_RIGHT_REAR
} JointRole;

typedef struct {
    uint32_t motor_id;
    JointRole role;
    float encoder_reference_rad; /* 标定姿态下的电机轴角，rad。 */
    float model_reference_rad;   /* 同一姿态对应的模型关节角，rad。 */
    float ratio;                 /* 电机轴角变化量 / 模型关节角变化量，必须为正。 */
    int32_t direction;           /* 安装方向单独表示，不并入传动比。 */
} JointCalibration;

/* User-confirmed vertical reference, direct drive; forward sweep 2026-10-01.
 * X points forward and Y downward. Rear=phi1, front=phi4 on each leg.
 * Geometric calibration only; it does not enable motor control.
 */
extern const JointCalibration g_joint_calibration[JOINT_CALIBRATION_COUNT];

/* Apply to a decoded frame before its atomic publication. No CAN writes.
 * All derived values require the MotorFeedback.online validity check.
 */
void JointCalibration_Apply(MotorFeedback *feedback);

#ifdef __cplusplus
}
#endif
#endif
