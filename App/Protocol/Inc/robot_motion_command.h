#ifndef ROBOT_MOTION_COMMAND_H
#define ROBOT_MOTION_COMMAND_H

#include "sbus_sticks.h"

/* Pure target builder: this module has no motor/CAN,
 * controller-reference, debugger-enable or Flash side effects. */
typedef struct {
    uint32_t enable; /* Defaults to 0; enabling only enables this target builder. */
    float max_forward_m_s, max_turn_rad_s;
    float acceleration_m_s2, turn_acceleration_rad_s2;
} RobotMotionConfig;

typedef struct {
    uint32_t remote_enabled, ch5_on, controller_on, normal_balance;
} RobotMotionPermission;

typedef enum {
    ROBOT_MOTION_OFF=0, ROBOT_MOTION_REMOTE_INVALID,
    ROBOT_MOTION_UNCALIBRATED, ROBOT_MOTION_NOT_READY,
    ROBOT_MOTION_TARGET_READY, ROBOT_MOTION_BAD_PARAMETERS
} RobotMotionStatus;

typedef struct {
    RobotMotionStatus status;
    uint32_t source_frame_count;
    float forward_axis, turn_axis; /* CH2 forward+, CH1 right+. */
    float requested_forward_m_s, requested_turn_rad_s;
    float forward_m_s, turn_rad_s; /* Slewed targets; turn is right-positive. */
} RobotMotionCommand;

extern const RobotMotionConfig g_robot_motion_defaults;
void RobotMotion_Clear(RobotMotionCommand *command);
/* Task-context integration point. Feed SBUS_GetSticks(), not its Watch mirror.
 * Caller owns command and permissions. Invalid/disabled/not-ready inputs
 * immediately clear targets; only normal valid motion uses a slew limit.
 * WheelLeg_Update integrates the targets after CH5 and neutral arming. */
void RobotMotion_Update(const RobotMotionConfig *config, const SbusSticks *sticks,
    const RobotMotionPermission *permission, uint32_t now_ms, uint32_t period_ms,
    RobotMotionCommand *command);
#endif
