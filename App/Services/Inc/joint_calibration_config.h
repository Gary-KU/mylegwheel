#ifndef JOINT_CALIBRATION_CONFIG_H
#define JOINT_CALIBRATION_CONFIG_H

/* User settings. Rebuild and download after editing.
 * Encoder reference: raw angle_rad measured with the active link vertical.
 * Direction: +1/-1; ratio: encoder shaft angle / joint angle (>0).
 * Trim is added to the displayed/model angle in DEGREES. Default 0.
 * X forward, Y down; vertical model reference is 90 degrees.
 * Keep all four joint CAN IDs distinct and in the range 1..8.
 */
#define JOINT_REFERENCE_DEG             90.0f

/* Left front: Watch motor[0], model phi4. */
#define JOINT_LF_CAN_ID                 1U
#define JOINT_LF_ENCODER_REFERENCE_RAD  3.965204f
#define JOINT_LF_DIRECTION              (-1)
#define JOINT_LF_RATIO                  1.0f
#define JOINT_LF_TRIM_DEG               0.0f

/* Left rear: Watch motor[1], model phi1. */
#define JOINT_LR_CAN_ID                 2U
#define JOINT_LR_ENCODER_REFERENCE_RAD  2.450796f
#define JOINT_LR_DIRECTION              (-1)
#define JOINT_LR_RATIO                  1.0f
#define JOINT_LR_TRIM_DEG               0.0f

/* Right front: Watch motor[4], model phi4. */
#define JOINT_RF_CAN_ID                 5U
#define JOINT_RF_ENCODER_REFERENCE_RAD  1.218796f
#define JOINT_RF_DIRECTION              1
#define JOINT_RF_RATIO                  1.0f
#define JOINT_RF_TRIM_DEG               0.0f

/* Right rear: Watch motor[5], model phi1. */
#define JOINT_RR_CAN_ID                 6U
#define JOINT_RR_ENCODER_REFERENCE_RAD  2.629204f
#define JOINT_RR_DIRECTION              1
#define JOINT_RR_RATIO                  1.0f
#define JOINT_RR_TRIM_DEG               0.0f

#if (JOINT_LF_DIRECTION != 1 && JOINT_LF_DIRECTION != -1) || \
    (JOINT_LR_DIRECTION != 1 && JOINT_LR_DIRECTION != -1) || \
    (JOINT_RF_DIRECTION != 1 && JOINT_RF_DIRECTION != -1) || \
    (JOINT_RR_DIRECTION != 1 && JOINT_RR_DIRECTION != -1)
#error "Joint direction must be +1 or -1"
#endif

#if JOINT_LF_CAN_ID < 1 || JOINT_LF_CAN_ID > 8 || \
    JOINT_LR_CAN_ID < 1 || JOINT_LR_CAN_ID > 8 || \
    JOINT_RF_CAN_ID < 1 || JOINT_RF_CAN_ID > 8 || \
    JOINT_RR_CAN_ID < 1 || JOINT_RR_CAN_ID > 8
#error "Joint CAN ID must be in range 1..8"
#endif

#if JOINT_LF_CAN_ID == JOINT_LR_CAN_ID || JOINT_LF_CAN_ID == JOINT_RF_CAN_ID || \
    JOINT_LF_CAN_ID == JOINT_RR_CAN_ID || JOINT_LR_CAN_ID == JOINT_RF_CAN_ID || \
    JOINT_LR_CAN_ID == JOINT_RR_CAN_ID || JOINT_RF_CAN_ID == JOINT_RR_CAN_ID
#error "Joint CAN IDs must be distinct"
#endif

#endif
