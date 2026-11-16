#ifndef CAN_TASK_H
#define CAN_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CAN_COUNT 8U
#define MOTOR_CAN_TIMEOUT_MS 100U

#define MOTOR_FAST_HISTORY_SAMPLES 3U
#define MOTOR_FEEDBACK_HEALTH_VERSION 1U
/* Original GetFeedback snapshot, kept outside MotorFeedback so observation
 * and control stacks do not carry this audit envelope for every sample. */
enum {
    MOTOR_FEEDBACK_REJECT_EPOCH = 1U << 0,
    MOTOR_FEEDBACK_REJECT_STATUS_MISSING = 1U << 1,
    MOTOR_FEEDBACK_REJECT_STATUS_STALE = 1U << 2,
    MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_MISSING = 1U << 3,
    MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_STALE = 1U << 4,
    MOTOR_FEEDBACK_REJECT_STATUS_SENSOR = 1U << 5,
    MOTOR_FEEDBACK_REJECT_FAULTS = 1U << 6,
    MOTOR_FEEDBACK_REJECT_BOARD_FAULTS = 1U << 7,
    MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_HEALTH = 1U << 8,
    MOTOR_FEEDBACK_REJECT_FAST_INVALID = 1U << 9,
    MOTOR_FEEDBACK_REJECT_FAST_CLIPPED = 1U << 10,
    MOTOR_FEEDBACK_REJECT_FAST_MISSING = 1U << 11,
    MOTOR_FEEDBACK_REJECT_FAST_STALE = 1U << 12,
    MOTOR_FEEDBACK_REJECT_ANCHOR = 1U << 13,
    MOTOR_FEEDBACK_REJECT_TIMING = 1U << 14,
    MOTOR_FEEDBACK_REJECT_CONFIGURATION = 1U << 15,
    MOTOR_FEEDBACK_REJECT_COMMUNICATION = 1U << 16
};
typedef struct {
    uint32_t valid, motor_id, sample_tick_ms, reject_mask;
    uint32_t node_epoch, transport_epoch;
    uint32_t status_ms, status_count, status_age_ms;
    uint32_t diagnostic_ms, diagnostic_count, diagnostic_age_ms;
    uint32_t fast_ms, fast_count, fast_age_ms;
    uint32_t flags, diagnostic_flags, faults, board_faults, mode;
    uint32_t fast_flags, fast_valid, fast_speed_clipped;
    uint32_t anchor_confirmed, timing_confirmed, history_configuration_valid;
    uint32_t history_generation, history_count, history_epoch;
    uint32_t communication_blocked, recovery_state;
} MotorFeedbackHealth;

typedef struct {
    uint32_t last_rx_ms, rx_count; /* Original FAST dequeue timestamp/count. */
    float angle_rad, speed_rad_s; /* Unwrapped encoder coordinate, before calibration. */
} MotorFastSample;

typedef struct {
    uint32_t motor_id;
    int32_t raw_angle_mrad;
    int32_t raw_speed_rpm_x10;
    float angle_rad;     /* Raw encoder angle, before zero/direction/ratio. */
    float angle_deg;
    float speed_rpm;
    float speed_rad_s;
    uint32_t rx_count;
    uint32_t last_rx_ms; /* HAL millisecond tick when FIFO frame was read. */
    uint32_t age_ms;
    uint8_t online;
    uint8_t protocol_a1;
    uint32_t status_age_ms, diagnostic_age_ms, angle_age_ms;
    float joint_angle_rad;     /* Calibrated model angle; valid only if online. */
    float joint_angle_deg;
    float joint_speed_rad_s;
    uint32_t joint_calibrated; /* Transform exists, NOT feedback/control validity. */
    /* A1 joints only: accepted RX samples from the same transport/unwrap and
     * anchor configuration as the latest healthy frame. Newest first. This
     * envelope never replaces latest feedback or extends any timestamp. */
    uint32_t feedback_epoch, fast_history_generation, fast_history_count;
    MotorFastSample fast_history[MOTOR_FAST_HISTORY_SAMPLES];
} MotorFeedback;

typedef struct {
    uint32_t identifier;
    uint32_t id_type;
    uint32_t frame_type;
    uint32_t fd_format;
    uint32_t length;
    uint32_t received_ms;
    uint8_t data[8];
} MotorCanRawFrame;

typedef struct {
    MotorFeedback motor[MOTOR_CAN_COUNT]; /* [0]=ID1, ..., [7]=ID8. */
    MotorCanRawFrame last_frame;
    uint32_t received_frames;
    uint32_t decoded_frames;
    uint32_t unknown_frames;
    uint32_t malformed_frames;
    uint32_t rx_irq_count;
    uint32_t last_irq_ms;
    uint32_t fifo_full_count;
    uint32_t fifo_lost_count;
    uint32_t rx_read_failures;
    uint32_t error_irq_count;
    uint32_t last_error_events;
    uint32_t hal_error_count;
    uint32_t last_hal_error;
    uint32_t bus_off;
    uint32_t error_passive;
    uint32_t error_warning;
    uint32_t last_error_code;
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;
    uint32_t task_iterations;
    uint32_t started;
    /* 0=normal, 1=quiet/stopped wait, 2=hardware recovery, 3=HAL failure.
     * Communication recovery never restores an application enable request. */
    uint32_t recovery_state;
    uint32_t recovery_count;
    uint32_t recovery_attempts;
    uint32_t recovery_failures;
    uint32_t recovery_tx_cancel_timeout;
    uint32_t recovery_discarded_frames;
    uint32_t recovery_started_ms;
    uint32_t recovery_completed_ms;
} MotorCanDebug;

/* Global Keil Watch object. Observation only; multi-field live reads need
 * not be coherent. Applications use MotorCAN_GetFeedback() in task context.
 */
extern volatile MotorCanDebug g_motor_can;

/* Call after MX_FDCAN1_Init(), before creating tasks. */
void CAN_Task_Init(void);
/* Receive-only task: no voltage commands, enabling or CAN ID changes. */
void CAN_Task_Create(void);
uint8_t MotorCAN_GetFeedback(uint8_t motor_id, MotorFeedback *feedback);
/* Optional audit is filled from the exact snapshot used for this result. */
uint8_t MotorCAN_GetFeedbackWithHealth(uint8_t motor_id, MotorFeedback *feedback,
                                     MotorFeedbackHealth *health);
/* Read inside the bounded-TX critical section. Bus-off blocks new targets
 * immediately, including while HAL Stop/Start changes handle state. */
uint32_t CAN_CommunicationBlocked(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TASK_H */
