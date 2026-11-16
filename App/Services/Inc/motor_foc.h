#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H
#include <stdint.h>
#include "motor_foc_config.h"
#include "can_task.h"

typedef struct {
    float kt_nm_per_a, current_limit_a;
    int32_t direction;
    uint32_t calibration_confirmed;
    uint32_t dynamic_response_confirmed; /* Retuned/verified with actual Iq slew. */
    float minimum_iq_slew_a_per_s;
    float angle_anchor_rad; /* 已确认的机械角整圈参考，用于 A1 单圈角的初始解圈。 */
    uint32_t angle_anchor_confirmed;
    uint32_t joint_timing_confirmed; /* FAST angle/speed age and joint skew verified. */
} MotorFocCalibration;

typedef struct {
    uint32_t status_ms, diagnostic_ms, angle_ms, ack_ms;
    uint32_t status_count, diagnostic_count, angle_count, ack_count;
    uint32_t faults, board_faults, flags, diagnostic_flags, mode;
    uint32_t session_ready, limit_verified, voltage_verified, slew_verified, inhibit, timeout_count, rejected_count;
    uint32_t last_result, last_opcode, last_sequence;
    float iq_a, speed_rad_s, angle_rad, unwrapped_angle_rad;
    float remote_iq_limit_a, requested_iq_a, applied_iq_a;
    float remote_voltage_limit_v, remote_iq_slew_a_per_s;
    uint32_t epoch;
    uint32_t fast_ms, fast_count, fast_flags, fast_valid, fast_seen;
    float fast_angle_rad, fast_unwrapped_angle_rad, fast_speed_rad_s, fast_iq_a;
    uint32_t fast_speed_clipped; /* Legal int16 endpoint; telemetry retained, kinematics unusable. */
} MotorFocNode;

typedef enum {
    MOTOR_FOC_FAILURE_NONE=0, MOTOR_FOC_FAILURE_STOP_REQUEST,
    MOTOR_FOC_FAILURE_TX_DEADLINE, MOTOR_FOC_FAILURE_MAINTENANCE,
    MOTOR_FOC_FAILURE_ACTIVE_MASK, MOTOR_FOC_FAILURE_TRANSPORT_EPOCH,
    MOTOR_FOC_FAILURE_ACK_TIMEOUT, MOTOR_FOC_FAILURE_ACK_REJECTED,
    MOTOR_FOC_FAILURE_DRIVER_HEALTH, MOTOR_FOC_FAILURE_FEEDBACK,
    MOTOR_FOC_FAILURE_SESSION, MOTOR_FOC_FAILURE_CALIBRATION_LIMIT,
    MOTOR_FOC_FAILURE_INVALID_TORQUE, MOTOR_FOC_FAILURE_INVALID_CURRENT,
    MOTOR_FOC_FAILURE_TX_ENQUEUE, MOTOR_FOC_FAILURE_SPEED_CLIPPED
} MotorFocFailureReason;
/* First failure before STOP destroys session context. Retained through STOP
 * and failed restarts, cleared only by the next accepted active run or init.
 * All fields are memory snapshots; no hardware status register is read.
 * node_id=0 means a global cause, and missing ages are UINT32_MAX. */
typedef struct {
    uint32_t valid, reason, node_id, tick_ms, active_mask;
    uint32_t transport_epoch, node_epoch, session_ready, inhibit;
    uint32_t pending, opcode, sequence, pending_age_ms, got_ack;
    uint32_t last_opcode, last_sequence, last_result, ack_age_ms;
    uint32_t mode, flags, diagnostic_flags, faults, board_faults;
    uint32_t status_age_ms, diagnostic_age_ms, fast_age_ms;
    uint32_t fast_valid, fast_speed_clipped;
    uint32_t tx_count, tx_failed, timeout_count, rejected_count;
    uint32_t deadline_expired_count, deadline_invalid_buffer_count, communication_blocked;
} MotorFocFailureDebug;
extern volatile MotorFocFailureDebug g_motor_foc_failure;

/* First controller-attributed feedback rejection, captured before recovery or
 * STOP can replace node health. Passive/offline observation does not latch it.
 * New accepted active run and Init clear it; STOP and failed starts retain it.
 * schema_version=1, every field is uint32_t for an explicit debug-reader ABI. */
typedef struct {
    uint32_t valid, schema_version, controller_tick_ms, feedback_detail, observer_age_ms;
    MotorFeedbackHealth sample;
} MotorFocFeedbackFailureDebug;
extern volatile MotorFocFeedbackFailureDebug g_motor_foc_feedback_failure;
void MotorFoc_LatchFeedbackFailure(const MotorFeedbackHealth *sample,
                                  uint32_t controller_tick_ms, uint32_t feedback_detail,
                                  uint32_t observer_age_ms);

/* Read-only result of an explicit remote electrical-calibration request.
 * completed requires the real matching ACK and calibration page, both OK.
 * SAVE completion means a stored/reset-pending reply was received, not that
 * a later boot has been verified. Read again after the new STOP handshake.
 * This electrical zero is never a robot mechanical angle reference.
 */
typedef struct {
    uint32_t rx_count, received_ms, request_opcode, request_sequence;
    uint32_t ack_count, ack_ms, ack_result;
    uint32_t reply_opcode, reply_sequence, reply_result, flags;
    int32_t sensor_direction;
    float electrical_zero_rad;
    uint32_t completed;
} MotorFocStoredCalibration;

typedef struct {
    /* 数组下标统一为电机 ID - 1，与 active_mask/debug_request_mask 的位号一致。 */
    MotorFocCalibration calibration[8];
    MotorFocNode node[8];
    uint32_t transport_epoch, tx_count, tx_failed, malformed, stops;
    /* Explicit maintenance only, while all robot outputs are disabled:
     * write value/aux first, then opcode: 4=ALIGN,5=CLEAR,6=DIRECTION,7=CAL,
     * 0x15=electrical-calibration READ, 0x16=SAVE and remote reset.
     * Direction is +/-1. ALIGN uses the firmware's alignment voltage;
     * all maintenance auxiliary fields must be zero in protocol A1.
     * READ/SAVE also require value=0. SAVE emits its CAL1 guard internally.
     * No automatic alignment or fault clearing occurs on boot/recovery. */
    int32_t maintenance_value[8];
    uint32_t maintenance_aux[8], maintenance_opcode[8];
    /* Optional direct firmware-Iq pulse, independent of unknown Kt. Request
     * only while ALL robot outputs are disabled; <=200mA, <=1000ms. New
     * requests during motion or missing readiness are discarded, not queued. */
    int32_t debug_current_ma[8];
    uint32_t debug_duration_ms[8], debug_request_mask;
    /* Append commissioning fields so the existing node/calibration offsets
     * remain stable. Explicit STOP cancels debug, ALIGN and maintenance too.
     */
    uint32_t stop_request;
    MotorFocStoredCalibration stored_calibration[8];
} MotorFocDebug;

extern volatile MotorFocDebug g_motor_foc;
/* Actual successful IQ enqueue timing for the latest active session, ID-1.
 * This measures host TX acceptance, not remote execution; matching ACK and
 * FAST freshness remain separately enforced. Retained after STOP for audit. */
typedef struct {
    uint32_t count, first_ms, last_ms, max_gap_ms;
} MotorFocCommandTiming;
extern volatile MotorFocCommandTiming g_motor_foc_command_timing[8];
/* Passive A1 trace pages. Bus voltage is the driver's configured value,
 * not a separately measured supply. Page timestamps must be checked. */
typedef struct {
    uint32_t page0_ms, page1_ms, page2_ms, seen;
    float id_a, iq_reference_a, vd_v, vq_v, configured_bus_v;
} MotorFocDqDebug;
extern volatile MotorFocDqDebug g_motor_foc_dq[8];
typedef struct {
    uint32_t tick_ms, active_mask;
    float requested_iq_a[6], actual_iq_a[6], reference_iq_a[6];
    float id_a[6], vd_v[6], vq_v[6];
    uint16_t page_age_ms[6][2];
} MotorFocDqTraceRow;
#define MOTOR_FOC_DQ_TRACE_ROWS 128U
typedef struct {
    uint32_t next, count, sequence;
    MotorFocDqTraceRow rows[MOTOR_FOC_DQ_TRACE_ROWS];
} MotorFocDqTraceDebug;
/* Last5.12s at40ms, IDs1/2/3/5/6/7, frozen when robot control stops.
 * Diagnostics only: no command, ownership, readiness or limit changes. */
extern volatile MotorFocDqTraceDebug g_motor_foc_dq_trace;
void MotorFoc_Init(void);
/* RX task only. Returns one if the frame belongs to a configured A1 node. */
uint32_t MotorFoc_OnFrame(uint32_t id, const uint8_t data[8], uint32_t now);
/* ISR-safe invalidation; no CAN TX and no floating-point work. */
void MotorFoc_InvalidateFromISR(void);
void MotorFoc_OnTransportDown(void); /* task only: discard all motion/maintenance intents */
uint32_t MotorFoc_TransportMotionStopped(void); /* includes >=200ms quiet since last motion TX */
uint8_t MotorFoc_GetFeedback(uint8_t id, MotorFeedback *feedback);
uint8_t MotorFoc_GetFeedbackWithHealth(uint8_t id, MotorFeedback *feedback,
                                     MotorFeedbackHealth *health);
float MotorFoc_TorqueLimitNm(uint8_t id);
float MotorFoc_CurrentForTorque(uint8_t id,float shaft_torque_nm);
/* Fresh measured FAST Iq converted to model shaft Nm, never the requested Iq. */
uint32_t MotorFoc_MeasuredTorqueNm(uint8_t id,float *shaft_torque_nm);
/* Sole shared control task owner. Requests are shaft Nm before direction.
 * Active zero means IQ=0; inactive means driver STOP, not an enabled zero. */
uint32_t MotorFoc_Service(const float shaft_torque_nm[8], uint32_t active_mask);
void MotorFoc_StopAll(void);
#endif
