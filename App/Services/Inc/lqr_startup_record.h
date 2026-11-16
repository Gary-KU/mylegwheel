#ifndef LQR_STARTUP_RECORD_H
#define LQR_STARTUP_RECORD_H
#include <stdint.h>
#include "wheel_leg_task.h"
#include "wheel_leg_config.h"
#include "motor_foc_config.h"

#define LQR_RECORD_MAGIC 0x3152514cU /* LQR1 */
#define LQR_RECORD_VERSION 5U
#if defined(ROBOT_LEG_VOLTAGE_PROBE) && ROBOT_LEG_VOLTAGE_PROBE && ROBOT_LEG_PD_TEST_SIDE == 0
#define LQR_RECORD_BUILD_ID 2026100718U /* Isolated left 2V/max2s direction probe. */
#elif defined(ROBOT_LEG_VOLTAGE_PROBE) && ROBOT_LEG_VOLTAGE_PROBE && ROBOT_LEG_PD_TEST_SIDE == 1
#define LQR_RECORD_BUILD_ID 2026100719U /* Isolated right 2V/max2s direction probe. */
#elif defined(ROBOT_LEG_PD_TEST_SIDE) && ROBOT_LEG_PD_TEST_SIDE == 0
#define LQR_RECORD_BUILD_ID 2026100716U /* Stronger isolated left PD; post-stop record. */
#elif defined(ROBOT_LEG_PD_TEST_SIDE) && ROBOT_LEG_PD_TEST_SIDE == 1
#define LQR_RECORD_BUILD_ID 2026100717U /* Stronger isolated right PD; post-stop record. */
#else
#if defined(ROBOT_TELEMETRY_READ_ONLY_BUILD) && ROBOT_TELEMETRY_READ_ONLY_BUILD
#define LQR_RECORD_BUILD_ID 2026111209U /* Inhibited diagnostic image after1208 runaway report. */
#elif ROBOT_CAPTURE_BUILD
#define LQR_RECORD_BUILD_ID 2026111210U /* Supported1500ms capture;1206 control, failed integral OFF. */
#elif ROBOT_NATIVE_LQR
#define LQR_RECORD_BUILD_ID 2026111602U /* 1531 +0821 simple parking theta compensation; other gains/switches unchanged. */
#else
#define LQR_RECORD_BUILD_ID 2026110801U /* Historical compensated control. */
#endif
#endif
#define LQR_RECORD_COMMIT 0xc0dec0deU
#define LQR_RECORD_ROWS 500U
#ifndef LQR_RECORD_PERIOD_MS
#if ROBOT_CAPTURE_BUILD
#define LQR_RECORD_PERIOD_MS 5U /* 1.5s at200Hz plus2s stopped tail fits500 rows. */
#elif defined(ROBOT_LEG_PD_TEST_SIDE) && ROBOT_LEG_PD_TEST_SIDE >= 0
#define LQR_RECORD_PERIOD_MS 20U /* Preserve isolated diagnostic timing. */
#else
#define LQR_RECORD_PERIOD_MS 40U /* 20s startup capture; stopped tail retains20ms cadence. */
#endif
#endif
#define LQR_RECORD_TAIL_PERIOD_MS 20U /* Keep fall/coasting tail at 50Hz. */
#define LQR_RECORD_TAIL_MS 2000U /* Observe coasting/fall after outputs stop. */
#define LQR_RECORD_FLASH_ADDRESS 0x080c0000U
#define LQR_RECORD_FLASH_BYTES 0x00040000U
enum { LQR_RECORD_READY=0, LQR_RECORD_RUNNING, LQR_RECORD_FROZEN,
       LQR_RECORD_SAVED, LQR_RECORD_SAVE_FAILED };
enum { LQR_RECORD_STOP=1, LQR_RECORD_FALL, LQR_RECORD_FAULT, LQR_RECORD_FULL };

typedef struct {
    uint32_t magic, version, build_id, row_bytes, capacity, count;
    uint32_t reason, trigger_index, start_tick_ms, end_tick_ms, period_ms, generation;
    float wheel_scale, leg_scale, wheel_limit_mv, joint_limit_mv;
    float support_target_n, pitch_trim_deg, back_emf_ratio, fall_pitch_deg;
    float supply_model_v, joint_driver_supply_v;
    float wheel_nm_per_v[2], joint_nm_per_v[4];
    float wheel_rate_scale;
    uint32_t recorded_startup_ok;
    uint32_t crc32, commit;
} LqrRecordHeader; /* 128 bytes; commit word is programmed last. */
typedef struct {
    float parameters[40]; /* Named order is shared with the Python decoder. */
    float encoder_reference_rad[4], model_reference_rad[4], encoder_ratio[4];
    int32_t encoder_direction[4], output_direction[4];
    uint32_t joint_motor_id[4];
    /* V5 extension, node order 1/2/3/5/6/7. Prefix remains V3/V4 compatible. */
    float foc_kt_nm_per_a[6];
    int32_t foc_direction[6];
    uint32_t foc_node_id[6], current_backend_mask, motion_enable_mask;
    float foc_current_limit_a[6], minimum_iq_slew_a_per_s[6];
} LqrRecordConfiguration; /* V5: 384 bytes following unchanged 128-byte header. */
typedef struct {
    uint32_t tick_ms, flags, validity, control_status, fault_reason, fault_count;
    float state[6], reference[6], wheel_component_nm[6];
    /* Build0710+: reference[2] is the true fixed parking goal, retained in
     * the stop tail. Position-input limiting is visible in wheel_component.
     * Before parking locks, reference[2] remains the active LQR reference. */
    float wheel_requested_nm, wheel_command_nm, leg_tp_nm;
    float leg_length_m[2], leg_angle_rad[2], support_n[2];
    int16_t wheel_voltage_mv[2], joint_voltage_mv[4];
    float body_pitch_rad, body_rate_rad_s, wheel_speed_rad_s[2];
    /* V1/V2 prefix above remains 160 bytes. New fields are LF/LR/RF/RR. */
    float joint_angle_rad[4], joint_shaft_speed_rad_s[4];
    uint16_t joint_feedback_age_ms[4]; /* Saturates at 65535 ms; validity is separate. */
    float length_speed_m_s[2], length_position_speed_m_s[2], angle_speed_rad_s[2];
    float angle_pd_nm[2], sync_nm[2], lqr_pitch_nm[2], total_pitch_nm[2];
    float joint_torque_command_nm[4], angle_reference_rad[2], support_feedforward_n;
    uint32_t control_period_ms;
    float forward_target_m_s, yaw_target_rad_s, yaw_rate_rad_s, differential_applied_nm;
    float wheel_requested_per_side_nm[2], wheel_applied_per_side_nm[2];
    /* V5 extension (192 bytes), node order 1/2/3/5/6/7.
     * requested/applied are H7 cached commands; actual is fresh driver FAST Iq.
     * Invalid/stale actual current is NaN. Faults combine driver/board bits. */
    float foc_requested_iq_a[6], foc_applied_iq_a[6], foc_actual_iq_a[6], foc_current_limit_a[6];
    uint16_t foc_feedback_age_ms[6], foc_flags[6], foc_mode[6];
    uint32_t foc_faults[6];
    float support_requested_n[2], support_limit_n[2];
    uint32_t support_saturated[2], feedback_detail, feedback_motor_id, feedback_ms;
} LqrRecordRow; /* V5: 512 bytes / 16 Flash words; V4 prefix remains 320 bytes.
                 * Steering torque fields are nominal commands, not measured current.
                 * validity bits16..19 joint,
                 * bits20..21 position-derived length speed; original bits unchanged.
                 * flags bit6: sync weakened by recovery allocation (build0711+).
                 * flags bit7: normal-parking wheel damping active (build0720+).
                 * flags bit16: parking lean learning (build0821+).
                 * reference[0]: applied world-leg parking lean in radians.
                 * In normal phase its command is wheel_requested minus six LQR terms. */
typedef struct {
    LqrRecordHeader header;
    LqrRecordConfiguration configuration;
    LqrRecordRow rows[LQR_RECORD_ROWS];
} LqrRecordData;
typedef struct {
    uint32_t enable, state, count, reason, trigger_index, ram_valid;
    uint32_t auto_save, save_request, rearm_request, storage_busy;
    uint32_t flash_present, flash_error, save_count;
    float fall_pitch_deg;
    uint32_t fall_confirm_ms, tail_ms;
} LqrRecordDebug;
extern volatile LqrRecordDebug g_lqr_record;
extern volatile LqrRecordData g_lqr_record_data;
void LqrStartupRecord_Init(void);
void LqrStartupRecord_Begin(const WheelLegDebug *state);
void LqrStartupRecord_Capture(const WheelLegDebug *state);
void LqrStartupRecord_Poll(void); /* Communications task, only saves after disarm. */
uint32_t LqrStartupRecord_CRC32(uint32_t crc, const void *bytes, uint32_t size);
/* Storage returns nonzero on success. HAL errors are published in flash_error. */
int LqrRecordStorage_Load(LqrRecordHeader *header);
int LqrRecordStorage_Save(const volatile LqrRecordData *record, uint32_t *error);
#endif
