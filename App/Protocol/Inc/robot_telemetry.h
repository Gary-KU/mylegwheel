#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H
#include <stdint.h>
#define ROBOT_TELEMETRY_CHANNELS 24U
#define ROBOT_TELEMETRY_META_WORDS 8U
#define ROBOT_TELEMETRY_PAYLOAD_BYTES 128U
#define ROBOT_TELEMETRY_FRAME_BYTES 136U
#define ROBOT_TELEMETRY_ANO_PAIRS 25U
/* Metadata: sequence, tick_ms, validity, robot_status, startup_phase,
 * last_fault_reason, last_fault_count, output_flags (startup ready mask in
 * bits 8..12). Floats are SI units,
 * except motor commands expressed in volts; see docs/robot_telemetry.md. */
typedef struct {
    uint32_t meta[ROBOT_TELEMETRY_META_WORDS];
    float value[ROBOT_TELEMETRY_CHANNELS];
    /* Producer-only extra fields for ANO pair 14; excluded from the fixed
     * robot24 wire payload so existing version-1 decoders remain valid. */
    float swing_command_nm[2];
    float leg_angle_rate_rad_s[2]; /* ANO pairs 16/17; excluded from robot24 payload. */
    float wheel_torque_command_nm; /* Common per-wheel request after optional Nm cap, before voltage clamp; NOT measured torque. */
    uint32_t ano_pair_override; /* Producer selector snapshot; UINT32_MAX means manual ano_pair. */
} RobotTelemetrySample;
uint16_t RobotTelemetry_CRC16(const uint8_t *bytes, uint16_t length);
uint16_t RobotTelemetry_Encode(uint8_t *frame, uint16_t capacity,
                              const RobotTelemetrySample *sample);
/* Preserve existing ANO F1 four-S32 layout; only the first two are active.
 * All pair channels use display scale 0.01. Returns 0 on unavailable data. */
uint16_t RobotTelemetry_EncodeAnoPair(uint8_t *frame, uint16_t capacity,
                                    const RobotTelemetrySample *sample, uint32_t pair);
uint16_t RobotTelemetry_EncodeAnoStatus(uint8_t *frame, uint16_t capacity, uint32_t status);
#endif
