#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <stdint.h>
#include "motor_foc_config.h"

/* Length-hold command TX; per-motor polarity remains explicitly configured. */
#ifndef MOTOR_COMMAND_TX_ENABLED
#define MOTOR_COMMAND_TX_ENABLED 1
#endif

/* Index 0..7 maps to motor ID 1..8; NULL sends all zeros.
 * Returns queued group frames (0..2), or 0 without sending when TX is disabled.
 * Call only from the shared control task. */
uint32_t MotorCommand_SendVoltages(const int16_t voltage_mv[8]);

/* 旧节点保留 mV 接口；A1 节点接收轴端 Nm，使用独立确认的 Nm/A 标定转换。 */
uint32_t MotorCommand_UsesCurrent(uint8_t id);
float MotorCommand_CurrentTorqueLimitNm(uint8_t id);
float MotorCommand_CurrentForTorque(uint8_t id,float shaft_torque_nm);
uint32_t MotorCommand_MeasuredTorqueNm(uint8_t id,float *shaft_torque_nm);
/* 完整周期成功固定返回 2，以兼容现有调用方；这不是 A1 实际发送帧数。
 * 未激活的 A1 节点只执行 STOP/握手，失败后不重放力矩目标。 */
uint32_t MotorCommand_SendOutputs(const int16_t voltage_mv[8],
                                 const float shaft_torque_nm[8], uint32_t active_mask);
void MotorCommand_BeginCycle(void);
void MotorCommand_EndCycle(void);
void MotorCommand_OnTransportFault(void); /* CAN task, cancels every output owner */
uint32_t MotorCommand_CommunicationStopped(void);

#endif
