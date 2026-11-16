#include "motor_command.h"
#include "bsp_fdcan.h"
#if MYLEGWHEEL_A1_CURRENT
#include "motor_foc.h"
#include "bsp_fdcan_deadline.h"
#include "wheel_leg_task.h"
static uint32_t output_called;
static volatile uint32_t last_active_mask;
#endif

void MotorCommand_BeginCycle(void)
{
#if MYLEGWHEEL_A1_CURRENT
    output_called=0U;
#endif
}

void MotorCommand_EndCycle(void)
{
#if MYLEGWHEEL_A1_CURRENT && MOTOR_COMMAND_TX_ENABLED
    /* 本周期没有控制输出时仍推进 STOP/握手状态机，避免关闭控制后遗留已使能节点。 */
    if(!output_called) (void)MotorFoc_Service(NULL,0U);
#endif
}

void MotorCommand_OnTransportFault(void)
{
#if MYLEGWHEEL_A1_CURRENT
    last_active_mask=0U;
    MotorFoc_OnTransportDown();
    WheelLeg_NotifyCanTransportFault();
#endif
}

uint32_t MotorCommand_CommunicationStopped(void)
{
#if MYLEGWHEEL_A1_CURRENT
    return last_active_mask==0U && WheelLeg_CanCommunicationStopped() && MotorFoc_TransportMotionStopped();
#else
    return 0U; /* Legacy driver motion expiry is not part of the A1 contract. */
#endif
}

uint32_t MotorCommand_UsesCurrent(uint8_t id)
{
    return MYLEGWHEEL_A1_CURRENT && id>=1U && id<=8U && (MOTOR_FOC_NODE_MASK&(1U<<(id-1U)));
}

float MotorCommand_CurrentTorqueLimitNm(uint8_t id)
{
#if MYLEGWHEEL_A1_CURRENT
    return MotorFoc_TorqueLimitNm(id);
#else
    (void)id;return 0;
#endif
}

float MotorCommand_CurrentForTorque(uint8_t id,float torque)
{
#if MYLEGWHEEL_A1_CURRENT
    return MotorFoc_CurrentForTorque(id,torque);
#else
    (void)id;(void)torque;return 0;
#endif
}

uint32_t MotorCommand_MeasuredTorqueNm(uint8_t id,float *torque)
{
#if MYLEGWHEEL_A1_CURRENT
    return MotorFoc_MeasuredTorqueNm(id,torque);
#else
    (void)id;if(torque)*torque=0;return 0;
#endif
}

uint32_t MotorCommand_SendOutputs(const int16_t voltage_mv[8],const float torque_nm[8],uint32_t active_mask)
{
#if MYLEGWHEEL_A1_CURRENT && MOTOR_COMMAND_TX_ENABLED
    output_called=1U;
    int16_t legacy[8]={0};
    uint32_t good=MotorFoc_Service(torque_nm,active_mask);
    last_active_mask=good?active_mask:0U;
    if(good && voltage_mv) for(unsigned i=0;i<8;++i)
        if(!MotorCommand_UsesCurrent((uint8_t)(i+1U))) legacy[i]=voltage_mv[i];
    /* 0x77 为六个实际电机槽位；只给未切换 A1 的槽位发送旧电压组帧。 */
    uint32_t mask=0x77U&~MOTOR_FOC_NODE_MASK;
    uint32_t groups=((mask&0x0fU)?1U:0U)+((mask&0xf0U)?1U:0U);
    if(MotorCommand_SendVoltages(legacy)!=groups) {MotorFoc_StopAll();return 0U;}
    return good?2U:0U;
#else
    (void)torque_nm;(void)active_mask;
    return MotorCommand_SendVoltages(voltage_mv);
#endif
}

uint32_t MotorCommand_SendVoltages(const int16_t voltage_mv[8])
{
#if MOTOR_COMMAND_TX_ENABLED
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8];
    uint32_t group;
    uint32_t queued=0U;

    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    for (group = 0U; group < 2U; ++group) {
#if MYLEGWHEEL_A1_CURRENT
        if(!((0x77U&~MOTOR_FOC_NODE_MASK)&(0x0fU<<(group*4U)))) continue;
#endif
        header.Identifier = group == 0U ? 0x100U : 0x200U;
        /* 每组四个有符号 16 位 mV，小端排列；A1 槽位置零，防止同时接受两种输出。 */
        for (uint32_t slot = 0U; slot < 4U; ++slot) {
            unsigned id=group*4U+slot+1U;
            uint16_t value = voltage_mv && !MotorCommand_UsesCurrent((uint8_t)id) ? (uint16_t)voltage_mv[group * 4U + slot] : 0U;
            data[slot * 2U] = (uint8_t)value;
            data[slot * 2U + 1U] = (uint8_t)(value >> 8);
        }
#if MYLEGWHEEL_A1_CURRENT
        if (Fdcan_SendBounded(&header,data) != HAL_OK) break;
#else
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) != HAL_OK) break;
#endif
        ++queued;
    }
    return queued;
#else
    (void)voltage_mv;
    return 0U;
#endif
}
