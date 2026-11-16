from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]/'build/lmpc_thermal_20261007'
def edit(name,old,new):
    path=ROOT/name
    text=path.read_text(encoding='utf-8')
    assert old in text,(name,old)
    path.write_text(text.replace(old,new,1),encoding='utf-8')
edit('Core/Inc/stm32h7xx_hal_conf.h','/* #define HAL_ADC_MODULE_ENABLED   */','#define HAL_ADC_MODULE_ENABLED')
edit('Core/Src/main.c','#include "sbus_receiver.h"','#include "sbus_receiver.h"\n#include "cpu_temperature.h"')
edit('Core/Src/main.c','  WheelLeg_Task_Create();','  CpuTemperature_Task_Create();\n  WheelLeg_Task_Create();')
edit('App/Services/Src/motor_command.c','#include "bsp_fdcan.h"','#include "bsp_fdcan.h"\n#include "cpu_temperature.h"')
edit('App/Services/Src/motor_command.c','    FDCAN_TxHeaderTypeDef header = {0};','    if (!CpuTemperature_OutputAllowed()) voltage_mv=0;\n    FDCAN_TxHeaderTypeDef header = {0};')
edit('App/Tasks/Inc/wheel_leg_task.h','ROBOT_FAULT_STATE, ROBOT_FAULT_LMPC','ROBOT_FAULT_STATE, ROBOT_FAULT_LMPC,\n    ROBOT_FAULT_CPU_OVERTEMP, ROBOT_FAULT_CPU_TEMP_SENSOR')
edit('App/Tasks/Src/wheel_leg_task.c','#include "lmpc_shadow.h"','#include "lmpc_shadow.h"\n#include "cpu_temperature.h"')
edit('App/Tasks/Src/wheel_leg_task.c','void WheelLeg_Update(void)',r'''/* Temperature permission applies to full control, isolation tests and shadow
 * computation. Only an actual healthy CH5 OFF detent acknowledges cooling;
 * robot_control_stop clears arming, so output still requires OFF -> ON. */
static uint32_t thermal_manual_off(void)
{
    SbusState remote;
    uint32_t tolerance=g_robot_control.remote_tolerance_raw;
    if (g_robot_control.remote_enable!=1U || tolerance==0U || tolerance>100U ||
        g_robot_control.remote_center_raw>2047U || g_robot_control.remote_off_raw>2047U ||
        !SBUS_GetState(&remote)) return 0;
    uint32_t raw=remote.frame.channel[ROBOT_REMOTE_CHANNEL_INDEX];
    return raw<=2047U &&
        (abs((int32_t)raw-(int32_t)g_robot_control.remote_center_raw)<=(int32_t)tolerance ||
         abs((int32_t)raw-(int32_t)g_robot_control.remote_off_raw)<=(int32_t)tolerance);
}
static uint32_t thermal_stop(const WheelLegDebug *state)
{
    static uint32_t previous_fault;
    CpuTemperature_Tick(state->tick_ms,thermal_manual_off());
    if (CpuTemperature_OutputAllowed()) {previous_fault=0;return 0;}
    uint32_t fault=g_cpu_temperature.status;
    robot_control_stop(fault>=CPU_TEMP_OVERTEMP ? 2U : 0U);
    g_wheel_test.enable=g_wheel_test.zero_request=0;
    balance_update(state,0U);dual_hold(state,0U);(void)wheel_test_output(state);
    wheel_output_clear();joint_output_clear();
    g_balance.enable=g_balance.status=0;
    g_balance.torque_nm[0]=g_balance.torque_nm[1]=0;
    g_dual_hold.enable=g_dual_hold.status=0;
    for(unsigned i=0;i<8;++i)g_dual_hold.voltage_mv[i]=0;
    Lmpc_Reset();
    (void)MotorCommand_SendVoltages(NULL); /* Explicit zero even with no prior output owner. */
    if (fault>=CPU_TEMP_OVERTEMP && fault!=previous_fault) {
        ++g_robot_control.last_fault.count;
        g_robot_control.last_fault.tick_ms=state->tick_ms;
        g_robot_control.last_fault.reason=fault==CPU_TEMP_OVERTEMP ?
            ROBOT_FAULT_CPU_OVERTEMP : ROBOT_FAULT_CPU_TEMP_SENSOR;
    }
    previous_fault=fault;
#if LMPC_SHADOW_BUILD
    g_robot_control.feedback_only=1U;
    g_lmpc_shadow.solve_us=g_lmpc_shadow.iterations=0;
    g_lmpc_shadow.solve_status=0;
    g_lmpc_shadow.candidate_nm[0]=g_lmpc_shadow.candidate_nm[1]=0;
    ++g_lmpc_shadow.skipped;
#endif
    return 1;
}
void WheelLeg_Update(void)''')
edit('App/Tasks/Src/wheel_leg_task.c','    preview_control(&out);','    if (thermal_stop(&out)) {\n        g_wheel_leg=out;RobotTelemetry_Capture(&out);return;\n    }\n    preview_control(&out);')
edit('App/Protocol/Inc/robot_telemetry.h','ROBOT_TELEMETRY_ANO_PAIRS 26U','ROBOT_TELEMETRY_ANO_PAIRS 27U')
edit('App/Protocol/Inc/robot_telemetry.h','    uint32_t ano_pair_override;','    float cpu_temperature_c; /* ANO-only: -999 on unavailable temperature. */\n    uint32_t cpu_temperature_status;\n    uint32_t ano_pair_override;')
edit('App/Protocol/Src/robot_telemetry.c','        else if(pair==22U)','        else if(pair==26U) value=(double)(i ? (float)sample->cpu_temperature_status : sample->cpu_temperature_c)*100.0;\n        else if(pair==22U)')
edit('App/Services/Src/robot_telemetry_service.c','#include "imu_task.h"','#include "imu_task.h"\n#include "cpu_temperature.h"')
edit('App/Services/Src/robot_telemetry_service.c','sample.lmpc_build_id=10701U;','sample.lmpc_build_id=10702U;')
edit('App/Services/Src/robot_telemetry_service.c','    sample.ano_pair_override=','    taskENTER_CRITICAL();\n    sample.cpu_temperature_c=g_cpu_temperature.valid ? g_cpu_temperature.temperature_c : -999.0f;\n    sample.cpu_temperature_status=g_cpu_temperature.status;\n    taskEXIT_CRITICAL();\n    sample.ano_pair_override=')
edit('App/Services/Src/robot_telemetry_service.c','#include "queue.h"','#include "queue.h"\n#include "task.h"')
edit('App/Services/Inc/robot_telemetry_config.h','ROBOT_TELEMETRY_DEFAULT_ANO_PAIR 23U /* solve us / iterations */','ROBOT_TELEMETRY_DEFAULT_ANO_PAIR 26U /* CPU temperature C / thermal status */')
edit('App/Services/Src/board_feedback.c','#include "imu_task.h"','#include "imu_task.h"\n#include "cpu_temperature.h"')
edit('App/Services/Src/board_feedback.c','    uint32_t imu_status, imu_fresh;','    uint32_t imu_status, imu_fresh, cpu_temperature_status;')
edit('App/Services/Src/board_feedback.c','    input->remote_enable=','    input->cpu_temperature_status=g_cpu_temperature.status;\n    input->remote_enable=')
edit('App/Services/Src/board_feedback.c','    if ((input->status>=2U','    if (input->cpu_temperature_status>=CPU_TEMP_WARNING) return BOARD_FEEDBACK_FAULT;\n    if ((input->status>=2U')
# Normal sleep while idle reduces CPU busy spinning; interrupts and RTOS tick stay on.
edit('Core/Inc/FreeRTOSConfig.h','#define configUSE_IDLE_HOOK                      0','#define configUSE_IDLE_HOOK                      1')
edit('Core/Src/main.c','void vApplicationGetIdleTaskMemory(','''void vApplicationIdleHook(void)
{
    __DSB();__WFI();__ISB(); /* Normal sleep only; no stop mode or clock/tick changes. */
}

void vApplicationGetIdleTaskMemory(''')
for project in ('H723_LMPC.uvprojx','H723_LMPC_Shadow.uvprojx'):
    path=ROOT/'MDK-ARM'/project
    tree=ET.parse(path)
    groups=tree.find('./Targets/Target/Groups')
    group=ET.SubElement(groups,'Group');ET.SubElement(group,'GroupName').text='CPU temperature protection'
    files=ET.SubElement(group,'Files')
    for name in ('../App/Services/Src/cpu_temperature.c','../App/Services/Src/cpu_temperature_adc.c',
                 '../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c',
                 '../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c'):
        file=ET.SubElement(files,'File')
        ET.SubElement(file,'FileName').text=Path(name).name
        ET.SubElement(file,'FileType').text='1';ET.SubElement(file,'FilePath').text=name
    ET.indent(tree,space='  ');tree.write(path,encoding='utf-8',xml_declaration=True)
for test in ('verify_integration.py','verify_shadow.py'):
    edit('tests/'+test,"sources=['App/Tasks/Src/wheel_leg_task.c'","sources=['App/Services/Src/cpu_temperature.c','App/Tasks/Src/wheel_leg_task.c'")
edit('tests/integration_fixture.c.in','#include "wheel_leg_task.h"','#include "wheel_leg_task.h"\n#include "cpu_temperature.h"')
edit('tests/integration_fixture.c.in','static void step(void) {now+=5;WheelLeg_Update();assert(!critical);}',
'''static unsigned temperature_auto=1;
static void step(void) {now+=5;if(temperature_auto)CpuTemperature_Record(now,1,40,3300);WheelLeg_Update();assert(!critical);}''')
edit('tests/verify_telemetry.py',"('override',C.c_uint32)","('cpu_temp',C.c_float),('cpu_status',C.c_uint32),('override',C.c_uint32)")
edit('tests/verify_telemetry.py','sample.value[3]=.125;','sample.cpu_temp=42.25;sample.cpu_status=1\n\nsample.value[3]=.125;')
edit('tests/verify_telemetry.py','(25,(432100,98700))','(25,(432100,98700)),(26,(4225,100))')
edit('tests/verify_telemetry.py','C.byref(sample),26)==0','C.byref(sample),27)==0')
edit('tests/verify_telemetry.py','before=bytes(frame);sample.build=0;sample.us=0','before=bytes(frame);sample.build=0;sample.us=0;sample.cpu_temp=-999;sample.cpu_status=4')
print(ROOT)
