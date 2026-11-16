"""Run actual Nm output allocation and mixed CAN master with fake CAN nodes."""
import ast
from pathlib import Path
import re
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
tree=ast.parse((root/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings={n.targets[0].id:ast.literal_eval(n.value) for n in tree.body
         if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and isinstance(n.value,ast.Constant) and isinstance(n.value.value,str)}
runtime=strings['runtime'].replace('typedef struct {int unused;} FDCAN_HandleTypeDef;',
    'typedef int HAL_StatusTypeDef;\n#define HAL_ERROR 1\n'
    'typedef struct {uint32_t TXBRP,TXBCR;} FDCAN_GlobalTypeDef;\n'
    'typedef struct {FDCAN_GlobalTypeDef *Instance;} FDCAN_HandleTypeDef;')
runtime=runtime.replace('FDCAN_HandleTypeDef *,FDCAN_TxHeaderTypeDef *,uint8_t *',
                        'FDCAN_HandleTypeDef *,const FDCAN_TxHeaderTypeDef *,const uint8_t *')
runtime=runtime.replace('#endif','uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *);\n#endif')
runtime=runtime.replace('#define taskEXIT_CRITICAL() (assert(critical),--critical)',
                        '#define taskEXIT_CRITICAL() do {assert(critical);--critical;} while(0)')
harness=strings['harness'].split('int main(void) {')[0]
harness=harness.replace('#include "runtime.h"','#include "runtime.h"\n#define TEST_JOINT_CALIBRATION_OVERRIDE 1\n#include "motor_foc.h"\n#include "bsp_fdcan_deadline.h"')
start=harness.index('int HAL_FDCAN_AddMessageToTxFifoQ(')
end=harness.index('static void step(void)',start)
harness=harness[:start]+r'''
static unsigned a1_modes[8],a1_flags[8],a1_iq_count;
static int a1_iq_ma[8];
static float a1_limits[8],a1_slew[8];
static FDCAN_GlobalTypeDef can_hw;
uint32_t CAN_CommunicationBlocked(void) {return 0;}
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *h) {assert(h==&hfdcan1);return 1;}
void JointCalibration_Apply(MotorFeedback *f) {(void)f;}
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,const FDCAN_TxHeaderTypeDef *header,const uint8_t *bytes) {
 assert(h==&hfdcan1);++calls;
 if(calls==fail_at) return 1;
 if(header->Identifier==0x100 || header->Identifier==0x200) {memcpy(data[header->Identifier==0x200],bytes,8);return HAL_OK;}
 unsigned slot=header->Identifier-0x201;assert(slot<8 && bytes[0]==0xa1);
 uint8_t ack[8]={0xa1,bytes[1],bytes[2],0,0,0,0,0};
 if(bytes[1]==0) {a1_modes[slot]=0;a1_flags[slot]&=~1U;}
 else if(bytes[1]==1) {
  unsigned raw=(unsigned)bytes[4]|((unsigned)bytes[5]<<8);
  a1_iq_ma[slot]=raw<32768?(int)raw:(int)raw-65536;
  a1_modes[slot]=3;a1_flags[slot]|=1;++a1_iq_count;
  assert(fabsf((float)a1_iq_ma[slot]*.001f)<=a1_limits[slot]+1e-6f);
 } else if(bytes[1]==0x11) {
  assert(a1_modes[slot]==0 && !(a1_flags[slot]&1U));
  float requested;memcpy(&requested,bytes+4,4);
  assert(bytes[3]==3 || bytes[3]==5);
  if(bytes[3]==3) {
   assert(requested==3.0f);a1_limits[slot]=requested;
  } else {
   assert(requested==MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S);a1_slew[slot]=requested;
  }
 } else assert(bytes[1]==0x10);
 MotorFoc_OnFrame(0x381+slot,ack,now);
 if(bytes[1]==0x10 || bytes[1]==0x11) {
  uint8_t parameter[8]={0xa1,bytes[3],bytes[2],0,0,0,0,0};
  float v=bytes[3]==3?a1_limits[slot]:bytes[3]==4?5:a1_slew[slot];memcpy(parameter+4,&v,4);
  MotorFoc_OnFrame(0x481+slot,parameter,now);
 }
 return HAL_OK;
}
static void feed_current_nodes(void) {
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  uint8_t f[8]={0xa1,0,0,0,0,0,0,0};f[1]=(uint8_t)a1_modes[i];f[2]=(uint8_t)a1_flags[i];
  MotorFoc_OnFrame(0x181+i,f,now);
  memset(f,0,8);f[0]=0xa1;MotorFoc_OnFrame(0x281+i,f,now);
  memset(f,0,8);f[0]=0xa1;f[1]=(uint8_t)a1_flags[i];f[2]=0xe8;f[3]=3;
  MotorFoc_OnFrame(0x701+i,f,now);
 }
}
''' +harness[end:]
harness+='\n#include "'+(root/'App/Tasks/Src/wheel_leg_task.c').as_posix()+'"\n'
harness+=r'''
int main(void) {
 g_balance_bias.enable=0; /* Isolate the Nm/Iq allocation contract. */
 hfdcan1.Instance=&can_hw;MotorFoc_Init();
 /* The controller's geared calibration fixture is also the FOC anchor source. */
 for(unsigned joint=0;joint<JOINT_CALIBRATION_COUNT;++joint) {
  unsigned slot=g_joint_calibration[joint].motor_id-1U,bit=1U<<slot;
  assert(g_motor_foc.calibration[slot].angle_anchor_rad==g_joint_calibration[joint].encoder_reference_rad);
  assert(g_motor_foc.calibration[slot].angle_anchor_confirmed==
      (unsigned)((MOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK&bit)!=0));
  assert(g_motor_foc.calibration[slot].joint_timing_confirmed==
      (unsigned)((MOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK&bit)!=0));
 }
 for(unsigned i=0;i<8;++i) {
  a1_flags[i]=0x3e;g_motor_foc.calibration[i].kt_nm_per_a=i==2?.1f:i==6?.2f:.23f;
  a1_limits[i]=1;a1_slew[i]=1;
  g_motor_foc.calibration[i].calibration_confirmed=1;
  g_motor_foc.calibration[i].dynamic_response_confirmed=1;
  g_motor_foc.calibration[i].minimum_iq_slew_a_per_s=1;
 }
 for(unsigned k=0;k<60;++k) {now+=5;feed_current_nodes();MotorFoc_Service(NULL,0);}
 assert(MOTOR_FOC_FIXED_IQ_LIMIT_A==3.0f && MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A==3.0f);
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  assert(a1_limits[i]==3 && a1_slew[i]==MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S);
  assert(g_motor_foc.node[i].limit_verified && g_motor_foc.node[i].remote_iq_limit_a==3);
 }
 assert(a1_iq_count==0);
#if ROBOT_NATIVE_LQR
 {
  MotorFocDebug saved=g_motor_foc;WheelLegDebug measured={0};
  for(unsigned side=0;side<2;++side) {
   float *j=measured.leg[side].jacobian;j[0]=-.03f;j[1]=.03f;j[2]=j[3]=.5f;
   const float tau[2]={.29f,-.25f}; /*9N +0.04Nm, front/rear. */
   for(unsigned joint=0;joint<2;++joint) {
    unsigned index=2*side+joint,id=g_joint_calibration[index].motor_id;
    volatile MotorFocNode *n=&g_motor_foc.node[id-1];
    volatile MotorFocCalibration *c=&g_motor_foc.calibration[id-1];
    n->mode=3;n->fast_flags=63;n->fast_valid=1;n->fast_count=1;n->fast_ms=now;
    n->epoch=g_motor_foc.transport_epoch;n->faults=n->board_faults=0;
    n->fast_iq_a=c->direction*tau[joint]/(g_joint_calibration[index].ratio*c->kt_nm_per_a);
    n->requested_iq_a=9; /* Wrong request must never enter the measured observer. */
   }
  }
  float tp=0;assert(measured_leg_pitch(&measured,&tp) && fabsf(tp-.04f)<1e-6f);
  g_motor_foc.node[JOINT_LF_CAN_ID-1].fast_ms=now-11;
  assert(!measured_leg_pitch(&measured,&tp));
  g_motor_foc=saved;
 }
#endif
 WheelLegDebug state={0};state.wheel_valid[0]=state.wheel_valid[1]=1;
 state.wheel_speed_rad_s[0]=-20;state.wheel_speed_rad_s[1]=20;
 int16_t voltage[8]={0};float torque[8]={0};
 /* Obsolete voltage calibration must not enter the current output math. */
 g_wheel_output.torque_ratio_nm_per_v[0]=NAN;g_wheel_output.torque_ratio_nm_per_v[1]=NAN;
 g_wheel_output.supply_voltage_v=NAN;
 assert(wheel_output_pair(&state,.02f,.001f,NAN,NAN,voltage));
 assert(g_wheel_output.voltage_mv[0]==0 && g_wheel_output.voltage_mv[1]==0);
 assert(g_wheel_output.back_emf_mv[0]==0 && g_wheel_output.back_emf_mv[1]==0);
 assert(fabsf(g_wheel_output.applied_per_side_nm[0]-.021f)<1e-6f);
 assert(fabsf(g_wheel_output.applied_per_side_nm[1]-.019f)<1e-6f);
 torque[2]=g_wheel_output.applied_per_side_nm[0];torque[6]=g_wheel_output.applied_per_side_nm[1];
 now+=10;feed_current_nodes();assert(MotorCommand_SendOutputs(voltage,torque,0x44)==2);
 assert(a1_iq_ma[2]==-210 && a1_iq_ma[6]==95); /* Direction exactly once. */
 assert(wheel_output_pair(&state,1.0f,.001f,6000,1,voltage));
 assert(fabsf(g_wheel_output.applied_torque_nm-.3f)<1e-6f);
 assert(g_wheel_output.applied_differential_nm==0 && g_wheel_output.torque_saturated);
 /* A smaller local limit remains authoritative after the remote3A sync. */
 g_motor_foc.calibration[2].current_limit_a=1.5f;
 assert(wheel_output_pair(&state,1.0f,.001f,6000,1,voltage));
 assert(fabsf(g_wheel_output.applied_torque_nm-.15f)<1e-6f);
 assert(g_wheel_output.applied_differential_nm==0 && g_wheel_output.torque_saturated);
 g_motor_foc.calibration[2].current_limit_a=3.0f;
 /* Actual controller reset requires new remote/Watch enable, not old intent. */
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=g_wheel_test.enable=1;
 MotorCommand_OnTransportFault();assert(!MotorCommand_CommunicationStopped());
 assert(can_transport_fault_pending);now+=5;WheelLeg_Update();
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable && !g_wheel_test.enable);
 for(unsigned k=0;k<50;++k) {now+=5;feed_current_nodes();MotorFoc_Service(NULL,0);}
 assert(MotorCommand_CommunicationStopped());
 /* VMC joint Nm goes directly through shaft ratio and Kt, with no voltage
  * coefficient or host back-EMF feedforward entering the A1 target. */
 for(unsigned i=0;i<8;++i) {motors[i].online=1;motors[i].joint_calibrated=1;motors[i].age_ms=0;}
 axial_feedback(.07f,0);imu=(ImuSample){0};imu_ok=1;
 g_robot_control=(RobotControlDebug){0};g_balance.enable=0;now+=5;feed_current_nodes();WheelLeg_Update();
 g_dual_hold.target_m=.07f;g_dual_hold.enable=1;
 g_dual_hold.feedforward_target_n=2;g_dual_hold.feedforward_rate_n_s=1000;
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
 g_dual_hold.sync_ratio=0;g_dual_hold.load_enable=0;
 for(unsigned i=0;i<4;++i)g_dual_hold.torque_ratio_nm_per_v[i]=NAN;
 for(unsigned k=0;k<30;++k) {now+=5;feed_current_nodes();WheelLeg_Update();}
 assert(g_dual_hold.status==1);
 for(unsigned i=0;i<4;++i) {
  unsigned id=g_joint_calibration[i].motor_id;
  float physical_cap=.23f*fminf(g_motor_foc.calibration[id-1].current_limit_a,a1_limits[id-1])*
                     g_joint_calibration[i].ratio;
  assert(JOINT_TORQUE_LIMIT_NM>=.23f*3.0f);
  assert(fabsf(g_dual_hold.joint_torque_nm[i])<=fminf(JOINT_TORQUE_LIMIT_NM,physical_cap)+1e-6f);
  float command=(float)g_motor_foc.calibration[id-1].direction*
       g_dual_hold.joint_torque_nm[i]/g_joint_calibration[i].ratio/.23f*1000;
  int expected=(int)(command+(command>=0?.5f:-.5f));
  assert(g_joint_output.voltage_mv[i]==0 && g_joint_output.back_emf_mv[i]==0);
  assert(abs(a1_iq_ma[id-1]-expected)<=1);
 }
 assert(fabsf(g_dual_hold.joint_torque_nm[0])>0);
 puts("PASS: actual wheel/VMC joint Nm allocation bypasses Nm/V+EMF, signed Nm/Kt CAN, physical current headroom, global enable revocation and quiet recovery gate");
}
'''
with tempfile.TemporaryDirectory(prefix='foc_control_') as directory:
    work=Path(directory);(work/'runtime.h').write_text(runtime)
    for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):
        (work/name).write_text('#include "runtime.h"\n')
    (work/'test.c').write_text(harness)
    sources=['App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_foc.c','App/Services/Src/motor_command.c',
             'BSP/Peripherals/Src/bsp_fdcan_deadline.c','App/Protocol/Src/robot_motion_command.c',
             'tests/support/wheel_leg/Src/wheel_leg_control.c','Control/WheelLeg/Src/lqr_k.c',
             'Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
    includes=['App/Tasks/Inc','App/Services/Inc','App/Protocol/Inc','BSP/Peripherals/Inc',
              'Control/WheelLeg/Inc','tests/support/wheel_leg/Inc']
    command=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-DROBOT_NATIVE_LQR=0',
             '-DMYLEGWHEEL_A1_CURRENT=1','-DROBOT_LEGS_ONLY_CALIBRATION=0','-DMOTOR_FOC_OUTPUT_ENABLE_MASK=0x77U','-I',str(work)]
    for folder in includes:command+=['-I',str(root/folder)]
    exe=work/'test.exe'
    subprocess.run(command+[str(work/'test.c'),*[str(root/s) for s in sources],'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
    subprocess.run([arg for arg in command if arg!='-DROBOT_NATIVE_LQR=0']+['-DROBOT_NATIVE_LQR=1',str(work/'test.c'),*[str(root/s) for s in sources],'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
