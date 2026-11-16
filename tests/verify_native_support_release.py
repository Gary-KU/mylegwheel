"""Historical Kp10/Ki0 support-release arithmetic with current output bounds.

Prescribed feedback verifies controller commands and existing output/stop bounds.
The historical gain override deliberately does not represent the native default.
It does not prove height convergence, physical standing stability or unloaded safety.
The coupled servo and motion governor retain production defaults. Current-limit
assertions follow the present node calibration/hard limits rather than the old
2A fixture assumption; their actual torque and encoded-current clamps are checked.
"""
import ast
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
tree = ast.parse((root / 'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
           if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
           and isinstance(n.value, ast.Constant) and isinstance(n.value.value, str)}
runtime = strings['runtime'].replace(
    'typedef struct {int unused;} FDCAN_HandleTypeDef;',
    'typedef int HAL_StatusTypeDef;\n#define HAL_ERROR 1\n'
    'typedef struct {uint32_t TXBRP,TXBCR;} FDCAN_GlobalTypeDef;\n'
    'typedef struct {FDCAN_GlobalTypeDef *Instance;} FDCAN_HandleTypeDef;')
runtime = runtime.replace('FDCAN_HandleTypeDef *,FDCAN_TxHeaderTypeDef *,uint8_t *',
                          'FDCAN_HandleTypeDef *,const FDCAN_TxHeaderTypeDef *,const uint8_t *')
runtime = runtime.replace('#endif',
    'uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *);\n#endif')
runtime = runtime.replace('#define taskEXIT_CRITICAL() (assert(critical),--critical)',
                          '#define taskEXIT_CRITICAL() do {assert(critical);--critical;} while(0)')
prefix = strings['harness'].split('int main(void) {')[0]
prefix = prefix.replace('#include "runtime.h"',
    '#include "runtime.h"\n#include "motor_foc.h"\n#include "bsp_fdcan_deadline.h"')
start = prefix.index('int HAL_FDCAN_AddMessageToTxFifoQ(')
end = prefix.index('static void step(void)', start)
prefix = prefix[:start] + r'''
/* Only the CAN peers are fake. The task, observer, VMC, current limits,
 * Nm/Iq conversion and A1 master below are unmodified production sources. */
static FDCAN_GlobalTypeDef can_hw;
static unsigned a1_mode[8],a1_flags[8],iq_commands;
static int a1_iq_ma[8];
static float a1_limits[8]={3.3f,3.3f,3.3f,3.3f,3.3f,3.3f,3.3f,3.3f};
static float a1_slew[8]={10,10,10,10,10,10,10,10};
uint32_t CAN_CommunicationBlocked(void) {return 0;}
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *h) {
 assert(h==&hfdcan1);return 1;
}
/* Shared task fixture supplies the calibration transform used by FAST history. */
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
 const FDCAN_TxHeaderTypeDef *header,const uint8_t *bytes) {
 assert(h==&hfdcan1);++calls;
 unsigned slot=header->Identifier-0x201;assert(slot<8 && bytes[0]==0xa1);
 uint8_t ack[8]={0xa1,bytes[1],bytes[2],0,0,0,0,0};
 if(bytes[1]==0) {a1_mode[slot]=0;a1_flags[slot]&=~1U;a1_iq_ma[slot]=0;}
 else if(bytes[1]==1) {
  unsigned raw=(unsigned)bytes[4]|((unsigned)bytes[5]<<8);
  a1_iq_ma[slot]=raw<32768?(int)raw:(int)raw-65536;
  float hard_limit=(slot==WHEEL_LEFT_CAN_ID-1 || slot==WHEEL_RIGHT_CAN_ID-1) ?
      MOTOR_FOC_CURRENT_HARD_LIMIT_A : MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A;
  float local_limit=g_motor_foc.calibration[slot].current_limit_a;
  assert(local_limit>0 && fabsf((float)a1_iq_ma[slot])<=
         fminf(a1_limits[slot],fminf(local_limit,hard_limit))*1000+.5f);
  a1_mode[slot]=3;a1_flags[slot]|=1;++iq_commands;
 } else if(bytes[1]==0x11) {
  assert(a1_mode[slot]==0 && !(a1_flags[slot]&1U));
  assert(bytes[3]==3 || bytes[3]==5);
  float requested;memcpy(&requested,bytes+4,4);
  if(bytes[3]==3) {
   assert(requested==MOTOR_FOC_FIXED_IQ_LIMIT_A);a1_limits[slot]=requested;
  } else {
   assert(requested==MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S);a1_slew[slot]=requested;
  }
 } else assert(bytes[1]==0x10);
 MotorFoc_OnFrame(0x381+slot,ack,now);
 if(bytes[1]==0x10 || bytes[1]==0x11) {
  uint8_t parameter[8]={0xa1,bytes[3],bytes[2],0,0,0,0,0};
  /* RAM limits start at3.3A, then track the real master's stopped write. */
  float v=bytes[3]==3?a1_limits[slot]:bytes[3]==4?5.0f:a1_slew[slot];
  memcpy(parameter+4,&v,4);MotorFoc_OnFrame(0x481+slot,parameter,now);
 }
 return HAL_OK;
}
static void feed_current_nodes(void) {
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  uint8_t f[8]={0xa1,0,0,0,0,0,0,0};
  f[1]=(uint8_t)a1_mode[i];f[2]=(uint8_t)a1_flags[i];
  uint16_t iq=(uint16_t)a1_iq_ma[i];f[4]=(uint8_t)iq;f[5]=(uint8_t)(iq>>8);
  MotorFoc_OnFrame(0x181+i,f,now);
  memset(f,0,8);f[0]=0xa1;MotorFoc_OnFrame(0x281+i,f,now);
  memset(f,0,8);f[0]=0xa1;f[1]=(uint8_t)a1_flags[i];f[2]=0xe8;f[3]=3;
  MotorFoc_OnFrame(0x701+i,f,now);
 }
}
static void current_step(void) {
 now+=5;feed_current_nodes();WheelLeg_Update();assert(!critical);
}
''' + prefix[end:]

fixture = r'''
static void pose(unsigned side,float length,float axial_speed) {
 const float angle=1.562069680534925f; /* 89.5 deg, near vertical. */
 float q[2];LegModel m;
 assert(leg_inverse(.03f+length*cosf(angle),length*sinf(angle),1.57f,1.57f,q));
 assert(leg_model(q[0],q[1],LEG_BRANCH_DOWN,&m));
 float det=m.jacobian[0]*m.jacobian[3]-m.jacobian[1]*m.jacobian[2];
 assert(fabsf(det)>1e-6f && fabsf(m.length-length)<1e-6f);
 MotorFeedback *front=&motors[g_joint_calibration[2*side].motor_id-1];
 MotorFeedback *rear=&motors[g_joint_calibration[2*side+1].motor_id-1];
 rear->joint_angle_rad=q[0];front->joint_angle_rad=q[1];
 rear->joint_speed_rad_s=m.jacobian[3]*axial_speed/det;
 front->joint_speed_rad_s=-m.jacobian[2]*axial_speed/det;
}
static void both_pose(float length,float speed) {
 for(unsigned side=0;side<2;++side)pose(side,length,speed);
}
static void output_bounds(void) {
 for(unsigned side=0;side<2;++side) {
  float axial_cap=g_dual_hold.max_force_n;
  for(unsigned joint=0;joint<2;++joint) {
   unsigned i=2*side+joint,id=g_joint_calibration[i].motor_id;
   float shaft_cap=.23f*fminf(g_motor_foc.calibration[id-1].current_limit_a,a1_limits[id-1]);
   assert(fabsf(MotorCommand_CurrentTorqueLimitNm((uint8_t)id)-shaft_cap)<1e-6f);
   float model_cap=fminf(JOINT_TORQUE_LIMIT_NM,shaft_cap*g_joint_calibration[i].ratio);
   float jac=g_wheel_leg.leg[side].jacobian[1-joint];
   if(fabsf(jac)>1e-6f)axial_cap=fminf(axial_cap,shaft_cap*g_joint_calibration[i].ratio/fabsf(jac));
   float raw=jac*g_dual_hold.force_n[side]+
       g_wheel_leg.leg[side].jacobian[3-joint]*g_dual_hold.pitch_torque_nm[side];
   float expected=fmaxf(-model_cap,fminf(model_cap,raw));
   assert(fabsf(g_dual_hold.joint_torque_nm[i]-expected)<1e-5f);
   float iq=g_motor_foc.calibration[id-1].direction*expected/
       g_joint_calibration[i].ratio/.23f;
   float current_limit=g_motor_foc.calibration[id-1].current_limit_a;
   assert(fabsf(iq)<=current_limit+1e-5f &&
          fabsf((float)a1_iq_ma[id-1])<=current_limit*1000+.5f);
   assert(fabsf((float)a1_iq_ma[id-1]*.001f-iq)<.0011f);
   assert(g_joint_output.voltage_mv[i]==0);
  }
  assert(fabsf(g_dual_hold.force_limit_n[side]-axial_cap)<1e-5f);
  assert(g_dual_hold.force_n[side]>=0 && g_dual_hold.force_n[side]<=axial_cap+1e-5f);
 }
}
static void running(void) {
 if(!g_robot_control.enable || g_robot_control.status!=1)
  fprintf(stderr,"not running: now=%u robot=%u dual=%u balance=%u reason=%u\n",
      now,g_robot_control.status,g_dual_hold.status,g_balance.status,g_robot_control.last_fault.reason);
 assert(g_robot_control.enable && g_robot_control.status==1);
 assert(g_dual_hold.status==1 && g_balance.status==1);
 output_bounds();
}
static void stopped(void) {
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_OFF);
 assert(g_dual_hold.feedforward_n==0 && g_dual_hold.overlength_blend==0);
 for(unsigned side=0;side<2;++side)
  assert(g_dual_hold.force_n[side]==0 && g_dual_hold.overlength_force_n[side]==0);
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i))
  assert(a1_mode[i]==0 && a1_iq_ma[i]==0);
}
static void watch_off(void) {
 g_robot_control.enable=0;
 /* STOP revokes the A1 readback session; let the real master rehandshake. */
 for(unsigned n=0;n<80;++n){current_step();stopped();}
}
static void arm_watch(float left,float right) {
 watch_off();pose(0,left,0);pose(1,right,0);
 g_robot_control.enable=1;
 for(unsigned n=0;n<250;++n){current_step();running();}
 assert(g_dual_hold.feedforward_n==9 && g_dual_hold.overlength_blend==1);
}
static void remote_arm(void) {
 remote_sample.frame.channel[4]=1000;
 for(unsigned n=0;n<80;++n){current_step();stopped();}
 remote_sample.frame.channel[4]=306;
 for(unsigned n=0;n<120;++n){current_step();running();}
}
int main(void) {
 assert(ROBOT_NATIVE_LQR && LEG_DEFAULT_OVERLENGTH_KP==0);
 assert(g_dual_hold.overlength_kp_n_per_m==0 && g_dual_hold.support_only==1);
 assert(g_dual_hold.target_m==.07f && g_dual_hold.max_force_n==15);
 assert(g_dual_hold.feedforward_target_n==9 && g_dual_hold.feedforward_rate_n_s==24);
 assert(g_dual_hold.speed_kp==LEG_DEFAULT_SPEED_KP &&
        g_dual_hold.speed_ki==LEG_DEFAULT_SPEED_KI && g_dual_hold.max_speed_m_s==.04f);
 /* Reproduce 1216 release commands using its historical nonintegrating
  * gains. New native defaults are covered by verify_native_height_control. */
 g_dual_hold.speed_kp=10;g_dual_hold.speed_ki=0;
 assert(MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A>0 && MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A<3.3f &&
        MOTOR_FOC_CURRENT_HARD_LIMIT_A>0 && MOTOR_FOC_CURRENT_HARD_LIMIT_A<3.3f);
 hfdcan1.Instance=&can_hw;MotorFoc_Init();
 for(unsigned i=0;i<8;++i) {
  a1_flags[i]=0x3e;motors[i].online=motors[i].joint_calibrated=1;
 }
 for(unsigned n=0;n<80;++n){now+=5;feed_current_nodes();MotorFoc_Service(NULL,0);}
 for(unsigned i=0;i<4;++i) {
  unsigned id=g_joint_calibration[i].motor_id;
  assert(g_motor_foc.calibration[id-1].current_limit_a==MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A);
  assert(fabsf(MotorCommand_CurrentTorqueLimitNm((uint8_t)id)-
               MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A*MOTOR_FOC_JOINT_KT_NM_PER_A)<1e-6f);
 }
 for(unsigned side=0;side<2;++side) {
  unsigned id=side ? WHEEL_RIGHT_CAN_ID : WHEEL_LEFT_CAN_ID;
  float limit=fminf(MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A,MOTOR_FOC_CURRENT_HARD_LIMIT_A);
  assert(g_motor_foc.calibration[id-1].current_limit_a==limit);
  assert(fabsf(MotorCommand_CurrentTorqueLimitNm((uint8_t)id)-
               limit*MOTOR_FOC_WHEEL_ESTIMATED_KT_NM_PER_A)<1e-6f);
 }
 imu=(ImuSample){0};imu_ok=1;g_robot_control.remote_enable=0;
 const float lengths[4]={.07f,.09f,.10f,.11f};
 for(unsigned trial=0;trial<4;++trial) {
  both_pose(lengths[trial],0);watch_off();g_robot_control.enable=1;
  unsigned ramp_completed=0;float speed_reference=0;
  for(unsigned n=0;n<1200;++n) {
   float previous=g_dual_hold.feedforward_n;current_step();running();
   speed_reference+=.005f/(.02f+.005f)*((trial?-.04f:0)-speed_reference);
   assert(g_dual_hold.feedforward_n>=previous &&
          g_dual_hold.feedforward_n-previous<=.12001f);
   if(g_dual_hold.feedforward_n==9)ramp_completed=1;
   for(unsigned side=0;side<2;++side) {
    assert(fabsf(g_wheel_leg.leg[side].length-lengths[trial])<1e-6f);
    assert(fabsf(g_wheel_leg.length_speed_m_s[side])<1e-6f);
    assert(g_dual_hold.overlength_force_n[side]==0);
    assert(fabsf(g_dual_hold.force_requested_n[side]-
       (g_dual_hold.feedforward_n+10*speed_reference))<1e-5f);
    if(n>=200) {
     float expected=trial?8.6f:9.0f;
     assert(fabsf(g_dual_hold.force_n[side]-expected)<.0001f);
     assert(!g_dual_hold.force_saturated[side]);
    }
   }
  }
  assert(ramp_completed && g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
  printf("PASS: historical Kp10/Ki0 static %.0fmm, overlength=0, final support %.4f/%.4f N\n",
         lengths[trial]*1000,g_dual_hold.force_n[0],g_dual_hold.force_n[1]);
 }
 /* Same measured lengths, static near-vertical feedback: reproduce only
  * the historical unloading arithmetic, not the historical body dynamics. */
 arm_watch(.10312f,.09846f);
 const float historical[2]={1.4132f,2.6481f};
 for(unsigned repeat=0;repeat<2;++repeat) {
  g_dual_hold.overlength_kp_n_per_m=265;current_step();running();
  for(unsigned side=0;side<2;++side) {
   assert(fabsf(g_dual_hold.force_requested_n[side]-historical[side])<.002f);
   assert(fabsf(g_dual_hold.force_n[side]-historical[side])<.002f);
   assert(g_dual_hold.overlength_force_n[side]<-5.9f);
   assert(!g_dual_hold.force_saturated[side]);
  }
  g_dual_hold.overlength_kp_n_per_m=0;current_step();running();
  for(unsigned side=0;side<2;++side) {
   assert(g_dual_hold.overlength_force_n[side]==0);
   assert(fabsf(g_dual_hold.force_n[side]-8.6f)<.0001f);
  }
 }
 puts("PASS: runtime kp265 reproduces 1.4132/2.6481 N; kp0 restores 8.6 N, reversibly");
 /* The support-only floor is unchanged when unloading is manually restored. */
 both_pose(.11f,0);g_dual_hold.overlength_kp_n_per_m=265;current_step();running();
 for(unsigned side=0;side<2;++side) {
  assert(g_dual_hold.force_requested_n[side]<0 && g_dual_hold.force_n[side]==0);
  assert(g_dual_hold.force_saturated[side]);
 }
 g_dual_hold.overlength_kp_n_per_m=0;
 /* Lower one real driver's local current allowance. Axial allocation must
  * still honor cap/|J| rather than pass the newly restored support unchanged. */
 unsigned limited=g_joint_calibration[0].motor_id-1;
 g_motor_foc.calibration[limited].current_limit_a=.25f;
 current_step();running();
 assert(g_dual_hold.force_requested_n[0]>8.59f && g_dual_hold.force_limit_n[0]<3);
 assert(g_dual_hold.force_n[0]==g_dual_hold.force_limit_n[0] && g_dual_hold.force_saturated[0]);
 g_motor_foc.calibration[limited].current_limit_a=MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A;
 current_step();running();assert(fabsf(g_dual_hold.force_n[0]-8.6f)<.0001f);
 /* Finite prescribed compression requests >15N; physical pose is real but
  * this rate stress is an output-bound check, not a dynamics simulation. */
 arm_watch(.07f,.07f);both_pose(.07f,-1.0f);current_step();running();
 for(unsigned side=0;side<2;++side) {
  assert(g_dual_hold.force_requested_n[side]>15);
  assert(g_dual_hold.force_limit_n[side]==15 && g_dual_hold.force_n[side]==15);
  assert(g_dual_hold.force_saturated[side]);
 }
 both_pose(.07f,0);imu.gyro_rad_s[BODY_PITCH_AXIS]=100/BODY_PITCH_DIRECTION;
 current_step();running();
 unsigned clipped=0;
 for(unsigned i=0;i<4;++i) {
  unsigned side=i/2,joint=i%2;
  float raw=g_wheel_leg.leg[side].jacobian[1-joint]*g_dual_hold.force_n[side]+
      g_wheel_leg.leg[side].jacobian[3-joint]*g_dual_hold.pitch_torque_nm[side];
  if(fabsf(raw)>fabsf(g_dual_hold.joint_torque_nm[i])+.001f)++clipped;
 }
 assert(clipped && g_joint_output.saturated);
 imu=(ImuSample){0};watch_off();
 puts("PASS: support floor, physical axial headroom, max-force and mixed-joint configured current caps");
 /* CH5 OFF, loss/reconnect, body fall and feedback faults all revoke motion.
  * Reconnect alone cannot rearm; a fresh OFF/ON sequence is required. */
 g_robot_control.remote_enable=1;remote_ok=remote_fresh=1;
 remote_sample.frame.channel[4]=306;current_step();stopped();
 remote_arm();remote_sample.frame.channel[4]=1000;current_step();stopped();
 remote_arm();remote_ok=0;current_step();stopped();
 remote_ok=1;
 for(unsigned n=0;n<20;++n){current_step();stopped();}
 remote_arm();imu.attitude_deg[BODY_PITCH_AXIS]=40/BODY_PITCH_DIRECTION;
 for(unsigned n=0;n<9;++n)current_step();
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_BODY_PITCH);stopped();
 imu=(ImuSample){0};remote_arm();
 motors[JOINT_LF_CAN_ID-1].online=0;current_step();
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);stopped();
 motors[JOINT_LF_CAN_ID-1].online=1;
 for(unsigned n=0;n<10;++n){current_step();stopped();}
 remote_arm();g_robot_control.feedback_only=1;current_step();stopped();
 g_robot_control.feedback_only=0;
 unsigned previous=iq_commands;
 for(unsigned n=0;n<50;++n){current_step();stopped();}
 assert(iq_commands==previous && MotorCommand_CommunicationStopped());
 puts("PASS: CH5 OFF/rearm, remote loss, posture, motor-feedback and feedback-only stops remain effective");
 puts("Command regression only: no height-convergence, physical-stability or unloaded-safety claim.");
}
'''

with tempfile.TemporaryDirectory(prefix='native_support_release_') as directory:
    work = Path(directory)
    (work / 'runtime.h').write_text(runtime)
    for name in ('main.h', 'FreeRTOS.h', 'task.h', 'bsp_fdcan.h'):
        (work / name).write_text('#include "runtime.h"\n')
    (work / 'test.c').write_text(prefix + fixture)
    sources = ['App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c',
               'App/Services/Src/motor_foc.c', 'App/Services/Src/motor_command.c',
               'BSP/Peripherals/Src/bsp_fdcan_deadline.c', 'App/Protocol/Src/robot_motion_command.c',
               'tests/support/wheel_leg/Src/wheel_leg_control.c', 'Control/WheelLeg/Src/lqr_k.c',
               'Control/WheelLeg/Src/leg_pos.c', 'tests/support/wheel_leg/Src/leg_trajectory.c']
    command = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
               '-DROBOT_NATIVE_LQR=1', '-DMYLEGWHEEL_A1_CURRENT=1',
               '-DROBOT_LEGS_ONLY_CALIBRATION=0', '-I', str(work)]
    for folder in ('App/Tasks/Inc', 'App/Services/Inc', 'App/Protocol/Inc', 'BSP/Peripherals/Inc',
                   'Control/WheelLeg/Inc', 'tests/support/wheel_leg/Inc'):
        command += ['-I', str(root / folder)]
    exe = work / 'test.exe'
    subprocess.run(command + [str(work / 'test.c'), *[str(root / s) for s in sources],
                              '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
