"""Observer compatibility checks through the original six-state controller.

The observer, geometry, motion permissions, LQR and output encoding are real C.
Only sensor feedback and measured shaft torques are prescribed fixtures. This
does not identify physical COM, friction, Kt, contact, or standing stability.
Compiles BALANCE_NATIVE_SERVO_ENABLE=0 because the LQI controller and active
gravity compensation are separate owners. The motion governor retains its
current default. Active observation is explicitly selected by the fixture;
the boot-default assertion follows BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE.
"""
import ast
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT / 'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings = {node.targets[0].id: ast.literal_eval(node.value) for node in tree.body
           if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
           and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)}
prefix = strings['harness'].split('int main(void) {')[0]

fixture = r'''
#include "balance_equilibrium.h"
#include "balance_bias_observer.h"
/* Sensor fixture: a specified virtual hip moment is converted independently
 * through the current geometry into model shaft torque. It is not derived
 * from the controller command and therefore cannot self-confirm its output. */
static unsigned measured_available=1U;
static float prescribed_tp;
uint32_t MotorCommand_MeasuredTorqueNm(uint8_t id,float *torque) {
 if(!torque || !measured_available)return 0;
 for(unsigned i=0;i<4;++i)if(g_joint_calibration[i].motor_id==id) {
  unsigned side=i/2;LegModel model;
  float rear=motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad;
  float front=motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad;
  if(!leg_model(rear,front,LEG_BRANCH_DOWN,&model))return 0;
  const float *j=model.jacobian;
  float joint=i%2 ? j[0]*9+j[2]*prescribed_tp : j[1]*9+j[3]*prescribed_tp;
  *torque=joint/g_joint_calibration[i].ratio;
  return 1;
 }
 return 0;
}
static void velocity(float v) {
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-v/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=v/WHEEL_RADIUS_M;
}
static void pose(void) {
 for(unsigned side=0;side<2;++side)swing_feedback(side,.10f,1.556f,0);
}
static void output_matches_reference(void) {
 assert(g_balance.status==1 && g_robot_control.enable);
 float length=g_balance.leg_length_m,phi=g_balance.reference[4];
 float b=g_balance_bias.applied_offset_m;
 double m=LQR_BODY_MASS_PER_SIDE_KG,ml=LQR_LEG_MASS_PER_SIDE_KG;
 double h=LQR_BODY_COM_HEIGHT_M,g=LQR_GRAVITY_M_S2;
 double theta=m*(h*phi-b)/((m+ml/2)*length);
 double tp=-m*g*(h*phi-b);
 assert(fabs(g_balance_equilibrium_theta_rad-theta)<1e-7);
 assert(fabs(g_balance_equilibrium_tp_nm-tp)<1e-7);
 assert(fabsf(b-LQR_BODY_COM_HEIGHT_M*g_balance_bias.applied_phi_rad)<1e-9f);
 float gain[12];lqr_k(length,gain);
 double request[2]={0,tp};
 for(unsigned i=0;i<6;++i)for(unsigned row=0;row<2;++row)
  request[row]-=gain[2*i+row]*(g_balance.state[i]-g_balance.reference[i]);
 for(unsigned row=0;row<2;++row)
  assert(fabs(g_balance.torque_nm[row]-request[row])<1e-6);
}
static void stopped(void) {
 assert(!g_robot_control.enable && !g_balance.enable && !g_dual_hold.enable);
 assert(g_balance_bias.applied_phi_rad==0 && g_balance_bias.applied_offset_m==0);
 assert(g_balance_bias.confidence_ms==0 && !g_balance_bias.learning);
}
static void disarm(unsigned reset) {
 remote_sample.frame.channel[4]=1000;
 g_balance_bias.reset_request=reset;
 for(unsigned i=0;i<5;++i){step();stopped();}
}
static void configure(unsigned application_enable) {
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 imu=(ImuSample){0};imu_ok=1;imu.attitude_deg[BODY_PITCH_AXIS]=.5f/BODY_PITCH_DIRECTION;
 pose();velocity(0);measured_available=1;
 /* Specify a positive0.8mm effective offset at the prescribed fixed phi. */
 float phi=-.5f*.017453292519943295f;
 prescribed_tp=LQR_BODY_MASS_PER_SIDE_KG*LQR_GRAVITY_M_S2*(.0008f-LQR_BODY_COM_HEIGHT_M*phi);
 g_robot_control.remote_enable=1;remote_ok=remote_fresh=1;
 motion_sticks=(SbusSticks){.valid=1,.calibrated=1};
 g_dual_hold.target_m=.10f;g_balance.pitch_trim_deg=.5f;
 g_balance_bias.enable=application_enable;g_balance_bias.shadow_enable=1;
 disarm(1);
}
static void arm_until_balanced(void) {
 remote_sample.frame.channel[4]=306;
 unsigned saw_prepare=0,saw_ramp=0;
 unsigned retained_samples=g_balance_bias.samples;
 for(unsigned n=0;n<600;++n) {
  step();assert(g_robot_control.enable);output_matches_reference();
  /* An explicit OFF reset clears the private estimate immediately; its
   * sample-count debug mirror is republished by this first active update. */
  if(n==0)retained_samples=g_balance_bias.samples;
  if(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE)saw_prepare=1;
  if(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP)saw_ramp=1;
  if(g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE) {
   assert(!g_balance_bias.learning && g_balance_bias.samples==retained_samples);
   assert(g_balance_bias.applied_phi_rad==0);
  }
  if(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE) {
   assert(saw_prepare && saw_ramp);return;
  }
 }
 assert(0);
}
static void freeze_for(unsigned frames,unsigned kind,unsigned missing_gate) {
 float estimate=g_balance_bias.estimated_offset_m,applied=g_balance_bias.applied_phi_rad;
 unsigned samples=g_balance_bias.samples;
 for(unsigned n=0;n<frames;++n) {
  if(kind==1)g_joint_output.saturated=1; /* Prescribed previous actuator-clamp report. */
  step();assert(g_robot_control.enable);output_matches_reference();
  assert(!g_balance_bias.learning && g_balance_bias.samples==samples);
  assert(g_balance_bias.estimated_offset_m==estimate && g_balance_bias.applied_phi_rad==applied);
  assert(g_balance_bias.confidence_ms==0);
  assert(!(g_balance_bias.gate_mask&missing_gate));
 }
}
static void regain_application(void) {
 unsigned started=0;
 for(unsigned n=0;n<700;++n) {
  float previous=g_balance_bias.applied_phi_rad;step();output_matches_reference();
  assert(fabsf(g_balance_bias.applied_phi_rad-previous)<=.004363323f*.005f+1e-8f);
  assert(fabsf(g_balance_bias.applied_phi_rad)<=.034906585f+1e-7f);
  if(g_balance_bias.confidence_ms<300U)assert(!g_balance_bias.learning);
  if(g_balance_bias.confidence_ms<1500U || g_balance_bias.samples<200U)
   assert(g_balance_bias.applied_phi_rad==previous);
  if(g_balance_bias.applied_phi_rad!=previous) {
   assert(g_balance_bias.learning && g_balance_bias.confidence_ms>=1500U && g_balance_bias.samples>=200U);
   started=1;break;
  }
 }
 assert(started);
}
static void equilibrium_contract(void) {
 const float lengths[]={.04f,.071f,.10f,.15f},phis[]={-.04f,0,.03f},offsets[]={-.001f,0,.001f};
 for(unsigned l=0;l<4;++l)for(unsigned p=0;p<3;++p)for(unsigned b=0;b<3;++b) {
  float theta,tp;assert(BalanceEquilibrium_CalculateWithOffset(lengths[l],phis[p],offsets[b],&theta,&tp));
  /* Independently evaluate both generalized static-torque balances. */
  double body=LQR_BODY_MASS_PER_SIDE_KG*LQR_GRAVITY_M_S2*(LQR_BODY_COM_HEIGHT_M*phis[p]-offsets[b])+tp;
  double leg=(LQR_BODY_MASS_PER_SIDE_KG+.5*LQR_LEG_MASS_PER_SIDE_KG)*LQR_GRAVITY_M_S2*lengths[l]*theta+tp;
  assert(fabs(body)<2e-8 && fabs(leg)<2e-8);
  float old_theta,old_tp,zero_theta,zero_tp;
  assert(BalanceEquilibrium_Calculate(lengths[l],phis[p],&old_theta,&old_tp));
  assert(BalanceEquilibrium_CalculateWithOffset(lengths[l],phis[p],0,&zero_theta,&zero_tp));
  assert(old_theta==zero_theta && old_tp==zero_tp);
 }
 float t,p;assert(!BalanceEquilibrium_CalculateWithOffset(.039f,0,0,&t,&p));
 assert(!BalanceEquilibrium_CalculateWithOffset(.10f,NAN,0,&t,&p));
 assert(!BalanceEquilibrium_CalculateWithOffset(.10f,0,NAN,&t,&p));
 BalanceBiasState observer={0};float x[6]={0};
 for(unsigned n=0;n<600;++n)BalanceBias_Update(&observer,x,.1f,.01f,.005f,1);
 assert(observer.samples>200 && observer.offset_m>0 && observer.phi_reference_rad>0);
 float offset=observer.offset_m,proposal=observer.phi_reference_rad;unsigned samples=observer.samples;
 for(unsigned n=0;n<200;++n)assert(!BalanceBias_Update(&observer,x,.1f,-.01f,.005f,0));
 assert(observer.offset_m==offset && observer.phi_reference_rad==proposal && observer.samples==samples);
 puts("PASS: independent signed-offset static balances, b0 legacy equivalence, learnfalse estimate/proposal freeze");
}
int main(void) {
 equilibrium_contract();
 assert(ROBOT_NATIVE_LQR && BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE<=1U);
 assert(g_balance_bias.enable==BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE && g_balance_bias.shadow_enable==1);
 /* Explicitly opt in to active observation with the LQI owner disabled.
  * Selecting the application flag must not bypass acquisition or confidence. */
 configure(1);arm_until_balanced();
 assert(g_balance_bias.enable==1 && g_balance_bias.samples==0);
 assert(g_balance_bias.applied_phi_rad==0 && g_balance_bias.confidence_ms==0);
 measured_available=0;freeze_for(40,0,128U);measured_available=1;
 motion_sticks.valid=0;freeze_for(40,0,256U);motion_sticks.valid=1;
 regain_application();
 assert(g_balance_bias.applied_phi_rad>0 && g_balance_bias.samples>=200);
 puts("PASS: explicit active-observer startup retains PREPARE/RAMP, measured-Iq, accepted-input, quiet dwell and application confidence gates");
 configure(0);arm_until_balanced();
 float fixed_theta=0,fixed_tp=0,fixed_request[2]={0};
 for(unsigned n=0;n<1000;++n) {
  step();output_matches_reference();
  assert(g_balance_bias.applied_phi_rad==0 && g_balance_bias.applied_offset_m==0);
  if(n>=200)assert(fabsf(g_balance.reference[4]+.5f*.017453292519943295f)<1e-7f);
  if(n==400) {
   fixed_theta=g_balance_equilibrium_theta_rad;fixed_tp=g_balance_equilibrium_tp_nm;
   for(unsigned i=0;i<2;++i)fixed_request[i]=g_balance.torque_nm[i];
  }
  if(n>400)for(unsigned i=0;i<2;++i)assert(g_balance.torque_nm[i]==fixed_request[i]);
 }
 assert(g_balance_bias.samples>200 && g_balance_bias.estimated_offset_m>.0007f);
 puts("PASS: explicitly selected shadow learns prescribed measured moment while full task outputs retain fixed-trim reference");
 /* OFF preserves the estimate but removes application and quiet confidence. */
 disarm(0);arm_until_balanced();g_balance_bias.enable=1;
 regain_application();
 for(unsigned n=0;n<400;++n){step();output_matches_reference();}
 assert(g_balance_bias.applied_phi_rad>0);
 assert(fabsf(g_balance_equilibrium_theta_rad-fixed_theta)<1e-7f);
 assert(fabsf(g_balance_equilibrium_tp_nm-fixed_tp)<1e-7f); /* No duplicate offset gravity term. */
 puts("PASS: BALANCE/neutral quiet dwell and sample confidence precede bounded application; b=h*phi avoids double feedforward");
 velocity(.1f);freeze_for(30,0,8U);velocity(0);regain_application();
 imu.gyro_rad_s[BODY_PITCH_AXIS]=.2f/BODY_PITCH_DIRECTION;
 freeze_for(30,0,8U);imu.gyro_rad_s[BODY_PITCH_AXIS]=0;regain_application();
 swing_feedback(1,.10f,1.656f,0);freeze_for(30,0,16U);pose();regain_application();
 g_dual_hold.feedforward_target_n=12;g_dual_hold.feedforward_rate_n_s=.1f;
 freeze_for(30,0,32U);g_dual_hold.feedforward_target_n=9;
 g_dual_hold.feedforward_rate_n_s=24;regain_application();
 freeze_for(30,1,64U);regain_application();
 motion_sticks.valid=0;freeze_for(30,0,256U);motion_sticks.valid=1;regain_application();
 motion_sticks.forward=.2f;freeze_for(30,0,256U);motion_sticks.forward=0;
 for(unsigned n=0;n<25;++n)step();
 regain_application();
 measured_available=0;freeze_for(30,0,128U);measured_available=1;regain_application();
 puts("PASS: speed/rate, asymmetric legs, support ramp, saturation, input and measured-torque loss freeze estimate/application and require fresh dwell");
 /* Exercise rearward as well as forward effective offset, without resetting
  * the physical sensor fixture to the controller's own requested torque. */
 disarm(0);g_balance_bias.enable=0;
 prescribed_tp=LQR_BODY_MASS_PER_SIDE_KG*LQR_GRAVITY_M_S2*
  (-.0008f+LQR_BODY_COM_HEIGHT_M*.5f*.017453292519943295f);
 arm_until_balanced();
 for(unsigned n=0;n<3000;++n){step();output_matches_reference();}
 assert(g_balance_bias.estimated_offset_m<-.0007f && g_balance_bias.applied_phi_rad==0);
 g_balance_bias.enable=1;regain_application();
 assert(g_balance_bias.applied_phi_rad<0 && g_balance_bias.applied_offset_m<0);
 assert(fabsf(g_balance_equilibrium_theta_rad-fixed_theta)<1e-7f);
 assert(fabsf(g_balance_equilibrium_tp_nm-fixed_tp)<1e-7f);
 puts("PASS: signed rearward observation/application uses the same gravity-consistent reference contract");
 disarm(0);arm_until_balanced();
 assert(g_balance_bias.applied_phi_rad==0);regain_application();
 motors[JOINT_LF_CAN_ID-1].online=0;step();stopped();
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
 puts("PASS: OFF clears applied correction, rearm reacquires dwell; missing motor feedback retains physical STOP");
}
'''

with tempfile.TemporaryDirectory(prefix='native_bias_adaptation_') as directory:
    work = Path(directory)
    (work / 'runtime.h').write_text(strings['runtime'])
    for name in ('main.h', 'FreeRTOS.h', 'task.h', 'bsp_fdcan.h'):
        (work / name).write_text('#include "runtime.h"\n')
    (work / 'test.c').write_text(prefix + fixture)
    command = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
               '-DROBOT_NATIVE_LQR=1', '-DROBOT_LEGS_ONLY_CALIBRATION=0', '-DMOTOR_COMMAND_TX_ENABLED=1',
               '-DBALANCE_NATIVE_SERVO_ENABLE=0',
               '-I', str(work)]
    for folder in ('App/Tasks/Inc', 'App/Services/Inc', 'App/Protocol/Inc', 'Control/WheelLeg/Inc',
                   'tests/support/wheel_leg/Inc'):
        command += ['-I', str(ROOT / folder)]
    # Rename only the legacy unavailable-Iq stub, leaving real output code
    # intact. The test supplies explicitly prescribed sensor torques above.
    motor_object = work / 'motor_command.o'
    subprocess.run(command + ['-DMotorCommand_MeasuredTorqueNm=UnusedLegacyMeasuredTorqueNm', '-c',
                             str(ROOT / 'App/Services/Src/motor_command.c'), '-o', str(motor_object)], check=True)
    sources = ['App/Protocol/Src/robot_motion_command.c', 'App/Tasks/Src/wheel_leg_task.c',
               'App/Tasks/Src/wheel_leg_observer.c', 'tests/support/wheel_leg/Src/wheel_leg_control.c',
               'Control/WheelLeg/Src/lqr_k.c', 'Control/WheelLeg/Src/leg_pos.c',
               'tests/support/wheel_leg/Src/leg_trajectory.c']
    exe = work / 'test.exe'
    subprocess.run(command + [str(work / 'test.c'), str(motor_object),
                             *[str(ROOT / source) for source in sources], '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
