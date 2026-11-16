"""Original six-state native controller compatibility regression.

Explicitly compiles BALANCE_NATIVE_SERVO_ENABLE=0 and
BALANCE_MOTION_REFERENCE_ENABLE=0 to preserve the original pure-K and
drive/release reference contracts. The default LQI and measured-stop governor
have separate regression suites. Prescribed feedback checks commands, not
physical stability.
"""
import ast
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
tree=ast.parse((root/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings={n.targets[0].id:ast.literal_eval(n.value) for n in tree.body
         if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name)
         and isinstance(n.value,ast.Constant) and isinstance(n.value.value,str)}
prefix=strings['harness'].split('int main(void) {')[0]
fixture=r'''
#include "balance_equilibrium.h"
static void equilibrium(float length,float phi,float *theta,float *tp) {
 BalanceEquilibrium_Calculate(length,phi,theta,tp);
 const float mb=LQR_BODY_MASS_PER_SIDE_KG,ml=LQR_LEG_MASS_PER_SIDE_KG;
 float expected_theta=mb*LQR_BODY_COM_HEIGHT_M/(mb*length+ml*length*.5f)*phi;
 float expected_tp=-mb*LQR_GRAVITY_M_S2*LQR_BODY_COM_HEIGHT_M*phi;
 assert(fabsf(*theta-expected_theta)<1e-7f && fabsf(*tp-expected_tp)<1e-7f);
}
static void native_check(void) {
 assert(g_balance.status==1 && g_dual_hold.status==1);
 float theta_eq,tp_ff;equilibrium(g_balance.leg_length_m,g_balance.reference[4],&theta_eq,&tp_ff);
 float K[12],expected[2]={0,tp_ff};lqr_k(g_balance.leg_length_m,K);
 for(unsigned i=0;i<6;++i)for(unsigned row=0;row<2;++row)
  expected[row]-=K[row+2*i]*(g_balance.state[i]-g_balance.reference[i]);
 for(unsigned row=0;row<2;++row)
  assert(fabsf(g_balance.torque_nm[row]-expected[row])<1e-6f);
 assert(fabsf(g_wheel_output.requested_torque_nm-
              g_balance.torque_nm[0])<1e-6f);
 assert(isfinite(g_balance.reference[0]) && fabsf(g_balance.reference[0])<1.0f &&
        g_balance.reference[4]==g_balance_bias.phi_reference_rad &&
        fabsf(g_balance.reference[4])<=BALANCE_NATIVE_PITCH_TRIM_LIMIT_DEG*.017453293f+1e-7f);
 float window=fabsf(g_balance.reference[3])>1e-6f ?
     ROBOT_MOTION_TRACK_WINDOW_M : BALANCE_NATIVE_PARKING_WINDOW_M;
 float expected_x=g_balance.state[2]+fmaxf(-window,
     fminf(window,g_balance.parking_target_m-g_balance.state[2]))-
     g_balance_parking_integral.correction_m;
 assert(fabsf(g_balance.reference[2]-expected_x)<1e-6f);
 assert(g_balance.parking_lean_rad==0 && g_balance.motion_lean_rad==0);
 assert(!g_balance.parking_lean_learning && !g_balance.motion_lean_learning);
 assert(!g_balance.motion_damping_active && !g_balance.parking_damping_active);
 assert(g_balance.motion_damping_torque_nm==0 && g_balance.startup_wheel_damping_torque_nm==0);
 for(unsigned side=0;side<2;++side) {
  float angle_error=remainderf(g_dual_hold.angle_reference_rad[side]-
                               g_wheel_leg.leg[side].angle,6.28318530718f);
  float hold=g_dual_hold.angle_kp_nm_per_rad*angle_error-
             g_dual_hold.angle_kd_nm_s_per_rad*g_wheel_leg.angle_speed_rad_s[side]+
             g_leg_angle_integral_nm[side];
  if (hold>g_dual_hold.angle_limit_nm) hold=g_dual_hold.angle_limit_nm;
  if (hold<-g_dual_hold.angle_limit_nm) hold=-g_dual_hold.angle_limit_nm;
  hold*=g_balance.leg_hold_ratio;
  assert(fabsf(g_dual_hold.angle_hold_torque_nm[side]-hold)<1e-5f);
  assert(g_dual_hold.lqr_pitch_torque_nm[side]==g_balance.torque_nm[1]);
  assert(fabsf(g_dual_hold.pitch_torque_nm[side]-g_dual_hold.angle_hold_torque_nm[side]-
               g_balance.torque_nm[1]-g_dual_hold.sync_torque_nm[side])<1e-6f);
 }
 assert(!g_dual_hold.sync_priority_active && g_dual_hold.sync_priority_scale==1);
}
static void fixture_velocity(float velocity) {
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-velocity/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=velocity/WHEEL_RADIUS_M;
}
static void fixture_pose(void) {
 float q[2];assert(leg_inverse(.03f,.10f,1.57f,1.57f,q));
 for(unsigned side=0;side<2;++side) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=q[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=q[0];
 }
}
static void zeros(void) {
 for(unsigned group=0;group<2;++group)for(unsigned i=0;i<8;++i)assert(data[group][i]==0);
}
static void theta_slew_check(float previous) {
 float theta_eq,tp_ff;equilibrium(g_balance.leg_length_m,g_balance.reference[4],&theta_eq,&tp_ff);
 float delta=LEG_DEFAULT_ANGLE_REFERENCE_RATE_RAD_S*.005f;
 float expected=previous+fmaxf(-delta,fminf(delta,theta_eq-previous));
 assert(fabsf(g_balance.reference[0]-expected)<1e-6f);
}
#if MOTOR_COMMAND_TX_ENABLED
static void startup_recovery_regression(void) {
 const float rad=.017453292519943295f,half_pi=1.5707963267948966f;
 g_robot_control.enable=0;step();
 g_balance.pitch_trim_deg=.5f;g_dual_hold.target_m=.09f;
 imu=(ImuSample){0};imu.attitude_deg[BODY_PITCH_AXIS]=3.98f/BODY_PITCH_DIRECTION;
 fixture_velocity(0);
 float theta_eq,tp_ff;equilibrium(.09f,-.5f*rad,&theta_eq,&tp_ff);
 const float initial[2]={79.6f*rad,58.5f*rad},neutral=half_pi-.5f*rad+theta_eq;
 for(unsigned side=0;side<2;++side)swing_feedback(side,.09f,initial[side],0);
 g_robot_control.enable=1;step();native_check();
 float theta_reference=g_balance.reference[0],parking_goal=g_balance.parking_target_m;
 assert(fabsf(theta_reference-g_balance.state[0])<1e-7f);
 assert(fabsf(parking_goal-g_balance.state[2])<1e-7f && g_balance.position_locked);
 fixture_velocity(.12f);
 for(unsigned n=0;n<160;++n) {
  if(n==40)imu.attitude_deg[BODY_PITCH_AXIS]=6/BODY_PITCH_DIRECTION;
  step();native_check();
  assert(g_robot_control.enable && g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  assert(g_robot_control.startup_blend==0);
  theta_slew_check(theta_reference);theta_reference=g_balance.reference[0];
  assert(fabsf(g_balance.parking_target_m-parking_goal)<1e-7f && g_balance.position_locked);
  assert(g_dual_hold.angle_hold_torque_nm[0]==0 && g_dual_hold.angle_hold_torque_nm[1]==0);
 }
 assert(fabsf(g_balance.reference[0]-g_balance.state[0])>.10f);
 assert(g_balance.parking_error_m>.09f);
 assert(fabsf(g_balance.reference[2]-parking_goal)<1e-6f);
 assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_POSE));
 assert(g_dual_hold.angle_reference_rad[1]<neutral-.2f);
 imu.attitude_deg[BODY_PITCH_AXIS]=3.98f/BODY_PITCH_DIRECTION;fixture_velocity(0);
 /* Prescribed physical recovery over1s: references and real pose are distinct.
  * Handover cannot depend on pretending that a slewed target is achieved. */
 for(unsigned n=1;n<=200;++n) {
  for(unsigned side=0;side<2;++side) {
   float rate=neutral-initial[side];
   swing_feedback(side,.09f,initial[side]+(neutral-initial[side])*(float)n/200,rate);
  }
  step();native_check();assert(g_robot_control.enable);
  theta_slew_check(theta_reference);theta_reference=g_balance.reference[0];
  assert(fabsf(g_balance.parking_target_m-parking_goal)<1e-7f);
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
 }
 assert(g_robot_control.startup_ready_mask&ROBOT_READY_POSE);
 assert(g_dual_hold.angle_reference_rad[1]<neutral-.1f);
 assert(g_robot_control.startup_settle_s==0); /* Pose alone cannot hand over. */
 for(unsigned side=0;side<2;++side)swing_feedback(side,.09f,neutral,0);
 unsigned entered_ramp=0,dwell_reset_checked=0;
 for(unsigned n=0;n<800 && g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE;++n) {
  step();native_check();assert(g_robot_control.enable);
  theta_slew_check(theta_reference);theta_reference=g_balance.reference[0];
  assert(fabsf(g_balance.parking_target_m-parking_goal)<1e-7f);
  if(!dwell_reset_checked && g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE &&
     g_robot_control.startup_settle_s>.10f) {
   imu.attitude_deg[BODY_PITCH_AXIS]=6/BODY_PITCH_DIRECTION;
   step();native_check();
   theta_slew_check(theta_reference);theta_reference=g_balance.reference[0];
   assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE && g_robot_control.startup_settle_s==0);
   imu.attitude_deg[BODY_PITCH_AXIS]=3.98f/BODY_PITCH_DIRECTION;
   dwell_reset_checked=1;
  }
  if(g_robot_control.startup_phase!=ROBOT_STARTUP_PREPARE) {
   entered_ramp=1;
   assert(g_robot_control.startup_ready_mask&ROBOT_READY_POSE);
   assert(g_robot_control.startup_settle_s>=ROBOT_NATIVE_PREPARE_SETTLE_S-1e-6f);
   for(unsigned side=0;side<2;++side)
    assert(fabsf(g_dual_hold.angle_reference_rad[side]-neutral)<.0011f);
  }
 }
 assert(dwell_reset_checked && entered_ramp && g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);

 /* Both signs: capture30deg once, then abruptly prescribe a ready pose.
  * Theta must ignore that measured change and slew at0.2rad/s through both
  * phase boundaries. A nonzero remainder survives entry into BALANCE. */
 for(int sign=-1;sign<=1;sign+=2) {
  g_robot_control.enable=0;step();
  imu=(ImuSample){0};imu.attitude_deg[BODY_PITCH_AXIS]=sign*30/BODY_PITCH_DIRECTION;
  for(unsigned side=0;side<2;++side)
   swing_feedback(side,.09f,half_pi,0);
  fixture_velocity(sign*.10f);
  g_robot_control.enable=1;
  float previous=0;unsigned previous_phase=ROBOT_STARTUP_OFF;
  unsigned saw_ramp=0,saw_continuous_handover=0;float first_goal=0;
  for(unsigned n=0;n<700;++n) {
   step();native_check();assert(g_robot_control.enable);
   unsigned phase=g_robot_control.startup_phase;
   float reference=g_balance.reference[0];
   if(n==0) {
    assert(phase==ROBOT_STARTUP_PREPARE && g_robot_control.startup_blend==0);
    assert(fabsf(reference-g_balance.state[0])<1e-7f);
    assert(fabsf(g_balance.torque_nm[0])>1e-4f && fabsf(g_balance.torque_nm[1])>1e-4f);
    first_goal=g_balance.parking_target_m;
    assert(fabsf(first_goal-g_balance.state[2])<1e-7f);
    imu.attitude_deg[BODY_PITCH_AXIS]=sign*4.5f/BODY_PITCH_DIRECTION;
   } else theta_slew_check(previous);
   assert(g_balance.position_locked && fabsf(g_balance.parking_target_m-first_goal)<1e-7f);
   if(phase==ROBOT_STARTUP_RAMP)saw_ramp=1;
   if(phase==ROBOT_STARTUP_BALANCE && previous_phase==ROBOT_STARTUP_RAMP) {
    assert(fabsf(reference)>.10f);
    saw_continuous_handover=1;
   }
   previous=reference;previous_phase=phase;
  }
  equilibrium(g_balance.leg_length_m,g_balance.reference[4],&theta_eq,&tp_ff);
  assert(saw_ramp && saw_continuous_handover && fabsf(g_balance.reference[0]-theta_eq)<1e-7f);
 }
 g_robot_control.enable=0;step();
 imu=(ImuSample){0};fixture_pose();fixture_velocity(0);g_dual_hold.target_m=.10f;
}

static void equilibrium_prescription_regression(void) {
 const float rad=.017453292519943295f,half_pi=1.5707963267948966f;
 const float lengths[3]={.075f,.10f,.12f};
 const uint32_t startup_enable=g_robot_control.startup_enable;
 g_robot_control.remote_enable=0;g_balance_bias.enable=0;
 /* Prescribe both signs of the equilibrium geometry. With startup disabled,
  * theta must immediately follow the current (still slewing) trim rather than
  * capturing the measured theta. Once trim settles, zero state error leaves
  * exactly T=0 and Tp=Tpff, without moving the fixed parking goal. */
 for(int sign=-1;sign<=1;sign+=2)for(unsigned i=0;i<3;++i) {
  g_robot_control.enable=0;step();
  g_robot_control.startup_enable=0;g_balance.pitch_trim_deg=sign*1.5f;
  g_dual_hold.target_m=lengths[i];
  float phi=-sign*1.5f*rad,theta_eq,tp_ff;
  equilibrium(lengths[i],phi,&theta_eq,&tp_ff);
  imu=(ImuSample){0};imu.attitude_deg[BODY_PITCH_AXIS]=-phi/rad/BODY_PITCH_DIRECTION;
  fixture_velocity(0);
  for(unsigned side=0;side<2;++side)
   swing_feedback(side,lengths[i],half_pi+phi+theta_eq,0);
  g_robot_control.enable=1;step();native_check();
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
  float parking_goal=g_balance.parking_target_m;
  assert(fabsf(parking_goal-g_balance.state[2])<1e-7f);
  assert(fabsf(g_balance.reference[0]-g_balance.state[0])>1e-3f);
  for(unsigned n=0;n<800;++n) {
   equilibrium(g_balance.leg_length_m,g_balance.reference[4],&theta_eq,&tp_ff);
   assert(fabsf(g_balance.reference[0]-theta_eq)<1e-7f);
   assert(g_balance.position_locked && fabsf(g_balance.parking_target_m-parking_goal)<1e-7f);
   step();native_check();
  }
  equilibrium(g_balance.leg_length_m,phi,&theta_eq,&tp_ff);
  assert(fabsf(g_balance.reference[4]-phi)<1e-7f);
  assert(fabsf(g_balance.reference[0]-theta_eq)<1e-7f);
  for(unsigned j=0;j<6;++j)
   assert(fabsf(g_balance.state[j]-g_balance.reference[j])<2e-6f);
  assert(fabsf(g_balance.torque_nm[0])<2e-5f);
  assert(fabsf(g_balance.torque_nm[1]-tp_ff)<2e-5f && sign*tp_ff>0);
  assert(fabsf(g_balance.parking_error_m)<1e-7f);
 }
 /* Enabled startup captures only once. Changes in trim and actual leg length
  * move its equilibrium destination, with the same rate limit throughout the
  * phase transitions and after BALANCE has been reached. */
 for(int sign=-1;sign<=1;sign+=2) {
  g_robot_control.enable=0;step();
  g_robot_control.startup_enable=1;g_balance.pitch_trim_deg=sign*.5f;
  g_dual_hold.target_m=.10f;
  imu=(ImuSample){0};imu.attitude_deg[BODY_PITCH_AXIS]=sign*20/BODY_PITCH_DIRECTION;
  fixture_velocity(0);
  for(unsigned side=0;side<2;++side)swing_feedback(side,.10f,half_pi,0);
  g_robot_control.enable=1;step();native_check();
  float previous=g_balance.reference[0],parking_goal=g_balance.parking_target_m;
  assert(fabsf(previous-g_balance.state[0])<1e-7f);
  unsigned saw_ramp=0,saw_balance=0,saw_length_change=0,saw_trim_change=0;
  for(unsigned n=0;n<1400;++n) {
   float length=n<700?.10f:.075f;
   if(n==900)g_balance.pitch_trim_deg=-sign*1.5f;
   float phi=-g_balance.pitch_trim_applied_deg*rad,theta_eq,tp_ff;
   equilibrium(length,phi,&theta_eq,&tp_ff);
   imu.attitude_deg[BODY_PITCH_AXIS]=-phi/rad/BODY_PITCH_DIRECTION;
   for(unsigned side=0;side<2;++side)swing_feedback(side,length,half_pi+phi+theta_eq,0);
   step();native_check();theta_slew_check(previous);previous=g_balance.reference[0];
   assert(g_balance.position_locked && fabsf(g_balance.parking_target_m-parking_goal)<1e-7f);
   if(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP)saw_ramp=1;
   if(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE)saw_balance=1;
   if(n==700) {
    assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
    saw_length_change=1;
   }
   if(n>900 && sign*g_balance.reference[4]>0)saw_trim_change=1;
  }
  assert(saw_ramp && saw_balance && saw_length_change && saw_trim_change);
 }
 /* At the permitted +/-3deg trim, a reachable short leg has a local
  * equilibrium more than5deg away from90deg. Both physical-pose readiness
  * and local-reference readiness must use that same equilibrium target;
  * the obsolete fixed90deg gate would remain in PREPARE forever. */
 for(int sign=-1;sign<=1;sign+=2) {
  g_robot_control.enable=0;step();
  g_robot_control.startup_enable=1;g_balance.pitch_trim_deg=sign*3;
  const float length=.052f,phi=-sign*3*rad;
  float theta_eq,tp_ff;equilibrium(length,phi,&theta_eq,&tp_ff);
  float angle=half_pi+phi+theta_eq;
  assert(fabsf(angle-half_pi)>ROBOT_STARTUP_ANGLE_ERROR_RAD);
  g_dual_hold.target_m=length;imu=(ImuSample){0};
  imu.attitude_deg[BODY_PITCH_AXIS]=sign*3/BODY_PITCH_DIRECTION;
  fixture_velocity(0);
  for(unsigned side=0;side<2;++side)swing_feedback(side,length,angle,0);
  g_robot_control.enable=1;step();native_check();
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  float previous=g_balance.reference[0],parking_goal=g_balance.parking_target_m;
  assert(fabsf(previous-g_balance.state[0])<1e-7f);
  unsigned saw_ramp=0,saw_balance=0;
  for(unsigned n=0;n<1800;++n) {
   step();native_check();theta_slew_check(previous);previous=g_balance.reference[0];
   assert(g_balance.position_locked && fabsf(g_balance.parking_target_m-parking_goal)<1e-7f);
   if(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP)saw_ramp=1;
   if(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE)saw_balance=1;
  }
  assert(saw_ramp && saw_balance && g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
  assert(g_robot_control.startup_ready_mask&ROBOT_READY_POSE);
  assert(fabsf(g_dual_hold.angle_reference_rad[0]-angle)<2e-6f);
  assert(fabsf(g_dual_hold.angle_reference_rad[1]-angle)<2e-6f);
  assert(fabsf(g_balance.reference[4]-phi)<1e-7f);
  assert(fabsf(g_balance.reference[0]-theta_eq)<1e-7f);
  assert(fabsf(g_balance.torque_nm[0])<2e-5f);
  assert(fabsf(g_balance.torque_nm[1]-tp_ff)<2e-5f);
 }
 g_robot_control.enable=0;step();
 g_robot_control.startup_enable=startup_enable;
 g_balance.pitch_trim_deg=.5f;g_dual_hold.target_m=.10f;
 imu=(ImuSample){0};fixture_pose();fixture_velocity(0);
}
#endif
int main(void) {
 extern volatile uint32_t g_robot_bench_limit_ms;
 assert(!ROBOT_CAPTURE_BUILD && g_robot_bench_limit_ms==0);
 assert(BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE<=1U);
 assert(g_balance_bias.enable==BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE);
 assert(fabsf(g_balance.pitch_trim_deg-BALANCE_DEFAULT_PITCH_TRIM_DEG)<1e-7f);
 assert(fabsf(g_dual_hold.target_m-.07f)<1e-7f);
 assert(fabsf(g_dual_hold.angle_kp_nm_per_rad-.60f)<1e-7f &&
        fabsf(g_dual_hold.angle_kd_nm_s_per_rad-.020f)<1e-7f &&
        fabsf(g_dual_hold.angle_limit_nm-.16f)<1e-7f);
 assert(g_balance.leg_hold_ratio==0);
 assert(ROBOT_NATIVE_LQR==1 && BALANCE_LEG_LQR_TORQUE_LIMIT_NM==0);
 assert(fabsf(BALANCE_NATIVE_PARKING_WINDOW_M-.30f)<1e-7f);
 assert(fabsf(ROBOT_MOTION_TRACK_WINDOW_M-.05f)<1e-7f);
 assert(g_balance.wheel_scale==1 && g_balance.leg_scale==1 && g_balance.wheel_rate_scale==1);
 assert(!g_balance.parking_lean_enable && !g_balance.parking_damping_gain_nm_per_m_s);
 g_dual_hold.target_m=.10f; /* Match the synthetic fixture during readiness-gated startup. */
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 imu=(ImuSample){0};imu_ok=1;fixture_pose();fixture_velocity(0);
 g_robot_control.remote_enable=0;g_robot_control.enable=1;
 g_dual_hold.feedforward_target_n=2;g_dual_hold.feedforward_rate_n_s=1000;
 step();
#if !MOTOR_COMMAND_TX_ENABLED
 assert(!g_robot_control.enable && calls==0);puts("PASS: native TX-disabled remains off");return 0;
#endif
 native_check();
 for(unsigned n=0;n<200 && g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE;++n)step();
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
#if MOTOR_COMMAND_TX_ENABLED
 startup_recovery_regression();
 equilibrium_prescription_regression();
#endif
 /* The user's absolute-angle hold ratio applies unchanged in every phase,
  * including its shipped zero default and an explicitly enabled hold. */
 const float hold_ratios[3]={0,.4f,1};
 for(unsigned trial=0;trial<3;++trial) {
  g_robot_control.enable=0;step();g_balance.leg_hold_ratio=hold_ratios[trial];
  float q[2];assert(leg_inverse(.03f+.10f*cosf(1.49f),.10f*sinf(1.49f),1.57f,1.57f,q));
  for(unsigned side=0;side<2;++side) {
   motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=q[1];
   motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=q[0];
  }
  g_robot_control.enable=1;
  for(unsigned n=0;n<30;++n){step();native_check();}
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  if(trial)assert(g_dual_hold.angle_hold_torque_nm[0]>.005f);
  else assert(g_dual_hold.angle_hold_torque_nm[0]==0 && g_dual_hold.angle_hold_torque_nm[1]==0);
  for(unsigned n=0;n<400 && g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE;++n){step();native_check();}
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
  assert(g_dual_hold.lqr_pitch_torque_nm[0]==g_balance.torque_nm[1]);
 }
 g_robot_control.enable=0;step();g_balance.leg_hold_ratio=BALANCE_DEFAULT_LEG_HOLD_RATIO;fixture_pose();
 g_robot_control.enable=1;
 for(unsigned n=0;n<200 && g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE;++n){step();native_check();}
 /* Obsolete scale/parking fields do not change native math; pitch trim does. */
 g_balance.wheel_scale=.07f;g_balance.leg_scale=.09f;g_balance.wheel_rate_scale=.03f;
 g_balance.parking_lean_enable=1;g_balance.parking_lean_gain_rad_per_m=.03f;
 g_balance.parking_damping_gain_nm_per_m_s=.009f;g_balance.pitch_trim_deg=2;
 g_dual_hold.sync_recovery_priority_enable=1;
 for(int pitch=-20;pitch<=20;pitch+=4) {
  imu.attitude_deg[BODY_PITCH_AXIS]=(float)pitch/BODY_PITCH_DIRECTION;
  step();native_check();
 }
 /* Synthetic finite-rate stress verifies no intermediate Tp clipping even
  * after the effort-weighted gain is reduced; not a physical-motion test. */
 imu.gyro_rad_s[BODY_PITCH_AXIS]=100;
 step();native_check();
 assert(fabsf(g_balance.torque_nm[1])>.12f); /* Removed old intermediate Tp ceiling. */
 g_robot_control.enable=0;step();
 /* Length/support are not handover gates. Wait for actual pose and neutral
  * local references, then the settle dwell even with non-target leg length. */
 g_dual_hold.target_m=.07f;imu=(ImuSample){0};fixture_pose();fixture_velocity(0);
 g_robot_control.enable=1;step();assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
 for(unsigned n=0;n<200 && g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE;++n){step();native_check();}
 assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_LENGTH));
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
 g_robot_control.enable=0;step();g_dual_hold.target_m=.10f;
 /* Hand-held startup: inclination and body motion within the existing
  * running posture envelope do not need a separate static-ready gate. */
 for(int sign=-1;sign<=1;sign+=2) {
  imu=(ImuSample){0};imu.attitude_deg[BODY_PITCH_AXIS]=sign*20/BODY_PITCH_DIRECTION;
  imu.gyro_rad_s[BODY_PITCH_AXIS]=sign*.3f/BODY_PITCH_DIRECTION;
  fixture_pose();fixture_velocity(0);g_robot_control.enable=1;step();
  assert(g_robot_control.enable);native_check();
  g_robot_control.enable=0;step();
  imu=(ImuSample){0};imu.attitude_deg[BODY_PITCH_AXIS]=sign*36/BODY_PITCH_DIRECTION;
  fixture_pose();fixture_velocity(0);g_robot_control.enable=1;step();
  assert(!g_robot_control.enable && g_robot_control.last_fault.reason==10);zeros();
 }
 /* Replay the measured generation-13 rejection: 11.2deg body, left world
  * leg -20.7deg, body rate -15.3deg/s and moving joints. Command admission
  * is verified here; this does not assert physical recovery or stability. */
 imu=(ImuSample){0};
 imu.attitude_deg[BODY_PITCH_AXIS]=11.204192f/BODY_PITCH_DIRECTION;
 imu.gyro_rad_s[BODY_PITCH_AXIS]=-.267385f/BODY_PITCH_DIRECTION;
 const float rejected_q[4]={-.504000545f,2.015592098f,-.351184964f,2.452592373f};
 const float rejected_dq[4]={.225f,-.026f,-.070f,-.541f};
 for(unsigned i=0;i<4;++i) {
  motors[g_joint_calibration[i].motor_id-1].joint_angle_rad=rejected_q[i];
  motors[g_joint_calibration[i].motor_id-1].joint_speed_rad_s=rejected_dq[i];
 }
 g_robot_control.enable=1;step();assert(g_robot_control.enable);native_check();
 assert(fabsf(g_wheel_leg.angle_speed_rad_s[1])>.2f);
 g_robot_control.enable=0;step();
 for(unsigned i=0;i<8;++i)motors[i].joint_speed_rad_s=0;
 /* The parking origin is fixed on the very first active frame, including
  * PREPARE, RAMP and BALANCE while measured wheels coast. Both directions
  * then integrate a drive goal and capture exactly once when release slews
  * the requested speed to zero. */
 g_robot_control.enable=0;step();
 g_balance.wheel_scale=g_balance.leg_scale=g_balance.wheel_rate_scale=1;
 g_balance.parking_lean_enable=0;g_balance.pitch_trim_deg=0;
 g_robot_control.remote_enable=1;
 remote_ok=remote_fresh=1;motion_sticks.valid=motion_sticks.calibrated=1;
 for(int sign=-1;sign<=1;sign+=2) {
  remote_sample.frame.channel[4]=1000;motion_sticks.forward=0;step();zeros();
  imu=(ImuSample){0};imu.attitude_deg[BODY_PITCH_AXIS]=6/BODY_PITCH_DIRECTION;
  fixture_pose();fixture_velocity(sign*.10f);
  remote_sample.frame.channel[4]=306;step();native_check();
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE && g_balance.position_locked);
  float launch_goal=g_balance.parking_target_m,previous_theta=g_balance.reference[0];
  assert(fabsf(launch_goal-g_balance.state[2])<1e-7f);
  for(unsigned n=0;n<80;++n) {
   motion_sticks.forward=(n>=20 && n<40)?sign:0;
   step();native_check();theta_slew_check(previous_theta);previous_theta=g_balance.reference[0];
   assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
   assert(g_balance.reference[3]==0 && g_robot_motion.command.forward_m_s==0);
   assert(g_balance.position_locked && fabsf(g_balance.parking_target_m-launch_goal)<1e-7f);
  }
  imu.attitude_deg[BODY_PITCH_AXIS]=0;motion_sticks.forward=0;
  unsigned saw_ramp=0,saw_balance=0;
  for(unsigned n=0;n<400;++n) {
   motion_sticks.forward=g_robot_control.startup_phase==ROBOT_STARTUP_RAMP?sign:0;
   step();native_check();theta_slew_check(previous_theta);previous_theta=g_balance.reference[0];
   if(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP)saw_ramp=1;
   if(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE)saw_balance=1;
   else assert(g_balance.reference[3]==0);
   assert(g_balance.position_locked && fabsf(g_balance.parking_target_m-launch_goal)<1e-7f);
   assert(fabsf(g_balance.state[2]-g_balance.reference[2])<=BALANCE_NATIVE_PARKING_WINDOW_M+1e-6f);
  }
  assert(saw_ramp && saw_balance && g_robot_motion.neutral_ready);
  assert(sign*g_balance.parking_error_m>.20f);
  /* Exact local integration, with only its existing5cm tracking bound. */
  fixture_velocity(0);motion_sticks.forward=sign;
  unsigned was_driving=0;float previous_goal=launch_goal;
  for(unsigned n=0;n<100;++n) {
   step();native_check();
   float velocity=g_balance.reference[3];
   assert(sign*velocity>0 && !g_balance.position_locked);
   float expected=(was_driving?previous_goal:g_balance.state[2])+velocity*.005f;
   expected=g_balance.state[2]+fmaxf(-ROBOT_MOTION_TRACK_WINDOW_M,
       fminf(ROBOT_MOTION_TRACK_WINDOW_M,expected-g_balance.state[2]));
   assert(fabsf(g_balance.parking_target_m-expected)<1e-6f);
   previous_goal=g_balance.parking_target_m;was_driving=1;
  }
  assert(sign*g_balance.reference[3]>.39f);
  assert(fabsf(sign*(g_balance.reference[2]-g_balance.state[2])-ROBOT_MOTION_TRACK_WINDOW_M)<1e-5f);
  motion_sticks.forward=0;fixture_velocity(sign*.10f);
  unsigned captured=0;float release_goal=0;
  for(unsigned n=0;n<120 && !captured;++n) {
   step();native_check();
   if(fabsf(g_balance.reference[3])<=1e-6f) {
    assert(fabsf(g_robot_motion.command.forward_m_s)<=1e-6f && g_balance.position_locked);
    assert(fabsf(g_balance.parking_target_m-g_balance.state[2])<1e-7f);
    captured=1;release_goal=g_balance.parking_target_m;
   } else assert(!g_balance.position_locked);
  }
  assert(captured);
  for(unsigned n=0;n<800;++n) {
   step();native_check();
   assert(g_balance.reference[3]==0 && g_robot_motion.command.forward_m_s==0);
   assert(g_balance.position_locked && fabsf(g_balance.parking_target_m-release_goal)<1e-7f);
   assert(fabsf(g_balance.parking_error_m-(g_balance.state[2]-release_goal))<1e-6f);
   assert(fabsf(g_balance.state[2]-g_balance.reference[2])<=BALANCE_NATIVE_PARKING_WINDOW_M+1e-6f);
  }
  assert(sign*g_balance.parking_error_m>.39f);
  assert(fabsf(sign*(g_balance.state[2]-g_balance.reference[2])-BALANCE_NATIVE_PARKING_WINDOW_M)<1e-6f);
  assert(sign*(g_balance.reference[2]-release_goal)>.09f);
 }
 imu=(ImuSample){0};fixture_velocity(0);
#if MOTOR_COMMAND_TX_ENABLED && LEG_ANGLE_INTEGRAL_ENABLED
 /* Loaded rearward-leg bias: prescribed geometry stays 0.1rad behind its
  * target, below the existing local cap. Bias must grow toward neutral,
  * obey its per-tick slew/cap, freeze at local saturation and reset OFF. */
 remote_sample.frame.channel[4]=1000;step();
 assert(g_leg_angle_integral_nm[0]==0 && g_leg_angle_integral_nm[1]==0);
 fixture_velocity(0);imu=(ImuSample){0};g_balance.pitch_trim_deg=0;
 swing_feedback(0,.10f,1.5707963268f-.10f,0);
 swing_feedback(1,.10f,1.5707963268f-.10f,0);
 remote_sample.frame.channel[4]=306;
 for(unsigned n=0;n<800;++n) {
  float old=g_leg_angle_integral_nm[0];step();native_check();
  assert(fabsf(g_leg_angle_integral_nm[0]-old)<=LEG_ANGLE_I_RATE_NM_S*.005f+1e-7f);
  assert(g_leg_angle_integral_nm[0]>=0 &&
         g_leg_angle_integral_nm[0]<=LEG_ANGLE_I_LIMIT_NM+1e-7f);
 }
 assert(g_leg_angle_integral_nm[0]>.07f && g_leg_angle_integral_nm[1]>.07f);
 swing_feedback(0,.10f,1.5707963268f-.30f,0);
 swing_feedback(1,.10f,1.5707963268f-.30f,0);
 float retained=g_leg_angle_integral_nm[0];
 for(unsigned n=0;n<50;++n){step();native_check();}
 assert(fabsf(g_leg_angle_integral_nm[0]-retained)<1e-7f);
 remote_sample.frame.channel[4]=1000;step();
 assert(g_leg_angle_integral_nm[0]==0 && g_leg_angle_integral_nm[1]==0);zeros();
 fixture_pose();
#endif
 /* OFF/feedback-only and invalid motor feedback retain fail-closed behavior. */
 remote_sample.frame.channel[4]=1000;step();assert(!g_robot_control.enable);zeros();
 remote_sample.frame.channel[4]=306;for(unsigned n=0;n<360;++n)step();
 g_robot_control.feedback_only=1;step();assert(!g_robot_control.enable);zeros();
 g_robot_control.feedback_only=0;step();assert(!g_robot_control.enable);
 remote_sample.frame.channel[4]=1000;step();remote_sample.frame.channel[4]=306;
 for(unsigned n=0;n<360;++n)step();
 /* Unused Watch scale fields cannot disable native posture protection. */
 g_balance.wheel_scale=g_balance.leg_scale=g_balance.back_emf_ratio=0;
 imu.attitude_deg[BODY_PITCH_AXIS]=40/BODY_PITCH_DIRECTION;
 for(unsigned n=0;n<9;++n)step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==10);zeros();
 imu=(ImuSample){0};g_balance.wheel_scale=g_balance.leg_scale=1;
 remote_sample.frame.channel[4]=1000;step();remote_sample.frame.channel[4]=306;
 for(unsigned n=0;n<360;++n)step();
 motors[JOINT_LF_CAN_ID-1].online=0;step();assert(!g_robot_control.enable);zeros();
 puts("PASS: native -K plus equilibrium Tpff, signed trim equilibrium and continuous theta, fixed parking goal with 30cm input cap, bidirectional 5cm drive/release capture, OFF/rearm and feedback stop");
}
'''
with tempfile.TemporaryDirectory(prefix='native_lqr_') as directory:
 work=Path(directory);(work/'runtime.h').write_text(strings['runtime'])
 for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):
  (work/name).write_text('#include "runtime.h"\n')
 (work/'test.c').write_text(prefix+fixture)
 sources=['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c',
          'App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_command.c',
          'tests/support/wheel_leg/Src/wheel_leg_control.c','Control/WheelLeg/Src/lqr_k.c',
          'Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
 command=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',
          '-DROBOT_NATIVE_LQR=1','-DROBOT_LEGS_ONLY_CALIBRATION=0',
          '-DBALANCE_NATIVE_SERVO_ENABLE=0','-DBALANCE_MOTION_REFERENCE_ENABLE=0',
          '-I',str(work)]
 for folder in ('App/Tasks/Inc','App/Services/Inc','App/Protocol/Inc','Control/WheelLeg/Inc','tests/support/wheel_leg/Inc'):
  command+=['-I',str(root/folder)]
 for tx in (0,1):
  exe=work/f'test{tx}.exe'
  subprocess.run(command+[f'-DMOTOR_COMMAND_TX_ENABLED={tx}',str(work/'test.c'),
       *[str(root/p) for p in sources],'-lm','-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
