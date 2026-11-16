"""Real task regression for g23 reverse failure, bounded learning and neutral yaw rejection.

Prescribed feedback verifies controller actions, not physical vehicle stability.
"""
import ast,subprocess,tempfile
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def strings(path):
 result={}
 for n in ast.parse(path.read_text(encoding='utf-8')).body:
  if isinstance(n,ast.Assign) and isinstance(n.value,ast.Constant) and isinstance(n.value.value,str):
   for t in n.targets:
    if isinstance(t,ast.Name):result[t.id]=n.value.value
 return result
base=strings(root/'tests/verify_wheel_leg_control.py')
prefix=base['harness'].split('int main(void) {',1)[0]
a=prefix.index('volatile LqrRecordDebug g_lqr_record;');b=prefix.index('void RobotTelemetry_Capture',a)
prefix=prefix[:a]+prefix[b:]
helpers=strings(root/'tests/verify_remote_motion_control.py')['fixture'].split('int main(int argc',1)[0]
main=r'''
int main(void){
 LqrStartupRecord_Init();
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 pose(0);pose(1);g_balance.back_emf_ratio=0;g_dual_hold.feedforward_target_n=0;
 remote_ok=remote_fresh=1;motion_sticks.valid=motion_sticks.calibrated=1;
 off();on();velocity(.25f);
 for(unsigned n=0;n<300;++n){step();checked();}
#if ROBOT_MOTION_STARTUP_RECOVERY_ENABLE
 assert(g_balance.startup_recovery && g_balance.motion_braking && !g_balance.position_locked);
 assert(fabsf(g_balance.motion_damping_torque_nm+ROBOT_MOTION_STARTUP_DAMPING_LIMIT_NM)<1e-6f);
 assert(g_robot_motion.command.forward_m_s==0 && g_balance.motion_damping_torque_nm<0);
 assert(g_balance.motion_lean_rad<0 && g_balance.parking_lean_rad<0);
#else
 assert(!g_balance.startup_recovery && !g_balance.motion_braking && g_balance.position_locked);
 assert(g_robot_motion.command.forward_m_s==0 && g_balance.reference[3]==0);
 assert(!g_balance.motion_damping_active && g_balance.motion_damping_torque_nm==0);
 assert(g_balance.motion_lean_rad==0 && g_balance.parking_damping_active);
 float parked_goal=g_balance.parking_target_m;
 for(unsigned direction=0;direction<2;++direction){
  velocity(direction ? -.25f : .25f);
  for(unsigned n=0;n<200;++n){
   step();checked();
   assert(g_balance.position_locked && !g_balance.motion_braking && !g_balance.startup_recovery);
   assert(g_balance.parking_target_m==parked_goal && g_balance.reference[3]==0);
   assert(fabsf(g_balance.state[2]-g_balance.reference[2])<=g_balance.parking_correction_limit_m+1e-6f);
   assert(g_balance.motion_lean_rad==0 && g_balance.motion_damping_torque_nm==0);
  }
 }
 puts("PASS: neutral startup fixed parking, retained goal across bidirectional rolling and no moving-recovery/fast-lean reference");
#endif
 velocity(0);for(unsigned n=0;n<200;++n){step();checked();}
 assert(!g_balance.startup_recovery && g_balance.position_locked);
 off();on();for(unsigned n=0;n<440;++n)step();center();
 /* Stationary demand builds bounded torque above the old P-only ceiling. */
 motion_sticks.yaw=.60f/ROBOT_MOTION_DEFAULT_YAW_RAD_S;
 for(unsigned n=0;n<800;++n){step();checked();
  assert(fabsf(g_robot_motion.yaw_integral_nm)<=ROBOT_MOTION_YAW_INTEGRAL_LIMIT_NM);
 }
 assert(g_robot_motion.yaw_integral_nm>.004f && g_wheel_output.applied_differential_nm>.007f);
 float integral=g_robot_motion.yaw_integral_nm;
 for(unsigned n=0;n<200;++n){step();checked();}
 assert(g_robot_motion.yaw_integral_nm==integral && g_robot_motion.yaw_integral_blocked);
 motion_sticks.yaw=0;step();assert(g_robot_motion.yaw_integral_nm==0);center();
 motion_sticks.yaw=-.60f/ROBOT_MOTION_DEFAULT_YAW_RAD_S;for(unsigned n=0;n<400;++n){step();checked();}
 assert(g_robot_motion.yaw_integral_nm<-.003f);
 motion_sticks.yaw=1;step();assert(g_robot_motion.yaw_integral_nm==0);
 center();motion_sticks.yaw=1;
 /* Prescribe no differential headroom: the integral must not wind up. */
 for(unsigned n=0;n<400;++n){
  g_robot_motion.differential_requested_nm=.008f;
  g_robot_motion.differential_applied_nm=0;
  step();checked();assert(fabsf(g_robot_motion.yaw_integral_nm)<1e-9f);
 }
 assert(g_robot_motion.yaw_integral_blocked);
 off();assert(g_robot_motion.yaw_integral_nm==0 && !g_robot_motion.yaw_integral_blocked);
 motion_sticks.yaw=0;on();for(unsigned n=0;n<440;++n)step();center();
 puts("PASS: stationary PI buildup/cap, release/reversal reset and headroom antiwindup");
 /* A neutral stick rejects yaw even before the first drive/turn request. */
 imu.gyro_rad_s[2]=-.3f;
 for(unsigned n=0;n<80;++n){step();checked();}
 assert(g_robot_motion.yaw_active && g_wheel_output.applied_differential_nm<-.001f);
 imu.gyro_rad_s[2]=0;center();
 assert(!g_robot_motion.yaw_active && g_wheel_output.applied_differential_nm==0);
 /* Alternating +/-0.1 rad/s sensor noise is attenuated below the deadband. */
 for(unsigned n=0;n<100;++n){imu.gyro_rad_s[2]=(n&1)?.1f:-.1f;step();checked();}
 assert(fabsf(g_robot_motion.filtered_yaw_rate_rad_s)<.02f);
 assert(g_wheel_output.applied_differential_nm==0);
 imu.gyro_rad_s[2]=0;center();
 /* Neutral coasting must retain braking and defer position capture. */
 motion_sticks.forward=.5f;velocity(.2f);for(unsigned n=0;n<100;++n){step();checked();}
 motion_sticks.forward=0;for(unsigned n=0;n<100;++n){step();checked();}
 assert(g_balance.motion_braking && !g_balance.position_locked);
 assert(g_balance.motion_damping_active && g_balance.motion_damping_torque_nm<0);
 assert(g_balance.reference[3]==0 && fabsf(g_balance.reference[2]-g_balance.state[2])<1e-7f);
 float saved_lean=g_balance.parking_lean_rad;
 for(unsigned n=0;n<100;++n){step();checked();}
 assert(g_balance.parking_lean_rad==saved_lean && !g_balance.parking_lean_learning);
 velocity(0);for(unsigned n=0;n<39;++n){step();checked();}
 assert(g_balance.motion_braking && !g_balance.position_locked);
 step();assert(!g_balance.motion_braking && g_balance.position_locked);
 for(unsigned n=0;n<100;++n)step();
 assert(g_balance.motion_damping_torque_nm==0 && g_balance.motion_lean_rad==0);
 /* Actual coasting limits yaw even when CH2 is neutral. */
 motion_sticks.yaw=1;velocity(.4f);for(unsigned n=0;n<140;++n){step();checked();}
 assert(fabsf(g_robot_motion.command.turn_rad_s-.55f)<1e-5f);
 velocity(0);for(unsigned n=0;n<140;++n){step();checked();}
 assert(fabsf(g_robot_motion.command.turn_rad_s-3.00f)<1e-5f);
 off();assert(!g_balance.motion_braking);motion_sticks.yaw=0;on();for(unsigned n=0;n<440;++n)step();center();
 /* g23: target -0.1, wheel speed +0.067. Lean must not freeze at near zero. */
 motion_sticks.forward=-.1f/ROBOT_MOTION_DEFAULT_FORWARD_M_S;velocity(.067f);imu.gyro_rad_s[2]=-.3f;
 float start=g_balance.parking_lean_rad,previous=start;
 for(unsigned n=0;n<600;++n){
  step();checked();
  assert(fabsf(g_balance.parking_lean_rad-previous)<=ROBOT_MOTION_LEAN_RATE_RAD_S*.005f+1e-7f);
  previous=g_balance.parking_lean_rad;
  assert(fabsf(previous)<=BALANCE_PARKING_LEAN_LIMIT_RAD);
 }
 assert(g_balance.motion_lean_learning && !g_balance.motion_stall_hold);
 assert(g_balance.parking_lean_rad<start-.015f);
 assert(g_balance.motion_damping_torque_nm<-.0044f);
 assert(fabsf(g_balance.reference[0]-g_balance.parking_lean_rad-g_balance.motion_lean_rad)<1e-7f);
 for(unsigned s=0;s<2;++s)
  assert(fabsf(g_dual_hold.angle_reference_rad[s]-(1.57079632679f+g_balance.parking_lean_rad+g_balance.motion_lean_rad))<1e-6f);
 for(unsigned n=0;n<600;++n){step();checked();}
 assert(fabsf(g_balance.parking_lean_rad+BALANCE_PARKING_LEAN_LIMIT_RAD)<1e-6f);
 float reduced=g_balance.parking_lean_rad;
 /* Reproduce g26 overspeed: .4 m/s must still permit bounded correction. */
 velocity(.4f);for(unsigned n=0;n<100;++n){step();checked();}
 assert(g_balance.motion_lean_learning);
 assert(g_balance.motion_lean_rad<-.017f);
 assert(g_balance.motion_damping_torque_nm<-.0089f);
 assert(fabsf(g_balance.reference[0])<=BALANCE_PARKING_LEAN_LIMIT_RAD+ROBOT_MOTION_FAST_LEAN_LIMIT_RAD+1e-7f);
 /* Matching backward motion is NOT integrated toward zero speed. */
 imu.gyro_rad_s[2]=0;velocity(-.1f);for(unsigned n=0;n<350;++n){step();checked();}
 float matched=g_balance.parking_lean_rad;
 for(unsigned n=0;n<200;++n){step();checked();}
 assert(fabsf(g_balance.parking_lean_rad-matched)<1e-5f);
 assert(fabsf(g_balance.motion_damping_torque_nm)<1e-6f);
 assert(fabsf(g_balance.motion_lean_rad)<1e-6f);
 /* A sign-reversed tracking error reverses correction. */
 motion_sticks.forward= .1f/ROBOT_MOTION_DEFAULT_FORWARD_M_S;velocity(-.067f);
 for(unsigned n=0;n<600;++n){step();checked();}
 assert(g_balance.parking_lean_rad>matched+.015f);
 assert(g_balance.motion_damping_torque_nm>.0044f);
 /* Saturated actuators pause integration without accumulating hidden error. */
 float hold=g_balance.parking_lean_rad;
 for(unsigned n=0;n<200;++n){g_joint_output.saturated=1;step();checked();}
 assert(g_balance.parking_lean_rad==hold && !g_balance.motion_lean_learning);
 /* Zero measured speed under sustained demand: freeze bias after 0.5s,
    but retain bounded proportional correction and all existing ownership. */
 velocity(0);for(unsigned n=0;n<200;++n){step();checked();}
 assert(g_balance.motion_stall_hold && !g_balance.motion_lean_learning);
 hold=g_balance.parking_lean_rad;
 for(unsigned n=0;n<800;++n){step();checked();}
 assert(g_balance.parking_lean_rad==hold && g_robot_control.enable);
 assert(g_balance.motion_damping_torque_nm>0 && g_balance.motion_damping_torque_nm<=ROBOT_MOTION_SPEED_DAMPING_LIMIT_NM+1e-7f);
 velocity(.02f);for(unsigned n=0;n<100;++n){step();checked();}
 assert(!g_balance.motion_stall_hold && g_balance.motion_lean_learning);
 /* Normal commanded turns keep longitudinal learning; excessive actual yaw freezes it. */
 motion_sticks.yaw=1;imu.gyro_rad_s[2]=-.3f;
 for(unsigned n=0;n<120;++n){step();checked();}
 assert(g_balance.motion_lean_learning);
 imu.gyro_rad_s[2]=-1.0f;step();
 hold=g_balance.parking_lean_rad;
 for(unsigned n=0;n<100;++n){step();checked();}
 assert(!g_balance.motion_lean_learning && g_balance.parking_lean_rad==hold);
 /* g32: target reverse while moving forward in a fast turn. Slow bias
  * stays frozen, but bounded fast lean must oppose the velocity error. */
 for(unsigned turn=0;turn<2;++turn){
  imu.gyro_rad_s[2]=turn ? 3.0f : -3.0f;
  motion_sticks.yaw=turn ? -1 : 1;
  for(unsigned direction=0;direction<2;++direction){
   float sign=direction ? -1.0f : 1.0f;
   motion_sticks.forward=-sign*.14f/ROBOT_MOTION_DEFAULT_FORWARD_M_S;
   velocity(sign*.15f);hold=g_balance.parking_lean_rad;
   for(unsigned n=0;n<300;++n){
    float previous_fast=g_balance.motion_lean_rad;
    step();checked();
    assert(!g_balance.motion_lean_learning && g_balance.parking_lean_rad==hold);
    assert(fabsf(g_balance.motion_lean_rad-previous_fast)<=ROBOT_MOTION_FAST_LEAN_RATE_RAD_S*.005f+1e-7f);
    assert(fabsf(g_balance.motion_lean_rad)<=ROBOT_MOTION_FAST_LEAN_LIMIT_RAD+1e-7f);
    assert(fabsf(g_balance.reference[0]-hold-g_balance.motion_lean_rad)<1e-7f);
   }
   assert(sign*g_balance.motion_lean_rad<-.017f);
   assert(sign*g_balance.motion_damping_torque_nm<0);
   assert(sign*g_balance.motion_damping_torque_nm<-.029f);
   for(unsigned side=0;side<2;++side)
    assert(fabsf(g_dual_hold.angle_reference_rad[side]-(1.57079632679f+hold+g_balance.motion_lean_rad))<1e-6f);
   /* Actuator saturation still fades fast correction out during the turn. */
   for(unsigned n=0;n<100;++n){g_joint_output.saturated=1;step();}
   assert(fabsf(g_balance.motion_lean_rad)<1e-7f);
  }
 }
 /* g33: crossing the slow-learning 0.6m/s gate must not remove fast
  * overspeed braking lean. Exercise forward/backward drive and neutral. */
 imu.gyro_rad_s[2]=0;motion_sticks.yaw=0;
 for(unsigned direction=0;direction<2;++direction){
  float sign=direction ? -1.0f : 1.0f;
  motion_sticks.forward=sign;velocity(sign*.59f);
  for(unsigned n=0;n<200;++n){step();checked();}
  float before_gate=g_balance.motion_lean_rad;
  velocity(sign*.61f);hold=g_balance.parking_lean_rad;
  for(unsigned n=0;n<100;++n){step();checked();}
  assert(sign*g_balance.motion_lean_rad<=sign*before_gate+1e-7f);
  for(unsigned neutral=0;neutral<2;++neutral){
   if(neutral)motion_sticks.forward=0;
   velocity(sign*.78f);
   for(unsigned n=0;n<200;++n){
    float previous=g_balance.motion_lean_rad;step();checked();
    assert(!g_balance.motion_lean_learning && g_balance.parking_lean_rad==hold);
    assert(fabsf(g_balance.motion_lean_rad-previous)<=ROBOT_MOTION_FAST_LEAN_RATE_RAD_S*.005f+1e-7f);
    assert(fabsf(g_balance.motion_lean_rad)<=ROBOT_MOTION_FAST_LEAN_LIMIT_RAD+1e-7f);
   }
   assert(sign*g_balance.motion_lean_rad<-.017f);
   assert(sign*g_balance.motion_damping_torque_nm<0);
   if(neutral)assert(g_balance.motion_braking && !g_balance.position_locked);
  }
  /* Overspeed is not permission to bypass the actuator health gate. */
  for(unsigned n=0;n<100;++n){g_joint_output.saturated=1;step();}
  assert(fabsf(g_balance.motion_lean_rad)<1e-7f);
 }
 /* g3: opposing leg errors during reversal need the faster sync slew,
  * while matched motion, small split and startup retain the ordinary rate. */
 for(unsigned mirror=0;mirror<2;++mirror){
  float sign=mirror ? -1.0f : 1.0f;
  for(unsigned matched=0;matched<2;++matched){
   off();motion_sticks.forward=motion_sticks.yaw=0;imu.gyro_rad_s[2]=0;
   velocity(0);pose(0);pose(1);on();center();
   assert(!g_balance.startup_recovery);
   if(matched){motion_sticks.forward=sign;velocity(sign*.4f);for(unsigned n=0;n<200;++n)step();}
   float before=g_dual_hold.sync_torque_nm[0];
   swing_feedback(0,.095f,1.57079632679f-sign*.2f,0);
   swing_feedback(1,.095f,1.57079632679f+sign*.2f,0);
   velocity(sign*.4f);motion_sticks.forward=matched ? sign : -sign;
   step();checked();
   float delta=sign*(g_dual_hold.sync_torque_nm[0]-before);
   float expected=g_dual_hold.sync_rate_nm_s*.005f*(matched?1.0f:LEG_SYNC_DECEL_RATE_SCALE);
   assert(fabsf(delta-expected)<1e-6f);
   assert(fabsf(g_dual_hold.sync_torque_nm[0])<=g_dual_hold.sync_ratio*g_dual_hold.sync_limit_nm);
  }
  off();motion_sticks.forward=motion_sticks.yaw=0;velocity(0);pose(0);pose(1);on();center();
  swing_feedback(0,.095f,1.57079632679f-sign*.04f,0);
  swing_feedback(1,.095f,1.57079632679f+sign*.04f,0);
  velocity(sign*.4f);motion_sticks.forward=-sign;step();checked();
  assert(fabsf(sign*g_dual_hold.sync_torque_nm[0]-g_dual_hold.sync_rate_nm_s*.005f)<1e-6f);
  off();motion_sticks.forward=0;velocity(0);
  swing_feedback(0,.095f,1.57079632679f-sign*.2f,0);
  swing_feedback(1,.095f,1.57079632679f+sign*.2f,0);
  on();assert(g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE);
  assert(fabsf(sign*g_dual_hold.sync_torque_nm[0]-g_dual_hold.sync_rate_nm_s*.005f)<1e-6f);
 }
 off();pose(0);pose(1);motion_sticks.forward=0;velocity(0);on();center();
 puts("PASS: stronger signed velocity correction, preserved startup damping, conditional reversal sync slew and matched/small-split/startup exclusions");
 imu.gyro_rad_s[2]=0;velocity(0);center();for(unsigned n=0;n<100;++n)step();
 assert(!g_balance.motion_damping_active && g_balance.motion_damping_torque_nm==0);
 assert(g_balance.position_locked && !g_balance.motion_stall_hold);
 off();
 assert(g_balance.parking_lean_rad==0 && g_balance.motion_damping_torque_nm==0);
 assert(!g_balance.motion_lean_learning && !g_balance.motion_stall_hold);
 assert(g_robot_motion.filtered_yaw_rate_rad_s==0 && g_balance.motion_lean_rad==0);
 printf("PASS: prescribed g23 reverse error changes shared reference %g -> %g rad; bounded and correctly signed (not a physical simulation)\n",start,reduced);
 puts("PASS: neutral yaw rejection, gyro noise filter, matched/nonzero speed equilibrium, reverse correction, saturation/stall antiwindup, recovery, normal-turn learning, excessive-yaw freeze and OFF reset");
}
'''
sources=['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c',
 'App/Services/Src/motor_command.c','App/Services/Src/lqr_startup_record.c',
 'tests/support/wheel_leg/Src/wheel_leg_control.c','Control/WheelLeg/Src/lqr_k.c',
 'Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
with tempfile.TemporaryDirectory(prefix='motion_tracking_') as folder:
 p=Path(folder);(p/'runtime.h').write_text(base['runtime'])
 for n in ['main.h','FreeRTOS.h','task.h','bsp_fdcan.h']:(p/n).write_text('#include "runtime.h"\n')
 (p/'test.c').write_text(prefix+helpers+main)
 cmd=['gcc','-DROBOT_NATIVE_LQR=0','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function']
 for inc in [p,root/'App/Tasks/Inc',root/'App/Services/Inc',root/'App/Protocol/Inc',
             root/'Control/WheelLeg/Inc',root/'tests/support/wheel_leg/Inc']:cmd+=['-I',str(inc)]
 for recovery in (0,1):
  exe=p/f'test_recovery{recovery}.exe'
  subprocess.run(cmd+[f'-DROBOT_MOTION_STARTUP_RECOVERY_ENABLE={recovery}',str(p/'test.c'),*[str(root/s) for s in sources],'-lm','-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
