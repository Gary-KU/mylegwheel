"""Original bounded-parking-correction compatibility through the host task.

Sensor feedback is prescribed independently of controller output. These checks
verify command arithmetic, permissions and lifecycle, not grounded stability.
Compiles BALANCE_NATIVE_SERVO_ENABLE=0 and BALANCE_MOTION_REFERENCE_ENABLE=0:
this older leaky correction and release-capture lifecycle are retained for
compatibility, not asserted as the new default LQI/governor behavior.
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

static const float dt=.005f,rad=.017453292519943295f;
static float clip(float x,float limit) {return fmaxf(-limit,fminf(limit,x));}
static float deadband(float x,float band) {
 return fabsf(x)<=band ? 0 : x-copysignf(band,x);
}
static void velocity(float v) {
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-v/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=v/WHEEL_RADIUS_M;
}
static void pose(float body_deg,float right_difference) {
 float theta,tp;
 BalanceEquilibrium_Calculate(.10f,4*rad,&theta,&tp);
 imu.attitude_deg[BODY_PITCH_AXIS]=body_deg/BODY_PITCH_DIRECTION;
 for(unsigned side=0;side<2;++side)
  swing_feedback(side,.10f,1.5707963267948966f+4*rad+theta+
                 (side?right_difference:0),0);
}
static void stopped(void) {
 assert(!g_robot_control.enable && !g_balance.enable && !g_dual_hold.enable);
 assert(!g_balance_parking_integral.learning && !g_balance_parking_integral.gate_mask);
 assert(!g_balance_parking_integral.settle_ms && g_balance_parking_integral.correction_m==0);
 for(unsigned group=0;group<2;++group)for(unsigned i=0;i<8;++i)assert(data[group][i]==0);
}
static void disarm(void) {
 remote_sample.frame.channel[4]=1000;step();stopped();
}
static void configure(unsigned startup) {
 disarm();
 for(unsigned i=0;i<8;++i) {
  motors[i].online=motors[i].joint_calibrated=1;
  motors[i].age_ms=0;
 }
 imu=(ImuSample){0};imu_ok=1;pose(-4,0);velocity(0);
 remote_ok=remote_fresh=1;
 motion_sticks=(SbusSticks){.valid=1,.calibrated=1,.raw_ch2=1000};
 g_robot_control.remote_enable=1;g_robot_control.startup_enable=startup;
 g_robot_control.feedback_only=0;
 g_dual_hold.target_m=.10f;g_dual_hold.sync_ratio=0;
 g_dual_hold.feedforward_target_n=9;g_dual_hold.feedforward_rate_n_s=24;
 g_balance.pitch_trim_deg=-4;
 g_balance_bias.enable=g_balance_bias.shadow_enable=0;
 g_balance_parking_integral.enable=1;
 remote_sample.frame.channel[4]=1000;step();stopped();
 remote_sample.frame.channel[4]=306;
}
static void reference_and_outputs(void) {
 assert(g_robot_control.enable && g_balance.status==1 && g_dual_hold.status==1);
 assert(g_balance.pitch_trim_deg==-4); /* Preserve the user's manual trim. */
 float c=g_balance_parking_integral.correction_m;
 assert(isfinite(c) && fabsf(c)<=.500001f);
 float window=fabsf(g_balance.reference[3])>1e-6f ?
     ROBOT_MOTION_TRACK_WINDOW_M : BALANCE_NATIVE_PARKING_WINDOW_M;
 float error=g_balance.state[2]-g_balance.parking_target_m;
 float base_x=g_balance.state[2]-clip(error,window);
 assert(fabsf(g_balance.reference[2]-(base_x-c))<2e-6f);
 assert(fabsf(g_balance.parking_error_m-error)<2e-6f);
 float theta,tp;
 assert(BalanceEquilibrium_CalculateWithOffset(g_balance.leg_length_m,
     g_balance.reference[4],g_balance_bias.applied_offset_m,&theta,&tp));
 float K[12];lqr_k(g_balance.leg_length_m,K);
 double with_c[2]={0,tp},without_c[2]={0,tp};
 for(unsigned i=0;i<6;++i)for(unsigned row=0;row<2;++row) {
  double ref=g_balance.reference[i];
  with_c[row]-=K[2*i+row]*(g_balance.state[i]-ref);
  if(i==2)ref=base_x;
  without_c[row]-=K[2*i+row]*(g_balance.state[i]-ref);
 }
 for(unsigned row=0;row<2;++row) {
  assert(fabs(g_balance.torque_nm[row]-with_c[row])<1e-6);
  assert(fabs((with_c[row]-without_c[row])+K[4+row]*c)<1e-6);
 }
 assert(fabsf(g_wheel_output.requested_torque_nm-g_balance.torque_nm[0])<1e-6f);
 for(unsigned side=0;side<2;++side)
  assert(fabsf(g_dual_hold.lqr_pitch_torque_nm[side]-g_balance.torque_nm[1])<1e-6f);
 assert(!g_balance.motion_damping_active && !g_balance.parking_damping_active);
}
static void checked_step(void) {
 float before=g_balance_parking_integral.correction_m;
 step();reference_and_outputs();
 float after=g_balance_parking_integral.correction_m;
 float expected=before;
 if(!g_balance_parking_integral.enable)expected=0;
 else if(g_balance_parking_integral.learning) {
  assert(g_balance_parking_integral.gate_mask==63U);
  assert(g_balance_parking_integral.settle_ms>=300U);
  float increment=clip((.20f*deadband(g_balance.parking_error_m,.01f)-
                        .20f*before)*dt,.10f*dt);
  expected=clip(before+increment,.50f);
 }
 assert(fabsf(after-expected)<2e-7f);
 if(g_balance_parking_integral.enable)
  assert(fabsf(after-before)<=.10f*dt+2e-7f);
}
static void arm(void) {
 for(unsigned n=0;n<2000;++n)checked_step();
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
 assert(g_robot_motion.neutral_ready && g_balance.position_locked);
 assert(g_balance_parking_integral.gate_mask==63 && g_balance_parking_integral.learning);
 assert(g_balance_parking_integral.correction_m==0);
 assert(fabsf(g_balance.pitch_trim_applied_deg+4)<1e-5f);
}
static void displace(float distance) {
 /* Move only the prescribed wheel feedback; the physical parking goal is
  * captured before this movement and must never follow it. */
 float goal=g_balance.parking_target_m;
 g_balance_parking_integral.enable=0;checked_step();
 velocity(copysignf(.20f,distance));
 unsigned frames=(unsigned)(fabsf(distance)/(.20f*dt)+.5f);
 for(unsigned n=0;n<frames;++n) {
  checked_step();assert(fabsf(g_balance.parking_target_m-goal)<1e-6f);
 }
 velocity(0);checked_step();checked_step();
 assert(g_balance_parking_integral.correction_m==0);
 g_balance_parking_integral.enable=1;
}
static void fresh_dwell(void) {
 assert(!g_balance_parking_integral.settle_ms);
 for(unsigned n=1;n<=60;++n) {
  checked_step();assert(g_balance_parking_integral.gate_mask==63U);
  assert(g_balance_parking_integral.settle_ms==n*5U);
  if(n<60)assert(!g_balance_parking_integral.learning);
 }
 assert(g_balance_parking_integral.learning);
}
static void regain(void) {
 for(unsigned n=0;n<300 && !g_balance_parking_integral.learning;++n)checked_step();
 assert(g_balance_parking_integral.learning && g_balance_parking_integral.gate_mask==63);
}
static void freeze(unsigned frames,unsigned missing_bit,unsigned saturation_kind) {
 float before=g_balance_parking_integral.correction_m;
 for(unsigned n=0;n<frames;++n) {
  if(saturation_kind==1)g_wheel_output.saturated=1;
  if(saturation_kind==2)g_wheel_output.torque_saturated=1;
  if(saturation_kind==3)g_joint_output.saturated=1;
  if(saturation_kind==4)g_dual_hold.force_saturated[0]=1;
  if(saturation_kind==5)g_dual_hold.force_saturated[1]=1;
  checked_step();
  assert(!(g_balance_parking_integral.gate_mask&missing_bit));
  assert(!g_balance_parking_integral.learning && !g_balance_parking_integral.settle_ms);
  assert(g_balance_parking_integral.correction_m==before);
 }
}
static void signed_accumulation(float sign) {
 configure(0);arm();displace(sign*.08f);
 float goal=g_balance.parking_target_m;
 fresh_dwell();
 for(unsigned n=0;n<400;++n) {
  checked_step();assert(fabsf(g_balance.parking_target_m-goal)<1e-6f);
 }
 assert(sign*g_balance_parking_integral.correction_m>.020f);
 /* True errors above .51m must reach the .1m/s rate limit, rather than
  * integrating the separately capped .3m LQR position input. */
 displace(sign*.80f);fresh_dwell();
 assert(sign*g_balance.parking_error_m>.85f);
 for(unsigned n=0;n<1200;++n) {
  checked_step();assert(fabsf(g_balance.parking_target_m-goal)<1e-6f);
 }
 assert(fabsf(g_balance_parking_integral.correction_m-sign*.50f)<1e-6f);
 for(unsigned n=0;n<50;++n)checked_step();
 assert(fabsf(g_balance_parking_integral.correction_m-sign*.50f)<1e-6f);
 g_balance_parking_integral.enable=0;checked_step();
 assert(!g_balance_parking_integral.learning && g_balance_parking_integral.correction_m==0);
 assert(fabsf(g_balance.parking_target_m-goal)<1e-6f);
 disarm();
}
int main(void) {
 assert(ROBOT_NATIVE_LQR==1 && BALANCE_DEFAULT_PITCH_TRIM_DEG==-4);
 assert(BALANCE_NATIVE_PARKING_INTEGRAL_GAIN_S==.20f &&
        BALANCE_NATIVE_PARKING_INTEGRAL_LEAK_S==.20f);
 assert(g_balance_parking_integral.enable==BALANCE_NATIVE_PARKING_INTEGRAL_ENABLE);
 assert(!g_balance_parking_integral.learning && g_balance_parking_integral.correction_m==0);
 signed_accumulation(1);signed_accumulation(-1);
 puts("PASS: both parking-error signs, 300ms dwell, true-error gain, .1m/s rate/.5m cap, fixed goal and both LQR rows");

 configure(0);arm();displace(.08f);fresh_dwell();
 for(unsigned n=0;n<100;++n)checked_step();
 assert(g_balance_parking_integral.correction_m>.006f);
 /* Input loss freezes learning; it does not silently learn from zeroed
  * commands after the accepted coherent remote frame becomes invalid. */
 motion_sticks.valid=0;freeze(20,4U,0);motion_sticks.valid=1;regain();
 motion_sticks.calibrated=0;freeze(20,4U,0);motion_sticks.calibrated=1;regain();
 motion_sticks.yaw=.2f;freeze(20,4U,0);motion_sticks.yaw=0;regain();
 velocity(.4f);freeze(20,8U,0);velocity(0);regain();
 imu.gyro_rad_s[BODY_PITCH_AXIS]=.4f/BODY_PITCH_DIRECTION;
 freeze(20,8U,0);imu.gyro_rad_s[BODY_PITCH_AXIS]=0;regain();
 pose(8,0);freeze(20,8U,0);pose(-4,0);regain();
 pose(-4,.12f);freeze(20,8U,0);pose(-4,0);regain();
 g_dual_hold.feedforward_target_n=12;g_dual_hold.feedforward_rate_n_s=.1f;
 freeze(20,16U,0);g_dual_hold.feedforward_target_n=9;g_dual_hold.feedforward_rate_n_s=24;
 regain();
 for(unsigned kind=1;kind<=5;++kind) {freeze(20,32U,kind);regain();}
 g_balance_bias.enable=1;freeze(20,1U,0);g_balance_bias.enable=0;regain();
 puts("PASS: invalid/uncalibrated/turning input, speed/rate/attitude, asymmetric pose, support, all caps and observer ownership freeze learning");

 /* Driving retains the learned correction and its coupled T/Tp effect;
  * the driving trajectory may move while the physical parking goal is
  * captured once on return to neutral. */
 float retained=g_balance_parking_integral.correction_m;
 motion_sticks.forward=.5f;
 for(unsigned n=0;n<120;++n) {
  checked_step();assert(!g_balance.position_locked);
  assert(!g_balance_parking_integral.learning && g_balance_parking_integral.correction_m==retained);
 }
 assert(g_balance.reference[3]>.19f);
 motion_sticks.forward=0;
 for(unsigned n=0;n<100 && !g_balance.position_locked;++n) {
  checked_step();assert(g_balance_parking_integral.correction_m==retained);
 }
 assert(g_balance.position_locked && g_balance.reference[3]==0);
 float release_goal=g_balance.parking_target_m;
 for(unsigned n=0;n<150;++n) {
  checked_step();
  if(g_balance_parking_integral.settle_ms<300)
   assert(g_balance_parking_integral.correction_m==retained);
  assert(g_balance_parking_integral.correction_m<=retained &&
         g_balance_parking_integral.correction_m>0);
  assert(fabsf(g_balance.parking_target_m-release_goal)<1e-6f);
 }
 assert(g_balance_parking_integral.correction_m<retained); /* Leak resumes only when qualified. */
 disarm();
 puts("PASS: driving freezes coupled compensation; neutral recaptures once and requalifies leakage; OFF clears it");

 configure(0);arm();displace(.005f);fresh_dwell();
 for(unsigned n=0;n<200;++n)checked_step();
 assert(fabsf(g_balance.parking_error_m)<.01f && g_balance_parking_integral.correction_m==0);
 displace(.08f);fresh_dwell();for(unsigned n=0;n<100;++n)checked_step();
 motors[JOINT_LF_CAN_ID-1].online=0;step();stopped();
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
 motors[JOINT_LF_CAN_ID-1].online=1;
 configure(0);arm();displace(-.08f);fresh_dwell();for(unsigned n=0;n<100;++n)checked_step();
 imu_ok=0;step();stopped();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
 imu_ok=1;
 puts("PASS: deadband, missing motor feedback and lost IMU preserve STOP and clear all integral state");

 configure(1);pose(12,0);velocity(.1f);
 for(unsigned n=0;n<200;++n) {
  checked_step();assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  assert(!(g_balance_parking_integral.gate_mask&2U));
  assert(!g_balance_parking_integral.learning && g_balance_parking_integral.correction_m==0);
 }
 disarm();
 puts("PASS: PREPARE cannot learn parking correction; host checks do not claim physical parking stability");
}
'''

with tempfile.TemporaryDirectory(prefix='native_parking_integral_') as directory:
    work = Path(directory)
    (work / 'runtime.h').write_text(strings['runtime'])
    for name in ('main.h', 'FreeRTOS.h', 'task.h', 'bsp_fdcan.h'):
        (work / name).write_text('#include "runtime.h"\n')
    (work / 'test.c').write_text(prefix + fixture)
    command = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
               '-DROBOT_NATIVE_LQR=1', '-DROBOT_LEGS_ONLY_CALIBRATION=0', '-DMOTOR_COMMAND_TX_ENABLED=1',
               '-DBALANCE_NATIVE_SERVO_ENABLE=0', '-DBALANCE_MOTION_REFERENCE_ENABLE=0',
               '-I', str(work)]
    for folder in ('App/Tasks/Inc', 'App/Services/Inc', 'App/Protocol/Inc', 'Control/WheelLeg/Inc',
                   'tests/support/wheel_leg/Inc'):
        command += ['-I', str(ROOT / folder)]
    sources = ['App/Protocol/Src/robot_motion_command.c', 'App/Tasks/Src/wheel_leg_task.c',
               'App/Tasks/Src/wheel_leg_observer.c', 'App/Services/Src/motor_command.c',
               'tests/support/wheel_leg/Src/wheel_leg_control.c', 'Control/WheelLeg/Src/lqr_k.c',
               'Control/WheelLeg/Src/leg_pos.c', 'tests/support/wheel_leg/Src/leg_trajectory.c']
    exe = work / 'test.exe'
    subprocess.run(command + [str(work / 'test.c'), *[str(ROOT / source) for source in sources],
                             '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
