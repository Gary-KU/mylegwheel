"""Verify baseline compatibility coupled-servo arithmetic, permissions and reset lifecycle.

The production task, observer, gain lookup, motion governor and output adapter
are compiled unchanged. Feedback is prescribed independently of commands, so
this checks implementation, not physical balance or braking. The reusable
governor sensor fixture first disables LQI; setup() explicitly restores the
production servo default before any active update. No controller-default
compile overrides are used here. Plant-loop validation is a separate test.
"""
import ast
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def literal_strings(path):
    return {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in ast.parse(path.read_text(encoding="utf-8")).body
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Constant)
        and isinstance(node.value.value, str)
    }


strings = literal_strings(ROOT / "tests/verify_wheel_leg_control.py")
prefix = strings["harness"].split("int main(void) {")[0]
governor = literal_strings(ROOT / "tests/verify_native_motion_governor.py")["fixture"]
helpers = governor.split("static void neutral_stop(float sign)")[0]
fixture = r'''
#include "balance_servo_gains.h"
#include "balance_parking_gains.h"
static float clip(float value,float limit) {return fmaxf(-limit,fminf(limit,value));}
static void setup(unsigned startup) {
 configure(startup);
 g_balance_servo.enable=BALANCE_NATIVE_SERVO_ENABLE;
 assert(g_balance_servo.enable==1 && g_balance_motion_reference.enable==1);
 assert(!g_balance_bias.enable && !g_balance_parking_integral.enable);
}
static void servo_cleared(void) {
 stopped();
 assert(!g_balance_servo.active && !g_balance_servo.learning && !g_balance_servo.gate_mask);
 assert(g_balance_servo.blend==0 && g_balance_servo.integral_m_s==0);
 assert(g_balance_servo.wheel_integral_nm==0 && g_balance_servo.leg_integral_nm==0);
 assert(g_balance_servo.parking_blend==0);
}
static void checked(void) {
 float z_before=g_balance_servo.integral_m_s,blend_before=g_balance_servo.blend;
 float parking_before=g_balance_servo.parking_blend;
 step();
 assert(g_robot_control.enable && g_balance.status==1 && g_dual_hold.status==1);
 assert(!g_balance_bias.applied_phi_rad && !g_balance_bias.applied_offset_m);
 float K[12],servo_K[12],Ki[2];
 lqr_k(g_balance.leg_length_m,K);
 assert(BalanceServo_Gain(g_balance.leg_length_m,servo_K,Ki));
 float parking_K[12],parking_Ki[2];
 assert(BalanceParking_Gain(g_balance.leg_length_m,parking_K,parking_Ki));
 float parking=g_balance_servo.parking_blend;
 assert(parking>=0 && parking<=1);
 assert(fabsf(parking-parking_before)<=.005f/BALANCE_PARKING_BLEND_S+1e-6f);
 for(unsigned i=0;i<12;++i)servo_K[i]+=parking*(parking_K[i]-servo_K[i]);
 for(unsigned row=0;row<2;++row)Ki[row]+=parking*(parking_Ki[row]-Ki[row]);
 float blend=g_balance_servo.active ?
     fminf(1,blend_before+.005f/BALANCE_SERVO_BLEND_S) : 0;
 assert(fabsf(g_balance_servo.blend-blend)<1e-7f);
 if(g_balance_servo.active)
  for(unsigned i=0;i<12;++i)K[i]+=blend*(servo_K[i]-K[i]);
 double expected[2]={0,g_balance_equilibrium_tp_nm};
 for(unsigned row=0;row<2;++row) {
  for(unsigned i=0;i<6;++i)expected[row]-=K[2*i+row]*(g_balance.state[i]-g_balance.reference[i]);
  if(g_balance_servo.active)expected[row]-=blend*Ki[row]*z_before;
  if(row==1)expected[row]=g_balance.leg_scale*(expected[row]-g_balance_equilibrium_tp_nm)+g_balance_equilibrium_tp_nm;
  assert(fabs(g_balance.torque_nm[row]-expected[row])<1e-6);
 }
 float wheel_i=g_balance_servo.active ? -blend*Ki[0]*z_before : 0;
 float leg_i=g_balance_servo.active ? -g_balance.leg_scale*blend*Ki[1]*z_before : 0;
 assert(fabsf(g_balance_servo.wheel_integral_nm-wheel_i)<1e-7f);
 assert(fabsf(g_balance_servo.leg_integral_nm-leg_i)<1e-7f);
 /* Verify both actually wired outputs, independently of their debug copies. */
 assert(fabsf(g_wheel_output.requested_torque_nm-g_balance.torque_nm[0])<1e-6f);
 for(unsigned side=0;side<2;++side)
  assert(fabsf(g_dual_hold.lqr_pitch_torque_nm[side]-g_balance.torque_nm[1])<1e-6f);
 float z_next=z_before;
 if(g_balance_servo.learning) {
  assert(g_balance_servo.gate_mask==31U);
  z_next=clip(z_before+clip(g_balance.parking_error_m,BALANCE_SERVO_INTEGRAL_RATE_M)*.005f,
              BALANCE_SERVO_INTEGRAL_LIMIT_M_S);
 }
 assert(fabsf(g_balance_servo.integral_m_s-z_next)<1e-7f);
 if(g_balance_motion_reference.phase!=BALANCE_MOTION_TRACKING)
  assert(g_balance.reference[2]==g_balance.parking_target_m);
}
static void ready(void) {
 for(unsigned n=0;n<200;++n)checked();
 assert(g_balance_servo.active && g_balance_servo.learning && g_balance_servo.blend==1);
 assert(g_balance_servo.parking_blend==1);
 assert(g_balance_servo.integral_m_s==0);
}
static void learn(float sign) {
 setup(0);ready();
 float goal=g_balance.parking_target_m;
 velocity(sign*.2f);
 for(unsigned n=0;n<80;++n){checked();fixed_goal(goal);}
 velocity(0);
 for(unsigned n=0;n<200;++n){checked();fixed_goal(goal);}
 assert(sign*g_balance_servo.integral_m_s>.06f);
 assert(g_balance_servo.learning);
 assert(fabsf(g_balance_servo.wheel_integral_nm)>1e-5f &&
        fabsf(g_balance_servo.leg_integral_nm)>1e-6f);
}
static void freeze_saturation(unsigned kind) {
 float z=g_balance_servo.integral_m_s;
 for(unsigned n=0;n<20;++n) {
  if(kind==0)g_wheel_output.saturated=1;
  if(kind==1)g_wheel_output.torque_saturated=1;
  if(kind==2)g_joint_output.saturated=1;
  if(kind==3)g_dual_hold.force_saturated[0]=1;
  if(kind==4)g_dual_hold.force_saturated[1]=1;
  checked();
  assert(!(g_balance_servo.gate_mask&16U) && !g_balance_servo.learning);
  assert(g_balance_servo.integral_m_s==z);
 }
}
static void tracking_window_roundoff(float sign) {
 setup(0);ready();
 motion_sticks.forward=sign;
 for(unsigned n=0;n<250;++n)checked();
 assert(g_balance_motion_reference.phase==BALANCE_MOTION_TRACKING);
 velocity(sign*.20f);
 unsigned at_window=0,strict_outside=0;
 for(unsigned n=0;n<1800;++n) {
  checked();
  if(n<300)continue;
  assert(g_balance_motion_reference.phase==BALANCE_MOTION_TRACKING);
  float error=fabsf(g_balance.state[2]-g_balance.parking_target_m);
  if(fabsf(error-ROBOT_MOTION_TRACK_WINDOW_M)<1e-6f) {
   ++at_window;
   if(error>ROBOT_MOTION_TRACK_WINDOW_M)++strict_outside;
   assert(g_balance_servo.gate_mask&8U);
   assert(g_balance_servo.learning);
  }
 }
 /* Use the real observer/governor at metre-scale travel: no direct goal or
  * position injection. Each direction must actually encounter float values
  * that a strict <=5cm comparison would incorrectly reject. */
 assert(at_window>100 && strict_outside>0);
 disarm();servo_cleared();
}
static void genuine_parking_window_exit(float sign) {
 setup(0);ready();
 float goal=g_balance.parking_target_m;
 velocity(sign*.20f);
 for(unsigned n=0;n<600;++n){checked();fixed_goal(goal);}
 velocity(0);checked();fixed_goal(goal);
 /* Approach a genuine parked-window overrun with small independent sensor
  * motion pulses. Tens of micrometres must remain outside; the rounding
  * tolerance must not become a physical enlargement of the .6m window. */
 for(unsigned n=0;n<8 && fabsf(g_balance.parking_error_m)<
       BALANCE_SERVO_POSITION_WINDOW_M+.00004f;++n) {
  velocity(sign*.01f);checked();fixed_goal(goal);
  velocity(0);checked();fixed_goal(goal);
 }
 float error=fabsf(g_balance.parking_error_m);
 assert(error>BALANCE_SERVO_POSITION_WINDOW_M+.00003f);
 assert(error<BALANCE_SERVO_POSITION_WINDOW_M+.00015f);
 assert(g_balance_motion_reference.phase==BALANCE_MOTION_PARKED);
 assert(!(g_balance_servo.gate_mask&8U) && !g_balance_servo.learning);
 float retained=g_balance_servo.integral_m_s;
 for(unsigned n=0;n<20;++n) {
  checked();fixed_goal(goal);
  assert(!(g_balance_servo.gate_mask&8U) && !g_balance_servo.learning);
  assert(g_balance_servo.integral_m_s==retained);
  assert(g_balance.reference[2]==goal);
 }
 disarm();servo_cleared();
}
int main(void) {
 assert(ROBOT_NATIVE_LQR && BALANCE_NATIVE_SERVO_ENABLE==1 && g_balance_servo.enable==1);
 assert(BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE==0 && g_balance_bias.enable==0);
 assert(g_balance_motion_reference.enable==1 && BALANCE_MOTION_REFERENCE_ENABLE==1);
 assert(BALANCE_SERVO_BLEND_S==.5f && BALANCE_SERVO_POSITION_WINDOW_M==.6f);
 assert(BALANCE_SERVO_INTEGRAL_LIMIT_M_S==2 && BALANCE_SERVO_INTEGRAL_RATE_M==.3f);
 setup(0);
 for(unsigned n=0;n<110;++n) {
  checked();
  if(g_balance_servo.blend<1)assert(!g_balance_servo.learning);
 }
 assert(g_balance_servo.blend==1 && g_balance_servo.learning);
 puts("PASS: production-default servo/bias/governor ownership, 0.5s blend and complete two-row feedback");

 tracking_window_roundoff(1);tracking_window_roundoff(-1);
 genuine_parking_window_exit(1);genuine_parking_window_exit(-1);
 puts("PASS: signed metre-scale tracking at5cm survives float roundoff; genuine parked-window overruns still freeze I and retain the true P goal");

 /* Accepted forward-neutral intent selects the complete parking rows before
  * coasting stops. Turning does not postpone this sagittal handover; the
  * governor independently preserves its brake/reversal safety conditions. */
 for(int sign=-1;sign<=1;sign+=2) {
  setup(0);ready();
  motion_sticks.forward=(float)sign;
  for(unsigned n=0;n<180;++n)checked();
  assert(g_balance_motion_reference.phase==BALANCE_MOTION_TRACKING);
  assert(g_balance_servo.parking_blend==0);
  float saved_z=g_balance_servo.integral_m_s;
  motion_sticks.forward=0;velocity(sign*.20f);
  checked();
  assert(g_balance_motion_reference.phase==BALANCE_MOTION_BRAKING);
  assert(sign*g_balance.reference[3]>.1f); /* Internal ramp is still driving. */
  assert(fabsf(g_balance.state[3]-sign*.20f)<1e-6f);
  assert(fabsf(g_balance_servo.parking_blend-.005f/BALANCE_PARKING_BLEND_S)<1e-6f);
  assert(fabsf(g_balance_servo.integral_m_s-saved_z)<=BALANCE_SERVO_INTEGRAL_RATE_M*.005f+1e-6f);
  float fixed=g_balance.parking_target_m;
  motion_sticks.yaw=.6f;
  for(unsigned n=0;n<110;++n) {
   checked();fixed_goal(fixed);
   assert(g_robot_motion.command.requested_turn_rad_s>.02f);
   assert(g_balance_motion_reference.phase==BALANCE_MOTION_BRAKING);
   assert(g_balance_motion_reference.quiet_s==0);
  }
  assert(g_balance_servo.parking_blend==1);
  assert(g_balance.reference[3]==0); /* Reference stopped; robot still coasts. */

  /* Both new forward signs fade back to driving gains without releasing a
   * pending reversal before measured speed qualifies. z and goal persist. */
  saved_z=g_balance_servo.integral_m_s;
  motion_sticks.forward=(float)-sign;
  checked();fixed_goal(fixed);
  assert(fabsf(g_balance_servo.parking_blend-(1-.005f/BALANCE_PARKING_BLEND_S))<1e-6f);
  assert(fabsf(g_balance_servo.integral_m_s-saved_z)<=BALANCE_SERVO_INTEGRAL_RATE_M*.005f+1e-6f);
  for(unsigned n=0;n<110;++n) {
   checked();fixed_goal(fixed);
   assert(g_balance_motion_reference.phase==BALANCE_MOTION_BRAKING);
   assert(g_balance.reference[3]==0 && g_balance_motion_reference.quiet_s==0);
  }
  assert(g_balance_servo.parking_blend==0);
  motion_sticks.forward=0;checked();fixed_goal(fixed);
  assert(g_balance_servo.parking_blend>0);
  velocity(0);
  for(unsigned n=0;n<130;++n){checked();fixed_goal(fixed);}
  assert(g_balance_motion_reference.phase==BALANCE_MOTION_PARKED);
  assert(g_balance_servo.parking_blend==1);
  disarm();servo_cleared();
 }
 puts("PASS: forward-neutral begins both parking rows while BRAKING at0.2m/s; yaw does not delay it, new signed forward fades out, reversal dwell/z/fixed goal remain");

 /* A slow intentional command is not neutral. This also detects using the
  * old0.01m/s threshold after removing the phase/yaw waits. */
 for(int sign=-1;sign<=1;sign+=2) {
  setup(0);ready();
  motion_sticks.forward=sign*.009f/ROBOT_MOTION_DEFAULT_FORWARD_M_S;
  checked();
  assert(fabsf(g_robot_motion.command.requested_forward_m_s-sign*.009f)<1e-6f);
  assert(g_balance_servo.parking_blend<1 && g_balance_servo.parking_blend>0);
  for(unsigned n=0;n<110;++n)checked();
  assert(g_balance_motion_reference.phase==BALANCE_MOTION_TRACKING);
  assert(g_balance_servo.parking_blend==0);
  motion_sticks.valid=0;motion_sticks.forward=0;
  for(unsigned n=0;n<130;++n) {
   checked();assert(g_robot_motion.command.status!=ROBOT_MOTION_TARGET_READY);
   assert(g_balance_servo.parking_blend==0);
  }
  disarm();servo_cleared();
 }
 puts("PASS: signed0.009m/s commands retain driving gains; rejected input cannot start a parking handover");

 for(int sign=-1;sign<=1;sign+=2) {
  learn((float)sign);
  g_balance_servo.integral_m_s=sign*(BALANCE_SERVO_INTEGRAL_LIMIT_M_S-.00001f);
  checked();
  assert(g_balance_servo.integral_m_s==sign*BALANCE_SERVO_INTEGRAL_LIMIT_M_S);
  checked();
  assert(g_balance_servo.integral_m_s==sign*BALANCE_SERVO_INTEGRAL_LIMIT_M_S);
  disarm();servo_cleared();
 }
 puts("PASS: signed I accumulation/caps; outputs use z_k before state advances to z_(k+1), through actual wheel/hip wiring");

 learn(1);
 for(unsigned kind=0;kind<5;++kind)freeze_saturation(kind);
 for(unsigned n=0;n<3;++n)checked();
 assert(g_balance_servo.learning);
 float retained=g_balance_servo.integral_m_s,goal=g_balance.parking_target_m;
 motion_sticks.valid=0;
 for(unsigned n=0;n<30;++n) {
  float parking_before=g_balance_servo.parking_blend;
  checked();fixed_goal(goal);
  assert(!(g_balance_servo.gate_mask&2U) && !g_balance_servo.learning);
  assert(g_balance_servo.integral_m_s==retained);
  assert(g_balance_servo.parking_blend<=parking_before);
 }
 motion_sticks.valid=1;
 for(unsigned n=0;n<60;++n)checked();
 assert(g_balance_servo.learning);
 retained=g_balance_servo.integral_m_s;
 pose(17);
 for(unsigned n=0;n<5;++n) {
  checked();assert(!(g_balance_servo.gate_mask&8U) && !g_balance_servo.learning);
  assert(g_balance_servo.integral_m_s==retained);
 }
 pose(0);
 puts("PASS: wheel/torque/joint/axial saturation, invalid motion input and large attitude freeze I without erasing learned torque");

 /* The true goal and proportional position error are unbounded in parking.
  * The old 0.3m cap and the 0.6m I eligibility gate must not clip P feedback. */
 velocity(.2f);
 for(unsigned n=0;n<800;++n) {
  checked();fixed_goal(goal);
  if(g_balance.parking_error_m>.3f)assert(g_balance.reference[2]==goal);
 }
 velocity(0);checked();
 assert(g_balance.parking_error_m>.8f && g_balance.reference[2]==goal);
 retained=g_balance_servo.integral_m_s;
 for(unsigned n=0;n<30;++n) {
  checked();fixed_goal(goal);
  assert(!(g_balance_servo.gate_mask&8U) && !g_balance_servo.learning);
  assert(g_balance_servo.integral_m_s==retained);
  assert(g_balance.reference[2]==goal);
 }
 disarm();servo_cleared();
 puts("PASS: beyond 0.3m and 0.6m parking uses the exact fixed goal; the I gate freezes while P remains uncropped");

 setup(1);pose(12);checked();goal=g_balance.parking_target_m;
 velocity(.1f);
 for(unsigned n=0;n<160;++n) {
  checked();fixed_goal(goal);
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  assert(g_balance_servo.parking_blend==0);
  assert(!g_balance_servo.active && g_balance_servo.integral_m_s==0 && g_balance_servo.blend==0);
 }
 pose(0);velocity(0);unsigned saw_ramp=0,saw_servo=0;
 for(unsigned n=0;n<600;++n) {
  checked();fixed_goal(goal);
  if(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP) {
   saw_ramp=1;
   assert(g_balance_servo.parking_blend==0);
   assert(!g_balance_servo.active && !g_balance_servo.learning && g_balance_servo.integral_m_s==0);
  }
  if(g_balance_servo.active)saw_servo=1;
 }
 assert(saw_ramp && saw_servo && g_balance_servo.blend==1);
 disarm();servo_cleared();
 puts("PASS: PREPARE/RAMP keep original feedback and zero I; only established balance blends in servo");

 /* Exercise fail-closed ownership and state validity with an already learned
  * correction; failure must clear both I and pending governor motion. */
 for(unsigned kind=0;kind<10;++kind) {
  learn(1);
  if(kind==0)g_balance_bias.enable=1;
  if(kind==1)g_balance_parking_integral.enable=1;
  if(kind==2)g_balance_servo.enable=2;
  if(kind==3)g_balance_servo.integral_m_s=NAN;
  if(kind==4)motors[JOINT_LF_CAN_ID-1].online=0;
  if(kind==5)imu_ok=0;
  if(kind==6)g_balance_servo.parking_blend=NAN;
  if(kind==7)g_balance_servo.parking_blend=1.1f;
  if(kind==8)g_balance.leg_scale=NAN;
  if(kind==9)g_balance.leg_scale=2.1f;
  step();servo_cleared();
  assert(g_robot_control.last_fault.reason==
         (kind<4 || kind>=6 ? ROBOT_FAULT_PARAMETERS : ROBOT_FAULT_FEEDBACK));
  g_balance_servo.enable=1;g_balance_bias.enable=g_balance_parking_integral.enable=0;
  g_balance.leg_scale=BALANCE_DEFAULT_LEG_SCALE;
  motors[JOINT_LF_CAN_ID-1].online=1;imu_ok=1;
 }
 puts("PASS: conflicting correction owners, invalid enable/NaN state and missing feedback STOP and clear servo/governor state");
 puts("LIMIT: independent prescribed feedback verifies implementation; it does not establish physical stability or parking performance");
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="native_servo_") as folder:
        work = Path(folder)
        (work / "runtime.h").write_text(strings["runtime"], encoding="utf-8")
        for name in ("main.h", "FreeRTOS.h", "task.h", "bsp_fdcan.h"):
            (work / name).write_text('#include "runtime.h"\n', encoding="utf-8")
        (work / "test.c").write_text(prefix + helpers + fixture, encoding="utf-8")
        command = [
            "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
            "-DROBOT_NATIVE_LQR=1", "-DROBOT_LEGS_ONLY_CALIBRATION=0", "-DMOTOR_COMMAND_TX_ENABLED=1",
            "-DBALANCE_NATIVE_POSITION_INTEGRAL_ENABLE=1", "-DBALANCE_NATIVE_DRIFT_COMPENSATION_ENABLE=0",
            "-I", str(work),
        ]
        for folder in (
            "App/Tasks/Inc", "App/Services/Inc", "App/Protocol/Inc",
            "Control/WheelLeg/Inc", "tests/support/wheel_leg/Inc",
        ):
            command += ["-I", str(ROOT / folder)]
        sources = [
            "App/Protocol/Src/robot_motion_command.c", "App/Tasks/Src/wheel_leg_task.c",
            "App/Tasks/Src/wheel_leg_observer.c", "App/Services/Src/motor_command.c",
            "tests/support/wheel_leg/Src/wheel_leg_control.c", "Control/WheelLeg/Src/lqr_k.c",
            "Control/WheelLeg/Src/leg_pos.c", "tests/support/wheel_leg/Src/leg_trajectory.c",
        ]
        executable = work / "test.exe"
        subprocess.run(
            command + [str(work / "test.c"), *[str(ROOT / source) for source in sources],
                       "-lm", "-o", str(executable)],
            check=True,
        )
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
