"""Exercise the production task's feedback-gated motion reference lifecycle.

Wheel and IMU feedback are prescribed independently of controller output.
These are command/permission checks, not a prediction of physical braking or
evidence that the robot can balance, stop, or follow a ground trajectory.
The fixture explicitly disables the coupled servo to isolate reference
lifecycle checks. The default servo is covered by verify_native_servo.py and
the separate plant-loop screen; this suite does not replace those checks.
"""
import ast
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT / "tests/verify_wheel_leg_control.py").read_text(encoding="utf-8"))
strings = {
    node.targets[0].id: ast.literal_eval(node.value)
    for node in tree.body
    if isinstance(node, ast.Assign)
    and isinstance(node.targets[0], ast.Name)
    and isinstance(node.value, ast.Constant)
    and isinstance(node.value.value, str)
}
prefix = strings["harness"].split("int main(void) {")[0]
fixture = r'''
#include "balance_motion_reference.h"

static unsigned dwell_frames(void) {
 unsigned frames=(unsigned)ceilf(BALANCE_MOTION_STOP_DWELL_S/.005f-1e-5f);
 assert(frames>0);return frames;
}
static void velocity(float v) {
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-v/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=v/WHEEL_RADIUS_M;
}
static void pose(float body_deg) {
 imu.attitude_deg[BODY_PITCH_AXIS]=body_deg/BODY_PITCH_DIRECTION;
 for(unsigned side=0;side<2;++side)swing_feedback(side,.10f,1.5707963267948966f,0);
}
static void stopped(void) {
 assert(!g_robot_control.enable && !g_balance.enable && !g_dual_hold.enable);
 assert(g_balance_motion_reference.phase==BALANCE_MOTION_PARKED);
 assert(g_balance_motion_reference.velocity_m_s==0 && g_balance_motion_reference.goal_m==0);
 assert(g_balance_motion_reference.quiet_s==0);
 assert(!g_balance.position_locked && !g_balance.motion_braking);
 for(unsigned group=0;group<2;++group)for(unsigned i=0;i<8;++i)assert(data[group][i]==0);
}
static void disarm(void) {
 remote_sample.frame.channel[4]=1000;
 for(unsigned n=0;n<3;++n){step();stopped();}
}
static void configure(unsigned startup) {
 disarm();
 for(unsigned i=0;i<8;++i) {
  motors[i].online=motors[i].joint_calibrated=1;
  motors[i].age_ms=0;
 }
 imu=(ImuSample){0};imu_ok=1;pose(0);velocity(0);
 remote_ok=remote_fresh=1;
 motion_sticks=(SbusSticks){.valid=1,.calibrated=1,.raw_ch2=1000};
 g_robot_control.remote_enable=1;g_robot_control.startup_enable=startup;
 g_robot_control.feedback_only=0;
 g_dual_hold.target_m=.10f;g_dual_hold.sync_ratio=0;
 g_dual_hold.feedforward_target_n=9;g_dual_hold.feedforward_rate_n_s=24;
 g_balance.pitch_trim_deg=0;
 g_balance_bias.enable=g_balance_bias.shadow_enable=0;
 g_balance_parking_integral.enable=0;
 g_balance_servo.enable=0; /* Exercise reference lifecycle independently of LQI. */
 g_balance_motion_reference.enable=1;
 g_robot_motion.config.enable=1;
 g_robot_motion.config.max_forward_m_s=ROBOT_MOTION_DEFAULT_FORWARD_M_S;
 remote_sample.frame.channel[4]=1000;step();stopped();
 remote_sample.frame.channel[4]=306;
}
static void active_step(void) {
 float before=g_balance.reference[3];
 step();
 assert(g_robot_control.enable && g_balance.status==1 && g_dual_hold.status==1);
 assert(g_balance.reference[3]==g_balance_motion_reference.velocity_m_s);
 assert(g_balance.parking_target_m==g_balance_motion_reference.goal_m);
 assert(fabsf(g_balance.reference[3])<=ROBOT_MOTION_DEFAULT_FORWARD_M_S+1e-6f);
 assert(fabsf(g_balance.reference[3]-before)<=
        BALANCE_MOTION_BRAKE_ACCELERATION_M_S2*.005f+1e-6f);
}
static void arm(void) {
 for(unsigned n=0;n<800;++n)active_step();
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
 assert(g_robot_motion.neutral_ready && g_balance.position_locked);
 assert(g_balance_motion_reference.phase==BALANCE_MOTION_PARKED);
}
static void cruise(float sign) {
 configure(0);arm();
 motion_sticks.forward=sign;
 for(unsigned n=0;n<250;++n)active_step();
 assert(g_balance_motion_reference.phase==BALANCE_MOTION_TRACKING);
 assert(fabsf(g_balance.reference[3]-sign*.4f)<1e-6f);
 assert(!g_balance.position_locked);
}
static void fixed_goal(float goal) {
 assert(g_balance.parking_target_m==goal);
 assert(g_balance_motion_reference.goal_m==goal);
}
static void neutral_stop(float sign) {
 cruise(sign);
 float goal=g_balance.parking_target_m;
 motion_sticks.forward=0;velocity(sign*.25f);
 active_step();fixed_goal(goal);
 assert(g_robot_motion.command.requested_forward_m_s==0);
 assert(sign*g_robot_motion.command.forward_m_s>.3f); /* Upstream still moving. */
 assert(g_balance.motion_braking && g_balance_motion_reference.phase==BALANCE_MOTION_BRAKING);
 for(unsigned n=0;n<240;++n) {
  active_step();fixed_goal(goal);
  assert(sign*g_balance.reference[3]>=0 && g_balance.motion_braking);
 }
 assert(g_balance.reference[3]==0 && g_balance.position_locked);
 assert(g_balance_motion_reference.quiet_s==0);
 assert(fabsf(g_balance.state[2]-goal)>.20f); /* Goal did not follow measured coasting. */
 velocity(0);
 for(unsigned n=0;n+1<dwell_frames();++n) {
  active_step();fixed_goal(goal);assert(g_balance.motion_braking);
  assert(fabsf(g_balance_motion_reference.quiet_s-(n+1)*.005f)<1e-6f);
 }
 active_step();fixed_goal(goal);
 assert(g_balance_motion_reference.phase==BALANCE_MOTION_PARKED);
 for(unsigned n=0;n<80;++n){active_step();fixed_goal(goal);assert(g_balance.reference[3]==0);}
 disarm();
}
static void reversal(float sign) {
 cruise(sign);
 float goal=g_balance.parking_target_m;
 motion_sticks.forward=-sign;velocity(sign*.25f);
 for(unsigned n=0;n<240;++n) {
  active_step();fixed_goal(goal);
  assert(g_balance_motion_reference.phase==BALANCE_MOTION_BRAKING);
  assert(sign*g_balance.reference[3]>=0);
 }
 assert(-sign*g_robot_motion.command.forward_m_s>.39f);
 assert(g_balance.reference[3]==0 && g_balance.position_locked);
 /* Upstream has already reversed; feedback alone holds the new reference. */
 velocity(sign*BALANCE_MOTION_STOP_SPEED_M_S*.95f);
 for(unsigned n=0;n+1<dwell_frames();++n) {
  active_step();fixed_goal(goal);assert(g_balance.motion_braking);
  assert(fabsf(g_balance_motion_reference.quiet_s-(n+1)*.005f)<1e-6f);
 }
 velocity(-sign*BALANCE_MOTION_STOP_SPEED_M_S*1.05f);active_step();fixed_goal(goal);
 assert(g_balance_motion_reference.quiet_s==0 && g_balance.motion_braking);
 velocity(-sign*BALANCE_MOTION_STOP_SPEED_M_S*.95f);
 for(unsigned n=0;n+1<dwell_frames();++n) {
  active_step();fixed_goal(goal);assert(g_balance.motion_braking);
  assert(fabsf(g_balance_motion_reference.quiet_s-(n+1)*.005f)<1e-6f);
 }
 active_step();fixed_goal(goal);
 assert(g_balance_motion_reference.phase==BALANCE_MOTION_TRACKING);
 assert(g_balance.reference[3]==0); /* Completion must not release a stored .4m/s target. */
 active_step();
 assert(fabsf(g_balance.reference[3]+sign*.002f)<1e-7f);
 disarm();
}
static void permissions(void) {
 /* A rejected motion source must retain the pre-existing immediate-zero
  * contract. It must not turn rejection into a neutral-intent brake ramp.
  * Preserving the parking goal also matters while startup is NOT_READY. */
 for(unsigned kind=0;kind<4;++kind) {
  cruise(1);float goal=g_balance.parking_target_m;
  if(kind==0)motion_sticks.valid=0;
  if(kind==1)motion_sticks.calibrated=0;
  if(kind==2)g_robot_motion.config.enable=0;
  if(kind==3)motion_sticks.forward=NAN;
  step();
  assert(g_robot_control.enable && g_balance.status==1 && g_dual_hold.status==1);
  assert(g_robot_motion.command.status!=ROBOT_MOTION_TARGET_READY);
  assert(g_robot_motion.command.forward_m_s==0);
  assert(g_balance.reference[3]==0 && g_balance_motion_reference.velocity_m_s==0);
  assert(g_balance_motion_reference.phase==BALANCE_MOTION_PARKED);
  assert(g_balance_motion_reference.quiet_s==0);fixed_goal(goal);
  for(unsigned n=0;n<20;++n){step();fixed_goal(goal);assert(g_balance.reference[3]==0);}
  disarm();
 }
}
int main(void) {
 assert(ROBOT_NATIVE_LQR && BALANCE_MOTION_REFERENCE_ENABLE==1);
 assert(g_balance_servo.enable==BALANCE_NATIVE_SERVO_ENABLE);
 assert(BALANCE_MOTION_ACCELERATION_M_S2==.4f && BALANCE_MOTION_BRAKE_ACCELERATION_M_S2==.8f);
 neutral_stop(1);neutral_stop(-1);
 puts("PASS: both neutral stops use raw intent, measured dwell and fixed parking targets through the production task");
 reversal(1);reversal(-1);
 puts("PASS: both reversals wait for actual low speed, reset noisy dwell and restart without a target jump");

 /* Startup NOT_READY is not a repeated new activation and cannot drag the
  * neutral parking goal along with prescribed, uncommanded wheel motion. */
 configure(1);pose(12);active_step();
 float goal=g_balance.parking_target_m;
 velocity(.10f);
 for(unsigned n=0;n<160;++n) {
  active_step();fixed_goal(goal);
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  assert(g_robot_motion.command.status!=ROBOT_MOTION_TARGET_READY);
  assert(g_balance.reference[3]==0 && g_balance.position_locked);
 }
 assert(g_balance.parking_error_m>.07f);
 disarm();
 puts("PASS: startup keeps its original goal through NOT_READY while uncommanded feedback moves");

 /* Stop while a direction is pending. A new session must start from zero,
  * without a stale reference, pending reversal, dwell or parking origin. */
 cruise(1);motion_sticks.forward=-1;velocity(.2f);active_step();
 assert(g_balance.motion_braking);disarm();
 configure(0);arm();assert(g_balance.reference[3]==0);
 for(unsigned kind=0;kind<2;++kind) {
  cruise(kind ? -1 : 1);motion_sticks.forward=kind ? 1 : -1;velocity(.2f);active_step();
  if(kind==0)motors[JOINT_LF_CAN_ID-1].online=0;
  else imu_ok=0;
  step();stopped();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
  if(kind==0)motors[JOINT_LF_CAN_ID-1].online=1;
  else imu_ok=1;
 }
 cruise(1);g_balance_motion_reference.enable=2;step();stopped();
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 g_balance_motion_reference.enable=1;
 puts("PASS: CH5 OFF, motor/IMU loss and invalid governor settings keep STOP and clear pending motion");

 permissions();
 puts("PASS: invalid, uncalibrated, disabled and nonfinite motion sources clear velocity immediately and retain the parking goal");
 puts("LIMIT: prescribed feedback checks task behavior, not physical stopping or grounded stability");
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="native_motion_governor_") as folder:
        work = Path(folder)
        (work / "runtime.h").write_text(strings["runtime"], encoding="utf-8")
        for name in ("main.h", "FreeRTOS.h", "task.h", "bsp_fdcan.h"):
            (work / name).write_text('#include "runtime.h"\n', encoding="utf-8")
        (work / "test.c").write_text(prefix + fixture, encoding="utf-8")
        command = [
            "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
            "-DROBOT_NATIVE_LQR=1", "-DROBOT_LEGS_ONLY_CALIBRATION=0", "-DMOTOR_COMMAND_TX_ENABLED=1",
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
