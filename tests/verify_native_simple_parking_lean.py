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

int main(void) {
 assert(BALANCE_NATIVE_POSITION_INTEGRAL_ENABLE==0);
 assert(BALANCE_NATIVE_SERVO_ENABLE==1 && BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE==0);
 assert(BALANCE_DEFAULT_LEG_SCALE==1.5f && BALANCE_PARKING_LEAN_FILTER_S==.30f);
 for(int sign=-1;sign<=1;sign+=2) {
  configure(0);g_balance_servo.enable=1;arm();
  float goal=g_balance.parking_target_m,phi=g_balance.reference[4],ff=g_balance_equilibrium_tp_nm;
  g_joint_output.saturated=1;step();g_joint_output.saturated=0;
  velocity(sign*.04f);
  for(unsigned n=0;n<199;++n){step();assert(g_balance.parking_lean_rad==0);}
  for(unsigned n=0;n<500;++n) {
   float prev=g_balance.parking_lean_rad;step();
   assert(sign*g_balance.parking_lean_rad<=0);
   assert(fabsf(g_balance.parking_lean_rad-prev)<=BALANCE_PARKING_LEAN_RATE_RAD_S*.005f+1e-8f);
   assert(fabsf(g_balance.parking_lean_rad)<=BALANCE_PARKING_LEAN_LIMIT_RAD+1e-7f);
   assert(fabsf(g_balance.reference[0]-g_balance_equilibrium_theta_rad-g_balance.parking_lean_rad)<1e-7f);
   assert(g_balance.reference[4]==phi && g_balance_equilibrium_tp_nm==ff);
   assert(g_balance_servo.integral_m_s==0 && g_balance_servo.wheel_integral_nm==0 && g_balance_servo.leg_integral_nm==0);
   assert(g_balance.parking_target_m==goal && g_balance.leg_scale==1.5f && g_balance_servo.enable==1);
  }
  printf("lean=%g learn=%u enable=%u gain=%g phase=%u sat=%u/%u mode=%u v=%g win=%g\n",g_balance.parking_lean_rad,g_balance.parking_lean_learning,g_balance.parking_lean_enable,g_balance.parking_lean_gain_rad_per_m,g_balance_motion_reference.phase,g_wheel_output.saturated,g_joint_output.saturated,g_dual_hold.angle_reference_mode,g_balance.state[3],g_balance.position_window_m);
  assert(sign*g_balance.parking_lean_rad<-.001f);
  float held=g_balance.parking_lean_rad;
  velocity(sign*.20f);for(unsigned n=0;n<50;++n){step();assert(g_balance.parking_lean_rad==held && !g_balance.parking_lean_learning);}
  velocity(0);for(unsigned n=0;n<199;++n){step();assert(g_balance.parking_lean_rad==held);}
  for(unsigned n=0;n<3000;++n)step(); // Filtered deadband leaves no hidden continuing drift.
  held=g_balance.parking_lean_rad;for(unsigned n=0;n<100;++n){step();assert(g_balance.parking_lean_rad==held);}
  // Saturation freezes, disable and OFF clear immediately.
  g_joint_output.saturated=1;step();assert(!g_balance.parking_lean_learning);g_joint_output.saturated=0;
  g_balance.parking_lean_enable=0;step();assert(g_balance.parking_lean_rad==0);
  g_balance.parking_lean_enable=1;disarm();assert(g_balance.parking_lean_rad==0);
 }
 // Long drift caps the actual state, then opposite drift unwinds immediately.
 configure(0);g_balance_servo.enable=1;arm();velocity(.1f);
 for(unsigned n=0;n<3000;++n){g_wheel_output.saturated=g_wheel_output.torque_saturated=g_joint_output.saturated=0;step();}
 assert(fabsf(g_balance.parking_lean_rad+BALANCE_PARKING_LEAN_LIMIT_RAD)<1e-7f);
 // Prescribe unsaturated prior-output telemetry to isolate cap/unwind logic.
 velocity(-.1f);for(unsigned n=0;n<500;++n){g_wheel_output.saturated=g_wheel_output.torque_saturated=g_joint_output.saturated=0;step();}
 assert(g_balance.parking_lean_rad>-BALANCE_PARKING_LEAN_LIMIT_RAD+.001f);
 disarm();
 puts("PASS: production1531 feedback/gain1.5 retained, position-I zero; signed drift,1s dwell, filter/deadband, actual state cap/unwind, saturation/disable/OFF, theta-only/fixed goal contract");
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
