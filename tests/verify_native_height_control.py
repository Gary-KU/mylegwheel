"""Native height PI through the real task, geometry, A1 master and current caps.

Only CAN peers and prescribed sensor feedback are fake. These assertions verify
integrator state and physical current allocation, not height convergence or
free-standing stability. LQR coefficients are not fixed by this test.
The coupled servo and motion governor retain their current defaults. The CAN
fixture and allocation checks use the configured/calibrated current limits.
"""
import ast
from pathlib import Path
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
historical = root / "tests/verify_native_support_release.py"
source = historical.read_text(encoding="utf-8")
scope = {"__file__": str(historical)}
exec(compile(source.split("fixture = r'''")[0], str(historical), "exec"), scope)
fixture = next(ast.literal_eval(node.value) for node in ast.parse(source).body
               if isinstance(node, ast.Assign)
               and any(isinstance(target, ast.Name) and target.id == "fixture"
                       for target in node.targets))
fixture = fixture.split("int main(void) {")[0]

checks = r'''
static void height_zero(void) {
 for(unsigned side=0;side<2;++side)assert(g_leg_height_integral_n[side]==0);
}
static void same_integral(float expected) {
 for(unsigned side=0;side<2;++side)
  assert(fabsf(g_leg_height_integral_n[side]-expected)<1e-5f);
}
static void start_height(float length,float target) {
 imu=(ImuSample){0};g_dual_hold.target_m=target;
 both_pose(length,0);watch_off();height_zero();g_robot_control.enable=1;
 for(unsigned n=0;n<120;++n){current_step();running();}
 assert(g_dual_hold.feedforward_n==9);
}
static void learn_positive(unsigned cycles) {
 start_height(.07f,.07f);height_zero();g_dual_hold.target_m=.11f;
 for(unsigned n=0;n<cycles;++n){current_step();running();}
 assert(g_leg_height_integral_n[0]>.1f && g_leg_height_integral_n[1]>.1f);
}
static float current_cap(unsigned index) {
 return fminf(JOINT_TORQUE_LIMIT_NM,
  MotorCommand_CurrentTorqueLimitNm((uint8_t)g_joint_calibration[index].motor_id)*
  g_joint_calibration[index].ratio);
}
int main(void) {
 assert(ROBOT_NATIVE_LQR && g_dual_hold.speed_kp==30 && g_dual_hold.speed_ki==80);
 assert(LEG_HEIGHT_INTEGRAL_LIMIT_N==4.5f && LEG_HEIGHT_INTEGRAL_RATE_N_S==3);
 assert(g_dual_hold.position_kp==5 && g_dual_hold.max_speed_m_s==.04f);
 assert(g_dual_hold.feedforward_target_n==9 && g_dual_hold.feedforward_rate_n_s==24);
 assert(g_dual_hold.overlength_kp_n_per_m==0 && g_dual_hold.max_force_n==15);
 hfdcan1.Instance=&can_hw;MotorFoc_Init();
 for(unsigned i=0;i<8;++i) {
  a1_flags[i]=0x3e;motors[i].online=motors[i].joint_calibrated=1;
 }
 for(unsigned n=0;n<80;++n){now+=5;feed_current_nodes();MotorFoc_Service(NULL,0);}
 imu=(ImuSample){0};imu_ok=1;g_robot_control.remote_enable=0;

 /* The default overlength correction grows only once the actual support
  * ramp reaches its readiness tolerance; each accepted step is <=.015N. */
 both_pose(.10f,0);watch_off();height_zero();g_robot_control.enable=1;
 unsigned gated=0,learned=0;
 for(unsigned n=0;n<500;++n) {
  float previous[2]={g_leg_height_integral_n[0],g_leg_height_integral_n[1]};
  current_step();running();
  for(unsigned side=0;side<2;++side) {
   float bias=g_leg_height_integral_n[side];
   assert(fabsf(bias)<=4.500001f && fabsf(bias-previous[side])<=.015001f);
   if(fabsf(g_dual_hold.feedforward_n-9)>LEG_HEIGHT_INTEGRAL_SUPPORT_READY_N) {
    assert(bias==0);++gated;
   } else if(bias<previous[side])++learned;
  }
 }
 assert(gated && learned);same_integral(-4.5f);
 watch_off();height_zero();
 puts("PASS: real task default Kp30/Ki80, support-ramp gate, 3N/s increments, -4.5N bias cap and STOP reset");

 learn_positive(500);same_integral(4.5f);watch_off();height_zero();
 start_height(.10f,.07f);
 float frozen[2]={g_leg_height_integral_n[0],g_leg_height_integral_n[1]};
 assert(frozen[0]<-.1f && frozen[0]>-4);
 g_dual_hold.feedforward_target_n=12;g_dual_hold.feedforward_rate_n_s=1;
 for(unsigned n=0;n<80;++n) {
  current_step();running();
  for(unsigned side=0;side<2;++side)assert(g_leg_height_integral_n[side]==frozen[side]);
 }
 g_dual_hold.feedforward_target_n=9;g_dual_hold.feedforward_rate_n_s=24;
 for(unsigned n=0;n<40;++n){current_step();running();}
 assert(g_leg_height_integral_n[0]<frozen[0]);
 puts("PASS: +4.5N bias cap; temporarily unavailable feedforward freezes and retains learned compensation");

 /* Ki=0 must clear even when dropping a positive bias increases a lower
  * axial-limit violation, which ordinary conditional integration rejects. */
 learn_positive(70);g_dual_hold.speed_ki=0;both_pose(.07f,.5f);
 current_step();running();height_zero();
 for(unsigned side=0;side<2;++side)
  assert(g_dual_hold.force_requested_n[side]<0 && g_dual_hold.force_n[side]==0);
 g_dual_hold.speed_ki=80;watch_off();height_zero();
 puts("PASS: Ki=0 explicitly clears an existing positive bias under lower-force saturation");

 start_height(.07f,.07f);height_zero();both_pose(.07f,-.5f);
 for(unsigned n=0;n<20;++n){current_step();running();height_zero();}
 for(unsigned side=0;side<2;++side)
  assert(g_dual_hold.force_saturated[side] && g_dual_hold.force_n[side]==g_dual_hold.force_limit_n[side]);
 both_pose(.07f,.5f);
 for(unsigned n=0;n<20;++n){current_step();running();height_zero();}
 for(unsigned side=0;side<2;++side)
  assert(g_dual_hold.force_saturated[side] && g_dual_hold.force_n[side]==0);

 /* Lower one actual A1 node's current allowance to make its physical
  * axial cap binding, then allow a negative increment to unwind +bias. */
 learn_positive(70);g_dual_hold.target_m=.07f;both_pose(.07f,.02f);
 unsigned limited=g_joint_calibration[0].motor_id-1;
 g_motor_foc.calibration[limited].current_limit_a=.15f;
 float before=g_leg_height_integral_n[0];
 for(unsigned n=0;n<40;++n){current_step();running();}
 assert(g_dual_hold.force_limit_n[0]<5 && g_dual_hold.force_saturated[0]);
 assert(g_leg_height_integral_n[0]<before);
 g_motor_foc.calibration[limited].current_limit_a=MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A;
 puts("PASS: real axial upper/lower clamps block windup; reduced current headroom permits bias unwinding");

 /* Sweep prescribed body rates until exactly one current-limited joint
  * is constrained by mixed F/Tp while pure axial force remains feasible.
  * Derive the assertion from present geometry/caps, never a fixed LQR gain. */
 unsigned blocked=0,unwound=0;
 const float rates[]={.1f,-.1f,.3f,-.3f,1,-1,3,-3,6,-6,10,-10,
                      15,-15,20,-20,30,-30,50,-50,100,-100};
 for(unsigned direction=0;direction<2;++direction)
 for(unsigned r=0;r<sizeof(rates)/sizeof(rates[0]);++r) {
  learn_positive(40);
  both_pose(.07f,direction ? .10f : 0);
  float prior[2]={g_leg_height_integral_n[0],g_leg_height_integral_n[1]};
  imu.gyro_rad_s[BODY_PITCH_AXIS]=rates[r]/BODY_PITCH_DIRECTION;
  current_step();running();
  for(unsigned side=0;side<2;++side) {
   float actual=g_leg_height_integral_n[side];
   float delta=direction ? -.015f : .015f;
   float proposed=prior[side]+delta;
   float base=g_dual_hold.force_requested_n[side]-actual;
   float previous_force=base+prior[side],candidate_force=base+proposed;
   if(previous_force<=0 || candidate_force<=0 ||
      previous_force>=g_dual_hold.force_limit_n[side] ||
      candidate_force>=g_dual_hold.force_limit_n[side])continue;
   unsigned clipped=0,worsened=0;float max_old=0,max_new=0;
   for(unsigned joint=0;joint<2;++joint) {
    unsigned index=2*side+joint;
    float jf=g_wheel_leg.leg[side].jacobian[1-joint];
    float jt=g_wheel_leg.leg[side].jacobian[3-joint];
    float tp=g_dual_hold.pitch_torque_nm[side],cap=current_cap(index);
    float old=fmaxf(0,fabsf(jf*previous_force+jt*tp)-cap);
    float next=fmaxf(0,fabsf(jf*candidate_force+jt*tp)-cap);
    clipped+=old>1e-5f;
    worsened+=next>old+1e-7f;
    max_old=fmaxf(max_old,old);max_new=fmaxf(max_new,next);
   }
   if(clipped!=1)continue;
   if(worsened) {assert(fabsf(actual-prior[side])<1e-6f);++blocked;}
   else if(max_old>1e-5f && max_new<max_old-1e-7f) {
    assert(fabsf(actual-proposed)<1e-6f);++unwound;
   }
  }
 }
 if(!blocked || !unwound)fprintf(stderr,"mixed counts blocked=%u unwound=%u\n",blocked,unwound);
 assert(blocked && unwound);watch_off();height_zero();
 printf("PASS: mixed J^T current-cap predictions blocked %u harmful and accepted %u helpful increments\n",blocked,unwound);
 puts("Real task/current integration only: no physical height-convergence or standing-stability claim.");
 return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="native_height_control_") as directory:
    work = Path(directory)
    (work / "runtime.h").write_text(scope["runtime"], encoding="utf-8")
    for name in ("main.h", "FreeRTOS.h", "task.h", "bsp_fdcan.h"):
        (work / name).write_text('#include "runtime.h"\n', encoding="utf-8")
    (work / "test.c").write_text(scope["prefix"] + fixture + checks, encoding="utf-8")
    sources = ["App/Tasks/Src/wheel_leg_task.c", "App/Tasks/Src/wheel_leg_observer.c",
               "App/Services/Src/motor_foc.c", "App/Services/Src/motor_command.c",
               "BSP/Peripherals/Src/bsp_fdcan_deadline.c", "App/Protocol/Src/robot_motion_command.c",
               "tests/support/wheel_leg/Src/wheel_leg_control.c", "Control/WheelLeg/Src/lqr_k.c",
               "Control/WheelLeg/Src/leg_pos.c", "tests/support/wheel_leg/Src/leg_trajectory.c"]
    command = ["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
               "-DROBOT_NATIVE_LQR=1", "-DMYLEGWHEEL_A1_CURRENT=1",
               "-DROBOT_LEGS_ONLY_CALIBRATION=0", "-I", str(work)]
    for folder in ("App/Tasks/Inc", "App/Services/Inc", "App/Protocol/Inc", "BSP/Peripherals/Inc",
                   "Control/WheelLeg/Inc", "tests/support/wheel_leg/Inc"):
        command += ["-I", str(root / folder)]
    exe = work / "test.exe"
    subprocess.run(command + [str(work / "test.c"), *[str(root / item) for item in sources],
                              "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
