"""Exercise the real task/observer at the measured startup-pose boundary.

Feedback is prescribed independently of output. The unchanged original trim
update is compiled as a negative control; these tests verify scheduling and
readiness, not whether the physical robot reaches the prescribed pose.
"""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
def strings(path):
    return {n.targets[0].id:ast.literal_eval(n.value) for n in ast.parse(path.read_text(encoding='utf-8')).body
            if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name)
            and isinstance(n.value,ast.Constant) and isinstance(n.value.value,str)}
base=strings(ROOT/'tests/verify_wheel_leg_control.py')
prefix=base['harness'].split('int main(void) {')[0]
helpers=strings(ROOT/'tests/verify_native_motion_governor.py')['fixture'].split('static void neutral_stop(float sign)')[0]
fixture=r'''
static const float deg=.017453292519943295f;
static void boundary_pose(float right_deg) {
 imu.attitude_deg[BODY_PITCH_AXIS]=0;imu.gyro_rad_s[BODY_PITCH_AXIS]=0;
 swing_feedback(0,.07f,92*deg,0);swing_feedback(1,.07f,right_deg*deg,0);
 velocity(0);
}
static void setup_boundary(unsigned startup,float right_deg) {
 configure(startup);
 g_balance_servo.enable=BALANCE_NATIVE_SERVO_ENABLE;
 g_dual_hold.target_m=.07f;
 g_dual_hold.sync_ratio=LEG_DEFAULT_SYNC_RATIO;
 g_balance.pitch_trim_deg=-4;
 boundary_pose(right_deg);
}
static void valid_step(void) {
 step();
 assert(g_robot_control.enable && g_balance.status==1 && g_dual_hold.status==1);
 assert(g_wheel_leg.leg_valid[0] && g_wheel_leg.leg_valid[1]);
}
int main(void) {
 setup_boundary(1,85.3f);
 unsigned balance_tick=0;
 float previous_trim=0;
 for(unsigned n=0;n<2400;++n) {
  boundary_pose(85.3f);valid_step();
  assert(fabsf(g_wheel_leg.leg[0].angle/deg-92)<.001f);
  assert(fabsf(g_wheel_leg.leg[1].angle/deg-85.3f)<.001f);
  if(n<49)assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
#if EXPECT_TRIM_DEFER
  assert(g_balance.pitch_trim_applied_deg<=previous_trim+1e-6f);
  assert(previous_trim-g_balance.pitch_trim_applied_deg<=BALANCE_PITCH_TRIM_RATE_DEG_S*.005f+1e-6f);
  previous_trim=g_balance.pitch_trim_applied_deg;
  if(g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE)
   assert(g_balance.pitch_trim_applied_deg==0);
  else if(!balance_tick)balance_tick=n+1;
#else
  (void)previous_trim;
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  assert(!g_balance_servo.active && g_balance_servo.blend==0);
#endif
 }
#if EXPECT_TRIM_DEFER
 assert(balance_tick>49 && balance_tick<300);
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_balance_servo.active);
#else
 assert(!balance_tick);
#endif
 assert(fabsf(g_balance.pitch_trim_applied_deg+4)<1e-5f);
 disarm();
 assert(g_balance.pitch_trim_applied_deg==0 && g_balance_servo.blend==0);

 setup_boundary(1,80);
 for(unsigned n=0;n<800;++n) {
  boundary_pose(80);valid_step();
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_PREPARE);
  assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_POSE));
  assert(!g_balance_servo.active);
#if EXPECT_TRIM_DEFER
  assert(g_balance.pitch_trim_applied_deg==0);
#endif
 }
 disarm();assert(g_balance.pitch_trim_applied_deg==0);

 setup_boundary(0,85.3f);
 float prior=0;
 for(unsigned n=0;n<200;++n) {
  boundary_pose(85.3f);valid_step();
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
  float trim=g_balance.pitch_trim_applied_deg;
  assert(trim<prior && fabsf(trim-prior)<=BALANCE_PITCH_TRIM_RATE_DEG_S*.005f+1e-6f);
  prior=trim;
 }
 assert(fabsf(g_balance.pitch_trim_applied_deg+.5f)<1e-5f);
 disarm();assert(g_balance.pitch_trim_applied_deg==0);
#if EXPECT_TRIM_DEFER
 printf("PASS: zero-trim preparation qualifies85.3deg right/92deg left after retained dwell; BALANCE at tick%u then original0.5deg/s trim,80deg remains blocked, OFF resets, startup-disabled path retained\n",balance_tick);
#else
 puts("PASS negative control: original moving trim keeps85.3deg right/92deg left in PREPARE for12s;80deg blocked; startup-disabled path retained");
#endif
}
'''

OLD='''    balance_pitch_trim+=clamp_symmetric(c.pitch_trim_deg-balance_pitch_trim,
        BALANCE_PITCH_TRIM_RATE_DEG_S*state->period_ms*.001f);'''
NEW='''    /* Qualify the initial pose at zero manual trim. Moving the equilibrium
     * during PREPARE can outrun a leg near the5deg readiness boundary before
     * its reference slew and250ms dwell complete. Full T/Tp remain active;
     * the configured trim starts at the original rate only after BALANCE. */
    if(robot_control_active && g_robot_control.startup_enable &&
       g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE)
        balance_pitch_trim=0;
    else balance_pitch_trim+=clamp_symmetric(c.pitch_trim_deg-balance_pitch_trim,
        BALANCE_PITCH_TRIM_RATE_DEG_S*state->period_ms*.001f);'''

def main():
    source_path=ROOT/'App/Tasks/Src/wheel_leg_task.c'
    task=source_path.read_text(encoding='utf-8');assert task.count(NEW)==1
    outputs=[]
    with tempfile.TemporaryDirectory(prefix='startup_trim_') as folder:
        work=Path(folder)
        (work/'runtime.h').write_text(base['runtime'])
        for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):(work/name).write_text('#include "runtime.h"\n')
        (work/'test.c').write_text(prefix+helpers+fixture)
        for fixed in (0,1):
            tested=work/f'task_{fixed}.c';tested.write_text(task if fixed else task.replace(NEW,OLD))
            cmd=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',
                 '-DROBOT_NATIVE_LQR=1','-DROBOT_LEGS_ONLY_CALIBRATION=0','-DMOTOR_COMMAND_TX_ENABLED=1',
                 f'-DEXPECT_TRIM_DEFER={fixed}','-I',str(work)]
            for name in ('App/Tasks/Inc','App/Services/Inc','App/Protocol/Inc','Control/WheelLeg/Inc','tests/support/wheel_leg/Inc'):cmd+=['-I',str(ROOT/name)]
            sources=['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_command.c',
                     'tests/support/wheel_leg/Src/wheel_leg_control.c','Control/WheelLeg/Src/lqr_k.c','Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
            exe=work/f'test_{fixed}.exe'
            subprocess.run(cmd+[str(work/'test.c'),str(tested),*[str(ROOT/s) for s in sources],'-lm','-o',str(exe)],check=True)
            run=subprocess.run([str(exe)],capture_output=True,text=True,check=True);print(run.stdout,end='');outputs.append(dict(fixed=bool(fixed),stdout=run.stdout))
    report=dict(scope=__doc__,source_sha256=hashlib.sha256(source_path.read_bytes()).hexdigest(),test_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),results=outputs)
    (ROOT/'diagnostics/native_startup_trim_1527_20261115.json').write_text(json.dumps(report,indent=2))
if __name__=='__main__':main()
