"""Check capture admission, fixed timeout, OFF rearm and trial preservation.

Prescribed sensors test production stop logic; not physical standing stability.
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
extern volatile uint32_t g_robot_bench_limit_ms,g_robot_bench_elapsed_ms;
int main(void) {
 assert(ROBOT_CAPTURE_BUILD && !LEG_ANGLE_INTEGRAL_ENABLED);
 assert(LQR_RECORD_PERIOD_MS==5);
 assert(ROBOT_CAPTURE_LIMIT_MS/LQR_RECORD_PERIOD_MS+
        LQR_RECORD_TAIL_MS/LQR_RECORD_TAIL_PERIOD_MS+4<LQR_RECORD_ROWS);
 assert(!g_robot_control.feedback_only);
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 imu=(ImuSample){0};imu_ok=1;
 for(unsigned side=0;side<2;++side)swing_feedback(side,.10f,1.5707963268f,0);
 g_dual_hold.target_m=.10f;
 g_dual_hold.feedforward_target_n=2;g_dual_hold.feedforward_rate_n_s=1000;
 remote_ok=remote_fresh=1;motion_sticks.valid=motion_sticks.calibrated=1;
 remote_sample.frame.channel[4]=306;
 for(unsigned n=0;n<20;++n)step();
 assert(!g_robot_control.enable); /* No ON-at-boot start. */
 remote_sample.frame.channel[4]=1000;step();
 remote_sample.frame.channel[4]=306;
 for(unsigned n=0;n<200;++n){g_robot_bench_limit_ms=0;step();}
 assert(g_robot_control.enable && g_robot_bench_limit_ms==1500);
 for(unsigned n=0;n<110;++n)step();
 assert(!g_robot_control.enable && !g_balance.enable && !g_dual_hold.enable);
 assert(g_robot_bench_elapsed_ms>=1500 && g_robot_bench_elapsed_ms<=1505);
 assert(!g_robot_control.feedback_only);
 for(unsigned n=0;n<100;++n)step();
 assert(!g_robot_control.enable); /* Holding CH5 ON cannot repeat. */
 g_lqr_record.state=LQR_RECORD_SAVED;
 remote_sample.frame.channel[4]=1000;step();
 remote_sample.frame.channel[4]=306;step();
 assert(!g_robot_control.enable); /* Preserve unread saved trial. */
 g_lqr_record.state=LQR_RECORD_READY;
 remote_sample.frame.channel[4]=1000;step();
 remote_sample.frame.channel[4]=306;step();
 assert(g_robot_control.enable);
 remote_sample.frame.channel[4]=1000;step();
 assert(!g_robot_control.enable && !g_balance.enable && !g_dual_hold.enable);
 puts("PASS: capture1500ms hard timeout, Watch bypass rejected, CH5 OFF rearm, unread trial blocks new launch, manual stop");
}
'''
with tempfile.TemporaryDirectory(prefix='supported_capture_') as directory:
    work=Path(directory);(work/'runtime.h').write_text(strings['runtime'])
    for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):
        (work/name).write_text('#include "runtime.h"\n')
    (work/'test.c').write_text(prefix+fixture)
    sources=['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c',
             'App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_command.c',
             'tests/support/wheel_leg/Src/wheel_leg_control.c','Control/WheelLeg/Src/lqr_k.c',
             'Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
    cmd=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',
         '-DMOTOR_COMMAND_TX_ENABLED=1','-DROBOT_NATIVE_LQR=1','-DROBOT_CAPTURE_BUILD=1',
         '-DROBOT_LEGS_ONLY_CALIBRATION=0','-I',str(work)]
    for folder in ('App/Tasks/Inc','App/Services/Inc','App/Protocol/Inc',
                   'Control/WheelLeg/Inc','tests/support/wheel_leg/Inc'):
        cmd+=['-I',str(root/folder)]
    exe=work/'capture.exe'
    subprocess.run(cmd+[str(work/'test.c'),*[str(root/p) for p in sources],'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
