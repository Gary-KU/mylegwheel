"""Native production task: raw -K error, velocity reference, VMC and stop guards.

Prescribed sensor feedback checks controller commands, not physical stability.
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
extern volatile struct {unsigned armed,phase,elapsed_ms,stop_reason;int direction;float requested_tp_nm,threshold_nm[2],baseline_rad[2],delta_rad[2];} g_fixed_support_probe;
static void pose_at(float angle) {
 float q[2];assert(leg_inverse(.03f+.09f*cosf(angle),.09f*sinf(angle),1.57f,1.57f,q));
 for(unsigned side=0;side<2;++side){motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=q[1];motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=q[0];}
}
int main(void){
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 imu=(ImuSample){0};imu_ok=1;g_robot_control.remote_enable=0;for(unsigned k=0;k<4;++k)g_dual_hold.torque_ratio_nm_per_v[k]=.08f;pose_at(1.57f);step();
 g_fixed_support_probe.armed=1;g_fixed_support_probe.direction=-1;g_robot_control.enable=1;
 for(unsigned i=0;i<340;++i){step();assert(g_balance.enable==0);assert(!g_wheel_test.enable);assert(g_dual_hold.sync_torque_nm[0]==0 && g_dual_hold.sync_torque_nm[1]==0);}
 assert(fabsf(g_dual_hold.force_n[0]-9)<.001f && fabsf(g_dual_hold.force_n[1]-9)<.001f);
 assert(fabsf(g_dual_hold.pitch_torque_nm[0]+.02f)<1e-6f);
 pose_at(1.63f);step();assert(!g_robot_control.enable && g_fixed_support_probe.stop_reason==3 && !g_fixed_support_probe.armed);
 step();pose_at(1.57f);g_fixed_support_probe.armed=1;g_fixed_support_probe.direction=1;g_robot_control.enable=1;
 for(unsigned i=0;i<862;++i)step();assert(!g_robot_control.enable && g_fixed_support_probe.stop_reason==4);
 step();g_fixed_support_probe.armed=1;g_robot_control.enable=1;step();g_robot_control.feedback_only=1;step();assert(!g_robot_control.enable && !g_dual_hold.enable);
 puts("PASS: fixed9N ramp, common Tp steps, zero sync/wheel owner,3deg stop,4.3s timeout and feedback stop");
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
          '-DROBOT_NATIVE_LQR=1','-DROBOT_FIXED_SUPPORT_PROBE=1','-DROBOT_LEGS_ONLY_CALIBRATION=0','-I',str(work)]
 for folder in ('App/Tasks/Inc','App/Services/Inc','App/Protocol/Inc','Control/WheelLeg/Inc','tests/support/wheel_leg/Inc'):
  command+=['-I',str(root/folder)]
 for tx in (1,):
  exe=work/f'test{tx}.exe'
  subprocess.run(command+[f'-DMOTOR_COMMAND_TX_ENABLED={tx}',str(work/'test.c'),
       *[str(root/p) for p in sources],'-lm','-o',str(exe)],check=True)
  subprocess.run([str(exe)],check=True)
