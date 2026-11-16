"""Production task regression for differential torque retained after a swing reversal.

Prescribed kinematics isolate command latency; this is not a standing simulation.
"""
import ast
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
tree = ast.parse((root / 'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
           if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
           and isinstance(n.value, ast.Constant) and isinstance(n.value.value, str)}
prefix = strings['harness'].split('int main(void) {')[0]
fixture = r'''
int main(void) {
 assert(ROBOT_NATIVE_LQR && g_balance_bias.enable==BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE);
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 g_robot_control.remote_enable=0;
 for(unsigned n=0;n<3;++n) {
  float length=n==0?.07f:n==1?.09f:.12f;
  for(int sign=-1;sign<=1;sign+=2) {
   g_robot_control.enable=0;step();
   swing_feedback(0,length,1.5707963268f+sign*.10f,0);
   swing_feedback(1,length,1.5707963268f-sign*.10f,0);
   g_robot_control.enable=1;
   for(unsigned j=0;j<100;++j)step();
   assert(g_robot_control.enable && g_balance.status==1);
   assert(sign*g_dual_hold.sync_torque_nm[0]<-.05f);
   /* After crossing through neutral the same old torque would now push
    * both legs farther apart. The fresh position AND velocity terms agree
    * on reversing torque. Allow two 5ms frames for the 4ms input filter. */
   swing_feedback(0,length,1.5707963268f-sign*.10f,-sign*1.0f);
   swing_feedback(1,length,1.5707963268f+sign*.10f,sign*1.0f);
   step();step();
   float delta=g_wheel_leg.leg[0].angle-g_wheel_leg.leg[1].angle;
   float velocity=g_wheel_leg.angle_speed_rad_s[0]-g_wheel_leg.angle_speed_rad_s[1];
   float correction=g_dual_hold.sync_torque_nm[0]-g_dual_hold.sync_torque_nm[1];
   printf("L=%.2f sign=%d split=%.4f rate=%.4f sync=(%.4f,%.4f)\n",
          length,sign,delta,velocity,g_dual_hold.sync_torque_nm[0],g_dual_hold.sync_torque_nm[1]);
   assert(delta*velocity>0);
   assert(correction*delta<0 && correction*velocity<0);
   for(unsigned side=0;side<2;++side) {
    assert(fabsf(g_dual_hold.sync_torque_nm[side])<=
           LEG_DEFAULT_SYNC_RATIO*LEG_DEFAULT_SYNC_LIMIT_NM+1e-6f);
    float neutral_error=1.5707963268f-g_wheel_leg.leg[side].angle;
    assert(g_dual_hold.angle_hold_torque_nm[side]*neutral_error>=0);
    assert(fabsf(g_dual_hold.angle_hold_torque_nm[side])<=LEG_DEFAULT_ANGLE_LIMIT_NM+1e-6f);
    assert(g_dual_hold.lqr_pitch_torque_nm[side]==g_balance.torque_nm[1]);
   }
   g_robot_control.enable=0;step();
   assert(!g_dual_hold.sync_torque_nm[0] && !g_dual_hold.sync_torque_nm[1]);
  }
 }
 puts("PASS: differential recovery reverses within10ms at7/9/12cm, both signs; sync and local-hold caps, native LQR and OFF retained");
}
'''
with tempfile.TemporaryDirectory(prefix='native_sync_') as directory:
    work = Path(directory)
    (work / 'runtime.h').write_text(strings['runtime'])
    for name in ('main.h', 'FreeRTOS.h', 'task.h', 'bsp_fdcan.h'):
        (work / name).write_text('#include "runtime.h"\n')
    (work / 'test.c').write_text(prefix + fixture)
    sources = ['App/Protocol/Src/robot_motion_command.c', 'App/Tasks/Src/wheel_leg_task.c',
               'App/Tasks/Src/wheel_leg_observer.c', 'App/Services/Src/motor_command.c',
               'tests/support/wheel_leg/Src/wheel_leg_control.c', 'Control/WheelLeg/Src/lqr_k.c',
               'Control/WheelLeg/Src/leg_pos.c', 'tests/support/wheel_leg/Src/leg_trajectory.c']
    command = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
               '-DMOTOR_COMMAND_TX_ENABLED=1', '-DROBOT_NATIVE_LQR=1',
               '-DROBOT_LEGS_ONLY_CALIBRATION=0', '-I', str(work)]
    for folder in ('App/Tasks/Inc', 'App/Services/Inc', 'App/Protocol/Inc',
                   'Control/WheelLeg/Inc', 'tests/support/wheel_leg/Inc'):
        command += ['-I', str(root / folder)]
    exe = work / 'sync.exe'
    subprocess.run(command + [str(work / 'test.c'), *[str(root / p) for p in sources],
                             '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
