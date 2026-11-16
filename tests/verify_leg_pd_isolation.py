"""Exercise the real task and CAN path in both bounded bench build profiles."""
import ast
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
fixture = ast.parse((root / 'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings = {}
for node in fixture.body:
    if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ('runtime', 'harness'):
                strings[target.id] = ast.literal_eval(node.value)
prefix = strings['harness'].split('int main(void) {', 1)[0]
checks = r'''
static void pose(unsigned side,float angle,float speed) {
 float q[2];LegModel m;
 assert(leg_inverse(.03f+.07f*cosf(angle),.07f*sinf(angle),1.57f,1.57f,q));
 assert(leg_model(q[0],q[1],LEG_BRANCH_DOWN,&m));
 float det=m.jacobian[0]*m.jacobian[3]-m.jacobian[1]*m.jacobian[2];
 MotorFeedback *front=&motors[g_joint_calibration[2*side].motor_id-1];
 MotorFeedback *rear=&motors[g_joint_calibration[2*side+1].motor_id-1];
 rear->joint_angle_rad=q[0];front->joint_angle_rad=q[1];
 rear->joint_speed_rad_s=-m.jacobian[1]*speed/det;
 front->joint_speed_rad_s=m.jacobian[0]*speed/det;
}
static int command(unsigned slot) {
 return (int16_t)((uint16_t)data[slot/4][2*(slot%4)] |
                 (uint16_t)data[slot/4][2*(slot%4)+1]<<8);
}
static void stopped(void) {
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
 for(unsigned i=0;i<8;++i) assert(command(i)==0);
}
static void isolated(void) {
 unsigned side=ROBOT_LEG_PD_TEST_SIDE,other=1-side;
 assert(g_robot_control.status==1 && g_dual_hold.status==1);
 assert(!g_balance.enable && !g_wheel_test.enable && !g_wheel_test.zero_request);
 assert(g_dual_hold.force_n[0]==0 && g_dual_hold.force_n[1]==0);
 assert(g_dual_hold.feedforward_n==0);
 assert(g_dual_hold.sync_torque_nm[0]==0 && g_dual_hold.sync_torque_nm[1]==0);
 assert(g_dual_hold.lqr_pitch_torque_nm[0]==0 && g_dual_hold.lqr_pitch_torque_nm[1]==0);
 assert(g_dual_hold.pitch_torque_nm[other]==0);
 assert(fabsf(g_dual_hold.pitch_torque_nm[side])<=.050001f);
 assert(command(WHEEL_LEFT_CAN_ID-1)==0 && command(WHEEL_RIGHT_CAN_ID-1)==0);
 assert(command(g_joint_calibration[2*other].motor_id-1)==0);
 assert(command(g_joint_calibration[2*other+1].motor_id-1)==0);
 for(unsigned i=0;i<8;++i) assert(abs(command(i))<=600);
}
static void arm(void) {
 remote_sample.frame.channel[4]=1000;step();stopped();
 remote_sample.frame.channel[4]=306;step();
}
int main(void) {
 const unsigned side=ROBOT_LEG_PD_TEST_SIDE;
 const float neutral=1.57079632679f,rad=.017453292519943295f;
 assert(g_robot_control.feedback_only==0);
 for(unsigned i=0;i<8;++i) motors[i].online=motors[i].joint_calibrated=1;
 pose(0,neutral-10*rad,0);pose(1,neutral-10*rad,0);
 remote_ok=remote_fresh=1;remote_sample.frame.channel[4]=306;
 step();step();stopped(); /* Boot with upper held cannot arm. */
 arm();
#if !MOTOR_COMMAND_TX_ENABLED
 stopped();assert(calls==0 && g_robot_control.status==5);
 puts("PASS: isolated profile with TX disabled sends nothing");return 0;
#endif
 isolated();assert(fabsf(g_dual_hold.pitch_torque_nm[side])<1e-6f);
 for(unsigned i=0;i<100;++i) {step();isolated();}
 assert(g_dual_hold.pitch_torque_nm[side]>0); /* Angle below 90 must increase. */
 /* Watch cannot introduce any other owner, component, or larger limit. */
 g_wheel_test.enable=g_wheel_test.zero_request=g_balance.enable=1;
 g_dual_hold.voltage_limit_mv=6900;g_dual_hold.angle_limit_nm=1;
 g_dual_hold.feedforward_target_n=15;g_dual_hold.sync_ratio=1;
 g_dual_hold.speed_kp=100;g_dual_hold.overlength_kp_n_per_m=1000;
 g_dual_hold.load_enable=1;g_dual_hold.back_emf_ratio=1;
 step();isolated();
 for(unsigned i=0;i<500;++i) step();
 stopped();
 for(unsigned i=0;i<50;++i) {step();stopped();} /* Timeout is latched. */
 pose(side,neutral+10*rad,0);arm();
 for(unsigned i=0;i<100;++i) {step();isolated();}
 assert(g_dual_hold.pitch_torque_nm[side]<0);
 remote_sample.frame.channel[4]=1694;step();stopped();
 pose(side,neutral,.5f);arm();isolated();
 assert(g_dual_hold.pitch_torque_nm[side]<0); /* Damping opposes +velocity. */
 pose(side,neutral,-.5f);step();isolated();
 assert(g_dual_hold.pitch_torque_nm[side]>0);
 pose(side,neutral,1.01f);step();stopped();
 pose(side,neutral,0);step();stopped();
 pose(side,neutral+16*rad,0);arm();stopped(); /* Start envelope. */
 pose(side,neutral,0);arm();isolated();
 pose(side,neutral+21*rad,0);step();stopped(); /* Running envelope. */
 pose(side,neutral,0);arm();isolated();
 motors[g_joint_calibration[2*side].motor_id-1].age_ms=11;step();stopped();
 motors[g_joint_calibration[2*side].motor_id-1].age_ms=0;step();stopped();
 arm();isolated();remote_ok=0;step();stopped();
 remote_ok=1;step();stopped();arm();isolated();
 fail_at=calls+1;step();stopped();fail_at=0;step();stopped();
 arm();isolated();now+=11;WheelLeg_Update();stopped();
 step();stopped();arm();isolated();
 g_robot_control.feedback_only=1;step();stopped();
 g_robot_control.feedback_only=0;step();stopped();arm();isolated();
 /* Even explicit Watch enables cannot skip remote OFF->ON. */
 remote_sample.frame.channel[4]=1000;step();stopped();
 g_robot_control.remote_enable=0;g_robot_control.enable=g_dual_hold.enable=1;
 step();stopped();assert(g_robot_control.remote_enable==1);
 remote_sample.frame.channel[4]=306;step();isolated();
 /* Clock rollover must not defeat the 3-second timeout. */
 remote_sample.frame.channel[4]=1000;now=UINT32_MAX-100;WheelLeg_Update();step();
 remote_sample.frame.channel[4]=306;step();isolated();
 for(unsigned i=0;i<600;++i) step();
 stopped();
 puts("PASS: single-leg PD polarity, damping, owner isolation, 600mV/50mNm caps, 3s timeout, pose/speed/feedback/SBUS/TX/timing stops and remote rearm");
}
'''
sources = ['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c', 'App/Services/Src/motor_command.c',
           'tests/support/wheel_leg/Src/wheel_leg_control.c', 'Control/WheelLeg/Src/lqr_k.c',
           'Control/WheelLeg/Src/leg_pos.c', 'tests/support/wheel_leg/Src/leg_trajectory.c']
with tempfile.TemporaryDirectory(prefix='leg_pd_') as directory:
    tmp = Path(directory)
    (tmp/'runtime.h').write_text(strings['runtime'])
    for name in ('main.h', 'FreeRTOS.h', 'task.h', 'bsp_fdcan.h'):
        (tmp/name).write_text('#include "runtime.h"\n')
    (tmp/'test.c').write_text(prefix+checks)
    base = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function']
    for path in (tmp, root/'App/Tasks/Inc', root/'App/Services/Inc', root/'Control/WheelLeg/Inc',
                 root/'tests/support/wheel_leg/Inc', root/'App/Protocol/Inc'):
        base += ['-I', str(path)]
    for side in (0, 1):
        for tx in (0, 1):
            executable = tmp/f'test_{side}_{tx}.exe'
            subprocess.run(base+[f'-DROBOT_LEG_PD_TEST_SIDE={side}', f'-DMOTOR_COMMAND_TX_ENABLED={tx}',
                           str(tmp/'test.c'), *[str(root/p) for p in sources], '-lm', '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
