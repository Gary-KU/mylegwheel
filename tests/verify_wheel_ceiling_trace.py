"""Real task/CAN checks for per-wheel requested torque and frozen RAM trace."""
import ast
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
tree = ast.parse((root/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings = {}
for node in tree.body:
    if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
        for name in node.targets:
            if isinstance(name, ast.Name) and name.id in ('runtime', 'harness'):
                strings[name.id] = ast.literal_eval(node.value)
checks = r'''
static void stationary_pose(void) {
 float q[2];assert(leg_inverse(.03f,.07f,1.57f,1.57f,q));
 for(unsigned i=0;i<8;++i) motors[i].online=motors[i].joint_calibrated=1;
 for(unsigned i=0;i<4;++i) motors[g_joint_calibration[i].motor_id-1].joint_angle_rad=q[i%2==0];
}
static void verify_ceiling(float lean) {
 imu.attitude_deg[BODY_PITCH_AXIS]=lean;step();
 assert(g_robot_control.status==1 && g_balance.status==1);
 assert(g_wheel_output.torque_saturated && g_wheel_output.saturated);
 assert(fabsf(g_wheel_output.requested_torque_nm)>.03f);
 assert(fabsf(fabsf(g_wheel_output.applied_torque_nm)-.03f)<1e-7f);
 assert(abs(g_wheel_output.voltage_mv[0])==2187 && abs(g_wheel_output.voltage_mv[1])==2079);
 for(unsigned side=0;side<2;++side) {
  unsigned slot=(side?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1;
  int16_t sent=(int16_t)((uint16_t)data[slot/4][2*(slot%4)] |
                       (uint16_t)data[slot/4][2*(slot%4)+1]<<8);
  assert(sent==g_wheel_output.voltage_mv[side]);
 }
}
int main(void) {
 assert(sizeof(WheelBalanceTraceRow)==64);
 assert(sizeof(WheelBalanceTraceDebug)==12+256*64);
 stationary_pose();g_robot_control.remote_enable=0;g_robot_control.feedback_only=0;
 g_robot_control.startup_enable=0;g_robot_control.enable=1;
 g_balance.parking_correction_limit_m=.03f; /* Explicit optional-governor fixture. */
 g_balance.back_emf_ratio=0;g_dual_hold.feedforward_target_n=0;
 /* 固定电压夹具系数，保留独立黄金值，不依赖实机标定默认值的变化。 */
 g_wheel_output.torque_ratio_nm_per_v[0]=.0096f;g_wheel_output.torque_ratio_nm_per_v[1]=.0101f;
 g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
 g_dual_hold.overlength_kp_n_per_m=0;
 step();
#if !MOTOR_COMMAND_TX_ENABLED
 assert(!g_robot_control.enable && calls==0 && g_balance_trace.count==0);
 puts("PASS: TX disabled records no active actuator trace");return 0;
#endif
 verify_ceiling(-10);float previous=g_wheel_output.applied_torque_nm;
 verify_ceiling(10);assert(previous*g_wheel_output.applied_torque_nm<0);
 for(unsigned i=0;i<1400;++i) step();
 assert(g_balance_trace.count==256 && g_balance_trace.next<256);
 unsigned start=g_balance_trace.next;
 uint32_t last_tick=0;
 for(unsigned i=0;i<256;++i) {
  WheelBalanceTraceRow row=g_balance_trace.rows[(start+i)%256];
  assert(row.flags&1U);assert(row.flags&4U);
  assert(fabsf(row.wheel_requested_nm)>.03f && fabsf(row.wheel_applied_nm)<=.030001f);
  assert(abs(row.wheel_voltage_mv[0])<=2187 && abs(row.wheel_voltage_mv[1])<=2079);
  assert(fabsf(row.leg_tp_nm)<=BALANCE_LEG_LQR_TORQUE_LIMIT_NM+1e-6f);
  if(i) assert((uint32_t)(row.tick_ms-last_tick)>=20U);
  last_tick=row.tick_ms;
 }
 uint32_t sequence=g_balance_trace.sequence,next=g_balance_trace.next;
 uint32_t count=g_balance_trace.count,updates=g_wheel_leg.updates;
 WheelBalanceTraceDebug frozen=g_balance_trace;
 g_robot_control.enable=0;step();
 /* 当前 64 字节 RAM 轨迹停机立即冻结；观察任务继续更新，不追加停机行。 */
 assert(g_balance_trace.sequence==sequence && g_balance_trace.next==next && g_balance_trace.count==count);
 WheelBalanceTraceDebug stopped=g_balance_trace;
 assert(memcmp(&frozen,&stopped,sizeof(frozen))==0);
 assert(g_wheel_leg.updates==updates+1 && g_wheel_leg.leg_valid[0] &&
        g_wheel_leg.leg_valid[1] && g_wheel_leg.imu_valid);
 assert(g_wheel_output.voltage_mv[0]==0 && g_wheel_output.voltage_mv[1]==0);
 sequence=g_balance_trace.sequence;next=g_balance_trace.next;
 for(unsigned i=0;i<200;++i)step();
 assert(g_balance_trace.sequence==sequence && g_balance_trace.next==next && g_balance_trace.count==count);
 stopped=g_balance_trace;assert(memcmp(&frozen,&stopped,sizeof(frozen))==0);
 assert(g_wheel_output.requested_torque_nm==0 && g_wheel_output.applied_torque_nm==0 && !g_wheel_output.torque_saturated);
 imu.attitude_deg[BODY_PITCH_AXIS]=-1;g_robot_control.enable=1;step();
 assert(g_balance_trace.count==1);
 assert(!g_wheel_output.torque_saturated);
 assert(g_wheel_output.requested_torque_nm==g_wheel_output.applied_torque_nm);
 assert(fabsf(g_wheel_output.applied_torque_nm)<.03f);
 /* A distant goal stays fixed; only the LQR correction is bounded. */
 assert(g_balance.parking_correction_limit_m==.03f);
 const float goal=g_balance.parking_target_m;
 float K[12];lqr_k(.07f,K);
 for(unsigned direction=0;direction<2;++direction) {
  motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=direction?8:-8;
  motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=direction?-8:8;
  for(unsigned i=0;i<(direction?1200:600);++i) {
   step();assert(g_robot_control.status==1 && g_balance.position_locked);
   assert(g_balance.parking_target_m==goal && g_balance.reference[3]==0);
   float actual_error=g_balance.state[2]-goal;
   float limited=fmaxf(-.03f,fminf(.03f,actual_error));
   assert(fabsf(g_balance.parking_error_m-actual_error)<1e-6f);
   assert(fabsf(g_balance.state[2]-g_balance.reference[2]-limited)<1e-6f);
   assert(fabsf(g_balance.wheel_torque_component_nm[2]+g_balance.wheel_scale*K[4]*limited)<1e-6f);
  }
  assert((direction?-1:1)*g_balance.parking_error_m>.5f);
 }
 g_balance.parking_correction_limit_m=0;step();
 assert(g_balance.parking_target_m==goal && g_balance.reference[2]==goal);
 g_balance.parking_correction_limit_m=.03f;
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 g_robot_control.enable=0;step();sequence=g_balance_trace.sequence;
 motors[JOINT_LF_CAN_ID-1].online=0;g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_balance_trace.sequence==sequence);
 motors[JOINT_LF_CAN_ID-1].online=1;
 const float bad_limits[]={-.01f,.101f,NAN,INFINITY};
 for(unsigned i=0;i<4;++i) {
  g_balance.parking_correction_limit_m=bad_limits[i];g_robot_control.enable=1;step();
  assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 }
 puts("PASS: +/-0.030Nm wheel request ceiling/CAN, unchanged small commands, 50Hz trace freeze/wrap/rearm, fixed true parking goal with bounded bidirectional correction, zero velocity target, legacy disable and invalid limits");
}
'''
sources = ['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c', 'App/Services/Src/motor_command.c',
           'tests/support/wheel_leg/Src/wheel_leg_control.c', 'Control/WheelLeg/Src/lqr_k.c',
           'Control/WheelLeg/Src/leg_pos.c', 'tests/support/wheel_leg/Src/leg_trajectory.c']
with tempfile.TemporaryDirectory(prefix='wheel_ceiling_') as directory:
    tmp = Path(directory)
    (tmp/'runtime.h').write_text(strings['runtime'])
    for name in ('main.h', 'FreeRTOS.h', 'task.h', 'bsp_fdcan.h'):
        (tmp/name).write_text('#include "runtime.h"\n')
    (tmp/'test.c').write_text(strings['harness'].split('int main(void) {', 1)[0]+checks)
    base = ['gcc','-DROBOT_NATIVE_LQR=0', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function']
    for path in (tmp, root/'App/Tasks/Inc', root/'App/Services/Inc', root/'Control/WheelLeg/Inc',
                 root/'tests/support/wheel_leg/Inc', root/'App/Protocol/Inc'):
        base += ['-I', str(path)]
    for tx in (0, 1):
        output = tmp/f'test_{tx}.exe'
        subprocess.run(base+[f'-DMOTOR_COMMAND_TX_ENABLED={tx}', '-DWHEEL_REQUEST_TORQUE_LIMIT_ENABLED=1', str(tmp/'test.c'),
                            *[str(root/p) for p in sources], '-lm', '-o', str(output)], check=True)
        subprocess.run([str(output)], check=True)
