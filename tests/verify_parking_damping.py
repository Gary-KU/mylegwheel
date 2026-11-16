"""Exercise enabled-default parking damping through real task, CAN and recorder."""
import ast
import json
import re
from pathlib import Path
import subprocess
import sys
import tempfile
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
from read_lqr_startup_record import header_decode, export, record_crc
tree=ast.parse((root/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings={}
for node in tree.body:
    if isinstance(node,ast.Assign) and isinstance(node.value,ast.Constant):
        for target in node.targets:
            if isinstance(target,ast.Name) and target.id in ('runtime','harness'):
                strings[target.id]=ast.literal_eval(node.value)
prefix=strings['harness'].split('int main(void) {',1)[0]
begin=prefix.index('volatile LqrRecordDebug g_lqr_record;')
end=prefix.index('void RobotTelemetry_Capture',begin)
prefix=prefix[:begin]+prefix[end:]
fixture=r'''
static unsigned saves;
int LqrRecordStorage_Load(LqrRecordHeader *h) {(void)h;return 0;}
int LqrRecordStorage_Save(const volatile LqrRecordData *r,uint32_t *error) {
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
 assert(r->header.commit==LQR_RECORD_COMMIT);++saves;*error=0;return 1;
}
static void pose(unsigned s,float angle) {
 float q[2];assert(leg_inverse(.03f+.095f*cosf(angle),.095f*sinf(angle),1.57f,1.57f,q));
 motors[g_joint_calibration[2*s].motor_id-1].joint_angle_rad=q[1];
 motors[g_joint_calibration[2*s+1].motor_id-1].joint_angle_rad=q[0];
}
static void velocity(float v) {
 /* 左右编码器方向相反，同向前进需要左负右正。 */
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-v/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=v/WHEEL_RADIUS_M;
}
static int command(unsigned slot) {
 return (int16_t)((uint16_t)data[slot/4][2*(slot%4)] |
                 (uint16_t)data[slot/4][2*(slot%4)+1]<<8);
}
static void checked(void) {
 assert(g_robot_control.status==1 && g_balance.status==1);
 float sum=g_balance.startup_wheel_damping_torque_nm+g_balance.parking_damping_torque_nm;
 for(unsigned i=0;i<6;++i)sum+=g_balance.wheel_torque_component_nm[i];
 assert(fabsf(sum-g_balance.torque_nm[0])<1e-6f);
 assert(fabsf(g_balance.parking_damping_torque_nm)<=.00150001f);
 if(g_balance.parking_damping_active) {
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_balance.position_locked);
  assert(g_balance.parking_damping_torque_nm*g_balance.state[3]<=0);
 }
 for(unsigned s=0;s<2;++s) {
  unsigned id=s?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID;
  float mv=(g_balance.torque_nm[0]*.7f/g_wheel_output.torque_ratio_nm_per_v[s])*1000;
  mv=fmaxf(-6000,fminf(6000,mv))*(s?1:-1);
  assert(abs(command(id-1)-(int)mv)<=1);
 }
}
static void stopped(void) {
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
 assert(g_balance.parking_damping_torque_nm==0 && !g_balance.parking_damping_active);
 for(unsigned i=0;i<8;++i)assert(command(i)==0);
}
int main(int argc,char **argv) {
 assert(argc==2);g_balance.parking_lean_enable=0;LqrStartupRecord_Init();
 assert(fabsf(g_balance.parking_damping_gain_nm_per_m_s-.01f)<1e-8f);
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 pose(0,80*.01745329252f);pose(1,87*.01745329252f);
 g_balance.back_emf_ratio=0;g_robot_control.remote_enable=0;
 g_dual_hold.feedforward_target_n=0; /* Constant support fixture for paired comparisons. */
 velocity(.06f);g_robot_control.enable=1;step();
#if !MOTOR_COMMAND_TX_ENABLED
 assert(!g_robot_control.enable && calls==0);return 0;
#endif
 if(g_robot_control.startup_phase!=ROBOT_STARTUP_RAMP)
  fprintf(stderr,"startup=%u status=%u fault=%u dual=%u balance=%u legvalid=%u/%u lengths=%g/%g\n",
   g_robot_control.startup_phase,g_robot_control.status,g_robot_control.last_fault.reason,
   g_dual_hold.status,g_balance.status,g_wheel_leg.leg_valid[0],g_wheel_leg.leg_valid[1],
   g_wheel_leg.leg[0].length,g_wheel_leg.leg[1].length);
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
 assert(!g_balance.parking_damping_active && g_balance.parking_damping_torque_nm==0);
 for(unsigned i=0;i<210;++i){step();checked();}
 assert(g_balance.parking_damping_active && fabsf(g_balance.parking_damping_torque_nm+.0006f)<1e-8f);
 velocity(.3f);for(unsigned i=0;i<100;++i){step();checked();}
 assert(fabsf(g_balance.parking_damping_torque_nm+.0015f)<1e-8f);
 velocity(.06f);step();checked();
 const BalanceDebug on=g_balance;const DualHoldDebug leg=g_dual_hold;
 g_balance.parking_damping_gain_nm_per_m_s=0;step();checked();
 assert(!g_balance.parking_damping_active && g_balance.parking_damping_torque_nm==0);
 assert(fabsf(g_balance.torque_nm[0]-on.torque_nm[0]-.0006f)<1e-6f);
 assert(fabsf(g_balance.torque_nm[1]-on.torque_nm[1])<1e-6f);
 for(unsigned i=0;i<4;++i)assert(fabsf(g_dual_hold.joint_torque_nm[i]-leg.joint_torque_nm[i])<1e-5f);
 g_balance.parking_damping_gain_nm_per_m_s=.01f;
 velocity(-.06f);step();checked();
 assert(fabsf(g_balance.parking_damping_torque_nm-.0006f)<1e-8f);
 velocity(-.3f);step();checked();assert(fabsf(g_balance.parking_damping_torque_nm-.0015f)<1e-8f);
 velocity(0);step();checked();assert(g_balance.parking_damping_torque_nm==0);
 g_balance.position_window_m=.02f;step();checked();assert(!g_balance.parking_damping_active);
 g_balance.position_window_m=0;g_balance.wheel_scale=0;step();checked();assert(!g_balance.parking_damping_active);
 g_balance.wheel_scale=.5f;velocity(.06f);for(unsigned i=0;i<4;++i)step();checked();
 g_robot_control.enable=0;step();stopped();
 for(unsigned i=0;i<400;++i){step();stopped();}
 assert(g_lqr_record.state==LQR_RECORD_FROZEN);
 LqrStartupRecord_Poll();now+=120;LqrStartupRecord_Poll();assert(saves==1);
 FILE *f=fopen(argv[1],"wb");assert(f);
 assert(fwrite((const void*)&g_lqr_record_data,384+g_lqr_record.count*sizeof(LqrRecordRow),1,f)==1);fclose(f);
 const float bad[]={-.001f,.0101f,NAN};
 for(unsigned i=0;i<3;++i) {
  g_balance.parking_damping_gain_nm_per_m_s=bad[i];g_robot_control.enable=1;step();stopped();
 }
 g_balance.parking_damping_gain_nm_per_m_s=.01f;g_robot_control.enable=1;step();
 for(unsigned i=0;i<120;++i)step();
 checked();
 motors[g_joint_calibration[0].motor_id-1].age_ms=11;step();stopped();
 puts("PASS: enabled default, forward/reverse/zero/cap CAN, no startup/moving-window/wheel-zero damping, unchanged leg outputs, OFF/feedback reset, invalid gain fails closed");
}
'''
sources=['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_command.c',
    'App/Services/Src/lqr_startup_record.c','tests/support/wheel_leg/Src/wheel_leg_control.c',
    'Control/WheelLeg/Src/lqr_k.c','Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
with tempfile.TemporaryDirectory(prefix='parking_damping_') as directory:
    tmp=Path(directory);(tmp/'runtime.h').write_text(strings['runtime'])
    for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):(tmp/name).write_text('#include "runtime.h"\n')
    (tmp/'test.c').write_text(prefix+fixture)
    base=['gcc','-DROBOT_NATIVE_LQR=0','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function']
    for folder in (tmp,root/'App/Tasks/Inc',root/'App/Services/Inc',root/'Control/WheelLeg/Inc',
        root/'tests/support/wheel_leg/Inc',root/'App/Protocol/Inc'):base+=['-I',str(folder)]
    for tx in (0,1):
        exe=tmp/f'test{tx}.exe';out=tmp/f'parking{tx}.bin'
        subprocess.run(base+[f'-DMOTOR_COMMAND_TX_ENABLED={tx}',str(tmp/'test.c'),
            *[str(root/p) for p in sources],'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe),str(out)],check=True)
        if not tx:continue
        raw=out.read_bytes();h=header_decode(raw[:128])
        default_build_id=int(re.findall(r'^#define LQR_RECORD_BUILD_ID (\d+)U',
            (root/'App/Services/Inc/lqr_startup_record.h').read_text(encoding='utf-8'),re.M)[-1])
        assert h['build_id']==default_build_id and h['period_ms']==50
        assert record_crc(raw[:128],raw[384:],raw[128:384])==h['crc32']
        export(h,raw[384:],tmp/'record.json','real task fixture',raw[128:384])
        d=json.loads((tmp/'record.json').read_text(encoding='utf-8'))
        seen=[r for r in d['records'] if r['parking_damping_active']]
        assert seen and all(abs(r['parking_damping_command_nm'])<=.0015001 for r in seen)
        assert any(abs(r['parking_damping_command_nm']+.0006)<1e-7 for r in seen)
        assert all(r['parking_damping_command_nm']==0 for r in d['records'] if not r['active'])
        assert d['summary']['post_trigger_duration_ms']==2000
        assert 'parking_damping_command_mNm' in (tmp/'record.csv').read_text(encoding='utf-8-sig')
    print('PASS: current-build CRC/flag/Python derived damping/CSV and post-stop tail')
