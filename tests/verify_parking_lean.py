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
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-v/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=v/WHEEL_RADIUS_M;
}
static int command(unsigned slot) {
 return (int16_t)((uint16_t)data[slot/4][2*(slot%4)] |
                 (uint16_t)data[slot/4][2*(slot%4)+1]<<8);
}
static void checked(void) {
 if(g_robot_control.status!=1 || g_balance.status!=1) fprintf(stderr,"at now=%u status=%u balance=%u fault=%u phase=%u v=%g lean=%g\n",now,g_robot_control.status,g_balance.status,g_robot_control.last_fault.reason,g_robot_control.startup_phase,g_balance.state[3],g_balance.parking_lean_rad);
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
 assert(argc==2);LqrStartupRecord_Init();
 assert(g_balance.parking_lean_enable==1 && fabsf(g_balance.parking_lean_gain_rad_per_m-.05f)<1e-8f);
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 pose(0,1.57079632679f);pose(1,1.57079632679f);
 g_balance.back_emf_ratio=0;g_robot_control.remote_enable=0;g_dual_hold.feedforward_target_n=0;
 velocity(.06f);g_robot_control.enable=1;step();
#if !MOTOR_COMMAND_TX_ENABLED
 assert(!g_robot_control.enable && calls==0);return 0;
#endif
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP && g_balance.parking_lean_rad==0);
 float previous=0;
 for(unsigned i=0;i<600;++i) {
  step();checked();
  assert(fabsf(g_balance.parking_lean_rad-previous)<=BALANCE_PARKING_LEAN_RATE_RAD_S*.00501f);
  previous=g_balance.parking_lean_rad;
  assert(fabsf(g_balance.reference[0]-previous)<1e-8f);
  assert(fabsf(g_dual_hold.angle_reference_rad[0]-(1.57079632679f+previous))<1e-5f);
 }
 assert(g_balance.parking_lean_learning && previous<-.002f);
 velocity(.20f); /* New rolling entry gate accepts upright, uncapped motion above .15m/s. */
 float before_roll=previous;
 for(unsigned i=0;i<200;++i){step();checked();assert(g_balance.parking_lean_learning);assert(fabsf(g_balance.parking_lean_rad-previous)<=BALANCE_PARKING_LEAN_RATE_RAD_S*.00501f);previous=g_balance.parking_lean_rad;}
 assert(previous<before_roll-.002f);
 g_joint_output.saturated=1;step();assert(!g_balance.parking_lean_learning && g_balance.parking_lean_rad==previous);
 velocity(.30f);for(unsigned i=0;i<20;++i)step();assert(!g_balance.parking_lean_learning && g_balance.parking_lean_rad==previous);
 imu.attitude_deg[BODY_PITCH_AXIS]=10;step();assert(!g_balance.parking_lean_learning && g_balance.parking_lean_rad==previous);imu.attitude_deg[BODY_PITCH_AXIS]=0;
 pose(0,1.57079632679f-.10f);pose(1,1.57079632679f+.10f);step();assert(!g_balance.parking_lean_learning && g_balance.parking_lean_rad==previous);pose(0,1.57079632679f);pose(1,1.57079632679f);
 velocity(0);for(unsigned i=0;i<800;++i)step();
 float held=g_balance.parking_lean_rad;for(unsigned i=0;i<100;++i)step();assert(g_balance.parking_lean_rad==held);
 velocity(-.06f);for(unsigned i=0;i<400;++i){step();checked();}
 assert(g_balance.parking_lean_rad>held);
 g_robot_control.enable=0;step();stopped();assert(!g_balance.parking_lean_learning && g_balance.parking_lean_rad==0);
 for(unsigned i=0;i<400;++i){step();stopped();}
 assert(g_lqr_record.state==LQR_RECORD_FROZEN);LqrStartupRecord_Poll();now+=120;LqrStartupRecord_Poll();assert(saves==1);
 FILE *f=fopen(argv[1],"wb");assert(f);assert(fwrite((const void*)&g_lqr_record_data,384+g_lqr_record.count*sizeof(LqrRecordRow),1,f)==1);fclose(f);
 step();stopped();velocity(0);g_robot_control.enable=1;step();for(unsigned i=0;i<150;++i)step();velocity(.14f);for(unsigned i=0;i<3500;++i){step();checked();}
 assert(fabsf(g_balance.parking_lean_rad+BALANCE_PARKING_LEAN_LIMIT_RAD)<1e-7f);
 velocity(-.14f);for(unsigned i=0;i<800;++i)step();assert(g_balance.parking_lean_rad>-.034f); /* No hidden windup at clamp. */
 g_balance.position_window_m=.02f;step();assert(g_balance.parking_lean_rad==0 && !g_balance.parking_lean_learning);
 g_balance.position_window_m=0;g_balance.wheel_scale=0;step();assert(g_balance.parking_lean_rad==0);
 g_balance.wheel_scale=.5f;g_balance.parking_lean_enable=0;step();assert(g_balance.parking_lean_rad==0);
 g_balance.parking_lean_enable=1;velocity(.06f);for(unsigned i=0;i<500;++i)step();assert(g_balance.parking_lean_rad<0);
 motors[g_joint_calibration[0].motor_id-1].age_ms=11;step();stopped();assert(g_balance.parking_lean_rad==0);
 motors[g_joint_calibration[0].motor_id-1].age_ms=0;
 const float bad[]={-.01f,.0501f,NAN};
 for(unsigned i=0;i<3;++i){g_balance.parking_lean_gain_rad_per_m=bad[i];g_robot_control.enable=1;step();stopped();}
 g_balance.parking_lean_gain_rad_per_m=.05f;g_robot_control.enable=0;step();
 g_lqr_record.rearm_request=1;LqrStartupRecord_Poll();now+=120;LqrStartupRecord_Poll();step();stopped();
 assert(g_lqr_record.state==LQR_RECORD_READY);
 velocity(0);g_robot_control.enable=1;step();
 for(unsigned i=0;i<6400;++i){step();checked();}
 assert(g_lqr_record.state==LQR_RECORD_RUNNING && g_lqr_record.count<LQR_RECORD_ROWS);
 assert(g_lqr_record_data.header.period_ms==50);
 g_robot_control.enable=0;step();stopped();
 uint32_t trigger=g_lqr_record.trigger_index,trigger_tick=g_lqr_record_data.rows[trigger].tick_ms;
 for(unsigned i=0;i<400;++i){step();stopped();}
 assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.reason==LQR_RECORD_STOP && g_lqr_record.count<LQR_RECORD_ROWS);
 assert(g_lqr_record_data.rows[g_lqr_record.count-1].tick_ms-trigger_tick==2000);
 assert(g_lqr_record_data.rows[trigger+1].tick_ms-trigger_tick==20);
 puts("PASS: real task records32s active at20Hz, immediate stop and2s50Hz tail without filling800 rows");
 puts("PASS: enabled adaptation, startup/settle, forward/reverse/deadband, coordinated LQR/PD references, slew/clamp/antiwindup, saturation/motion freeze, mode/OFF/fault reset, invalid parameters");
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
        seen=[r for r in d['records'] if r['parking_lean_learning']]
        assert seen and any(r['parking_lean_deg']<-.1 for r in seen)
        assert all(abs(r['parking_lean_deg'])<=2.00001 for r in d['records'])
        assert all(r['parking_lean_deg']==0 for r in d['records'] if not r['active'])
        assert d['summary']['post_trigger_duration_ms']==2000
        assert 'parking_lean_deg' in (tmp/'record.csv').read_text(encoding='utf-8-sig')
    print('PASS: current-build CRC/flag/Python derived damping/CSV and post-stop tail')
