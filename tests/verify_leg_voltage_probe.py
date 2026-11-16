"""Check the real task/CAN/record path for the isolated 2V/max2s probe."""
import ast
import json
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
 assert(!g_robot_control.enable && !g_robot_control.remote_switch_on);
 assert(r->header.commit==LQR_RECORD_COMMIT);*error=0;++saves;return 1;
}
static void pose(unsigned side,float angle,float speed) {
 float q[2];LegModel m;
 assert(leg_inverse(.03f+.07f*cosf(angle),.07f*sinf(angle),1.57f,1.57f,q));
 assert(leg_model(q[0],q[1],LEG_BRANCH_DOWN,&m));
 float det=m.jacobian[0]*m.jacobian[3]-m.jacobian[1]*m.jacobian[2];
 MotorFeedback *f=&motors[g_joint_calibration[2*side].motor_id-1];
 MotorFeedback *r=&motors[g_joint_calibration[2*side+1].motor_id-1];
 r->joint_angle_rad=q[0];f->joint_angle_rad=q[1];
 r->joint_speed_rad_s=-m.jacobian[1]*speed/det;
 f->joint_speed_rad_s=m.jacobian[0]*speed/det;
}
static int command(unsigned slot) {
 return (int16_t)((uint16_t)data[slot/4][2*(slot%4)] |
                 (uint16_t)data[slot/4][2*(slot%4)+1]<<8);
}
static void stopped(void) {
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
 for(unsigned i=0;i<8;++i) assert(command(i)==0);
}
static void arm(void) {
 remote_sample.frame.channel[4]=1000;step();stopped();
 remote_sample.frame.channel[4]=306;step();
}
static void isolated(float direction) {
 const unsigned s=ROBOT_LEG_PD_TEST_SIDE;
 assert(g_robot_control.enable && g_dual_hold.status==1 && !g_balance.enable);
 assert(g_dual_hold.pitch_torque_nm[s]*direction>0);
 assert(g_dual_hold.pitch_torque_nm[1-s]==0);
 for(unsigned k=0;k<2;++k) {
  assert(g_dual_hold.angle_hold_torque_nm[k]==0 && g_dual_hold.sync_torque_nm[k]==0);
  assert(g_dual_hold.lqr_pitch_torque_nm[k]==0 && g_dual_hold.force_n[k]==0);
 }
 assert(!g_wheel_test.enable && !g_wheel_test.zero_request);
 assert(command(WHEEL_LEFT_CAN_ID-1)==0 && command(WHEEL_RIGHT_CAN_ID-1)==0);
 assert(command(g_joint_calibration[2*(1-s)].motor_id-1)==0);
 assert(command(g_joint_calibration[2*(1-s)+1].motor_id-1)==0);
 int peak=0;
 for(unsigned j=0;j<2;++j) {
  unsigned i=2*s+j;int mv=command(g_joint_calibration[i].motor_id-1);
  assert(abs(mv)<=2000);if(abs(mv)>peak)peak=abs(mv);
  float predicted=g_wheel_leg.leg[s].jacobian[3-j]*g_dual_hold.pitch_torque_nm[s]*
                  .7f/g_dual_hold.torque_ratio_nm_per_v[i]*1000*g_dual_hold.direction[i];
  assert(fabsf(predicted-mv)<1.5f); /* Preserve the same J^T ratio/polarity. */
 }
 assert(peak>=1999); /* Actual CAN request, not merely a raised ceiling. */
}
int main(int argc,char **argv) {
 assert(argc==2);const unsigned s=ROBOT_LEG_PD_TEST_SIDE;
 const float n=1.57079632679f,rad=.017453292519943295f;
 LqrStartupRecord_Init();
 for(unsigned i=0;i<8;++i) motors[i].online=motors[i].joint_calibrated=1;
 pose(0,n,0);pose(1,n,0);remote_ok=remote_fresh=1;
 remote_sample.frame.channel[4]=306;step();stopped();
 arm();stopped();assert(g_lqr_record.count==0); /* Neutral has no probe direction. */
 pose(s,n-10*rad,0);arm();
#if !MOTOR_COMMAND_TX_ENABLED
 stopped();assert(calls==0 && g_lqr_record.count==0);return 0;
#endif
 isolated(1);
 for(unsigned i=0;i<399;++i){step();isolated(1);} /* At1995ms still bounded active. */
 step();stopped();
 assert(g_lqr_record_data.rows[g_lqr_record.trigger_index].tick_ms-
        g_lqr_record_data.header.start_tick_ms==2000);
 for(unsigned i=0;i<10;++i){step();stopped();} /* Upper held cannot repeat. */
 pose(s,n-8*rad,0);
 for(unsigned i=0;i<390;++i)step();
 assert(g_lqr_record.state==LQR_RECORD_FROZEN);
 LqrStartupRecord_Poll();assert(!saves); /* CH5 high blocks save. */
 remote_sample.frame.channel[4]=1000;step();LqrStartupRecord_Poll();
 for(unsigned i=0;i<25;++i)step();
 LqrStartupRecord_Poll();assert(saves==1 && g_lqr_record.state==LQR_RECORD_SAVED);
 FILE *f=fopen(argv[1],"wb");assert(f);
 assert(fwrite((const void*)&g_lqr_record_data,384+g_lqr_record.count*sizeof(LqrRecordRow),1,f)==1);fclose(f);
 pose(s,n+10*rad,0);arm();isolated(-1);
 pose(s,n-.5f*rad,0);step();isolated(-1); /* Crossing90does not flip direction or stop. */
 remote_sample.frame.channel[4]=1000;step();stopped();
 pose(s,n-10*rad,1.01f);arm();stopped(); /* Starting while moving retains1rad/s gate. */
 pose(s,n-10*rad,0);arm();isolated(1);pose(s,n-8*rad,1.31f);step();isolated(1);
 pose(s,n-8*rad,4.01f);step();stopped();
 pose(s,n-10*rad,0);arm();isolated(1);pose(s,n+21*rad,0);step();stopped();
 pose(s,n-10*rad,0);arm();isolated(1);
 motors[g_joint_calibration[2*s].motor_id-1].age_ms=11;step();stopped();
 motors[g_joint_calibration[2*s].motor_id-1].age_ms=0;arm();isolated(1);
 remote_sample.frame.channel[4]=1000;step();stopped();
 arm();isolated(1);fail_at=calls+1;step();stopped();fail_at=0;
 arm();isolated(1);now+=11;WheelLeg_Update();stopped();
 remote_sample.frame.channel[4]=1000;now=UINT32_MAX-100;WheelLeg_Update();step();
 remote_sample.frame.channel[4]=306;step();isolated(1);
 for(unsigned i=0;i<400;++i)step();
 stopped();
 puts("PASS: actual2V CAN, J^T ratio/polarity both directions, isolation, neutral rejection, 2s/wrap timeout, neutral crossing keeps direction, start/run speed/pose/feedback/CH5/TX/timing stops and2s record");
}
'''
sources=['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_command.c',
    'App/Services/Src/lqr_startup_record.c','tests/support/wheel_leg/Src/wheel_leg_control.c',
    'Control/WheelLeg/Src/lqr_k.c','Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
with tempfile.TemporaryDirectory(prefix='leg_voltage_probe_') as directory:
    tmp=Path(directory)
    (tmp/'runtime.h').write_text(strings['runtime'])
    for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):
        (tmp/name).write_text('#include "runtime.h"\n')
    (tmp/'test.c').write_text(prefix+fixture)
    base=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function']
    for folder in (tmp,root/'App/Tasks/Inc',root/'App/Services/Inc',root/'Control/WheelLeg/Inc',
        root/'tests/support/wheel_leg/Inc',root/'App/Protocol/Inc'):
        base+=['-I',str(folder)]
    for side in (0,1):
        for tx in (0,1):
            exe=tmp/f'test_{side}_{tx}.exe';out=tmp/f'probe_{side}_{tx}.bin'
            subprocess.run(base+[f'-DROBOT_LEG_PD_TEST_SIDE={side}','-DROBOT_LEG_VOLTAGE_PROBE=1',
                f'-DMOTOR_COMMAND_TX_ENABLED={tx}',str(tmp/'test.c'),*[str(root/p) for p in sources],
                '-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe),str(out)],check=True)
            if not tx:continue
            raw=out.read_bytes();h=header_decode(raw[:128])
            assert h['build_id']==(2026100718 if side==0 else 2026100719)
            assert h['control_mode']=='isolated leg voltage probe'
            assert record_crc(raw[:128],raw[384:],raw[128:384])==h['crc32']
            export(h,raw[384:],tmp/f'probe_{side}.json','real isolated task host fixture',raw[128:384])
            d=json.loads((tmp/f'probe_{side}.json').read_text(encoding='utf-8'))
            assert d['summary']['trigger_elapsed_ms']==2000
            assert d['summary']['post_trigger_duration_ms']==2000
            assert d['records'][0]['direction_probe_nm'][side]>0
            assert all(not any(r['joint_voltage_mv']) for r in d['records'] if not r['active'])
    print('PASS: both probe profiles, CRC, source component labeling and post-stop export')
