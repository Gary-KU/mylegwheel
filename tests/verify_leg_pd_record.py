"""Real task + recorder integration for both isolated diagnostic profiles."""
import ast
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'scripts'))
from read_lqr_startup_record import header_decode, configuration_decode, rows_decode, record_crc

tree = ast.parse((root/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings = {}
for node in tree.body:
    if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ('runtime','harness'):
                strings[target.id] = ast.literal_eval(node.value)
prefix = strings['harness'].split('int main(void) {',1)[0]
begin = prefix.index('volatile LqrRecordDebug g_lqr_record;')
end = prefix.index('void RobotTelemetry_Capture',begin)
prefix = prefix[:begin]+prefix[end:]
fixture = r'''
static unsigned saves;
int LqrRecordStorage_Load(LqrRecordHeader *header) {(void)header;return 0;}
int LqrRecordStorage_Save(const volatile LqrRecordData *record,uint32_t *error) {
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
 assert(!g_robot_control.remote_switch_on && g_lqr_record.storage_busy);
 assert(record->header.commit==LQR_RECORD_COMMIT);
 ++saves;*error=0;return 1;
}
static void pose(unsigned side,float angle) {
 float q[2];
 assert(leg_inverse(.03f+.07f*cosf(angle),.07f*sinf(angle),1.57f,1.57f,q));
 motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=q[1];
 motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=q[0];
}
static int command(unsigned slot) {
 return (int16_t)((uint16_t)data[slot/4][2*(slot%4)] |
                 (uint16_t)data[slot/4][2*(slot%4)+1]<<8);
}
int main(int argc,char **argv) {
 assert(argc==2);const unsigned side=ROBOT_LEG_PD_TEST_SIDE;
 const float neutral=1.57079632679f,rad=.017453292519943295f;
 LqrStartupRecord_Init();
 for(unsigned i=0;i<8;++i) motors[i].online=motors[i].joint_calibrated=1;
 pose(0,neutral);pose(1,neutral);pose(side,neutral+16*rad);
 remote_ok=remote_fresh=1;remote_sample.frame.channel[4]=1000;step();
 remote_sample.frame.channel[4]=306;step();
 assert(g_robot_control.enable==0 && g_lqr_record.state==LQR_RECORD_READY && g_lqr_record.count==0);
 pose(side,neutral-10*rad);
 remote_sample.frame.channel[4]=1000;step();remote_sample.frame.channel[4]=306;step();
 assert(g_lqr_record.state==LQR_RECORD_RUNNING && g_lqr_record.count==1);
 assert(g_lqr_record_data.header.build_id==(side?2026100717U:2026100716U));
 assert(g_lqr_record_data.header.wheel_scale==0 && g_lqr_record_data.header.leg_scale==0);
 for(unsigned i=0;i<600;++i) step();
 assert(!g_robot_control.enable && g_lqr_record.state==LQR_RECORD_RUNNING);
 uint32_t trigger=g_lqr_record.trigger_index;
 assert(trigger>0 && g_lqr_record_data.rows[trigger].flags%2==0);
 assert(g_lqr_record_data.rows[trigger].tick_ms-g_lqr_record_data.header.start_tick_ms==3000);
 pose(side,neutral-8*rad); /* Passive movement after torque is removed. */
 for(unsigned i=0;i<400;++i) step();
 assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.reason==LQR_RECORD_STOP);
 LqrStartupRecord_Poll();assert(!saves); /* CH5 still high blocks Flash. */
 for(unsigned row=0;row<g_lqr_record.count;++row) {
  const volatile LqrRecordRow *r=&g_lqr_record_data.rows[row];
  assert(r->wheel_voltage_mv[0]==0 && r->wheel_voltage_mv[1]==0);
  assert(r->joint_voltage_mv[2*(1-side)]==0 && r->joint_voltage_mv[2*(1-side)+1]==0);
  assert(r->support_n[0]==0 && r->support_n[1]==0 && r->sync_nm[0]==0 && r->sync_nm[1]==0);
  assert(r->lqr_pitch_nm[0]==0 && r->lqr_pitch_nm[1]==0);
  assert(r->total_pitch_nm[1-side]==0);
  for(unsigned j=0;j<4;++j) assert(abs(r->joint_voltage_mv[j])<=600);
  if(row>=trigger) for(unsigned j=0;j<4;++j) assert(r->joint_voltage_mv[j]==0);
 }
 assert(fabsf(g_lqr_record_data.rows[g_lqr_record.count-1].leg_angle_rad[side]-(neutral-8*rad))<1e-5f);
 remote_sample.frame.channel[4]=1000;step();LqrStartupRecord_Poll();
 for(unsigned i=0;i<25;++i) step();
 LqrStartupRecord_Poll();assert(saves==1 && g_lqr_record.state==LQR_RECORD_SAVED);
 for(unsigned i=0;i<8;++i) assert(command(i)==0);
 FILE *f=fopen(argv[1],"wb");assert(f);
 assert(fwrite((const void*)&g_lqr_record_data,384+g_lqr_record.count*sizeof(LqrRecordRow),1,f)==1);fclose(f);
 uint32_t kept=g_lqr_record.count;
 remote_sample.frame.channel[4]=306;step();
 assert(g_lqr_record.state==LQR_RECORD_SAVED && g_lqr_record.count==kept);
 puts("PASS: real isolated task records accepted start, 3s timeout, 2s passive tail, retained trial and CH5-gated save");
}
'''
sources = ['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_command.c',
    'App/Services/Src/lqr_startup_record.c','tests/support/wheel_leg/Src/wheel_leg_control.c',
    'Control/WheelLeg/Src/lqr_k.c','Control/WheelLeg/Src/leg_pos.c',
    'tests/support/wheel_leg/Src/leg_trajectory.c']
with tempfile.TemporaryDirectory(prefix='leg_pd_record_') as directory:
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
        exe=tmp/f'test_{side}.exe';out=tmp/f'pd_{side}.bin'
        subprocess.run(base+[f'-DROBOT_LEG_PD_TEST_SIDE={side}','-DMOTOR_COMMAND_TX_ENABLED=1',
            str(tmp/'test.c'),*[str(root/p) for p in sources],'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe),str(out)],check=True)
        raw=out.read_bytes();h=header_decode(raw[:128]);c=configuration_decode(raw[128:384])
        assert h['control_mode']=='isolated leg PD' and h['isolated_leg_side']==('left' if side==0 else 'right')
        assert h['position_reference_semantics']=='unused by isolated leg diagnostic control'
        assert abs(c['angle_limit_nm']-.05)<1e-8 and c['sync_ratio']==0
        assert record_crc(raw[:128],raw[384:],raw[128:384])==h['crc32']
        rows=rows_decode(raw[384:],h['count'],h['start_tick_ms'],h['version'],c)
        assert rows[-1]['elapsed_ms']==5000 and rows[h['trigger_index']]['elapsed_ms']==3000
        assert all(r['joint_feedback_valid']==[True]*4 for r in rows)
        assert any(r['total_pitch_nm'][side]>0 for r in rows[:h['trigger_index']])
    print('PASS: both profile IDs, actual record configuration, CRC/decoder and zero inactive outputs')
