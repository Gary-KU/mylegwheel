"""Real task/CAN integration checks for calibrated motion and current Flash logging."""
import ast,json,subprocess,sys,tempfile
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'scripts'))
from read_lqr_startup_record import header_decode,export,record_crc
tree=ast.parse((root/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
strings={}
for node in tree.body:
 if isinstance(node,ast.Assign) and isinstance(node.value,ast.Constant):
  for target in node.targets:
   if isinstance(target,ast.Name) and target.id in ('runtime','harness'):strings[target.id]=node.value.value
prefix=strings['harness'].split('int main(void) {',1)[0]
begin=prefix.index('volatile LqrRecordDebug g_lqr_record;');end=prefix.index('void RobotTelemetry_Capture',begin)
prefix=prefix[:begin]+prefix[end:]
fixture=r'''
#include <stddef.h>
int LqrRecordStorage_Load(LqrRecordHeader *h){(void)h;return 0;}
int LqrRecordStorage_Save(const volatile LqrRecordData *r,uint32_t *error){(void)r;*error=0;return 1;}
static void pose(unsigned s) {
 float q[2];assert(leg_inverse(.03f,.095f,1.57f,1.57f,q));
 motors[g_joint_calibration[2*s].motor_id-1].joint_angle_rad=q[1];
 motors[g_joint_calibration[2*s+1].motor_id-1].joint_angle_rad=q[0];
}
static void velocity(float v){
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-v/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=v/WHEEL_RADIUS_M;
}
static int command(unsigned slot){return (int16_t)((uint16_t)data[slot/4][2*(slot%4)]|
                                       (uint16_t)data[slot/4][2*(slot%4)+1]<<8);}
static WheelBalanceTraceRow latest_trace(void){
 assert(g_balance_trace.count>0 && g_balance_trace.count<=WHEEL_BALANCE_TRACE_ROWS);
 return g_balance_trace.rows[(g_balance_trace.next+WHEEL_BALANCE_TRACE_ROWS-1U)%WHEEL_BALANCE_TRACE_ROWS];
}
static void trace_checked(void){
 WheelBalanceTraceRow row=latest_trace();
 if(row.tick_ms!=now)return; /* The actual balance ring retains its 80 ms cadence. */
 assert(row.raw_ch2==(motion_sticks.valid?motion_sticks.raw_ch2:UINT16_MAX));
 assert(row.source_frame_count==motion_sticks.frame_count);
 assert(row.sbus_age_ms==(motion_sticks.frame_count?0U:UINT32_MAX));
 assert(!!(row.input_flags&WHEEL_TRACE_INPUT_VALID)==!!motion_sticks.valid);
 assert(row.normalized_forward==motion_sticks.forward);
 assert(row.motion_status==g_robot_motion.command.status);
 assert(row.requested_forward_m_s==g_robot_motion.command.requested_forward_m_s);
 assert(row.slewed_forward_m_s==g_robot_motion.command.forward_m_s);
 assert(row.requested_yaw_rad_s==g_robot_motion.command.requested_turn_rad_s);
 assert(row.slewed_yaw_rad_s==g_robot_motion.command.turn_rad_s);
 const unsigned indices[4]={0,2,3,4};
 for(unsigned i=0;i<4;++i)assert(row.reference[i]==g_balance.reference[indices[i]]);
 assert(row.parking_goal_m==g_balance.parking_target_m);
 assert(!!(row.flags&16U)==!!g_balance.position_locked);
 assert(row.bias_offset_m==g_balance_bias.estimated_offset_m);
 assert(row.bias_applied_phi_rad==g_balance_bias.applied_phi_rad);
 assert(row.bias_gate_flags==g_balance_bias.gate_mask);
}
static void checked(void){
 assert(g_robot_control.status==1 && g_balance.status==1);
 trace_checked();
 float mean=.5f*(g_wheel_output.applied_per_side_nm[0]+g_wheel_output.applied_per_side_nm[1]);
 assert(fabsf(mean-g_wheel_output.applied_torque_nm)<1e-7f);
 assert(fabsf(g_wheel_output.applied_differential_nm)<=ROBOT_MOTION_YAW_TORQUE_LIMIT_NM+1e-8f);
 for(unsigned s=0;s<2;++s){
  float mv=g_wheel_output.applied_per_side_nm[s]*.7f/g_wheel_output.torque_ratio_nm_per_v[s]*1000;
  mv=fmaxf(-g_balance.wheel_limit_mv,fminf(g_balance.wheel_limit_mv,mv))*(s?1:-1);
  assert(abs(command((s?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1)-(int)mv)<=1);
 }
}
static void off(void){
 remote_sample.frame.channel[4]=1000;step();
 assert(!g_robot_control.enable && !g_robot_motion.neutral_ready);
 assert(!g_robot_motion.translating && !g_robot_motion.yaw_active);
 assert(g_robot_motion.command.forward_m_s==0 && g_robot_motion.command.turn_rad_s==0);
 assert(g_wheel_output.applied_differential_nm==0);
 for(unsigned i=0;i<8;++i)assert(command(i)==0);
}
static void on(void){remote_sample.frame.channel[4]=306;step();}
static void center(void){motion_sticks.forward=motion_sticks.yaw=0;for(unsigned n=0;n<435;++n)step();}
int main(int argc,char **argv){
 assert(argc==2);LqrStartupRecord_Init();
 assert(sizeof(WheelBalanceTraceRow)==180 && offsetof(WheelBalanceTraceRow,raw_ch2)==64);
 assert(offsetof(WheelBalanceTraceRow,servo_integral_next_m_s)==128 &&
        offsetof(WheelBalanceTraceRow,parking_blend)==176);
 assert(sizeof(WheelBalanceTraceDebug)==12+WHEEL_BALANCE_TRACE_ROWS*180 && sizeof(SbusSticks)==28);
 assert(WHEEL_BALANCE_TRACE_PERIOD_MS==80U);
 for(unsigned i=0;i<8;++i)motors[i].online=motors[i].joint_calibrated=1;
 pose(0);pose(1);g_balance.back_emf_ratio=0;g_dual_hold.feedforward_target_n=0;
 remote_ok=remote_fresh=1;motion_sticks.valid=motion_sticks.calibrated=1;
 motion_sticks.raw_ch2=317;motion_sticks.frame_count=77;
 assert(g_robot_motion.yaw_enable==1 && g_robot_motion.yaw_feedback_direction==-1);
 off();motion_sticks.forward=1;on();
#if !MOTOR_COMMAND_TX_ENABLED
 assert(!g_robot_control.enable && calls==0 && !g_robot_motion.translating);return 0;
#endif
 for(unsigned n=0;n<180;++n){step();checked();assert(!g_robot_motion.neutral_ready);}
 {
  WheelBalanceTraceRow row=latest_trace();
  assert(row.raw_ch2==317 && row.normalized_forward==1 && row.source_frame_count==77);
  assert(row.motion_status==ROBOT_MOTION_NOT_READY && row.requested_forward_m_s==0 && row.slewed_forward_m_s==0);
  assert((row.input_flags&63U)==63U && !(row.input_flags&WHEEL_TRACE_NEUTRAL_READY));
 }
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
 motion_sticks.calibrated=0;step();assert(g_robot_motion.command.status==ROBOT_MOTION_UNCALIBRATED);
 motion_sticks.calibrated=1;center();assert(g_robot_motion.neutral_ready);
 assert(g_balance.position_locked && !g_robot_motion.yaw_active);
 velocity(.03f);for(unsigned n=0;n<230;++n)step();
 float lean=g_balance.parking_lean_rad;assert(lean<0);velocity(0);
 motion_sticks.forward=1;
 float previous=0;
 for(unsigned n=0;n<210;++n){
  step();checked();float v=g_robot_motion.command.forward_m_s;
  assert(v>=previous && v-previous<=.004001f && v<=.400001f);previous=v;
  assert(g_balance.reference[3]==v && !g_balance.position_locked);
  assert(fabsf(g_balance.reference[2]-g_balance.state[2])<=.050001f);
  assert(!g_balance.parking_lean_learning && fabsf(g_balance.parking_lean_rad)<=BALANCE_PARKING_LEAN_LIMIT_RAD);
  assert(!g_balance.parking_damping_active);
 }
 assert(fabsf(previous-.40f)<1e-6f);
 /* Straight driving counters uncommanded yaw in both directions. */
 imu.gyro_rad_s[2]=-.2f;
 for(unsigned n=0;n<60;++n){step();checked();}
 assert(g_robot_motion.command.turn_rad_s==0 && g_robot_motion.yaw_active);
 assert(g_wheel_output.applied_differential_nm<0);
 imu.gyro_rad_s[2]=.2f;
 for(unsigned n=0;n<60;++n){step();checked();}
 assert(g_wheel_output.applied_differential_nm>0);
 imu.gyro_rad_s[2]=0;
 motion_sticks.forward=0;
 for(unsigned n=0;n<215;++n){step();checked();}
 assert(!g_robot_motion.translating && g_balance.position_locked);
 assert(g_balance.reference[3]==0);
 assert(fabsf(g_balance.parking_target_m-g_balance.state[2])<1e-6f);
 motion_sticks.forward=-1;for(unsigned n=0;n<90;++n)step();
 assert(g_balance.reference[3]<-.04f && g_balance.reference[2]<g_balance.state[2]);
 center();for(unsigned n=0;n<50;++n)step();
 motion_sticks.yaw=1;imu.gyro_rad_s[2]=0;
 previous=0;
 for(unsigned n=0;n<140;++n){
  step();checked();float diff=g_robot_motion.differential_requested_nm;
  assert(fabsf(diff-previous)<=ROBOT_MOTION_YAW_TORQUE_RATE_NM_S*.005f+1e-8f);previous=diff;
  assert(g_robot_motion.yaw_active && !g_balance.parking_lean_learning);
 }
 assert(g_robot_motion.command.turn_rad_s>1.4999f);
 assert(g_wheel_output.applied_per_side_nm[0]>g_wheel_output.applied_per_side_nm[1]);
 assert(g_wheel_output.applied_differential_nm>0);
 /* An overspeed right turn reverses the differential; both configured gyro signs work. */
 imu.gyro_rad_s[2]=-3.5f;for(unsigned n=0;n<120;++n){step();checked();}
 assert(g_robot_motion.measured_yaw_rate_rad_s==3.5f && g_wheel_output.applied_differential_nm<0);
 g_robot_motion.yaw_feedback_direction=1;imu.gyro_rad_s[2]=3.5f;step();
 assert(g_robot_motion.measured_yaw_rate_rad_s==3.5f);
 /* Balance voltage saturation suppresses steering, retaining the common command. */
 float saved_wheel_limit=g_balance.wheel_limit_mv;
 g_balance.wheel_limit_mv=100; /* Force voltage headroom exhaustion independent of LQR gains. */
 imu.gyro_rad_s[2]=0;imu.attitude_deg[BODY_PITCH_AXIS]=15/BODY_PITCH_DIRECTION;
 for(unsigned n=0;n<100;++n){step();checked();}
 assert(g_wheel_output.saturated);
 for(unsigned s=0;s<2;++s) {
  assert(fabsf(g_wheel_output.requested_mv[s])>=g_balance.wheel_limit_mv);
  assert(abs(g_wheel_output.voltage_mv[s])==100);
 }
 assert(g_wheel_output.applied_differential_nm==0);
 g_balance.wheel_limit_mv=saved_wheel_limit;
 imu.attitude_deg[BODY_PITCH_AXIS]=0;
 motion_sticks.yaw=-1;for(unsigned n=0;n<260;++n){step();checked();}
 assert(g_robot_motion.command.turn_rad_s<-1.4999f && g_wheel_output.applied_differential_nm<0);
 center();assert(!g_robot_motion.yaw_active && g_wheel_output.applied_differential_nm==0);
 assert(g_wheel_output.applied_per_side_nm[0]==g_wheel_output.applied_per_side_nm[1]);
 motion_sticks.forward=.5f;for(unsigned n=0;n<30;++n)step();
 motion_sticks.valid=0;motion_sticks.frame_count=0;
 /* Wait through a complete diagnostic interval before inspecting its input
  * snapshot; controller input rejection still happens on the first tick. */
 for(unsigned n=0;n<WHEEL_BALANCE_TRACE_PERIOD_MS/5U+1U;++n)step();
 {
  WheelBalanceTraceRow row=latest_trace();
  assert(row.raw_ch2==UINT16_MAX && isnan(row.normalized_forward));
  assert(row.sbus_age_ms==UINT32_MAX && row.source_frame_count==0);
  assert(!(row.input_flags&WHEEL_TRACE_INPUT_VALID) && row.motion_status==ROBOT_MOTION_REMOTE_INVALID);
 }
 assert(!g_robot_motion.translating && g_balance.motion_braking && !g_balance.position_locked && g_balance.reference[3]==0);
 motion_sticks.valid=1;motion_sticks.frame_count=78;
 for(unsigned n=0;n<50;++n)step();assert(!g_robot_motion.neutral_ready);
 center();assert(g_robot_motion.neutral_ready);motion_sticks.forward=.5f;step();
 off();
 uint32_t frozen_sequence=g_balance_trace.sequence;
 WheelBalanceTraceRow frozen_row=latest_trace();
 assert(g_lqr_record.trigger_index<g_lqr_record.count);
 uint32_t trigger_tick=g_lqr_record_data.rows[g_lqr_record.trigger_index].tick_ms;
 for(unsigned n=0;n<400;++n){step();assert(!g_robot_control.enable);}
 assert(g_balance_trace.sequence==frozen_sequence);
 WheelBalanceTraceRow still_frozen=latest_trace();
 assert(memcmp(&frozen_row,&still_frozen,sizeof(frozen_row))==0);
 assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.reason==LQR_RECORD_STOP);
 assert(g_lqr_record_data.rows[g_lqr_record.count-1].tick_ms-trigger_tick==2000);
 LqrStartupRecord_Poll();for(unsigned n=0;n<40;++n)step();LqrStartupRecord_Poll();
 assert(g_lqr_record.state==LQR_RECORD_SAVED);
 FILE *f=fopen(argv[1],"wb");assert(f);
 assert(fwrite((const void*)&g_lqr_record_data,offsetof(LqrRecordData,rows)+g_lqr_record.count*sizeof(LqrRecordRow),1,f)==1);fclose(f);
 /* Restart held at the end remains unarmed; remote loss zeros all actuators. */
 on();for(unsigned n=0;n<180;++n)step();assert(!g_robot_motion.neutral_ready);
 assert(g_balance_trace.count<256 && g_balance_trace.sequence>frozen_sequence);
 center();assert(g_robot_motion.neutral_ready);motion_sticks.yaw=1;step();
 remote_ok=0;step();assert(!g_robot_control.enable && !g_robot_motion.yaw_active);
 for(unsigned i=0;i<8;++i)assert(command(i)==0);
 remote_ok=1;off();on();center();for(unsigned n=0;n<100;++n)step();center();
 motion_sticks.yaw=1;for(unsigned n=0;n<40;++n)step();assert(g_robot_motion.yaw_active);
 imu.attitude_deg[BODY_PITCH_AXIS]=40/BODY_PITCH_DIRECTION;
 for(unsigned n=0;n<9;++n)step();
 assert(!g_robot_control.enable && !g_robot_motion.yaw_active && !g_robot_motion.neutral_ready);
 assert(g_robot_control.last_fault.reason==10);
 for(unsigned i=0;i<8;++i)assert(command(i)==0);
 puts("PASS: real task neutral arming, calibrated CH2 ramp/reference/release/reverse, parking relatch, bounded motion learning, CH1 signed differential/CAN, gyro overspeed, balance headroom, invalid input, OFF/loss zero and V5 tail");
}
'''
sources=['App/Protocol/Src/robot_motion_command.c','App/Tasks/Src/wheel_leg_task.c', 'App/Tasks/Src/wheel_leg_observer.c',
 'App/Services/Src/motor_command.c','App/Services/Src/lqr_startup_record.c',
 'tests/support/wheel_leg/Src/wheel_leg_control.c','Control/WheelLeg/Src/lqr_k.c',
 'Control/WheelLeg/Src/leg_pos.c','tests/support/wheel_leg/Src/leg_trajectory.c']
with tempfile.TemporaryDirectory(prefix='remote_motion_') as d:
 tmp=Path(d);(tmp/'runtime.h').write_text(strings['runtime'])
 for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):(tmp/name).write_text('#include "runtime.h"\n')
 (tmp/'test.c').write_text(prefix+fixture)
 # This long prescribed-feedback scenario must fit the V5 buffer so that its
 # final OFF event, steering directions and 2s stop tail remain inspectable.
 # Only this host fixture uses100ms V5 rows; production keeps its configured
 # V5 cadence. The independent frozen balance ring uses80ms samples.
 base=['gcc','-DROBOT_NATIVE_LQR=0','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-DLQR_RECORD_PERIOD_MS=100U']
 for p in (tmp,root/'App/Tasks/Inc',root/'App/Services/Inc',root/'App/Protocol/Inc',
           root/'Control/WheelLeg/Inc',root/'tests/support/wheel_leg/Inc'):base+=['-I',str(p)]
 for tx in (0,1):
  exe=tmp/f'test{tx}.exe';out=tmp/f'record{tx}.bin'
  subprocess.run(base+[f'-DMOTOR_COMMAND_TX_ENABLED={tx}',str(tmp/'test.c'),
   *[str(root/p) for p in sources],'-lm','-o',str(exe)],check=True)
  subprocess.run([str(exe),str(out)],check=True)
  if not tx:continue
  raw=out.read_bytes();h=header_decode(raw[:128]);assert h['version']==5
  payload=h['payload_offset']
  assert record_crc(raw[:128],raw[payload:],raw[128:payload])==h['crc32']
  export(h,raw[payload:],tmp/'export.json','real task/CAN fixture',raw[128:payload])
  rows=json.loads((tmp/'export.json').read_text(encoding='utf-8'))['records']
  assert any(r['translating'] and r['forward_target_m_s']>.09 for r in rows)
  assert any(r['differential_applied_nm']>0 for r in rows)
  assert any(r['differential_applied_nm']<0 for r in rows)
  assert all(r['left_wheel_torque_command_nm']==r['wheel_applied_per_side_nm'][0] for r in rows)
  assert all(r['right_wheel_torque_command_nm']==r['wheel_applied_per_side_nm'][1] for r in rows)
  assert 'yaw_rate_rad_s' in (tmp/'export.csv').read_text(encoding='utf-8-sig')
  assert all('motion_lean_learning' in r and 'motion_stall_hold' in r for r in rows)
  assert any(r['motion_damping_active'] for r in rows)
  assert max(abs(r['motion_damping_command_nm']) for r in rows)<=.030001
  import csv
  csv_rows=list(csv.reader((tmp/'export.csv').open(encoding='utf-8-sig',newline='')))
  assert len({len(r) for r in csv_rows})==1
  assert 'motion_damping_mNm' in csv_rows[0]
  assert all(row[csv_rows[0].index('motion_damping_mNm')]!='' for row in csv_rows[1:])
  print('PASS: V5 CRC, measured yaw tail, actual per-wheel command decode and motion CSV')
