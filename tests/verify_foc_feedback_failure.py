"""Real A1 bytes -> original health snapshot -> observer-attributed fault latch.

No target access. Exercises recovery before latch, not a later live reread.
STOP/failed restarts must retain evidence; only accepted active service clears.
"""
import ast
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
pair_path=ROOT/'tests/verify_foc_feedback_pair.py'
pair_ns={'__file__':str(pair_path)}
exec(compile(pair_path.read_text(encoding='utf8').split("fixture=r'''")[0],str(pair_path),'exec'),pair_ns)
tree=ast.parse(pair_path.read_text(encoding='utf8'))
pair_fixture=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign)
    and any(isinstance(t,ast.Name) and t.id=='fixture' for t in n.targets))
prefix=pair_ns['prefix']+pair_fixture.split('int main(void) {')[0]
fixture=r'''
static void fast_all_except(unsigned except) {
 for(unsigned id=1;id<=8;++id)if((MOTOR_FOC_NODE_MASK&(1U<<(id-1))) && id!=except)fast_motor(id,0,0);
}
static void pages_except(unsigned target,unsigned missing_kind) {
 for(unsigned id=1;id<=8;++id)if(MOTOR_FOC_NODE_MASK&(1U<<(id-1))) {
  uint8_t status[8]={0xa1,0,0x3e,0,0,0,0,0};
  uint8_t diagnostic[8]={0xa1,0,0,0,0,0,0,0};
  if(id!=target || missing_kind!=0)assert(MotorFoc_OnFrame(0x180+id,status,now));
  if(id!=target || missing_kind!=1)assert(MotorFoc_OnFrame(0x280+id,diagnostic,now));
 }
}
static MotorFeedbackHealth read_health(unsigned id,unsigned required) {
 MotorFeedback f,old;MotorFeedbackHealth h;
 unsigned online=MotorFoc_GetFeedbackWithHealth(id,&f,&h);
 assert(MotorFoc_GetFeedback(id,&old)==online && old.online==f.online);
 assert(h.valid && h.motor_id==id && h.sample_tick_ms==now);
 assert(online==(h.reject_mask==0) && (h.reject_mask&required)==required);
 return h;
}
static void aged_page(unsigned id,unsigned kind,unsigned wrap) {
 setup(5,6,wrap);unsigned start=base;
 for(unsigned dt=5;dt<=40;dt+=5) {
  now=start+dt;pages_except(id,kind);
  fast_all_except(dt==40?id:0);observe();
 }
 assert(observation.feedback_detail==WHEEL_LEG_FEEDBACK_OK);
 assert(!read_health(id,0).reject_mask); /* Exactly40ms is still legal. */
 now=start+41;pages_except(id,kind);fast_all_except(id);observe();
 unsigned expected=kind==0?MOTOR_FEEDBACK_REJECT_STATUS_STALE:MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_STALE;
 MotorFeedbackHealth h=read_health(id,expected);
 assert(h.reject_mask==expected && h.fast_age_ms==6);
 assert((kind==0?h.status_age_ms:h.diagnostic_age_ms)==41);
 assert(observation.feedback_detail==WHEEL_LEG_FEEDBACK_MOTOR && observation.feedback_motor_id==id);
 assert(!g_motor_foc_feedback_failure.valid); /* Passive observation never latches. */
 WheelLegDebug failed=observation;
 /* Recover the live node and consume more GetFeedback calls before the
  * controller records its fault. The original observer sample must survive. */
 pages_except(0,0);fast_all_except(0);assert(!read_health(id,0).reject_mask);
 WheelLeg_LatchFeedbackFailure(&failed);
 MotorFocFeedbackFailureDebug saved=g_motor_foc_feedback_failure;
 assert(saved.valid && saved.schema_version==1 && saved.controller_tick_ms==failed.tick_ms);
 assert(saved.feedback_detail==WHEEL_LEG_FEEDBACK_MOTOR && saved.observer_age_ms==6);
 assert(saved.sample.motor_id==id && saved.sample.reject_mask==expected);
 assert(saved.sample.sample_tick_ms==start+41 && saved.sample.fast_age_ms==6);
 assert((kind==0?saved.sample.status_age_ms:saved.sample.diagnostic_age_ms)==41);
 assert(saved.sample.status_count && saved.sample.diagnostic_count && saved.sample.fast_count);
 MotorFoc_StopAll();observe();WheelLeg_LatchFeedbackFailure(&observation);
 assert(!memcmp(&saved,(const void*)&g_motor_foc_feedback_failure,sizeof(saved)));
 if(wrap)assert(now<start);
}
static void failure_bits(void) {
 MotorFeedbackHealth h;
 setup(5,6,0);now+=5;pages_except(0,0);fast_all_except(0);
 uint8_t status[8]={0xa1,0,0x36,0,0,0,0,0};
 assert(MotorFoc_OnFrame(0x185,status,now));
 h=read_health(5,MOTOR_FEEDBACK_REJECT_STATUS_SENSOR);assert(!(h.flags&8));
 setup(5,6,0);uint8_t diagnostic[8]={0xa1,0,7,0,0,0,3,2};
 assert(MotorFoc_OnFrame(0x285,diagnostic,now));
 h=read_health(5,MOTOR_FEEDBACK_REJECT_FAULTS|MOTOR_FEEDBACK_REJECT_BOARD_FAULTS|MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_HEALTH);
 assert(h.faults==7 && h.board_faults==3 && h.diagnostic_flags==2);
 for(unsigned kind=0;kind<3;++kind) {
  setup(5,6,0);uint8_t d[8]={0xa1,0x3e,0,0,0,0,0,0};put16(d+2,1000);
  if(kind==0)d[0]=0;
  if(kind==1)put16(d+2,6284);
  if(kind==2)put16(d+4,INT16_MAX);
  assert(MotorFoc_OnFrame(0x705,d,now));
  h=read_health(5,kind==2?MOTOR_FEEDBACK_REJECT_FAST_CLIPPED:MOTOR_FEEDBACK_REJECT_FAST_INVALID);
  if(kind==2)assert(h.fast_speed_clipped);else assert(!h.fast_valid);
 }
 setup(5,6,0);MotorFoc_InvalidateFromISR();h=read_health(5,MOTOR_FEEDBACK_REJECT_EPOCH);
 assert(h.node_epoch!=h.transport_epoch);
 setup(5,6,0);g_motor_foc.calibration[4].angle_anchor_confirmed=0;
 h=read_health(5,MOTOR_FEEDBACK_REJECT_ANCHOR|MOTOR_FEEDBACK_REJECT_CONFIGURATION);
 assert(!h.anchor_confirmed && !h.history_configuration_valid);
 setup(5,6,0);g_motor_foc.calibration[4].joint_timing_confirmed=0;
 h=read_health(5,MOTOR_FEEDBACK_REJECT_TIMING|MOTOR_FEEDBACK_REJECT_CONFIGURATION);
 assert(!h.timing_confirmed);
 setup(5,6,0);now+=11;h=read_health(5,MOTOR_FEEDBACK_REJECT_FAST_STALE);assert(h.fast_age_ms==11);
 setup(5,6,0);g_motor_foc.node[4].status_count=g_motor_foc.node[4].diagnostic_count=g_motor_foc.node[4].fast_count=0;
 read_health(5,MOTOR_FEEDBACK_REJECT_STATUS_MISSING|MOTOR_FEEDBACK_REJECT_DIAGNOSTIC_MISSING|MOTOR_FEEDBACK_REJECT_FAST_MISSING);
}
static void qualification_equivalence(void) {
 uint32_t random=0x13579bdfU;
 for(unsigned trial=0;trial<4096;++trial) {
  setup(5,6,trial%8==0);random=random*1664525U+1013904223U;
  unsigned bits=trial<17?(trial?(1U<<(trial-1U)):0U):random;
  volatile MotorFocNode *n=&g_motor_foc.node[4];
  volatile MotorFocCalibration *c=&g_motor_foc.calibration[4];
  n->status_count=(bits&1)?0:1;n->diagnostic_count=(bits&2)?0:1;n->fast_count=(bits&4)?0:1;
  n->status_ms=now-((bits&8)?41:40);n->diagnostic_ms=now-((bits&16)?41:40);
  n->fast_ms=now-((bits&32)?11:10);n->epoch=g_motor_foc.transport_epoch+!!(bits&64);
  n->flags=(bits&128)?0x36:0x3e;n->faults=!!(bits&256);n->board_faults=!!(bits&512);
  n->diagnostic_flags=(bits&1024)?0x23:0x40;n->fast_valid=!(bits&2048);n->fast_speed_clipped=!!(bits&4096);
  c->angle_anchor_confirmed=!(bits&8192);c->joint_timing_confirmed=!(bits&16384);
  if(bits&32768)c->angle_anchor_rad+=.001f;
  unsigned expected=n->epoch==g_motor_foc.transport_epoch && n->status_count &&
   (uint32_t)(now-n->status_ms)<=40U && n->diagnostic_count && (uint32_t)(now-n->diagnostic_ms)<=40U &&
   (n->flags&8U) && !n->faults && !n->board_faults && !(n->diagnostic_flags&0x23U) &&
   n->fast_valid && !n->fast_speed_clipped && n->fast_count && (uint32_t)(now-n->fast_ms)<=10U &&
   c->angle_anchor_confirmed && c->joint_timing_confirmed && !(bits&32768);
  MotorFeedback f;MotorFeedbackHealth h;
  assert(MotorFoc_GetFeedbackWithHealth(5,&f,&h)==expected);
  assert((h.reject_mask==0)==expected);
 }
}
static void lifecycle(void) {
 aged_page(5,0,0);MotorFocFeedbackFailureDebug saved=g_motor_foc_feedback_failure;
 now+=5;pages_except(0,0);fast_all_except(0);
 uint8_t bad_status[8]={0xa1,0,0x36,0,0,0,0,0};assert(MotorFoc_OnFrame(0x187,bad_status,now));
 observe();assert(observation.feedback_motor_id==7);WheelLeg_LatchFeedbackFailure(&observation);
 assert(!memcmp(&saved,(const void*)&g_motor_foc_feedback_failure,sizeof(saved)));
 g_motor_foc_failure.valid=1;g_motor_foc_failure.reason=MOTOR_FOC_FAILURE_ACTIVE_MASK;
 g_motor_foc_failure.node_id=1;g_motor_foc_failure.tick_ms=now;
 assert(!memcmp(&saved,(const void*)&g_motor_foc_feedback_failure,sizeof(saved)));
 idle_ready();calibrate();g_motor_foc.calibration[2].calibration_confirmed=0;
 float torque[8]={0};assert(!cycle(torque,0x44U));
 assert(!memcmp(&saved,(const void*)&g_motor_foc_feedback_failure,sizeof(saved)));
 idle_ready();calibrate();assert(cycle(torque,0x44U));
 assert(!g_motor_foc_feedback_failure.valid);MotorFoc_StopAll();
 /* A different accepted run can now capture a different first node. */
 aged_page(7,1,0);assert(g_motor_foc_feedback_failure.sample.motor_id==7);
 MotorFoc_Init();assert(!g_motor_foc_feedback_failure.valid);
 /* A candidate from an earlier observation cannot be assigned to a new tick. */
 setup(5,6,0);now+=5;pages_except(0,0);fast_all_except(0);
 uint8_t status[8]={0xa1,0,0x36,0,0,0,0,0};assert(MotorFoc_OnFrame(0x185,status,now));
 observe();WheelLegDebug old=observation;
 now+=5;pages_except(0,0);fast_all_except(0);observe();
 WheelLeg_LatchFeedbackFailure(&old);assert(!g_motor_foc_feedback_failure.valid);
}
int main(void) {
 hfdcan1.Instance=&hw;
 for(unsigned i=0;i<8;++i)individual_slew[i]=1;
 const unsigned ids[6]={1,2,3,5,6,7};
 for(unsigned i=0;i<6;++i)for(unsigned kind=0;kind<2;++kind)aged_page(ids[i],kind,0);
 aged_page(5,0,1);failure_bits();qualification_equivalence();lifecycle();
 puts("PASS: real A1 pages -> observer fault attribution for all six IDs; STATUS/DIAG40 accepted and41 rejected with FAST6; UINT32 wrap");
 puts("PASS: original health survives RX recovery/STOP, passive observation never latches, first fault retained, failed restart retains, accepted run and Init clear");
 puts("PASS: sensor/fault/board/DIAG flags, bad/invalid/clipped/stale/missing FAST, epochs, anchor/timing/configuration and old-observation mismatch");
 puts("PASS:4096 combinations match the independent original online predicate including40/41ms,10/11ms, missing counts, configuration and unsigned wrap");
 return 0;
}
'''

def main():
    with tempfile.TemporaryDirectory(prefix='foc_feedback_failure_') as directory:
        work=Path(directory)
        (work/'runtime.h').write_text(pair_ns['runtime'],encoding='utf8')
        for name in ('main.h','FreeRTOS.h','task.h','bsp_fdcan.h'):
            (work/name).write_text('#include "runtime.h"\n',encoding='utf8')
        (work/'test.c').write_text(prefix+fixture,encoding='utf8')
        command=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',
            '-DMYLEGWHEEL_A1_CURRENT=1','-DROBOT_LEGS_ONLY_CALIBRATION=0','-I',str(work)]
        for folder in ('App/Tasks/Inc','App/Services/Inc','App/Protocol/Inc','Control/WheelLeg/Inc','BSP/Peripherals/Inc','tests/support/wheel_leg/Inc'):
            command+=['-I',str(ROOT/folder)]
        sources=['App/Tasks/Src/wheel_leg_observer.c','App/Services/Src/motor_foc.c','App/Services/Src/joint_calibration.c',
            'App/Services/Src/motor_command.c','Control/WheelLeg/Src/lqr_k.c','Control/WheelLeg/Src/leg_pos.c',
            'tests/support/wheel_leg/Src/leg_trajectory.c','BSP/Peripherals/Src/bsp_fdcan_deadline.c']
        exe=work/'test.exe'
        subprocess.run(command+[str(work/'test.c'),*[str(ROOT/s) for s in sources],'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        # Compile the real controller fault recorder into this translation
        # unit. Unused application flows are discarded, so this case tests
        # the actual attribution hook without prescribing a control plant.
        task_prefix=prefix.replace('void WheelLeg_NotifyCanTransportFault(void) {}','')
        task_prefix=task_prefix.replace('uint32_t WheelLeg_CanCommunicationStopped(void) {return 1U;}','')
        task_prefix=task_prefix.replace('volatile WheelLegDebug g_wheel_leg;','')
        task_prefix=task_prefix.replace('volatile float g_wheel_leg_reference[2][6];','')
        task_prefix+='\n#include "'+(ROOT/'App/Tasks/Src/wheel_leg_task.c').as_posix()+'"\n'
        task_prefix+=r'''
volatile LqrRecordDebug g_lqr_record;
uint8_t SBUS_GetSticks(SbusSticks *out) {memset(out,0,sizeof(*out));return 0;}
uint8_t SBUS_GetState(SbusState *out) {memset(out,0,sizeof(*out));return 0;}
void LqrStartupRecord_Begin(const WheelLegDebug *state) {(void)state;}
void LqrStartupRecord_Capture(const WheelLegDebug *state) {(void)state;}
void RobotTelemetry_Capture(const WheelLegDebug *state) {(void)state;}
'''
        task_fixture=fixture.replace('WheelLeg_LatchFeedbackFailure(&failed);',
            'robot_control_record_fault(&failed,2U);assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK && g_robot_control.last_fault.motor_id==id);')
        (work/'task_hook.c').write_text(task_prefix+task_fixture,encoding='utf8')
        subprocess.run(command+['-ffunction-sections','-fdata-sections','-Wl,--gc-sections',
            str(work/'task_hook.c'),*[str(ROOT/s) for s in sources],
            str(ROOT/'App/Protocol/Src/robot_motion_command.c'),'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        print('PASS: actual robot_control_record_fault path attributes and locks the original A1 failure')

if __name__=='__main__':main()
