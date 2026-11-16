"""Recorder/storage golden fixtures at20ms; production40ms cadence covers20s."""
from pathlib import Path
import subprocess
import tempfile
import struct
import sys
import zlib
import json

root = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
from read_lqr_startup_record import header_decode, rows_decode, export, configuration_decode, record_crc
mock = r'''
#ifndef MOCK_H
#define MOCK_H
#include <stdint.h>
#define FLASH_NB_32BITWORD_IN_FLASHWORD 8U
#define HAL_OK 0
#define FLASH_TYPEPROGRAM_FLASHWORD 1U
#define FLASH_TYPEERASE_SECTORS 0U
#define FLASH_BANK_1 1U
#define FLASH_SECTOR_6 6U
#define FLASH_VOLTAGE_RANGE_3 2U
typedef struct {uint32_t TypeErase,Banks,Sector,NbSectors,VoltageRange;} FLASH_EraseInitTypeDef;
uint32_t HAL_GetTick(void);
int HAL_FLASH_Unlock(void);
int HAL_FLASH_Lock(void);
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *,uint32_t *);
int HAL_FLASH_Program(uint32_t,uint32_t,uintptr_t);
uint32_t HAL_FLASH_GetError(void);
#endif
'''
harness = r'''
#include "lqr_startup_record.h"
#include "wheel_leg_config.h"
#include "joint_calibration.h"
#include "mock.h"
#include "motor_foc.h"
#include <windows.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
volatile RobotControlDebug g_robot_control;
volatile DualHoldDebug g_dual_hold;
volatile BalanceDebug g_balance;
volatile WheelOutputDebug g_wheel_output;
volatile JointOutputDebug g_joint_output;
volatile WheelTestDebug g_wheel_test;
volatile RobotMotionDebug g_robot_motion;
volatile MotorFocDebug g_motor_foc;
static unsigned transport_stopped=1,refresh_nodes=1;
uint32_t MotorFoc_TransportMotionStopped(void) {return transport_stopped;}
const JointCalibration g_joint_calibration[4]={
 {1,JOINT_LEFT_FRONT,1,1.57f,1,-1},{2,JOINT_LEFT_REAR,2,1.57f,1,-1},
 {5,JOINT_RIGHT_FRONT,3,1.57f,1,1},{6,JOINT_RIGHT_REAR,4,1.57f,1.5f,1}};
static uint32_t now,erases,programs,last_address,fail_program;
static unsigned unlocked;
uint32_t HAL_GetTick(void) {
 if(refresh_nodes)for(unsigned i=0;i<8;++i) {
  g_motor_foc.node[i].status_ms=now;g_motor_foc.node[i].status_count=1;
  g_motor_foc.node[i].fast_ms=now;
 }
 return now;
}
int HAL_FLASH_Unlock(void) {unlocked=1;return 0;}
int HAL_FLASH_Lock(void) {unlocked=0;return 0;}
uint32_t HAL_FLASH_GetError(void) {return 123;}
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *bad) {
 assert(unlocked && g_lqr_record.storage_busy && !g_robot_control.enable);
 assert(!g_dual_hold.enable && !g_balance.enable && !g_wheel_test.enable);
 assert(e->TypeErase==0 && e->Banks==1 && e->Sector==6 && e->NbSectors==2);
 memset((void*)LQR_RECORD_FLASH_ADDRESS,255,LQR_RECORD_FLASH_BYTES);*bad=UINT32_MAX;++erases;return 0;
}
int HAL_FLASH_Program(uint32_t type,uint32_t at,uintptr_t source) {
 assert(type==1 && unlocked && at>=LQR_RECORD_FLASH_ADDRESS);
 assert(at+32<=LQR_RECORD_FLASH_ADDRESS+LQR_RECORD_FLASH_BYTES && !(at%32) && !(source%32));
 ++programs;if(fail_program && programs==fail_program)return 1;
 unsigned char *dst=(void*)(uintptr_t)at,*src=(void*)source;
 for(unsigned i=0;i<32;i++){assert(dst[i]==255);dst[i]=src[i];}
 last_address=at;return 0;
}
static void off(void) {
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;
 g_robot_control.remote_switch_on=0;
 for(unsigned i=0;i<8;++i){g_motor_foc.node[i].flags=g_motor_foc.node[i].fast_flags=62;g_motor_foc.node[i].mode=0;g_motor_foc.node[i].fast_iq_a=0;}
}
static void on(void) {
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=1;
 g_robot_control.status=g_dual_hold.status=g_balance.status=1;
 for(unsigned i=0;i<8;++i){g_motor_foc.node[i].flags=g_motor_foc.node[i].fast_flags=63;g_motor_foc.node[i].mode=3;g_motor_foc.node[i].fast_iq_a=.25f*(i+1);}
}
static void rearm(void) {
 off();g_lqr_record.rearm_request=1;now+=200;LqrStartupRecord_Poll();
 now+=200;LqrStartupRecord_Poll();assert(g_lqr_record.state==LQR_RECORD_READY);
}
int main(int argc,char **argv) {
 assert(sizeof(LqrRecordHeader)==128 && sizeof(LqrRecordRow)==512 &&
        sizeof(LqrRecordConfiguration)==384 && sizeof(LqrRecordData)<=LQR_RECORD_FLASH_BYTES);
 assert(VirtualAlloc((void*)LQR_RECORD_FLASH_ADDRESS,LQR_RECORD_FLASH_BYTES,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE)==(void*)LQR_RECORD_FLASH_ADDRESS);
 memset((void*)LQR_RECORD_FLASH_ADDRESS,255,LQR_RECORD_FLASH_BYTES);
 if(argc>1 && strcmp(argv[1],"cadence")==0) {
  assert(LQR_RECORD_PERIOD_MS==40U);
  LqrStartupRecord_Init();on();
  WheelLegDebug sample={0};sample.period_ms=5;sample.tick_ms=100;
  LqrStartupRecord_Begin(&sample);
  for(unsigned i=0;i<4000;++i) {
   sample.tick_ms=100+5*i;LqrStartupRecord_Capture(&sample);
   now=sample.tick_ms;LqrStartupRecord_Poll();
  }
  assert(g_lqr_record.count==500 && g_lqr_record.reason==LQR_RECORD_FULL);
  assert(g_lqr_record_data.header.period_ms==40 &&
         g_lqr_record_data.rows[499].tick_ms==20060);
  assert(!erases && !programs && g_robot_control.enable);
  puts("PASS: production40ms cadence retains20s; recorder capacity never stops control or writes active Flash");
  return 0;
 }
 if(argc>1 && strcmp(argv[1],"restart")==0) {
  LqrStartupRecord_Init();on();
  WheelLegDebug sample={0};sample.period_ms=5;sample.tick_ms=100;
  sample.imu_valid=1;sample.body_pitch_rad=.1f;
  g_balance.position_locked=1;g_balance.parking_target_m=.25f;
  LqrStartupRecord_Begin(&sample);LqrStartupRecord_Capture(&sample);
  off();g_balance.position_locked=0;sample.tick_ms=105;
  LqrStartupRecord_Capture(&sample);
  assert(g_lqr_record.state==LQR_RECORD_RUNNING && g_lqr_record.count==2);
  assert(g_lqr_record.trigger_index==1 && !(g_lqr_record_data.rows[1].flags&1U));
  sample.tick_ms=125;sample.body_pitch_rad=.2f;LqrStartupRecord_Capture(&sample);
  assert(g_lqr_record.count==3 && !(g_lqr_record_data.rows[2].flags&1U));
  assert(g_lqr_record_data.rows[2].reference[2]==.25f);
  LqrRecordRow retained[3];memcpy(retained,(const void*)g_lqr_record_data.rows,sizeof(retained));
  /* Restart before the next20ms tail tick: neither Begin nor Capture may
   * append its frame or replace the first run's parking goal/STOP trigger. */
  on();sample.tick_ms=130;sample.body_pitch_rad=.9f;
  g_balance.position_locked=1;g_balance.parking_target_m=.75f;
  LqrStartupRecord_Begin(&sample);LqrStartupRecord_Capture(&sample);
  assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.count==3);
  assert(g_lqr_record.reason==LQR_RECORD_STOP && g_lqr_record.trigger_index==1);
  assert(g_lqr_record_data.header.reason==LQR_RECORD_STOP &&
         g_lqr_record_data.header.trigger_index==1 &&
         g_lqr_record_data.header.end_tick_ms==125 && g_lqr_record_data.header.count==3);
  assert(!memcmp(retained,(const void*)g_lqr_record_data.rows,sizeof(retained)));
  assert(g_lqr_record.save_request && g_robot_control.enable &&
         g_dual_hold.enable && g_balance.enable); /* Recording never stops control. */
  now=130;LqrStartupRecord_Poll();assert(!erases && !programs);
  sample.tick_ms=170;LqrStartupRecord_Capture(&sample);assert(g_lqr_record.count==3);
  off();now=200;LqrStartupRecord_Poll();now+=100;LqrStartupRecord_Poll();
  LqrRecordHeader kept;assert(g_lqr_record.state==LQR_RECORD_SAVED && erases==1);
  assert(LqrRecordStorage_Load(&kept) && kept.reason==LQR_RECORD_STOP &&
         kept.count==3 && kept.trigger_index==1 && kept.end_tick_ms==125);
  puts("PASS: restart during STOP tail freezes first run before new frame, retains tail/goal/reason, never changes control or writes active Flash");
  return 0;
 }
 if(argc>1 && (strcmp(argv[1],"v3")==0 || strcmp(argv[1],"v4")==0)) {
  LqrRecordHeader *old=(void*)LQR_RECORD_FLASH_ADDRESS;
  unsigned version=strcmp(argv[1],"v3")==0?3:4;
  unsigned width=version==3?288:320,capacity=version==3?896:800;
  memset(old,0,384+capacity*width);
  old->magic=LQR_RECORD_MAGIC;old->version=version;old->build_id=2026100823;
  old->row_bytes=width;old->capacity=old->count=capacity;old->recorded_startup_ok=1;
  old->commit=LQR_RECORD_COMMIT;
  uint32_t crc=LqrStartupRecord_CRC32(UINT32_MAX,old,120);
  crc=LqrStartupRecord_CRC32(crc,(const uint8_t*)old+128,256+capacity*width);
  old->crc32=crc^UINT32_MAX;
  LqrRecordHeader loaded;assert(LqrRecordStorage_Load(&loaded) && loaded.count==capacity);
  LqrStartupRecord_Init();assert(g_lqr_record.state==LQR_RECORD_SAVED && !g_lqr_record.ram_valid);
  rearm();assert(!erases && old->count==capacity && LqrRecordStorage_Load(&loaded));
  puts("PASS: full-capacity V3 Flash retained and validated with smaller V4 RAM, explicit rearm without erase");return 0;
 }
 if(argc>1) {
  LqrRecordData *f=(LqrRecordData*)LQR_RECORD_FLASH_ADDRESS;
  unsigned failed=argv[1][0]=='f';unsigned rows=failed?1:2;
  memset(f,0,128+rows*160);
  f->header.magic=LQR_RECORD_MAGIC;f->header.version=strchr(argv[1],'2')?2:1;
  f->header.row_bytes=160;f->header.capacity=1536;
  f->header.count=rows;f->header.reason=LQR_RECORD_FAULT;f->header.generation=7;
  uint32_t *legacy=(void*)((uint8_t*)f+128);
  if(!failed) {legacy[1]=1;legacy[3]=1;}
  uint32_t crc=LqrStartupRecord_CRC32(UINT32_MAX,&f->header,120);
  crc=LqrStartupRecord_CRC32(crc,legacy,rows*160);
  f->header.crc32=crc^UINT32_MAX;f->header.commit=LQR_RECORD_COMMIT;
  LqrStartupRecord_Init();assert(g_lqr_record.flash_present && !g_lqr_record.ram_valid);
  assert(g_lqr_record_data.header.generation==7);
  assert(g_lqr_record.state==(failed?LQR_RECORD_READY:LQR_RECORD_SAVED));
  assert(g_lqr_record.count==(failed?0:2));
  puts("PASS: v1 rejected-start recovery and preservation of a real started trial");return 0;
 }
 for(unsigned i=0;i<8;++i) {
  g_motor_foc.calibration[i].kt_nm_per_a=.23f;g_motor_foc.calibration[i].current_limit_a=2;
  g_motor_foc.calibration[i].direction=i<2?-1:1;g_motor_foc.calibration[i].minimum_iq_slew_a_per_s=1;
  g_motor_foc.node[i].fast_valid=g_motor_foc.node[i].fast_seen=1;
  g_motor_foc.node[i].requested_iq_a=.1f*(i+1);g_motor_foc.node[i].applied_iq_a=.05f*(i+1);
 }
 LqrStartupRecord_Init();assert(g_lqr_record.state==LQR_RECORD_READY);
 #if ROBOT_NATIVE_LQR
 assert(LQR_RECORD_BUILD_ID==2026111602U);
 #endif
 assert(g_lqr_record.tail_ms==LQR_RECORD_TAIL_MS && LQR_RECORD_TAIL_MS==2000U);
 g_lqr_record.tail_ms=0; /* Immediate-freeze fixture for the existing decoder golden data. */
 WheelLegDebug s={0};s.tick_ms=100;s.period_ms=5;s.imu_valid=1;
 s.leg_valid[0]=s.leg_valid[1]=s.state_valid[0]=s.state_valid[1]=1;
 s.wheel_valid[0]=s.wheel_valid[1]=1;s.leg[0].length=.07f;s.leg[1].length=.08f;
 s.body_pitch_rad=.05f;s.body_pitch_rate_rad_s=.25f;
 for(unsigned i=0;i<4;i++) {
  s.joint_feedback_valid[i]=1;s.joint_angle_rad[i]=1.1f*(i+1);
  s.joint_feedback_age_ms[i]=i+1;
  g_dual_hold.direction[i]=i<2?-1:1;
  g_dual_hold.joint_torque_nm[i]=(i%2?-1:1)*(.04f+.01f*i);
 }
 s.joint_shaft_speed_rad_s[0]=-1;s.joint_shaft_speed_rad_s[1]=2;
 s.joint_shaft_speed_rad_s[2]=3;s.joint_shaft_speed_rad_s[3]=-6;
 for(unsigned i=0;i<2;i++) {
  s.length_speed_m_s[i]=.01f*(i+1);s.angle_speed_rad_s[i]=.1f*(i+1);
  s.length_speed_position_valid[i]=1;s.length_speed_from_position_m_s[i]=.009f+.012f*i;
  g_dual_hold.angle_hold_torque_nm[i]=.01f*(i+1);
  g_dual_hold.sync_torque_nm[i]=i?.04f:-.03f;
  g_dual_hold.lqr_pitch_torque_nm[i]=.12f;
  g_dual_hold.pitch_torque_nm[i]=i?.18f:.10f;
  g_dual_hold.angle_reference_rad[i]=1.57f;
 }
 g_dual_hold.sync_ratio=.5f;g_dual_hold.sync_limit_nm=.3f;g_dual_hold.feedforward_n=2.5f;
 g_dual_hold.speed_kp=LEG_DEFAULT_SPEED_KP;g_dual_hold.speed_ki=LEG_DEFAULT_SPEED_KI;
 g_balance.leg_hold_ratio=BALANCE_DEFAULT_LEG_HOLD_RATIO;
 g_dual_hold.sync_priority_active=1;
 s.state[0][2]=.1f;s.state[1][2]=.3f;s.state[0][3]=s.state[1][3]=.5f;
 g_balance.wheel_scale=.7f;g_balance.leg_scale=.2f;g_balance.reference[2]=.123f;
 g_balance.wheel_rate_scale=.5f;
 g_balance.torque_nm[1]=BALANCE_LEG_LQR_TORQUE_LIMIT_NM;g_joint_output.saturated=1;
 g_wheel_output.applied_torque_nm=-.03f;g_wheel_output.voltage_mv[0]=2187;
 g_wheel_output.applied_per_side_nm[0]=-.029f;g_wheel_output.applied_per_side_nm[1]=-.031f;
 g_wheel_output.requested_per_side_nm[0]=-.029f;g_wheel_output.requested_per_side_nm[1]=-.031f;
 g_wheel_output.applied_differential_nm=.001f;
 g_robot_motion.command.forward_m_s=.05f;g_robot_motion.command.turn_rad_s=.2f;
 g_robot_motion.command.status=ROBOT_MOTION_TARGET_READY;g_robot_motion.neutral_ready=1;
 g_robot_motion.translating=1;g_robot_motion.yaw_active=1;g_robot_motion.yaw_enable=1;
 g_robot_motion.yaw_feedback_direction=-1;
 g_wheel_output.voltage_mv[1]=-2079;g_robot_control.startup_phase=ROBOT_STARTUP_RAMP;
 on();LqrStartupRecord_Begin(&s);LqrStartupRecord_Capture(&s);
 assert(g_lqr_record.count==1 && g_lqr_record_data.rows[0].wheel_command_nm==-.03f);
 assert(fabsf(g_lqr_record_data.rows[0].state[2]-.2f)<1e-6f);
 s.tick_ms+=5;LqrStartupRecord_Capture(&s);assert(g_lqr_record.count==1);
 s.tick_ms+=15;LqrStartupRecord_Capture(&s);assert(g_lqr_record.count==2);
 s.tick_ms+=5;g_robot_control.last_fault.count=1;g_robot_control.last_fault.reason=ROBOT_FAULT_BODY_PITCH;
 s.joint_feedback_valid[2]=0;s.joint_feedback_age_ms[2]=100000;
 s.leg_valid[1]=0;s.length_speed_position_valid[1]=0;
 off();LqrStartupRecord_Capture(&s);
 assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.reason==LQR_RECORD_FAULT && g_lqr_record.count==3);
 assert(!(g_lqr_record_data.rows[2].flags&1) && g_lqr_record_data.rows[2].fault_count==1);
 now=s.tick_ms;LqrStartupRecord_Poll();assert(!erases);
 now+=99;LqrStartupRecord_Poll();assert(!erases);
 now+=1;g_motor_foc.node[0].flags|=1U;LqrStartupRecord_Poll();assert(!erases);
 g_motor_foc.node[0].flags&=~1U;transport_stopped=0;LqrStartupRecord_Poll();assert(!erases);
 transport_stopped=1;LqrStartupRecord_Poll();now+=100;LqrStartupRecord_Poll();assert(erases==1 && !unlocked && g_lqr_record.state==LQR_RECORD_SAVED);
 assert(last_address==LQR_RECORD_FLASH_ADDRESS+96);
 LqrRecordHeader h;assert(LqrRecordStorage_Load(&h) && h.count==3 && h.reason==LQR_RECORD_FAULT);
 FILE *saved=fopen("record.bin","wb");assert(saved);
 size_t record_bytes=offsetof(LqrRecordData,rows)+h.count*sizeof(LqrRecordRow);
 assert(fwrite((void*)LQR_RECORD_FLASH_ADDRESS,1,record_bytes,saved)==record_bytes);
 fclose(saved);
 uint32_t first_crc=h.crc32;
 on();s.tick_ms+=100;LqrStartupRecord_Begin(&s);LqrStartupRecord_Capture(&s);
 assert(g_lqr_record.count==3); /* Frozen/saved records cannot be overwritten by a new start. */
 g_lqr_record.rearm_request=1;LqrStartupRecord_Poll();assert(g_lqr_record.state==LQR_RECORD_SAVED);
 /* Actual default: one immediate fault frame, then 50 Hz observation for
  * two seconds, without restarting outputs or saving the unfinished tail. */
 rearm();g_lqr_record.tail_ms=LQR_RECORD_TAIL_MS;on();s.tick_ms=1000;
 s.leg_valid[1]=1;s.body_pitch_rad=.1f;s.state[0][3]=s.state[1][3]=.4f;
 g_balance.position_locked=1;g_balance.parking_target_m=.75f;g_balance.reference[2]=.10f;
 LqrStartupRecord_Begin(&s);LqrStartupRecord_Capture(&s);
 assert(g_lqr_record_data.rows[0].reference[2]==.75f); /* True goal, not clipped input. */
 off();g_robot_control.last_fault.count++;
 memset((void*)&g_wheel_output,0,sizeof(g_wheel_output));
 memset((void*)&g_joint_output,0,sizeof(g_joint_output));
 memset((void*)&g_dual_hold,0,sizeof(g_dual_hold));
 g_balance.position_locked=0;g_balance.reference[2]=g_balance.parking_target_m=0;
 s.tick_ms=1005;LqrStartupRecord_Capture(&s);
 assert(g_lqr_record.state==LQR_RECORD_RUNNING && g_lqr_record.count==2 &&
        g_lqr_record.trigger_index==1 && !g_lqr_record.save_request);
 uint32_t tail_erases=erases;
 for(unsigned i=1;i<=399;i++) {
  s.tick_ms=1005+5*i;s.body_pitch_rad=.1f+.0002f*i;
  s.joint_angle_rad[0]=1.1f+.001f*i;s.joint_shaft_speed_rad_s[0]=.2f;
  s.state[0][3]=s.state[1][3]=.4f-.0005f*i;
  LqrStartupRecord_Capture(&s);now=s.tick_ms;LqrStartupRecord_Poll();
  assert(g_lqr_record.state==LQR_RECORD_RUNNING && erases==tail_erases);
  assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
 }
 assert(g_lqr_record.count==101); /* No 200 Hz stopped-frame flood. */
 s.tick_ms=3005;LqrStartupRecord_Capture(&s);
 assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.reason==LQR_RECORD_FAULT &&
        g_lqr_record.count==102 && g_lqr_record.trigger_index==1);
 const LqrRecordRow *tail=(const LqrRecordRow*)&g_lqr_record_data.rows[101];
 assert(!(tail->flags&1U) && tail->body_pitch_rad>.17f && tail->state[3]>.19f);
 assert(tail->joint_angle_rad[0]>1.49f && tail->joint_shaft_speed_rad_s[0]==.2f);
 assert(tail->reference[2]==.75f); /* Goal survives stop/reference clear. */
 for(unsigned i=0;i<4;i++) assert(tail->joint_voltage_mv[i]==0 && tail->joint_torque_command_nm[i]==0);
 now=s.tick_ms;LqrStartupRecord_Poll();
 assert(g_lqr_record.state==LQR_RECORD_SAVED && erases==tail_erases+1);
 assert(LqrRecordStorage_Load(&h) && h.count==102 && h.end_tick_ms-h.start_tick_ms==2005);
 saved=fopen("tail_record.bin","wb");assert(saved);
 record_bytes=offsetof(LqrRecordData,rows)+h.count*sizeof(LqrRecordRow);
 assert(fwrite((void*)LQR_RECORD_FLASH_ADDRESS,1,record_bytes,saved)==record_bytes);fclose(saved);
 /* Zero-tail mode still works; subsequent original fixtures stay isolated. */
 g_lqr_record.tail_ms=200;
 rearm();on();s.tick_ms=1000;s.body_pitch_rad=.8f;LqrStartupRecord_Begin(&s);
 for(unsigned i=0;i<13;i++){s.tick_ms=1000+20*i;LqrStartupRecord_Capture(&s);}
 assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.reason==LQR_RECORD_FALL);
 assert(g_lqr_record.trigger_index==2 && g_robot_control.enable==1);
 now=2000;LqrStartupRecord_Poll();assert(erases==tail_erases+1); /* Never erase while balance is active. */
 off();LqrStartupRecord_Poll();now+=100;LqrStartupRecord_Poll();
 assert(erases==tail_erases+2 && LqrRecordStorage_Load(&h) && h.reason==LQR_RECORD_FALL && h.crc32!=first_crc);
 ((uint8_t*)LQR_RECORD_FLASH_ADDRESS)[128+10]^=1;
 assert(!LqrRecordStorage_Load(&h)); /* CRC protects configuration as well as rows. */
 ((uint8_t*)LQR_RECORD_FLASH_ADDRESS)[128+10]^=1;
 ((uint8_t*)LQR_RECORD_FLASH_ADDRESS)[offsetof(LqrRecordData,rows)+10]^=1;
 assert(!LqrRecordStorage_Load(&h)); /* CRC rejects corrupted payload. */
 rearm();on();s.body_pitch_rad=0;s.tick_ms=3000;LqrStartupRecord_Begin(&s);
 for(unsigned i=0;i<LQR_RECORD_ROWS+10;i++){s.tick_ms=3000+20*i;LqrStartupRecord_Capture(&s);}
 assert(g_lqr_record.count==LQR_RECORD_ROWS && g_lqr_record.reason==LQR_RECORD_FULL);
 assert(g_lqr_record_data.rows[0].tick_ms==3000); /* No ring overwrite of launch data. */
 off();now=40000;LqrStartupRecord_Poll();now+=100;
 fail_program=programs+LQR_RECORD_ROWS*(sizeof(LqrRecordRow)/32)+sizeof(LqrRecordConfiguration)/32+4;LqrStartupRecord_Poll(); /* Interrupt final commit. */
 assert(g_lqr_record.state==LQR_RECORD_SAVE_FAILED && g_lqr_record.ram_valid && !unlocked);
 assert(!LqrRecordStorage_Load(&h)); /* A failed/incomplete save has no valid commit. */
 fail_program=0;g_lqr_record.save_request=1;LqrStartupRecord_Poll();
 assert(g_lqr_record.state==LQR_RECORD_SAVED && LqrRecordStorage_Load(&h) && h.count==LQR_RECORD_ROWS);
 rearm();on();s.tick_ms=UINT32_MAX-10;LqrStartupRecord_Begin(&s);LqrStartupRecord_Capture(&s);
 s.tick_ms=9;LqrStartupRecord_Capture(&s);assert(g_lqr_record.count==2);
 off();s.tick_ms=14;LqrStartupRecord_Capture(&s);
 assert(g_lqr_record.state==LQR_RECORD_RUNNING && g_lqr_record.trigger_index==2);
 s.tick_ms=214;LqrStartupRecord_Capture(&s);assert(g_lqr_record.reason==LQR_RECORD_STOP);
 /* Preserve a first-cycle rejection and mark invalid/stale current as NaN. */
 rearm();g_lqr_record.tail_ms=0;on();g_robot_control.status=0;
 s.tick_ms=1000;LqrStartupRecord_Begin(&s);off();g_robot_control.last_fault.count++;
 g_robot_control.last_fault.reason=ROBOT_FAULT_FEEDBACK;
 s.feedback_detail=WHEEL_LEG_FEEDBACK_SKEW;s.feedback_ms=7;
 g_motor_foc.node[0].fast_valid=0;refresh_nodes=0;now+=11;
 LqrStartupRecord_Capture(&s);
 assert(g_lqr_record.state==LQR_RECORD_FROZEN && g_lqr_record.count==1);
 assert(!g_lqr_record_data.header.recorded_startup_ok);
 assert(isnan(g_lqr_record_data.rows[0].foc_actual_iq_a[0]) && isnan(g_lqr_record_data.rows[0].foc_actual_iq_a[1]));
 assert(g_lqr_record_data.rows[0].feedback_detail==WHEEL_LEG_FEEDBACK_SKEW && g_lqr_record_data.rows[0].feedback_ms==7);
 refresh_nodes=1;g_motor_foc.node[0].fast_valid=1;now+=200;LqrStartupRecord_Poll();now+=100;LqrStartupRecord_Poll();
 assert(LqrRecordStorage_Load(&h) && h.version==5 && h.count==1 && !h.recorded_startup_ok);
 puts("PASS: first-cycle rejection retained, measured-current freshness, inactive-driver Flash guard");
 puts("PASS: launch/stop/fault/fall/tail/capacity/wrap, no active Flash writes, preserved trials, CRC/commit and failed-save retry");
}
'''
with tempfile.TemporaryDirectory(prefix='lqr_record_') as directory:
    tmp = Path(directory)
    (tmp/'mock.h').write_text(mock)
    (tmp/'main.h').write_text('#include "mock.h"\n')
    (tmp/'FreeRTOS.h').write_text('#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n')
    (tmp/'task.h').write_text('#include "FreeRTOS.h"\n')
    (tmp/'test.c').write_text(harness)
    exe = tmp/'test.exe'
    command=['gcc','-DMYLEGWHEEL_A1_CURRENT=1','-DLQR_RECORD_PERIOD_MS=20','-std=c99','-O2','-Wall','-Wextra','-Werror',
                    '-I',str(tmp),'-I',str(root/'App/Tasks/Inc'),
                    '-I',str(root/'App/Services/Inc'),'-I',str(root/'Control/WheelLeg/Inc'),
                    '-I',str(root/'App/Protocol/Inc'),
                    str(tmp/'test.c'),str(root/'App/Services/Src/lqr_startup_record.c'),
                    str(root/'App/Services/Src/lqr_record_flash.c'),'-o',str(exe)]
    subprocess.run(command,check=True)
    production=tmp/'production.exe'
    production_command=[arg for arg in command if arg!='-DLQR_RECORD_PERIOD_MS=20']
    production_command[-1]=str(production)
    subprocess.run(production_command,check=True)
    subprocess.run([str(production),'cadence'],check=True,cwd=tmp)
    subprocess.run([str(production),'restart'],check=True,cwd=tmp)
    subprocess.run([str(exe),'restart'],check=True,cwd=tmp)
    subprocess.run([str(exe)],check=True,cwd=tmp)
    subprocess.run([str(exe),'failed'],check=True,cwd=tmp)
    subprocess.run([str(exe),'started'],check=True,cwd=tmp)
    subprocess.run([str(exe),'started2'],check=True,cwd=tmp)
    subprocess.run([str(exe),'v3'],check=True,cwd=tmp)
    subprocess.run([str(exe),'v4'],check=True,cwd=tmp)
    raw = (tmp/'record.bin').read_bytes()
    header = header_decode(raw[:128])
    assert header['version']==5 and header['configuration']['wheel_rate_scale']==.5
    assert header['recorded_startup_ok']
    config=configuration_decode(raw[128:512]);payload=raw[512:]
    assert config['joint_encoder_ratio']==[1,1,1,1.5]
    assert config['joint_motor_id']==[1,2,5,6]
    assert config['foc_node_id']==[1,2,3,5,6,7] and config['foc_current_limit_a']==[2]*6

    assert abs(config['shared_leg_Tp_limit_nm']-(0 if header['build_id'] in (2026110802,2026110803,2026110804,2026110805,2026110806,2026110807,2026110808,2026110809,2026110810,2026110811,2026110812,2026110813,2026110814,2026110815,2026110817,2026110818,2026110819,2026110820,2026110821,2026110822,2026110823,2026111201,2026111202,2026111203,2026111204,2026111205,2026111206,2026111207,2026111208,2026111211,2026111212,2026111213,2026111214,2026111215,2026111216,2026111217,2026111218,2026111219,2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111602) else .12))<1e-7
    assert record_crc(raw[:128],payload,raw[128:512]) == header['crc32']
    rows = rows_decode(payload,header['count'],header['start_tick_ms'],5,config)
    assert abs(rows[0]['left_wheel_torque_command_nm']+.029)<1e-7
    assert rows[0]['foc_actual_iq_a']==[.25,.5,.75,1.25,1.5,1.75]
    assert rows[0]['foc_mode']==[3]*6 and rows[-1]['foc_actual_iq_a']==[0]*6
    assert rows[0]['foc_current_limit_a']==[2]*6

    assert abs(rows[0]['right_wheel_torque_command_nm']+.031)<1e-7
    assert abs(rows[0]['forward_target_m_s']-.05)<1e-7 and rows[0]['motion_ready']
    assert rows[0]['yaw_feedback_direction']==-1 and rows[0]['neutral_ready']
    assert rows[0]['wheel_voltage_mv']==[2187,-2079]
    assert rows[0]['leg_length_m'][1]>.0799 and rows[-1]['fault_reason']==10
    assert rows[-1]['fault_reason_text']=='excessive body pitch'
    assert not rows[0]['new_fault_event'] and rows[-1]['new_fault_event']
    assert rows[0]['leg_Tp_saturated']==(header['build_id'] not in (2026110802,2026110803,2026110804,2026110805,2026110806,2026110807,2026110808,2026110809,2026110810,2026110811,2026110812,2026110813,2026110814,2026110815,2026110817,2026110818,2026110819,2026110820,2026110821,2026110822,2026110823,2026111201,2026111202,2026111203,2026111204,2026111205,2026111206,2026111207,2026111208,2026111211,2026111212,2026111213,2026111214,2026111215,2026111216,2026111217,2026111218,2026111219,2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111602))
    assert rows[0]['joint_output_saturated']
    assert rows[0]['sync_priority_limited'] and not rows[-1]['sync_priority_limited']
    assert rows[0]['joint_shaft_speed_rad_s']==[-1,2,3,-6]
    assert rows[0]['joint_model_speed_rad_s']==[1,-2,3,-4]
    assert rows[0]['joint_feedback_age_ms']==[1,2,3,4]
    assert rows[0]['control_period_ms']==5 and abs(rows[0]['support_feedforward_n']-2.5)<1e-7
    assert abs(rows[0]['total_pitch_nm'][0]-.10)<1e-7
    assert not rows[-1]['joint_feedback_valid'][2] and rows[-1]['joint_feedback_age_ms'][2]==65535
    assert rows[-1]['joint_angle_rad'][2]!=rows[-1]['joint_angle_rad'][2] # Invalid -> NaN.
    assert not rows[-1]['length_position_speed_valid'][1]
    export(header,payload,tmp/'record.json','emulated Flash',raw[128:512])
    report=json.loads((tmp/'record.json').read_text(encoding='utf-8'))
    if header['build_id'] in (2026111217,2026111218,2026111219,2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111602):
        recorded=report['header']
        assert recorded['control_mode']=='native LQR'
        assert recorded['configuration']['leg_speed_kp']==30
        assert recorded['configuration']['leg_speed_ki']==80
        assert recorded['configuration']['leg_hold_ratio']==0
        assert recorded['native_lqr_design']['total_mass_kg']==1.6
        assert recorded['native_lqr_design']['Q']==([800,15,200,80,200,160] if header['build_id']==2026111220 else [800,15,200,40,200,80])
        assert recorded['native_lqr_design']['R']==[100000,10000]
        height=recorded['height_control_build_defaults']
        assert height['integral_limit_n']==4.5 and height['integral_rate_n_s']==3
        assert height['support_feedforward_ready_error_n']==.05
        assert 'support-feedforward-ready gate' in recorded['native_lqr_semantics']
        assert 'no run-duration cutoff' in recorded['native_lqr_semantics']
        if header['build_id'] in (2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111602):
            assert 'local/hard/remote current caps and10A/s current slew retained' in recorded['native_lqr_semantics']
            assert 'without clipping the position feedback' in recorded['position_reference_semantics']
            assert '+/-0.30m' not in recorded['position_reference_semantics']
        else:
            assert 'Motor2A caps and10A/s current slew retained' in recorded['native_lqr_semantics']
            assert '+/-0.30m' in recorded['position_reference_semantics']
        assert '+/-0.05m' in recorded['position_reference_semantics']
        assert 'theta_eq=' in recorded['equilibrium_reference_semantics']
        assert 'Tp_ff=' in recorded['equilibrium_reference_semantics']
        assert 'once-captured startup theta' in recorded['equilibrium_reference_semantics']
        # Build-specific interpretation must not overwrite historical1216.
        historical=header_decode(raw[:128]);historical['build_id']=2026111216
        export(historical,payload,tmp/'historical1216.json','historical semantic fixture',raw[128:512])
        historical=json.loads((tmp/'historical1216.json').read_text(encoding='utf-8'))['header']
        assert '+/-5cm' in historical['position_reference_semantics']
        assert 'overlength support subtraction defaults OFF' in historical['native_lqr_semantics']
        assert 'native_lqr_design' not in historical and 'height_control_build_defaults' not in historical
        if header['build_id'] in (2026111220,2026111221,2026111222,2026111423,2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111602):
            old=header_decode(raw[:128]);old['build_id']=2026111219
            export(old,payload,tmp/'historical1219.json','historical1219 semantic fixture',raw[128:512])
            old=json.loads((tmp/'historical1219.json').read_text(encoding='utf-8'))['header']
            assert old['native_lqr_design']['Q']==[800,15,200,40,200,80]
            assert 'current_command_schedule_semantics' in old
            assert 'parking_damping_candidate' not in old
            if header['build_id']==2026111220:
                assert 'parking_damping_candidate' in recorded
            else:
                assert 'parking_damping_candidate' not in recorded
                assert 'fast_speed_guard_semantics' in recorded
                assert 'output_failure_snapshot_semantics' in recorded
                assert 'imu_angle_semantics' in recorded
                if header['build_id']==2026111222:
                    assert recorded['gravity_bias_semantics']['application_default']==0
                    assert recorded['gravity_bias_semantics']['shadow_default']==1
                    assert 'b_applied' in recorded['gravity_bias_semantics']['equilibrium']
                if header['build_id']==2026111423:
                    bias=recorded['gravity_bias_semantics']
                    assert bias['application_default']==1 and bias['shadow_default']==1
                    assert bias['manual_trim_default_deg']==-4.0
                    assert bias['adaptive_phi_limit_deg']==2.0
                    assert bias['adaptive_phi_rate_deg_s']==.25
                    assert bias['learning_dwell_ms']==300
                    assert bias['application_dwell_ms']==1500 and bias['application_min_samples']==200
                    assert bias['off_clears_applied_correction'] and not bias['off_disables_observer']
                    assert 'preserves the observer application enable' in bias['limits']
                    assert 'Original1222 gates retained' in bias['learning']
                    assert 'matching128-byte balance tail ring' in bias['recording']
                    assert 'b_applied' in recorded['equilibrium_reference_semantics']
                    assert 'parking integral OFF' in recorded['parking_revision']
                    assert 'recording_boundary_semantics' in recorded
                    assert 'current_command_schedule_semantics' in recorded
                    legacy=header_decode(raw[:128]);legacy['build_id']=2026111222
                    export(legacy,payload,tmp/'historical1222.json','historical1222 semantic fixture',raw[128:512])
                    legacy=json.loads((tmp/'historical1222.json').read_text(encoding='utf-8'))['header']
                    old_bias=legacy['gravity_bias_semantics']
                    assert old_bias['application_default']==0 and old_bias['shadow_default']==1
                    assert old_bias['limits']=='0.25deg/s and2deg maximum adaptive phi; manual0.5deg trim unchanged. OFF clears applied correction and dwell.'
                    assert 'active correction OFF' in legacy['parking_revision']
                    assert 'manual_trim_default_deg' not in old_bias
                    assert 'off_disables_observer' not in old_bias
                if header['build_id'] in (2026111424,2026111525,2026111526,2026111527,2026111528,2026111529,2026111530,2026111531,2026111533,2026111602):
                    servo=recorded['normal_balance_servo_design']
                    assert servo['Q']==[800,15,200,40,8000,80,100]
                    assert servo['R']==[100000,10000] and servo['sample_s']==.005
                    assert 'without a30cm/60cm P-input cap' in servo['parking_position_error']
                    assert 'below0.04m/s for100ms' in servo['governor']
                    assert servo['gravity_application_default']==0
                    assert 'measured speed' in servo['governor']
                    assert 'z_k' in servo['integral'] and 'z_next' in servo['integral']
                    assert 'normal_balance_servo_design' not in old
                if header['build_id'] in (2026111528,2026111529,2026111530,2026111531,2026111533,2026111602):
                    assert recorded['current_limit_semantics']['fixed_remote_iq_limit_a']==3.0
                    assert 'PREPARE/RAMP' in recorded['trim_application_semantics']
                    schedule=recorded['normal_balance_servo_design']['parking_schedule']
                    assert schedule['transition_s']==.5
                    assert 'First accepted forward-neutral tick' in schedule['activation']
                    assert 'yaw request' in schedule['activation']
                    assert '100ms' in schedule['reference_continuity']
                    legacy=header_decode(raw[:128]);legacy['build_id']=2026111527
                    export(legacy,payload,tmp/'historical1527.json','historical1527 semantic fixture',raw[128:512])
                    legacy=json.loads((tmp/'historical1527.json').read_text(encoding='utf-8'))['header']
                    assert 'neither tracking nor braking' in legacy['normal_balance_servo_design']['parking_schedule']['activation']
                    assert legacy['current_limit_semantics']['fixed_remote_iq_limit_a']==3.0
                if header['build_id'] in (2026111529,2026111530,2026111531,2026111533,2026111602):
                    pairing=recorded['feedback_pairing_semantics']
                    assert recorded['normal_balance_servo_design']['parking_schedule']['R']==[100000,40000]
                    assert pairing['freshness_ms']==10 and pairing['max_same_leg_skew_ms']==6
                    assert 'real FAST receive history' in pairing['selection']
                    assert 'retimestamping' in pairing['timestamps']
                if header['build_id'] in (2026111530,2026111531,2026111533,2026111602):
                    diagnostic=recorded['feedback_failure_diagnostics']
                    assert diagnostic['ram_symbol']=='g_motor_foc_feedback_failure'
                    assert 'no later live-node reread' in diagnostic['source']
                    assert 'RAM only' in diagnostic['lifetime']
                    if header['build_id']==2026111530:
                        assert 'same1529 parking/driving gains' in diagnostic['control_change']
                    else:
                        assert recorded['native_dynamic_hip_gain']['default']==1.5
                previous=header_decode(raw[:128]);previous['build_id']=2026111220
                export(previous,payload,tmp/'historical1220.json','historical1220 semantic fixture',raw[128:512])
                previous=json.loads((tmp/'historical1220.json').read_text(encoding='utf-8'))['header']
                assert previous['native_lqr_design']['Q']==[800,15,200,80,200,160]
                assert 'parking_damping_candidate' in previous
                assert 'fast_speed_guard_semantics' not in previous
                assert 'output_failure_snapshot_semantics' not in previous
    tail_raw=(tmp/'tail_record.bin').read_bytes();tail_header=header_decode(tail_raw[:128])
    assert record_crc(tail_raw[:128],tail_raw[512:],tail_raw[128:512])==tail_header['crc32']
    export(tail_header,tail_raw[512:],tmp/'tail_record.json','emulated post-stop tail',tail_raw[128:512])
    tail_report=json.loads((tmp/'tail_record.json').read_text(encoding='utf-8'))
    assert tail_report['summary']['post_trigger_duration_ms']==2000
    assert tail_report['summary']['post_trigger_records']==100
    assert tail_report['summary']['inactive_records']==101
    assert tail_report['summary']['first_inactive_elapsed_ms']==5
    assert abs(tail_report['records'][-1]['position_error_m']+.55)<1e-6
    assert all(r['joint_voltage_mv']==[0,0,0,0] for r in tail_report['records'][1:])
    assert (tmp/'record.csv').read_text(encoding='utf-8-sig').splitlines()[1].startswith('0,2,1,')
    assert 'RR_model_speed_rad_s' in (tmp/'record.csv').read_text(encoding='utf-8-sig')
    assert 'sync_priority_limited' in (tmp/'record.csv').read_text(encoding='utf-8-sig')
    import csv
    csv_rows=list(csv.reader((tmp/'record.csv').open(encoding='utf-8-sig')))
    assert len(set(map(len,csv_rows)))==1 and 'ID7_actual_Iq_A' in csv_rows[0]

    legacy_payload=b''.join(payload[i*header['row_bytes']:i*header['row_bytes']+160] for i in range(header['count']))
    v3_payload=b''.join(payload[i*header['row_bytes']:i*header['row_bytes']+288] for i in range(header['count']))
    v3=bytearray(raw[:128]);struct.pack_into('<II',v3,4,3,2026100823)
    struct.pack_into('<II',v3,12,288,896)
    struct.pack_into('<I',v3,120,record_crc(v3,v3_payload,raw[128:384]))
    v3_header=header_decode(v3)
    v3_rows=rows_decode(v3_payload,v3_header['count'],v3_header['start_tick_ms'],3,config)
    assert abs(v3_rows[0]['left_wheel_torque_command_nm']+.03)<1e-7 and 'yaw_target_rad_s' not in v3_rows[0]
    export(v3_header,v3_payload,tmp/'legacy_v3.json','legacy V3 fixture',raw[128:384])
    for version in (1,2):
        legacy=bytearray(raw[:128]);struct.pack_into('<I',legacy,4,version)
        struct.pack_into('<II',legacy,12,160,1536)
        if version==1: legacy[112:120]=bytes(8)
        struct.pack_into('<I',legacy,120,record_crc(legacy,legacy_payload))
        old_header=header_decode(legacy)
        assert old_header['configuration']['wheel_rate_scale']==(1 if version==1 else .5)
        old_rows=rows_decode(legacy_payload,old_header['count'],old_header['start_tick_ms'],version)
        assert old_rows[0]['wheel_voltage_mv']==[2187,-2079] and 'joint_angle_rad' not in old_rows[0]
        export(old_header,legacy_payload,tmp/f'legacy_v{version}.json','legacy fixture')
    print('PASS: actual C record layout, Python CRC/decoder, signed commands and CSV export')
