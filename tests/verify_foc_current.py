"""Exercise the real A1 master and mixed output encoder against byte-level nodes."""
import ast
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
tree=ast.parse((root/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
runtime=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='runtime' for t in n.targets))
runtime=runtime.replace('#endif','int HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *,uint32_t);\n#endif')
runtime=runtime.replace('typedef struct {int unused;} FDCAN_HandleTypeDef;',
    'typedef int HAL_StatusTypeDef;\n#define HAL_ERROR 1\n'
    'typedef struct {uint32_t TXBRP,TXBCR;} FDCAN_GlobalTypeDef;\n'
    'typedef struct {FDCAN_GlobalTypeDef *Instance;} FDCAN_HandleTypeDef;')
runtime=runtime.replace('FDCAN_HandleTypeDef *,FDCAN_TxHeaderTypeDef *,uint8_t *',
                        'FDCAN_HandleTypeDef *,const FDCAN_TxHeaderTypeDef *,const uint8_t *')
runtime=runtime.replace('#endif','uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *);\n#endif')
runtime=runtime.replace('#endif','void test_interleave(void);\nvoid test_before_enter(void);\n#endif')
runtime=runtime.replace('(assert(critical),--critical)', '(assert(critical),--critical,test_interleave())')
runtime=runtime.replace('#define taskENTER_CRITICAL() (++critical)',
    '#define taskENTER_CRITICAL() (test_before_enter(),++critical)')
harness=r'''
#include "runtime.h"
#include "motor_foc.h"
#include "motor_command.h"
#include "bsp_fdcan_deadline.h"
#include "joint_calibration.h"
#include "leg_model.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
FDCAN_HandleTypeDef hfdcan1;
unsigned critical;
static uint32_t now=10,tx,iq_tx,stop_tx,aborts,fail_next;
static unsigned drop_ack[8],mode[8],flags[8],fault[8],last_ms[8],align_ms[8];
static unsigned drop_parameter_reply[8],wrong_write_echo;
static unsigned drop_write_ack[8],drop_write_parameter_reply[8],limit_write_tx[8];
static unsigned parameter_write_tx;
static unsigned align_stopped_ms[8],alignment_duration_ms=100,align_tx;
static unsigned saved_calibration[8],saved_zero_mrad[8],drop_calibration_reply[8],calibration_save_tx;
static int sensor_direction[8],saved_direction[8];
static unsigned node_sequence[8],node_have_seq[8],wraps[8];
static unsigned diagnostic_flags[8],stall_tx,latest_buffer=1,emit_fast=1;
static FDCAN_GlobalTypeDef hw;
static int iq_ma[8];
static uint8_t legacy[2][8];
static float remote_iq_limit=1,remote_voltage=5,remote_slew=1;
static float individual_slew[8];
static float individual_iq_limit[8];
static float individual_voltage[8];
static uint32_t transport_blocked;
static unsigned inject_enable,inject_pending,inject_kind,inject_slot,inject_before;
uint32_t CAN_CommunicationBlocked(void) {return transport_blocked;}
void WheelLeg_NotifyCanTransportFault(void) {}
uint32_t WheelLeg_CanCommunicationStopped(void) {return 1U;}
uint32_t HAL_GetTick(void) {return now;}
void Error_Handler(void) {assert(0);}
static void put16(uint8_t *p,int v) {p[0]=(uint8_t)(uint16_t)v;p[1]=(uint8_t)((uint16_t)v>>8);}
static void put32(uint8_t *p,unsigned v) {for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(v>>(8*i));}
static int signed16(const uint8_t *p) {unsigned v=p[0]|((unsigned)p[1]<<8);return v<32768?(int)v:(int)v-65536;}
void test_before_enter(void) {
 if(critical || !inject_before)return;
 inject_before=0;MotorFoc_InvalidateFromISR();
}
void test_interleave(void) {
 if(critical || !inject_pending)return;
 inject_pending=0;
 if(inject_kind==3) {inject_before=1;return;}
 if(inject_kind==2) {MotorFoc_InvalidateFromISR();return;}
 uint8_t fast[8]={0xa1,0x3f,0,0,0,0,0,0};
 put16(fast+2,1234);put16(fast+4,INT16_MAX);put16(fast+6,10);
 assert(MotorFoc_OnFrame(0x701+inject_slot,fast,now));
 put16(fast+4,1234);assert(MotorFoc_OnFrame(0x701+inject_slot,fast,now));
}
static void feed(unsigned i,int angle) {
 uint8_t d[8]={0xa1};
 if(mode[i]==2 && now-align_ms[i]>=alignment_duration_ms) {mode[i]=0;flags[i]|=32;flags[i]&=~1U;}
 if(mode[i]==2 && now-last_ms[i]>=100) {mode[i]=0;flags[i]&=~1U;}
 if(mode[i]==3 && now-last_ms[i]>=100) {mode[i]=0;flags[i]&=~1U;}
 d[1]=(uint8_t)mode[i];d[2]=(uint8_t)flags[i];put16(d+4,iq_ma[i]);put16(d+6,-1234);
 assert(MotorFoc_OnFrame(0x181+i,d,now));
 memset(d,0,8);d[0]=0xa1;put32(d+2,fault[i]);d[7]=(uint8_t)diagnostic_flags[i];
 assert(MotorFoc_OnFrame(0x281+i,d,now));
 if(angle) {memset(d,0,8);d[0]=0xa1;d[1]=2;put16(d+2,1000);assert(MotorFoc_OnFrame(0x581+i,d,now));}
 if(emit_fast) {memset(d,0,8);d[0]=0xa1;d[1]=(uint8_t)flags[i];put16(d+2,1000);put16(d+4,-1234);assert(MotorFoc_OnFrame(0x701+i,d,now));}
}
int HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h,uint32_t mask) {
 assert(h==&hfdcan1 && mask==0xffU);++aborts;return 0;
}
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(const FDCAN_HandleTypeDef *h) {assert(h==&hfdcan1);return latest_buffer;}
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,const FDCAN_TxHeaderTypeDef *header,const uint8_t *d) {
 assert(h==&hfdcan1);++tx;
 if(fail_next) {fail_next=0;return 1;}
 if(stall_tx) hw.TXBRP|=latest_buffer;else hw.TXBRP&=~latest_buffer;
 assert(header->IdType==FDCAN_STANDARD_ID && header->FDFormat==FDCAN_CLASSIC_CAN && header->DataLength==FDCAN_DLC_BYTES_8);
 if(header->Identifier==0x100 || header->Identifier==0x200) {memcpy(legacy[header->Identifier==0x200],d,8);return 0;}
 unsigned i=header->Identifier-0x201;
 assert(i<8 && (MOTOR_FOC_NODE_MASK&(1U<<i)) && d[0]==0xa1);
 if(node_have_seq[i] && d[1]!=0) {assert((uint8_t)(d[2]-node_sequence[i])==1U);if(d[2]<node_sequence[i])++wraps[i];}
 node_have_seq[i]=1;node_sequence[i]=d[2];
 uint8_t ack[8]={0xa1,d[1],d[2],0,0,0,0,0};
 if(d[1]==0) {++stop_tx;if(mode[i]==2)align_stopped_ms[i]=now;mode[i]=fault[i]?5:0;flags[i]&=~1U;diagnostic_flags[i]&=~8U;}
 else if(d[1]==1) {++iq_tx;mode[i]=3;flags[i]|=1;iq_ma[i]=signed16(d+4);last_ms[i]=now;assert(d[3]==1);}
 else if(d[1]==6) {assert(signed16(d+4)==1 || signed16(d+4)==-1);sensor_direction[i]=signed16(d+4);flags[i]=(flags[i]|16U)&~32U;}
 else if(d[1]==4) {++align_tx;if(mode[i]!=2)align_ms[i]=now;last_ms[i]=now;mode[i]=2;flags[i]|=1U;assert(d[3]==1);}
 else if(d[1]==5) fault[i]=0;
 else if(d[1]==0x15) {assert(d[3]==0 && !memcmp(d+4,"\0\0\0\0",4));}
 else if(d[1]==0x16) {
  assert(d[3]==0 && !memcmp(d+4,"CAL1",4));assert(mode[i]==0 && (flags[i]&0x3eU)==0x3eU);
  ++calibration_save_tx;saved_calibration[i]=1;saved_direction[i]=sensor_direction[i];saved_zero_mrad[i]=1000+10*i;
 }
 else if(d[1]==0x11) {
  assert(mode[i]==0 && !(flags[i]&1U) && (d[3]==3 || d[3]==5));
  for(unsigned j=0;j<8;++j)if(MOTOR_FOC_NODE_MASK&(1U<<j))
   assert(mode[j]!=2 && mode[j]!=3 && !(flags[j]&1U));
  ++parameter_write_tx;
  float requested;memcpy(&requested,d+4,4);
  if(d[3]==3) {
   assert(requested==MOTOR_FOC_FIXED_IQ_LIMIT_A && requested>=.01f && requested<=3.3f);
   individual_iq_limit[i]=requested;++limit_write_tx[i];
  } else {
   assert(requested>=.01f && requested<=10);remote_slew=requested;individual_slew[i]=requested;
  }
 }
 else assert(d[1]==7 || d[1]==0x10);
 put32(ack+4,fault[i]);
 if(!drop_ack[i] && !(d[1]==0x11 && drop_write_ack[i])) MotorFoc_OnFrame(0x381+i,ack,now);
 if((d[1]==0x10 || d[1]==0x11) && !drop_parameter_reply[i] &&
    !(d[1]==0x11 && drop_write_parameter_reply[i])) {
  uint8_t p[8]={0xa1,d[3],d[2],0,0,0,0,0};
  float value=d[3]==3?(individual_iq_limit[i]>0?individual_iq_limit[i]:remote_iq_limit):d[3]==4?(individual_voltage[i]>0?individual_voltage[i]:remote_voltage):
       MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S>0?individual_slew[i]:remote_slew;
  assert(d[3]>=3 && d[3]<=5);if(d[1]==0x11 && wrong_write_echo)value+=wrong_write_echo==2?.25f:1.0f;
  memcpy(p+4,&value,4);MotorFoc_OnFrame(0x481+i,p,now);
 }
 if((d[1]==0x15 || d[1]==0x16) && !drop_calibration_reply[i]) {
  uint8_t info[8]={0xa1,d[1],d[2],0,0,0,0,0};
  info[4]=(uint8_t)((saved_calibration[i]?1U:0U)|((flags[i]&32U)?8U:0U)|(d[1]==0x16?16U:0U));
  info[5]=(uint8_t)(int8_t)(saved_calibration[i]?saved_direction[i]:0);
  put16(info+6,saved_zero_mrad[i]);assert(MotorFoc_OnFrame(0x641+i,info,now));
  if(d[1]==0x16) {mode[i]=0;flags[i]&=~1U;diagnostic_flags[i]|=8U;}
 }
 if(inject_enable && d[1]==1U && i==inject_slot) {inject_enable=0;inject_pending=1;}
 return 0;
}
static void fresh_nodes(int angle) {for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i))feed(i,angle);}
static uint32_t cycle(const float *torque,unsigned mask) {now+=5;fresh_nodes(1);uint32_t good=MotorFoc_Service(torque,mask);assert(!critical);return good;}
static void idle_ready(void) {
 for(unsigned k=0;k<60;++k) cycle(NULL,0);
 for(unsigned i=0;i<8;++i) if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  assert(g_motor_foc.node[i].session_ready && g_motor_foc.node[i].limit_verified &&
         g_motor_foc.node[i].voltage_verified && g_motor_foc.node[i].slew_verified);
 }
}
static void calibrate(void) {
 for(unsigned i=0;i<8;++i) {
  g_motor_foc.calibration[i].kt_nm_per_a=i==2?.05f:.1f;
  g_motor_foc.calibration[i].calibration_confirmed=1;
  g_motor_foc.calibration[i].dynamic_response_confirmed=1;
  g_motor_foc.calibration[i].minimum_iq_slew_a_per_s=1;
 }
}
static void check_initialized_joint_configuration(void) {
 for(unsigned slot=0;slot<8;++slot) {
  const JointCalibration *joint=NULL;
  for(unsigned j=0;j<JOINT_CALIBRATION_COUNT;++j)
   if(g_joint_calibration[j].motor_id==slot+1U) joint=&g_joint_calibration[j];
  const volatile MotorFocCalibration *c=&g_motor_foc.calibration[slot];
  unsigned bit=1U<<slot;
  assert(c->current_limit_a==2.0f);
  unsigned configured=joint && (MOTOR_FOC_NODE_MASK&MOTOR_FOC_JOINT_NODE_MASK&bit);
  assert(c->angle_anchor_confirmed==(unsigned)(configured && (MOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK&bit)));
  assert(c->joint_timing_confirmed==(unsigned)(configured && (MOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK&bit)));
  assert(c->angle_anchor_rad==(configured?joint->encoder_reference_rad:0.0f));
  unsigned commissioned=(MOTOR_FOC_NODE_MASK&MOTOR_FOC_COMMISSIONED_RESPONSE_MASK&bit)!=0;
  assert(c->dynamic_response_confirmed==commissioned);
  assert(c->minimum_iq_slew_a_per_s==(commissioned?MOTOR_FOC_COMMISSIONED_MIN_IQ_SLEW_A_PER_S:0.0f));
  if(commissioned) assert(c->calibration_confirmed);
 }
}
static void check_startup_joint_observation(void) {
 const float tau=6.28318530718f;
 MotorFeedback observed[JOINT_CALIBRATION_COUNT];unsigned valid=0;
 for(unsigned j=0;j<JOINT_CALIBRATION_COUNT;++j) {
  const JointCalibration *cal=&g_joint_calibration[j];unsigned slot=cal->motor_id-1U,bit=1U<<slot;
  if(!(MOTOR_FOC_NODE_MASK&bit)) continue;
  feed(slot,1);
  float wrapped=fmodf(cal->encoder_reference_rad,tau);if(wrapped<0)wrapped+=tau;
  uint8_t fast[8]={0xa1,0x3e,0,0,0,0,0,0};
  put16(fast+2,(int)lroundf(wrapped*1000.0f));put16(fast+4,1234);
  assert(MotorFoc_OnFrame(0x701+slot,fast,now));
  unsigned confirmed=(MOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK&MOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK&bit)!=0;
  assert(MotorFoc_GetFeedback((uint8_t)cal->motor_id,&observed[j])==confirmed);
  assert(MotorFoc_TorqueLimitNm((uint8_t)cal->motor_id)==0);
  if(!confirmed) continue;
  assert(observed[j].joint_calibrated);
  assert(fabsf(observed[j].angle_rad-cal->encoder_reference_rad)<.0006f);
  assert(fabsf(observed[j].joint_angle_rad-1.57079632679f)<.0006f);
  valid|=1U<<j;
  /* Each confirmation remains independently necessary on valid fresh frames. */
  g_motor_foc.calibration[slot].angle_anchor_confirmed=0;
  MotorFeedback rejected;assert(!MotorFoc_GetFeedback((uint8_t)cal->motor_id,&rejected));
  g_motor_foc.calibration[slot].angle_anchor_confirmed=1;
  g_motor_foc.calibration[slot].joint_timing_confirmed=0;
  assert(!MotorFoc_GetFeedback((uint8_t)cal->motor_id,&rejected));
  g_motor_foc.calibration[slot].joint_timing_confirmed=1;
 }
 if(valid==15U) {
  for(unsigned side=0;side<2;++side) {
   LegModel leg;
   assert(leg_model(observed[2*side+1].joint_angle_rad,observed[2*side].joint_angle_rad,LEG_BRANCH_DOWN,&leg));
   assert(fabsf(leg.length-.150623f)<.0001f);
   assert(fabsf(leg.angle-1.57079632679f)<.001f);
  }
 }
}
static void check_dq_trace_current_session(void) {
 MotorFoc_Init();remote_iq_limit=3.3f;remote_voltage=5;remote_slew=1;
 emit_fast=1;stall_tx=fail_next=transport_blocked=0;hw=(FDCAN_GlobalTypeDef){0};
 for(unsigned i=0;i<8;++i) {
  mode[i]=fault[i]=diagnostic_flags[i]=drop_ack[i]=drop_parameter_reply[i]=0;
  flags[i]=0x3e;individual_voltage[i]=individual_slew[i]=0;
  g_motor_foc.calibration[i].angle_anchor_confirmed=1;
  g_motor_foc.calibration[i].joint_timing_confirmed=1;
 }
 idle_ready();calibrate();
 float torque[8]={0};torque[2]=.10f;torque[6]=.20f;
 assert(cycle(torque,0x44U));
 assert(g_motor_foc_dq_trace.count==1);
 assert(fabsf(g_motor_foc_dq_trace.rows[0].requested_iq_a[2]+2)<1e-6f &&
        fabsf(g_motor_foc_dq_trace.rows[0].requested_iq_a[5]-2)<1e-6f);
 MotorFoc_StopAll();unsigned sequence=g_motor_foc_dq_trace.sequence;
 idle_ready();assert(g_motor_foc_dq_trace.count==1 && g_motor_foc_dq_trace.sequence==sequence);
 /* Real asynchronous pages keep their own ages. Neither they nor the old
  * +/-2A cached commands are this new run's freshly queued torque request. */
 uint8_t page[8]={0xa1,0,0,0,0,0,0,0};
 put16(page+2,-125);put16(page+4,700);put16(page+6,1500);
 assert(MotorFoc_OnFrame(0x583,page,now-25U));
 page[1]=1;put16(page+2,2250);assert(MotorFoc_OnFrame(0x583,page,now-15U));
 torque[2]=.0125f;torque[6]=.035f;
 assert(cycle(torque,0x44U));
 assert(g_motor_foc_dq_trace.count==1 && g_motor_foc_dq_trace.sequence==sequence+1);
 const volatile MotorFocDqTraceRow *row=&g_motor_foc_dq_trace.rows[0];
 assert(row->tick_ms==now && row->active_mask==0x44U);
 assert(fabsf(row->requested_iq_a[2]+.25f)<1e-6f &&
        fabsf(row->requested_iq_a[5]-.35f)<1e-6f);
 assert(row->requested_iq_a[2]==g_motor_foc.node[2].requested_iq_a &&
        g_motor_foc_command_timing[2].count==1 && iq_ma[2]==-250 && iq_ma[6]==350);
 assert(isnan(row->requested_iq_a[0]) && isnan(row->requested_iq_a[3]));
 assert(row->page_age_ms[2][0]==30 && row->page_age_ms[2][1]==20);
 assert(fabsf(row->reference_iq_a[2]-.7f)<1e-6f &&
        fabsf(row->id_a[2]+.125f)<1e-6f && row->vq_v[2]==2.25f);
 MotorFocDqTraceDebug retained;memcpy(&retained,(const void*)&g_motor_foc_dq_trace,sizeof(retained));
 MotorFoc_StopAll();idle_ready();
 g_motor_foc.calibration[2].calibration_confirmed=0;
 assert(!cycle(torque,0x44U)); /* Rejected launches must not emit a trace. */
 assert(!memcmp(&retained,(const void*)&g_motor_foc_dq_trace,sizeof(retained)));
 idle_ready();calibrate();fail_next=1;
 assert(!cycle(torque,0x44U)); /* Failed IQ enqueue must not emit a trace. */
 assert(!memcmp(&retained,(const void*)&g_motor_foc_dq_trace,sizeof(retained)));
 idle_ready();calibrate();MotorFoc_InvalidateFromISR();
 assert(!cycle(torque,0x44U));
 assert(!memcmp(&retained,(const void*)&g_motor_foc_dq_trace,sizeof(retained)));
 idle_ready();calibrate();torque[2]=.02f;torque[6]=.04f;
 assert(cycle(torque,0x44U));row=&g_motor_foc_dq_trace.rows[0];
 assert(g_motor_foc_dq_trace.count==1 && g_motor_foc_dq_trace.sequence==sequence+2);
 assert(fabsf(row->requested_iq_a[2]+.4f)<1e-6f && fabsf(row->requested_iq_a[5]-.4f)<1e-6f);
 assert(g_motor_foc.node[2].epoch==g_motor_foc.transport_epoch);
 MotorFoc_StopAll();idle_ready();
 puts("PASS: DQ first row follows this run's successful IQ enqueue, excludes inactive/stale epoch caches, retains page ages and STOP ring, skips rejection/TX failure");
}
static void check_current_schedule_phase(void) {
 const unsigned offsets[2][6]={{0,6,10,16,20,26},{0,4,10,14,20,24}};
 float torque[8]={0};
 for(unsigned trial=0;trial<3;++trial) {
  MotorFoc_Init();remote_iq_limit=3.3f;remote_voltage=5;remote_slew=1;
  emit_fast=1;stall_tx=fail_next=transport_blocked=0;hw=(FDCAN_GlobalTypeDef){0};
  for(unsigned i=0;i<8;++i) {
   mode[i]=fault[i]=diagnostic_flags[i]=drop_ack[i]=drop_parameter_reply[i]=0;
   flags[i]=0x3e;individual_voltage[i]=individual_slew[i]=0;
   g_motor_foc.calibration[i].angle_anchor_confirmed=1;
   g_motor_foc.calibration[i].joint_timing_confirmed=1;
  }
  idle_ready();calibrate();
  uint32_t base=trial==2 ? UINT32_MAX-8U : now;
  unsigned phase=trial==1;
  for(unsigned step=0;step<6;++step) {
   now=base+offsets[phase][step];fresh_nodes(1);
   torque[2]=.01f+.001f*step;torque[6]=.02f+.001f*step;
   unsigned before=iq_tx;
   assert(MotorFoc_Service(torque,0x44U) && !critical);
   assert(iq_tx==before+2U); /* Every4/6ms control entry updates both nodes. */
   assert(g_motor_foc_command_timing[2].count==step+1U &&
          g_motor_foc_command_timing[6].count==step+1U);
   assert(abs(iq_ma[2]+(200+20*(int)step))<=1 &&
          abs(iq_ma[6]-(200+10*(int)step))<=1);
   assert(MotorFoc_Service(torque,0x44U) && iq_tx==before+2U);
   /* Same-tick calls cannot replay the newly sent command. */
  }
  assert(g_motor_foc_command_timing[2].max_gap_ms==6 &&
         g_motor_foc_command_timing[6].max_gap_ms==6);
  if(trial==0) {
   /* Last send26ms, next planned slot30ms. A50ms gap skips ten
    * expired slots once, preserving the next phase deadline80ms. */
   now=base+76U;fresh_nodes(1);unsigned before=iq_tx;
   assert(MotorFoc_Service(torque,0x44U) && iq_tx==before+2U);
   assert(g_motor_foc_command_timing[2].count==7 &&
          g_motor_foc_command_timing[2].max_gap_ms==50);
   for(unsigned offset=76;offset<=78;++offset) {
    now=base+offset;fresh_nodes(1);
    assert(MotorFoc_Service(torque,0x44U) && iq_tx==before+2U);
   }
   now=base+79U;fresh_nodes(1);
   assert(MotorFoc_Service(torque,0x44U) && iq_tx==before+4U);
   now=base+80U;fresh_nodes(1);
   assert(MotorFoc_Service(torque,0x44U) && iq_tx==before+4U);
  }
  MotorFoc_StopAll();idle_ready();
 }
 /* The1ms IQ scheduling tolerance never relaxes a pending command's ACK
  * gate or its40ms timeout, even if another node reaches its next slot. */
 drop_ack[2]=1;now+=5;fresh_nodes(1);
 assert(MotorFoc_Service(torque,0x44U));
 assert(g_motor_foc_command_timing[2].count==1 && g_motor_foc_command_timing[6].count==1);
 now+=4;fresh_nodes(1);assert(MotorFoc_Service(torque,0x44U));
 assert(g_motor_foc_command_timing[2].count==1 && g_motor_foc_command_timing[6].count==2);
 now+=MOTOR_FOC_ACK_TIMEOUT_MS-3U;fresh_nodes(1);unsigned before=iq_tx;
 assert(!MotorFoc_Service(torque,0x44U) && iq_tx==before && g_motor_foc.node[2].timeout_count);
 assert(g_motor_foc_command_timing[2].count==1 && mode[2]==0 && mode[6]==0);
 drop_ack[2]=0;idle_ready();
 puts("PASS: IQ absolute5ms phase updates every4/6ms task entry in either HAL phase and UINT32 wrap; one enqueue after long gaps, no same-tick replay");
}
static void failure_ready(float torque[8]) {
 inject_enable=inject_pending=inject_before=0;
 MotorFoc_Init();remote_iq_limit=3.3f;remote_voltage=5;remote_slew=1;
 emit_fast=1;stall_tx=fail_next=transport_blocked=0;hw=(FDCAN_GlobalTypeDef){0};
 for(unsigned i=0;i<8;++i) {
  mode[i]=fault[i]=diagnostic_flags[i]=drop_ack[i]=drop_parameter_reply[i]=0;
  flags[i]=0x3e;individual_voltage[i]=0;individual_slew[i]=1;torque[i]=0;
  g_motor_foc.calibration[i].angle_anchor_confirmed=1;
  g_motor_foc.calibration[i].joint_timing_confirmed=1;
 }
 idle_ready();calibrate();torque[2]=.01f;torque[6]=.02f;
 assert(cycle(torque,0x44U) && !g_motor_foc_failure.valid);
 assert(cycle(torque,0x44U) && !g_motor_foc_failure.valid); /* Fresh CURRENT status. */
}
static MotorFocFailureDebug failure_snapshot(void) {
 MotorFocFailureDebug result;memcpy(&result,(const void*)&g_motor_foc_failure,sizeof(result));return result;
}
static void check_speed_clipping_and_failure_context(void) {
 float torque[8];const unsigned slots[2]={2,6};const int endpoints[2]={INT16_MIN,INT16_MAX};
 for(unsigned side=0;side<2;++side) for(unsigned sign=0;sign<2;++sign) {
  failure_ready(torque);unsigned slot=slots[side],malformed=g_motor_foc.malformed;
  uint8_t fast[8]={0xa1,0x3f,0,0,0,0,0,0};
  put16(fast+2,1234);put16(fast+4,endpoints[sign]);put16(fast+6,321);
  assert(MotorFoc_OnFrame(0x701+slot,fast,now));MotorFeedback f;
  assert(g_motor_foc.malformed==malformed && g_motor_foc.node[slot].fast_valid &&
         g_motor_foc.node[slot].fast_speed_clipped && !MotorFoc_GetFeedback((uint8_t)(slot+1),&f));
  assert(fabsf(f.angle_rad-1.234f)<1e-5f &&
         fabsf(f.speed_rad_s-endpoints[sign]*.001f)<1e-5f &&
         fabsf(g_motor_foc.node[slot].fast_iq_a-.321f)<1e-6f);
  MotorFocFailureDebug clipped=failure_snapshot();
  assert(clipped.valid && clipped.reason==MOTOR_FOC_FAILURE_SPEED_CLIPPED &&
         clipped.node_id==slot+1 && clipped.fast_speed_clipped && clipped.fast_valid &&
         clipped.active_mask==0x44U && clipped.mode==3 && clipped.fast_age_ms==0);
  /* Even an interior sample before the next control cycle cannot erase the
   * unsafe event: telemetry recovers, but the active session must STOP. */
  put16(fast+4,sign?INT16_MAX-1:INT16_MIN+1);
  assert(MotorFoc_OnFrame(0x701+slot,fast,now));
  assert(!g_motor_foc.node[slot].fast_speed_clipped && MotorFoc_GetFeedback((uint8_t)(slot+1),&f));
  unsigned before=iq_tx;assert(!MotorFoc_Service(torque,0x44U) && iq_tx==before);
  assert(mode[2]==0 && mode[6]==0);
  idle_ready();assert(!memcmp(&clipped,(const void*)&g_motor_foc_failure,sizeof(clipped)));
  assert(cycle(torque,0x44U) && !g_motor_foc_failure.valid);MotorFoc_StopAll();idle_ready();
 }
 failure_ready(torque);unsigned failed=g_motor_foc.tx_failed;fail_next=1;
 assert(!cycle(torque,0x44U));MotorFocFailureDebug tx_failure=failure_snapshot();
 assert(tx_failure.valid && tx_failure.reason==MOTOR_FOC_FAILURE_TX_ENQUEUE &&
        tx_failure.node_id==3 && tx_failure.pending && tx_failure.opcode==1 &&
        tx_failure.session_ready && tx_failure.tx_failed==failed+1U && mode[2]==0 && mode[6]==0);
 idle_ready();g_motor_foc.calibration[2].calibration_confirmed=0;
 assert(!cycle(torque,0x44U)); /* Failed restarts preserve the first event. */
 assert(!memcmp(&tx_failure,(const void*)&g_motor_foc_failure,sizeof(tx_failure)));
 idle_ready();calibrate();assert(cycle(torque,0x44U) && !g_motor_foc_failure.valid);
 MotorFoc_StopAll();idle_ready();

 failure_ready(torque);drop_ack[2]=1;assert(cycle(torque,0x44U));
 now+=MOTOR_FOC_ACK_TIMEOUT_MS+1U;fresh_nodes(1);
 assert(!MotorFoc_Service(torque,0x44U));MotorFocFailureDebug timeout=failure_snapshot();
 assert(timeout.reason==MOTOR_FOC_FAILURE_ACK_TIMEOUT && timeout.node_id==3 &&
        timeout.pending && timeout.opcode==1 && !timeout.got_ack &&
        timeout.pending_age_ms==MOTOR_FOC_ACK_TIMEOUT_MS+1U && timeout.timeout_count==1);
 drop_ack[2]=0;idle_ready();

 failure_ready(torque);drop_ack[2]=1;assert(cycle(torque,0x44U));
 uint8_t ack[8]={0xa1,1,(uint8_t)node_sequence[2],7,0,0,0,0};
 assert(MotorFoc_OnFrame(0x383,ack,now));
 assert(!MotorFoc_Service(torque,0x44U));MotorFocFailureDebug rejected=failure_snapshot();
 assert(rejected.reason==MOTOR_FOC_FAILURE_ACK_REJECTED && rejected.node_id==3 &&
        rejected.pending && rejected.last_opcode==1 && rejected.last_result==7 && rejected.rejected_count==1);
 drop_ack[2]=0;idle_ready();

 failure_ready(torque);uint32_t epoch=g_motor_foc.transport_epoch;MotorFoc_InvalidateFromISR();
 assert(!MotorFoc_Service(torque,0x44U));MotorFocFailureDebug invalidated=failure_snapshot();
 assert(invalidated.reason==MOTOR_FOC_FAILURE_TRANSPORT_EPOCH && invalidated.node_id==3 &&
        invalidated.transport_epoch==epoch+1U && invalidated.node_epoch==epoch && invalidated.session_ready);
 idle_ready();

 failure_ready(torque);uint8_t diagnostic[8]={0xa1,0,0,0,0,0,0,8};
 assert(MotorFoc_OnFrame(0x283,diagnostic,now));
 assert(!MotorFoc_Service(torque,0x44U));MotorFocFailureDebug health=failure_snapshot();
 assert(health.reason==MOTOR_FOC_FAILURE_DRIVER_HEALTH && health.node_id==3 &&
        health.diagnostic_flags==8 && health.session_ready && !health.faults);
 diagnostic_flags[2]=0;idle_ready();

 failure_ready(torque);emit_fast=0;now+=MOTOR_FOC_FAST_TIMEOUT_MS+1U;fresh_nodes(0);
 assert(!MotorFoc_Service(torque,0x44U));MotorFocFailureDebug stale=failure_snapshot();
 assert(stale.reason==MOTOR_FOC_FAILURE_FEEDBACK && stale.node_id==3 &&
        stale.fast_age_ms==MOTOR_FOC_FAST_TIMEOUT_MS+1U && stale.status_age_ms==0 && stale.diagnostic_age_ms==0);
 emit_fast=1;idle_ready();
 /* Run RX at the real outermost lock exit of the first/last IQ enqueue.
  * A clipped->normal pair still latches inhibit; an ISR only changes epoch.
  * Both must fail THIS service, abort remaining IQ and STOP the whole group. */
 for(unsigned kind=1;kind<=2;++kind)for(unsigned node=0;node<2;++node) {
  failure_ready(torque);inject_kind=kind;inject_slot=slots[node];inject_enable=1;
  unsigned before=iq_tx;assert(!cycle(torque,0x44U));
  assert(iq_tx==before+(node?2U:1U) && !inject_pending && !inject_enable);
  assert(mode[2]==0 && mode[6]==0 && g_motor_foc_failure.valid);
  assert(g_motor_foc_failure.reason==(kind==1?MOTOR_FOC_FAILURE_SPEED_CLIPPED:MOTOR_FOC_FAILURE_TRANSPORT_EPOCH));
  if(kind==1)assert(g_motor_foc_failure.node_id==slots[node]+1 && !g_motor_foc.node[slots[node]].fast_speed_clipped);
  idle_ready();
 }
 puts("PASS: RX clipped-then-normal FAST or epoch ISR after first/last enqueue fails this call and STOPs all nodes, without submitting later IQ");
 /* Preserve the old first cause if an ISR invalidates a restart immediately
  * before its acceptance/clear critical section. IRQs after that acceptance
  * boundary belong to the new run and may legitimately capture a new cause. */
 failure_ready(torque);fail_next=1;assert(!cycle(torque,0x44U));
 MotorFocFailureDebug first_cause=failure_snapshot();
 assert(first_cause.reason==MOTOR_FOC_FAILURE_TX_ENQUEUE);
 idle_ready();inject_kind=3;inject_slot=6;inject_enable=1;
 assert(!cycle(torque,0x44U));
 assert(!inject_pending && !inject_enable && !inject_before);
 assert(mode[2]==0 && mode[6]==0);
 assert(!memcmp(&first_cause,(const void*)&g_motor_foc_failure,sizeof(first_cause)));
 idle_ready();assert(cycle(torque,0x44U) && !g_motor_foc_failure.valid);
 MotorFoc_StopAll();idle_ready();
 puts("PASS: epoch ISR before restart acceptance preserves the old first failure; next accepted run clears it");
 puts("PASS: both FAST int16 endpoints retain telemetry without malformed frames, transient clipping stops the active group, recovery/rearm; first failure TX/ACK timeout/reject/epoch/health/staleness snapshots survive STOP and failed starts");
}
static void fixed_limit_fixture_reset(void) {
 MotorFoc_Init();remote_iq_limit=2;remote_slew=1;remote_voltage=5;
 wrong_write_echo=0;emit_fast=1;stall_tx=fail_next=transport_blocked=0;
 hw=(FDCAN_GlobalTypeDef){0};
 for(unsigned i=0;i<8;++i) {
  mode[i]=fault[i]=diagnostic_flags[i]=drop_ack[i]=drop_parameter_reply[i]=0;
  drop_write_ack[i]=drop_write_parameter_reply[i]=0;
  flags[i]=0x3e;individual_voltage[i]=0;individual_slew[i]=1;
  individual_iq_limit[i]=(MOTOR_FOC_JOINT_NODE_MASK&(1U<<i))?2.0f:3.3f;
 }
}
static void fixed_three_amp_limits(void) {
 assert(MOTOR_FOC_FIXED_IQ_LIMIT_A==3.0f);
 assert(MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A==3.0f &&
        MOTOR_FOC_CURRENT_HARD_LIMIT_A==3.0f && MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A==3.0f);
 fixed_limit_fixture_reset();
 float torque[8]={0};
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  assert(g_motor_foc.calibration[i].current_limit_a==3.0f);
  assert(MotorFoc_TorqueLimitNm((uint8_t)(i+1U))==0);
  torque[i]=4.0f*g_motor_foc.calibration[i].kt_nm_per_a;
 }
 unsigned before_iq=iq_tx,before_writes=parameter_write_tx;
 assert(!cycle(torque,0x77U));
 assert(iq_tx==before_iq && parameter_write_tx==before_writes);
 for(unsigned k=0;k<100;++k) {
  cycle(NULL,0);
  for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
   if(!g_motor_foc.node[i].limit_verified || g_motor_foc.node[i].remote_iq_limit_a!=3.0f ||
      !g_motor_foc.node[i].slew_verified)
    assert(MotorFoc_TorqueLimitNm((uint8_t)(i+1U))==0);
  }
 }
 idle_ready();
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  assert(individual_iq_limit[i]==3.0f && limit_write_tx[i]>0);
  assert(g_motor_foc.node[i].remote_iq_limit_a==3.0f && g_motor_foc.node[i].limit_verified);
  assert(individual_slew[i]==MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S);
  assert(fabsf(MotorFoc_TorqueLimitNm((uint8_t)(i+1U))-
               3.0f*g_motor_foc.calibration[i].kt_nm_per_a)<1e-6f);
 }
 assert(iq_tx==before_iq);
 assert(cycle(torque,0x77U));
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i))assert(abs(iq_ma[i])==3000);
 before_writes=parameter_write_tx;
 g_motor_foc.maintenance_aux[0]=3;
 float three=3;uint32_t bits;memcpy(&bits,&three,4);
 g_motor_foc.maintenance_value[0]=bits;g_motor_foc.maintenance_opcode[0]=0x11;
 for(unsigned k=0;k<10;++k)assert(cycle(torque,0x77U));
 assert(parameter_write_tx==before_writes && !g_motor_foc.maintenance_opcode[0]);
 puts("PASS: production-default six-node3A STOP sync, ACK+exact readback before torque,3A effective clamps and no active parameter writes");

 /* An independent driver reboot restores its RAM setting. Existing motion
  * shutdown still requires200ms quiet before any new parameter write. */
 MotorFoc_StopAll();before_writes=parameter_write_tx;
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  individual_iq_limit[i]=2;individual_slew[i]=1;diagnostic_flags[i]|=8U;
 }
 for(unsigned k=0;k<35;++k) {
  cycle(NULL,0);assert(parameter_write_tx==before_writes);
 }
 idle_ready();
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  assert(individual_iq_limit[i]==3 && g_motor_foc.node[i].remote_iq_limit_a==3);
  assert(g_motor_foc.node[i].limit_verified && individual_slew[i]==MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S);
 }
 puts("PASS: reboot revokes cached caps and restores3A under STOP only after the existing200ms no-motion interval");

 /* A finite, in-range but wrong echoed value must revoke verification. */
 fixed_limit_fixture_reset();wrong_write_echo=2;
 unsigned rejected=g_motor_foc.node[0].rejected_count;
 before_iq=iq_tx;
 for(unsigned k=0;k<80 && g_motor_foc.node[0].rejected_count==rejected;++k)cycle(NULL,0);
 assert(g_motor_foc.node[0].rejected_count>rejected && !g_motor_foc.node[0].limit_verified);
 assert(MotorFoc_TorqueLimitNm(1)==0 && iq_tx==before_iq);
 assert(!cycle(torque,0x77U) && iq_tx==before_iq);
 wrong_write_echo=0;idle_ready();

 for(unsigned missing=0;missing<2;++missing) {
  fixed_limit_fixture_reset();
  if(missing)drop_write_parameter_reply[0]=1;else drop_write_ack[0]=1;
  unsigned writes=limit_write_tx[0];before_iq=iq_tx;
  for(unsigned k=0;k<80 && limit_write_tx[0]==writes;++k)cycle(NULL,0);
  assert(limit_write_tx[0]>writes && !g_motor_foc.node[0].limit_verified);
  assert(MotorFoc_TorqueLimitNm(1)==0 && iq_tx==before_iq);
  unsigned timeouts=g_motor_foc.node[0].timeout_count;
  for(unsigned k=0;k<9;++k) {
   cycle(NULL,0);assert(iq_tx==before_iq);
  }
  assert(g_motor_foc.node[0].timeout_count>timeouts);
  drop_write_parameter_reply[0]=drop_write_ack[0]=0;idle_ready();
 }
 puts("PASS: wrong3A echo, missing write ACK and missing parameter page fail closed; later fresh complete readback permits recovery");

 /* The maintenance API may set only its configured fixed IQ_LIMIT and
  * bounded IQ_SLEW. It cannot become an arbitrary configuration writer. */
 before_writes=parameter_write_tx;
 float other=2.5f;memcpy(&bits,&other,4);
 g_motor_foc.maintenance_value[0]=bits;g_motor_foc.maintenance_aux[0]=3;
 rejected=g_motor_foc.node[0].rejected_count;g_motor_foc.maintenance_opcode[0]=0x11;
 for(unsigned k=0;k<4;++k)cycle(NULL,0);
 assert(g_motor_foc.node[0].rejected_count>rejected && parameter_write_tx==before_writes);
 g_motor_foc.maintenance_aux[0]=4;g_motor_foc.maintenance_opcode[0]=0x11;
 for(unsigned k=0;k<4;++k)cycle(NULL,0);
 assert(parameter_write_tx==before_writes && individual_iq_limit[0]==3);
 puts("PASS: only configured3A param3 and bounded param5 writes are allowed; voltage/gains and other current ceilings stay excluded");
}
int main(void) {
 hfdcan1.Instance=&hw;
 if(MOTOR_FOC_FIXED_IQ_LIMIT_A>0) {fixed_three_amp_limits();return 0;}
 if (!(MOTOR_FOC_OUTPUT_ENABLE_MASK&0x44U)) {
  MotorFoc_Init();emit_fast=1;remote_iq_limit=2.0f;
  for(unsigned i=0;i<8;++i) {flags[i]=0x3e;sensor_direction[i]=1;
   g_motor_foc.calibration[i].angle_anchor_confirmed=1;
   g_motor_foc.calibration[i].joint_timing_confirmed=1;}
  idle_ready();calibrate();float t[8]={.1f,.1f,.1f,0,.1f,.1f,.1f,0};
  for(unsigned k=0;k<10;++k)assert(cycle(t,0x77U));
  assert(mode[0]==3 && mode[1]==3 && mode[4]==3 && mode[5]==3);
  assert(mode[2]==0 && mode[6]==0 && !iq_ma[2] && !iq_ma[6]);
  MotorFoc_StopAll();idle_ready();unsigned prior=iq_tx,prior_align=align_tx;
  g_motor_foc.debug_current_ma[2]=g_motor_foc.debug_current_ma[6]=200;
  g_motor_foc.debug_duration_ms[2]=g_motor_foc.debug_duration_ms[6]=100;
  g_motor_foc.debug_request_mask=0x44U;
  g_motor_foc.maintenance_opcode[2]=g_motor_foc.maintenance_opcode[6]=4;
  for(unsigned k=0;k<40;++k)cycle(NULL,0);
  assert(iq_tx==prior && align_tx==prior_align && mode[2]==0 && mode[6]==0);
  assert(!g_motor_foc.debug_request_mask && !g_motor_foc.maintenance_opcode[2] && !g_motor_foc.maintenance_opcode[6]);
  puts("PASS: wheel feedback retained, wheel torque/pulses/alignment inhibited; four joints operate");return 0;
 }
 MotorFoc_Init();check_initialized_joint_configuration();
 /* D/Q trace is passive and cannot renew FAST/status freshness or arm output. */
 {
  uint8_t d[8]={0xa1,0,0,0,0,0,0,0};unsigned prior=tx;
  put16(d+2,-100);put16(d+4,2000);put16(d+6,-250);
  unsigned status=g_motor_foc.node[2].status_count,fast=g_motor_foc.node[2].fast_count;
  assert(MotorFoc_OnFrame(0x583,d,now));d[1]=1;put16(d+2,5000);
  assert(MotorFoc_OnFrame(0x583,d,now));
  assert(fabsf(g_motor_foc_dq[2].id_a+.1f)<1e-6f &&
         g_motor_foc_dq[2].iq_reference_a==2 && g_motor_foc_dq[2].vq_v==5 &&
         fabsf(g_motor_foc_dq[2].vd_v+.25f)<1e-6f);
  assert((g_motor_foc_dq[2].seen&3U)==3 && tx==prior &&
         g_motor_foc.node[2].status_count==status && g_motor_foc.node[2].fast_count==fast);
 }
 if(MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S>0) {
  for(unsigned i=0;i<8;++i){flags[i]=0x3e;individual_slew[i]=1;}
  idle_ready();
  for(unsigned k=0;k<60;++k)cycle(NULL,0);
  for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
   assert(individual_slew[i]==MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S &&
          g_motor_foc.node[i].remote_iq_slew_a_per_s==MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S);
   assert(g_motor_foc.node[i].slew_verified && mode[i]==0 && !(flags[i]&1));
  }
  assert(iq_tx==0);
  check_speed_clipping_and_failure_context();
  printf("PASS: six independent driver defaults restored to fixed%gA/s under STOP, readback before torque, no motion\n",(double)MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S);
  return 0;
 }
 for(unsigned i=0;i<8;++i) {
  g_motor_foc.calibration[i].angle_anchor_rad=NAN;
  g_motor_foc.calibration[i].angle_anchor_confirmed=1;
  g_motor_foc.calibration[i].joint_timing_confirmed=1;
  g_motor_foc.calibration[i].dynamic_response_confirmed=1;
  g_motor_foc.calibration[i].minimum_iq_slew_a_per_s=2;
 }
 MotorFoc_Init();check_initialized_joint_configuration();assert(tx==0);
 for(unsigned i=0;i<8;++i) {flags[i]=0x3e;sensor_direction[i]=1;}
 check_startup_joint_observation();
 assert(fabsf(g_motor_foc.calibration[2].current_limit_a-2.0f)<1e-6f);
 /* Retain the smaller per-node limit regression independently of hard cap. */
 g_motor_foc.calibration[2].current_limit_a=g_motor_foc.calibration[6].current_limit_a=1.0f;
 idle_ready();assert(iq_tx==0 && stop_tx>0);
 assert(g_motor_foc.node[2].remote_iq_limit_a==1 && g_motor_foc.node[2].remote_iq_slew_a_per_s==1 && g_motor_foc.node[2].remote_voltage_limit_v==5);
 float torque[8]={0};torque[2]=torque[6]=.02f;
 if(MOTOR_FOC_COMMISSIONED_RESPONSE_MASK&4U)assert(MotorFoc_TorqueLimitNm(3)>0);
 g_motor_foc.calibration[2].calibration_confirmed=0;
 assert(!cycle(torque,0x44));assert(iq_tx==0);idle_ready();
 calibrate();g_motor_foc.calibration[2].dynamic_response_confirmed=0;
 assert(!cycle(torque,0x44));assert(iq_tx==0);idle_ready();
 calibrate();g_motor_foc.calibration[2].minimum_iq_slew_a_per_s=2;
 assert(!cycle(torque,0x44));assert(iq_tx==0);idle_ready();calibrate();
 for(unsigned k=0;k<650;++k) assert(cycle(torque,0x44));
 assert(iq_ma[2]==-400 && iq_ma[6]==200);assert(wraps[2]>0 && wraps[6]>0);
 /* An external STOP must revoke our old target rather than re-enable it. */
 mode[2]=0;flags[2]&=~1U;unsigned external_stop_iq=iq_tx;
 assert(!cycle(torque,0x44));assert(iq_tx==external_stop_iq);idle_ready();calibrate();
 /* Tiny/invalid Kt cannot turn an infinite current into a clipped full-scale command. */
 g_motor_foc.calibration[2].kt_nm_per_a=1e-44f;unsigned before_nonfinite=iq_tx;
 assert(!cycle(torque,0x44));assert(iq_tx==before_nonfinite);idle_ready();calibrate();
 assert(fabsf(MotorCommand_CurrentForTorque(3,.02f)+.4f)<1e-6f);
 assert(fabsf(MotorCommand_CurrentForTorque(7,.02f)-.2f)<1e-6f);
 MotorFeedback f;assert(MotorFoc_GetFeedback(3,&f));assert(f.protocol_a1 && fabsf(f.speed_rad_s+1.234f)<1e-6f);
 /* Deliberately disagree: STATUS must never override fresh wheel FAST.
  * Fresh health pages cannot mask stale or malformed control kinematics. */
 {
  uint8_t fast[8]={0xa1,0x3e,0,0,0,0,0,0};
  put16(fast+2,1200);put16(fast+4,2345);
  assert(MotorFoc_OnFrame(0x703,fast,now));
  assert(MotorFoc_GetFeedback(3,&f) && fabsf(f.speed_rad_s-2.345f)<1e-6f);
  assert(f.last_rx_ms==now && f.angle_age_ms==0 && f.age_ms==0);
  emit_fast=0;now+=MOTOR_FOC_FAST_TIMEOUT_MS+1;fresh_nodes(0);
  assert(!MotorFoc_GetFeedback(3,&f));
  fast[1]=0;assert(MotorFoc_OnFrame(0x703,fast,now));
  assert(!MotorFoc_GetFeedback(3,&f));
  fast[1]=0x3e;assert(MotorFoc_OnFrame(0x703,fast,now));
  assert(MotorFoc_GetFeedback(3,&f));
  fault[2]=1;feed(2,0);assert(!MotorFoc_GetFeedback(3,&f));
  fault[2]=0;feed(2,0);assert(MotorFoc_GetFeedback(3,&f));
  emit_fast=1;
 }
 now+=80;fresh_nodes(0);assert(MotorFoc_GetFeedback(3,&f));assert(f.angle_age_ms==0);
 now+=41;assert(!MotorFoc_GetFeedback(3,&f));assert(!MotorFoc_Service(torque,0x44));
 unsigned lost_iq=iq_tx;for(unsigned k=0;k<10;++k)cycle(torque,0x44);assert(iq_tx==lost_iq);
 idle_ready();calibrate();for(unsigned k=0;k<4;++k)cycle(torque,0x44);
 drop_ack[2]=1;for(unsigned k=0;k<12;++k)cycle(torque,0x44);
 unsigned after_timeout=iq_tx;for(unsigned k=0;k<20;++k)cycle(torque,0x44);
 assert(g_motor_foc.node[2].timeout_count>0 && iq_tx==after_timeout && hw.TXBCR==0xffU);
 drop_ack[2]=0;idle_ready();calibrate();
 /* Mixed legacy packets never contain A1-node mV slots. */
 int16_t volts[8]={111,222,333,444,555,666,777,888};
 now+=5;fresh_nodes(1);assert(MotorCommand_SendOutputs(volts,torque,0x44)==2);
 assert(signed16(legacy[0])==(MotorCommand_UsesCurrent(1)?0:111));
 assert(signed16(legacy[1]+2)==(MotorCommand_UsesCurrent(6)?0:666));
 assert(signed16(legacy[0]+4)==0 && signed16(legacy[1]+4)==0);
 MotorFoc_InvalidateFromISR();assert(!MotorFoc_GetFeedback(3,&f));
 assert(MotorCommand_SendOutputs(volts,torque,0x44)==0);
 for(unsigned a=0;a<2;++a)for(unsigned b=0;b<8;++b)assert(legacy[a][b]==0);
 idle_ready();calibrate();
 /* Supply/remote cap is confirmed by readback, never silently raised. */
 g_motor_foc.calibration[2].current_limit_a=.25f;
 for(unsigned k=0;k<4;++k)cycle(torque,0x44);
 assert(iq_ma[2]==-250 && iq_ma[6]==200);
 idle_ready();
 /* A board's request ceiling is independent of the host's physical 1 A cap. */
 g_motor_foc.calibration[2].current_limit_a=1.0f;
 const float accepted_limits[]={2.0f,3.3f,.25f};
 for(unsigned k=0;k<3;++k) {
  remote_iq_limit=accepted_limits[k];MotorFoc_StopAll();idle_ready();calibrate();
  assert(g_motor_foc.node[2].remote_iq_limit_a==remote_iq_limit);
  assert(g_motor_foc.node[2].limit_verified);
  float expected_limit=fminf(1.0f,remote_iq_limit),large_torque[8]={0};large_torque[2]=.2f;
  assert(fabsf(MotorFoc_TorqueLimitNm(3)-.05f*expected_limit)<1e-6f);
  for(unsigned step=0;step<4;++step)assert(cycle(large_torque,1U<<2));
  assert(abs(iq_ma[2]+(int)(expected_limit*1000.0f))<=1);
  assert(fabsf(g_motor_foc.node[2].applied_iq_a+expected_limit)<1e-6f);
  assert(g_motor_foc.calibration[2].current_limit_a==1.0f);
  MotorFoc_StopAll();idle_ready();
 }
 /* User-requested wheels now permit 2 A, while a value above 2 A fails closed. */
 remote_iq_limit=3.3f;MotorFoc_StopAll();idle_ready();calibrate();
 g_motor_foc.calibration[2].current_limit_a=2.0f;
 { float wheel_torque[8]={0};wheel_torque[2]=.5f;
   assert(fabsf(MotorFoc_TorqueLimitNm(3)-.1f)<1e-6f);
   for(unsigned k=0;k<4;++k)assert(cycle(wheel_torque,4U));
   assert(iq_ma[2]==-2000);
   MotorFoc_StopAll();idle_ready();
   g_motor_foc.calibration[2].current_limit_a=2.001f;
   assert(MotorFoc_TorqueLimitNm(3)==0 && !cycle(wheel_torque,4U));
   g_motor_foc.calibration[2].current_limit_a=1.0f;idle_ready();
 }
 if(MOTOR_FOC_NODE_MASK&1U) {
  unsigned previous_fast=emit_fast;emit_fast=1;
  g_motor_foc.calibration[0].angle_anchor_confirmed=1;
  g_motor_foc.calibration[0].joint_timing_confirmed=1;
  remote_iq_limit=2.0f;MotorFoc_StopAll();idle_ready();calibrate();
  float joint_torque[8]={0};joint_torque[0]=.5f;
  assert(fabsf(MotorFoc_TorqueLimitNm(1)-.2f)<1e-6f);
  for(unsigned k=0;k<4;++k)assert(cycle(joint_torque,1U));
  assert(iq_ma[0]==-2000 && g_motor_foc.calibration[2].current_limit_a==1.0f);
  MotorFoc_StopAll();idle_ready();
  g_motor_foc.calibration[0].current_limit_a=2.001f;
  assert(MotorFoc_TorqueLimitNm(1)==0);
  assert(!cycle(joint_torque,1U));
  g_motor_foc.calibration[0].current_limit_a=2.0f;idle_ready();emit_fast=previous_fast;
 }
 const float rejected_limits[]={3.301f,NAN,0.0f};
 for(unsigned k=0;k<3;++k) {
  remote_iq_limit=rejected_limits[k];MotorFoc_StopAll();unsigned no_current=iq_tx;
  for(unsigned step=0;step<100;++step)cycle(NULL,0);
  assert(!g_motor_foc.node[2].limit_verified);
  assert(MotorFoc_TorqueLimitNm(3)==0.0f && iq_tx==no_current);
 }
 remote_iq_limit=1.0f;MotorFoc_StopAll();idle_ready();calibrate();
 /* Only wheels3/7 may advertise7.2V. Capability never raises the2A ceiling. */
 remote_iq_limit=3.3f;
 individual_voltage[2]=individual_voltage[6]=MOTOR_FOC_WHEEL_VOLTAGE_LIMIT_MAX_V;
 MotorFoc_StopAll();idle_ready();calibrate();
 assert(g_motor_foc.node[2].remote_voltage_limit_v==MOTOR_FOC_WHEEL_VOLTAGE_LIMIT_MAX_V &&
        g_motor_foc.node[6].remote_voltage_limit_v==MOTOR_FOC_WHEEL_VOLTAGE_LIMIT_MAX_V);
 { float t[8]={0};t[2]=.5f;g_motor_foc.calibration[2].current_limit_a=2;
   for(unsigned k=0;k<4;++k)assert(cycle(t,4U));
   assert(iq_ma[2]==-2000);g_motor_foc.calibration[2].current_limit_a=1;
   MotorFoc_StopAll();idle_ready(); }
 for(unsigned slot=2;slot<=6;slot+=4) {
  individual_voltage[slot]=MOTOR_FOC_WHEEL_VOLTAGE_LIMIT_MAX_V+.001f;MotorFoc_StopAll();unsigned prior=iq_tx;
  for(unsigned k=0;k<100;++k)cycle(NULL,0);
  assert(!g_motor_foc.node[slot].voltage_verified && iq_tx==prior);
  individual_voltage[slot]=MOTOR_FOC_WHEEL_VOLTAGE_LIMIT_MAX_V;MotorFoc_StopAll();idle_ready();
 }
 if(MOTOR_FOC_NODE_MASK&1U) {
  individual_voltage[0]=5.001f;MotorFoc_StopAll();unsigned prior=iq_tx;
  for(unsigned k=0;k<100;++k)cycle(NULL,0);
  assert(!g_motor_foc.node[0].voltage_verified && iq_tx==prior);
  individual_voltage[0]=0;MotorFoc_StopAll();idle_ready();
 }
 individual_voltage[2]=individual_voltage[6]=0;remote_iq_limit=1;MotorFoc_StopAll();idle_ready();
 /* Stopped node slew write needs ACK and exact echoed float; nothing else writable. */
 for(unsigned slot=0;slot<8;++slot) if(MOTOR_FOC_NODE_MASK&(1U<<slot)) {
  float slew=5;uint32_t bits;memcpy(&bits,&slew,4);unsigned prior=iq_tx;
  g_motor_foc.maintenance_value[slot]=bits;g_motor_foc.maintenance_aux[slot]=5;
  g_motor_foc.maintenance_opcode[slot]=0x11;
  for(unsigned k=0;k<12;++k)cycle(NULL,0);
  assert(!g_motor_foc.maintenance_opcode[slot] &&
         g_motor_foc.node[slot].remote_iq_slew_a_per_s==5 && iq_tx==prior && mode[slot]==0);
  unsigned rejected=g_motor_foc.node[slot].rejected_count;
  g_motor_foc.maintenance_aux[slot]=3;g_motor_foc.maintenance_opcode[slot]=0x11;
  for(unsigned k=0;k<4;++k)cycle(NULL,0);
  assert(g_motor_foc.node[slot].rejected_count>rejected && remote_iq_limit==1);
  slew=1;memcpy(&bits,&slew,4);g_motor_foc.maintenance_value[slot]=bits;
  g_motor_foc.maintenance_aux[slot]=5;g_motor_foc.maintenance_opcode[slot]=0x11;
  for(unsigned k=0;k<12;++k)cycle(NULL,0);
  assert(!g_motor_foc.maintenance_opcode[slot] && remote_slew==1 && iq_tx==prior);
  g_motor_foc.maintenance_aux[slot]=0;
 }
 { float slew=5;uint32_t bits;memcpy(&bits,&slew,4);unsigned prior=iq_tx;
  g_motor_foc.maintenance_value[2]=bits;g_motor_foc.maintenance_aux[2]=5;g_motor_foc.maintenance_opcode[2]=0x11;
  for(unsigned k=0;k<12;++k)cycle(NULL,0);
  assert(!g_motor_foc.maintenance_opcode[2] && g_motor_foc.node[2].remote_iq_slew_a_per_s==5 && iq_tx==prior);
  unsigned rejected=g_motor_foc.node[2].rejected_count;
  g_motor_foc.maintenance_aux[2]=3;g_motor_foc.maintenance_opcode[2]=0x11;
  for(unsigned k=0;k<4;++k)cycle(NULL,0);assert(g_motor_foc.node[2].rejected_count>rejected && remote_iq_limit==1);
  g_motor_foc.maintenance_aux[2]=5;wrong_write_echo=1;g_motor_foc.maintenance_opcode[2]=0x11;
  rejected=g_motor_foc.node[2].rejected_count;for(unsigned k=0;k<12;++k)cycle(NULL,0);
  assert(g_motor_foc.node[2].rejected_count>rejected);wrong_write_echo=0;idle_ready();
  unsigned old_timeout=g_motor_foc.node[2].timeout_count;
  drop_parameter_reply[2]=1;g_motor_foc.maintenance_opcode[2]=0x11;
  for(unsigned k=0;k<12;++k)cycle(NULL,0);
  assert(g_motor_foc.node[2].timeout_count>old_timeout && iq_tx==prior);
  drop_parameter_reply[2]=0;idle_ready();
  slew=1;memcpy(&bits,&slew,4);g_motor_foc.maintenance_value[2]=bits;g_motor_foc.maintenance_opcode[2]=0x11;
  for(unsigned k=0;k<12;++k)cycle(NULL,0);assert(remote_slew==1 && !g_motor_foc.maintenance_opcode[2]);
  g_motor_foc.maintenance_aux[2]=0;
 }
 /* Explicit direction/alignment while stopped, never on initialization. */
 g_motor_foc.maintenance_value[2]=-1;g_motor_foc.maintenance_opcode[2]=6;
 for(unsigned k=0;k<10;++k)cycle(NULL,0);assert(!(flags[2]&32U));
 alignment_duration_ms=4000;unsigned before_align=align_tx;
 g_motor_foc.maintenance_opcode[2]=4;
 for(unsigned k=0;k<850;++k)cycle(NULL,0);
 assert(flags[2]&32U);assert(mode[2]==0 && !g_motor_foc.maintenance_opcode[2]);
 assert(align_tx-before_align>=70); /* 50 ms refreshes survive the 100 ms board watchdog. */
 /* A board that never completes alignment still receives STOP at the deadline. */
 g_motor_foc.maintenance_value[2]=-1;g_motor_foc.maintenance_opcode[2]=6;
 for(unsigned k=0;k<10;++k)cycle(NULL,0);assert(!(flags[2]&32U));
 alignment_duration_ms=MOTOR_FOC_ALIGN_TIMEOUT_MS+1000U;
 g_motor_foc.maintenance_opcode[2]=4;
 for(unsigned k=0;k<(MOTOR_FOC_ALIGN_TIMEOUT_MS+200U)/5U;++k)cycle(NULL,0);
 assert(mode[2]==0 && !(flags[2]&32U) && !g_motor_foc.maintenance_opcode[2]);
 assert(align_stopped_ms[2]-align_ms[2]>MOTOR_FOC_ALIGN_TIMEOUT_MS);
 assert(align_stopped_ms[2]-align_ms[2]<=MOTOR_FOC_ALIGN_TIMEOUT_MS+10U);
 alignment_duration_ms=100;idle_ready();g_motor_foc.maintenance_opcode[2]=4;
 for(unsigned k=0;k<50;++k)cycle(NULL,0);assert(flags[2]&32U);
 /* Persistent-calibration maintenance correlates ACK and info, never a stale page. */
 idle_ready();g_motor_foc.maintenance_value[2]=0;g_motor_foc.maintenance_aux[2]=0;
 g_motor_foc.maintenance_opcode[2]=0x15;
 for(unsigned k=0;k<5;++k)cycle(NULL,0);
 assert(g_motor_foc.stored_calibration[2].completed);
 assert(g_motor_foc.stored_calibration[2].flags==8U);
 assert(g_motor_foc.stored_calibration[2].sensor_direction==0);
 assert(g_motor_foc.stored_calibration[2].electrical_zero_rad==0.0f);
 drop_calibration_reply[2]=1;g_motor_foc.maintenance_opcode[2]=0x15;
 cycle(NULL,0);
 assert(!g_motor_foc.stored_calibration[2].completed);
 unsigned info_before=g_motor_foc.stored_calibration[2].rx_count;
 uint8_t info[8]={0xa1,0x15,(uint8_t)(g_motor_foc.stored_calibration[2].request_sequence+1U),0,8,0,0,0};
 MotorFoc_OnFrame(0x643,info,now);
 assert(g_motor_foc.stored_calibration[2].rx_count==info_before);
 info[2]=(uint8_t)g_motor_foc.stored_calibration[2].request_sequence;info[4]=0x80;
 unsigned malformed_before=g_motor_foc.malformed;MotorFoc_OnFrame(0x643,info,now);
 assert(g_motor_foc.malformed==malformed_before+1 && !g_motor_foc.stored_calibration[2].completed);
 uint8_t bad_zero[8];memcpy(bad_zero,info,8);bad_zero[4]=1;bad_zero[5]=1;put16(bad_zero+6,6284);
 MotorFoc_OnFrame(0x643,bad_zero,now);
 assert(g_motor_foc.malformed==malformed_before+2 && !g_motor_foc.stored_calibration[2].completed);
 for(unsigned k=0;k<100;++k)cycle(NULL,0); /* Valid ACK alone is insufficient, but 500ms is allowed. */
 assert(!g_motor_foc.stored_calibration[2].completed);
 info[4]=8;MotorFoc_OnFrame(0x643,info,now);cycle(NULL,0);
 assert(g_motor_foc.stored_calibration[2].completed);drop_calibration_reply[2]=0;
 /* Info-first completion also requires its matching successful ACK. */
 drop_ack[2]=1;g_motor_foc.maintenance_opcode[2]=0x15;cycle(NULL,0);
 assert(!g_motor_foc.stored_calibration[2].completed);
 uint8_t late_ack[8]={0xa1,0x15,(uint8_t)g_motor_foc.stored_calibration[2].request_sequence,0,0,0,0,0};
 MotorFoc_OnFrame(0x383,late_ack,now);cycle(NULL,0);
 assert(g_motor_foc.stored_calibration[2].completed);drop_ack[2]=0;
 /* A control request cannot overlap or queue behind an in-flight READ. */
 drop_calibration_reply[2]=1;g_motor_foc.maintenance_opcode[2]=0x15;cycle(NULL,0);
 assert(!g_motor_foc.stored_calibration[2].completed);unsigned current_before_read=iq_tx;
 assert(!cycle(torque,1U<<2));assert(iq_tx==current_before_read);
 drop_calibration_reply[2]=0;idle_ready();assert(iq_tx==current_before_read);
 assert(!g_motor_foc.stored_calibration[2].completed && !g_motor_foc.maintenance_opcode[2]);
 unsigned saves_before=calibration_save_tx,stops_before_save=stop_tx;
 flags[2]&=~32U;g_motor_foc.maintenance_opcode[2]=0x16;cycle(NULL,0);
 assert(calibration_save_tx==saves_before && !g_motor_foc.maintenance_opcode[2]);
 flags[2]|=32U;idle_ready();calibrate();
 g_motor_foc.maintenance_opcode[2]=0x16;assert(cycle(torque,1U<<2));
 assert(calibration_save_tx==saves_before && !g_motor_foc.maintenance_opcode[2]);
 MotorFoc_StopAll();idle_ready();
 g_motor_foc.maintenance_opcode[2]=0x16;
 for(unsigned k=0;k<80;++k)cycle(NULL,0);
 assert(calibration_save_tx==saves_before+1 && stop_tx>stops_before_save);
 assert(g_motor_foc.stored_calibration[2].completed);
 assert(g_motor_foc.stored_calibration[2].flags==0x19U);
 assert(g_motor_foc.stored_calibration[2].sensor_direction==-1);
 assert(fabsf(g_motor_foc.stored_calibration[2].electrical_zero_rad-1.02f)<1e-6f);
 assert(g_motor_foc.node[2].session_ready && g_motor_foc.node[2].limit_verified &&
        g_motor_foc.node[2].voltage_verified && g_motor_foc.node[2].slew_verified);
 /* Raw Iq pulse is useful before Kt/LQR calibration, but bounded and explicit. */
 g_motor_foc.calibration[2].kt_nm_per_a=0;
 g_motor_foc.calibration[2].calibration_confirmed=0;
 g_motor_foc.debug_current_ma[2]=-100;g_motor_foc.debug_duration_ms[2]=100;
 g_motor_foc.debug_request_mask=1U<<2;
 unsigned before_debug=iq_tx;for(unsigned k=0;k<30;++k)cycle(NULL,0);
 assert(iq_tx>before_debug && iq_ma[2]==-100 && mode[2]==0);
 for(int test_ma=-301;test_ma<=301;test_ma+=(test_ma==-301?1:test_ma==-300?600:1)) {
  idle_ready();unsigned prior=iq_tx;
  g_motor_foc.debug_current_ma[2]=test_ma;g_motor_foc.debug_duration_ms[2]=100;
  g_motor_foc.debug_request_mask=1U<<2;
  for(unsigned k=0;k<30;++k)cycle(NULL,0);
  if(test_ma==-300 || test_ma==300)assert(iq_tx>prior && iq_ma[2]==test_ma && mode[2]==0);
  else assert(iq_tx==prior && mode[2]==0);
 }
 idle_ready();g_motor_foc.debug_current_ma[2]=100;g_motor_foc.debug_duration_ms[2]=1000;
 g_motor_foc.debug_request_mask=1U<<2;
 for(unsigned k=0;k<4;++k)cycle(NULL,0);assert(mode[2]==3);
 g_motor_foc.stop_request=1;cycle(NULL,0);assert(!g_motor_foc.stop_request && mode[2]==0);
 unsigned no_replay=iq_tx;for(unsigned k=0;k<60;++k)cycle(NULL,0);assert(iq_tx==no_replay);
 idle_ready();unsigned no_debug=iq_tx;
 g_motor_foc.debug_request_mask=1U<<2;g_motor_foc.debug_current_ma[2]=100;
 for(unsigned k=0;k<30;++k)cycle(NULL,1U); /* legacy joint still active */
 assert(iq_tx==no_debug && g_motor_foc.debug_request_mask==0);
 if(MOTOR_FOC_NODE_MASK&1U) {
  /* Direct joint Iq commissioning does not require a kinematic turn anchor. */
  g_motor_foc.calibration[0].angle_anchor_confirmed=0;
  emit_fast=1;idle_ready();assert(!g_motor_foc.calibration[0].angle_anchor_confirmed);
  g_motor_foc.calibration[0].kt_nm_per_a=0;g_motor_foc.calibration[0].calibration_confirmed=0;
  g_motor_foc.debug_current_ma[0]=100;g_motor_foc.debug_duration_ms[0]=50;
  g_motor_foc.debug_request_mask=1U;unsigned before_joint_debug=iq_tx;
  assert(!MotorFoc_TransportMotionStopped());
  for(unsigned k=0;k<20;++k)cycle(NULL,0);
  assert(iq_tx>before_joint_debug && iq_ma[0]==100 && mode[0]==0);
  emit_fast=0;MotorFoc_OnTransportDown();idle_ready();
 }
 /* Physical robot fault clear is explicit; no automatic CLEAR or ALIGN. */
 fault[2]=0x1000000;MotorFoc_InvalidateFromISR();
 for(unsigned k=0;k<50;++k)cycle(NULL,0);
 g_motor_foc.maintenance_opcode[2]=5;
 for(unsigned k=0;k<50;++k)cycle(NULL,0);
 assert(fault[2]==0);
 /* A reboot revokes all readback caches; old slew10 cannot survive slew1. */
 remote_slew=10;MotorFoc_StopAll();idle_ready();calibrate();
 g_motor_foc.calibration[2].minimum_iq_slew_a_per_s=5;
 assert(MotorFoc_TorqueLimitNm(3)>0);
 remote_slew=1;diagnostic_flags[2]=8;feed(2,1);
 assert(!g_motor_foc.node[2].slew_verified);
 idle_ready();assert(g_motor_foc.node[2].remote_iq_slew_a_per_s==1);
 assert(MotorFoc_TorqueLimitNm(3)==0);
 /* Fast joint angle/speed must be one valid 200Hz frame, never trace2. */
 if(MOTOR_FOC_NODE_MASK&1U) {
  MotorFeedback j;fresh_nodes(1);assert(!MotorFoc_GetFeedback(1,&j));
  g_motor_foc.calibration[0].angle_anchor_confirmed=1;g_motor_foc.calibration[0].joint_timing_confirmed=1;
  g_motor_foc.calibration[0].angle_anchor_rad=6.373f;
  uint8_t fast[8]={0xa1,0x3e,0,0,0,0,0,0};put16(fast+2,90);put16(fast+4,1234);put16(fast+6,-250);
  assert(MotorFoc_OnFrame(0x701,fast,now));assert(MotorFoc_GetFeedback(1,&j));
  assert(fabsf(j.angle_rad-6.3731853f)<.0001f && fabsf(j.speed_rad_s-1.234f)<1e-6f);
  now+=11;fresh_nodes(1);assert(!MotorFoc_GetFeedback(1,&j)); /* fresh trace2 does not renew joint fast age */
  put16(fast+2,6284);MotorFoc_OnFrame(0x701,fast,now);assert(!MotorFoc_GetFeedback(1,&j));
  put16(fast+2,90);fast[1]&=~8U;MotorFoc_OnFrame(0x701,fast,now);assert(!MotorFoc_GetFeedback(1,&j));
  fast[1]=0x3e;MotorFoc_OnFrame(0x701,fast,now);assert(MotorFoc_GetFeedback(1,&j));
  fault[0]=1;feed(0,1);assert(!MotorFoc_GetFeedback(1,&j));
  fault[0]=0;feed(0,1);assert(MotorFoc_GetFeedback(1,&j));
 }
 /* All configured nodes submit fresh independent sequences on the same
  * 200Hz schedule; pure-A1 output never emits empty legacy group packets. */
 emit_fast=1;idle_ready();calibrate();
 for(unsigned i=0;i<8;++i) {
  g_motor_foc.calibration[i].angle_anchor_confirmed=1;
  g_motor_foc.calibration[i].joint_timing_confirmed=1;torque[i]=.01f;
 }
 assert(MOTOR_FOC_COMMAND_PERIOD_MS==5U);
 unsigned before_200hz=iq_tx,node_count=0;
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i))++node_count;
 for(unsigned k=0;k<30;++k)assert(cycle(torque,MOTOR_FOC_NODE_MASK));
 assert(iq_tx-before_200hz==30U*node_count);
 for(unsigned i=0;i<8;++i)if(MOTOR_FOC_NODE_MASK&(1U<<i)) {
  assert(now-last_ms[i]<MOTOR_FOC_COMMAND_PERIOD_MS);
  assert(g_motor_foc_command_timing[i].count==30U);
  assert(g_motor_foc_command_timing[i].max_gap_ms==5U);
  assert(g_motor_foc_command_timing[i].last_ms-g_motor_foc_command_timing[i].first_ms==145U);
  float expected=MotorFoc_CurrentForTorque((uint8_t)(i+1),torque[i])*1000;
  assert(abs(iq_ma[i]-(int)(expected+(expected>=0?.5f:-.5f)))<=1);
 }
 MotorFoc_StopAll();idle_ready();assert(MotorFoc_TransportMotionStopped());
 g_motor_foc.debug_request_mask=4U;assert(!MotorFoc_TransportMotionStopped());
 g_motor_foc.debug_request_mask=0;g_motor_foc.maintenance_opcode[2]=4;
 assert(!MotorFoc_TransportMotionStopped());MotorFoc_OnTransportDown();
 assert(MotorFoc_TransportMotionStopped());
 check_dq_trace_current_session();
 check_current_schedule_phase();
 check_speed_clipping_and_failure_context();
 /* The 1ms integer ISR bounds retries even if the control task never runs. */
 FDCAN_TxHeaderTypeDef h={0};h.Identifier=0x203;h.IdType=FDCAN_STANDARD_ID;
 h.FDFormat=FDCAN_CLASSIC_CAN;h.DataLength=FDCAN_DLC_BYTES_8;
 uint8_t stop[8]={0xa1,0,0,0,0,0,0,0};
 FdcanDeadline_Init();hw=(FDCAN_GlobalTypeDef){0};now=UINT32_MAX-8U;stall_tx=1;latest_buffer=1;
 assert(Fdcan_SendBounded(&h,stop)==HAL_OK);
 FdcanDeadline_TickFromISR(now+14U);assert(hw.TXBCR==0);
 FdcanDeadline_TickFromISR(now+15U);assert(hw.TXBCR==1 && g_fdcan_deadline.expired);
 assert(Fdcan_SendBounded(&h,stop)==HAL_ERROR);assert(FdcanDeadline_TakeExpired());
 FdcanDeadline_Init();hw=(FDCAN_GlobalTypeDef){0};now=100;
 assert(Fdcan_SendBounded(&h,stop)==HAL_OK);
 hw.TXBRP=0;now=112;assert(Fdcan_SendBounded(&h,stop)==HAL_OK);
 FdcanDeadline_TickFromISR(116);assert(hw.TXBCR==0);
 FdcanDeadline_TickFromISR(126);assert(hw.TXBCR==0);
 FdcanDeadline_TickFromISR(127);assert(hw.TXBCR==1);
 FdcanDeadline_Init();hw=(FDCAN_GlobalTypeDef){0};now=100;latest_buffer=1;
 assert(Fdcan_SendBounded(&h,stop)==HAL_OK);now=105;latest_buffer=2;
 assert(Fdcan_SendBounded(&h,stop)==HAL_OK);
 FdcanDeadline_TickFromISR(115);assert(hw.TXBCR==3); /* cancel entire stale batch */
 puts("PASS: joint startup/reinit anchors and geometry, unconfirmed fail-closed, A1/mixed STOP+fresh state, 3.3A readback/joint and wheel 2A clamps, invalid limits, 4s align/5s abort, CAL1 save+ACK/info+reboot STOP, stop-request/no replay, ACK/FAST/fault checks, bounded Iq");
}
'''
with tempfile.TemporaryDirectory(prefix='mylegwheel_foc_') as directory:
    work=Path(directory)
    (work/'runtime.h').write_text(runtime)
    for name in ('main.h','bsp_fdcan.h','FreeRTOS.h','task.h'):
        (work/name).write_text('#include "runtime.h"\n')
    (work/'test.c').write_text(harness)
    exe=work/'test.exe'
    command=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-DMYLEGWHEEL_A1_CURRENT=1','-DROBOT_LEGS_ONLY_CALIBRATION=0',
             '-I',str(work),'-I',str(root/'App/Tasks/Inc'),'-I',str(root/'App/Services/Inc'),
             '-I',str(root/'BSP/Peripherals/Inc'),
             '-I',str(root/'Control/WheelLeg/Inc'),
             '-I',str(root/'App/Protocol/Inc'),
             str(work/'test.c'),str(root/'App/Services/Src/motor_foc.c'),
             str(root/'App/Services/Src/motor_command.c'),str(root/'App/Services/Src/joint_calibration.c'),
             str(root/'Control/WheelLeg/Src/leg_pos.c'),
             str(root/'BSP/Peripherals/Src/bsp_fdcan_deadline.c'),
             '-lm','-o',str(exe)]
    # Historical protocol scenarios deliberately retain their old2A caps.
    # The final invocation below uses unmodified production defaults3A.
    historical = ['-DMOTOR_FOC_FIXED_IQ_LIMIT_A=0.0f',
                  '-DMOTOR_FOC_DEFAULT_CURRENT_LIMIT_A=2.0f',
                  '-DMOTOR_FOC_CURRENT_HARD_LIMIT_A=2.0f',
                  '-DMOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A=2.0f']
    for mask, overrides in (
        ('0x44U', []),
        ('0x77U', ['-DMOTOR_FOC_COMMISSIONED_RESPONSE_MASK=0U']),
        ('0x77U', []),
        ('0x77U', ['-DMOTOR_FOC_JOINT_ANCHOR_CONFIRMED_MASK=0U',
                    '-DMOTOR_FOC_JOINT_TIMING_CONFIRMED_MASK=0U']),
    ):
        subprocess.run(command+historical+['-DMOTOR_FOC_FIXED_IQ_SLEW_A_PER_S=0.0f','-DMOTOR_FOC_OUTPUT_ENABLE_MASK=0x77U','-DMOTOR_FOC_NODE_MASK='+mask]+overrides,check=True)
        subprocess.run([str(exe)],check=True)

    # Independent output-layer inhibition, including direct debug/ALIGN paths.
    subprocess.run(command+historical+['-DMOTOR_FOC_FIXED_IQ_SLEW_A_PER_S=0.0f','-DMOTOR_FOC_OUTPUT_ENABLE_MASK=0x33U','-DMOTOR_FOC_NODE_MASK=0x77U'],check=True)
    subprocess.run([str(exe)],check=True)
    subprocess.run(command+historical+['-DMOTOR_FOC_OUTPUT_ENABLE_MASK=0x77U','-DMOTOR_FOC_NODE_MASK=0x77U'],check=True)
    subprocess.run([str(exe)],check=True)
    # No cap, fixed-current or slew overrides: exercise the real default3A.
    subprocess.run(command+['-DMOTOR_FOC_OUTPUT_ENABLE_MASK=0x77U','-DMOTOR_FOC_NODE_MASK=0x77U'],check=True)
    subprocess.run([str(exe)],check=True)
