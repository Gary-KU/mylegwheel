"""Host checks for the observation task, pure LQR/VMC core and CAN encoder."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
runtime = r'''
#ifndef RUNTIME_H
#define RUNTIME_H
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef struct {int unused;} FDCAN_HandleTypeDef;
typedef struct {
 uint32_t Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator;
 uint32_t BitRateSwitch,FDFormat,TxEventFifoControl;
} FDCAN_TxHeaderTypeDef;
extern FDCAN_HandleTypeDef hfdcan1;
extern unsigned critical;
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (assert(critical),--critical)
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define pdPASS 1
#define HAL_OK 0
#define FDCAN_STANDARD_ID 1
#define FDCAN_DATA_FRAME 2
#define FDCAN_DLC_BYTES_8 8
#define FDCAN_ESI_ACTIVE 3
#define FDCAN_BRS_OFF 4
#define FDCAN_CLASSIC_CAN 5
#define FDCAN_NO_TX_EVENTS 6
uint32_t HAL_GetTick(void);
TickType_t xTaskGetTickCount(void);
unsigned uxTaskGetStackHighWaterMark(void *);
void vTaskDelayUntil(TickType_t *,TickType_t);
int xTaskCreate(void (*)(void *),const char *,unsigned,void *,unsigned,void *);
void Error_Handler(void);
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *,FDCAN_TxHeaderTypeDef *,uint8_t *);
/* Historical controller fixtures. Native production has a dedicated suite. */
#ifndef ROBOT_NATIVE_LQR
#define ROBOT_NATIVE_LQR 0
#endif
#endif
'''
harness = r'''
#include "runtime.h"
#include "wheel_leg_task.h"
#include "lqr_startup_record.h"
volatile LqrRecordDebug g_lqr_record;
void LqrStartupRecord_Init(void) {}
void LqrStartupRecord_Begin(const WheelLegDebug *state) {
 assert(state && g_robot_control.enable && !g_robot_control.feedback_only);
}
void LqrStartupRecord_Capture(const WheelLegDebug *state) { (void)state; }
void RobotTelemetry_Capture(const WheelLegDebug *state) { (void)state; }
#include "wheel_leg_config.h"
#include "wheel_leg_control.h"
#include "joint_calibration.h"
#include "joint_calibration_config.h"
#include "motor_command.h"
#include "can_task.h"
#include "imu_task.h"
#include "leg_trajectory.h"
#include "lqr_k.h"
#include "sbus_receiver.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
#define assert(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)

unsigned critical;
FDCAN_HandleTypeDef hfdcan1;
const JointCalibration g_joint_calibration[4]={
 {JOINT_LF_CAN_ID,JOINT_LEFT_FRONT,0,0,1,JOINT_LF_DIRECTION},
 {JOINT_LR_CAN_ID,JOINT_LEFT_REAR,0,0,1,JOINT_LR_DIRECTION},
 {JOINT_RF_CAN_ID,JOINT_RIGHT_FRONT,0,0,1,JOINT_RF_DIRECTION},
 /* Exercise shaft-rate reconstruction with a geared host fixture. */
 {JOINT_RR_CAN_ID,JOINT_RIGHT_REAR,0,0,1.5f,JOINT_RR_DIRECTION}};
static uint32_t now=100, calls, fail_at, created, received_mask;
static int imu_ok=1;
static MotorFeedback motors[8];
static ImuSample imu;
static SbusState remote_sample;
static unsigned remote_ok, remote_fresh;
static SbusSticks motion_sticks;
uint8_t SBUS_GetSticks(SbusSticks *out) {
 *out=motion_sticks;
 if(out->valid) out->received_ms=now;
 return out->valid;
}
static uint8_t data[2][8];
uint8_t SBUS_GetState(SbusState *out) {
 if(remote_ok && remote_fresh) {remote_sample.received_ms=now;++remote_sample.frame_count;}
 *out=remote_sample;
 out->link_ok=remote_ok && !out->frame.failsafe && !out->frame.frame_lost &&
              (uint32_t)(now-out->received_ms)<=SBUS_TIMEOUT_MS;
 return out->link_ok;
}
uint32_t HAL_GetTick(void) {return now;}
TickType_t xTaskGetTickCount(void) {return now;}
unsigned uxTaskGetStackHighWaterMark(void *task) {assert(!task);return 512;}
void vTaskDelayUntil(TickType_t *last,TickType_t ticks) {(void)last;assert(ticks==5);}
int xTaskCreate(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,void *handle) {
 assert(fn && strcmp(name,"wheel_leg")==0 && stack==1024 && !arg && priority==4 && !handle);
 ++created;return pdPASS;
}
void Error_Handler(void) {assert(0);}
uint8_t IMU_GetSample(ImuSample *sample) {*sample=imu;return (uint8_t)imu_ok;}
uint8_t MotorCAN_GetFeedback(uint8_t id,MotorFeedback *f) {
 assert(critical && id>=1 && id<=8);received_mask|=1U<<(id-1);
 *f=motors[id-1];
 /* This prescribed-feedback fixture specifies age directly. Give A1
  * samples a consistent original timestamp for the common-time observer. */
 if(f->protocol_a1)f->last_rx_ms=now-f->age_ms;
 return f->online;
}
#ifdef MOTOR_FEEDBACK_HEALTH_VERSION
uint8_t MotorCAN_GetFeedbackWithHealth(uint8_t id,MotorFeedback *f,MotorFeedbackHealth *health) {
 if(health)memset(health,0,sizeof(*health));
 return MotorCAN_GetFeedback(id,f);
}
#endif
#ifndef TEST_JOINT_CALIBRATION_OVERRIDE
void JointCalibration_Apply(MotorFeedback *f) {
 f->joint_calibrated=0;
 for(unsigned i=0;i<4;++i)if(g_joint_calibration[i].motor_id==f->motor_id) {
  const JointCalibration *c=&g_joint_calibration[i];
  f->joint_angle_rad=c->model_reference_rad+c->direction*(f->angle_rad-c->encoder_reference_rad)/c->ratio;
  f->joint_speed_rad_s=c->direction*f->speed_rad_s/c->ratio;
  f->joint_calibrated=1;break;
 }
}
#endif
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,FDCAN_TxHeaderTypeDef *header,uint8_t *bytes) {
 assert(h==&hfdcan1);++calls;
 assert(header->Identifier==0x100 || header->Identifier==0x200);
 assert(header->IdType==FDCAN_STANDARD_ID && header->TxFrameType==FDCAN_DATA_FRAME);
 assert(header->DataLength==FDCAN_DLC_BYTES_8 && header->ErrorStateIndicator==FDCAN_ESI_ACTIVE);
 assert(header->BitRateSwitch==FDCAN_BRS_OFF && header->FDFormat==FDCAN_CLASSIC_CAN);
 assert(header->TxEventFifoControl==FDCAN_NO_TX_EVENTS);
 if(calls==fail_at) return 1;
 memcpy(data[header->Identifier==0x200],bytes,8);return HAL_OK;
}
static void step(void) {now+=5;WheelLeg_Update();assert(!critical);}
static void zero_output(const WheelLegControlOutput *o) {
 for(unsigned s=0;s<2;s++) {
  assert(o->wheel_torque_nm[s]==0 && o->pitch_torque_nm[s]==0);
  assert(o->joint_torque_nm[s][0]==0 && o->joint_torque_nm[s][1]==0);
 }
}
static void check_joint_commands(void) {
 float ratio[4];for(unsigned i=0;i<4;i++) ratio[i]=g_dual_hold.torque_ratio_nm_per_v[i];
 float supply=g_wheel_output.supply_voltage_v;
 float effective=supply<8?12:supply,output_ratio=(12-effective)/10+.7f;
 assert(fabsf(g_joint_output.output_ratio-output_ratio)<1e-6f);
 assert(g_joint_output.effective_supply_voltage_v==effective);
 unsigned saturated=0;
 for(unsigned i=0;i<4;i++) {
  const JointCalibration *cal=&g_joint_calibration[i];
  assert(g_wheel_leg.joint_feedback_valid[i]);
  assert(g_wheel_leg.joint_feedback_age_ms[i]==motors[cal->motor_id-1].age_ms);
  assert(g_wheel_leg.joint_angle_rad[i]==motors[cal->motor_id-1].joint_angle_rad);
  assert(g_wheel_leg.joint_shaft_speed_rad_s[i]==motors[cal->motor_id-1].speed_rad_s);
  float motor_request=g_wheel_leg.leg[i/2].jacobian[1-i%2]*g_dual_hold.force_n[i/2]+
      g_wheel_leg.leg[i/2].jacobian[3-i%2]*g_dual_hold.pitch_torque_nm[i/2];
  if(fabsf(motor_request)>JOINT_TORQUE_LIMIT_NM) saturated=1;
  assert(fabsf(g_dual_hold.joint_torque_nm[i])<=JOINT_TORQUE_LIMIT_NM+1e-6f);
  float raw=motors[cal->motor_id-1].joint_speed_rad_s*cal->ratio*cal->direction;
  float s=fabsf(raw);
  float emf=g_dual_hold.back_emf_ratio*(.00008f*s*s*s-.0035f*s*s+.2322f*s)*1000;
  if(raw<0) emf=-emf;
  /* Independent expectation: back-EMF is in raw shaft voltage coordinates;
   * flipping torque polarity must not flip the final speed compensation. */
  float torque_mv=g_dual_hold.joint_torque_nm[i]/ratio[i]*output_ratio*1000;
  float raw_request=g_dual_hold.direction[i]*torque_mv+emf;
  float limit=g_dual_hold.voltage_limit_mv;
  /* Independent arithmetic below checks the request within 0.02 mV. At
   * exactly the clamp, rounding order can straddle it: classify the final
   * published float used by the actuator rather than an equivalent reorder. */
  if(fabsf(g_joint_output.requested_mv[i])>limit) saturated=1;
  float clamped=fmaxf(-limit,fminf(limit,raw_request));
  int actual=g_dual_hold.voltage_mv[cal->motor_id-1];
  assert(abs(actual-(int)clamped)<=1);
  assert(actual==g_joint_output.voltage_mv[i]);
  assert(fabsf(g_joint_output.torque_voltage_mv[i]-torque_mv)<.02f);
  assert(fabsf(g_joint_output.back_emf_mv[i]-g_dual_hold.direction[i]*emf)<.02f);
  assert(fabsf(g_joint_output.requested_mv[i]-g_dual_hold.direction[i]*raw_request)<.02f);
  assert(fabsf(g_joint_output.speed_rad_s[i]-g_dual_hold.direction[i]*raw)<1e-5f);
  unsigned slot=cal->motor_id-1;
  uint16_t encoded=(uint16_t)data[slot/4][2*(slot%4)]|
                   ((uint16_t)data[slot/4][2*(slot%4)+1]<<8);
  assert(encoded==(uint16_t)actual);
 }
 assert(g_joint_output.saturated==saturated);
}
#if MOTOR_COMMAND_TX_ENABLED
static void axial_feedback(float length,float velocity) {
 float q[2];LegModel model;
 assert(leg_inverse(.03f,length,1.57f,1.57f,q));
 assert(leg_model(q[0],q[1],LEG_BRANCH_DOWN,&model));
 float det=model.jacobian[0]*model.jacobian[3]-model.jacobian[1]*model.jacobian[2];
 assert(fabsf(det)>1e-6f);
 for(unsigned side=0;side<2;side++) {
  MotorFeedback *front=&motors[g_joint_calibration[2*side].motor_id-1];
  MotorFeedback *rear=&motors[g_joint_calibration[2*side+1].motor_id-1];
  front->joint_angle_rad=q[1];rear->joint_angle_rad=q[0];
  front->joint_speed_rad_s=-model.jacobian[2]*velocity/det;
  rear->joint_speed_rad_s=model.jacobian[3]*velocity/det;
 }
}
typedef struct {float undershoot, tail_error, tail_speed;unsigned faults;} AxialResponse;
static void swing_feedback(unsigned side,float length,float angle,float speed) {
 float q[2];LegModel m;
 assert(leg_inverse(.03f+length*cosf(angle),length*sinf(angle),1.57f,1.57f,q));
 assert(leg_model(q[0],q[1],LEG_BRANCH_DOWN,&m));
 float det=m.jacobian[0]*m.jacobian[3]-m.jacobian[1]*m.jacobian[2];
 MotorFeedback *front=&motors[g_joint_calibration[2*side].motor_id-1];
 MotorFeedback *rear=&motors[g_joint_calibration[2*side+1].motor_id-1];
 rear->joint_angle_rad=q[0];front->joint_angle_rad=q[1];
 rear->joint_speed_rad_s=-m.jacobian[1]*speed/det;
 front->joint_speed_rad_s=m.jacobian[0]*speed/det;
}
static void verify_load_adaptation(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;
 const ImuSample saved_imu=imu;
 const unsigned saved_imu_ok=imu_ok;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 assert(defaults.load_enable==0 && defaults.load_ki_n_per_m_s==20);
 assert(defaults.load_limit_n==2 && defaults.load_rate_n_s==.25f);
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;step();
 g_robot_control=(RobotControlDebug){0};
 g_balance=(BalanceDebug){0};
 imu=(ImuSample){0};imu_ok=1;
 g_dual_hold=defaults;g_dual_hold.enable=1;
 g_dual_hold.load_enable=1;
 g_dual_hold.feedforward_rate_n_s=100000;
 g_dual_hold.overlength_startup_s=0;
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
 g_dual_hold.sync_ratio=0;
 axial_feedback(.065f,0);step();
 assert(g_dual_hold.load_force_n[0]==0);
 float previous=0;
 for(unsigned n=0;n<600;n++) {
  step();float now_load=g_dual_hold.load_force_n[0];
  assert(g_dual_hold.status==1 && !g_dual_hold.load_frozen[0]);
  assert(now_load>=previous && now_load-previous<=.001251f);
  assert(fabsf(now_load-g_dual_hold.load_force_n[1])<1e-5f);
  previous=now_load;
 }
 assert(previous>.23f && previous<.25f);
 assert(g_dual_hold.force_n[0]>defaults.feedforward_target_n+previous-.01f);check_joint_commands();
 axial_feedback(.07f,0);step();float held=g_dual_hold.load_force_n[0];
 for(unsigned n=0;n<100;n++) step();
 assert(fabsf(g_dual_hold.load_force_n[0]-held)<1e-5f);
 axial_feedback(.074f,0);
 for(unsigned n=0;n<200;n++) step();
 assert(g_dual_hold.load_force_n[0]<held-.05f);

 /* Existing saturation freezes learning, including zero support floor. */
 g_dual_hold.voltage_limit_mv=800;step();held=g_dual_hold.load_force_n[0];
 for(unsigned n=0;n<20;n++) step();
 assert(g_dual_hold.force_saturated[0] && g_dual_hold.load_frozen[0]);
 assert(g_dual_hold.load_force_n[0]==held);
 g_dual_hold.voltage_limit_mv=6900;
 axial_feedback(.11f,0);step();
 assert(g_dual_hold.force_n[0]==0 && g_dual_hold.load_frozen[0]);
 assert(g_dual_hold.load_force_n[0]==held);

 /* Proposed next force must fit even before actual output is saturated. */
 axial_feedback(.065f,0);imu_ok=0;for(unsigned n=0;n<30;n++) step();
 assert(g_dual_hold.status==1 && g_dual_hold.load_frozen[0]);
 g_dual_hold.max_force_n=g_dual_hold.force_requested_n[0]+.0001f;
 imu_ok=1;step();
 assert(!g_dual_hold.force_saturated[0] && g_dual_hold.load_frozen[0]);
 assert(g_dual_hold.load_force_n[0]==held);
 g_dual_hold.max_force_n=defaults.max_force_n;

 /* Rate/offset ceilings and motion/pose gates. */
 g_dual_hold.load_enable=0;step();assert(g_dual_hold.load_force_n[0]==0);
 g_dual_hold.load_enable=1;g_dual_hold.load_limit_n=.02f;
 g_dual_hold.load_ki_n_per_m_s=100;
 axial_feedback(.055f,0);previous=0;
 for(unsigned n=0;n<40;n++) {
  step();float f=g_dual_hold.load_force_n[0];
  assert(f<=.020001f && f-previous<=.001251f);previous=f;
 }
 assert(fabsf(previous-.02f)<1e-6f);
 axial_feedback(.065f,.03f);step();
 assert(g_dual_hold.load_frozen[0] && g_dual_hold.load_force_n[0]==previous);
 axial_feedback(.065f,0);imu.attitude_deg[BODY_PITCH_AXIS]=10;step();
 assert(g_dual_hold.load_frozen[0]);imu.attitude_deg[BODY_PITCH_AXIS]=0;

 /* Manual stop, TX/feedback faults and wheel takeover clear correction. */
 g_dual_hold.enable=0;step();assert(g_dual_hold.load_force_n[0]==0);
 g_dual_hold.enable=1;for(unsigned n=0;n<40;n++) step();
 assert(g_dual_hold.load_force_n[0]>0);
 fail_at=calls+1;step();
 assert(g_dual_hold.status==4 && g_dual_hold.load_force_n[0]==0);fail_at=0;
 g_dual_hold.enable=1;for(unsigned n=0;n<40;n++) step();
 motors[JOINT_LF_CAN_ID-1].online=0;step();
 assert(g_dual_hold.status==2 && g_dual_hold.load_force_n[0]==0);
 motors[JOINT_LF_CAN_ID-1].online=1;
 g_dual_hold.enable=1;for(unsigned n=0;n<40;n++) step();
 g_wheel_test.enable=1;step();assert(g_dual_hold.load_force_n[0]==0);
 g_wheel_test.enable=0;for(unsigned n=0;n<4;n++) step();

 /* Separate load-learning leg envelope accepts this trace's 8.7 deg pose,
  * freezes outside 10 deg, and preserves the stricter 5 deg BODY gate. */
 for(unsigned mirror=0;mirror<2;++mirror) {
  g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;step();
  g_robot_control=(RobotControlDebug){0};g_balance=(BalanceDebug){0};
  g_dual_hold=defaults;g_dual_hold.enable=g_dual_hold.load_enable=1;
  g_dual_hold.feedforward_rate_n_s=100000;g_dual_hold.sync_ratio=0;
  g_dual_hold.overlength_startup_s=0;g_dual_hold.feedforward_target_n=5;
  g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
  imu=(ImuSample){0};imu_ok=1;
  float sign=mirror?-1:1;
  for(unsigned side=0;side<2;++side)
   swing_feedback(side,defaults.target_m-.005f,1.57079632679f+sign*.151843645f,0);
  for(unsigned n=0;n<30;++n)step();
  assert(g_dual_hold.load_force_n[0]>0 && !g_dual_hold.load_frozen[0]);
  held=g_dual_hold.load_force_n[0];
  for(unsigned side=0;side<2;++side)
   swing_feedback(side,defaults.target_m-.005f,1.57079632679f+sign*.191986218f,0);
  step();held=g_dual_hold.load_force_n[0];assert(g_dual_hold.load_frozen[0]);
  for(unsigned n=0;n<5;++n)step();
  assert(g_dual_hold.load_frozen[0] && g_dual_hold.load_force_n[0]==held);
  for(unsigned side=0;side<2;++side)
   swing_feedback(side,defaults.target_m-.005f,1.57079632679f,0);
  imu.attitude_deg[BODY_PITCH_AXIS]=6;
  step();held=g_dual_hold.load_force_n[0];assert(g_dual_hold.load_frozen[0]);
  for(unsigned n=0;n<5;++n)step();
  assert(g_dual_hold.load_frozen[0] && g_dual_hold.load_force_n[0]==held);
 }
 /* Reject nonfinite/oversized settings and simultaneous legacy integration. */
 g_dual_hold=defaults;g_dual_hold.load_enable=1;g_dual_hold.speed_ki=1;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 g_dual_hold.speed_ki=0;g_dual_hold.load_ki_n_per_m_s=NAN;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3);
 g_dual_hold.load_ki_n_per_m_s=20;g_dual_hold.load_rate_n_s=1.01f;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3);
 /* Master ramp must not learn load; current wheel saturation
  * freezes it independently of the body-pose gate. */
 g_dual_hold=defaults;g_dual_hold.load_enable=1;
 g_dual_hold.feedforward_rate_n_s=100000;g_dual_hold.overlength_startup_s=0;
 g_dual_hold.sync_ratio=0;
 g_balance=(BalanceDebug){.wheel_rate_scale=1.0f,.wheel_scale=.8f,.leg_scale=BALANCE_DEFAULT_LEG_SCALE,.leg_hold_ratio=1,.wheel_limit_mv=6000};
 g_robot_control=(RobotControlDebug){.enable=1,.startup_enable=1};
 axial_feedback(.065f,0);imu=(ImuSample){0};imu_ok=1;
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 for(unsigned n=0;n<20;n++) {
  step();assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
  assert(g_dual_hold.load_force_n[0]==0 && g_dual_hold.load_frozen[0]);
 }
 for(unsigned n=0;n<200;n++) step();
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_dual_hold.load_force_n[0]>0);
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-80;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=80;step();
 assert(g_wheel_output.saturated && g_dual_hold.load_frozen[0]);held=g_dual_hold.load_force_n[0];
 step();assert(g_dual_hold.load_force_n[0]==held);
 g_robot_control.enable=0;step();assert(g_dual_hold.load_force_n[0]==0);
 g_dual_hold=saved_hold;g_dual_hold.enable=0;g_balance.enable=0;step();
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;
 imu=saved_imu;imu_ok=saved_imu_ok;memcpy(motors,saved_motors,sizeof(motors));step();
 puts("PASS: optional bounded load integral, hold/unload, prediction/saturation gates, motion/pose gates, stop/fault/test reset; no measured weight/current model");
}
static void verify_pitch_trim_position_window(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;
 const ImuSample saved_imu=imu;
 const unsigned saved_imu_ok=imu_ok;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 const float half_pi=1.57079632679f,rad=.017453292519943295f;
 const BalanceDebug balance_defaults={.wheel_scale=.8f,.leg_scale=BALANCE_DEFAULT_LEG_SCALE,.leg_hold_ratio=1,
  .wheel_limit_mv=6000,.position_window_m=.1f};
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;
 for(unsigned n=0;n<4;n++) step();
 g_robot_control=(RobotControlDebug){.enable=1};
 g_dual_hold=defaults;g_dual_hold.feedforward_rate_n_s=100000;
 g_balance=balance_defaults;imu=(ImuSample){0};imu_ok=1;
 for(unsigned i=0;i<8;i++) motors[i].speed_rad_s=motors[i].joint_speed_rad_s=0;
 for(unsigned side=0;side<2;side++) swing_feedback(side,.07f,half_pi,0);
 step();assert(g_robot_control.status==1);
 float old_target=g_balance.reference[2];
 /* Feed actual encoder rates, so the production odometry advances. The
  * target stays fixed for small moves, follows beyond the window in BOTH
  * directions, and does not reset at an ordinary velocity sign change. */
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-8;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=8;
 for(unsigned n=0;n<600;n++) {
  step();assert(g_robot_control.status==1);
  assert(fabsf(g_balance.state[2]-g_balance.reference[2])<=.100001f);
  if(n<50) assert(g_balance.reference[2]==old_target);
 }
 assert(g_balance.state[2]-old_target>.6f);
 assert(g_balance.reference[2]>old_target+.5f);
 float K[12];lqr_k(.07f,K);
 assert(fabsf(g_balance.wheel_torque_component_nm[2]+K[4]*.1f*.8f)<1e-6f);
 float capped_position_torque=fabsf(g_balance.wheel_torque_component_nm[2]);
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=8;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=-8;
 for(unsigned n=0;n<600;n++) {
  step();assert(fabsf(g_balance.state[2]-g_balance.reference[2])<=.100001f);
 }
 assert(g_balance.state[2]-g_balance.reference[2]<-.09999f);
 /* Zero selects fixed parking after normal-balance entry. */
 g_robot_control.enable=0;step();g_balance.position_window_m=0;
 g_robot_control.enable=1;step();old_target=g_balance.reference[2];
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-8;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=8;
 for(unsigned n=0;n<600;n++) {step();assert(g_balance.reference[2]==old_target);}
 assert(fabsf(g_balance.state[2]-old_target)>.6f);
 assert(fabsf(g_balance.wheel_torque_component_nm[2])>5.9f*capped_position_torque);

 /* Nonzero trim is deferred throughout the short RAMP, then slews in normal
  * balance. The measured IMU angle remains unmodified; theta stays world
  * vertical while BOTH local leg targets and phi reference are coordinated. */
 g_robot_control.enable=0;step();
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 g_balance=balance_defaults;g_balance.pitch_trim_deg=.8f; /* Explicit nonzero trim ramp fixture. */
 g_robot_control=(RobotControlDebug){.enable=1,.startup_enable=1};
 for(unsigned n=0;n<20;n++) {
  step();assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
  assert(g_balance.pitch_trim_applied_deg==0 && g_balance.reference[4]==0);
 }
 unsigned saw_ramp=0;
 for(unsigned n=0;n<300 && g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE;n++) {
  step();assert(g_robot_control.status==1);
  if(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP) saw_ramp=1;
  if(g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE) assert(g_balance.pitch_trim_applied_deg==0);
 }
 assert(saw_ramp && g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
 const float trim_trials[]={BALANCE_DEFAULT_PITCH_TRIM_DEG,.8f,-.8f};
 for(unsigned trial=0;trial<3;trial++) {
  g_balance.pitch_trim_deg=trim_trials[trial];
  float previous=g_balance.pitch_trim_applied_deg;
  /* Allow the configured preset to finish at the specified slew rate.
   * The historical fixed 3.3 s window only covered trims below 1.65 deg. */
  unsigned trim_steps=(unsigned)ceilf(fabsf(g_balance.pitch_trim_deg-previous)/
                                    (BALANCE_PITCH_TRIM_RATE_DEG_S*.005f))+4U;
  for(unsigned n=0;n<trim_steps;n++) {
   float b=g_balance.pitch_trim_applied_deg;
   imu.attitude_deg[BODY_PITCH_AXIS]=b/BODY_PITCH_DIRECTION;
   for(unsigned side=0;side<2;side++) swing_feedback(side,.07f,half_pi-b*rad,0);
   step();assert(g_robot_control.status==1);
   float applied=g_balance.pitch_trim_applied_deg;
   assert(fabsf(applied-previous)<=.002501f);previous=applied;
   assert(fabsf(g_balance.reference[4]+applied*rad)<1e-7f);
   for(unsigned side=0;side<2;side++)
    assert(fabsf(g_dual_hold.angle_reference_rad[side]-(half_pi-applied*rad))<1e-6f);
   check_joint_commands();
  }
  assert(g_balance.pitch_trim_applied_deg==g_balance.pitch_trim_deg);
  assert(fabsf(g_wheel_leg.body_pitch_rad-g_balance.pitch_trim_deg*rad)<1e-7f);
  assert(fabsf(g_balance.state[0])<1e-6f);
  assert(fabsf(g_balance.torque_nm[0])<1e-6f && fabsf(g_balance.torque_nm[1])<1e-6f);
  assert(fabsf(g_dual_hold.angle_hold_torque_nm[0])<1e-6f);
  assert(fabsf(g_dual_hold.angle_hold_torque_nm[1])<1e-6f);
 }
 /* Manual stop, feedback failure, TX failure and wheel takeover immediately
  * clear the applied trim; its user setting remains for a slow fresh rearm. */
 g_robot_control.enable=0;step();assert(g_balance.pitch_trim_applied_deg==0);
 assert(g_balance.pitch_trim_deg==-.8f);
 g_robot_control.startup_enable=0;g_robot_control.enable=1;step();
 assert(fabsf(g_balance.pitch_trim_applied_deg+.0025f)<1e-7f);
 motors[JOINT_LF_CAN_ID-1].online=0;step();
 assert(!g_robot_control.enable && g_balance.pitch_trim_applied_deg==0);
 motors[JOINT_LF_CAN_ID-1].online=1;
 g_robot_control.enable=1;for(unsigned n=0;n<20;n++) step();
 assert(g_balance.pitch_trim_applied_deg<-.04f);
 fail_at=calls+1;step();fail_at=0;
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_TX && g_balance.pitch_trim_applied_deg==0);
 g_robot_control.enable=1;for(unsigned n=0;n<20;n++) step();
 g_wheel_test.enable=1;step();assert(g_balance.pitch_trim_applied_deg==0);
 g_wheel_test.enable=0;for(unsigned n=0;n<4;n++) step();
 /* Invalid trim/window and incompatible pose settings fail closed. */
 const float bad_windows[]={-.01f,.5001f,NAN,INFINITY};
 for(unsigned i=0;i<4;i++) {
  g_balance=balance_defaults;g_balance.position_window_m=bad_windows[i];
  g_robot_control.enable=1;step();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
  for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
 }
 const float bad_trims[]={-10.001f,10.001f,NAN,INFINITY};
 for(unsigned i=0;i<4;i++) {
  g_balance=balance_defaults;g_balance.pitch_trim_deg=bad_trims[i];
  g_robot_control.enable=1;step();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
  assert(g_balance.pitch_trim_applied_deg==0);
 }
 g_balance=balance_defaults;g_balance.pitch_trim_deg=.2f;
 g_dual_hold.angle_reference_mode=LEG_ANGLE_CAPTURE;
 g_robot_control.enable=1;step();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 g_dual_hold.angle_reference_mode=LEG_ANGLE_NEUTRAL;g_dual_hold.angle_target_rad=half_pi+.02f;
 g_robot_control.enable=1;step();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;step();
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;
 imu=saved_imu;imu_ok=saved_imu_ok;memcpy(motors,saved_motors,sizeof(motors));step();
 puts("PASS: moving position window, fixed parking, coordinated +/-pitch trim, slew/startup gates, unchanged IMU, stop/fault/test reset; command checks only");
}
static void verify_swing_sync(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const ImuSample saved_imu=imu;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 assert(defaults.sync_ratio==LEG_DEFAULT_SYNC_RATIO && defaults.sync_position_kp==12 &&
        defaults.sync_speed_kp==LEG_DEFAULT_SYNC_SPEED_KP);
 assert(defaults.sync_limit_nm==LEG_DEFAULT_SYNC_LIMIT_NM &&
        defaults.sync_rate_nm_s==LEG_DEFAULT_SYNC_RATE_NM_S);
 const float sync_cap=defaults.sync_ratio*defaults.sync_limit_nm;
 const float sync_step=defaults.sync_rate_nm_s*.005f;
 g_dual_hold.enable=0;step();
 g_dual_hold=defaults;g_dual_hold.angle_reference_mode=LEG_ANGLE_CAPTURE;
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
 g_dual_hold.feedforward_rate_n_s=100000;
 g_balance.wheel_scale=g_balance.leg_scale=g_balance.back_emf_ratio=0;
 g_balance.leg_hold_ratio=0;
 for(unsigned i=0;i<8;i++) motors[i].speed_rad_s=motors[i].joint_speed_rad_s=0;
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=imu.attitude_deg[i]=0;
 /* User's geometry screenshot with left wheel ahead (enable not shown). Replay the
  * measured geometry with stationary feedback and a LEVEL body assumption;
  * this checks commands only, never hardware motion or actual IMU attitude. */
 g_dual_hold=defaults;
 g_dual_hold.sync_ratio=0; /* Recorded pre-sync baseline remains an explicit comparison. */
 g_balance.wheel_scale=.8f;g_balance.leg_scale=.2f;g_balance.leg_hold_ratio=1;
 swing_feedback(0,.084855267f,.984336078f,0);
 swing_feedback(1,.119260348f,1.98686242f,0);
 g_robot_control.enable=1;
 for(unsigned n=0;n<1000;n++) {step();assert(g_robot_control.enable && g_robot_control.status==1);}
 float front_offset[2];
 for(unsigned side=0;side<2;side++) {
  front_offset[side]=g_wheel_leg.leg[side].length*cosf(g_wheel_leg.leg[side].angle);
  assert(g_dual_hold.sync_torque_nm[side]==0);
 }
 assert(front_offset[0]>.0469f && front_offset[1]<-.0481f);
 assert(fabsf(front_offset[0]-front_offset[1]-.0951612f)<1e-5f);
 assert(fabsf(g_dual_hold.angle_hold_torque_nm[0]-defaults.angle_limit_nm)<1e-6f);
 assert(fabsf(g_dual_hold.angle_hold_torque_nm[1]+defaults.angle_limit_nm)<1e-6f);
 assert(g_dual_hold.force_n[0]>0 && g_dual_hold.force_n[1]==0);
 printf("PASS: recorded split pose offset=%g mm, stationary default axial=%g/%g N, local swing=%g/%g Nm; assumed level body, commands only\n",
        1000*(front_offset[0]-front_offset[1]),g_dual_hold.force_n[0],g_dual_hold.force_n[1],
        g_dual_hold.angle_hold_torque_nm[0],g_dual_hold.angle_hold_torque_nm[1]);
 g_robot_control.enable=0;step();
 /* Explicit ratio=0 retains the pre-sync local PD + LQR rollback behavior. */
 g_dual_hold=defaults;g_dual_hold.feedforward_rate_n_s=100000;
 g_dual_hold.sync_ratio=0;
 g_balance.wheel_scale=.8f;g_balance.leg_scale=.2f;g_balance.leg_hold_ratio=1;
 swing_feedback(0,.07f,1.45079632679f,0);swing_feedback(1,.07f,1.69079632679f,0);
 g_robot_control.enable=1;
 for(unsigned n=0;n<150;n++) {
  step();assert(g_robot_control.enable && g_robot_control.status==1);check_joint_commands();
  for(unsigned side=0;side<2;side++) {
   assert(g_dual_hold.sync_torque_nm[side]==0);
   assert(g_dual_hold.pitch_torque_nm[side]==g_dual_hold.angle_hold_torque_nm[side]+g_dual_hold.lqr_pitch_torque_nm[side]);
  }
 }
 assert(fabsf(g_dual_hold.pitch_torque_nm[0]-.12f*defaults.angle_kp_nm_per_rad)<1e-6f &&
        fabsf(g_dual_hold.pitch_torque_nm[1]+.12f*defaults.angle_kp_nm_per_rad)<1e-6f);
 g_robot_control.enable=0;step();
 printf("PASS: explicit sync disable restores pre-sync PD/LQR outputs\n");
 /* Configured feedback opposes either split direction within its torque
  * and per-cycle slew bounds, including first activation.
  * Equal angles with separating rates also receive opposing damping. */
 for(unsigned trial=0;trial<5;trial++) {
  float difference=trial==0?-.24f:trial==1?.24f:trial==2?-.04f:trial==3?.04f:0;
  g_dual_hold=defaults;g_dual_hold.feedforward_rate_n_s=100000;
  for(unsigned side=0;side<2;side++)
   swing_feedback(side,side?.09f:.07f,1.57079632679f+(side?-.5f:.5f)*difference,
                  trial==4?(side?-.2f:.2f):0);
  g_robot_control.enable=1;
  float prev[2]={0,0};
  for(unsigned n=0;n<150;n++) {
   step();assert(g_robot_control.enable && g_robot_control.status==1);check_joint_commands();
   for(unsigned side=0;side<2;side++) {
    float applied=g_dual_hold.sync_torque_nm[side];
    float error=trial==4?(side?-.4f:.4f):(side?-difference:difference);
    assert(applied*error<0 && fabsf(applied)<=sync_cap+1e-6f);
    assert(fabsf(applied-prev[side])<=sync_step+1e-6f);prev[side]=applied;
    assert(fabsf(g_dual_hold.pitch_torque_nm[side]-g_dual_hold.angle_hold_torque_nm[side]-
                 g_dual_hold.lqr_pitch_torque_nm[side]-applied)<1e-6f);
   }
   assert(fabsf(g_balance.torque_nm[0])<1e-6f); /* Sync does not alter common wheel balance. */
  }
  if(trial==2 || trial==3) assert(fabsf(g_dual_hold.sync_torque_nm[1]/g_dual_hold.sync_torque_nm[0]+.09f/.07f)<1e-4f);
  if(trial==0 || trial==1) {
   float expected=defaults.sync_ratio*fminf(defaults.sync_limit_nm,
       defaults.sync_speed_kp*defaults.sync_position_kp*fabsf(difference));
   assert(fabsf(fabsf(g_dual_hold.sync_torque_nm[0])-expected)<1e-5f);
  }
  motors[JOINT_LF_CAN_ID-1].online=0;step();
  assert(!g_robot_control.enable && g_dual_hold.sync_torque_nm[0]==0 && g_dual_hold.sync_torque_nm[1]==0);
  motors[JOINT_LF_CAN_ID-1].online=1;step();assert(!g_robot_control.enable);
  g_robot_control.enable=1;step();assert(fabsf(g_dual_hold.sync_torque_nm[0])<=sync_step+1e-6f);
  g_robot_control.enable=0;step();assert(g_dual_hold.sync_torque_nm[0]==0);
 }
 printf("PASS: configured sync opposes both split/rate directions, per-leg <=%g Nm, first/rearm slew <=%g Nm, wheel mean independent and fault reset\n",sync_cap,sync_step);
 /* The remaining sync cases explicitly opt in; they are algorithm checks,
  * not evidence that these parameters produce stable hardware standing. */
 defaults.sync_ratio=1;
 defaults.sync_limit_nm=.1f; /* Fixed algorithm fixture; configured cap tested above. */
 defaults.sync_rate_nm_s=.2f; /* Explicit strong-algorithm fixture, not shipped defaults. */
 g_dual_hold=defaults;g_dual_hold.angle_reference_mode=LEG_ANGLE_CAPTURE;
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
 g_dual_hold.feedforward_rate_n_s=100000;
 g_dual_hold.overlength_startup_s=0; /* Fixed axial support isolates sync; this case is not a force-ramp test. */
 g_balance.wheel_scale=g_balance.leg_scale=0;g_balance.leg_hold_ratio=0;
 const float differences[]={-.24f,.24f,-.04f,.04f};
 for(unsigned trial=0;trial<4;trial++) {
  g_dual_hold.enable=0;step();g_dual_hold.sync_ratio=0;
  for(unsigned side=0;side<2;side++)
   swing_feedback(side,side?.09f:.07f,1.57079632679f+(side?-.5f:.5f)*differences[trial],0);
  g_dual_hold.enable=g_balance.enable=1;
  for(unsigned n=0;n<150;n++) step();
  const float baseline_force[2]={g_dual_hold.force_n[0],g_dual_hold.force_n[1]};
  float baseline_torque[4];for(unsigned j=0;j<4;j++) baseline_torque[j]=g_dual_hold.joint_torque_nm[j];
  assert(g_dual_hold.pitch_torque_nm[0]==0 && g_dual_hold.pitch_torque_nm[1]==0);
  g_dual_hold.sync_ratio=1;
  for(unsigned n=0;n<150;n++) {
   float previous[2]={g_dual_hold.sync_torque_nm[0],g_dual_hold.sync_torque_nm[1]};
   step();assert(g_dual_hold.status==1 && g_balance.status==1);check_joint_commands();
   for(unsigned side=0;side<2;side++) {
    float tp=g_dual_hold.sync_torque_nm[side];
    assert(tp*(side?-differences[trial]:differences[trial])<0);
    assert(fabsf(tp)<=.100001f && fabsf(tp-previous[side])<=.001001f);
    assert(fabsf(g_dual_hold.force_n[side]-baseline_force[side])<1e-6f);
    for(unsigned j=0;j<2;j++)
     assert(fabsf(g_dual_hold.joint_torque_nm[2*side+j]-baseline_torque[2*side+j]-
                  g_wheel_leg.leg[side].jacobian[3-j]*tp)<1e-6f);
   }
   assert(g_dual_hold.voltage_mv[2]==0 && g_dual_hold.voltage_mv[6]==0);
  }
  if(trial>=2) assert(fabsf(g_dual_hold.sync_torque_nm[1]/g_dual_hold.sync_torque_nm[0]+.09f/.07f)<1e-4f);
  /* Sync survives balance toggles and missing unused wheel/IMU feedback. */
  float tp=g_dual_hold.sync_torque_nm[0];g_balance.enable=0;step();
  assert(fabsf(g_dual_hold.sync_torque_nm[0]-tp)<1e-6f);
  motors[WHEEL_LEFT_CAN_ID-1].online=motors[WHEEL_RIGHT_CAN_ID-1].online=0;imu_ok=0;
  g_balance.enable=1;step();assert(g_dual_hold.status==1 && g_balance.status==1);
  assert(fabsf(g_dual_hold.sync_torque_nm[0]-tp)<1e-6f);
  motors[WHEEL_LEFT_CAN_ID-1].online=motors[WHEEL_RIGHT_CAN_ID-1].online=1;imu_ok=1;
  g_dual_hold.sync_ratio=0;step();assert(g_dual_hold.sync_torque_nm[0]==0 && g_dual_hold.sync_torque_nm[1]==0);
 }
 /* Explicit opt-in master control combines neutral PD, sync and shared LQR. The
  * symmetric split is invisible to the mean, but sync still corrects it. */
 g_dual_hold.enable=0;step();g_dual_hold=defaults;
 g_dual_hold.feedforward_rate_n_s=100000;
 g_balance.wheel_scale=.8f;g_balance.leg_scale=.2f;g_balance.leg_hold_ratio=1;
 swing_feedback(0,.07f,1.45079632679f,0);swing_feedback(1,.07f,1.69079632679f,0);
 g_robot_control.enable=1;
 for(unsigned n=0;n<150;n++) {step();assert(g_robot_control.enable && g_robot_control.status==1);}
 assert(fabsf(g_balance.torque_nm[0])<1e-6f && fabsf(g_balance.torque_nm[1])<1e-6f);
 assert(g_dual_hold.pitch_torque_nm[0]>.11f && g_dual_hold.pitch_torque_nm[1]<-.11f);
 for(unsigned side=0;side<2;side++)
  assert(fabsf(g_dual_hold.pitch_torque_nm[side]-g_dual_hold.angle_hold_torque_nm[side]-
              g_dual_hold.lqr_pitch_torque_nm[side]-g_dual_hold.sync_torque_nm[side])<1e-6f);
 g_robot_control.enable=0;step();assert(g_dual_hold.sync_torque_nm[0]==0);
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
 g_balance.wheel_scale=g_balance.leg_scale=0;
 /* Equal angles with separating velocities must be damped. */
 g_dual_hold.enable=0;step();g_dual_hold.sync_ratio=1;
 swing_feedback(0,.07f,1.57079632679f,.2f);swing_feedback(1,.07f,1.57079632679f,-.2f);
 g_dual_hold.enable=g_balance.enable=1;
 for(unsigned n=0;n<50;n++) step();
 assert(g_dual_hold.sync_torque_nm[0]<0 && g_dual_hold.sync_torque_nm[1]>0);
 /* Decode actual CAN voltage through J^T for a simple fixed-length angular
  * inertia/damping plant. This verifies physical work sign, not real stability. */
 float final_difference[2];
 for(unsigned mode=0;mode<2;mode++) {
  g_dual_hold.enable=0;step();g_dual_hold.sync_ratio=(float)mode;
  float angle[2]={1.57079632679f-.12f,1.57079632679f+.12f},speed[2]={0,0};
  for(unsigned n=0;n<600;n++) {
   for(unsigned side=0;side<2;side++) swing_feedback(side,.07f,angle[side],speed[side]);
   g_dual_hold.enable=g_balance.enable=1;step();assert(g_dual_hold.status==1);check_joint_commands();
   for(unsigned side=0;side<2;side++) {
    float tau[2];
    for(unsigned j=0;j<2;j++) {
     unsigned index=2*side+1-j,id=g_joint_calibration[index].motor_id-1;
     uint16_t packed=(uint16_t)data[id/4][2*(id%4)]|((uint16_t)data[id/4][2*(id%4)+1]<<8);
     tau[j]=(int16_t)packed*g_dual_hold.direction[index]/g_joint_output.torque_mv_per_nm[index];
    }
    const float *J=(const float *)g_wheel_leg.leg[side].jacobian;
    float actual_tp=(J[0]*tau[1]-J[1]*tau[0])/(J[0]*J[3]-J[1]*J[2]);
    speed[side]+=.005f*(actual_tp-.02f*speed[side])/.002f;
    angle[side]+=.005f*speed[side];
   }
  }
  final_difference[mode]=fabsf(angle[0]-angle[1]);
 }
 assert(final_difference[0]>.2f && final_difference[1]<.005f);
 printf("PASS: decoded-CAN ideal angular plant mismatch %g -> %g rad; sync direction, damping, slew, length scaling and support isolation\n",
        final_difference[0],final_difference[1]);
 g_dual_hold.enable=0;step();assert(g_dual_hold.sync_torque_nm[0]==0 && g_dual_hold.sync_torque_nm[1]==0);
 /* Invalid new parameters disarm with zero command and controller state. */
 float *fields[]={ (float *)&g_dual_hold.sync_ratio,(float *)&g_dual_hold.sync_position_kp,
  (float *)&g_dual_hold.sync_speed_kp,(float *)&g_dual_hold.sync_max_speed_rad_s,
  (float *)&g_dual_hold.sync_filter_tau_s,(float *)&g_dual_hold.sync_limit_nm,(float *)&g_dual_hold.sync_rate_nm_s};
 const float upper[]={1,50,.2f,20,.1f,.3f,2};
 const float good[]={1,12,.04f,20,.004f,.1f,.2f};
 for(unsigned f=0;f<7;f++) {
  const float bad[]={NAN,INFINITY,-.01f,upper[f]+.01f};
  for(unsigned b=0;b<4;b++) {
   *fields[f]=bad[b];g_dual_hold.enable=g_balance.enable=1;step();
   assert(!g_dual_hold.enable && g_dual_hold.status==3);
   for(unsigned id=0;id<8;id++) assert(g_dual_hold.voltage_mv[id]==0);
   assert(g_dual_hold.sync_torque_nm[0]==0 && g_dual_hold.sync_torque_nm[1]==0);
  }
  *fields[f]=good[f];
 }
 /* Fresh rearm starts at the first 1 mNm step after a feedback fault. */
 swing_feedback(0,.07f,1.45079632679f,0);swing_feedback(1,.07f,1.69079632679f,0);
 g_dual_hold.enable=g_balance.enable=1;step();assert(fabsf(g_dual_hold.sync_torque_nm[0])<=.001001f);
 motors[JOINT_LF_CAN_ID-1].online=0;step();assert(!g_dual_hold.enable && g_dual_hold.sync_torque_nm[0]==0);
 motors[JOINT_LF_CAN_ID-1].online=1;g_dual_hold.enable=g_balance.enable=1;step();
 assert(fabsf(g_dual_hold.sync_torque_nm[0])<=.001001f);
 /* CAN failure and wheel-test takeover clear sync; no automatic restart. */
 fail_at=calls+1;step();assert(!g_dual_hold.enable && g_dual_hold.sync_torque_nm[0]==0);fail_at=0;
 step();g_dual_hold.enable=g_balance.enable=1;step();
 g_wheel_test.enable=1;step();assert(!g_dual_hold.enable && g_dual_hold.sync_torque_nm[0]==0);
 g_wheel_test.enable=0;for(unsigned n=0;n<4;n++) step();
 g_dual_hold=saved_hold;g_balance=saved_balance;imu=saved_imu;
 memcpy(motors,saved_motors,sizeof(motors));
}
static void verify_sync_recovery_priority(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;
 const ImuSample saved_imu=imu;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 const float rad=.017453292519943295f,neutral=1.57079632679f;
 const float step_limit=defaults.sync_rate_nm_s*.005f;
 assert(defaults.sync_recovery_priority_enable==1);
 for(unsigned mirror=0;mirror<2;mirror++) {
  int sign=mirror?-1:1;
  float old_force[2]={0,0},old_base[2]={0,0},old_wheel=0;
  for(unsigned priority=0;priority<2;priority++) {
   g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;step();
   g_robot_control=(RobotControlDebug){.enable=1};
   g_dual_hold=defaults;g_dual_hold.sync_recovery_priority_enable=priority;
   g_dual_hold.feedforward_rate_n_s=100000;
   g_balance=(BalanceDebug){.wheel_scale=.5f,.wheel_rate_scale=.5f,.leg_scale=.3f,
      .leg_hold_ratio=1,.wheel_limit_mv=6000,.parking_correction_limit_m=.1f};
   imu=(ImuSample){0};imu_ok=1;imu.attitude_deg[BODY_PITCH_AXIS]=sign*11.0f/BODY_PITCH_DIRECTION;
   for(unsigned i=0;i<8;i++) {
    motors[i].online=1;motors[i].speed_rad_s=motors[i].joint_speed_rad_s=0;
   }
   swing_feedback(0,.07f,neutral-sign*16*rad,0);
   swing_feedback(1,.09f,neutral-sign*6*rad,0);
   float previous_raw[2]={0,0};
   for(unsigned n=0;n<350;n++) {
    step();assert(g_robot_control.enable && g_dual_hold.status==1);check_joint_commands();
    for(unsigned side=0;side<2;side++) {
     float raw=g_dual_hold.sync_raw_torque_nm[side];
     assert(fabsf(raw-previous_raw[side])<=step_limit+1e-6f);previous_raw[side]=raw;
     assert(fabsf(g_dual_hold.sync_torque_nm[side]-raw*g_dual_hold.sync_priority_scale)<1e-6f);
     assert(fabsf(g_dual_hold.pitch_torque_nm[side]-g_dual_hold.angle_hold_torque_nm[side]-
        g_dual_hold.lqr_pitch_torque_nm[side]-g_dual_hold.sync_torque_nm[side])<1e-6f);
     if(priority) {
      float base=g_dual_hold.angle_hold_torque_nm[side]+g_dual_hold.lqr_pitch_torque_nm[side];
      if(base*sign>0) assert(g_dual_hold.pitch_torque_nm[side]*sign>=.5f*base*sign-1e-6f);
     }
    }
   }
   if(!priority) {
    assert(g_dual_hold.pitch_torque_nm[1]*sign<0 && !g_dual_hold.sync_priority_active);
    for(unsigned side=0;side<2;side++) {
     old_force[side]=g_dual_hold.force_n[side];
     old_base[side]=g_dual_hold.angle_hold_torque_nm[side]+g_dual_hold.lqr_pitch_torque_nm[side];
    }
    old_wheel=g_balance.torque_nm[0];
   } else {
    assert(g_dual_hold.pitch_torque_nm[1]*sign>0 && g_dual_hold.sync_priority_active);
    assert(g_dual_hold.sync_priority_scale>0 && g_dual_hold.sync_priority_scale<1);
    assert(fabsf(g_dual_hold.sync_torque_nm[1]/g_dual_hold.sync_torque_nm[0]+.09f/.07f)<1e-4f);
    assert(fabsf(g_balance.torque_nm[0]-old_wheel)<1e-6f);
    for(unsigned side=0;side<2;side++) {
     assert(fabsf(g_dual_hold.force_n[side]-old_force[side])<1e-6f);
     assert(fabsf(g_dual_hold.angle_hold_torque_nm[side]+g_dual_hold.lqr_pitch_torque_nm[side]-old_base[side])<1e-6f);
    }
   }
  }
  /* Releasing priority restores the original sync with bounded command steps. */
  g_dual_hold.sync_recovery_priority_enable=0;
  for(unsigned n=0;n<150;n++) {
   float previous[2]={g_dual_hold.sync_torque_nm[0],g_dual_hold.sync_torque_nm[1]};
   step();
   for(unsigned side=0;side<2;side++) assert(fabsf(g_dual_hold.sync_torque_nm[side]-previous[side])<=step_limit+1e-6f);
  }
  assert(!g_dual_hold.sync_priority_active && g_dual_hold.sync_priority_scale==1);
  g_dual_hold.sync_recovery_priority_enable=1;step();assert(g_dual_hold.sync_priority_active);
  g_dual_hold.sync_ratio=0;step();
  assert(!g_dual_hold.sync_priority_active && g_dual_hold.sync_priority_scale==1 &&
         g_dual_hold.sync_torque_nm[0]==0 && g_dual_hold.sync_raw_torque_nm[0]==0);
  g_dual_hold.sync_ratio=defaults.sync_ratio;step();
  assert(fabsf(g_dual_hold.sync_raw_torque_nm[0])<=step_limit+1e-6f);
  motors[JOINT_LF_CAN_ID-1].online=0;step();
  assert(!g_robot_control.enable && !g_dual_hold.sync_priority_active && g_dual_hold.sync_priority_scale==1);
  for(unsigned side=0;side<2;side++) assert(g_dual_hold.sync_torque_nm[side]==0 && g_dual_hold.sync_raw_torque_nm[side]==0);
  motors[JOINT_LF_CAN_ID-1].online=1;g_robot_control.enable=1;step();
  assert(fabsf(g_dual_hold.sync_raw_torque_nm[0])<=step_limit+1e-6f);
  fail_at=calls+1;step();fail_at=0;
  assert(!g_robot_control.enable && !g_dual_hold.sync_priority_active && g_dual_hold.sync_priority_scale==1);
 }
 /* Opposite-sided errors still use the original differential controller. */
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;step();
 g_robot_control=(RobotControlDebug){.enable=1};g_dual_hold=defaults;
 g_balance=(BalanceDebug){.wheel_scale=.5f,.wheel_rate_scale=.5f,.leg_scale=.3f,.leg_hold_ratio=1,.wheel_limit_mv=6000};
 imu=(ImuSample){0};
 swing_feedback(0,.07f,neutral-16*rad,0);swing_feedback(1,.09f,neutral+16*rad,0);
 for(unsigned n=0;n<350;n++) {step();assert(!g_dual_hold.sync_priority_active && g_robot_control.enable);}
 assert(g_dual_hold.sync_priority_scale==1 && g_dual_hold.sync_torque_nm[0]>0 && g_dual_hold.sync_torque_nm[1]<0);
 /* Capture and pure-support modes retain their legacy composition. */
 for(unsigned mode=0;mode<2;mode++) {
  g_robot_control.enable=0;step();g_dual_hold=defaults;
  g_dual_hold.angle_reference_mode=mode?LEG_ANGLE_NEUTRAL:LEG_ANGLE_CAPTURE;
  g_balance.leg_scale=mode?0:.3f;
  imu.attitude_deg[BODY_PITCH_AXIS]=11.0f/BODY_PITCH_DIRECTION;
  swing_feedback(0,.07f,neutral-16*rad,0);swing_feedback(1,.09f,neutral-6*rad,0);
  g_robot_control.enable=1;
  for(unsigned n=0;n<350;n++) {step();assert(g_robot_control.enable && !g_dual_hold.sync_priority_active);}
  assert(g_dual_hold.sync_priority_scale==1);
 }
 g_robot_control.enable=0;step();g_dual_hold=defaults;g_dual_hold.sync_recovery_priority_enable=2;
 g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 for(unsigned id=0;id<8;id++) assert(g_dual_hold.voltage_mv[id]==0);
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;step();
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;imu=saved_imu;
 memcpy(motors,saved_motors,sizeof(motors));step();
 puts("PASS: common recovery retains >=50% base in both signs, differential length ratio, original wheel/support, bounded release, capture/opposite/pure-support bypass and OFF/feedback/TX resets");
}
static void check_startup_wheel_lqr(void) {
 float x[6],ref[6],u[2];
 for(unsigned i=0;i<6;i++) {x[i]=g_balance.state[i];ref[i]=g_balance.reference[i];}
 assert(lqr_control(g_balance.leg_length_m,x,ref,u));
 assert(fabsf(g_balance.torque_nm[0]-u[0]*g_balance.wheel_scale-g_balance.startup_wheel_damping_torque_nm)<1e-6f);
 float sum=g_balance.startup_wheel_damping_torque_nm;
 for(unsigned i=0;i<6;i++) sum+=g_balance.wheel_torque_component_nm[i];
 assert(fabsf(sum-g_balance.torque_nm[0])<1e-6f);
}
static void verify_staged_startup(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;
 const ImuSample saved_imu=imu;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;for(unsigned i=0;i<4;i++) step();
 g_robot_control.startup_enable=1;g_dual_hold=defaults;g_dual_hold.sync_ratio=0;
 g_balance=(BalanceDebug){.wheel_rate_scale=1.0f,.wheel_scale=.8f,.leg_scale=BALANCE_DEFAULT_LEG_SCALE,.leg_hold_ratio=1,
  .wheel_limit_mv=6000,.back_emf_ratio=1,.position_window_m=.1f,
  .startup_wheel_damping_nm_s_per_rad=.004f,.startup_wheel_limit_nm=.012f};
 for(unsigned i=0;i<8;i++) motors[i].speed_rad_s=motors[i].joint_speed_rad_s=0;
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=imu.attitude_deg[i]=0;
 /* Replay the old deadlock: split wheels, unequal/long legs, 10 deg body
  * lean, body motion, and support initially zero. Feedback remains valid. */
 swing_feedback(0,.084855267f,.984336078f,0);swing_feedback(1,.119260348f,1.98686242f,0);
 imu.attitude_deg[BODY_PITCH_AXIS]=-10;imu.gyro_rad_s[BODY_PITCH_AXIS]=.2f;
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-2;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=2;
 g_robot_control.enable=1;
 unsigned ramp_steps=0;float previous=0,previous_force=0;
 do {
  step();++ramp_steps;assert(g_robot_control.enable && g_robot_control.status==1);
  float blend=g_robot_control.startup_blend;
  assert(blend>previous && blend-previous<=.010001f);
  assert(g_robot_control.startup_settle_s==0 && g_robot_control.startup_ready_mask==0);
  assert(g_robot_control.startup_phase==(blend<1?ROBOT_STARTUP_RAMP:ROBOT_STARTUP_BALANCE));
  assert(fabsf(g_dual_hold.feedforward_n-previous_force-defaults.feedforward_rate_n_s*.005f)<1e-5f);
  assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_SUPPORT));
  float x[6],ref[6],u[2];
  for(unsigned i=0;i<6;i++) {x[i]=g_balance.state[i];ref[i]=g_balance.reference[i];}
  assert(lqr_control(g_balance.leg_length_m,x,ref,u));
  assert(fabsf(g_balance.torque_nm[1]-fmaxf(-BALANCE_LEG_LQR_TORQUE_LIMIT_NM,
       fminf(BALANCE_LEG_LQR_TORQUE_LIMIT_NM,u[1]*g_balance.leg_scale*blend)))<1e-6f);
  assert(fabsf(g_balance.torque_nm[1])>1e-5f);
  assert(g_dual_hold.lqr_pitch_torque_nm[0]==g_balance.torque_nm[1]);
  assert(fabsf(g_balance.startup_wheel_damping_torque_nm+.0064f*(1-blend))<1e-6f);
  assert(fabsf(g_wheel_output.back_emf_blend-blend)<1e-6f);
  assert(g_dual_hold.sync_torque_nm[0]==0);check_startup_wheel_lqr();check_joint_commands();
  if(ramp_steps==1) assert(g_balance.reference[2]==g_balance.state[2]);
  previous=blend;previous_force=g_dual_hold.feedforward_n;
 } while(g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE && ramp_steps<110);
 assert(ramp_steps>=100 && ramp_steps<=101 && g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
 assert(g_dual_hold.feedforward_n<1.02f && g_robot_control.startup_blend==1);
 assert(fabsf(g_robot_control.wheel_forward_gap_m-.0951612f)<1e-5f);
 printf("PASS: all five readiness bits absent; nonzero leg LQR from first frame, full blend at %u ms with support only %.3f N\n",ramp_steps*5,g_dual_hold.feedforward_n);
 for(unsigned n=0;n<1000;n++) {step();assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);}
 assert(fabsf(g_dual_hold.feedforward_n-defaults.feedforward_target_n)<1e-6f);
 /* Limit a replay of rapidly reversing large Tp requests even when Watch
  * asks for full gain. Check applied VMC/CAN and clear limits on stop. */
 g_robot_control.enable=0;step();
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 imu.attitude_deg[BODY_PITCH_AXIS]=0;imu.gyro_rad_s[BODY_PITCH_AXIS]=0;
 swing_feedback(0,.07f,1.57079632679f,0);swing_feedback(1,.07f,1.57079632679f,0);
 g_balance.leg_scale=1;g_robot_control.enable=1;for(unsigned n=0;n<110;n++) step();
 unsigned positive_tp=0,negative_tp=0;
 for(unsigned cycle=0;cycle<40;cycle++) {
  imu.attitude_deg[BODY_PITCH_AXIS]=(cycle%2)?-20:20;
  imu.gyro_rad_s[BODY_PITCH_AXIS]=0;
  swing_feedback(0,.07f,1.57079632679f,0);swing_feedback(1,.07f,1.57079632679f,0);
  g_balance.leg_scale=1;step();
  assert(g_robot_control.enable && g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
  assert(fabsf(fabsf(g_balance.torque_nm[1])-BALANCE_LEG_LQR_TORQUE_LIMIT_NM)<1e-7f && g_balance.saturated);
  assert(g_dual_hold.lqr_pitch_torque_nm[0]==g_balance.torque_nm[1]);
  assert(g_dual_hold.lqr_pitch_torque_nm[1]==g_balance.torque_nm[1]);
  float requested_tp[2];float cap_state[6],cap_ref[6];
  for(unsigned i=0;i<6;i++) {cap_state[i]=g_balance.state[i];cap_ref[i]=g_balance.reference[i];}
  assert(lqr_control(g_balance.leg_length_m,cap_state,cap_ref,requested_tp));
  assert(fabsf(requested_tp[1])>BALANCE_LEG_LQR_TORQUE_LIMIT_NM && g_balance.torque_nm[1]*requested_tp[1]>0);
  if(g_balance.torque_nm[1]>0) ++positive_tp;else ++negative_tp;
  check_startup_wheel_lqr();check_joint_commands();
 }
 assert(positive_tp && negative_tp);
 g_balance.leg_scale=0;step();assert(g_balance.torque_nm[1]==0);
 g_robot_control.enable=0;step();assert(g_balance.torque_nm[1]==0 && !g_balance.saturated);
 g_balance.leg_scale=BALANCE_DEFAULT_LEG_SCALE;g_robot_control.enable=1;
 for(unsigned n=0;n<1100;n++) step();
 printf("PASS: reversing full-gain 20 deg replay clips applied leg LQR to +/-%g Nm; zero scale/OFF clear Tp, wheel LQR arithmetic unchanged\n",BALANCE_LEG_LQR_TORQUE_LIMIT_NM);
 /* Readiness still identifies each condition, but never suspends active LQR. */
 imu.attitude_deg[BODY_PITCH_AXIS]=0;imu.gyro_rad_s[BODY_PITCH_AXIS]=0;
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 swing_feedback(0,.07f,1.57079632679f,0);swing_feedback(1,.07f,1.57079632679f,0);step();
 assert(g_robot_control.startup_ready_mask==ROBOT_READY_ALL);
 swing_feedback(1,.09f,1.57079632679f,0);step();
 assert((g_robot_control.startup_ready_mask&ROBOT_READY_GAP) && !(g_robot_control.startup_ready_mask&ROBOT_READY_LENGTH));
 swing_feedback(0,.061f,1.57079632679f,0);swing_feedback(1,.079f,1.57079632679f,0);step();
 assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_LENGTH));
 swing_feedback(0,.07f,1.57079632679f,0);swing_feedback(1,.07f,1.57079632679f,0);
 imu.attitude_deg[BODY_PITCH_AXIS]=-10;step();assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_POSE));
 imu.attitude_deg[BODY_PITCH_AXIS]=0;imu.gyro_rad_s[BODY_PITCH_AXIS]=.2f;step();
 assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_SPEED));imu.gyro_rad_s[BODY_PITCH_AXIS]=0;
 swing_feedback(0,.07f,1.57079632679f,.3f);step();assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_SPEED));
 swing_feedback(0,.07f,1.57079632679f,0);motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-2;step();
 assert(g_robot_control.startup_ready_mask&ROBOT_READY_SPEED);
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_robot_control.startup_blend==1);
 /* Optional common damping retains direction, bounds and differential freedom. */
 for(unsigned trial=0;trial<4;trial++) {
  g_robot_control.enable=0;step();
  float speed=trial==0?-2.0f:trial==1?100.0f:trial==2?-100.0f:2.0f;
  motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-speed;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=speed;
  g_robot_control.enable=1;step();assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
  assert(g_balance.startup_wheel_damping_torque_nm*speed<0 && fabsf(g_balance.startup_wheel_damping_torque_nm)<=.012001f);
  if(trial==1 || trial==2) assert(fabsf(fabsf(g_balance.startup_wheel_damping_torque_nm)-.012f*.99f)<1e-6f);
  check_startup_wheel_lqr();check_joint_commands();
 }
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=2;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=2;
 step();assert(g_balance.startup_wheel_damping_torque_nm==0);check_startup_wheel_lqr();
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-2;
 g_balance.wheel_scale=0;step();assert(g_balance.torque_nm[0]==0);
 g_balance.wheel_scale=.8f;g_balance.startup_wheel_damping_nm_s_per_rad=0;step();
 assert(g_balance.startup_wheel_damping_torque_nm==0);check_startup_wheel_lqr();
 /* First-frame forward/backward lean produces wheel AND gradual leg output. */
 for(unsigned trial=0;trial<2;trial++) {
  g_robot_control.enable=0;step();
  swing_feedback(0,.11f,1.57079632679f,0);swing_feedback(1,.11f,1.57079632679f,0);
  motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
  imu.attitude_deg[BODY_PITCH_AXIS]=trial?1:-1;
  g_robot_control.enable=1;step();
  assert(g_robot_control.enable && g_robot_control.status==1);
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP && fabsf(g_robot_control.startup_blend-.01f)<1e-6f);
  assert(!(g_robot_control.startup_ready_mask&ROBOT_READY_LENGTH));
  assert(abs(g_dual_hold.voltage_mv[2])>1000 && abs(g_dual_hold.voltage_mv[6])>1000);
  assert(g_balance.torque_nm[0]*g_wheel_leg.body_pitch_rad>0);
  assert(g_dual_hold.voltage_mv[2]*g_wheel_leg.body_pitch_rad<0 && g_dual_hold.voltage_mv[6]*g_wheel_leg.body_pitch_rad>0);
  assert(fabsf(g_balance.torque_nm[1])>1e-6f && g_dual_hold.lqr_pitch_torque_nm[0]==g_balance.torque_nm[1]);
  check_startup_wheel_lqr();check_joint_commands();
 }
 /* Missing wheel/joint/IMU feedback faults DURING the ramp. Recovery requires
  * rearming and a fresh 1% first step, never the previous ramp fraction. */
 for(unsigned fault=0;fault<3;fault++) {
  g_robot_control.enable=0;step();imu.attitude_deg[BODY_PITCH_AXIS]=0;
  g_robot_control.enable=1;for(unsigned n=0;n<10;n++) step();
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
  if(fault==0) motors[WHEEL_LEFT_CAN_ID-1].online=0;
  if(fault==1) motors[JOINT_LF_CAN_ID-1].age_ms=11;
  if(fault==2) imu_ok=0;
  step();assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_OFF && g_robot_control.startup_blend==0);
  assert(g_balance.torque_nm[0]==0 && g_balance.torque_nm[1]==0);
  for(unsigned id=0;id<8;id++) assert(g_dual_hold.voltage_mv[id]==0);
  motors[WHEEL_LEFT_CAN_ID-1].online=1;motors[JOINT_LF_CAN_ID-1].age_ms=0;imu_ok=1;
  step();assert(!g_robot_control.enable);
  g_robot_control.enable=1;step();assert(fabsf(g_robot_control.startup_blend-.01f)<1e-6f);
 }
 g_robot_control.startup_enable=2;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 g_robot_control.startup_enable=1;g_robot_control.enable=1;step();fail_at=calls+1;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_TX);fail_at=0;
 for(unsigned n=0;n<4;n++) step();
 g_robot_control.enable=1;step();g_wheel_test.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.status==6 && g_robot_control.startup_phase==ROBOT_STARTUP_OFF);
 g_wheel_test.enable=0;for(unsigned n=0;n<4;n++) step();
 const float bad_damping[]={NAN,-.001f,.0201f};
 for(unsigned field=0;field<2;field++) for(unsigned i=0;i<3;i++) {
  g_balance.startup_wheel_damping_nm_s_per_rad=.004f;g_balance.startup_wheel_limit_nm=.012f;
  if(field==0) g_balance.startup_wheel_damping_nm_s_per_rad=bad_damping[i];
  else g_balance.startup_wheel_limit_nm=bad_damping[i];
  g_robot_control.enable=1;step();
  assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
  assert(g_balance.startup_wheel_damping_torque_nm==0);
 }
 /* Temporary local PD in capture mode fades out with the short LQR ramp. */
 g_balance.startup_wheel_damping_nm_s_per_rad=.004f;g_balance.startup_wheel_limit_nm=.012f;
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 g_dual_hold.angle_reference_mode=LEG_ANGLE_CAPTURE;g_balance.leg_hold_ratio=0;
 swing_feedback(0,.07f,1.54079632679f,0);swing_feedback(1,.07f,1.60079632679f,0);
 g_robot_control.enable=1;
 for(unsigned n=0;n<200;n++) step();
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_dual_hold.angle_hold_torque_nm[0]==0);
 assert(g_dual_hold.angle_reference_rad[0]==g_dual_hold.angle_target_rad);
 g_robot_control.enable=0;step();g_dual_hold.angle_reference_mode=LEG_ANGLE_NEUTRAL;g_balance.leg_hold_ratio=1;
 g_robot_control.startup_enable=0;g_robot_control.enable=1;step();
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_robot_control.startup_blend==1);
 g_robot_control.enable=0;step();g_robot_control.startup_enable=1;
 g_balance.wheel_scale=g_balance.leg_scale=g_balance.back_emf_ratio=0;
 motors[WHEEL_LEFT_CAN_ID-1].online=0;imu_ok=0;g_robot_control.enable=1;step();
 assert(g_robot_control.enable && g_robot_control.status==1 && g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE);
 g_robot_control.enable=0;step();motors[WHEEL_LEFT_CAN_ID-1].online=1;imu_ok=1;
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;imu=saved_imu;
 memcpy(motors,saved_motors,sizeof(motors));
 puts("PASS: parallel support/leg LQR ramp, diagnostic readiness, immediate wheel LQR, short-ramp feedback faults/rearm and bypass");
}
#if MOTOR_COMMAND_TX_ENABLED
static void verify_feedback_only(void) {
 const DualHoldDebug saved_hold=g_dual_hold;const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;const WheelTestDebug saved_wheel=g_wheel_test;
 g_robot_control.remote_enable=0;g_robot_control.feedback_only=1;
 for(unsigned trial=0;trial<4;trial++) {
  g_robot_control.enable=g_dual_hold.enable=g_balance.enable=1;
  g_wheel_test.enable=trial&1;g_wheel_test.zero_request=trial&2;
  step();
  assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable);
  assert(!g_wheel_test.enable && !g_wheel_test.zero_request);
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_OFF);
  assert(g_balance.torque_nm[0]==0 && g_balance.torque_nm[1]==0);
  for(unsigned slot=0;slot<8;slot++) {
   assert(g_dual_hold.voltage_mv[slot]==0 && g_wheel_test.voltage_mv[slot]==0);
   assert(data[slot/4][2*(slot%4)]==0 && data[slot/4][2*(slot%4)+1]==0);
  }
  assert(g_wheel_leg.leg_valid[0] && g_wheel_leg.leg_valid[1]);
 }
 /* CH5 upper cannot override read-only mode; clearing mode at upper cannot
  * restart either. A fresh OFF->ON after leaving it is still required. */
 remote_ok=remote_fresh=1;memset(&remote_sample,0,sizeof(remote_sample));
 g_robot_control.remote_enable=1;
 remote_sample.frame.channel[4]=1000;step();remote_sample.frame.channel[4]=306;step();
 assert(!g_robot_control.enable && g_robot_control.remote_switch_on);
 g_robot_control.feedback_only=0;step();assert(!g_robot_control.enable);
 g_robot_control.remote_enable=0;
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;g_wheel_test=saved_wheel;
 puts("PASS: read-only feedback blocks master/Watch/wheel-test/zeroing/CH5 outputs, keeps leg feedback, and requires OFF/rearm when cleared");
}
static void check_remote_zero(void) {
 assert(!g_robot_control.enable && !g_dual_hold.enable && !g_balance.enable && !g_wheel_test.enable);
 assert(!g_balance.position_locked);
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_OFF);
 for(unsigned i=0;i<8;i++) {
  assert(g_dual_hold.voltage_mv[i]==0);
  assert(data[i/4][2*(i%4)]==0 && data[i/4][2*(i%4)+1]==0);
 }
}
static void remote_upper(void) {remote_sample.frame.channel[4]=306;step();}
static void remote_rearm(void) {remote_sample.frame.channel[4]=1000;step();remote_upper();}
static void verify_ch5_fixed_parking(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;
 const ImuSample saved_imu=imu;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;step();
 g_dual_hold=defaults;g_dual_hold.feedforward_rate_n_s=100000;
 g_balance=(BalanceDebug){.wheel_rate_scale=1.0f,.wheel_scale=.8f,.leg_scale=BALANCE_DEFAULT_LEG_SCALE,
  .leg_hold_ratio=1,.wheel_limit_mv=6000,.position_window_m=BALANCE_DEFAULT_POSITION_WINDOW_M};
 g_robot_control=(RobotControlDebug){.remote_enable=1,.startup_enable=1,
  .remote_on_raw=306,.remote_center_raw=1000,.remote_off_raw=1694,.remote_tolerance_raw=50};
 memset(&remote_sample,0,sizeof(remote_sample));remote_ok=remote_fresh=1;
 imu=(ImuSample){0};imu_ok=1;
 for(unsigned i=0;i<8;i++) {
  motors[i].online=1;motors[i].age_ms=0;
  motors[i].speed_rad_s=motors[i].joint_speed_rad_s=0;
 }
 for(unsigned side=0;side<2;side++) swing_feedback(side,.11f,1.57079632679f,0);
 remote_rearm();assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
 float startup_origin=g_balance.state[2];
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-8;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=8;
 for(unsigned n=0;n<40;n++) {
  step();assert(g_robot_control.enable && !g_balance.position_locked);
  assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP);
  assert(fabsf(g_balance.state[2]-g_balance.reference[2])<=.100001f);
 }
 assert(g_balance.state[2]-startup_origin>.04f);
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-2;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=2;
 for(unsigned side=0;side<2;side++) swing_feedback(side,.07f,1.57079632679f,0);
 for(unsigned n=0;n<300 && g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE;n++) {
  step();assert(g_robot_control.enable);
  if(g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE) assert(!g_balance.position_locked);
 }
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_balance.position_locked);
 assert(g_balance.state[3]>.05f); /* Enter full balance while still rolling. */
 const float parked=g_balance.reference[2];
 assert(parked==g_balance.state[2] && parked-startup_origin>.04f);
 float K[12];lqr_k(.07f,K);
 /* Ground-motion simulation uses real production encoder integration. CH1-4
  * have no driving role, speed target stays zero, and drift/sign changes,
  * body lean and trim must never replace the parking target. */
 for(unsigned trial=0;trial<2;trial++) {
  const float speed=trial?-8:8;
  motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-speed;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=speed;
  for(unsigned ch=0;ch<4;ch++) remote_sample.frame.channel[ch]=trial?0:2047;
  g_balance.pitch_trim_deg=trial?-.2f:.2f;
  imu.attitude_deg[BODY_PITCH_AXIS]=trial?1:-1;
  for(unsigned n=0;n<(trial?1200:600);n++) {
   step();assert(g_robot_control.enable && g_balance.position_locked);
   assert(g_balance.reference[2]==parked && g_balance.reference[3]==0);
   float error=g_balance.state[2]-parked;
   assert(fabsf(g_balance.wheel_torque_component_nm[2]+K[4]*error*.8f)<1e-6f);
  }
  assert((trial?-1:1)*(g_balance.state[2]-parked)>.6f);
 }
 /* The configured bound limits input without moving the true parking goal.
  * Exercise both directions using production odometry, including returning
  * into the 10cm band and changing the parameter while velocity is zero. */
 g_balance.parking_correction_limit_m=BALANCE_DEFAULT_PARKING_CORRECTION_M;
 assert(g_balance.parking_correction_limit_m==.1f);
 for(unsigned trial=0;trial<2;trial++) {
  float speed=trial?-8:8;
  motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-speed;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=speed;
  for(unsigned n=0;n<(trial?1200:600);n++) {
   step();assert(g_robot_control.enable && g_balance.position_locked);
   float real_error=g_balance.state[2]-parked;
   float applied=fmaxf(-.1f,fminf(.1f,real_error));
   assert(g_balance.parking_target_m==parked && fabsf(g_balance.parking_error_m-real_error)<1e-6f);
   assert(fabsf(g_balance.state[2]-g_balance.reference[2]-applied)<1e-6f);
   assert(fabsf(g_balance.wheel_torque_component_nm[2]+K[4]*applied*.8f)<1e-6f);
  }
 }
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 step();step();
 g_balance.parking_correction_limit_m=0;step();
 float unbounded_components[6];
 for(unsigned i=0;i<6;i++) unbounded_components[i]=g_balance.wheel_torque_component_nm[i];
 assert(g_balance.reference[2]==parked && fabsf(g_balance.parking_error_m)>.6f);
 g_balance.parking_correction_limit_m=.1f;step();
 assert(fabsf(g_balance.wheel_torque_component_nm[2])<fabsf(unbounded_components[2])/5);
 for(unsigned i=0;i<6;i++) if(i!=2) assert(fabsf(g_balance.wheel_torque_component_nm[i]-unbounded_components[i])<1e-6f);
 assert(g_balance.parking_target_m==parked);check_joint_commands();
 g_balance.parking_correction_limit_m=0; /* Subsequent legacy recapture fixture. */
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=2;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=-2;
 g_balance.position_window_m=.1f;step();assert(!g_balance.position_locked);
 g_balance.position_window_m=0;step();assert(g_balance.position_locked);
 float recaptured=g_balance.reference[2];assert(recaptured==g_balance.state[2]);
 step();assert(g_balance.reference[2]==recaptured && g_balance.state[2]!=recaptured);
 /* OFF, failed feedback and TX clear the lock immediately; immediate rearm
  * with startup bypass captures the new position, not the former footprint. */
 g_robot_control.startup_enable=0;
 remote_sample.frame.channel[4]=1000;step();check_remote_zero();
 remote_upper();assert(g_balance.position_locked && g_balance.reference[2]==g_balance.state[2]);
 motors[JOINT_LF_CAN_ID-1].online=0;step();check_remote_zero();
 motors[JOINT_LF_CAN_ID-1].online=1;remote_rearm();
 assert(g_balance.position_locked && g_balance.reference[2]==g_balance.state[2]);
 fail_at=calls+1;step();fail_at=0;
 assert(!g_robot_control.enable && !g_balance.position_locked &&
        g_robot_control.last_fault.reason==ROBOT_FAULT_TX);
 remote_rearm();
 assert(g_balance.position_locked && g_balance.reference[2]==g_balance.state[2]);
 remote_ok=0;step();check_remote_zero();remote_ok=1;remote_rearm();
 assert(g_balance.position_locked && g_balance.reference[2]==g_balance.state[2]);
 /* Reverse rolling must also pass real preparation without bypassing it. */
 remote_sample.frame.channel[4]=1000;step();check_remote_zero();
 g_robot_control.startup_enable=1;
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=2;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=-2;
 remote_upper();
 for(unsigned n=0;n<300 && g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE;n++) {
  step();assert(g_robot_control.enable);
  if(g_robot_control.startup_phase!=ROBOT_STARTUP_BALANCE) assert(!g_balance.position_locked);
 }
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_BALANCE && g_balance.position_locked);
 assert(g_balance.state[3]<-.05f && g_balance.reference[2]==g_balance.state[2]);
 remote_sample.frame.channel[4]=1000;step();check_remote_zero();
 g_robot_control.remote_enable=0;remote_ok=remote_fresh=0;
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;imu=saved_imu;
 memcpy(motors,saved_motors,sizeof(motors));step();
 puts("PASS: CH5-only rolling startup reaches phase 3 and captures parking, holds through bidirectional drift/lean/trim, zero speed target, mode switch, OFF/SBUS/feedback/TX rearm; simulated commands only");
}
static void verify_remote_ch5(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;
 const ImuSample saved_imu=imu;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=0;for(unsigned n=0;n<4;n++) step();
 g_dual_hold=defaults;g_balance=(BalanceDebug){.wheel_rate_scale=1.0f,.wheel_scale=.8f,.leg_scale=BALANCE_DEFAULT_LEG_SCALE,.leg_hold_ratio=1,.wheel_limit_mv=6000};
 for(unsigned i=0;i<8;i++) {motors[i].online=1;motors[i].age_ms=0;motors[i].speed_rad_s=motors[i].joint_speed_rad_s=0;}
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=imu.attitude_deg[i]=0;
 imu.attitude_deg[BODY_PITCH_AXIS]=-1;imu_ok=1;
 swing_feedback(0,.11f,1.57079632679f,0);swing_feedback(1,.11f,1.57079632679f,0);
 memset(&remote_sample,0,sizeof(remote_sample));remote_ok=remote_fresh=1;
 g_robot_control.remote_enable=1;g_robot_control.remote_on_raw=306;g_robot_control.startup_enable=1;
 g_robot_control.remote_center_raw=1000;g_robot_control.remote_off_raw=1694;g_robot_control.remote_tolerance_raw=50;
 /* Already UPPER at boot does not start; neither Watch nor individual test
  * enables can bypass CH5 ownership without explicitly selecting Watch mode. */
 remote_upper();check_remote_zero();assert(g_robot_control.remote_status==3);
 assert(g_robot_control.remote_ch5_percent==100);
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=g_wheel_test.enable=1;
 step();check_remote_zero();
 const unsigned off_values[]={0,1,100,172,255,357,992,993,1000,1010,1200,1694,1811,2047};
 for(unsigned trial=0;trial<sizeof(off_values)/sizeof(off_values[0]);trial++) {
  for(unsigned ch=0;ch<4;ch++) remote_sample.frame.channel[ch]=2047;
  remote_sample.frame.channel[4]=off_values[trial];step();check_remote_zero();
  assert(g_robot_control.remote_status==2 && g_robot_control.remote_ch5_raw==off_values[trial]);
  if(off_values[trial]==1000) assert(g_robot_control.remote_ch5_percent==0);
  if(off_values[trial]==1694) assert(g_robot_control.remote_ch5_percent==-100);
  if(off_values[trial]==993) assert(g_robot_control.remote_ch5_percent>1 && g_robot_control.remote_ch5_percent<1.02f);
 }
 assert(g_robot_control.remote_rearm_ready);
 remote_upper();assert(g_robot_control.enable && g_robot_control.status==1 && g_robot_control.remote_status==4);
 assert(g_robot_control.remote_link_ok && g_robot_control.remote_switch_on);
 assert(g_robot_control.startup_phase==ROBOT_STARTUP_RAMP && fabsf(g_robot_control.startup_blend-.01f)<1e-6f);
 assert(g_dual_hold.voltage_mv[2]<-1000 && g_dual_hold.voltage_mv[6]>1000); /* Immediate wheel response retained. */
 for(unsigned n=0;n<10;n++) {step();assert(g_robot_control.enable);}
 /* Both upper detent band edges enable; immediately outside stops and does
  * not itself acknowledge OFF, even if the displayed percentage clips +100. */
 const unsigned upper_edges[]={256,356};
 for(unsigned n=0;n<2;n++) {remote_sample.frame.channel[4]=upper_edges[n];step();assert(g_robot_control.enable);}
 const unsigned outside_edges[]={255,357,0};
 for(unsigned n=0;n<3;n++) {
  remote_sample.frame.channel[4]=outside_edges[n];step();check_remote_zero();
  assert(!g_robot_control.remote_rearm_ready && !g_robot_control.remote_switch_on);
  remote_upper();check_remote_zero();remote_rearm();assert(g_robot_control.enable);
 }
 for(unsigned level=0;level<3;level++) {
  remote_sample.frame.channel[4]=level==0?1000:level==1?993:1694;
  step();check_remote_zero();remote_upper();assert(g_robot_control.enable);
 }
 /* Link flags/transport rejection and timeout stop, and HIGH on recovery
  * cannot rearm without a new real center/low frame. */
 for(unsigned failure=0;failure<4;failure++) {
  if(failure==0) remote_ok=0;
  if(failure==1) remote_sample.frame.failsafe=1;
  if(failure==2) remote_sample.frame.frame_lost=1;
  if(failure==3) remote_fresh=0;
  if(failure==3) for(unsigned n=0;n<21;n++) step();
  else step();
  check_remote_zero();assert(g_robot_control.remote_status==1);
  remote_ok=remote_fresh=1;remote_sample.frame.failsafe=remote_sample.frame.frame_lost=0;
  for(unsigned n=0;n<4;n++) {remote_upper();check_remote_zero();}
  assert(g_robot_control.remote_status==3);remote_rearm();assert(g_robot_control.enable);
 }
 motors[JOINT_LF_CAN_ID-1].online=0;step();
 check_remote_zero();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK && g_robot_control.remote_status==3);
 motors[JOINT_LF_CAN_ID-1].online=1;for(unsigned n=0;n<4;n++) {step();check_remote_zero();}
 remote_sample.frame.channel[4]=357;step();remote_upper();check_remote_zero(); /* Upper-end jitter is not an OFF acknowledgement. */
 remote_rearm();assert(g_robot_control.enable);
 g_robot_control.enable=0;step();check_remote_zero();step();check_remote_zero();
 remote_rearm();assert(g_robot_control.enable);
 g_dual_hold.enable=0;step();check_remote_zero();step();check_remote_zero();
 remote_rearm();assert(g_robot_control.enable);
 fail_at=calls+1;step();assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_TX);
 fail_at=0;step();check_remote_zero();remote_rearm();assert(g_robot_control.enable);
 g_robot_control.remote_on_raw=1000;step();check_remote_zero();assert(g_robot_control.remote_status==5);
 g_robot_control.remote_on_raw=2048;step();check_remote_zero();
 g_robot_control.remote_on_raw=UINT32_MAX;step();check_remote_zero();assert(g_robot_control.remote_status==5);
 g_robot_control.remote_on_raw=970;step();check_remote_zero();assert(g_robot_control.remote_status==5);
 g_robot_control.remote_on_raw=306;g_robot_control.remote_off_raw=500;step();check_remote_zero();assert(g_robot_control.remote_status==5);
 g_robot_control.remote_off_raw=1694;g_robot_control.remote_tolerance_raw=0;step();check_remote_zero();assert(g_robot_control.remote_status==5);
 g_robot_control.remote_tolerance_raw=101;step();check_remote_zero();assert(g_robot_control.remote_status==5);
 g_robot_control.remote_tolerance_raw=50;g_robot_control.remote_enable=2;step();check_remote_zero();
 g_robot_control.remote_enable=1;step();check_remote_zero();remote_rearm();assert(g_robot_control.enable);
 /* Explicit Watch mode preserves existing bench operation without a radio. */
 remote_sample.frame.channel[4]=1000;step();check_remote_zero();
 remote_ok=0;g_robot_control.remote_enable=0;g_robot_control.enable=1;step();
 assert(g_robot_control.enable && g_robot_control.status==1 && g_robot_control.remote_status==0);
 g_robot_control.enable=0;for(unsigned n=0;n<4;n++) step();
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;imu=saved_imu;
 memcpy(motors,saved_motors,sizeof(motors));remote_ok=remote_fresh=0;
 puts("PASS: measured CH5 306/1000/1694 upper-only enable, signed +100/0/-100 preview, detent edges, first wheel response, SBUS/motor/TX fault rearm, config validation, Watch override");
}
#endif
static void verify_pose_guard(DualHoldDebug defaults) {
 const DualHoldDebug saved_hold=g_dual_hold;
 const BalanceDebug saved_balance=g_balance;
 const RobotControlDebug saved_robot=g_robot_control;
 const ImuSample saved_imu=imu;
 const SbusState saved_remote=remote_sample;
 const unsigned saved_remote_ok=remote_ok,saved_remote_fresh=remote_fresh;
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=g_wheel_test.enable=0;
 for(unsigned n=0;n<4;n++) step();
 g_dual_hold=defaults;
 g_balance=(BalanceDebug){.wheel_scale=.5f,.wheel_rate_scale=.5f,.leg_scale=.2f,
     .leg_hold_ratio=1,.wheel_limit_mv=6000};
 g_robot_control.remote_enable=1;g_robot_control.startup_enable=1;
 g_robot_control.remote_on_raw=306;g_robot_control.remote_center_raw=1000;
 g_robot_control.remote_off_raw=1694;g_robot_control.remote_tolerance_raw=50;
 remote_ok=remote_fresh=1;memset(&remote_sample,0,sizeof(remote_sample));
 imu_ok=1;memset(&imu,0,sizeof(imu));
 for(unsigned i=0;i<8;i++) {
  motors[i].online=1;motors[i].age_ms=0;
  motors[i].speed_rad_s=motors[i].joint_speed_rad_s=0;
 }
 const float pi=3.141592653589793f,neutral=pi/2;
 swing_feedback(0,.07f,neutral,0);swing_feedback(1,.07f,neutral,0);
 remote_rearm();assert(g_robot_control.enable);
 /* A 70-degree symmetric split has a perfectly upright shared mean.
  * A 35 ms transient must not drop support; a normal tick resets it. */
 swing_feedback(0,.07f,neutral+35*pi/180,0);
 swing_feedback(1,.07f,neutral-35*pi/180,0);
 for(unsigned n=0;n<7;n++) {step();assert(g_robot_control.enable);}
 assert(fabsf(g_balance.state[0])<1e-5f);
 swing_feedback(0,.07f,neutral,0);swing_feedback(1,.07f,neutral,0);step();
 swing_feedback(0,.07f,neutral+35*pi/180,0);
 swing_feedback(1,.07f,neutral-35*pi/180,0);
 uint32_t faults=g_robot_control.last_fault.count;
 for(unsigned n=0;n<7;n++) {step();assert(g_robot_control.enable);}
 step();check_remote_zero();
 assert(g_robot_control.last_fault.count==faults+1);
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_LEG_POSE);
 assert(g_robot_control.last_fault.dual_status==7 && g_robot_control.remote_status==3);
 for(unsigned n=0;n<12;n++) {step();check_remote_zero();}
 assert(g_robot_control.last_fault.count==faults+1);
 /* HIGH and software enables cannot restart after a pose fault. */
 swing_feedback(0,.07f,neutral,0);swing_feedback(1,.07f,neutral,0);
 g_robot_control.enable=g_dual_hold.enable=g_balance.enable=1;step();check_remote_zero();
 remote_rearm();assert(g_robot_control.enable);
 /* Same-direction collapse: no left/right difference, but both single-leg
  * angles 160 deg must trip after 40 ms. */
 swing_feedback(0,.11f,160*pi/180,0);swing_feedback(1,.11f,160*pi/180,0);
 for(unsigned n=0;n<7;n++) {step();assert(g_robot_control.enable);}
 step();check_remote_zero();assert(g_robot_control.last_fault.reason==ROBOT_FAULT_LEG_POSE);
 /* The recorded 172/3 deg fallen pose is rejected on the FIRST start tick,
  * with zero commands, not another 40 ms of drive. */
 swing_feedback(0,.11f,172*pi/180,0);swing_feedback(1,.11f,3*pi/180,0);
 faults=g_robot_control.last_fault.count;
 remote_rearm();check_remote_zero();
 assert(g_robot_control.last_fault.count==faults+1 &&
        g_robot_control.last_fault.reason==ROBOT_FAULT_LEG_POSE);
 /* Revised cap is valid, but even Watch ratio=1 may not exceed the old
  * maximum effective sync authority of 0.2 Nm. */
 swing_feedback(0,.07f,neutral+.35f,0);swing_feedback(1,.07f,neutral-.35f,0);
 g_dual_hold.sync_limit_nm=.3f;g_dual_hold.sync_ratio=1;
 remote_rearm();check_remote_zero();
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 g_dual_hold=defaults;remote_rearm();assert(g_robot_control.enable);
 for(unsigned n=0;n<200;n++) step();
 const float configured_cap=defaults.sync_ratio*defaults.sync_limit_nm;
 assert(fabsf(g_dual_hold.sync_torque_nm[0]+configured_cap)<1e-5f &&
        fabsf(g_dual_hold.sync_torque_nm[1]-configured_cap)<1e-5f);
 g_dual_hold.sync_limit_nm=.2f;
 for(unsigned n=0;n<200;n++) step();
 const float reduced_cap=defaults.sync_ratio*.2f;
 assert(fabsf(g_dual_hold.sync_torque_nm[0]+reduced_cap)<1e-5f &&
        fabsf(g_dual_hold.sync_torque_nm[1]-reduced_cap)<1e-5f);
 swing_feedback(0,.07f,neutral+.02f,0);swing_feedback(1,.07f,neutral-.02f,0);
 for(unsigned n=0;n<200;n++) step();
 float old_small=g_dual_hold.sync_torque_nm[0];
 g_dual_hold.sync_limit_nm=.3f;
 for(unsigned n=0;n<200;n++) step();
 assert(fabsf(g_dual_hold.sync_torque_nm[0]-old_small)<1e-6f);
 /* The captured failure had small split but a 39 deg body tilt. Test both
  * signs, a 35 ms transient, immediate start rejection and CH5 latching. */
 for(int sign=-1;sign<=1;sign+=2) {
  imu.attitude_deg[BODY_PITCH_AXIS]=0;
  swing_feedback(0,.10f,neutral,0);swing_feedback(1,.10f,neutral,0);
  remote_rearm();assert(g_robot_control.enable);
  for(unsigned transient=0;transient<2;transient++) {
   imu.attitude_deg[BODY_PITCH_AXIS]=sign*39.0f/BODY_PITCH_DIRECTION;
   for(unsigned side=0;side<2;side++) swing_feedback(side,.10f,neutral-sign*39*pi/180,0);
   for(unsigned n=0;n<7;n++) {step();assert(g_robot_control.enable);}
   assert(fabsf(g_balance.state[0])<1e-5f); /* World legs remain vertical. */
   if(!transient) {
    imu.attitude_deg[BODY_PITCH_AXIS]=0;
    for(unsigned side=0;side<2;side++) swing_feedback(side,.10f,neutral,0);
    step();assert(g_robot_control.enable);
   }
  }
  step();check_remote_zero();
  assert(g_robot_control.last_fault.reason==ROBOT_FAULT_BODY_PITCH &&
         g_robot_control.last_fault.dual_status==8);
  faults=g_robot_control.last_fault.count;
  for(unsigned n=0;n<10;n++) {step();check_remote_zero();}
  assert(g_robot_control.last_fault.count==faults);
  remote_rearm();check_remote_zero(); /* Unsafe initial body pose rejected now. */
  assert(g_robot_control.last_fault.count==faults+1 &&
         g_robot_control.last_fault.reason==ROBOT_FAULT_BODY_PITCH);
 }
 imu.attitude_deg[BODY_PITCH_AXIS]=0;
 for(unsigned side=0;side<2;side++) swing_feedback(side,.10f,neutral,0);
 remote_rearm();assert(g_robot_control.enable);
 /* Recovering a moderate head-down state must use more than the old
  * 20 mNm diagnostic cap, with the same leg scale and wheel arithmetic. */
 for(int sign=-1;sign<=1;sign+=2) {
  imu.attitude_deg[BODY_PITCH_AXIS]=sign*20.0f/BODY_PITCH_DIRECTION;
  for(unsigned side=0;side<2;side++) swing_feedback(side,.10f,neutral-sign*20*pi/180,0);
  for(unsigned n=0;n<110;n++) {step();assert(g_robot_control.enable);}
  float sx[6],sr[6],expected[2];
  for(unsigned i=0;i<6;i++) {sx[i]=g_balance.state[i];sr[i]=g_balance.reference[i];}
  assert(lqr_control(g_balance.leg_length_m,sx,sr,expected));
  assert(fabsf(g_balance.torque_nm[1])>.04f &&
         fabsf(g_balance.torque_nm[1]-expected[1]*.2f)<1e-6f &&
         g_balance.torque_nm[1]*sign>0);
 }
 remote_sample.frame.channel[4]=1000;step();check_remote_zero();
 /* Isolate the post-J^T 0.68 Nm motor clamp below voltage saturation.
  * Artificial host-only Nm/V=0.3 makes it distinguishable from 6.9 V. */
 g_robot_control.remote_enable=0;
 g_balance.wheel_scale=g_balance.leg_scale=g_balance.back_emf_ratio=0;
 g_dual_hold=defaults;g_dual_hold.sync_ratio=0;
 g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
 g_dual_hold.feedforward_target_n=g_dual_hold.overlength_kp_n_per_m=0;
 g_dual_hold.angle_reference_mode=LEG_ANGLE_CAPTURE;
 g_dual_hold.angle_kp_nm_per_rad=10;g_dual_hold.angle_kd_nm_s_per_rad=0;
 g_dual_hold.angle_limit_nm=2;
 for(unsigned i=0;i<4;i++) g_dual_hold.torque_ratio_nm_per_v[i]=.3f;
 for(unsigned side=0;side<2;side++) swing_feedback(side,.10f,neutral,0);
 g_dual_hold.enable=1;step();
 for(int sign=-1;sign<=1;sign+=2) {
  for(unsigned side=0;side<2;side++) swing_feedback(side,.10f,neutral+sign*.25f,0);
  step();assert(g_dual_hold.status==1 && g_joint_output.saturated);
  unsigned clipped=0;
  for(unsigned i=0;i<4;i++) {
   float request=g_wheel_leg.leg[i/2].jacobian[3-i%2]*g_dual_hold.pitch_torque_nm[i/2];
   float expected=fmaxf(-.68f,fminf(.68f,request));
   assert(fabsf(g_dual_hold.joint_torque_nm[i]-expected)<1e-6f);
   assert(fabsf(g_dual_hold.joint_torque_nm[i])<=.680001f);
   assert(abs(g_joint_output.voltage_mv[i])<6900);
   clipped+=fabsf(request)>.68f;
  }
  assert(clipped);check_joint_commands();
 }
 g_dual_hold.enable=0;step();
 g_robot_control=saved_robot;g_dual_hold=saved_hold;g_balance=saved_balance;
 imu=saved_imu;remote_sample=saved_remote;
 remote_ok=saved_remote_ok;remote_fresh=saved_remote_fresh;
 memcpy(motors,saved_motors,sizeof(motors));
 puts("PASS: mean-hidden split/single-leg guard, 35ms transient reset, 40ms zero-CAN fault, unsafe start rejection, CH5 rearm, configured/reduced sync caps and effective 0.2 Nm ceiling");
 puts("PASS: +/-39 deg body guard with upright world legs, transient reset, fault10 zero-CAN/rearm/start rejection; nominal-gain moderate pitch >40mNm allowed and post-J^T motor torque limited to +/-0.68Nm below voltage clipping");
}
static AxialResponse simulate_axial_response(float stiffness,float damping,float mass,float load) {
 /* A deliberately simplified per-leg mass/load plant, not a motor or full
  * robot model. Feed the actual controller geometric angles and rates and
  * include one 5 ms force delay. It checks damping against the previous
  * preset without claiming hardware stability, contact or friction behavior. */
 g_robot_control.enable=0;step();for(unsigned i=0;i<4;i++) step();
 g_dual_hold.target_m=.07f;g_dual_hold.position_kp=5;g_dual_hold.speed_kp=damping;
 g_dual_hold.speed_ki=0;g_dual_hold.max_speed_m_s=.04f;g_dual_hold.filter_tau_s=.02f;
 g_dual_hold.feedforward_target_n=8.5f;g_dual_hold.feedforward_rate_n_s=100000;
 g_dual_hold.overlength_kp_n_per_m=stiffness;g_dual_hold.overlength_margin_m=.005f;
 g_dual_hold.overlength_startup_s=0; /* Isolate the idealized damping hypothesis. */
 g_dual_hold.overlength_transition_m=0;
 g_dual_hold.length_error_deadband_m=g_dual_hold.length_speed_deadband_m_s=0;
 g_dual_hold.support_only=0; /* Historical signed-force ideal plant, not standing defaults. */
 g_dual_hold.max_force_n=15;g_dual_hold.voltage_limit_mv=6900;g_dual_hold.back_emf_ratio=0;
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=g_dual_hold.angle_limit_nm=0;
 g_balance.wheel_scale=g_balance.leg_scale=g_balance.back_emf_ratio=0;
 imu.attitude_deg[BODY_PITCH_AXIS]=0;
 for(unsigned axis=0;axis<3;axis++) imu.gyro_rad_s[axis]=0;
 for(unsigned i=0;i<8;i++) motors[i].joint_speed_rad_s=motors[i].speed_rad_s=0;
 float length=.11f,velocity=0,delayed_force=load;
 float equilibrium=.075f+(8.5f-load-damping*.04f)/stiffness;
 assert(equilibrium>.083f && equilibrium<.11f);
 AxialResponse response={0};
 for(unsigned n=0;n<600;n++) {
  axial_feedback(length,velocity);
  if(n==0) g_robot_control.enable=1;
  step();
  if(!g_robot_control.enable) {++response.faults;break;}
  assert(g_dual_hold.voltage_mv[2]==0 && g_dual_hold.voltage_mv[6]==0);
  velocity+=.005f*(delayed_force-load)/mass;
  length+=.005f*velocity;
  delayed_force=g_dual_hold.force_n[0];
  response.undershoot=fmaxf(response.undershoot,equilibrium-length);
  if(n>=400) {
   response.tail_error=fmaxf(response.tail_error,fabsf(length-equilibrium));
   response.tail_speed=fmaxf(response.tail_speed,fabsf(velocity));
  }
 }
 g_robot_control.enable=0;step();for(unsigned i=0;i<4;i++) step();
 return response;
}
#endif
int main(void) {
#if ROBOT_LEGS_ONLY_CALIBRATION
 imu_ok=1;remote_ok=remote_fresh=1;
 g_robot_control.feedback_only=0;g_robot_control.remote_enable=1;
 for(unsigned i=0;i<8;++i) {motors[i].online=1;motors[i].joint_calibrated=1;}
 for(unsigned j=0;j<4;++j)g_dual_hold.direction[j]=1;
 axial_feedback(.07f,0);
 remote_sample.frame.channel[4]=1000;step();assert(!g_robot_control.enable);
 remote_sample.frame.channel[4]=306;step();
 assert(g_robot_control.enable && g_dual_hold.enable && g_dual_hold.status==1U);
 for(unsigned n=0;n<50;++n) {
  g_balance.enable=1;g_wheel_test.enable=1;g_wheel_test.zero_request=1;step();
  assert(!g_balance.enable && !g_wheel_test.enable && !g_wheel_test.zero_request);
  assert(g_dual_hold.enable && g_robot_control.status==1U);
  assert(!data[0][4] && !data[0][5] && !data[1][4] && !data[1][5]);
 }
 { extern volatile uint32_t g_robot_bench_limit_ms,g_robot_bench_elapsed_ms;
   g_robot_bench_limit_ms=60000U;for(unsigned n=0;n<3;++n)step();
   assert(g_robot_control.enable && g_robot_bench_elapsed_ms<60000U);
   g_robot_bench_limit_ms=60001U;step();
   assert(!g_robot_control.enable && g_robot_control.feedback_only && !g_robot_bench_limit_ms);
   g_robot_control.feedback_only=0;remote_sample.frame.channel[4]=1000;step();
   remote_sample.frame.channel[4]=306;step();g_robot_bench_limit_ms=10U;
   for(unsigned n=0;n<3;++n)step();
   assert(!g_robot_control.enable && !g_dual_hold.enable && g_robot_bench_elapsed_ms==10U);
   g_robot_control.feedback_only=0;
 }
 remote_sample.frame.channel[4]=1694;step();
 assert(!g_robot_control.enable && !g_dual_hold.enable);
 puts("PASS: leg calibration CH5 controls both legs, forced wheel/LQR owner off, physical OFF stops legs");return 0;
#endif
 assert(g_balance.parking_lean_enable==1);
 g_balance.parking_lean_enable=0; /* Dedicated enabled-adaptation fixture covers this path. */
 /* Preserve the pre-damping regression baseline. The real enabled-default
  * path is separately exercised by verify_parking_damping.py. */
 assert(fabsf(g_balance.parking_damping_gain_nm_per_m_s-.01f)<1e-8f);
 g_balance.parking_damping_gain_nm_per_m_s=0;
 (void)check_joint_commands; /* TX-disabled compilation still validates helper syntax. */
 assert(g_balance.wheel_rate_scale==BALANCE_DEFAULT_WHEEL_RATE_SCALE);
 g_balance.wheel_rate_scale=1.0f; /* Existing full-feedback regression fixture. */
 const float configured_joint_ratio[4]={JOINT_LF_TORQUE_RATIO,JOINT_LR_TORQUE_RATIO,JOINT_RF_TORQUE_RATIO,JOINT_RR_TORQUE_RATIO};
 const float fixture_joint_ratio[4]={.0316f,.0317f,.0299f,.0321f};
 for(unsigned i=0;i<4;i++) {
  assert(g_dual_hold.torque_ratio_nm_per_v[i]==configured_joint_ratio[i]);
  /* Pin the established saturation/response fixture after checking defaults. */
  g_dual_hold.torque_ratio_nm_per_v[i]=fixture_joint_ratio[i];
 }
 assert(g_dual_hold.enable==0);
 assert(g_robot_control.enable==0 && g_robot_control.status==0);
 assert(g_robot_control.feedback_only==ROBOT_DEFAULT_FEEDBACK_ONLY);
 g_robot_control.feedback_only=0; /* Remaining control fixtures explicitly leave read-only mode. */
 assert(g_dual_hold.voltage_limit_mv==6900 && g_dual_hold.speed_kp==10);
 assert(g_dual_hold.overlength_startup_s==1.0f);
 assert(g_dual_hold.overlength_kp_n_per_m==265);
 assert(g_dual_hold.support_only==1);
 assert(g_dual_hold.length_error_deadband_m==.001f && g_dual_hold.length_speed_deadband_m_s==.003f);
 assert(g_dual_hold.target_m==LEG_DEFAULT_TARGET_M && g_dual_hold.max_force_n==15);
 g_dual_hold.target_m=.07f; /* Historical fixtures retain their explicit 7cm reference. */
 assert(g_dual_hold.max_speed_m_s==.04f && g_dual_hold.feedforward_target_n==9.0f);
 assert(g_dual_hold.back_emf_ratio==0);
 assert(g_dual_hold.angle_kp_nm_per_rad==.11f && g_dual_hold.angle_kd_nm_s_per_rad==.011f);
 assert(g_dual_hold.angle_limit_nm==.022f);
 assert(g_dual_hold.angle_reference_mode==LEG_ANGLE_NEUTRAL);
 assert(fabsf(g_dual_hold.angle_target_rad-1.57079632679f)<1e-6f);
 assert(g_dual_hold.angle_reference_rate_rad_s==.2f);
 assert(g_dual_hold.sync_ratio==LEG_DEFAULT_SYNC_RATIO &&
        g_dual_hold.sync_rate_nm_s==LEG_DEFAULT_SYNC_RATE_NM_S);
 assert(g_robot_control.startup_enable==1);
 assert(g_robot_control.remote_enable==1 && g_robot_control.remote_on_raw==306);
 assert(g_robot_control.remote_center_raw==1000 && g_robot_control.remote_off_raw==1694 && g_robot_control.remote_tolerance_raw==50);
 g_robot_control.remote_enable=0; /* Legacy actuator fixtures explicitly use Watch ownership. */
 assert(g_balance.startup_wheel_damping_nm_s_per_rad==.001f && g_balance.startup_wheel_limit_nm==.012f);
 assert(g_balance.startup_wheel_damping_torque_nm==0);
 g_robot_control.startup_enable=0; /* Legacy control fixtures deliberately bypass staging. */
 assert(g_balance.position_window_m==0 && !g_balance.position_locked &&
        g_balance.pitch_trim_deg==BALANCE_DEFAULT_PITCH_TRIM_DEG && g_balance.pitch_trim_applied_deg==0);
 assert(g_balance.parking_correction_limit_m==BALANCE_DEFAULT_PARKING_CORRECTION_M);
 g_balance.pitch_trim_deg=0; /* Legacy captured-angle fixtures require zero trim; preset tested separately. */
 const DualHoldDebug boot_hold=g_dual_hold;
 /* Historical unloaded PD fixtures retain their explicit old gains. */
 g_dual_hold.angle_kp_nm_per_rad=.1f;g_dual_hold.angle_kd_nm_s_per_rad=.01f;g_dual_hold.angle_limit_nm=.02f;
 (void)boot_hold;
 /* Explicitly retain independent captured references for legacy fixtures;
  * the new default neutral mode is exercised separately below. */
 g_dual_hold.angle_reference_mode=LEG_ANGLE_CAPTURE;
 g_dual_hold.sync_ratio=0; /* Isolate legacy local-PD/axial/LQR fixtures. */
 g_dual_hold.enable=0;
 WheelLeg_Update(); /* No feedback/polarity at boot: only zero commands. */
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
#if MOTOR_COMMAND_TX_ENABLED
 assert(calls==0);
 for(unsigned group=0;group<2;group++) for(unsigned i=0;i<8;i++) assert(data[group][i]==0);
#else
 assert(calls==0);
#endif
 g_dual_hold.enable=0;step();calls=0;now+=5;
 float angles[2];assert(leg_inverse(.03f,.1f,1.57f,1.57f,angles));
 for(unsigned id=0;id<8;id++) {motors[id].online=1;motors[id].joint_calibrated=1;}
 for(unsigned joint=0;joint<4;joint++) {
  MotorFeedback *f=&motors[g_joint_calibration[joint].motor_id-1];
  f->joint_angle_rad=angles[joint%2==0];
  f->joint_speed_rad_s=joint%2==0?-.2f:.3f;
 }
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=2;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=-2;
 imu.attitude_deg[0]=-10;imu.attitude_deg[1]=23;imu.gyro_rad_s[1]=-.4f;
 WheelLeg_Task_Create();assert(created==1);WheelLeg_Update();
 assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE);
 assert(g_wheel_leg.imu_valid && g_wheel_leg.leg_valid[0] && g_wheel_leg.leg_valid[1] &&
        g_wheel_leg.wheel_valid[0] && g_wheel_leg.wheel_valid[1]);
 assert(fabsf(g_wheel_leg.body_pitch_rad-.174532925f)<1e-6f);
 imu.attitude_deg[1]=-35;step();
 assert(fabsf(g_wheel_leg.body_pitch_rad-.174532925f)<1e-6f);
 assert(g_wheel_leg.gyro_rad_s[1]==-.4f && g_wheel_leg.wheel_speed_rad_s[1]==-2);
 uint32_t expected_mask=(1U<<(WHEEL_LEFT_CAN_ID-1))|(1U<<(WHEEL_RIGHT_CAN_ID-1));
 for(unsigned i=0;i<4;i++) expected_mask|=1U<<(g_joint_calibration[i].motor_id-1);
 assert(received_mask==expected_mask);
 for(unsigned side=0;side<2;side++) {
  LegModel m;assert(leg_model(angles[0],angles[1],LEG_BRANCH_MATLAB,&m));
  assert(fabsf(g_wheel_leg.leg[side].length-.1f)<1e-6f);
  assert(fabsf(g_wheel_leg.length_speed_m_s[side]-(m.jacobian[0]*.3f-m.jacobian[1]*.2f))<1e-6f);
 }
 imu_ok=0;step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL);
 assert(g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_IMU && g_wheel_leg.feedback_motor_id==0);
 assert(!g_wheel_leg.imu_valid && g_wheel_leg.body_pitch_rad==0 && g_wheel_leg.leg_valid[0] &&
        g_wheel_leg.leg_valid[1] && g_wheel_leg.wheel_valid[0] && g_wheel_leg.wheel_valid[1]);
 imu_ok=1;
 for(unsigned id=1;id<=8;id++) {
  if(!(expected_mask&(1U<<(id-1)))) continue;
  motors[id-1].online=0;step();
  assert(g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_MOTOR && g_wheel_leg.feedback_motor_id==id);
  assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL && g_wheel_leg.imu_valid);
  assert(g_wheel_leg.leg_valid[0]==(id!=JOINT_LF_CAN_ID && id!=JOINT_LR_CAN_ID));
  assert(g_wheel_leg.leg_valid[1]==(id!=JOINT_RF_CAN_ID && id!=JOINT_RR_CAN_ID));
  assert(g_wheel_leg.wheel_valid[0]==(id!=WHEEL_LEFT_CAN_ID));
  assert(g_wheel_leg.wheel_valid[1]==(id!=WHEEL_RIGHT_CAN_ID));
  for(unsigned side=0;side<2;side++) if(!g_wheel_leg.leg_valid[side]) {
   assert(g_wheel_leg.leg[side].length==0 && g_wheel_leg.length_speed_m_s[side]==0);
  }
  motors[id-1].online=1;step();
  assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE && g_wheel_leg.feedback_detail==0 &&
         g_wheel_leg.feedback_motor_id==0 && g_wheel_leg.feedback_ms==0);
 }
 motors[WHEEL_LEFT_CAN_ID-1].online=0;step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL);
 motors[WHEEL_LEFT_CAN_ID-1].online=1;
 motors[JOINT_LF_CAN_ID-1].age_ms=6;step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL);
 assert(g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_SKEW && g_wheel_leg.feedback_ms==6);
 assert(!g_wheel_leg.leg_valid[0] && g_wheel_leg.leg_valid[1] && g_wheel_leg.imu_valid);
 motors[JOINT_LF_CAN_ID-1].protocol_a1=motors[JOINT_LR_CAN_ID-1].protocol_a1=1;
 step();assert(g_wheel_leg.leg_valid[0] && g_wheel_leg.feedback_detail==0);
 motors[JOINT_LF_CAN_ID-1].age_ms=7;step();
 assert(!g_wheel_leg.leg_valid[0] && g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_SKEW);
 /* A1 can select only genuine RX history supplied with current healthy
  * feedback; the observer cannot synthesize its own old pair timestamps. */
 motors[JOINT_LF_CAN_ID-1].age_ms=motors[JOINT_LR_CAN_ID-1].age_ms=0;
 motors[JOINT_LF_CAN_ID-1].last_rx_ms=motors[JOINT_LR_CAN_ID-1].last_rx_ms=now;
 step();assert(g_wheel_leg.leg_valid[0]);
 motors[JOINT_LF_CAN_ID-1].age_ms=7;
 MotorFeedback *hf=&motors[JOINT_LF_CAN_ID-1];
 const JointCalibration *hc=&g_joint_calibration[0];
 hf->fast_history_count=1;
 hf->fast_history[0]=(MotorFastSample){now,1,
  hc->encoder_reference_rad+(hf->joint_angle_rad-hc->model_reference_rad)*hc->ratio/hc->direction,
  hf->speed_rad_s};
 step();assert(g_wheel_leg.leg_valid[0] && g_wheel_leg.feedback_detail==0);
 assert(g_wheel_leg.joint_feedback_age_ms[0]==5 && g_wheel_leg.joint_feedback_age_ms[1]==0);
 step();assert(!g_wheel_leg.leg_valid[0] && g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_SKEW);
 /* A current fault/offline channel cannot be concealed by cached data. */
 motors[JOINT_LF_CAN_ID-1].age_ms=0;
 motors[JOINT_LF_CAN_ID-1].last_rx_ms=motors[JOINT_LR_CAN_ID-1].last_rx_ms=now;
 step();motors[JOINT_LF_CAN_ID-1].online=0;step();
 assert(!g_wheel_leg.leg_valid[0] && g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_MOTOR);
 motors[JOINT_LF_CAN_ID-1].online=1;motors[JOINT_LF_CAN_ID-1].age_ms=7;step();
 assert(!g_wheel_leg.leg_valid[0] && g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_SKEW);
 motors[JOINT_LF_CAN_ID-1].protocol_a1=motors[JOINT_LR_CAN_ID-1].protocol_a1=0;
 motors[JOINT_LF_CAN_ID-1].fast_history_count=0;
 motors[JOINT_LF_CAN_ID-1].age_ms=0;
 for(unsigned i=0;i<8;i++) motors[i].age_ms=11;
 step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL);
 assert(g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_STALE &&
        g_wheel_leg.feedback_motor_id==JOINT_LF_CAN_ID && g_wheel_leg.feedback_ms==11);
 for(unsigned i=0;i<8;i++) motors[i].age_ms=0;
 motors[JOINT_RR_CAN_ID-1].joint_calibrated=0;step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL);
 assert(g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_CALIBRATION && g_wheel_leg.feedback_motor_id==JOINT_RR_CAN_ID);
 motors[JOINT_RR_CAN_ID-1].joint_calibrated=1;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=NAN;step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL);
 assert(g_wheel_leg.feedback_detail==WHEEL_LEG_FEEDBACK_NONFINITE && g_wheel_leg.feedback_motor_id==WHEEL_RIGHT_CAN_ID);
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=-2;
 imu.gyro_rad_s[2]=NAN;step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL);imu.gyro_rad_s[2]=0;
 /* Removing the whole right side must not freeze left-side or IMU data. */
 motors[JOINT_RF_CAN_ID-1].online=motors[JOINT_RR_CAN_ID-1].online=0;
 motors[WHEEL_RIGHT_CAN_ID-1].online=0;
 imu.attitude_deg[0]=-20;motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=3;step();
 assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL && g_wheel_leg.imu_valid);
 assert(g_wheel_leg.leg_valid[0] && !g_wheel_leg.leg_valid[1]);
 assert(g_wheel_leg.wheel_valid[0] && !g_wheel_leg.wheel_valid[1]);
 assert(fabsf(g_wheel_leg.body_pitch_rad-.34906585f)<1e-6f);
 assert(g_wheel_leg.wheel_speed_rad_s[0]==3 && g_wheel_leg.wheel_speed_rad_s[1]==0);
 assert(g_wheel_leg.leg[1].length==0 && g_wheel_leg.feedback_motor_id==JOINT_RF_CAN_ID);
 float moved[2];assert(leg_inverse(.03f,.11f,angles[0],angles[1],moved));
 motors[JOINT_LR_CAN_ID-1].joint_angle_rad=moved[0];
 motors[JOINT_LF_CAN_ID-1].joint_angle_rad=moved[1];step();
 assert(fabsf(g_wheel_leg.leg[0].length-.11f)<1e-6f);
 /* IMU-only and then no valid channels; no stale geometry survives. */
 for(unsigned id=0;id<8;id++) motors[id].online=0;
 step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE_PARTIAL && g_wheel_leg.imu_valid);
 assert(!g_wheel_leg.leg_valid[0] && !g_wheel_leg.leg_valid[1] && g_wheel_leg.leg[0].length==0);
 imu_ok=0;step();assert(g_wheel_leg.status==WHEEL_LEG_WAIT_FEEDBACK && !g_wheel_leg.imu_valid);
 imu_ok=1;imu.attitude_deg[0]=-10;
 for(unsigned id=0;id<8;id++) motors[id].online=1;
 motors[JOINT_LR_CAN_ID-1].joint_angle_rad=angles[0];
 motors[JOINT_LF_CAN_ID-1].joint_angle_rad=angles[1];
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=2;
 /* Independent channels need no global CAN skew gate. */
 motors[JOINT_RF_CAN_ID-1].age_ms=motors[JOINT_RR_CAN_ID-1].age_ms=8;
 motors[WHEEL_RIGHT_CAN_ID-1].age_ms=10;step();
 assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE && g_wheel_leg.feedback_detail==0);
 for(unsigned id=0;id<8;id++) motors[id].age_ms=0;
 /* Replay recorded left-leg poses on both model-coordinate inputs. */
 const float front_deg[2]={145,185},rear_deg[2]={31.37f,11};
 const float expected_length[2]={.131621616f,.105513486f};
 for(unsigned pose=0;pose<2;pose++) {
  const float rad=.017453292519943295f;
  float rear=rear_deg[pose]*rad,front=front_deg[pose]*rad;
  motors[JOINT_LR_CAN_ID-1].joint_angle_rad=rear;
  motors[JOINT_LF_CAN_ID-1].joint_angle_rad=front;step();
  motors[JOINT_RR_CAN_ID-1].joint_angle_rad=rear;
  motors[JOINT_RF_CAN_ID-1].joint_angle_rad=front;step();
  assert(g_wheel_leg.leg_valid[1] && g_wheel_leg.leg[1].angle>0);
  assert(fabsf(g_wheel_leg.leg[1].length-expected_length[pose])<1e-6f);
  assert(fabsf(g_wheel_leg.leg[1].angle-g_wheel_leg.leg[0].angle)<1e-6f);
  assert(g_wheel_leg.leg_valid[0] && g_wheel_leg.leg[0].angle>0);
  assert(fabsf(g_wheel_leg.leg[0].length-expected_length[pose])<1e-6f);
  for(unsigned joint=0;joint<2;joint++) {
   LegModel plus,minus;float a[2]={rear,front};
   a[joint]+=.0001f;assert(leg_model(a[0],a[1],LEG_BRANCH_DOWN,&plus));
   a[joint]-=.0002f;assert(leg_model(a[0],a[1],LEG_BRANCH_DOWN,&minus));
   assert(fabsf((plus.length-minus.length)/.0002f-g_wheel_leg.leg[0].jacobian[joint])<.001f);
   assert(fabsf((plus.angle-minus.angle)/.0002f-g_wheel_leg.leg[0].jacobian[joint+2])<.005f);
  }
 }
 LegModel ambiguous;
 motors[JOINT_RR_CAN_ID-1].joint_angle_rad=angles[0];
 motors[JOINT_RF_CAN_ID-1].joint_angle_rad=angles[1];
 assert(!leg_model(0,acosf(-.2f),LEG_BRANCH_DOWN,&ambiguous));
 motors[JOINT_LR_CAN_ID-1].joint_angle_rad=angles[0];
 motors[JOINT_LF_CAN_ID-1].joint_angle_rad=angles[1];
 now+=11;WheelLeg_Update();assert(g_wheel_leg.status==WHEEL_LEG_BAD_TIMING);
 step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE);
 now=UINT32_MAX-2;WheelLeg_Update();step();assert(g_wheel_leg.status==WHEEL_LEG_OBSERVE);
 /* Shadow six-state mapping, signed wheel odometry and matrix multiplication. */
 imu.attitude_deg[0]=-10;imu.gyro_rad_s[0]=-.3f;
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-2;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=2;
 now+=11;WheelLeg_Update();
 assert(!g_wheel_leg.state_valid[0] && !g_wheel_leg.lqr_valid[0]);
 step();
 for(unsigned side=0;side<2;side++) {
  assert(g_wheel_leg.state_valid[side] && g_wheel_leg.lqr_valid[side]);
  assert(g_wheel_leg.state[side][2]==0);
  assert(fabsf(g_wheel_leg.state[side][3]-.052f)<1e-7f);
  assert(fabsf(g_wheel_leg.state[side][0]-(g_wheel_leg.leg[side].angle-1.570796327f+.174532925f))<1e-6f);
  assert(fabsf(g_wheel_leg.state[side][1]-(g_wheel_leg.angle_speed_rad_s[side]+.3f))<1e-6f);
  assert(fabsf(g_wheel_leg.state[side][4]+.174532925f)<1e-6f);
  assert(fabsf(g_wheel_leg.state[side][5]+.3f)<1e-6f);
 }
 g_wheel_leg_reference[0][0]=.02f;step();
 assert(fabsf(g_wheel_leg.state[0][2]-.00026f)<1e-8f);
 float gain[12];lqr_k(g_wheel_leg.leg[0].length,gain);
 for(unsigned row=0;row<2;row++) {
  float expected=0;
  for(unsigned col=0;col<6;col++)
   expected-=gain[row+2*col]*(g_wheel_leg.state[0][col]-g_wheel_leg.reference[0][col]);
  assert(fabsf(g_wheel_leg.lqr_torque_nm[0][row]-expected)<1e-6f);
 }
 motors[WHEEL_RIGHT_CAN_ID-1].online=0;step();
 assert(g_wheel_leg.lqr_valid[0] && !g_wheel_leg.state_valid[1] && !g_wheel_leg.lqr_valid[1]);
 assert(g_wheel_leg.lqr_torque_nm[1][0]==0 && g_wheel_leg.lqr_torque_nm[1][1]==0);
 motors[WHEEL_RIGHT_CAN_ID-1].online=1;step();assert(g_wheel_leg.state[1][2]==0);
 g_wheel_leg_reference[0][0]=NAN;step();
 assert(g_wheel_leg.state_valid[0] && !g_wheel_leg.lqr_valid[0] && g_wheel_leg.lqr_valid[1]);
 assert(g_wheel_leg.lqr_torque_nm[0][0]==0 && g_wheel_leg.lqr_torque_nm[0][1]==0);
 g_wheel_leg_reference[0][0]=0;
 float long_angles[2];assert(leg_inverse(.03f,.151f,1.57f,1.57f,long_angles));
 motors[JOINT_LR_CAN_ID-1].joint_angle_rad=long_angles[0];
 motors[JOINT_LF_CAN_ID-1].joint_angle_rad=long_angles[1];step();
 assert(g_wheel_leg.state_valid[0] && !g_wheel_leg.lqr_valid[0]);
 motors[JOINT_LR_CAN_ID-1].joint_angle_rad=angles[0];
 motors[JOINT_LF_CAN_ID-1].joint_angle_rad=angles[1];step();
 assert(calls==0); /* The observation task must never issue CAN commands. */

 /* Dual leg hold: unknown polarity inhibits all output, never drives wheels. */
 g_dual_hold.direction[2]=0;g_dual_hold.direction[3]=0;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 for(unsigned i=0;i<4;i++) step();
 g_dual_hold.direction[2]=1;g_dual_hold.direction[3]=1; /* Host fixture only. */
#if MOTOR_COMMAND_TX_ENABLED
 /* Grounded support: ramp the requested 2 kg load at the short-leg pose,
  * then show the physical voltage bound at the original longer pose. */
 const DualHoldDebug support_preset=g_dual_hold;
 g_dual_hold.feedforward_target_n=9.8f; /* Explicit full-weight estimate fixture. */
 MotorFeedback support_motors[8];memcpy(support_motors,motors,sizeof(motors));
 const ImuSample support_imu=imu;
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=0;
 imu.attitude_deg[0]=0;
 for(unsigned i=0;i<8;i++) motors[i].joint_speed_rad_s=motors[i].speed_rad_s=0;
 float support_pose[2];assert(leg_inverse(.03f,.07f,1.57f,1.57f,support_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=support_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=support_pose[0];
 }
 g_dual_hold.enable=1;
 for(unsigned ramp=0;ramp<1000;ramp++) {
  float last=g_dual_hold.feedforward_n;step();
  assert(g_dual_hold.status==1);
  assert(g_dual_hold.feedforward_n>=last && g_dual_hold.feedforward_n-last<=.01001f);
  for(unsigned i=0;i<4;i++) assert(abs(g_joint_output.voltage_mv[i])<=6900);
 }
 check_joint_commands();
 for(unsigned side=0;side<2;side++) {
  assert(fabsf(g_dual_hold.force_n[side]-9.8f)<1e-4f);
  assert(g_dual_hold.force_limit_n[side]>9.8f && !g_dual_hold.force_saturated[side]);
 }
 assert(!g_joint_output.saturated);
 printf("support at 0.07m: force=%g/%g N, joints=%d %d %d %d mV\n",
  g_dual_hold.force_n[0],g_dual_hold.force_n[1],g_joint_output.voltage_mv[0],
  g_joint_output.voltage_mv[1],g_joint_output.voltage_mv[2],g_joint_output.voltage_mv[3]);
 /* An online force reduction must ramp down smoothly and settle at the
  * lower default on both legs without changing length or swing gains. */
 g_dual_hold.feedforward_target_n=support_preset.feedforward_target_n;
 for(unsigned ramp=0;ramp<300;ramp++) {
  float last=g_dual_hold.feedforward_n;step();
  assert(g_dual_hold.status==1);
  assert(g_dual_hold.feedforward_n<=last && last-g_dual_hold.feedforward_n<=.01001f);
 }
 check_joint_commands();
 for(unsigned side=0;side<2;side++) {
  assert(fabsf(g_dual_hold.force_n[side]-support_preset.feedforward_target_n)<1e-4f);
  assert(!g_dual_hold.force_saturated[side]);
 }
 printf("reduced support: %g/%g N\n",g_dual_hold.force_n[0],g_dual_hold.force_n[1]);
 /* Replay the hand-supported overextension: the old capped controller
  * still pushes upward at 11 cm with a 7 cm target. The soft ceiling must
  * oppose the support force before reaching the LQR range boundary. */
 assert(leg_inverse(.03f,.11f,1.57f,1.57f,support_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=support_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=support_pose[0];
 }
 g_dual_hold.overlength_kp_n_per_m=0;
 for(unsigned settle=0;settle<100;settle++) step();
 for(unsigned side=0;side<2;side++) assert(fabsf(g_dual_hold.force_n[side]-(support_preset.feedforward_target_n-.4f))<1e-4f);
 g_dual_hold.overlength_kp_n_per_m=support_preset.overlength_kp_n_per_m;
 step();check_joint_commands();
 for(unsigned side=0;side<2;side++) {
  assert(fabsf(g_dual_hold.overlength_force_n[side]+.034f*support_preset.overlength_kp_n_per_m)<1e-4f);
  assert(fabsf(g_dual_hold.force_requested_n[side]-(support_preset.feedforward_target_n-.4f-.034f*support_preset.overlength_kp_n_per_m))<1e-4f);
  assert(g_dual_hold.force_n[side]==0 && g_dual_hold.force_saturated[side]);
 }
 puts("PASS: support-only overlength at 0.11m changes 8.6 N outward to zero axial force, without active pulling or enlarging LQR range");
 /* At/below the soft ceiling, no extra extension force is introduced. */
 assert(leg_inverse(.03f,.07f,1.57f,1.57f,support_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=support_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=support_pose[0];
 }
 for(unsigned settle=0;settle<100;settle++) step();
 for(unsigned side=0;side<2;side++) {
  assert(g_dual_hold.overlength_force_n[side]==0);
  assert(fabsf(g_dual_hold.force_n[side]-support_preset.feedforward_target_n)<1e-4f);
 }
 g_dual_hold.feedforward_target_n=9.8f;
 for(unsigned ramp=0;ramp<300;ramp++) step();
 assert(leg_inverse(.03f,.11f,1.57f,1.57f,support_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=support_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=support_pose[0];
 }
 g_dual_hold.target_m=.11f;step();check_joint_commands();
 for(unsigned side=0;side<2;side++) {
  assert(g_dual_hold.force_requested_n[side]>9.79f);
  assert(g_dual_hold.force_saturated[side]);
  assert(g_dual_hold.force_n[side]==g_dual_hold.force_limit_n[side]);
  assert(g_dual_hold.force_n[side]<9.8f);
 }
 printf("support at 0.11m: voltage-bound force=%g/%g N; axial saturation=%u/%u\n",
  g_dual_hold.force_limit_n[0],g_dual_hold.force_limit_n[1],
  g_dual_hold.force_saturated[0],g_dual_hold.force_saturated[1]);
 /* Replay stationary measurement jitter with full support already present.
  * The zero-band comparison retains the same gains, geometry and force limits.
  * Different noise periods avoid assuming correlated position/speed noise. */
 float noise_ripple[2]={0};
 for(unsigned bands=0;bands<2;bands++) {
  g_dual_hold.enable=0;step();for(unsigned i=0;i<4;i++) step();
  g_dual_hold=support_preset;g_dual_hold.feedforward_rate_n_s=100000;
  if(!bands) g_dual_hold.length_error_deadband_m=g_dual_hold.length_speed_deadband_m_s=0;
  float min_force=100,max_force=-100;
  for(unsigned n=0;n<160;n++) {
   axial_feedback(.07f+(n%2? .0002f:-.0002f),(n/2)%2? .002f:-.002f);
   if(!n) g_dual_hold.enable=1;
   step();assert(g_dual_hold.status==1);check_joint_commands();
   if(bands) for(unsigned side=0;side<2;side++) {
    assert(g_dual_hold.length_error_control_m[side]==0);
    assert(g_dual_hold.length_speed_control_m_s[side]==0);
   }
   if(n>=80) {
    min_force=fminf(min_force,g_dual_hold.force_n[0]);
    max_force=fmaxf(max_force,g_dual_hold.force_n[0]);
   }
  }
  noise_ripple[bands]=max_force-min_force;
  if(bands) assert(fabsf(min_force-support_preset.feedforward_target_n)<1e-5f && fabsf(max_force-support_preset.feedforward_target_n)<1e-5f);
 }
 assert(noise_ripple[0]>.03f && noise_ripple[1]<1e-5f);
 printf("PASS: 0.2mm/2mm-per-second measurement jitter force ripple %g -> %g N; support stays 9.0 N\n",
        noise_ripple[0],noise_ripple[1]);
 /* Continuous deadbands preserve real compression/extension direction;
  * restoring onset at 5..7 mm is quadratic, then joins the linear spring. */
 /* The noise fixture restarted the controller only0.8s ago; finish its
  * configured1s force ramp before checking full-strength static forces. */
 for(unsigned settle=0;settle<250 && g_dual_hold.overlength_blend<1;settle++)step();
 assert(g_dual_hold.overlength_blend==1);
 const float knee_lengths[]={.075f,.076f,.077f,.078f};
 const float knee_forces[]={0,-.06625f,-.265f,-.53f};
 for(unsigned n=0;n<4;n++) {
  axial_feedback(knee_lengths[n],0);step();
  for(unsigned side=0;side<2;side++)
   assert(fabsf(g_dual_hold.overlength_force_n[side]-knee_forces[n])<1e-4f);
 }
 for(int sign=-1;sign<=1;sign+=2) {
  axial_feedback(.07f+sign*.004f,sign*.01f);step();
  for(unsigned side=0;side<2;side++) {
   assert(fabsf(g_dual_hold.length_error_control_m[side]+sign*.003f)<1e-5f);
   assert(fabsf(g_dual_hold.length_speed_control_m_s[side]-sign*.007f)<1e-5f);
  }
 }
 /* Lower-bound anti-windup must reject accumulated pull requests, so
  * returning to the support pose does not lose the gravity feedforward. */
 g_dual_hold.speed_ki=10;
 axial_feedback(.12f,0);
 for(unsigned n=0;n<400;n++) {
  step();
  for(unsigned side=0;side<2;side++) assert(g_dual_hold.force_n[side]==0);
 }
 axial_feedback(.07f,0);
 for(unsigned n=0;n<100;n++) step();
 for(unsigned side=0;side<2;side++) assert(fabsf(g_dual_hold.force_n[side]-support_preset.feedforward_target_n)<.02f);
 g_dual_hold.speed_ki=0;
 g_dual_hold.enable=0;step();
 for(unsigned side=0;side<2;side++) {
  assert(g_dual_hold.force_requested_n[side]==0 && g_dual_hold.force_limit_n[side]==0);
  assert(!g_dual_hold.force_saturated[side]);
  assert(g_dual_hold.overlength_force_n[side]==0);
 }
 memcpy(motors,support_motors,sizeof(motors));imu=support_imu;g_dual_hold=support_preset;
 /* Explicit no-load diagnostic fixture keeps the earlier 800 mV tests. */
 g_dual_hold.target_m=.11f;g_dual_hold.max_force_n=10;
 g_dual_hold.feedforward_target_n=0;g_dual_hold.voltage_limit_mv=800;
 g_dual_hold.speed_kp=10; /* Keep prior speed-loop fixtures independent of preset tuning. */
 g_dual_hold.overlength_kp_n_per_m=0; /* Isolate the legacy speed-loop checks. */
 g_dual_hold.length_error_deadband_m=g_dual_hold.length_speed_deadband_m_s=0;
 g_dual_hold.overlength_transition_m=0;
 g_dual_hold.support_only=0; /* Explicitly retain signed unloaded controller tests. */
 /* The bench preset produces no compensation, limits all joints to 800 mV,
  * and preserves immediate velocity damping at every motion reversal. */
 const DualHoldDebug bench_hold=g_dual_hold;
 MotorFeedback bench_motors[8];memcpy(bench_motors,motors,sizeof(motors));
 g_dual_hold.position_kp=0;
 for(int sign=-1;sign<=1;sign+=2) {
  for(unsigned i=0;i<4;i++)
   motors[g_joint_calibration[i].motor_id-1].joint_speed_rad_s=
      sign*g_wheel_leg.leg[i/2].jacobian[1-i%2];
  g_dual_hold.enable=1;step();
  assert(g_dual_hold.status==1);check_joint_commands();
  for(unsigned side=0;side<2;side++) {
   float rate=g_wheel_leg.length_speed_m_s[side];
   assert(rate*sign>0);
   assert(fabsf(g_dual_hold.force_n[side]+10*rate)<1e-6f);
  }
  for(unsigned i=0;i<4;i++) {
   assert(g_joint_output.back_emf_mv[i]==0);
   assert(abs(g_joint_output.voltage_mv[i])<=800);
  }
 }
 /* Reference smoothing starts with a small desired speed; zero measured
  * speed cannot see an abrupt full reference step when the controller arms. */
 for(unsigned i=0;i<4;i++) motors[g_joint_calibration[i].motor_id-1].joint_speed_rad_s=0;
 g_dual_hold.enable=0;step();
 g_dual_hold=bench_hold;g_dual_hold.enable=1;step();
 for(unsigned side=0;side<2;side++) {
  assert(g_dual_hold.force_n[side]>0 && g_dual_hold.force_n[side]<.09f);
 }
 g_dual_hold.enable=0;step();
 memcpy(motors,bench_motors,sizeof(motors));g_dual_hold=bench_hold;
 /* Replay compressed/extended legs: the new length preset restores both
  * signs, gives 2x force for 2 mm error and 4x for 20 mm error, before clamps. */
 for(unsigned i=0;i<4;i++) motors[g_joint_calibration[i].motor_id-1].joint_speed_rad_s=0;
 const float length_errors[]={-.02f,-.002f,.002f,.02f};
 for(unsigned n=0;n<4;n++) {
  float old_force[2]={0};
  for(unsigned preset=0;preset<2;preset++) {
   g_dual_hold.enable=0;step();g_dual_hold=bench_hold;
   g_dual_hold.target_m=.1f+length_errors[n];
   if(!preset) {g_dual_hold.speed_kp=5;g_dual_hold.max_speed_m_s=.02f;}
   g_dual_hold.enable=1;
   for(unsigned settle=0;settle<65;settle++) step();
   assert(g_dual_hold.status==1);check_joint_commands();
   for(unsigned side=0;side<2;side++) {
    float force=g_dual_hold.force_n[side];assert(force*length_errors[n]>0);
    if(!preset) old_force[side]=force;
    else {
     float ratio=fabsf(length_errors[n])>.01f ? 4 : 2;
     assert(fabsf(force-ratio*old_force[side])<1e-5f);
     printf("length hold error=%g m side=%u: %g -> %g N\n",length_errors[n],side,old_force[side],force);
    }
   }
  }
 }
 g_dual_hold.enable=0;step();
 memcpy(motors,bench_motors,sizeof(motors));g_dual_hold=bench_hold;
 /* Regression: constant length with fore/aft swing must receive restoring
  * torque and damping in dual-only mode, without adding a second leg switch. */
 g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
 for(unsigned side=0;side<2;side++) {
  float pose[2],delta=side?.03f:-.04f;
  assert(leg_inverse(.03f+.1f*sinf(delta),.1f*cosf(delta),1.57f,1.57f,pose));
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=pose[0];
 }
 for(unsigned i=0;i<4;i++) motors[g_joint_calibration[i].motor_id-1].joint_speed_rad_s=0;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==1);
 float captured[2]={g_dual_hold.angle_reference_rad[0],g_dual_hold.angle_reference_rad[1]};
 assert(fabsf(captured[0]-captured[1])>.05f);
 for(unsigned side=0;side<2;side++) {
  assert(g_dual_hold.angle_error_rad[side]==0 && g_dual_hold.pitch_torque_nm[side]==0);
 }
 const float deviations[]={-.3f,-.05f,.05f,.3f};
 for(unsigned n=0;n<4;n++) {
  for(unsigned side=0;side<2;side++) {
   float desired=captured[side]+(side?1:-1)*deviations[n],pose[2];
   assert(leg_inverse(.03f+.1f*cosf(desired),.1f*sinf(desired),1.57f,1.57f,pose));
   motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=pose[1];
   motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=pose[0];
  }
  step();assert(g_dual_hold.status==1);check_joint_commands();
  for(unsigned side=0;side<2;side++) {
   float deviation=g_wheel_leg.leg[side].angle-captured[side];
   float tp=g_dual_hold.pitch_torque_nm[side];
   assert(fabsf(g_wheel_leg.leg[side].length-.1f)<1e-6f);
   assert(g_dual_hold.angle_reference_rad[side]==captured[side]);
   assert(tp*deviation<0 && fabsf(tp)<=.02f);
   float expected=fmaxf(-.02f,fminf(.02f,-.1f*deviation));
   assert(fabsf(tp-expected)<1e-6f);
   for(unsigned j=0;j<2;j++)
    assert(fabsf(g_dual_hold.joint_torque_nm[2*side+j]-g_wheel_leg.leg[side].jacobian[3-j]*tp)<1e-6f);
  }
 }
 /* Disable/re-enable at the displaced pose captures the new pose. Rate
  * reversal receives opposing damping in the very next control update. */
 g_dual_hold.enable=0;step();g_dual_hold.enable=1;step();
 for(unsigned side=0;side<2;side++) assert(g_dual_hold.pitch_torque_nm[side]==0);
 for(int sign=-1;sign<=1;sign+=2) {
  for(unsigned i=0;i<4;i++)
   motors[g_joint_calibration[i].motor_id-1].joint_speed_rad_s=
      sign*g_wheel_leg.leg[i/2].jacobian[3-i%2];
  step();check_joint_commands();
  for(unsigned side=0;side<2;side++) {
   float rate=g_wheel_leg.angle_speed_rad_s[side],tp=g_dual_hold.pitch_torque_nm[side];
   assert(rate*sign>0 && tp*rate<0);
   float expected=fmaxf(-.02f,fminf(.02f,-.01f*rate));
   assert(fabsf(tp-expected)<1e-6f);
  }
 }
 g_dual_hold.enable=0;step();
 memcpy(motors,bench_motors,sizeof(motors));g_dual_hold=bench_hold;
 /* Mode-handoff regression: supported body, constant leg length, displaced
  * from the angle captured by dual hold. Enabling balance must retain hold. */
 const BalanceDebug handoff_balance=g_balance;
 const ImuSample handoff_imu=imu;
 g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
 imu.attitude_deg[0]=0;
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=0;
 for(unsigned i=0;i<8;i++) motors[i].joint_speed_rad_s=motors[i].speed_rad_s=0;
 for(unsigned moved_pose=0;moved_pose<2;moved_pose++) {
  float angle=1.57079632679f+.1f*moved_pose,pose[2];
  assert(leg_inverse(.03f+.11f*cosf(angle),.11f*sinf(angle),1.57f,1.57f,pose));
  for(unsigned side=0;side<2;side++) {
   motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=pose[1];
   motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=pose[0];
  }
  g_dual_hold.enable=1;step();assert(g_dual_hold.status==1);
 }
 int16_t before_balance[4];
 for(unsigned i=0;i<4;i++) {
  before_balance[i]=g_joint_output.voltage_mv[i];assert(before_balance[i]!=0);
 }
 g_balance.enable=1;step();
 assert(g_dual_hold.status==1 && g_balance.status==1);
 printf("handoff joints: %d %d %d %d -> %d %d %d %d; LQR Tp=%g Nm\n",
  before_balance[0],before_balance[1],before_balance[2],before_balance[3],
  g_joint_output.voltage_mv[0],g_joint_output.voltage_mv[1],
  g_joint_output.voltage_mv[2],g_joint_output.voltage_mv[3],g_balance.torque_nm[1]);
 fflush(stdout);
 for(unsigned i=0;i<4;i++) assert(g_joint_output.voltage_mv[i]!=0);
 /* Blend diagnostic terms, preserve the reference through repeated toggles,
  * and verify the legacy pure LQR behavior remains explicitly selectable. */
 float handoff_ref[2]={g_dual_hold.angle_reference_rad[0],g_dual_hold.angle_reference_rad[1]};
 const float hold_ratios[]={0,.5f,1};
 for(unsigned n=0;n<3;n++) {
  g_balance.leg_hold_ratio=hold_ratios[n];step();check_joint_commands();
  for(unsigned side=0;side<2;side++) {
   assert(g_dual_hold.angle_reference_rad[side]==handoff_ref[side]);
   assert(fabsf(g_dual_hold.angle_hold_torque_nm[side]+.01f*hold_ratios[n])<1e-6f);
   assert(g_dual_hold.lqr_pitch_torque_nm[side]==g_balance.torque_nm[1]);
   assert(g_dual_hold.pitch_torque_nm[side]==g_dual_hold.angle_hold_torque_nm[side]+g_dual_hold.lqr_pitch_torque_nm[side]);
  }
   for(unsigned i=0;i<4;i++) {
    /* leg_hold_ratio=0 selects pure LQR swing torque; it must not suppress
     * the LQR term itself.  This fixture has a deliberately non-zero Tp. */
    assert(g_joint_output.voltage_mv[i]!=0);
    if(n>0) assert(g_joint_output.voltage_mv[i]*before_balance[i]>0);
   }
  g_balance.enable=0;step();check_joint_commands();
  for(unsigned i=0;i<4;i++) assert(g_joint_output.voltage_mv[i]==before_balance[i]);
  g_balance.enable=1;step();
 }
 /* Length support is retained as well, with zero swing gains isolating the
  * axial term and leg_scale=0 isolating the balance handoff. */
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
 g_dual_hold.position_kp=5;g_dual_hold.speed_kp=5;g_dual_hold.filter_tau_s=0;
 g_dual_hold.target_m=.12f;g_balance.leg_scale=0;
 g_balance.enable=0;step();
 float handoff_force[2]={g_dual_hold.force_n[0],g_dual_hold.force_n[1]};
 for(unsigned i=0;i<4;i++) {
  before_balance[i]=g_joint_output.voltage_mv[i];assert(before_balance[i]!=0);
 }
 g_balance.enable=1;step();check_joint_commands();
 for(unsigned side=0;side<2;side++) assert(g_dual_hold.force_n[side]==handoff_force[side]);
 for(unsigned i=0;i<4;i++) assert(g_joint_output.voltage_mv[i]==before_balance[i]);
 g_dual_hold.enable=0;step();
 g_balance=handoff_balance;imu=handoff_imu;
 memcpy(motors,bench_motors,sizeof(motors));g_dual_hold=bench_hold;
 /* Opposing initial angles are invisible to the shared LQR mean. First
  * reproduce independent capture preserving the mismatch, then check that
  * neutral mode corrects each side toward the same target without a snap. */
 g_dual_hold=boot_hold;g_dual_hold.feedforward_rate_n_s=100000;
 g_dual_hold.overlength_startup_s=0; /* Isolate angle-mode handoff from the axial-force startup ramp. */
 g_dual_hold.sync_ratio=0; /* Reproduce the old missing-sync behavior. */
 g_dual_hold.angle_reference_mode=LEG_ANGLE_CAPTURE;
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=0;
 imu.attitude_deg[BODY_PITCH_AXIS]=0;
 for(unsigned i=0;i<8;i++) motors[i].joint_speed_rad_s=motors[i].speed_rad_s=0;
 const float initial_angles[]={1.57079632679f-.12f,1.57079632679f+.12f};
 const float initial_lengths[]={.07f,.09f};
 for(unsigned side=0;side<2;side++) {
  float pose[2];assert(leg_inverse(.03f+initial_lengths[side]*cosf(initial_angles[side]),
                                 initial_lengths[side]*sinf(initial_angles[side]),1.57f,1.57f,pose));
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=pose[0];
 }
 g_dual_hold.enable=1;g_balance.enable=1;
 for(unsigned i=0;i<150;i++) step();
 assert(g_dual_hold.status==1 && g_balance.status==1);
 assert(fabsf(g_balance.state[0])<1e-6f);
 assert(fabsf(g_balance.torque_nm[0])<1e-6f && fabsf(g_balance.torque_nm[1])<1e-6f);
 assert(fabsf(g_dual_hold.angle_reference_rad[0]-g_dual_hold.angle_reference_rad[1])>.23f);
 for(unsigned side=0;side<2;side++) assert(g_dual_hold.angle_hold_torque_nm[side]==0);
 const float neutral_force[2]={g_dual_hold.force_n[0],g_dual_hold.force_n[1]};
 g_dual_hold.enable=0;step();g_dual_hold.angle_reference_mode=LEG_ANGLE_NEUTRAL;
 g_dual_hold.enable=1;g_balance.enable=1;step();
 for(unsigned side=0;side<2;side++) {
  assert(g_dual_hold.angle_reference_rad[side]==g_wheel_leg.leg[side].angle);
  assert(g_dual_hold.angle_hold_torque_nm[side]==0);
 }
 for(unsigned n=0;n<150;n++) {
  float old_ref[2]={g_dual_hold.angle_reference_rad[0],g_dual_hold.angle_reference_rad[1]};
  /* Balance toggles and differing hand-supported body pitch must not recapture
   * two different angle targets or restart the neutral slew. */
  if(n==20) g_balance.enable=0;
  if(n==40) g_balance.enable=1;
  imu.attitude_deg[BODY_PITCH_AXIS]=n<50?0:n<100?-5:5;
  step();assert(g_dual_hold.status==1);check_joint_commands();
  for(unsigned side=0;side<2;side++) {
   float ref=g_dual_hold.angle_reference_rad[side],target=g_dual_hold.angle_target_rad;
   assert(fabsf(ref-old_ref[side])<=.001001f);
   assert(fabsf(ref-target)<=fabsf(old_ref[side]-target)+1e-7f);
   assert(g_dual_hold.angle_hold_torque_nm[side]*(target-initial_angles[side])>=0);
   assert(fabsf(g_dual_hold.angle_hold_torque_nm[side])<=boot_hold.angle_limit_nm);
   if(n>=100) assert(fabsf(g_dual_hold.force_n[side]-neutral_force[side])<1e-6f);
  }
 }
 for(unsigned side=0;side<2;side++)
  assert(g_dual_hold.angle_reference_rad[side]==g_dual_hold.angle_target_rad);
 assert(g_dual_hold.angle_hold_torque_nm[0]>.011f && g_dual_hold.angle_hold_torque_nm[1]<-.011f);
 printf("PASS: opposing initial legs converge to common 90 deg references; bounded slew, opposite PD, unchanged axial support\n");
 /* Changing the configured common target also slews, rather than stepping. */
 g_dual_hold.angle_target_rad+=.02f;
 float previous_ref=g_dual_hold.angle_reference_rad[0];step();
 assert(g_dual_hold.angle_reference_rad[0]>previous_ref &&
        g_dual_hold.angle_reference_rad[0]-previous_ref<=.001001f);
 g_dual_hold.enable=0;step();
 for(unsigned side=0;side<2;side++) assert(g_dual_hold.angle_reference_rad[side]==0);
 /* Invalid neutral-reference configuration must retain fail-closed output. */
 const float bad_angle_targets[]={0,3.141592653589793f,NAN,INFINITY};
 for(unsigned i=0;i<4;i++) {
  g_dual_hold.angle_target_rad=bad_angle_targets[i];g_dual_hold.enable=1;step();
  assert(g_dual_hold.status==3 && !g_dual_hold.enable);
  for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
 }
 g_dual_hold.angle_target_rad=boot_hold.angle_target_rad;
 const float bad_reference_rates[]={0,-1,1.01f,NAN,INFINITY};
 for(unsigned i=0;i<5;i++) {
  g_dual_hold.angle_reference_rate_rad_s=bad_reference_rates[i];g_dual_hold.enable=1;step();
  assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 }
 g_dual_hold.angle_reference_rate_rad_s=boot_hold.angle_reference_rate_rad_s;
 g_dual_hold.angle_reference_mode=(LegAngleReferenceMode)2;g_dual_hold.enable=1;step();
 assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 g_balance=handoff_balance;imu=handoff_imu;
 memcpy(motors,bench_motors,sizeof(motors));g_dual_hold=bench_hold;
 verify_load_adaptation(boot_hold);
 verify_pitch_trim_position_window(boot_hold);
 verify_swing_sync(boot_hold);
 verify_sync_recovery_priority(boot_hold);
 verify_feedback_only();
 verify_staged_startup(boot_hold);
 verify_remote_ch5(boot_hold);
 verify_pose_guard(boot_hold);
 verify_ch5_fixed_parking(boot_hold);
 /* Explicitly opt into the original full reference actuator calibration for
  * the reference-equation/clamp tests below; boot defaults stay gentle. */
 g_dual_hold.feedforward_target_n=5.81733f;
 g_dual_hold.voltage_limit_mv=6900;
 g_dual_hold.speed_kp=20;g_dual_hold.max_speed_m_s=.1f;
 g_dual_hold.back_emf_ratio=1;
 g_dual_hold.angle_kp_nm_per_rad=g_dual_hold.angle_kd_nm_s_per_rad=0;
#endif
 g_dual_hold.enable=1;step();
#if MOTOR_COMMAND_TX_ENABLED
 assert(g_dual_hold.status==1 && fabsf(g_dual_hold.feedforward_n-.01f)<1e-6f);
 for(unsigned i=0;i<20;i++) step();
 assert(fabsf(g_dual_hold.feedforward_n-.21f)<1e-5f);
 assert(g_dual_hold.voltage_limit_mv==6900);
 for(unsigned i=0;i<8;i++) assert(abs(g_dual_hold.voltage_mv[i])<=6900);
 check_joint_commands();
 assert(g_dual_hold.voltage_mv[2]==0 && g_dual_hold.voltage_mv[6]==0);
 for(unsigned side=0;side<2;side++) for(unsigned j=0;j<2;j++)
  assert(fabsf(g_dual_hold.joint_torque_nm[2*side+j]-g_wheel_leg.leg[side].jacobian[1-j]*g_dual_hold.force_n[side])<1e-6f);
 for(unsigned i=0;i<4;i++) g_dual_hold.torque_ratio_nm_per_v[i]=1e-6f;
 step();
 int peak=0;
 for(unsigned i=0;i<8;i++) {
  int magnitude=abs(g_dual_hold.voltage_mv[i]);
  assert(magnitude<=6900);if(magnitude>peak) peak=magnitude;
 }
 assert(peak>6200 && g_dual_hold.voltage_mv[2]==0 && g_dual_hold.voltage_mv[6]==0);
 const float joint_ratios[4]={.0316f,.0317f,.0299f,.0321f};
 for(unsigned i=0;i<4;i++) g_dual_hold.torque_ratio_nm_per_v[i]=joint_ratios[i];
 /* Compare the four ESP32 actuator equations across supplies, signed speeds,
  * torque directions, length hold and full LQR/VMC. */
 MotorFeedback joint_saved_motors[8];memcpy(joint_saved_motors,motors,sizeof(motors));
 const DualHoldDebug joint_saved_hold=g_dual_hold;
 const float joint_supplies[]={0,9,12,14};
 const float joint_speeds[]={-20,-5,0,5,20};
 for(unsigned supply=0;supply<4;supply++) for(unsigned si=0;si<5;si++) {
  g_wheel_output.supply_voltage_v=joint_supplies[supply];
  g_dual_hold.target_m=si%2?.04f:.14f;
  for(unsigned i=0;i<4;i++) {
   motors[g_joint_calibration[i].motor_id-1].joint_speed_rad_s=joint_speeds[(si+i)%5];
   g_dual_hold.direction[i]=si%2?-joint_saved_hold.direction[i]:joint_saved_hold.direction[i];
  }
  step();assert(g_dual_hold.status==1);check_joint_commands();
 }
 /* Partial compensation scales only EMF; torque-to-voltage calibration stays. */
 const float emf_ratios[]={0,.1f,.5f,1};
 for(unsigned i=0;i<4;i++) {
  g_dual_hold.back_emf_ratio=emf_ratios[i];step();check_joint_commands();
 }
 /* With zero requested force/Tp, compensation follows shaft motion. */
 g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
 g_dual_hold.feedforward_target_n=0;
 g_dual_hold.feedforward_rate_n_s=100000;
 step();check_joint_commands();
 for(unsigned i=0;i<4;i++) assert(g_dual_hold.joint_torque_nm[i]==0);
 memcpy(motors,joint_saved_motors,sizeof(motors));g_dual_hold=joint_saved_hold;
 g_wheel_output.supply_voltage_v=12;
 /* Lower configured clamp is respected after compensation as well. */
 g_dual_hold.voltage_limit_mv=500;step();check_joint_commands();
 for(unsigned i=0;i<4;i++) assert(abs(g_joint_output.voltage_mv[i])<=500);
 g_dual_hold.voltage_limit_mv=6900;
 motors[JOINT_RF_CAN_ID-1].online=0;step();
 assert(g_dual_hold.status==2 && !g_dual_hold.enable && g_dual_hold.feedforward_n==0);
 assert(g_joint_output.output_ratio==0);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 motors[JOINT_RF_CAN_ID-1].online=1;
#else
 assert(g_dual_hold.status==5 && !g_dual_hold.enable);
#endif
 for(unsigned i=0;i<4;i++) step();
 g_dual_hold.target_m=NAN;g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 g_dual_hold.target_m=.11f;
 for(unsigned i=0;i<4;i++) step();
#if MOTOR_COMMAND_TX_ENABLED
 /* Bad conversion parameters must stop all slots rather than leave stale EMF. */
 const float bad_joint_ratios[]={0,-1,NAN,INFINITY};
 for(unsigned i=0;i<4;i++) {
  g_dual_hold.torque_ratio_nm_per_v[i]=bad_joint_ratios[i];
  g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
  for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
  assert(g_joint_output.output_ratio==0);
  g_dual_hold.torque_ratio_nm_per_v[i]=joint_ratios[i];
  for(unsigned j=0;j<4;j++) step();
 }
 g_dual_hold.voltage_limit_mv=JOINT_VOLTAGE_LIMIT_MV+1;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 g_dual_hold.voltage_limit_mv=JOINT_VOLTAGE_LIMIT_MV;
 const float bad_supplies[]={-1,NAN,19,INFINITY};
 for(unsigned i=0;i<4;i++) {
  g_wheel_output.supply_voltage_v=bad_supplies[i];
  g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
  for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
 }
 g_wheel_output.supply_voltage_v=12;
 for(unsigned j=0;j<4;j++) step();
 const float bad_emf_ratios[]={-.1f,1.1f,NAN,INFINITY};
 for(unsigned i=0;i<4;i++) {
  g_dual_hold.back_emf_ratio=bad_emf_ratios[i];
  g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
  for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
  assert(g_joint_output.output_ratio==0);
 }
 g_dual_hold.back_emf_ratio=1;
 /* Reject malformed swing-hold gains/limits before any output. */
 g_dual_hold.angle_kp_nm_per_rad=NAN;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 g_dual_hold.angle_kp_nm_per_rad=0;g_dual_hold.angle_kd_nm_s_per_rad=-1;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 g_dual_hold.angle_kd_nm_s_per_rad=0;g_dual_hold.angle_limit_nm=INFINITY;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 g_dual_hold.angle_limit_nm=.02f;
 for(unsigned j=0;j<4;j++) step();
 /* Nonfinite/stale joint rates cannot be used for active compensation. */
 MotorFeedback rate_saved=motors[JOINT_RR_CAN_ID-1];
 motors[JOINT_RR_CAN_ID-1].joint_speed_rad_s=NAN;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==2 && !g_dual_hold.enable);
 for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
 motors[JOINT_RR_CAN_ID-1]=rate_saved;
 motors[JOINT_RR_CAN_ID-1].age_ms=11;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==2 && !g_dual_hold.enable);
 for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
 motors[JOINT_RR_CAN_ID-1]=rate_saved;
 motors[JOINT_RR_CAN_ID-1].joint_speed_rad_s=1e30f;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable);
 assert(g_joint_output.output_ratio==0);
 for(unsigned j=0;j<8;j++) assert(g_dual_hold.voltage_mv[j]==0);
 /* Disabling EMF bypasses its polynomial; unrelated finite rate feedback
  * must not make a supposedly disabled feedforward term become NaN. */
 g_dual_hold.back_emf_ratio=0;
 g_dual_hold.enable=1;step();assert(g_dual_hold.status==1);
 for(unsigned j=0;j<4;j++) assert(g_joint_output.back_emf_mv[j]==0);
 g_dual_hold.enable=0;step();g_dual_hold.back_emf_ratio=1;
 motors[JOINT_RR_CAN_ID-1]=rate_saved;
 for(unsigned j=0;j<4;j++) step();
#endif
 calls=0;

 assert(g_balance.enable==0);
 assert(g_balance.wheel_scale==BALANCE_DEFAULT_WHEEL_SCALE && g_balance.leg_scale==BALANCE_DEFAULT_LEG_SCALE &&
        g_balance.back_emf_ratio==BALANCE_DEFAULT_BACK_EMF_RATIO);
 /* Explicit historical control fixture, independent of user tuning defaults. */
 g_balance.wheel_scale=.8f;g_balance.back_emf_ratio=0;
 assert(g_balance.leg_hold_ratio==1);
 assert(g_wheel_output.torque_ratio_nm_per_v[0]==WHEEL_LEFT_TORQUE_RATIO);
 assert(g_wheel_output.torque_ratio_nm_per_v[1]==WHEEL_RIGHT_TORQUE_RATIO);
 assert(g_wheel_output.supply_voltage_v==12);
 assert(g_balance.wheel_limit_mv==6000.0f && WHEEL_VOLTAGE_LIMIT_MV==6000.0f);
 assert(g_wheel_test.scale==1 && g_wheel_test.wheel_limit_mv==6000);
#if MOTOR_COMMAND_TX_ENABLED
 g_dual_hold.angle_kp_nm_per_rad=.1f;g_dual_hold.angle_kd_nm_s_per_rad=.01f;
 /* Full-balance bench defaults disable wheel EMF and clamp torque commands,
  * without changing the established wheel-only test defaults. */
 g_balance.enable=1;g_dual_hold.enable=1;step();
 assert(g_balance.status==1 && g_dual_hold.status==1);
 assert(g_wheel_output.back_emf_blend==0);
 for(unsigned side=0;side<2;side++) {
  assert(g_wheel_output.back_emf_mv[side]==0);
  assert(abs(g_wheel_output.voltage_mv[side])<=6000);
 }
 float component_sum=0;
 for(unsigned i=0;i<6;i++) component_sum+=g_balance.wheel_torque_component_nm[i];
 assert(fabsf(component_sum-g_balance.torque_nm[0])<1e-6f);
 g_dual_hold.enable=0;step();
 for(unsigned i=0;i<6;i++) assert(g_balance.wheel_torque_component_nm[i]==0);
 /* Opt into the reference's full actuator output for existing clamp tests. */
 g_balance.leg_hold_ratio=0;
 g_balance.wheel_scale=g_balance.leg_scale=1;g_balance.wheel_limit_mv=4000;g_balance.back_emf_ratio=1;
 g_balance.enable=1;g_dual_hold.enable=1;step();
 assert(g_balance.status==1 && g_dual_hold.status==1);
 check_joint_commands();
 assert(g_balance.reference[2]==g_balance.state[2]);
 float bx[6],br[6],bu[2];
 for(unsigned i=0;i<6;i++) {
  bx[i]=g_balance.state[i];br[i]=g_balance.reference[i];
  assert(fabsf(bx[i]-.5f*(g_wheel_leg.state[0][i]+g_wheel_leg.state[1][i]))<1e-6f);
 }
 assert(lqr_control(g_balance.leg_length_m,bx,br,bu));
 assert(fabsf(g_balance.torque_nm[0]-bu[0]*g_balance.wheel_scale)<1e-6f);
 assert(fabsf(g_balance.torque_nm[1]-fmaxf(-BALANCE_LEG_LQR_TORQUE_LIMIT_NM,
      fminf(BALANCE_LEG_LQR_TORQUE_LIMIT_NM,bu[1]*g_balance.leg_scale)))<1e-6f);
 const float balance_emf_ratios[]={0,.1f,.5f,1};
 for(unsigned n=0;n<4;n++) {
  g_balance.back_emf_ratio=balance_emf_ratios[n];step();
  assert(g_balance.status==1 && g_dual_hold.status==1);
  assert(g_wheel_output.back_emf_blend==balance_emf_ratios[n]);
  float sum=0;
  for(unsigned i=0;i<6;i++) sum+=g_balance.wheel_torque_component_nm[i];
  assert(fabsf(sum-g_balance.torque_nm[0])<1e-6f);
  for(unsigned side=0;side<2;side++) {
   float speed=(side?1:-1)*motors[(side?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1].speed_rad_s;
   float s=fabsf(speed),emf=1000*(.000004f*s*s*s-.0003f*s*s+.0266f*s);
   if(speed<0) emf=-emf;
   assert(fabsf(g_wheel_output.back_emf_mv[side]-balance_emf_ratios[n]*emf)<.002f);
   float tv=g_balance.torque_nm[0]/(side?WHEEL_RIGHT_TORQUE_RATIO:WHEEL_LEFT_TORQUE_RATIO)*.7f*1000;
   assert(fabsf(g_wheel_output.torque_voltage_mv[side]-tv)<.002f);
  }
 }
 for(unsigned side=0;side<2;side++) {
  assert(g_dual_hold.pitch_torque_nm[side]==g_balance.torque_nm[1]);
  assert(g_dual_hold.angle_hold_torque_nm[side]==0);
  assert(g_dual_hold.lqr_pitch_torque_nm[side]==g_balance.torque_nm[1]);
 }
 assert(g_dual_hold.voltage_mv[2]==-g_dual_hold.voltage_mv[6]);
 for(unsigned s=0;s<2;s++) for(unsigned j=0;j<2;j++) {
  float expected=g_wheel_leg.leg[s].jacobian[1-j]*g_dual_hold.force_n[s]+
                 g_wheel_leg.leg[s].jacobian[3-j]*g_balance.torque_nm[1];
  assert(fabsf(g_dual_hold.joint_torque_nm[2*s+j]-expected)<1e-6f);
 }
 step();
 assert(abs(g_dual_hold.voltage_mv[2])==4000 && g_balance.saturated);
 g_balance.enable=0;step();
 assert(g_dual_hold.enable && g_dual_hold.voltage_mv[2]==0 && g_dual_hold.voltage_mv[6]==0);
 for(unsigned side=0;side<2;side++)
  assert(g_dual_hold.angle_reference_rad[side]==g_wheel_leg.leg[side].angle);
 g_balance.enable=1;motors[WHEEL_RIGHT_CAN_ID-1].online=0;step();
 assert(!g_dual_hold.enable && !g_balance.enable && g_dual_hold.status==2);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 motors[WHEEL_RIGHT_CAN_ID-1].online=1;
 for(unsigned i=0;i<4;i++) step();
 g_balance.enable=1;g_balance.wheel_scale=NAN;g_dual_hold.enable=1;step();
 assert(g_dual_hold.status==3 && !g_dual_hold.enable && !g_balance.enable);
 g_balance.wheel_scale=1; /* Explicit full-output reference fixture. */
 const float bad_hold_ratios[]={-.1f,1.1f,NAN,INFINITY};
 for(unsigned n=0;n<4;n++) {
  g_balance.leg_hold_ratio=bad_hold_ratios[n];
  g_dual_hold.enable=1;g_balance.enable=1;step();
  assert(g_dual_hold.status==3 && !g_dual_hold.enable && !g_balance.enable);
  for(unsigned side=0;side<2;side++) {
   assert(g_dual_hold.angle_hold_torque_nm[side]==0);
   assert(g_dual_hold.lqr_pitch_torque_nm[side]==0);
  }
  for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 }
 g_balance.leg_hold_ratio=0;
 const float bad_scales[]={-.1f,1.1f,NAN,INFINITY};
 for(unsigned channel=0;channel<2;channel++) for(unsigned n=0;n<4;n++) {
  if(channel) g_balance.leg_scale=bad_scales[n];
  else g_balance.wheel_scale=bad_scales[n];
  g_dual_hold.enable=1;g_balance.enable=1;step();
  assert(g_dual_hold.status==3 && !g_dual_hold.enable && !g_balance.enable);
  for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
  g_balance.wheel_scale=g_balance.leg_scale=1;
 }
 const float bad_balance_emf[]={-.1f,1.1f,NAN,INFINITY};
 for(unsigned n=0;n<4;n++) {
  g_balance.back_emf_ratio=bad_balance_emf[n];
  g_dual_hold.enable=1;g_balance.enable=1;step();
  assert(g_dual_hold.status==3 && !g_dual_hold.enable && !g_balance.enable);
  for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
  for(unsigned i=0;i<6;i++) assert(g_balance.wheel_torque_component_nm[i]==0);
 }
 g_balance.back_emf_ratio=1;
 for(unsigned i=0;i<4;i++) step();
 /* A disabled wheel feedforward polynomial stays zero even at very large
  * finite speed. Enabling an overflowing fit disarms all actuation and clears
  * torque contributions along with the reported wheel torque. */
 MotorFeedback overflow_wheels[2]={motors[WHEEL_LEFT_CAN_ID-1],motors[WHEEL_RIGHT_CAN_ID-1]};
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=1e30f;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=-1e30f;
 g_balance.back_emf_ratio=0;g_balance.enable=1;g_dual_hold.enable=1;step();
 assert(g_balance.status==1 && g_dual_hold.status==1);
 assert(g_wheel_output.back_emf_mv[0]==0 && g_wheel_output.back_emf_mv[1]==0);
 g_balance.back_emf_ratio=1;step();assert(g_dual_hold.status==3 && !g_dual_hold.enable && !g_balance.enable);
 assert(g_balance.torque_nm[0]==0);
 for(unsigned i=0;i<6;i++) assert(g_balance.wheel_torque_component_nm[i]==0);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 motors[WHEEL_LEFT_CAN_ID-1]=overflow_wheels[0];
 motors[WHEEL_RIGHT_CAN_ID-1]=overflow_wheels[1];
 /* Reset observation integration after this synthetic rate stress. */
 motors[WHEEL_LEFT_CAN_ID-1].online=motors[WHEEL_RIGHT_CAN_ID-1].online=0;step();
 motors[WHEEL_LEFT_CAN_ID-1].online=motors[WHEEL_RIGHT_CAN_ID-1].online=1;
 for(unsigned i=0;i<4;i++) step();
 /* Reject requests above the wheel driver's physical voltage ceiling. */
 g_balance.wheel_limit_mv=WHEEL_VOLTAGE_LIMIT_MV+1;
 g_dual_hold.enable=1;g_balance.enable=1;step();
 assert(g_dual_hold.status==3 && !g_dual_hold.enable && !g_balance.enable);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 g_balance.wheel_limit_mv=WHEEL_VOLTAGE_LIMIT_MV;
 for(unsigned i=0;i<4;i++) step();
 /* Stationary, vertical 0.11 m legs: +/-10 deg must produce opposite wheel
  * commands at the requested 6 V clamp. Verify the CAN
  * payload as well as the watch value, then restore the original inputs. */
 MotorFeedback saved_motors[8];memcpy(saved_motors,motors,sizeof(motors));
 ImuSample saved_imu=imu;
 float vertical[2];assert(leg_inverse(.03f,.11f,1.57f,1.57f,vertical));
 for(unsigned joint=0;joint<4;joint++) {
  MotorFeedback *f=&motors[g_joint_calibration[joint].motor_id-1];
  f->joint_angle_rad=vertical[joint%2==0];f->joint_speed_rad_s=0;
 }
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=0;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 imu.gyro_rad_s[0]=imu.gyro_rad_s[1]=imu.gyro_rad_s[2]=0;
 for(int lean=-10;lean<=10;lean+=10) {
  g_dual_hold.enable=0;step();
  imu.attitude_deg[BODY_PITCH_AXIS]=(float)lean;
  g_dual_hold.enable=1;g_balance.enable=1;step();
  assert(g_dual_hold.status==1 && g_balance.status==1);
  int left=g_dual_hold.voltage_mv[WHEEL_LEFT_CAN_ID-1];
  int right=g_dual_hold.voltage_mv[WHEEL_RIGHT_CAN_ID-1];
  assert(left==-right);
  if(lean) {
   assert(abs(right)==6000);
   assert((lean<0 && right>0) || (lean>0 && right<0));
  } else assert(abs(right)<=1);
  for(unsigned side=0;side<2;side++) {
   unsigned slot=(side?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1;
   uint16_t encoded=(uint16_t)data[slot/4][2*(slot%4)]|
                    ((uint16_t)data[slot/4][2*(slot%4)+1]<<8);
   assert(encoded==(uint16_t)g_dual_hold.voltage_mv[slot]);
  }
 }
 /* Stronger wheel fixture: the same 1 deg lean doubles the previous request
  * without changing LQR leg torque. Also check signed CAN and both clamps. */
 g_dual_hold.enable=0;step();
 g_balance.wheel_scale=.8f;
 g_balance.leg_scale=BALANCE_DEFAULT_LEG_SCALE;
 g_balance.wheel_limit_mv=BALANCE_DEFAULT_WHEEL_LIMIT_MV;
 g_balance.back_emf_ratio=0;
 imu.attitude_deg[BODY_PITCH_AXIS]=-1;
 g_dual_hold.enable=1;g_balance.enable=1;step();
 assert(g_dual_hold.status==1 && g_balance.status==1);
 float updated_model_u[2];
 const BalanceDebug updated_model_balance=g_balance;
 assert(lqr_control(updated_model_balance.leg_length_m,updated_model_balance.state,updated_model_balance.reference,updated_model_u));
 assert(updated_model_u[0]>0);
 assert(fabsf(g_balance.torque_nm[0]-updated_model_u[0]*.8f)<1e-7f);
 assert(g_wheel_output.back_emf_mv[0]==0 && g_wheel_output.back_emf_mv[1]==0);
 assert(fabsf(g_dual_hold.voltage_mv[2]+g_balance.torque_nm[0]/WHEEL_LEFT_TORQUE_RATIO*.7f*1000)<1.1f);
 assert(fabsf(g_dual_hold.voltage_mv[6]-g_balance.torque_nm[0]/WHEEL_RIGHT_TORQUE_RATIO*.7f*1000)<1.1f);
 const int stronger_wheels[2]={g_dual_hold.voltage_mv[2],g_dual_hold.voltage_mv[6]};
 const float stronger_leg_torque=g_balance.torque_nm[1];
 /* Freeze the axial request to compare joint output at an identical state. */
 const DualHoldDebug strength_hold=g_dual_hold;
 g_dual_hold.feedforward_target_n=g_dual_hold.feedforward_n;
 g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
 g_dual_hold.overlength_kp_n_per_m=0;
 step();
 int16_t stronger_joints[4];
 for(unsigned i=0;i<4;i++) stronger_joints[i]=g_joint_output.voltage_mv[i];
 g_balance.wheel_scale=.4f;g_balance.wheel_limit_mv=1500;step();
 for(unsigned side=0;side<2;side++)
  assert(abs(stronger_wheels[side]-2*g_wheel_output.voltage_mv[side])<=1);
 assert(g_balance.torque_nm[1]==stronger_leg_torque);
 for(unsigned i=0;i<4;i++) assert(g_joint_output.voltage_mv[i]==stronger_joints[i]);
 g_balance.wheel_scale=BALANCE_DEFAULT_WHEEL_SCALE;
 g_balance.wheel_limit_mv=BALANCE_DEFAULT_WHEEL_LIMIT_MV;
 for(int lean=-10;lean<=10;lean+=20) {
  imu.attitude_deg[BODY_PITCH_AXIS]=(float)lean;step();
  assert(g_dual_hold.status==1 && g_balance.status==1 && g_wheel_output.saturated);
  assert(abs(g_wheel_output.voltage_mv[0])==6000 && abs(g_wheel_output.voltage_mv[1])==6000);
  assert((lean<0 && g_wheel_output.voltage_mv[1]>0) || (lean>0 && g_wheel_output.voltage_mv[1]<0));
  for(unsigned side=0;side<2;side++) {
   unsigned slot=(side?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1;
   uint16_t encoded=(uint16_t)data[slot/4][2*(slot%4)]|
                    ((uint16_t)data[slot/4][2*(slot%4)+1]<<8);
   assert(encoded==(uint16_t)g_wheel_output.voltage_mv[side]);
  }
 }
 imu.attitude_deg[BODY_PITCH_AXIS]=-1;step();
 g_dual_hold=strength_hold;
 /* Isolate the new wheel-rate scaling with identical nonzero gyro feedback.
  * Position/angle contributions and leg Tp must remain unchanged. */
 float saved_gyro=imu.gyro_rad_s[BODY_PITCH_AXIS];
 imu.gyro_rad_s[BODY_PITCH_AXIS]=.5f;g_balance.wheel_rate_scale=1;step();
 BalanceDebug full_rate=g_balance;
 g_balance.wheel_rate_scale=.5f;step();
 float scaled_sum=0;
 for(unsigned i=0;i<6;i++) {
  float expected=full_rate.wheel_torque_component_nm[i]*((i==1 || i==5)?.5f:1.0f);
  assert(fabsf(g_balance.wheel_torque_component_nm[i]-expected)<1e-6f);
  scaled_sum+=expected;
 }
 assert(fabsf(g_balance.torque_nm[0]-scaled_sum)<1e-6f);
 assert(fabsf(g_balance.torque_nm[1]-full_rate.torque_nm[1])<1e-7f);
 g_balance.wheel_rate_scale=NAN;step();assert(g_balance.status==3 && !g_dual_hold.enable);
 g_balance.wheel_rate_scale=1;imu.gyro_rad_s[BODY_PITCH_AXIS]=saved_gyro;
 g_dual_hold.enable=g_balance.enable=1;step();
 puts("PASS: wheel dtheta/dphi scaling, unchanged other terms/Tp, component sum and invalid-rate disarm");
 printf("PASS: stronger default wheels at 1 deg: %d/%d mV; 6 V clamp, signed CAN and unchanged joint output\n",
        stronger_wheels[0],stronger_wheels[1]);
 float fixed_leg_torque=g_balance.torque_nm[1];
 float scale_model_u[2];
 const BalanceDebug scale_model_balance=g_balance;
 assert(lqr_control(scale_model_balance.leg_length_m,scale_model_balance.state,scale_model_balance.reference,scale_model_u));
 const float wheel_scales[]={.1f,.2f,.4f,.6f,.8f};
 for(unsigned n=0;n<5;n++) {
  g_balance.wheel_scale=wheel_scales[n];step();
  assert(g_dual_hold.status==1 && g_balance.status==1);
  assert(fabsf(g_balance.torque_nm[1]-fixed_leg_torque)<1e-7f);
  assert(fabsf(g_balance.torque_nm[0]-scale_model_u[0]*wheel_scales[n])<1e-6f);
  float sum=0;
  for(unsigned i=0;i<6;i++) sum+=g_balance.wheel_torque_component_nm[i];
  assert(fabsf(sum-g_balance.torque_nm[0])<1e-6f);
  assert(g_dual_hold.voltage_mv[2]<0 && g_dual_hold.voltage_mv[6]>0);
 }
 /* Scaling only the leg must leave stationary wheel voltage untouched. */
 int left_before=g_dual_hold.voltage_mv[2],right_before=g_dual_hold.voltage_mv[6];
 const float higher_leg_scale=fminf(1,2*BALANCE_DEFAULT_LEG_SCALE);
 g_balance.leg_scale=higher_leg_scale;step();
 assert(g_dual_hold.voltage_mv[2]==left_before && g_dual_hold.voltage_mv[6]==right_before);
 assert(fabsf(g_balance.torque_nm[1]-fmaxf(-BALANCE_LEG_LQR_TORQUE_LIMIT_NM,
      fminf(BALANCE_LEG_LQR_TORQUE_LIMIT_NM,scale_model_u[1]*higher_leg_scale)))<1e-7f);
 /* The larger scale still obeys a reduced wheel-only voltage limit. */
 g_balance.wheel_limit_mv=300;step();
 assert(abs(g_dual_hold.voltage_mv[2])==300 && abs(g_dual_hold.voltage_mv[6])==300);
 assert(g_wheel_output.saturated);
 /* Replay the reported forward lean and its reverse at the support preset.
  * The current LQR swing preset must retain axial support, captured-angle hold and
  * wheel commands; the joint increment must be J_angle * delta_Tp. */
 const DualHoldDebug swing_hold=g_dual_hold;
 const BalanceDebug swing_balance=g_balance;
 g_dual_hold.position_kp=g_dual_hold.speed_kp=g_dual_hold.speed_ki=0;
 g_dual_hold.overlength_kp_n_per_m=0;
 g_dual_hold.feedforward_target_n=LEG_DEFAULT_SUPPORT_FORCE_N;g_dual_hold.feedforward_rate_n_s=100000;
 g_balance.wheel_scale=BALANCE_DEFAULT_WHEEL_SCALE;
 g_balance.wheel_limit_mv=BALANCE_DEFAULT_WHEEL_LIMIT_MV;
 g_balance.leg_hold_ratio=BALANCE_DEFAULT_LEG_HOLD_RATIO;
 const float swing_lengths[]={.07f,.11f};
 for(unsigned li=0;li<2;li++) for(int lean=-5;lean<=5;lean+=10) {
  g_dual_hold.enable=0;step();
  float pose[2];assert(leg_inverse(.03f,swing_lengths[li],1.57f,1.57f,pose));
  for(unsigned joint=0;joint<4;joint++)
   motors[g_joint_calibration[joint].motor_id-1].joint_angle_rad=pose[joint%2==0];
  g_dual_hold.target_m=swing_lengths[li];
  imu.attitude_deg[BODY_PITCH_AXIS]=(float)lean;
  g_balance.leg_scale=.20f;g_balance.enable=1;g_dual_hold.enable=1;step();
  assert(g_balance.status==1 && g_dual_hold.status==1);
  const DualHoldDebug old_swing=g_dual_hold;
  const float old_tp=g_balance.torque_nm[1];
  g_balance.leg_scale=BALANCE_DEFAULT_LEG_SCALE;step();check_joint_commands();
  assert(g_balance.status==1 && g_dual_hold.status==1);
  const float scale_ratio=BALANCE_DEFAULT_LEG_SCALE/.20f;
  assert(fabsf(g_balance.torque_nm[1]-scale_ratio*old_tp)<1e-7f);
  float swing_model_u[2];
  const BalanceDebug swing_model_balance=g_balance;
  assert(lqr_control(swing_model_balance.leg_length_m,swing_model_balance.state,swing_model_balance.reference,swing_model_u));
  assert(fabsf(g_balance.torque_nm[1]-swing_model_u[1]*BALANCE_DEFAULT_LEG_SCALE)<1e-7f);
  assert((lean<0 && g_balance.torque_nm[1]>0) || (lean>0 && g_balance.torque_nm[1]<0));
  for(unsigned side=0;side<2;side++) {
   assert(g_dual_hold.force_n[side]==old_swing.force_n[side]);
   assert(g_dual_hold.angle_hold_torque_nm[side]==old_swing.angle_hold_torque_nm[side]);
   assert(g_dual_hold.voltage_mv[(side?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1]==
          old_swing.voltage_mv[(side?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1]);
   for(unsigned joint=0;joint<2;joint++) {
    unsigned index=2*side+joint;
    float delta=g_wheel_leg.leg[side].jacobian[3-joint]*(scale_ratio-1)*old_tp;
    assert(fabsf(g_dual_hold.joint_torque_nm[index]-old_swing.joint_torque_nm[index]-delta)<1e-6f);
    assert(abs(g_joint_output.voltage_mv[index])<=6900);
   }
  }
  printf("PASS: L=%.2fm lean=%d deg: swing %g -> %g Nm, axial support/wheels unchanged\n",
         swing_lengths[li],-lean,old_tp,g_balance.torque_nm[1]);
 }
 g_dual_hold=swing_hold;g_balance=swing_balance;
 g_dual_hold.enable=0;step();
 g_balance.wheel_scale=g_balance.leg_scale=1;g_balance.wheel_limit_mv=4000;g_balance.back_emf_ratio=1;
 g_dual_hold.enable=0;step();
 memcpy(motors,saved_motors,sizeof(motors));imu=saved_imu;
 for(unsigned i=0;i<4;i++) step();
 g_dual_hold.enable=1;g_balance.enable=1;step();
 g_dual_hold.enable=0;step();
 assert(!g_balance.enable && g_dual_hold.status==0);
 assert(g_joint_output.output_ratio==0);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 for(unsigned f=0;f<2;f++) for(unsigned b=0;b<8;b++) assert(data[f][b]==0);
 for(unsigned i=0;i<4;i++) step();
 g_dual_hold.enable=1;g_balance.enable=1;step();
 fail_at=calls+1;step();
 assert(!g_dual_hold.enable && !g_balance.enable && g_dual_hold.status==4);
 assert(g_joint_output.output_ratio==0);
 fail_at=0;step();
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 for(unsigned i=0;i<4;i++) step();
 assert(g_dual_hold.status==4); /* Fault stays visible; no automatic restart. */
 calls=0;
#endif

 /* Combined output is armed/stopped with just the new master switch. It
  * must never rearm automatically after faults or wheel-only takeover. */
#if MOTOR_COMMAND_TX_ENABLED
 const DualHoldDebug combined_hold=g_dual_hold;
 const BalanceDebug combined_balance=g_balance;
 MotorFeedback combined_motors[8];memcpy(combined_motors,motors,sizeof(motors));
 const ImuSample combined_imu=imu;
 g_dual_hold=support_preset;g_dual_hold.enable=0;
 g_balance.enable=0;g_balance.wheel_scale=.4f;g_balance.leg_scale=.1f;
 g_balance.leg_hold_ratio=1;g_balance.wheel_limit_mv=1500;g_balance.back_emf_ratio=0;
 float combined_pose[2];assert(leg_inverse(.03f,.07f,1.57f,1.57f,combined_pose));
 for(unsigned i=0;i<8;i++) motors[i].joint_speed_rad_s=motors[i].speed_rad_s=0;
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=combined_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=combined_pose[0];
 }
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=0;
 /* An extended leg must not receive a full restoring-force step as soon
  * as it arms, while support feedforward is still almost zero. */
 assert(leg_inverse(.03f,.11f,1.57f,1.57f,combined_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=combined_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=combined_pose[0];
 }
 imu.attitude_deg[BODY_PITCH_AXIS]=0;g_balance.wheel_scale=g_balance.leg_scale=0;
 g_dual_hold.support_only=0;g_dual_hold.overlength_startup_s=0;g_robot_control.enable=1;step();
 assert(fabsf(g_dual_hold.force_n[0])>8);
 g_robot_control.enable=0;step();for(unsigned i=0;i<4;i++) step();
 g_dual_hold.overlength_startup_s=support_preset.overlength_startup_s;
 g_dual_hold.support_only=1;
 g_robot_control.enable=1;step();
 assert(g_robot_control.status==1 && fabsf(g_dual_hold.overlength_blend-.01f)<1e-6f);
 for(unsigned side=0;side<2;side++) {
  assert(fabsf(g_dual_hold.overlength_force_n[side]+.00034f*support_preset.overlength_kp_n_per_m)<1e-5f);
  assert(fabsf(g_dual_hold.force_n[side])<.2f);
  assert(g_dual_hold.force_n[side]==0 && g_dual_hold.force_requested_n[side]<0);
 }
 for(unsigned i=0;i<4;i++) assert(abs(g_joint_output.voltage_mv[i])<200);
 float first_start_force=g_dual_hold.force_n[0];
 for(unsigned i=0;i<99;i++) {
  float last=g_dual_hold.overlength_blend;step();
  assert(g_dual_hold.overlength_blend>=last && g_dual_hold.overlength_blend-last<.01001f);
 }
 assert(fabsf(g_dual_hold.overlength_blend-1)<1e-6f);
 g_robot_control.enable=0;step();assert(g_dual_hold.overlength_blend==0);
 /* Position-derived rate observes actual geometry independently of CAN
  * rate: known 0.5 mm compression over 5 ms is negative 0.1 m/s. */
 assert(leg_inverse(.03f,.1095f,1.57f,1.57f,combined_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=combined_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=combined_pose[0];
 }
 step();
 for(unsigned side=0;side<2;side++) {
  assert(g_wheel_leg.length_speed_position_valid[side]);
  assert(fabsf(g_wheel_leg.length_speed_from_position_m_s[side]+.1f)<1e-4f);
  assert(g_wheel_leg.length_speed_m_s[side]==0); /* Deliberately stale CAN rate fixture. */
 }
 motors[JOINT_LF_CAN_ID-1].online=0;step();
 assert(!g_wheel_leg.length_speed_position_valid[0]);
 motors[JOINT_LF_CAN_ID-1].online=1;step();
 assert(!g_wheel_leg.length_speed_position_valid[0]);
 step();assert(g_wheel_leg.length_speed_position_valid[0]);
 assert(leg_inverse(.03f,.07f,1.57f,1.57f,combined_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=combined_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=combined_pose[0];
 }
 g_balance.wheel_scale=.4f;g_balance.leg_scale=.1f;
 printf("PASS: overlength startup first force=%g N (<200mV), ramp/reset and independent position-rate diagnostics\n",first_start_force);
 imu.attitude_deg[BODY_PITCH_AXIS]=-2;
 unsigned combined_calls=calls;
 g_robot_control.enable=1;
 for(unsigned i=0;i<20;i++) step();
 assert(calls-combined_calls==40); /* One pair of CAN frames per cycle. */
 assert(g_robot_control.status==1 && g_dual_hold.enable && g_balance.enable);
 check_joint_commands();
 for(unsigned i=0;i<4;i++) assert(g_joint_output.voltage_mv[i]!=0);
 assert(g_dual_hold.voltage_mv[2]!=0 && g_dual_hold.voltage_mv[6]!=0);
 /* Tune length through the same switch while wheel/LQR swing scales are
  * zero: all joints retain support and wheel commands are true zero. */
 g_balance.wheel_scale=0;g_balance.leg_scale=0;step();
 assert(g_robot_control.enable && g_robot_control.status==1);
 assert(g_balance.torque_nm[0]==0 && g_balance.torque_nm[1]==0);
 assert(g_dual_hold.voltage_mv[2]==0 && g_dual_hold.voltage_mv[6]==0);
 for(unsigned i=0;i<4;i++) assert(g_joint_output.voltage_mv[i]!=0);
 check_joint_commands();
 /* Zero LQR and compensation means only leg support is requested. The
  * unused wheel/IMU feedback must not disarm a functioning length hold. */
 float unloaded_channels_force[2]={g_dual_hold.force_n[0],g_dual_hold.force_n[1]};
 motors[WHEEL_LEFT_CAN_ID-1].online=motors[WHEEL_RIGHT_CAN_ID-1].online=0;
 imu_ok=0;step();
 assert(g_robot_control.enable && g_robot_control.status==1);
 assert(g_dual_hold.enable && g_dual_hold.status==1 && g_balance.status==1);
 for(unsigned side=0;side<2;side++) {
  assert(g_wheel_leg.leg_valid[side] && !g_wheel_leg.state_valid[side]);
  assert(g_dual_hold.force_n[side]>=unloaded_channels_force[side]);
 }
 assert(g_dual_hold.voltage_mv[2]==0 && g_dual_hold.voltage_mv[6]==0);
 check_joint_commands();
 /* Re-enable a real LQR channel without those inputs: stop immediately. */
 g_balance.wheel_scale=.1f;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 motors[WHEEL_LEFT_CAN_ID-1].online=motors[WHEEL_RIGHT_CAN_ID-1].online=1;
 imu_ok=1;g_balance.wheel_scale=g_balance.leg_scale=0;
 g_robot_control.enable=1;step();
 motors[JOINT_LF_CAN_ID-1].age_ms=11;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
 assert(g_robot_control.last_fault.motor_id==JOINT_LF_CAN_ID);
 motors[JOINT_LF_CAN_ID-1].age_ms=0;
 /* Keep the leg envelope even in length-only mode, and do not mislabel
  * a valid overlong leg as an unused IMU/wheel feedback fault. */
 imu_ok=0;motors[WHEEL_LEFT_CAN_ID-1].online=0;
 axial_feedback(.151f,0);g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_LENGTH_RANGE);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 imu_ok=1;motors[WHEEL_LEFT_CAN_ID-1].online=1;axial_feedback(.07f,0);
 g_robot_control.enable=1;step();
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-4;motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=4;
 for(unsigned i=0;i<10;i++) step();
 assert(fabsf(g_wheel_leg.state[0][2])>.002f);
 g_balance.wheel_scale=.4f;g_balance.leg_scale=.1f;step();
 assert(g_robot_control.status==1 && g_balance.reference[2]==g_balance.state[2]);
 { extern volatile uint32_t g_robot_bench_limit_ms,g_robot_bench_elapsed_ms;
   g_robot_bench_limit_ms=10;step();step();step();
   assert(!g_robot_control.enable && g_robot_control.feedback_only && !g_robot_bench_limit_ms);
   assert(g_robot_bench_elapsed_ms>=10 && g_robot_bench_elapsed_ms<=15);
   g_robot_control.feedback_only=0;g_robot_control.enable=1;step();
 }

 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=0;
 puts("PASS: length-only hold ignores unused wheel/IMU feedback, retains joint/range faults and relatches real LQR origin");
 g_robot_control.enable=0;step();
 assert(g_robot_control.status==0 && !g_dual_hold.enable && !g_balance.enable);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 for(unsigned f=0;f<2;f++) for(unsigned b=0;b<8;b++) assert(data[f][b]==0);
 for(unsigned i=0;i<4;i++) step();
 g_robot_control.enable=1;step();
 motors[WHEEL_LEFT_CAN_ID-1].online=0;step();
 assert(!g_robot_control.enable && g_robot_control.status==2);
 assert(!g_dual_hold.enable && !g_balance.enable);
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_FEEDBACK);
 assert(g_robot_control.last_fault.motor_id==WHEEL_LEFT_CAN_ID);
 uint32_t fault_tick=g_robot_control.last_fault.tick_ms;
 uint32_t fault_count=g_robot_control.last_fault.count;
 motors[WHEEL_LEFT_CAN_ID-1].online=1;
 for(unsigned i=0;i<4;i++) step();
 assert(!g_robot_control.enable && g_robot_control.status==2);
 assert(g_robot_control.last_fault.tick_ms==fault_tick && g_robot_control.last_fault.count==fault_count);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 g_dual_hold.feedforward_target_n=NAN;g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.status==3);
 g_dual_hold.feedforward_target_n=support_preset.feedforward_target_n;
 /* 5000/6000 mV settings work through the master; >6000 still stops all slots. */
 const float saved_wheel_limit=g_balance.wheel_limit_mv;
 const float accepted_wheel_limits[]={5000,6000};
 for(unsigned i=0;i<2;i++) {
  g_balance.wheel_limit_mv=accepted_wheel_limits[i];g_robot_control.enable=1;step();
  assert(g_robot_control.enable && g_robot_control.status==1);
  assert(g_dual_hold.status==1 && g_balance.status==1);
 }
 g_balance.wheel_limit_mv=6001;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 g_balance.wheel_limit_mv=6000;step();assert(!g_robot_control.enable);
 g_robot_control.enable=1;step();assert(g_robot_control.status==1);
 g_robot_control.enable=0;step();g_balance.wheel_limit_mv=saved_wheel_limit;
 puts("PASS: master accepts 5/6 V settings, rejects >6 V and retains stop/rearm behavior");
 /* Height-triggered fault: preserve the valid geometry that exceeded the
  * fitted gain range, even after the observation returns to a normal pose. */
 assert(leg_inverse(.03f,.151f,1.57f,1.57f,combined_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=combined_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=combined_pose[0];
 }
 g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.status==2);
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_LENGTH_RANGE);
 for(unsigned side=0;side<2;side++) {
  assert(g_robot_control.last_fault.leg_valid[side]);
  assert(fabsf(g_robot_control.last_fault.leg_length_m[side]-.151f)<1e-6f);
 }
 fault_tick=g_robot_control.last_fault.tick_ms;
 assert(leg_inverse(.03f,.07f,1.57f,1.57f,combined_pose));
 for(unsigned side=0;side<2;side++) {
  motors[g_joint_calibration[2*side].motor_id-1].joint_angle_rad=combined_pose[1];
  motors[g_joint_calibration[2*side+1].motor_id-1].joint_angle_rad=combined_pose[0];
 }
 for(unsigned i=0;i<4;i++) step();
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_LENGTH_RANGE);
 assert(g_robot_control.last_fault.tick_ms==fault_tick && !g_robot_control.enable);
 for(unsigned f=0;f<2;f++) for(unsigned b=0;b<8;b++) assert(data[f][b]==0);
 g_dual_hold.overlength_kp_n_per_m=NAN;g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 g_dual_hold.overlength_kp_n_per_m=support_preset.overlength_kp_n_per_m;
 const float invalid_startup[]={NAN,-.1f,5.01f};
 for(unsigned i=0;i<3;i++) {
  g_dual_hold.overlength_startup_s=invalid_startup[i];g_robot_control.enable=1;step();
  assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
  assert(g_dual_hold.overlength_blend==0);
  for(unsigned f=0;f<2;f++) for(unsigned b=0;b<8;b++) assert(data[f][b]==0);
 }
 g_dual_hold.overlength_startup_s=support_preset.overlength_startup_s;
 g_dual_hold.support_only=2;g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
 g_dual_hold.support_only=1;
 volatile float *quiet_params[]={&g_dual_hold.length_error_deadband_m,
     &g_dual_hold.length_speed_deadband_m_s,&g_dual_hold.overlength_transition_m};
 const float quiet_limits[]={.005f,.02f,.01f};
 for(unsigned p=0;p<3;p++) {
  float saved=*quiet_params[p];
  for(unsigned bad=0;bad<2;bad++) {
   *quiet_params[p]=bad ? quiet_limits[p]+.001f : NAN;
   g_robot_control.enable=1;step();
   assert(!g_robot_control.enable && g_robot_control.last_fault.reason==ROBOT_FAULT_PARAMETERS);
   for(unsigned side=0;side<2;side++) {
    assert(g_dual_hold.force_n[side]==0);
    assert(g_dual_hold.length_error_control_m[side]==0 && g_dual_hold.length_speed_control_m_s[side]==0);
   }
   for(unsigned f=0;f<2;f++) for(unsigned b=0;b<8;b++) assert(data[f][b]==0);
  }
  *quiet_params[p]=saved;
 }
 g_robot_control.enable=2;step();assert(!g_robot_control.enable && g_robot_control.status==3);
 /* Either component's manual stop must also stop the combined controller. */
 for(unsigned component=0;component<2;component++) {
  g_robot_control.enable=1;step();assert(g_robot_control.status==1);
  if(component) g_balance.enable=0;else g_dual_hold.enable=0;
  step();assert(!g_robot_control.enable && g_robot_control.status==0);
  assert(!g_dual_hold.enable && !g_balance.enable);
  for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 }
 g_robot_control.enable=1;step();fail_at=calls+1;step();
 assert(!g_robot_control.enable && g_robot_control.status==4);
 assert(g_robot_control.last_fault.reason==ROBOT_FAULT_TX);
 fail_at=0;
 for(unsigned i=0;i<4;i++) step();
 assert(!g_robot_control.enable && g_robot_control.status==4);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
 g_robot_control.enable=1;step();
 combined_calls=calls;g_wheel_test.enable=1;step();
 assert(calls-combined_calls==2);
 assert(!g_robot_control.enable && g_robot_control.status==6);
 assert(!g_dual_hold.enable && !g_balance.enable && g_wheel_test.status==1);
 for(unsigned i=0;i<4;i++) {
  unsigned slot=g_joint_calibration[i].motor_id-1;
  assert(g_wheel_test.voltage_mv[slot]==0);
  assert(data[slot/4][2*(slot%4)]==0 && data[slot/4][2*(slot%4)+1]==0);
 }
 g_robot_control.enable=1;step();assert(!g_robot_control.enable && g_robot_control.status==6);
 g_wheel_test.enable=0;
 for(unsigned i=0;i<4;i++) step();
 assert(!g_robot_control.enable);
 /* Reduced load represents the supported body. Sweep plausible effective
  * masses and two loads; noise/backlash/electrical dynamics remain unmodeled. */
 const float plant_masses[]={.5f,1.0f,1.5f},plant_loads[]={2.0f,4.0f};
 for(unsigned m=0;m<3;m++) for(unsigned l=0;l<2;l++) {
  AxialResponse previous=simulate_axial_response(400,10,plant_masses[m],plant_loads[l]);
  /* Counterexample: 35 looks better in this ideal plant but was rejected
   * by hardware. Preserve that model check without shipping the high gain. */
  AxialResponse damped=simulate_axial_response(300,35,plant_masses[m],plant_loads[l]);
  printf("ideal plant only, mass=%g kg load=%g N: undershoot %g -> %g mm; tail=%g mm/%g m/s, faults=%u/%u\n",
   plant_masses[m],plant_loads[l],1000*previous.undershoot,1000*damped.undershoot,
   1000*damped.tail_error,damped.tail_speed,previous.faults,damped.faults);
  assert(!previous.faults && !damped.faults);
  assert(damped.undershoot<previous.undershoot*.4f);
  assert(damped.tail_error<.0002f && damped.tail_speed<.001f);
 }
 g_dual_hold=combined_hold;g_balance=combined_balance;
 memcpy(motors,combined_motors,sizeof(motors));imu=combined_imu;
 puts("PASS: single robot switch, combined output/stop, fault latch and wheel-only ownership");
#else
 g_robot_control.enable=1;step();
 assert(!g_robot_control.enable && g_robot_control.status==5);
 assert(!g_dual_hold.enable && !g_balance.enable);
 for(unsigned i=0;i<8;i++) assert(g_dual_hold.voltage_mv[i]==0);
#endif

 /* A single switch owns wheel output even with no joint/wheel feedback and
  * unusable leg-controller settings. Joint slots must be zero on the bus. */
 assert(!g_wheel_test.enable);
#if MOTOR_COMMAND_TX_ENABLED
 g_dual_hold.enable=1;g_balance.enable=1;step();
 assert(g_dual_hold.status==1 && g_balance.status==1);
#endif
 MotorFeedback test_saved_motors[8];memcpy(test_saved_motors,motors,sizeof(motors));
 ImuSample test_saved_imu=imu;
 for(unsigned i=0;i<8;i++) motors[i].online=0;
 imu_ok=1;imu.attitude_deg[BODY_PITCH_AXIS]=-10;
 for(unsigned i=0;i<3;i++) imu.gyro_rad_s[i]=0;
 g_dual_hold.position_kp=NAN;
 g_dual_hold.enable=1;g_balance.enable=1;g_wheel_test.enable=1;
 unsigned test_calls=calls;step();
 assert(!g_dual_hold.enable && !g_balance.enable);
 assert(g_joint_output.output_ratio==0);
 assert(!g_wheel_leg.leg_valid[0] && !g_wheel_leg.wheel_valid[0]);
#if MOTOR_COMMAND_TX_ENABLED
 assert(g_wheel_test.status==1 && g_wheel_test.enable && calls==test_calls+2);
 assert(g_wheel_test.voltage_mv[2]==-6000 && g_wheel_test.voltage_mv[6]==6000);
 assert(!g_wheel_output.speed_valid[0] && !g_wheel_output.speed_valid[1]);
 assert(g_wheel_output.back_emf_mv[0]==0 && g_wheel_output.back_emf_mv[1]==0);
 for(unsigned slot=0;slot<8;slot++) {
  uint16_t encoded=(uint16_t)data[slot/4][2*(slot%4)]|
                   ((uint16_t)data[slot/4][2*(slot%4)+1]<<8);
  assert(encoded==(uint16_t)g_wheel_test.voltage_mv[slot]);
  if(slot!=2 && slot!=6) assert(encoded==0);
  assert(g_dual_hold.voltage_mv[slot]==0);
 }
 imu.attitude_deg[BODY_PITCH_AXIS]=10;test_calls=calls;step();
 assert(g_wheel_test.voltage_mv[2]==6000 && g_wheel_test.voltage_mv[6]==-6000);
 assert(calls==test_calls+2);
 step();
 assert(g_wheel_test.saturated && abs(g_wheel_test.voltage_mv[2])==6000);
 /* Below saturation: compare both sides against independent ESP32 equations,
  * including mounting polarity, signed back-EMF and supply scaling. */
 imu.attitude_deg[BODY_PITCH_AXIS]=-1;
 for(unsigned side=0;side<2;side++) motors[(side?WHEEL_RIGHT_CAN_ID:WHEEL_LEFT_CAN_ID)-1].online=1;
 const float speeds[]={-20,-5,0,5,20};
 const float supplies[]={0,9,12,14};
 for(unsigned vi=0;vi<4;vi++) for(unsigned si=0;si<5;si++) {
  g_wheel_output.supply_voltage_v=supplies[vi];
  motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-speeds[si];
  motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=speeds[4-si];
  step();assert(g_wheel_test.status==1);
  float supply=supplies[vi]<8?12:supplies[vi];
  float ratio=(12-supply)/10+.7f;
  assert(fabsf(g_wheel_output.output_ratio-ratio)<1e-6f);
  for(unsigned side=0;side<2;side++) {
   float speed=side?speeds[4-si]:speeds[si],s=fabsf(speed);
   float emf=(.000004f*s*s*s-.0003f*s*s+.0266f*s)*1000;
   if(speed<0) emf=-emf;
   float tq=g_wheel_test.torque_nm/(side?WHEEL_RIGHT_TORQUE_RATIO:WHEEL_LEFT_TORQUE_RATIO)*ratio*1000;
   assert(g_wheel_output.speed_valid[side]);
   assert(g_wheel_output.speed_rad_s[side]==speed);
   assert(fabsf(g_wheel_output.back_emf_mv[side]-emf)<.001f);
   assert(fabsf(g_wheel_output.torque_voltage_mv[side]-tq)<.002f);
   assert(fabsf(g_wheel_output.requested_mv[side]-(tq+emf))<.002f);
   float clamped=fmaxf(-6000,fminf(6000,tq+emf));
   int16_t expected=(int16_t)((side?1:-1)*clamped);
   assert(abs(g_wheel_test.voltage_mv[(side?7:3)-1]-expected)<=1);
  }
 }
 g_wheel_output.supply_voltage_v=12;
 /* In the wheel-only test, the center is true zero even if wheels rotate. */
 imu.attitude_deg[BODY_PITCH_AXIS]=0;step();
 assert(g_wheel_test.torque_nm==0 && g_wheel_test.centered);
 assert(g_wheel_test.voltage_mv[2]==0 && g_wheel_test.voltage_mv[6]==0);
 assert(g_wheel_output.back_emf_mv[0]==0 && g_wheel_output.back_emf_mv[1]==0);
 imu.attitude_deg[BODY_PITCH_AXIS]=-.14f;step();
 assert(g_wheel_test.centered && g_wheel_test.voltage_mv[2]==0);
 imu.attitude_deg[BODY_PITCH_AXIS]=-.151f;step();
 assert(!g_wheel_test.centered);
 assert(g_wheel_output.back_emf_blend>0 && g_wheel_output.back_emf_blend<.01f);
 assert(abs(g_wheel_test.voltage_mv[2])<10); /* No compensation jump at the edge. */
 /* Stale speed disables only its own compensation, never the tilt response. */
 motors[WHEEL_LEFT_CAN_ID-1].age_ms=11;
 imu.attitude_deg[BODY_PITCH_AXIS]=-1;step();
 assert(g_wheel_test.status==1 && !g_wheel_output.speed_valid[0]);
 assert(g_wheel_output.back_emf_mv[0]==0 && g_wheel_output.speed_valid[1]);
 assert(g_wheel_test.voltage_mv[2]!=0);
 motors[WHEEL_LEFT_CAN_ID-1].age_ms=0;
 g_wheel_test.enable=0;step();assert(g_wheel_test.status==0);
 for(unsigned group=0;group<2;group++) for(unsigned i=0;i<8;i++) assert(data[group][i]==0);
 assert(g_wheel_output.voltage_mv[0]==0 && g_wheel_output.voltage_mv[1]==0);
 for(unsigned i=0;i<4;i++) step();
 test_calls=calls;step();assert(calls==test_calls);
 g_wheel_test.enable=1;step();imu_ok=0;step();
 assert(!g_wheel_test.enable && g_wheel_test.status==2);
 for(unsigned group=0;group<2;group++) for(unsigned i=0;i<8;i++) assert(data[group][i]==0);
 imu_ok=1;for(unsigned i=0;i<4;i++) step();
 assert(g_wheel_test.status==2 && !g_wheel_test.enable);
 g_wheel_test.enable=1;step();now+=11;WheelLeg_Update();
 assert(g_wheel_test.status==2 && !g_wheel_test.enable);
 for(unsigned i=0;i<4;i++) step();
 g_wheel_test.leg_length_m=nextafterf(LQR_LEG_MAX_M,INFINITY);g_wheel_test.enable=1;step();
 assert(g_wheel_test.status==3 && !g_wheel_test.enable);
 g_wheel_test.leg_length_m=.11f;for(unsigned i=0;i<4;i++) step();
 g_wheel_output.torque_ratio_nm_per_v[1]=0;g_wheel_test.enable=1;step();
 assert(g_wheel_test.status==3 && !g_wheel_test.enable);
 for(unsigned i=0;i<8;i++) assert(g_wheel_test.voltage_mv[i]==0);
 g_wheel_output.torque_ratio_nm_per_v[1]=WHEEL_RIGHT_TORQUE_RATIO;
 for(unsigned i=0;i<4;i++) step();
 g_wheel_output.supply_voltage_v=NAN;g_wheel_test.enable=1;step();
 assert(g_wheel_test.status==3 && !g_wheel_test.enable);
 g_wheel_output.supply_voltage_v=12;for(unsigned i=0;i<4;i++) step();
 g_wheel_test.enable=1;step();fail_at=calls+2;step();
 assert(g_wheel_test.status==4 && !g_wheel_test.enable);
 for(unsigned i=0;i<8;i++) assert(g_wheel_test.voltage_mv[i]==0);
 fail_at=0;step();
 for(unsigned group=0;group<2;group++) for(unsigned i=0;i<8;i++) assert(data[group][i]==0);
 for(unsigned i=0;i<4;i++) step();assert(g_wheel_test.status==4);
 /* Capture 500 ms of the chosen upright pose and stationary gyro bias.
  * Motors stay zero; neither capture nor completion automatically enables. */
 imu.attitude_deg[BODY_PITCH_AXIS]=-2;imu.gyro_rad_s[BODY_PITCH_AXIS]=-.02f;
 g_wheel_test.zero_request=1;
 for(unsigned n=0;n<100;n++) {
  step();assert(!g_wheel_test.enable);
  for(unsigned slot=0;slot<8;slot++) assert(g_wheel_test.voltage_mv[slot]==0);
  if(n<99) assert(g_wheel_test.zero_request && g_wheel_test.status==6);
 }
 assert(!g_wheel_test.zero_request && g_wheel_test.zero_status==2 && g_wheel_test.status==0);
 assert(g_wheel_test.zero_samples==100);
 assert(fabsf(g_wheel_test.pitch_zero_deg-2)<1e-5f);
 assert(fabsf(g_wheel_test.pitch_rate_bias_rad_s-.02f)<1e-6f);
 g_wheel_test.enable=1;step();
 assert(g_wheel_test.centered && g_wheel_test.status==1);
 for(unsigned slot=0;slot<8;slot++) assert(g_wheel_test.voltage_mv[slot]==0);
 /* Fast angular movement is rejected and resets the zero collection window. */
 g_wheel_test.zero_request=1;imu.gyro_rad_s[BODY_PITCH_AXIS]=-.1f;
 step();assert(g_wheel_test.zero_status==3 && g_wheel_test.zero_samples==0);
 assert(g_wheel_test.pitch_zero_deg>1.9f);
 imu.gyro_rad_s[BODY_PITCH_AXIS]=0;
 for(unsigned n=0;n<30;n++) step();assert(g_wheel_test.zero_samples==30);
 imu.attitude_deg[BODY_PITCH_AXIS]=-2.3f;step();
 assert(g_wheel_test.zero_status==3 && g_wheel_test.zero_samples==0);
 /* Invalid IMU also restarts capture instead of changing calibration. */
 imu_ok=0;step();assert(g_wheel_test.zero_status==3 && g_wheel_test.zero_request);
 imu_ok=1;
 for(unsigned n=0;n<100;n++) step();
 assert(g_wheel_test.zero_status==2 && !g_wheel_test.zero_request && g_wheel_test.enable);
 step();assert(g_wheel_test.centered && g_wheel_test.status==1);
 /* Rate outside the deadband still produces a damping command at zero angle. */
 imu.gyro_rad_s[BODY_PITCH_AXIS]=-.03f;step();
 assert(!g_wheel_test.centered && g_wheel_test.torque_nm!=0);
 g_wheel_test.enable=0;step();for(unsigned n=0;n<4;n++) step();
 g_wheel_test.pitch_zero_deg=0;g_wheel_test.pitch_rate_bias_rad_s=0;
#else
 assert(g_wheel_test.status==5 && !g_wheel_test.enable && calls==test_calls);
 for(unsigned i=0;i<4;i++) step();
#endif
 g_dual_hold.position_kp=5;
 memcpy(motors,test_saved_motors,sizeof(motors));imu=test_saved_imu;step();calls=0;

 WheelLegControlInput in={0};WheelLegControlOutput out;
 in.leg[0]=g_wheel_leg.leg[0];in.leg[1]=g_wheel_leg.leg[1];
 in.support_force_n[0]=4;in.support_force_n[1]=7;
 for(unsigned n=0;n<100;n++) {
  in.vmc_pitch_direction=n%2?1:-1;
  for(unsigned side=0;side<2;side++) for(unsigned j=0;j<6;j++) {
   in.state[side][j]=.001f*(float)(n+j+side);
   in.reference[side][j]=.0003f*(float)j;
  }
  assert(WheelLeg_Compute(&in,&out));
  for(unsigned side=0;side<2;side++) {
   float u[2];assert(lqr_control(in.leg[side].length,in.state[side],in.reference[side],u));
   assert(out.wheel_torque_nm[side]==u[0] && out.pitch_torque_nm[side]==u[1]);
   for(unsigned j=0;j<2;j++) {
    float torque=in.leg[side].jacobian[j]*in.support_force_n[side]+
      in.leg[side].jacobian[j+2]*u[1]*in.vmc_pitch_direction;
    assert(fabsf(out.joint_torque_nm[side][j]-torque)<1e-6f);
   }
  }
 }
 in.leg[1].length=nextafterf(LQR_LEG_MAX_M,INFINITY);assert(!WheelLeg_Compute(&in,&out));zero_output(&out);
 in.leg[1].length=.1f;in.state[1][3]=NAN;assert(!WheelLeg_Compute(&in,&out));zero_output(&out);
 in.state[1][3]=0;in.vmc_pitch_direction=0;assert(!WheelLeg_Compute(&in,&out));zero_output(&out);
 in.vmc_pitch_direction=1;in.leg[1].jacobian[3]=NAN;
 assert(!WheelLeg_Compute(&in,&out));zero_output(&out);
 assert(!WheelLeg_Compute(NULL,&out));zero_output(&out);assert(!WheelLeg_Compute(&in,NULL));

 int16_t voltages[8]={-32768,-3000,-1,0,1,3000,32767,127};
#if MOTOR_COMMAND_TX_ENABLED
 assert(MotorCommand_SendVoltages(voltages)==2 && calls==2);
 for(unsigned slot=0;slot<8;slot++) {
  uint16_t encoded=(uint16_t)data[slot/4][2*(slot%4)]|((uint16_t)data[slot/4][2*(slot%4)+1]<<8);
  assert(encoded==(uint16_t)voltages[slot]);
 }
 assert(MotorCommand_SendVoltages(NULL)==2);
 for(unsigned g=0;g<2;g++) for(unsigned i=0;i<8;i++) assert(data[g][i]==0);
 fail_at=calls+1;assert(MotorCommand_SendVoltages(voltages)==0);
 fail_at=calls+2;assert(MotorCommand_SendVoltages(voltages)==1);
#else
 assert(MotorCommand_SendVoltages(voltages)==0);
 assert(MotorCommand_SendVoltages(NULL)==0 && calls==0);
#endif
 puts("PASS: observation, dual hold, shared LQR/VMC, independent wheel test, faults and CAN encoding");
}
'''
with tempfile.TemporaryDirectory(prefix="wheel_leg_") as directory:
    tmp = Path(directory)
    (tmp / "runtime.h").write_text(runtime)
    for name in ("main.h", "FreeRTOS.h", "task.h", "bsp_fdcan.h"):
        (tmp / name).write_text('#include "runtime.h"\n')
    (tmp / "test.c").write_text(harness)
    sources = ["App/Protocol/Src/robot_motion_command.c","App/Tasks/Src/wheel_leg_task.c", "App/Tasks/Src/wheel_leg_observer.c", "App/Tasks/Src/wheel_leg_runtime.c",
               "App/Services/Src/motor_command.c",
               "tests/support/wheel_leg/Src/wheel_leg_control.c", "Control/WheelLeg/Src/lqr_k.c",
               "Control/WheelLeg/Src/leg_pos.c", "tests/support/wheel_leg/Src/leg_trajectory.c"]
    command = ["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-DROBOT_NATIVE_LQR=0",
                    "-I", str(tmp), "-I", str(root / "App/Tasks/Inc"),
                    "-I", str(root / "App/Services/Inc"), "-I", str(root / "Control/WheelLeg/Inc"),
                    "-I", str(root / "tests/support/wheel_leg/Inc"),
                    "-I", str(root / "App/Protocol/Inc"),
                    str(tmp / "test.c"), *[str(root / path) for path in sources],
               "-lm"]
    # Cover both compile-time modes regardless of the current firmware default.
    for index, flags in enumerate((["-DMOTOR_COMMAND_TX_ENABLED=0","-DROBOT_LEGS_ONLY_CALIBRATION=0"], ["-DMOTOR_COMMAND_TX_ENABLED=1","-DROBOT_LEGS_ONLY_CALIBRATION=0"], ["-DMOTOR_COMMAND_TX_ENABLED=1","-DROBOT_LEGS_ONLY_CALIBRATION=1","-DMOTOR_FOC_OUTPUT_ENABLE_MASK=0x33U"])):
        executable = tmp / f"test_{index}.exe"
        subprocess.run(command + flags + ["-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
