"""Close the production C task around a sampled nominal sagittal plant.

Unlike prescribed-feedback fixtures, every measured sample after initialization
comes from the previous controller outputs. The real task, observer, input
builder, leg geometry, gain polynomial and VMC allocation are compiled. Only
the MotorCommand hardware boundary is replaced, with role-specific production
Kt/current caps and the driver's configured current slew.

This is an established-balance screen, not hardware or startup validation.
Leg length is an ideal radial constraint; axial support is still included in
the real J^T allocation, motor current and inverse-Jacobian effort recovery.
Optional wheel-port viscous load and common-hip stiffness/damping are
hypothetical uncertainty proxies. It cannot assess vertical collapse, slip,
Coulomb/static friction, yaw, left/right splitting, CAN timing, voltage
headroom or nonlinear large-angle dynamics. A failed or
large-angle trajectory is diagnostic, not a physically accurate fall replay.
The simulated linkage keeps its inverse-kinematics assembly continuous from
its initialized joint pose. Re-selecting the nearest pose to a fixed 1.57rad
seed can teleport a joint to another elbow assembly and invent an impulse
when continuous motor currents are projected through the new Jacobian.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
from zipfile import ZipFile

import numpy as np

from verify_foc_sampled_timing import continuous_plant, macro
from verify_lqr_extended_domain import exponential


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / 'diagnostics/native_parking_closed_loop_current.json'


def plant(parameters, length, tau=.020, wheel_drag=0, hip_stiffness=0, hip_damping=0):
    """Exact 1ms ZOH for six physical plus two single velocity-LPF states."""
    ac, bc = continuous_plant(parameters, length)
    # External mechanical proxies, not extra controller damping. The wheel
    # port is shaft rate dx/R-dtheta; hip rate is dtheta+dphi. These uncertain
    # loads expose failures omitted by the ideal free-hip, zero-drag plant.
    wheel_port = np.array([0., -1., 0., 1/parameters['wheel_radius_m'], 0., 0.])
    hip_angle = np.array([1., 0., 0., 0., 1., 0.])
    hip_rate = np.array([0., 1., 0., 0., 0., 1.])
    ac -= np.outer(bc[:, 0], wheel_port)*wheel_drag
    ac -= np.outer(bc[:, 1], hip_angle*hip_stiffness+hip_rate*hip_damping)
    a, b = np.zeros((8, 8)), np.zeros((8, 2))
    a[:6, :6], b[:6] = ac, bc
    a[6, [1, 5, 6]] = [1/tau, 1/tau, -1/tau]
    a[7, [3, 7]] = [1/tau, -1/tau]
    p = parameters
    m, ml, h, r = p['body_mass_kg']/2, p['leg_mass_per_side_kg'], p['body_com_height_m'], p['wheel_radius_m']
    inertia = np.array([
        [m+ml+p['wheel_mass_per_side_kg']+p['wheel_inertia_per_side']/r**2,
         m*length+ml*length/2, -m*h],
        [m*length+ml*length/2, p['leg_inertia_per_side']+ml*(length/2)**2+m*length**2, -m*length*h],
        [-m*h, -m*length*h, p['body_pitch_inertia_per_side']+m*h*h]])
    disturbance = np.zeros(8)
    disturbance[[3, 1, 5]] = np.linalg.solve(inertia, [0, 0, -m*p['gravity_m_s2']])
    aug = np.zeros((11, 11))
    aug[:8, :8], aug[:8, 8:10], aug[:8, 10] = a, b, disturbance
    d = exponential(aug*.001)
    return d[:8, :8], d[:8, 8:10], d[:8, 10]


FIXTURE = r'''
#include "balance_equilibrium.h"
#include "plant.h"
#ifndef MODEL_REMOTE_JOINT_IQ_LIMIT_A
#define MODEL_REMOTE_JOINT_IQ_LIMIT_A 2.0f
#endif
static double z[8], requested_shaft[8], actual_shaft[8], actual_iq[8];
static const int motor_direction[8]=MOTOR_FOC_DEFAULT_DIRECTIONS;
static double max_wheel_iq,max_joint_iq;
static float geometry_q[2];
static unsigned geometry_initialized;
static double max_joint_step,geometry_self_test_max_joint_step;
static uint32_t plant_active_mask;
static unsigned output_count, current_clips, slew_limited;
static const double rad=.017453292519943295;
static double clipd(double v,double cap) {return fmax(-cap,fmin(cap,v));}
#ifdef MOTOR_FEEDBACK_HEALTH_VERSION
/* The model boundary prescribes healthy MotorFeedback and does not link the
 * FOC transport. A real attributed motor-health failure is unexpected here. */
void MotorFoc_LatchFeedbackFailure(const MotorFeedbackHealth *sample,
                                 uint32_t tick,uint32_t detail,uint32_t age) {
 (void)sample;(void)tick;(void)detail;(void)age;
 assert(!"Unexpected motor-health latch at the nominal model boundary");
}
#endif
static float kt(uint8_t id) {
 return (id==WHEEL_LEFT_CAN_ID || id==WHEEL_RIGHT_CAN_ID) ?
  MOTOR_FOC_WHEEL_ESTIMATED_KT_NM_PER_A : MOTOR_FOC_JOINT_KT_NM_PER_A;
}
void MotorCommand_BeginCycle(void) {}
void MotorCommand_EndCycle(void) {}
void MotorCommand_OnTransportFault(void) {plant_active_mask=0;}
uint32_t MotorCommand_CommunicationStopped(void) {return plant_active_mask==0;}
uint32_t MotorCommand_UsesCurrent(uint8_t id) {return id>=1 && id<=8 && (0x77U&(1U<<(id-1)));}
float MotorCommand_CurrentTorqueLimitNm(uint8_t id) {
 return kt(id)*((id==WHEEL_LEFT_CAN_ID || id==WHEEL_RIGHT_CAN_ID) ?
  fminf(MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A,MOTOR_FOC_CURRENT_HARD_LIMIT_A) :
  fminf(MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A,MODEL_REMOTE_JOINT_IQ_LIMIT_A));
}
float MotorCommand_CurrentForTorque(uint8_t id,float torque) {return motor_direction[id-1]*torque/kt(id);}
uint32_t MotorCommand_MeasuredTorqueNm(uint8_t id,float *torque) {
 if(!torque || !MotorCommand_UsesCurrent(id))return 0;
 *torque=(float)(motor_direction[id-1]*kt(id)*actual_iq[id-1]);return 1;
}
uint32_t MotorCommand_SendOutputs(const int16_t voltage[8],const float torque[8],uint32_t mask) {
 (void)voltage;++output_count;plant_active_mask=mask;
 for(unsigned i=0;i<8;++i) requested_shaft[i]=torque && (mask&(1U<<i))?torque[i]:0;
 return 2;
}
uint32_t MotorCommand_SendVoltages(const int16_t voltage[8]) {(void)voltage;return 2;}

/* A physical linkage cannot change elbow assembly between samples. Use the
 * previous solution at BOTH the sensor and 1ms actual-effort boundaries. */
static unsigned continuous_geometry(double length,double angle,float q[2],LegModel *geometry) {
 const float rear=geometry_initialized?geometry_q[0]:1.57f;
 const float front=geometry_initialized?geometry_q[1]:1.57f;
 const float x=(float)(.03+length*cos(angle)),y=(float)(length*sin(angle));
 if(!leg_inverse(x,y,rear,front,q) || !leg_model(q[0],q[1],LEG_BRANCH_DOWN,geometry))return 0;
 /* Keep the existing reachable/unique-branch/singular-J checks. Never repair
  * an invalid geometry by silently moving to another assembly. */
 const double det=geometry->jacobian[0]*geometry->jacobian[3]-geometry->jacobian[1]*geometry->jacobian[2];
 if(fabs(det)<=1e-8 || hypot(.03+geometry->length*cos(geometry->angle)-x,
                             geometry->length*sin(geometry->angle)-y)>2e-5)return 0;
 if(geometry_initialized) {
  const double dq=fmax(fabs(q[0]-geometry_q[0]),fabs(q[1]-geometry_q[1]));
  /* A >1rad joint step at a 1ms boundary is outside this small-angle screen;
   * in particular it must not become an artificial change of assembly. */
  if(dq>1)return 0;
  max_joint_step=fmax(max_joint_step,dq);
 }
 memcpy(geometry_q,q,sizeof(geometry_q));geometry_initialized=1;
 return 1;
}

static void verify_geometry_continuity(void) {
 /* The old fixed seed switched the rear elbow by2.829rad at relative angle
  * .339rad for L=.09m. Cross that boundary in both directions while also
  * checking the length used by the other established-balance cases. */
 for(unsigned model=0;model<2;++model) {
  const double length=model?.09:.07;
  geometry_initialized=0;max_joint_step=0;
  for(unsigned leg=0;leg<4;++leg)for(unsigned n=0;n<=450;++n) {
   const double relative=leg==0?n*.001:leg==1?.45-n*.001:leg==2?-(double)n*.001:-.45+n*.001;
   float q[2];LegModel geometry;
   if(!continuous_geometry(length,1.5707963267948966+relative,q,&geometry)) {
    fprintf(stderr,"Geometry self-test domain: L=%.9g relative=%.9g q=[%.9g,%.9g]\n",length,relative,q[0],q[1]);
    assert(0);
   }
   assert(max_joint_step<.02);
  }
  geometry_self_test_max_joint_step=fmax(geometry_self_test_max_joint_step,max_joint_step);
 }
 LegModel ambiguous;
 assert(!leg_model(0,acosf(-.2f),LEG_BRANCH_DOWN,&ambiguous));
 geometry_initialized=0;max_joint_step=0;
}

static unsigned feedback_sample(double length) {
 const double angle=1.5707963267948966+z[0]+z[4];
 if(fabs(z[0]+z[4])>.65 || fabs(z[4])>.65)return 0;
 float q[2];LegModel geometry;
 if(!continuous_geometry(length,angle,q,&geometry))return 0;
 const double det=geometry.jacobian[0]*geometry.jacobian[3]-geometry.jacobian[1]*geometry.jacobian[2];
 for(unsigned side=0;side<2;++side) {
  MotorFeedback *front=&motors[g_joint_calibration[2*side].motor_id-1];
  MotorFeedback *rear=&motors[g_joint_calibration[2*side+1].motor_id-1];
  rear->joint_angle_rad=q[0];front->joint_angle_rad=q[1];
  rear->joint_speed_rad_s=(float)(-geometry.jacobian[1]*z[6]/det);
  front->joint_speed_rad_s=(float)(geometry.jacobian[0]*z[6]/det);
 }
 imu.attitude_deg[BODY_PITCH_AXIS]=(float)(-z[4]/rad/BODY_PITCH_DIRECTION);
 imu.gyro_rad_s[BODY_PITCH_AXIS]=(float)(-z[5]/BODY_PITCH_DIRECTION);
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=(float)(-z[7]/WHEEL_RADIUS_M);
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=(float)(z[7]/WHEEL_RADIUS_M);
 for(unsigned i=0;i<8;++i) {
  motors[i].online=motors[i].joint_calibrated=motors[i].protocol_a1=1;
  motors[i].last_rx_ms=now+5;motors[i].age_ms=0;
 }
 for(unsigned i=0;i<4;++i) {
  const JointCalibration *cal=&g_joint_calibration[i];
  MotorFeedback *f=&motors[cal->motor_id-1];
  f->speed_rad_s=f->joint_speed_rad_s*cal->ratio*cal->direction;
 }
 return 1;
}

static unsigned actual_effort(double length,double *wheel,double *pitch) {
 *wheel=.5*(actual_shaft[WHEEL_LEFT_CAN_ID-1]+actual_shaft[WHEEL_RIGHT_CAN_ID-1]);
 *pitch=0;
 float q[2];LegModel m;
 double angle=1.5707963267948966+z[0]+z[4];
 if(!continuous_geometry(length,angle,q,&m))return 0;
 double det=m.jacobian[0]*m.jacobian[3]-m.jacobian[1]*m.jacobian[2];
 if(fabs(det)<=1e-8)return 0;
 for(unsigned side=0;side<2;++side) {
  const JointCalibration *front=&g_joint_calibration[2*side],*rear=&g_joint_calibration[2*side+1];
  double tf=actual_shaft[front->motor_id-1]*front->ratio;
  double tr=actual_shaft[rear->motor_id-1]*rear->ratio;
  *pitch+=.5*(m.jacobian[0]*tf-m.jacobian[1]*tr)/det;
 }
 return 1;
}

static unsigned advance(unsigned model,double length,double offset,double wheel_gain,double joint_gain) {
 for(unsigned sub=0;sub<5;++sub) {
  for(unsigned i=0;i<8;++i) {
   if(!MotorCommand_UsesCurrent((uint8_t)(i+1)))continue;
   double cap=MotorCommand_CurrentTorqueLimitNm((uint8_t)(i+1));
   double requested_iq=MotorCommand_CurrentForTorque((uint8_t)(i+1),(float)requested_shaft[i]);
   double target=clipd(requested_iq,cap/kt((uint8_t)(i+1)));
   if(fabs(target-requested_iq)>1e-7)++current_clips;
   double delta=target-actual_iq[i];
   double maxdelta=MOTOR_FOC_FIXED_IQ_SLEW_A_PER_S*.001;
   if(fabs(delta)>maxdelta)++slew_limited;
   /* STOP is a separate disabled-driver operation; no residual model torque. */
   if(!(plant_active_mask&(1U<<i)))actual_iq[i]=0;
   else actual_iq[i]+=clipd(delta,maxdelta);
   actual_shaft[i]=motor_direction[i]*kt((uint8_t)(i+1))*actual_iq[i];
   if(i+1==WHEEL_LEFT_CAN_ID || i+1==WHEEL_RIGHT_CAN_ID)max_wheel_iq=fmax(max_wheel_iq,fabs(actual_iq[i]));
   else max_joint_iq=fmax(max_joint_iq,fabs(actual_iq[i]));
   assert(fabs(actual_shaft[i])<=cap+1e-7);
  }
  double wheel,pitch,next[8];if(!actual_effort(length,&wheel,&pitch))return 0;
  for(unsigned i=0;i<8;++i) {
   next[i]=plant_d[model][i][10]*offset;
   for(unsigned j=0;j<8;++j)next[i]+=plant_d[model][i][j]*z[j];
   next[i]+=plant_d[model][i][8]*wheel*wheel_gain+plant_d[model][i][9]*pitch*joint_gain;
   assert(isfinite(next[i]));
  }
  memcpy(z,next,sizeof(z));
 }
 return 1;
}

int main(int argc,char **argv) {
 assert(argc==11);
 verify_geometry_continuity();
 unsigned scenario=(unsigned)atoi(argv[1]),model=(unsigned)atoi(argv[2]);
 double length=atof(argv[3]),offset=atof(argv[4]),trim=atof(argv[5]);
 double wheel_gain=atof(argv[6]),joint_gain=atof(argv[7]),sign=atof(argv[8]);
 unsigned bias=(unsigned)atoi(argv[9]),integral=(unsigned)atoi(argv[10]);
 float theta,tp;
 assert(BalanceEquilibrium_Calculate((float)length,(float)(-trim*rad),&theta,&tp));
 z[0]=theta;z[4]=-trim*rad;
 g_robot_control.feedback_only=0;g_robot_control.startup_enable=0;
 g_robot_control.remote_enable=1;remote_ok=remote_fresh=1;imu_ok=1;
 g_dual_hold.target_m=(float)length;g_balance.pitch_trim_deg=(float)trim;
 g_balance_bias.enable=g_balance_bias.shadow_enable=0;
 g_balance_parking_integral.enable=integral;
 motion_sticks=(SbusSticks){.valid=1,.calibrated=1,.raw_ch2=1000,.frame_count=1};
 remote_sample.frame.channel[4]=1000;
 assert(feedback_sample(length));step();
 remote_sample.frame.channel[4]=306;
 /* Initialize established-balance references and currents at the equilibrium.
  * These prescribed initialization samples are excluded from every metric.
  * After this loop no sensor sample is prescribed independently of z. */
 for(unsigned n=0;n<2200;++n) {
  assert(feedback_sample(length));step();
  for(unsigned i=0;i<8;++i) {
   actual_shaft[i]=requested_shaft[i];
   actual_iq[i]=MotorCommand_CurrentForTorque((uint8_t)(i+1),(float)actual_shaft[i]);
  }
 }
 assert(g_balance.status==1 && g_dual_hold.status==1 && g_robot_motion.neutral_ready);
 assert(g_balance_bias.samples==0 && g_balance_bias.estimated_offset_m==0 && g_balance_bias.applied_phi_rad==0);
 double warmup_integral=0;
#ifdef BALANCE_NATIVE_SERVO_ENABLE
 warmup_integral=g_balance_servo.integral_m_s;
 assert(fabs(warmup_integral)<.0001);
#endif
 /* A separately labeled large-displacement initial condition: externally
  * constrained upright translation drives the real observer's odometry.
  * No outputs from this initialization translation are applied to the plant.
  * Restore the equilibrium motor currents before freeing the plant. */
 if(scenario==3) {
  double saved_shaft[8],saved_iq[8];
  memcpy(saved_shaft,actual_shaft,sizeof(saved_shaft));memcpy(saved_iq,actual_iq,sizeof(saved_iq));
  z[7]=sign*.8;
  for(unsigned n=0;n<200;++n){assert(feedback_sample(length));step();}
  z[7]=0;assert(feedback_sample(length));step();z[2]=g_balance.state[2];
  memcpy(actual_shaft,saved_shaft,sizeof(saved_shaft));memcpy(actual_iq,saved_iq,sizeof(saved_iq));
#ifdef BALANCE_NATIVE_SERVO_ENABLE
  g_balance_servo.integral_m_s=0;
#endif
 }
 g_balance_bias.enable=bias;g_balance_bias.shadow_enable=1;
 double parking_goal=g_balance.parking_target_m;
 if(scenario==0)z[3]=z[7]=sign*.18; /* Ground velocity impulse after arming. */
 double max_speed=0,max_pitch=0,tail_v=0,tail_err=0,tail_pitch=0,tail_goal_min=1e9,tail_goal_max=-1e9;
 double drive_sum=0,drive_peak_error=0,reverse_peak=0,return_travel=0,last_actual_x=z[2];
 double reverse_hold_sum=0,neutral_time=scenario==1?5:scenario==2?9:0;
 double neutral_settling_time=-1,neutral_start_x=0,neutral_coast=0,neutral_peak_error=0;
 double neutral_direction=scenario==2?-sign:sign;
 unsigned reverse_hold_count=0,neutral_quiet_ticks=0,neutral_started=0;
 unsigned tail_count=0,drive_count=0,completed=1,completed_ticks=0,locked_count=0,saturations=0;
 unsigned termination_reason=0;
 unsigned governor_enabled=0,governor_violations=0,brake_releases=0;
 double brake_goal_max_drift=0,min_release_dwell_s=1e9,max_true_speed_on_release=0;
 unsigned unclipped_reference_checks=0,unclipped_reference_violations=0;
 double max_abs_reference_input_error=0,max_true_parking_error=0;
#ifdef BALANCE_MOTION_REFERENCE_ENABLE
 unsigned previous_motion_phase=g_balance_motion_reference.phase;
 double frozen_brake_goal=0,observed_quiet_s=0;
 governor_enabled=g_balance_motion_reference.enable;
#endif
 puts("TRACE,time_s,true_x_m,true_v_m_s,observed_x_m,observed_v_m_s,body_pitch_deg,reference_v_m_s,parking_goal_m,reference_x_m,T_nm,Tp_nm,bias_phi_rad,bias_gate,locked,governor_phase,governor_quiet_s");
 for(unsigned n=0;n<4000;++n) {
  double time=n*.005,axis=0;
  unsigned debug_phase=0;double debug_quiet=0;
  if(scenario==1 && time>=1 && time<5)axis=sign;
  if(scenario==2 && time>=1 && time<5)axis=sign;
  if(scenario==2 && time>=5 && time<9)axis=-sign;
  motion_sticks.forward=(float)axis;motion_sticks.raw_ch2=axis>0?310:axis<0?1690:1000;
  ++motion_sticks.frame_count;
  if(!feedback_sample(length)){completed=0;termination_reason=1;break;}
  step();completed_ticks=n+1;
  assert(fabs(g_balance.state[0]-z[0])<3e-6 && fabs(g_balance.state[4]-z[4])<3e-6);
  assert(fabs(g_balance.state[1]-(z[6]-z[5]))<3e-5 && fabs(g_balance.state[3]-z[7])<1e-6);
  if(!g_robot_control.enable || g_balance.status!=1 || g_dual_hold.status!=1){completed=0;termination_reason=2;break;}
#if defined(BALANCE_NATIVE_SERVO_ENABLE) && defined(BALANCE_MOTION_REFERENCE_ENABLE)
  if(g_balance_servo.enable && g_balance_motion_reference.enable && g_balance_motion_reference.phase!=1) {
   ++unclipped_reference_checks;
   if(fabs(g_balance.reference[2]-g_balance.parking_target_m)>1e-6)++unclipped_reference_violations;
  }
#endif
  max_abs_reference_input_error=fmax(max_abs_reference_input_error,fabs(g_balance.state[2]-g_balance.reference[2]));
  max_true_parking_error=fmax(max_true_parking_error,fabs(g_balance.parking_error_m));
#ifdef BALANCE_MOTION_REFERENCE_ENABLE
  debug_phase=g_balance_motion_reference.phase;debug_quiet=g_balance_motion_reference.quiet_s;
  if(governor_enabled) {
   unsigned phase=g_balance_motion_reference.phase;
   double speed=g_balance.state[3],refspeed=g_balance.reference[3];
   if(phase==2 && previous_motion_phase!=2) {
    frozen_brake_goal=g_balance_motion_reference.goal_m;observed_quiet_s=0;
   }
   if(phase==2 || previous_motion_phase==2) {
    double drift=fabs(g_balance_motion_reference.goal_m-frozen_brake_goal);
    brake_goal_max_drift=fmax(brake_goal_max_drift,drift);
    if(drift>1e-6)++governor_violations;
    if(fabs(refspeed)<=1e-6 && fabs(speed)<=BALANCE_MOTION_STOP_SPEED_M_S)observed_quiet_s+=.005;
    else observed_quiet_s=0;
   }
   if(previous_motion_phase==2 && phase!=2) {
    ++brake_releases;min_release_dwell_s=fmin(min_release_dwell_s,observed_quiet_s);
    max_true_speed_on_release=fmax(max_true_speed_on_release,fabs(z[3]));
    if(observed_quiet_s+1e-6<BALANCE_MOTION_STOP_DWELL_S || fabs(refspeed)>1e-6)++governor_violations;
   }
   previous_motion_phase=phase;
  }
#endif
  if(g_balance.position_locked)++locked_count;
  if(g_wheel_output.saturated || g_joint_output.saturated)++saturations;
  max_speed=fmax(max_speed,fabs(z[3]));max_pitch=fmax(max_pitch,fabs(z[4]/rad));
  if(time>=3.5 && time<5) {
   drive_sum+=z[3];++drive_count;drive_peak_error=fmax(drive_peak_error,fabs(z[3]-sign*.4));
  }
  if(scenario==2 && time>=5 && time<9)reverse_peak=fmax(reverse_peak,-sign*z[3]);
  if(scenario==2 && time>=7.5 && time<9){reverse_hold_sum+=-sign*z[3];++reverse_hold_count;}
  if(time>=neutral_time) {
   if(!neutral_started){neutral_start_x=z[2];neutral_started=1;}
   neutral_coast=fmax(neutral_coast,neutral_direction*(z[2]-neutral_start_x));
   neutral_peak_error=fmax(neutral_peak_error,fabs(g_balance.parking_error_m));
   if(fabs(z[3])<.02 && fabs(g_balance.parking_error_m)<.03)++neutral_quiet_ticks;
   else neutral_quiet_ticks=0;
   if(neutral_quiet_ticks>=100 && neutral_settling_time<0)neutral_settling_time=time-neutral_time-.495;
  }
  if(time>=18) {
   tail_v=fmax(tail_v,fabs(z[3]));tail_err=fmax(tail_err,fabs(g_balance.parking_error_m));
   tail_pitch=fmax(tail_pitch,fabs(z[4]/rad));++tail_count;
   tail_goal_min=fmin(tail_goal_min,g_balance.parking_target_m);tail_goal_max=fmax(tail_goal_max,g_balance.parking_target_m);
  }
  if(n%10==0 || (scenario==2 && time>=5 && time<=6.5))printf("TRACE,%.5g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%.9g\n",
   time,z[2],z[3],g_balance.state[2],g_balance.state[3],-z[4]/rad,
   g_balance.reference[3],g_balance.parking_target_m,g_balance.reference[2],
   g_balance.torque_nm[0],g_balance.torque_nm[1],g_balance_bias.applied_phi_rad,
   (unsigned)g_balance_bias.gate_mask,(unsigned)g_balance.position_locked,debug_phase,debug_quiet);
  if(!advance(model,length,offset,wheel_gain,joint_gain)){completed=0;termination_reason=3;break;}
  return_travel+=fabs(z[2]-last_actual_x);last_actual_x=z[2];
 }
 if(!tail_count)tail_v=tail_err=tail_pitch=1e9;
 double goal_span=tail_count?tail_goal_max-tail_goal_min:1e9;
 unsigned parking_success=completed && tail_count>=399 && tail_v<.02 && tail_err<.03 && max_pitch<15 && goal_span<1e-5;
 double forward_mean=drive_count?sign*drive_sum/drive_count:0,reverse_mean=reverse_hold_count?reverse_hold_sum/reverse_hold_count:0;
 unsigned motion_response_ok=scenario==0 || scenario==3 ||
  (forward_mean>=.2 && (scenario!=2 || (reverse_mean>=.2 && reverse_peak<=.55)));
 unsigned pass=parking_success && motion_response_ok && !governor_violations && !unclipped_reference_violations;
 printf("RESULT,{\"completed\":%u,\"duration_s\":%.8g,\"pass\":%u,\"max_abs_true_speed_m_s\":%.9g,\"max_abs_body_pitch_deg\":%.9g,\"last2s_max_abs_speed_m_s\":%.9g,\"last2s_max_abs_parking_error_m\":%.9g,\"last2s_max_abs_body_pitch_deg\":%.9g,\"last2s_goal_span_m\":%.9g,\"last_drive_mean_speed_m_s\":%.9g,\"last_drive_peak_error_m_s\":%.9g,\"reverse_peak_speed_m_s\":%.9g,\"absolute_travel_m\":%.9g,\"initial_goal_m\":%.9g,\"last_goal_m\":%.9g,\"locked_ticks\":%u,\"saturated_ticks\":%u,\"backend_current_clips\":%u,\"slew_limited_substeps\":%u,\"output_calls\":%u,\"last_fault_reason\":%u,\"bias_samples\":%u,\"bias_applied_phi_rad\":%.9g,\"max_abs_wheel_iq_a\":%.9g,\"max_abs_joint_iq_a\":%.9g,\"governor_enabled\":%u,\"governor_violations\":%u,\"brake_releases\":%u,\"min_release_dwell_s\":%.9g,\"brake_goal_max_drift_m\":%.9g,\"max_true_speed_on_brake_release_m_s\":%.9g,\"warmup_bias_samples\":0,\"warmup_servo_integral_m_s\":%.9g,\"parking_success\":%u,\"motion_response_ok\":%u,\"reverse_hold_mean_signed_speed_m_s\":%.9g,\"neutral_first_half_second_settling_time_s\":%.9g,\"neutral_forward_coast_m\":%.9g,\"neutral_peak_parking_error_m\":%.9g,\"unclipped_reference_checks\":%u,\"unclipped_reference_violations\":%u,\"max_abs_reference_input_error_m\":%.9g,\"max_abs_true_parking_error_m\":%.9g,\"termination_reason\":%u,\"max_joint_angle_step_rad\":%.9g,\"geometry_self_test_max_joint_step_rad\":%.9g}\n",
  completed,completed_ticks*.005,pass,max_speed,max_pitch,tail_v,tail_err,tail_pitch,goal_span,
  drive_count?drive_sum/drive_count:0,drive_peak_error,reverse_peak,return_travel,parking_goal,
  g_balance.parking_target_m,locked_count,saturations,current_clips,slew_limited,output_count,
  (unsigned)g_robot_control.last_fault.reason,(unsigned)g_balance_bias.samples,g_balance_bias.applied_phi_rad,
  max_wheel_iq,max_joint_iq,governor_enabled,governor_violations,brake_releases,
  brake_releases?min_release_dwell_s:0,brake_goal_max_drift,max_true_speed_on_release,warmup_integral,
  parking_success,motion_response_ok,reverse_mean,neutral_settling_time,neutral_coast,neutral_peak_error,
  unclipped_reference_checks,unclipped_reference_violations,max_abs_reference_input_error,max_true_parking_error,termination_reason,
  max_joint_step,geometry_self_test_max_joint_step);
 return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_REPORT)
    parser.add_argument('--require-pass', action='store_true', help='Fail if any model case misses its numerical acceptance criteria')
    parser.add_argument('--quick', action='store_true', help='Run nominal zero-trim impulse, drive-release and reversal only')
    parser.add_argument('--baseline-from', type=Path, help='Compile archived production C/headers in a temporary overlay; keep the same current harness, plant and initialization')
    parser.add_argument('--gain-header', type=Path, help='Temporarily override balance_servo_gains.h without changing production files')
    parser.add_argument('--parking-gain-header', type=Path,
                        help='Override only the neutral-parking table in the private source snapshot')
    parser.add_argument('--large-displacement', action='store_true', help='Add separately labeled +/-0.8m displaced initial-condition recovery cases')
    parser.add_argument('--extended-offsets', action='store_true', help='Also screen +/-3mm and +/-5mm gravity offset uncertainty')
    parser.add_argument('--wheel-drag', type=float, default=0,
                        help='External wheel shaft viscous torque coefficient in Nm*s/rad; a hypothetical plant load')
    parser.add_argument('--hip-stiffness', type=float, default=0,
                        help='External common-hip stiffness in Nm/rad, around relative neutral')
    parser.add_argument('--hip-damping', type=float, default=0,
                        help='External common-hip damping in Nm*s/rad')
    parser.add_argument('--joint-remote-limit', type=float, default=3.0,
                        help='Remote joint Iq capability used at the hardware boundary; default3A matches verified1527 six-node readback')
    args = parser.parse_args()
    assert all(math.isfinite(v) and v >= 0 for v in
               (args.wheel_drag,args.hip_stiffness,args.hip_damping))
    assert math.isfinite(args.joint_remote_limit) and 0 < args.joint_remote_limit <= 3.3
    assert not (args.baseline_from and (args.gain_header or args.parking_gain_header)), 'Baseline comparison must use its archived gains'
    gain_info = None
    gain_bytes = None
    if args.gain_header:
        gain_bytes = args.gain_header.read_bytes()
        gain_info = {'path':str(args.gain_header.resolve()), 'sha256':hashlib.sha256(gain_bytes).hexdigest()}
    parking_gain_bytes = args.parking_gain_header.read_bytes() if args.parking_gain_header else None
    parking_gain_info = dict(path=str(args.parking_gain_header.resolve()),
        sha256=hashlib.sha256(parking_gain_bytes).hexdigest()) if parking_gain_bytes is not None else None
    archived = {}
    archive_info = None
    if args.baseline_from:
        with ZipFile(args.baseline_from) as archive:
            archived = {name: archive.read(name) for name in archive.namelist()
                        if name.startswith(('App/', 'Control/', 'tests/support/')) and name.endswith(('.c', '.h'))}
        archive_info = {'path':str(args.baseline_from.resolve()), 'sha256':hashlib.sha256(args.baseline_from.read_bytes()).hexdigest()}
    p = json.loads((ROOT/'models/lqr/lqr_model_metadata.json').read_text())['parameters']
    tree = ast.parse((ROOT/'tests/verify_wheel_leg_control.py').read_text(encoding='utf-8'))
    strings = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
               if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
               and isinstance(n.value, ast.Constant) and isinstance(n.value.value, str)}
    prefix = strings['harness'].split('int main(void) {')[0]
    # Remove the historical fixture's artificial 1.5:1 right-rear gearing.
    prefix = prefix.replace('0,0,1.5f,JOINT_RR_DIRECTION', '0,0,JOINT_RR_RATIO,JOINT_RR_DIRECTION')
    variants = [dict(name='nominal_zero_trim', offset_m=0, trim_deg=0, wheel_gain=1, joint_gain=1)]
    if not args.quick:
        variants += [dict(name='manual_trim_minus4', offset_m=0, trim_deg=-4, wheel_gain=1, joint_gain=1)]
        for offset in (-.001, .001):
            variants.append(dict(name=f'offset_{offset:+g}', offset_m=offset, trim_deg=-4, wheel_gain=1, joint_gain=1))
        for wheel_gain, joint_gain in ((.7, 1), (1.3, 1), (1, .7), (1, 1.3)):
            variants.append(dict(name=f'wheel{wheel_gain:g}_joint{joint_gain:g}', offset_m=0, trim_deg=-4, wheel_gain=wheel_gain, joint_gain=joint_gain))
        if args.extended_offsets:
            for offset in (-.005,-.003,.003,.005):
                variants.append(dict(name=f'offset_{offset:+g}', offset_m=offset, trim_deg=-4, wheel_gain=1, joint_gain=1))
    sources = ['App/Protocol/Src/robot_motion_command.c', 'App/Tasks/Src/wheel_leg_task.c',
               'App/Tasks/Src/wheel_leg_observer.c', 'tests/support/wheel_leg/Src/wheel_leg_control.c',
               'Control/WheelLeg/Src/lqr_k.c', 'Control/WheelLeg/Src/leg_pos.c',
               'tests/support/wheel_leg/Src/leg_trajectory.c']
    include_folders = ('App/Tasks/Inc', 'App/Services/Inc', 'App/Protocol/Inc', 'Control/WheelLeg/Inc', 'tests/support/wheel_leg/Inc')
    source_paths = ['tests/verify_native_parking_closed_loop.py', 'tests/verify_wheel_leg_control.py',
                    'App/Protocol/Src/robot_motion_command.c', 'App/Tasks/Src/wheel_leg_task.c',
                    'App/Tasks/Src/wheel_leg_observer.c', 'Control/WheelLeg/Src/lqr_k.c',
                    'Control/WheelLeg/Src/leg_pos.c', 'App/Tasks/Inc/wheel_leg_config.h', 'App/Tasks/Inc/wheel_leg_task.h',
                    'App/Services/Inc/motor_foc_config.h', 'models/lqr/lqr_model_metadata.json']
    source_paths += ['tests/verify_foc_sampled_timing.py','tests/verify_lqr_extended_domain.py',*sources]
    source_paths += [str(path.relative_to(ROOT)).replace('\\', '/')
                     for folder in include_folders for path in (ROOT/folder).glob('*.h')]
    source_paths = sorted(set(source_paths))
    snapshot = {path:gain_bytes if gain_bytes is not None and path=='Control/WheelLeg/Inc/balance_servo_gains.h'
                else parking_gain_bytes if parking_gain_bytes is not None and path=='Control/WheelLeg/Inc/balance_parking_gains.h'
                else archived[path] if path in archived else (ROOT/path).read_bytes() for path in source_paths}
    source_before = {path:hashlib.sha256(content).hexdigest() for path,content in snapshot.items()}
    results = []
    with tempfile.TemporaryDirectory(prefix='native_closed_loop_') as directory:
        work = Path(directory)
        overlay = work/'archived'
        for name, content in snapshot.items():
            path = overlay/name
            assert overlay.resolve() in path.resolve().parents, name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        config = overlay/'App/Tasks/Inc/wheel_leg_config.h'
        bias_default = int(macro(config, 'BALANCE_DEFAULT_GRAVITY_BIAS_ENABLE'))
        integral_default = int(macro(config, 'BALANCE_NATIVE_PARKING_INTEGRAL_ENABLE'))
        config_text = config.read_text(encoding='utf-8')
        governor_parameters = {name:macro(config,name) for name in (
            'BALANCE_MOTION_STOP_SPEED_M_S','BALANCE_MOTION_STOP_DWELL_S',
            'BALANCE_MOTION_ACCELERATION_M_S2','BALANCE_MOTION_BRAKE_ACCELERATION_M_S2') if name in config_text}
        foc_config = overlay/'App/Services/Inc/motor_foc_config.h'
        effective_current_limits = {'wheel_a':min(macro(foc_config,'MOTOR_FOC_DEFAULT_CURRENT_LIMIT_A'),
                                                  macro(foc_config,'MOTOR_FOC_CURRENT_HARD_LIMIT_A')),
                                    'joint_a':min(macro(foc_config,'MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A'),args.joint_remote_limit),
                                    'remote_joint_capability_a':args.joint_remote_limit,
                                    'remote_capability_assumption':'Joint boundary uses min(local, CLI remote Iq); default3A matches the1527 six-node20261115 readback. Wheel remote capability assumed at least its local cap. No live hardware calls in this test.'}
        (work/'runtime.h').write_text(strings['runtime'])
        if gain_bytes is not None:
            (work/'balance_servo_gains.h').write_bytes(gain_bytes)
        for name in ('main.h', 'FreeRTOS.h', 'task.h', 'bsp_fdcan.h'):
            (work/name).write_text('#include "runtime.h"\n')
        lengths = [.07, .09] if not args.quick else [.07]
        matrices = []
        for length in lengths:
            a, b, d = plant(p, length, wheel_drag=args.wheel_drag,
                            hip_stiffness=args.hip_stiffness, hip_damping=args.hip_damping)
            matrices.append(np.column_stack((a, b, d)))
        header = 'static const double plant_d[][8][11] = {\n'
        header += ',\n'.join('{'+',\n'.join('{'+','.join(format(v, '.17g') for v in row)+'}' for row in mat)+'}' for mat in matrices)
        (work/'plant.h').write_text(header+'\n};\n')
        (work/'test.c').write_text(prefix+FIXTURE)
        command = ['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                   '-DMYLEGWHEEL_A1_CURRENT=1', '-DROBOT_NATIVE_LQR=1',
                   '-DMODEL_REMOTE_JOINT_IQ_LIMIT_A='+repr(args.joint_remote_limit),
                   '-DROBOT_LEGS_ONLY_CALIBRATION=0', '-DMOTOR_COMMAND_TX_ENABLED=1', '-I', str(work)]
        for folder in include_folders:
            command += ['-I', str(overlay/folder)]
        exe = work/'test.exe'
        subprocess.run(command+[str(work/'test.c'), *[str(overlay/path) for path in sources], '-lm', '-o', str(exe)], check=True)
        for variant in variants:
            for model_index, length in enumerate(lengths):
                scenarios = ['rolling_impulse', 'drive_release', 'reversal_release']
                if args.large_displacement:
                    scenarios.append('displaced_initial_condition')
                for scenario, scenario_name in enumerate(scenarios):
                    for sign in (-1, 1):
                        invocation = [str(exe), str(scenario), str(model_index), str(length), str(variant['offset_m']),
                                      str(variant['trim_deg']), str(variant['wheel_gain']), str(variant['joint_gain']), str(sign),
                                      str(bias_default), str(integral_default)]
                        run = subprocess.run(invocation, capture_output=True, text=True)
                        if run.returncode:
                            raise AssertionError(f'Host controller/plant fixture failed for {invocation[1:]}: {run.stderr[-3000:]}\n{run.stdout[-3000:]}')
                        rows = [line for line in run.stdout.splitlines() if line.startswith('RESULT,')]
                        assert len(rows) == 1, run.stdout[-2000:]
                        result = json.loads(rows[0][7:])
                        result['termination_reason_name'] = ('completed','plant_state_or_geometry_domain_exceeded',
                                                             'production_controller_stopped','plant_inverse_kinematics_domain_exceeded')[result['termination_reason']]
                        trace = [[float(v) for v in line.split(',')[1:]] for line in run.stdout.splitlines()
                                 if line.startswith('TRACE,') and not line.startswith('TRACE,time')]
                        result.update(variant=variant, scenario=scenario_name, leg_length_m=length, sign=sign, trace=trace)
                        results.append(result)
    report = dict(result='Production C controller closed around a nominal plant; numerical screen only',
                  source_provenance=source_before,
                  archived_production_source=archive_info,
                  gain_header_override=gain_info,
                  parking_gain_header_override=parking_gain_info,
                  control_defaults={'gravity_bias_enable':bias_default, 'parking_integral_enable':integral_default},
                  governor_parameters=governor_parameters, effective_current_limits=effective_current_limits,
                  model_parameters=p, limitations=__doc__,
                  hypothetical_mechanical_loads=dict(wheel_drag_nm_s_per_rad=args.wheel_drag,
                      common_hip_stiffness_nm_per_rad=args.hip_stiffness,
                      common_hip_damping_nm_s_per_rad=args.hip_damping,
                      identification='Hypothetical uncertainty proxies, not identified robot parameters. '
                                     'The declared free-model equilibrium is unchanged during initialization; '
                                     'external stiffness may therefore create an initial effort mismatch.'),
                  measurement_pipeline='Plant has one 20ms LPF for relative-leg angular velocity and one for travel speed. Leg angles and body angle/rate are unfiltered. Real observer reconstructs theta rate and integrates filtered wheel speed; no separate M or duplicate LPF is applied.',
                  actuator_pipeline='Real task J^T support+swing allocation and current caps; wheel local cap=min(DEFAULT_CURRENT_LIMIT_A,CURRENT_HARD_LIMIT_A), joint local cap=JOINT_CURRENT_HARD_LIMIT_A, remote capability assumed no lower than local. Firmware-Iq uses installation direction, configured Kt and10A/s slew at1ms; measured Nm reverses direction once. Inverse actual-current J^T recovers Tp, preserving support/slew coupling. Independent gains multiply plant wheel and joint efforts as calibration uncertainty.',
                  geometry_pipeline='One persistent inverse-kinematics joint pose is shared by sensor synthesis and actual-current effort recovery at1ms. First pose uses1.57rad seeds; subsequent solves use the previous joint pose. The DOWN branch, Cartesian foot match and singular-J domain checks remain enforced; joint steps over1rad are outside this numerical screen. Each invocation first checks both lengths across relative angle+/-0.45rad, including the former0.339rad assembly-switch boundary in both directions, and verifies an ambiguous DOWN pose is rejected. Historical fixed-seed fixture reports may contain artificial elbow switches and cannot be compared directly against these trajectories.',
                  compilation='All C and project headers are copied to a private temporary snapshot before compilation. Report hashes describe those compiled bytes; concurrent source edits cannot change the compiled test.',
                  initialization='Startup staging disabled and equilibrium feedback prescribed for11s to initialize established-balance references/support/current. Estimation AND application are disabled during initialization; zero learned samples/estimate/application are asserted before restoring production observer defaults. All reported trajectories then close controller-to-plant feedback; no trajectory feedback prescription.',
                  large_displacement_initialization='Optional separate cases: upright pose externally constrained while wheel feedback translates0.8m over1s; actual production observer integrates odometry and retains its parking goal. Equilibrium currents and zero servo-I are restored at release. Reported20s are then fully closed. This is an initial-condition recovery test, not a simulation of the contact forces creating it.',
                  acceptance={'completed_s':20, 'last2s_speed_m_s':.02, 'last2s_parking_error_m':.03,
                              'max_body_pitch_deg':15, 'last2s_goal_span_m':1e-5,
                              'drive_hold_signed_mean_min_m_s':.2, 'reverse_hold_signed_mean_min_m_s':.2,
                              'reverse_peak_max_m_s':.55, 'neutral_settling_definition':'First continuous0.5s interval after neutral with true|v|<.02 and |parking error|<.03; retained terminal windows must also pass.',
                              'governor_zero_speed_dwell_s':governor_parameters.get('BALANCE_MOTION_STOP_DWELL_S'),
                              'frozen_brake_goal_tolerance_m':1e-6,
                              'unclipped_servo_parking_reference_tolerance_m':1e-6},
                  trace_columns=['time_s','true_x_m','true_v_m_s','observed_x_m','observed_v_m_s','body_pitch_deg',
                                 'reference_v_m_s','parking_goal_m','reference_x_m','T_nm','Tp_nm','bias_phi_rad','bias_gate','locked',
                                 'governor_phase','governor_quiet_s'],
                  passed=sum(r['pass'] for r in results), count=len(results), cases=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'passed':report['passed'], 'count':report['count'], 'report':str(args.output),
                      'cases':[{k:v for k,v in r.items() if k!='trace'} for r in results]}, indent=2))
    if args.require_pass:
        assert report['passed'] == report['count'], f"{report['count']-report['passed']} closed-loop cases failed; inspect {args.output}"


if __name__ == '__main__':
    main()
