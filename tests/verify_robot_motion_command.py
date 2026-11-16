"""Exercise the future remote target builder; no hardware interaction."""
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
fixture=r'''
#include "robot_motion_command.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void zero(const RobotMotionCommand *o) {
 assert(o->forward_m_s==0 && o->turn_rad_s==0);
 assert(o->requested_forward_m_s==0 && o->requested_turn_rad_s==0);
}
int main(void) {
 RobotMotionConfig c=g_robot_motion_defaults;
 RobotMotionPermission p={1,1,1,1};
 SbusSticks s={.forward=1,.yaw=-1,.valid=1,.calibrated=1,.received_ms=100,.frame_count=8};
 RobotMotionCommand o;RobotMotion_Clear(&o);
 RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);assert(o.status==ROBOT_MOTION_OFF);
 c.enable=1;
 RobotMotion_Update(&c,&s,&p,100,5,&o);
 assert(o.status==ROBOT_MOTION_TARGET_READY && o.source_frame_count==8);
 assert(fabsf(o.forward_m_s-.004f)<1e-8f && fabsf(o.turn_rad_s+.020f)<1e-8f);
 assert(o.requested_forward_m_s==.40f && o.requested_turn_rad_s==-3.00f);
 for(unsigned i=0;i<400;++i)RobotMotion_Update(&c,&s,&p,100,5,&o);
 assert(fabsf(o.forward_m_s-.40f)<1e-7f && fabsf(o.turn_rad_s+3.00f)<1e-7f);
 /* Reversal first brakes to zero, then accelerates in the new direction. */
 s.forward=-1;
 for(unsigned i=0;i<50;++i){
  float before=o.forward_m_s;RobotMotion_Update(&c,&s,&p,100,5,&o);
  assert(o.forward_m_s>=-1e-7f && before-o.forward_m_s<=.008001f);
 }
 assert(fabsf(o.forward_m_s)<1e-6f);
 for(unsigned i=0;i<101;++i){
  float before=o.forward_m_s;RobotMotion_Update(&c,&s,&p,100,5,&o);
  assert(fabsf(o.forward_m_s-before)<=.004001f);
 }
 assert(fabsf(o.forward_m_s+.40f)<1e-6f);
 s.forward=0;
 for(unsigned i=0;i<51;++i)RobotMotion_Update(&c,&s,&p,100,5,&o);
 assert(o.forward_m_s==0);
 s.forward=1;for(unsigned i=0;i<101;++i)RobotMotion_Update(&c,&s,&p,100,5,&o);
 s.forward=-.2f;
 for(unsigned i=0;i<80;++i)RobotMotion_Update(&c,&s,&p,100,5,&o);
 assert(fabsf(o.forward_m_s+.08f)<1e-6f);
 s.forward=s.yaw=0;
 for(unsigned i=0;i<400;++i)RobotMotion_Update(&c,&s,&p,100,5,&o);
 zero(&o);
 s.forward=-1;s.yaw=1;
 for(unsigned i=0;i<400;++i)RobotMotion_Update(&c,&s,&p,100,5,&o);
 assert(fabsf(o.forward_m_s+.40f)<1e-7f && fabsf(o.turn_rad_s-3.00f)<1e-7f);
 s.valid=0;RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);assert(o.status==ROBOT_MOTION_REMOTE_INVALID);s.valid=1;
 s.calibrated=0;RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);assert(o.status==ROBOT_MOTION_UNCALIBRATED);s.calibrated=1;
 for(unsigned i=0;i<4;++i) {
  p=(RobotMotionPermission){1,1,1,1};
  if(i==0)p.remote_enabled=0;if(i==1)p.ch5_on=0;if(i==2)p.controller_on=0;if(i==3)p.normal_balance=0;
  RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);assert(o.status==ROBOT_MOTION_NOT_READY);
 }
 p=(RobotMotionPermission){1,1,1,1};
 s.received_ms=100;RobotMotion_Update(&c,&s,&p,201,5,&o);zero(&o);assert(o.status==ROBOT_MOTION_REMOTE_INVALID);
 s.received_ms=UINT32_MAX-50U;RobotMotion_Update(&c,&s,&p,30,5,&o);assert(o.status==ROBOT_MOTION_TARGET_READY);
 RobotMotion_Update(&c,&s,&p,60,5,&o);zero(&o);assert(o.status==ROBOT_MOTION_REMOTE_INVALID);
 s.received_ms=100;s.forward=NAN;RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);s.forward=-1;
 s.yaw=1.01f;RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);s.yaw=1;
 const float bad[]={NAN,-.1f,.401f};
 for(unsigned i=0;i<3;++i){c.max_forward_m_s=bad[i];RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);assert(o.status==ROBOT_MOTION_BAD_PARAMETERS);}
 c=g_robot_motion_defaults;c.enable=1;
 RobotMotion_Update(&c,&s,&p,100,0,&o);zero(&o);RobotMotion_Update(&c,&s,&p,100,21,&o);zero(&o);
 o.forward_m_s=NAN;o.turn_rad_s=2;RobotMotion_Update(&c,&s,&p,100,5,&o);assert(isfinite(o.forward_m_s) && fabsf(o.forward_m_s)<=.004001f);
 c.enable=0;RobotMotion_Update(&c,&s,&p,100,5,&o);zero(&o);
 RobotMotion_Update(NULL,&s,&p,100,5,&o);zero(&o);
 RobotMotion_Update(&c,&s,&p,100,5,NULL);RobotMotion_Clear(NULL);
 puts("PASS: default OFF, signed CH2/CH1 targets, slew/neutral/reverse, uncalibrated/not-ready/loss zero, timeout/wrap, invalid input/config/state");
}
'''
with tempfile.TemporaryDirectory(prefix='robot_motion_') as folder:
    tmp=Path(folder);(tmp/'test.c').write_text(fixture)
    exe=tmp/'test.exe'
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
        '-I',str(ROOT/'App/Protocol/Inc'),str(tmp/'test.c'),
        str(ROOT/'App/Protocol/Src/robot_motion_command.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
