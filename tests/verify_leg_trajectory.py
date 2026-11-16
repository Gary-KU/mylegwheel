"""Verify inverse/FK round trips, continuous squares and failure handling."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
harness=r'''
#include "leg_trajectory.h"
#include "leg_model.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
static void check(float x,float y,float rear,float front) {
 float out[2];LegModel m;
 assert(leg_inverse(x,y,rear,front,out));
 assert(leg_model(out[0],out[1],LEG_BRANCH_MATLAB,&m));
 assert(hypotf(.03f+m.length*cosf(m.angle)-x,m.length*sinf(m.angle)-y)<2e-5f);
}
int main(void) {
 for(int a=60;a<=120;a+=3) for(int b=60;b<=120;b+=3) {
  float rear=a*.01745329252f,front=b*.01745329252f;LegModel m;
  if(leg_model(rear,front,LEG_BRANCH_MATLAB,&m))
   check(.03f+m.length*cosf(m.angle),m.length*sinf(m.angle),rear,front);
 }
 float rear=1.5707963268f,front=rear;
 LegTrajectoryPoint p;
 for(int step=0;step<=3200;step++) {
  assert(leg_square_sample(.03f,.14f,.01f,step*.005f,rear,front,&p));
  if(step) assert(fabsf(p.rear_rad-rear)<.03f && fabsf(p.front_rad-front)<.03f);
  assert(fabsf(fabsf(p.x_m-.03f)-.005f)<1e-6f || fabsf(fabsf(p.y_m-.14f)-.005f)<1e-6f);
  check(p.x_m,p.y_m,p.rear_rad,p.front_rad);rear=p.rear_rad;front=p.front_rad;
 }
 float out[2]={8,9};assert(!leg_inverse(.5f,.5f,rear,front,out));assert(out[0]==0&&out[1]==0);
 assert(!leg_inverse(NAN,0,rear,front,out));assert(!leg_inverse(.03f,.14f,rear,front,0));
 assert(!leg_square_sample(.03f,.14f,-.01f,0,rear,front,&p));assert(p.x_m==0);
 assert(!leg_square_sample(.03f,.3f,.01f,0,rear,front,&p));assert(p.x_m==0);
 assert(leg_square_sample(.03f,.14f,.01f,-1,rear,front,&p));
 LegTrajectoryPoint plus,minus;
 assert(leg_square_sample(.03f,.14f,.01f,.3701f,rear,front,&plus));
 assert(leg_square_sample(.03f,.14f,.01f,.3699f,rear,front,&minus));
 assert(leg_square_sample(.03f,.14f,.01f,.37f,rear,front,&p));
 assert(fabsf(p.dx_dphase-(plus.x_m-minus.x_m)/.0002f)<2e-5f);
 assert(fabsf(p.dy_dphase-(plus.y_m-minus.y_m)/.0002f)<2e-5f);
 for(int corner=0;corner<=4;corner++) {
  assert(leg_square_sample(.03f,.14f,.01f,(float)corner,rear,front,&p));
  assert(p.dx_dphase==0 && p.dy_dphase==0);
 }
 puts("PASS: 441 FK/IK poses, four continuous square cycles, reachable boundaries and invalid inputs");
}
'''
with tempfile.TemporaryDirectory() as directory:
    tmp=Path(directory);(tmp/'test.c').write_text(harness);exe=tmp/'test.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-O2',
                    '-I',str(root/'Control/WheelLeg/Inc'),
                    '-I',str(root/'tests/support/wheel_leg/Inc'),str(tmp/'test.c'),
                    str(root/'tests/support/wheel_leg/Src/leg_trajectory.c'),
                    str(root/'Control/WheelLeg/Src/leg_pos.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
