"""Run production gravity-bias C observer against independent coupled plants.

Nominal physics screens are not evidence of real-robot stability.
"""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from verify_lqr_extended_domain import discrete_plant, exponential
sys.path.insert(0,str(ROOT/'diagnostics'))
from analyze_native_split_20261108 import jacobian
p=json.loads((ROOT/'models/lqr/lqr_model_metadata.json').read_text())['parameters']
coefficients=np.loadtxt(ROOT/'models/lqr/lqr_coefficients.csv',delimiter=',')
def plant(height):
    q=copy.deepcopy(p);q['body_com_height_m']=height
    a,b=discrete_plant(q,.12)
    mass=q['body_mass_kg']/2;leg=q['leg_mass_per_side_kg'];radius=q['wheel_radius_m']
    e=np.array([[mass+leg+q['wheel_mass_per_side_kg']+q['wheel_inertia_per_side']/radius**2,
                 mass*.12+leg*.06,-mass*height],
                [mass*.12+leg*.06,q['leg_inertia_per_side']+leg*.06**2+mass*.12**2,-mass*.12*height],
                [-mass*height,-mass*.12*height,q['body_pitch_inertia_per_side']+mass*height**2]])
    ac=np.zeros((6,6));ac[0,1]=ac[2,3]=ac[4,5]=1
    ac[[3,1,5],0]=np.linalg.solve(e,[0,(leg*.06+mass*.12)*q['gravity_m_s2'],0])
    ac[[3,1,5],4]=np.linalg.solve(e,[0,0,mass*q['gravity_m_s2']*height])
    bc=np.zeros((6,3));bc[[3,1,5],:]=np.linalg.solve(e,[[1/radius,0,0],[-1,1,0],[0,1,-mass*q['gravity_m_s2']]])
    matrix=np.zeros((9,9));matrix[:6,:6]=ac;matrix[:6,6:]=bc
    full=exponential(matrix*q['Ts'])
    assert np.max(abs(a-full[:6,:6]))<1e-12 and np.max(abs(b-full[:6,6:8]))<1e-12
    return a,b,full[:6,8]
def values(array):
    return '{'+','.join(format(float(x),'.17g') for x in np.asarray(array).ravel())+'}'
data=[]
for height in (.01,.025,.04,.06):
    a,b,d=plant(height)
    data.append('{'+f'{height},'+values(a)+','+values(b)+','+values(d)+'}')
angle=math.atan2(.12,.03)
alpha=math.acos((.03**2+.12**2+.05**2-.105**2)/(2*math.hypot(.03,.12)*.05))
j=np.asarray(jacobian(angle+alpha,math.pi-angle-alpha)).reshape(2,2)
header='''#include "balance_bias_observer.h"
#include "lqr_k.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct {double h,a[36],b[12],d[6];} Plant;
static const Plant plant[4]={'''+','.join(data)+'''};
static const double J[4]='''+values(j)+''';
static double clip(double x,double lo,double hi){return fmax(lo,fmin(hi,x));}
int main(void) {
 float k[12];lqr_k(.12f,k);
 for(unsigned case_id=0;case_id<4;++case_id)for(int sign=-1;sign<=1;++sign) {
  const Plant *p=&plant[case_id];BalanceBiasState observer={0};
  double x[6]={.01745329252,0,0,0,0,0},iq[3]={J[0]*9/.23,J[1]*9/.23,0};
  double offset=p->h*tan(sign*.034906585f),tp=0,peak=0;
  for(unsigned step=0;step<4000;++step) {
   float sample[6];for(unsigned i=0;i<6;++i)sample[i]=(float)x[i];
   float previous=observer.phi_reference_rad;
   unsigned learn=fabs(x[4])<.2618 && fabs(x[5])<.8 && fabs(x[3])<.3;
   BalanceBias_Update(&observer,sample,.12f,(float)tp,.005f,learn);
   assert(fabsf(observer.phi_reference_rad-previous)<=.004363323f*.005f+1e-7f);
   assert(fabsf(observer.phi_reference_rad)<=.034906585f+1e-6f);
   double u[2]={0};for(unsigned i=0;i<6;++i)for(unsigned row=0;row<2;++row)
    u[row]-=k[row+2*i]*(x[i]-(i==4?observer.phi_reference_rad:0));
   double request[3]={(J[0]*9+J[2]*u[1])/.23,(J[1]*9+J[3]*u[1])/.23,u[0]/.07643};
   for(unsigned i=0;i<3;++i)iq[i]+=clip(clip(request[i],-2,2)-iq[i],-.05,.05);
   double emf=(2.0/3.0)*.07643*x[3]/.026;
   iq[2]=clip(iq[2],(-5-emf)/4.5,(5-emf)/4.5);
   double det=J[0]*J[3]-J[1]*J[2];
   tp=(-J[1]*iq[0]+J[0]*iq[1])*.23/det;
   double input[2]={iq[2]*.07643,tp},next[6]={0};
   for(unsigned row=0;row<6;++row){
    next[row]=p->d[row]*offset;
    for(unsigned col=0;col<6;++col)next[row]+=p->a[6*row+col]*x[col];
    for(unsigned col=0;col<2;++col)next[row]+=p->b[2*row+col]*input[col];
   }
   memcpy(x,next,sizeof(x));peak=fmax(peak,fabs(x[4]));
   assert(fabs(x[0])<.2618 && fabs(x[4])<.2618);
  }
  assert(fabs(observer.phi_reference_rad-sign*.034906585f)<.003);
  assert(fabs(x[2])<.012 && fabs(x[3])<.005);
  if(sign<0)assert(observer.phi_reference_rad<0); /* Rear bias => positive forward body-pitch target. */
  printf("CASE h=%.3f sign=%d learned_phi=%.3fdeg final_x=%.5fm peak_pitch=%.3fdeg\\n",
   p->h,sign,observer.phi_reference_rad*57.2957795,x[2],peak*57.2957795);
 }
 BalanceBiasState s={0};float x[6]={0};
 BalanceBias_Update(&s,x,.12f,-.02f,.005f,1);
 for(unsigned i=0;i<500;++i)BalanceBias_Update(&s,x,.12f,-.02f,.005f,1);
 float learned=s.offset_m;unsigned count=s.samples;
 for(unsigned i=0;i<100;++i)BalanceBias_Update(&s,x,.12f,.05f,.005f,0);
 assert(s.offset_m==learned && s.samples==count);
 BalanceBias_ResetMotion(&s);assert(s.offset_m==learned && !s.initialized);
 assert(!BalanceBias_Update(&s,x,.12f,NAN,.005f,1));
 assert(!BalanceBias_Update(&s,x,.12f,0,0,1));
 s=(BalanceBiasState){0};
 for(unsigned i=0;i<2000;++i) {
  float noise=(i&1U)?.002f:-.002f;
  x[5]=noise;x[1]=-noise;x[3]=noise*.5f;
  BalanceBias_Update(&s,x,.12f,noise,.005f,1);
 }
 assert(fabsf(s.phi_reference_rad)<.00872665f);
 puts("PASS: production observer, rear/front signs, height10..60mm,5V wheel ceiling,2A/10A/s,freeze,rate/bound and invalid-input checks");
}
'''
with tempfile.TemporaryDirectory(prefix='balance_bias_') as directory:
    tmp=Path(directory);(tmp/'test.c').write_text(header)
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-I',str(ROOT/'Control/WheelLeg/Inc'),
        str(tmp/'test.c'),str(ROOT/'Control/WheelLeg/Src/lqr_k.c'),'-lm','-o',str(tmp/'test.exe')],check=True)
    result=subprocess.run([str(tmp/'test.exe')],check=True,capture_output=True,text=True)
    print(result.stdout,end='')
# Independently screen estimated-height feedback across the whole gain domain.
worst=0.0
for length in np.linspace(.04,.15,45):
    k=(coefficients@[length**3,length**2,length,1]).reshape((2,6),order='F')
    for height in np.linspace(.01,.06,11):
        parameters=copy.deepcopy(p);parameters['body_com_height_m']=float(height)
        a,b=discrete_plant(parameters,float(length))
        worst=max(worst,float(max(abs(np.linalg.eigvals(a-b@k)))))
assert worst<1, worst
report={'scope':'Nominal linear-height uncertainty and production-C observer tests; no hardware stability claim',
        'nominal_height_m':p['body_com_height_m'],'height_screen_m':[.01,.06],
        'gain_length_domain_m':[.04,.15],'max_uncertain_linear_pole_radius':worst,
        'plant_test_output':result.stdout}
(ROOT/'diagnostics/adaptive_balance_tests_1222_20261114.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('PASS:4..15cm/10..60mm uncertain-height pole screen; worst radius',worst)
