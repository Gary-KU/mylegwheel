"""Generate coupled two-input LQI gain lookup and report interpolation/poles.

No target or hardware access. Coefficients use the declared model, not an
identified robot. Dynamic current-limit and slew screens are separate checks.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tests'))
from verify_foc_sampled_timing import continuous_plant, coefficients
from verify_lqr_extended_domain import discrete_plant, exponential

def dare(a,b,q,r):
    aa,gg,hh,eye=a.copy(),b@np.linalg.solve(r,b.T),q.copy(),np.eye(len(a))
    for _ in range(100):
        w=np.linalg.solve(eye+gg@hh,aa)
        an=aa@w;gn=gg+aa@gg@np.linalg.solve(eye+hh@gg,aa.T);hn=hh+aa.T@hh@w
        gn,hn=(gn+gn.T)/2,(hn+hn.T)/2
        if np.linalg.norm(hn-hh,np.inf)<1e-11*max(1,np.linalg.norm(hn,np.inf)):
            hh=hn;break
        aa,gg,hh=an,gn,hn
    k=np.linalg.solve(r+b.T@hh@b,b.T@hh@a)
    residual=a.T@hh@a-a.T@hh@b@k+q-hh
    assert np.linalg.norm(residual,np.inf)/np.linalg.norm(hh,np.inf)<1e-9
    return k

def gain(p,length,qi,rs,rate,phi_scale=1):
    a,b=discrete_plant(p,length)
    aa=np.eye(7);aa[:6,:6]=a;aa[6,:6]=p['Ts']*np.eye(6)[2]
    bb=np.zeros((7,2));bb[:6]=b
    q=np.array([*p['Q'],qi],float);q[[1,3,5]]*=rate;q[4]*=phi_scale
    return dare(aa,bb,np.diag(q),np.diag(p['R'])*rs)

def filtered_plant(p,length,mass,gw,gh,tau):
    pp=copy.deepcopy(p);body=mass-p['non_body_mass_kg'];pp['body_pitch_inertia_per_side']*=body/p['body_mass_kg'];pp['body_mass_kg']=body
    ac,bc=continuous_plant(pp,length);a,b=np.zeros((8,8)),np.zeros((8,2));a[:6,:6]=ac;b[:6]=bc@np.diag([gw,gh]);a[6,[1,5,6]]=[1/tau,1/tau,-1/tau];a[7,[3,7]]=[1/tau,-1/tau]
    z=np.zeros((10,10));z[:8,:8]=a;z[:8,8:]=b;zz=exponential(z*p['Ts']);return zz[:8,:8],zz[:8,8:]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--qi',type=float,default=10)
    ap.add_argument('--r-scale',type=float,default=1)
    ap.add_argument('--rate-scale',type=float,default=1)
    ap.add_argument('--phi-scale',type=float,default=1)
    ap.add_argument('--output',type=Path,default=ROOT/'Control/WheelLeg/Inc/balance_servo_gains.h')
    ap.add_argument('--report',type=Path,default=ROOT/'diagnostics/parking_servo_gain_generation_1424.json')
    ap.add_argument('--quick',action='store_true',help='Use coarser length grid for exploratory pole validation')
    args=ap.parse_args()
    source_paths=['models/lqr/lqr_model_metadata.json','Control/WheelLeg/Src/lqr_k.c','Control/WheelLeg/Inc/lqr_model_config.h','App/Tasks/Src/wheel_leg_task.c','App/Tasks/Inc/wheel_leg_config.h','App/Services/Inc/motor_foc_config.h','Control/WheelLeg/Inc/balance_servo_gains.h','Control/WheelLeg/Inc/balance_motion_reference.h','tests/verify_native_parking_closed_loop.py','tests/verify_foc_sampled_timing.py','tests/verify_lqr_extended_domain.py','scripts/generate_parking_servo_1424.py']
    def hashes():return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in source_paths if (ROOT/name).is_file() and (ROOT/name).resolve()!=args.output.resolve()}
    before_hashes=hashes()
    meta=ROOT/'models/lqr/lqr_model_metadata.json';p=json.loads(meta.read_text())['parameters'];assert p['Ts']==.005
    knots=np.linspace(.04,.15,45);ks=np.array([gain(p,float(L),args.qi,args.r_scale,args.rate_scale,args.phi_scale) for L in knots]);table=np.array([np.r_[k[:,:6].reshape(-1,order='F'),k[:,6]] for k in ks],np.float32)
    co=coefficients();dt=p['Ts'];M=np.zeros((6,8));M[0,0]=M[2,2]=M[4,4]=M[5,5]=M[1,6]=M[3,7]=1;M[1,5]=-1;M[2,7]=-.02
    worst={};supported_worst={};nominal_worst={};max_abs_error=0.;max_normalized_error=0.;failures=[]
    for L in np.linspace(.04,.15,23 if args.quick else 221):
        index=min(43,int(math.floor((L-.04)/.0025+1e-10)));t=(L-knots[index])/.0025
        vals=table[index]*(1-t)+table[index+1]*t;k=vals[:12].reshape((2,6),order='F').astype(float);ki=vals[12:].astype(float);exact=gain(p,float(L),args.qi,args.r_scale,args.rate_scale,args.phi_scale);full=np.column_stack((k,ki));max_abs_error=max(max_abs_error,float(np.max(abs(full-exact))));max_normalized_error=max(max_normalized_error,float(np.max(abs(full-exact)/np.maximum(.001,abs(exact)))))
        old=(co@np.array([L**3,L**2,L,1])).reshape((2,6),order='F')
        for mass in (1.47,1.6):
            for gw in (.7,1,1.3):
                for gh in (.7,1,1.3):
                    a,b=filtered_plant(p,float(L),mass,gw,gh,.02)
                    for alpha in np.linspace(0,1,11):
                        mix=(1-alpha)*old+alpha*k;ii=alpha*ki
                        cl=np.zeros((9,9));cl[:8,:8]=a-b@mix@M;cl[:8,8]=-b@ii;cl[8,:8]=dt*M[2];cl[8,8]=1
                        # Integral freezes at blend=0. Its unit memory pole is
                        # expected and excluded from that endpoint's plant test.
                        rho=float(max(abs(np.linalg.eigvals(cl[:8,:8] if alpha==0 else cl))))
                        key='baseline' if alpha==0 else 'mixed' if alpha<1 else 'servo'
                        if key not in worst or rho>worst[key]['radius']:worst[key]=dict(radius=rho,case=[float(L),mass,gw,gh,float(alpha)])
                        if .07-1e-9<=L<=.12+1e-9 and (key not in supported_worst or rho>supported_worst[key]['radius']):supported_worst[key]=dict(radius=rho,case=[float(L),mass,gw,gh,float(alpha)])
                        if mass==1.6 and gw==gh==1 and (key not in nominal_worst or rho>nominal_worst[key]['radius']):nominal_worst[key]=dict(radius=rho,case=[float(L),mass,gw,gh,float(alpha)])
                        if rho>=1+1e-9:failures.append(dict(case=[float(L),mass,gw,gh,float(alpha)],radius=rho))
    dc_rows=[];g=p['gravity_m_s2'];ml=p['leg_mass_per_side_kg'];h=p['body_com_height_m'];mnom=p['body_mass_kg']/2;phiref=math.radians(4)
    for L in (.07,.095,.12):
        kk=gain(p,L,args.qi,args.r_scale,args.rate_scale,args.phi_scale)
        thref=mnom*h*phiref/((mnom+ml/2)*L);tpff=-mnom*g*h*phiref
        for mass in (1.47,1.6):
            m=(mass-p['non_body_mass_kg'])/2
            for gh in (.7,1.,1.3):
                for offset in (-.005,-.003,-.001,.001,.003,.005):
                    mat=np.array([[(m+ml/2)*g*L,0,0,gh],[0,m*g*h,0,gh],[kk[0,0],kk[0,4],kk[0,6],0],[kk[1,0],kk[1,4],kk[1,6],1]])
                    rhs=np.array([0,m*g*offset,kk[0,0]*thref+kk[0,4]*phiref,tpff+kk[1,0]*thref+kk[1,4]*phiref])
                    th,phi,integ,tp=np.linalg.solve(mat,rhs)
                    dc_rows.append(dict(case=[L,mass,gh,offset],theta_deg=math.degrees(th),phi_deg=math.degrees(phi),integral_m_s=integ,pitch_torque_nm=tp))
    values='\n'.join('    {'+', '.join(f'{float(v):.9g}f' if '.' in f'{float(v):.9g}' or 'e' in f'{float(v):.9g}' else f'{float(v):.9g}.0f' for v in row)+'},' for row in table)
    header=f'''/* Generated by scripts/generate_parking_servo_1424.py.
 * Discrete two-input LQI, z_next=z+dt*(x-goal); u=ff-K*error-Ki*z.
 * Q_integral={args.qi:g}, R_scale={args.r_scale:g}, rate_Q_scale={args.rate_scale:g}, phi_Q_scale={args.phi_scale:g}, dt=.005s.
 * Both complete feedback rows are generated jointly; no row scaling.
 * Piecewise linear2.5mm lookup. Model validation is not hardware validation.
 */
#ifndef BALANCE_SERVO_GAINS_H
#define BALANCE_SERVO_GAINS_H
#include <math.h>
static const float balance_servo_gain_table[45][14]={{
{values}
}};
static inline int BalanceServo_Gain(float length,float K[12],float Ki[2]) {{
    if(!K || !Ki || !isfinite(length) || length<0.04f || length>0.15f)return 0;
    float grid=(length-0.04f)*400.0f;
    unsigned index=(unsigned)grid;
    if(index>43U)index=43U;
    float t=grid-(float)index;
    if(t<0)t=0;
    if(t>1)t=1;
    for(unsigned i=0;i<12;++i)K[i]=balance_servo_gain_table[index][i]+t*(balance_servo_gain_table[index+1][i]-balance_servo_gain_table[index][i]);
    for(unsigned i=0;i<2;++i)Ki[i]=balance_servo_gain_table[index][12+i]+t*(balance_servo_gain_table[index+1][12+i]-balance_servo_gain_table[index][12+i]);
    return 1;
}}
#endif
'''
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(header,encoding='utf-8')
    after_hashes=hashes()
    dc_summary=dict(cases=len(dc_rows),max_abs_integral_m_s=max(abs(row['integral_m_s']) for row in dc_rows),max_abs_theta_deg=max(abs(row['theta_deg']) for row in dc_rows),max_abs_phi_deg=max(abs(row['phi_deg']) for row in dc_rows),note='Unclipped theoretical DC solutions at zero true position/speed error, body trim-4deg; may be unreachable under current/pose/gating constraints.')
    report=dict(scope=__doc__,parameters=dict(qi=args.qi,r_scale=args.r_scale,rate_scale=args.rate_scale,phi_scale=args.phi_scale,dt_s=dt),source_sha256=before_hashes,source_after_sha256=after_hashes,source_changed_during_run=before_hashes!=after_hashes,header=str(args.output.resolve()),header_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),metadata_sha256=hashlib.sha256(meta.read_bytes()).hexdigest(),knots_m=knots.tolist(),interpolation_max_abs_error=max_abs_error,interpolation_max_normalized_error=max_normalized_error,normalized_error_floor=.001,worst_poles=worst,supported70_120_worst_poles=supported_worst,nominal40_150_worst_poles=nominal_worst,unstable_count=len(failures),unstable70_120_count=sum(.07-1e-9<=row['case'][0]<=.12+1e-9 for row in failures),unstable_cases=failures,dc_summary=dc_summary,dc_solutions=dc_rows,quick=args.quick,gain_table_column_major_K_then_Ki=table.astype(float).tolist(),limitations=['Constant length, linear small-angle plant and declared mass/inertia.','Pole checks do not include current saturation, driver slew, contact, yaw, jitter, voltage limits or variable geometry.','Blending poles are frozen-alpha local checks, not a switched-system or nonlinear stability proof.'])
    args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('gain_table_column_major_K_then_Ki','unstable_cases','knots_m','source_sha256','source_after_sha256','dc_solutions')},indent=2))

if __name__=='__main__':main()
