"""Offline timing screen using the current coefficient/model/configuration files.

The hold-ratio-one/10ms comparison is a current-input counterfactual. No
verified frozen coefficient/model pair tied to build1211 is loaded, so this
test does not reconstruct that historical deployment or its pole radius.

The physical plant is the project's six-state small-angle model. Two states
approximate each F103's 20 ms velocity low-pass: relative-leg angular speed
and wheel travel speed. The first-order filters are continuous proxies for
the encoder's approximately 1 ms backward-Euler update. The augmented plant
is exactly ZOH-discretized by a scaling/squaring matrix exponential for each
candidate command interval. No current-loop, friction, CAN phase/jitter,
feedback packet holding, current slew, saturation or contact is modeled.

This can expose a nominal stability risk omitted by A-BK tests; it cannot
prove the physical cause, coordinate conventions or safe hardware behavior.
"""
from pathlib import Path
import hashlib
import json
import re
import numpy as np
from verify_lqr_extended_domain import exponential

ROOT=Path(__file__).resolve().parents[1]
BOARD=ROOT.parents[3]/'Gimbal_borad/software'


def macro(path,name):
    text=path.read_text(encoding='utf-8')
    m=re.search(r'^#define\s+'+re.escape(name)+r'\s+([0-9.eE+-]+)[fFuU]?\b',text,re.M)
    assert m,(path,name)
    return float(m[1])


def coefficients():
    source=(ROOT/'Control/WheelLeg/Src/lqr_k.c').read_text(encoding='utf-8')
    result=[]
    for index in range(12):
        expression=re.search(r'K\['+str(index)+r'\]\s*=\s*([^;]+);',source)[1]
        literals=re.findall(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?f',expression)
        assert len(literals)==4,(index,literals)
        result.append([float(item[:-1]) for item in literals])
    return np.array(result)


def continuous_plant(p,length):
    half=length/2;mass=p['body_mass_kg']/2;leg=p['leg_mass_per_side_kg']
    wheel=p['wheel_mass_per_side_kg'];radius=p['wheel_radius_m'];h=p['body_com_height_m']
    inertia=np.array([
        [mass+leg+wheel+p['wheel_inertia_per_side']/radius**2,mass*length+leg*half,-mass*h],
        [mass*length+leg*half,p['leg_inertia_per_side']+leg*half**2+mass*length**2,-mass*length*h],
        [-mass*h,-mass*length*h,p['body_pitch_inertia_per_side']+mass*h*h]])
    a=np.zeros((6,6));b=np.zeros((6,2))
    a[0,1]=a[2,3]=a[4,5]=1
    a[[3,1,5],0]=np.linalg.solve(inertia,[0,(leg*half+mass*length)*p['gravity_m_s2'],0])
    a[[3,1,5],4]=np.linalg.solve(inertia,[0,0,mass*p['gravity_m_s2']*h])
    b[[3,1,5]]=np.linalg.solve(inertia,[[1/radius,0],[-1,1],[0,1]])
    return a,b


def closed_loop(p,length,period,tau,kp,kd,co,filtered):
    base_a,base_b=continuous_plant(p,length)
    k=(co@[length**3,length**2,length,1]).reshape((2,6),order='F')
    if not filtered:
        a,b=base_a,base_b
        pd=np.zeros((2,6));pd[1]=[kp,kd,0,0,kp,kd]
        feedback=k+pd
    else:
        # z=[theta,dtheta,x_true,dx_true,phi,dphi,alpha_rate_LPF,dx_LPF].
        # alpha=theta+phi is relative-leg inclination. Body rate is unfiltered.
        a=np.zeros((8,8));b=np.zeros((8,2))
        a[:6,:6]=base_a;b[:6]=base_b
        a[6,[1,5,6]]=[1/tau,1/tau,-1/tau]
        a[7,[3,7]]=[1/tau,-1/tau]
        measurement=np.zeros((6,8))
        measurement[0,0]=measurement[2,2]=measurement[4,4]=measurement[5,5]=1
        measurement[1,6]=1;measurement[1,5]=-1
        measurement[3,7]=1
        # For matched initial conditions, integral(dx_LPF)=x_true-tau*dx_LPF.
        # Actual 5 ms trapezoidal integration/packet holding adds omitted error.
        measurement[2,7]=-tau
        feedback=k@measurement
        feedback[1,[0,4]]+=kp;feedback[1,6]+=kd
    n=len(a);augmented=np.zeros((n+2,n+2))
    augmented[:n,:n]=a;augmented[:n,n:]=b
    discrete=exponential(augmented*period)
    poles=np.linalg.eigvals(discrete[:n,:n]-discrete[:n,n:]@feedback)
    return float(max(abs(poles))),[[float(v.real),float(v.imag)] for v in poles]


def main():
    config=ROOT/'App/Tasks/Inc/wheel_leg_config.h'
    master=ROOT/'App/Services/Inc/motor_foc_config.h'
    board_can=BOARD/'Config/can_config.h'
    p0=json.loads((ROOT/'models/lqr/lqr_model_metadata.json').read_text())['parameters']
    hold=macro(config,'BALANCE_DEFAULT_LEG_HOLD_RATIO')
    base_kp=macro(config,'LEG_DEFAULT_ANGLE_KP_NM_PER_RAD')
    base_kd=macro(config,'LEG_DEFAULT_ANGLE_KD_NM_S_PER_RAD')
    kp=hold*base_kp
    kd=hold*base_kd
    tau=macro(BOARD/'Config/motor_setup.h','MOTOR_SPEED_FILTER_TAU_S')
    command_ms=macro(master,'MOTOR_FOC_COMMAND_PERIOD_MS')
    loop_ms=macro(ROOT/'App/Tasks/Inc/wheel_leg_runtime.h','WHEEL_LEG_CONTROL_PERIOD_MS')
    co=coefficients();cases=[]
    for total_mass in dict.fromkeys((p0['total_mass_kg'],1.6)):
        p=dict(p0);body=total_mass-p['non_body_mass_kg']
        p['body_pitch_inertia_per_side']*=body/p['body_mass_kg'];p['body_mass_kg']=body
        for filtered in (False,True):
            for ms in sorted({5,10,command_ms}):
                points=[]
                for length in np.linspace(.04,.15,111):
                    radius,poles=closed_loop(p,float(length),ms*.001,tau,kp,kd,co,filtered)
                    points.append({'leg_length_m':float(length),'pole_radius':radius})
                relevant=[v for v in points if .06999<=v['leg_length_m']<=.09001]
                sample_radius,sample_poles=closed_loop(p,.077,ms*.001,tau,kp,kd,co,filtered)
                cases.append({'total_mass_kg':total_mass,'command_period_ms':ms,
                    'velocity_filter_in_model':filtered,
                    'worst_radius_4_to_15cm':max(v['pole_radius'] for v in points),
                    'worst_radius_7_to_9cm':max(v['pole_radius'] for v in relevant),
                    'radius_at_7_7cm':sample_radius,'poles_at_7_7cm':sample_poles,
                    'stable_nominal_7_to_9cm':all(v['pole_radius']<1 for v in relevant),
                    'points':points})
    nominal_current=next(c for c in cases if c['total_mass_kg']==p0['total_mass_kg'] and c['velocity_filter_in_model'] and c['command_period_ms']==command_ms)
    # This changes the current controller's local hold ratio and command
    # interval only. Its current gain/model inputs are not historical1211.
    counterfactual_radius,_=closed_loop(p0,.077,.010,tau,base_kp,base_kd,co,True)
    assert nominal_current['stable_nominal_7_to_9cm'], 'Current nominal coupled loop is unstable'
    fast_hz=1e6/macro(board_can,'CAN_FAST_FEEDBACK_PERIOD_US')
    status_hz=1e3/macro(board_can,'CAN_FEEDBACK_PERIOD_MS')
    periodic=6*(fast_hz+3*status_hz) # FAST plus STATUS, DIAG and one TRACE page.
    bus=[]
    for hz in (100,200):
        frames=periodic+6*hz*2 # One current command plus executed-command ACK.
        bus.append({'command_hz':hz,'six_node_periodic_frames_s':periodic,
            'command_plus_ack_frames_s':12*hz,'total_frames_s':frames,
            'load_at_1Mbps_minimal111bit_frame':frames*111/1e6,
            'load_at_1Mbps_pessimistic135bit_frame':frames*135/1e6})
    provenance_paths={'coefficients':'Control/WheelLeg/Src/lqr_k.c',
        'model_metadata':'models/lqr/lqr_model_metadata.json',
        'task_configuration':'App/Tasks/Inc/wheel_leg_config.h',
        'master_configuration':'App/Services/Inc/motor_foc_config.h'}
    report={'result':'current nominal setting screened; current-input counterfactual comparison; not hardware validation',
        'comparison_scope':'Current coefficient/model inputs with hold_ratio=1 and command_period=10ms; not a build1211 reconstruction. No verified frozen1211 coefficient/model pair is loaded.',
        'counterfactual_hold1_10ms_7_7cm_pole_radius':counterfactual_radius,
        'counterfactual_hold1_10ms_stable_at_7_7cm':counterfactual_radius<1,
        'source_provenance':{key:{'path':path,'sha256':hashlib.sha256((ROOT/path).read_bytes()).hexdigest()}
                             for key,path in provenance_paths.items()},
        'model_parameters':p0,
        'source_settings':{'lqr_design_ms':p0['Ts']*1000,'task_ms':loop_ms,
            'current_command_ms':command_ms,'velocity_filter_tau_s':tau,
            'balance_leg_hold_ratio':hold,'relative_leg_PD':[kp,kd],
            'fast_hz':fast_hz,'status_diag_trace_hz':status_hz},
        'assumptions_and_limits':__doc__,'cases':cases,'bus_budget':bus,
        'bus_budget_limits':'8-byte standard classical CAN.111bits includes intermission without stuffing;135bits is pessimistic stuffing budget. Excludes errors,retransmissions,initialization/maintenance/other nodes; six-node burst phasing still needs measurement.'}
    output=ROOT/'diagnostics/foc_sampled_timing_current_counterfactual.json'
    output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'comparison_scope':report['comparison_scope'],
        'counterfactual_hold1_10ms_7_7cm_pole_radius':counterfactual_radius,
        'source_settings':report['source_settings'],'cases':[
        {k:v for k,v in c.items() if k not in ('points','poles_at_7_7cm')} for c in cases],
        'bus_budget':bus,'report':str(output)},indent=2))


if __name__=='__main__':main()
