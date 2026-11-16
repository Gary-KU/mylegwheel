"""Historical1208 mathematical screen; failed hardware trial, disabled by default.

Nominal sampled linear plant with relative-leg PD, at both 1.47 and 1.6kg.
This does not model contact, friction breakaway, driver delays or real stability.
The field replay verifies correction direction/slew, not a simulated new run.
"""
import copy
import json
from pathlib import Path
import numpy as np
from verify_lqr_extended_domain import discrete_plant, source_float

root=Path(__file__).resolve().parents[1]
cfg=(root/'App/Tasks/Inc/wheel_leg_config.h').read_text(encoding='utf-8')
ki=source_float(cfg,'LEG_ANGLE_KI_NM_PER_RAD_S')
cap=source_float(cfg,'LEG_ANGLE_I_LIMIT_NM')
slew=source_float(cfg,'LEG_ANGLE_I_RATE_NM_S')
band=source_float(cfg,'LEG_ANGLE_I_DEADBAND_RAD')
kp=source_float(cfg,'LEG_DEFAULT_ANGLE_KP_NM_PER_RAD')
kd=source_float(cfg,'LEG_DEFAULT_ANGLE_KD_NM_S_PER_RAD')
p0=json.loads((root/'models/lqr/lqr_model_metadata.json').read_text())['parameters']
co=np.loadtxt(root/'models/lqr/lqr_coefficients.csv',delimiter=',')
relative=np.array([1.,0,0,0,1.,0])
worst=0.0
for mass in (1.47,1.6):
    p=copy.deepcopy(p0)
    ratio=(mass-p['non_body_mass_kg'])/p['body_mass_kg']
    p['body_mass_kg']*=ratio
    p['body_pitch_inertia_per_side']*=ratio
    for length in np.linspace(.04,.15,111):
        a,b=discrete_plant(p,length)
        k=(co@[length**3,length**2,length,1]).reshape(2,6,order='F')
        pd=np.zeros((2,6));pd[1]=[kp,kd,0,0,kp,kd]
        closed=np.zeros((7,7))
        closed[:6,:6]=a-b@(k+pd)-np.outer(b[:,1],ki*p['Ts']*relative)
        closed[:6,6]=b[:,1]
        closed[6,:6]=-ki*p['Ts']*relative;closed[6,6]=1
        radius=float(max(abs(np.linalg.eigvals(closed))))
        assert radius<1,(mass,length,radius)
        worst=max(worst,radius)

trial=json.loads((root/'diagnostics/backward1206_latest_before1208_20261112.json').read_text())
bias=np.zeros(2);eligible=0
for row in trial['records']:
    if not row['active']:
        continue
    for side in range(2):
        error=row['angle_reference_rad'][side]-row['leg_angle_rad'][side]
        if (row['phase']==3 and abs(row['forward_target_m_s'])<.01 and
            abs(row['yaw_target_rad_s'])<.1 and abs(row['body_pitch_deg'])<10 and
            abs(error)<np.deg2rad(20) and abs(row['angle_speed_rad_s'][side])<.5):
            eligible+=1
            before=bias[side]
            e=np.sign(error)*max(0,abs(error)-band)
            # Recorded at40ms: integrate elapsed record time, not ISR time.
            delta=np.clip(ki*e,-slew,slew)*trial['header']['period_ms']*.001
            candidate=np.clip(before+delta,-cap,cap)
            pd=kp*error-kd*row['angle_speed_rad_s'][side]
            if abs(pd+candidate)<=.16 or delta*(pd+candidate)<0:
                bias[side]=candidate
            assert abs(bias[side]-before)<=slew*.04+1e-12
assert eligible>0 and np.all(bias>0),bias
assert max(abs(bias))<=cap
assert 'LEG_ANGLE_INTEGRAL_ENABLED                 0U' in cfg
report={'result':'mathematical screen only; hardware report FAILED, production integral disabled',
        'production_integral_enabled':False,'worst_sampled_pole_radius':worst,
        'mass_kg':[1.47,1.6],'leg_range_m':[.04,.15],
        'field_replay_eligible_leg_samples':eligible,
        'field_replay_positive_bias_nm':bias.tolist(),
        'ki_nm_per_rad_s':ki,'limit_nm':cap,'slew_nm_s':slew,
        'limitations':__doc__}
output=root/'diagnostics/loaded_leg_integral_1208_model_check.json'
output.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
