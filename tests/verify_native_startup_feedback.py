"""Screen the retained original six-state startup feedback and small-angle model.

The truncated PREPARE law is a counterfactual applied to the current gain,
physical parameters and configuration. No frozen build1214/1215 inputs are
loaded, so neither comparison is a reconstruction of a historical build.
The controller modeled here uses Control/WheelLeg/Src/lqr_k.c, not the new
default LQI gain table or measured-stop reference governor. This standalone
Python model does not compile the task or override its default settings.
This test retains the current complete-loop stability assertion. It does not
model nonlinear standing, ground contact, physical polarity, current-loop
dynamics, friction, actuator saturation or uncertain motor torque constants.
Model poles cannot certify that the actual robot balances.
"""
import json
import hashlib
from pathlib import Path
import numpy as np
from verify_foc_sampled_timing import BOARD, coefficients, continuous_plant, exponential, macro

ROOT = Path(__file__).resolve().parents[1]


def model(parameters, length, truncated, co, tau, period, pd):
    a0, b0 = continuous_plant(parameters, length)
    a = np.zeros((8, 8))
    b = np.zeros((8, 2))
    a[:6, :6] = a0
    b[:6] = b0
    a[6, [1, 5, 6]] = [1/tau, 1/tau, -1/tau]
    a[7, [3, 7]] = [1/tau, -1/tau]
    measurement = np.zeros((6, 8))
    measurement[0, 0] = measurement[2, 2] = measurement[4, 4] = measurement[5, 5] = 1
    measurement[1, 6] = 1
    measurement[1, 5] = -1
    measurement[3, 7] = 1
    measurement[2, 7] = -tau
    k = (co @ [length**3, length**2, length, 1]).reshape((2, 6), order='F')
    if truncated:
        # Counterfactual PREPARE follows measured theta/x every frame,
        # removes hip LQR and substitutes relative-leg absolute PD.
        k[0, [0, 2]] = 0
        k[1] = 0
    feedback = k @ measurement
    if truncated:
        feedback[1, [0, 4]] += pd[0]
        feedback[1, 6] += pd[1]
    augmented = np.zeros((10, 10))
    augmented[:8, :8] = a
    augmented[:8, 8:] = b
    discrete = exponential(augmented*period)
    poles = np.linalg.eigvals(discrete[:8, :8]-discrete[:8, 8:] @ feedback)
    return float(max(abs(poles))), [[float(v.real), float(v.imag)] for v in poles]


def main():
    parameters = json.loads((ROOT/'models/lqr/lqr_model_metadata.json').read_text())['parameters']
    co=coefficients()
    tau=macro(BOARD/'Config/motor_setup.h','MOTOR_SPEED_FILTER_TAU_S')
    period=macro(ROOT/'App/Services/Inc/motor_foc_config.h','MOTOR_FOC_COMMAND_PERIOD_MS')*.001
    configuration=ROOT/'App/Tasks/Inc/wheel_leg_config.h'
    pd=[macro(configuration,'LEG_DEFAULT_ANGLE_KP_NM_PER_RAD'),
        macro(configuration,'LEG_DEFAULT_ANGLE_KD_NM_S_PER_RAD')]
    cases = []
    for mass in dict.fromkeys((parameters['total_mass_kg'], 1.600)):
        p = dict(parameters)
        p['body_mass_kg'] = mass-p['non_body_mass_kg']
        p['body_pitch_inertia_per_side'] *= p['body_mass_kg']/parameters['body_mass_kg']
        for truncated in (True, False):
            points = []
            for length in np.linspace(.070, .090, 21):
                radius, _ = model(p, float(length), truncated,co,tau,period,pd)
                points.append(dict(length_m=float(length), pole_radius=radius))
            radius, poles = model(p, .077, truncated,co,tau,period,pd)
            cases.append(dict(mass_kg=mass, law='current_counterfactual_truncated_prepare' if truncated else 'current_complete_coupled_lqr',
                              radius_at_77mm=radius, poles_at_77mm=poles,
                              worst_radius=max(point['pole_radius'] for point in points), points=points))
            if not truncated:
                assert all(point['pole_radius'] < 1 for point in points), 'Nominal coupled loop is unstable'
    provenance_paths={'coefficients':'Control/WheelLeg/Src/lqr_k.c',
        'model_metadata':'models/lqr/lqr_model_metadata.json',
        'task_configuration':'App/Tasks/Inc/wheel_leg_config.h',
        'master_configuration':'App/Services/Inc/motor_foc_config.h'}
    report = dict(limits=__doc__,comparison_scope='Current-input counterfactual; no frozen1214/1215 coefficient/model inputs loaded.',
        source_provenance={key:dict(path=path,sha256=hashlib.sha256((ROOT/path).read_bytes()).hexdigest())
                           for key,path in provenance_paths.items()},
        model_parameters=parameters,period_ms=period*1000,velocity_filter_ms=tau*1000,
        counterfactual_relative_leg_pd=pd,cases=cases)
    destination = ROOT/'diagnostics/native_startup_feedback_current_counterfactual.json'
    destination.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('PASS: current complete coupled loop passes nominal 7..9cm screen; truncation is a current-input counterfactual, not historical reconstruction')
    print('Report:',destination)
    print(json.dumps([{key:value for key,value in case.items()
                       if key not in ('points', 'poles_at_77mm')} for case in cases], indent=2))


if __name__ == '__main__':
    main()
