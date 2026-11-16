"""Screen the fixed 4..14 cm LQR polynomial for operation through 15 cm.

Reads the production coefficients and original model metadata without changing
them. This is a nominal linear-model/extrapolation check, not a proof of motor
current-loop dynamics, Iq slew performance, loaded balance or new robot mass.
Only the independent JSON report under diagnostics/ is written.
"""
from pathlib import Path
import hashlib
import json
import math
import re

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "diagnostics/lqr_extended_domain_20261108.json"
METADATA_PATH = ROOT / "models/lqr/lqr_model_metadata.json"
CSV_PATH = ROOT / "models/lqr/lqr_coefficients.csv"
C_PATH = ROOT / "Control/WheelLeg/Src/lqr_k.c"
HEADER_PATH = ROOT / "Control/WheelLeg/Inc/lqr_k.h"
CONFIG_PATH = ROOT / "App/Tasks/Inc/wheel_leg_config.h"
EXTENDED_MAX_M = 0.15


def source_float(source, name):
    match = re.search(r"^#define\s+" + re.escape(name) + r"\s+([0-9.]+)f", source, re.M)
    assert match, name
    return float(match[1])


def exponential(matrix):
    """Scaling/squaring Taylor exponential; the scaled norm is at most 0.5."""
    norm = float(np.linalg.norm(matrix, ord=np.inf))
    squares = max(0, int(math.ceil(math.log2(norm / 0.5)))) if norm else 0
    scaled = matrix / (2 ** squares)
    term = np.eye(matrix.shape[0])
    result = term.copy()
    for index in range(1, 80):
        term = term @ scaled / index
        result += term
        if np.linalg.norm(term, ord=np.inf) < 1e-17:
            break
    else:
        raise AssertionError("Matrix exponential did not converge")
    for _ in range(squares):
        result = result @ result
    return result


def discrete_plant(parameters, length):
    # Same six-state analytical plant as models/lqr/init_lqr_model.m.
    half_length = length / 2
    mass = parameters["body_mass_kg"] / 2
    leg_mass = parameters["leg_mass_per_side_kg"]
    radius = parameters["wheel_radius_m"]
    height = parameters.get("body_com_height_m",0.0)
    inertia = np.array([
        [mass + leg_mass + parameters["wheel_mass_per_side_kg"] +
         parameters["wheel_inertia_per_side"] / radius ** 2,
         mass * length + leg_mass * half_length, -mass*height],
        [mass * length + leg_mass * half_length,
         parameters["leg_inertia_per_side"] + leg_mass * half_length ** 2 + mass * length ** 2, -mass*length*height],
        [-mass*height, -mass*length*height, parameters["body_pitch_inertia_per_side"]+mass*height**2],
    ])
    gravity = np.linalg.solve(inertia, [0, (leg_mass * half_length + mass * length) *
                                       parameters["gravity_m_s2"], 0])
    actuation = np.linalg.solve(inertia, [[1 / radius, 0], [-1, 1], [0, 1]])
    continuous_a = np.zeros((6, 6))
    continuous_a[0, 1] = continuous_a[2, 3] = continuous_a[4, 5] = 1
    continuous_a[[3, 1, 5], 0] = gravity
    continuous_a[[3, 1, 5], 4] = np.linalg.solve(inertia,[0,0,mass*parameters["gravity_m_s2"]*height])
    continuous_b = np.zeros((6, 2))
    continuous_b[[3, 1, 5], :] = actuation
    augmented = np.zeros((8, 8))
    augmented[:6, :6] = continuous_a
    augmented[:6, 6:] = continuous_b
    discrete = exponential(augmented * parameters["Ts"])
    return discrete[:6, :6], discrete[:6, 6:]


def independent_dlqr(a, b, parameters):
    q, r = np.diag(parameters["Q"]), np.diag(parameters["R"])
    cost = q.copy()
    for iteration in range(30000):
        gain = np.linalg.solve(r + b.T @ cost @ b, b.T @ cost @ a)
        updated = a.T @ cost @ a - a.T @ cost @ b @ gain + q
        updated = (updated + updated.T) / 2
        if np.linalg.norm(updated - cost, ord=np.inf) < 1e-10:
            break
        cost = updated
    else:
        raise AssertionError("Discrete Riccati iteration did not converge")
    gain = np.linalg.solve(r + b.T @ updated @ b, b.T @ updated @ a)
    residual = a.T @ updated @ a - a.T @ updated @ b @ gain + q - updated
    assert np.linalg.norm(residual, ord=np.inf) / np.linalg.norm(updated, ord=np.inf) < 1e-10
    return np.diag(parameters["feedback_gain_scale"]) @ gain, iteration + 1


def main():
    metadata = json.loads(METADATA_PATH.read_text(encoding="utf-8"))
    parameters = metadata["parameters"]
    coefficients = np.loadtxt(CSV_PATH, delimiter=",")
    assert coefficients.shape == (12, 4)
    assert parameters["length_min_m"] == 0.04 and 0.14 <= parameters["length_max_m"] <= EXTENDED_MAX_M
    original_c = C_PATH.read_text(encoding="utf-8")
    source_coefficients = []
    for index in range(12):
        expression = re.search(r"K\[" + str(index) + r"\]\s*=\s*([^;]+);", original_c)
        assert expression, index
        literals = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?f", expression[1])
        assert len(literals) == 4, (index, literals)
        source_coefficients.append([float(value[:-1]) for value in literals])
    source_coefficients = np.asarray(source_coefficients)
    assert np.max(np.abs(source_coefficients - coefficients)) < 1e-11
    float_coefficients = source_coefficients.astype(np.float32)
    config = CONFIG_PATH.read_text(encoding="utf-8")
    output_scale = np.diag([source_float(config, "BALANCE_DEFAULT_WHEEL_SCALE"),
                            source_float(config, "BALANCE_DEFAULT_LEG_SCALE")])

    def gain(length, single=False):
        if single:
            value = np.float32(length)
            c = float_coefficients
            values = ((c[:, 0] * value + c[:, 1]) * value + c[:, 2]) * value + c[:, 3]
        else:
            values = coefficients @ [length ** 3, length ** 2, length, 1]
        return values.astype(float).reshape((2, 6), order="F")

    def pole_grid(lower, upper):
        radii, nominal, single = [], [], []
        lengths = np.linspace(lower, upper, 111)
        for length in lengths:
            a, b = discrete_plant(parameters, length)
            k = gain(length)
            nominal.append(float(max(abs(np.linalg.eigvals(a - b @ k)))))
            radii.append(float(max(abs(np.linalg.eigvals(a - b @ output_scale @ k)))))
            single.append(float(max(abs(np.linalg.eigvals(a - b @ output_scale @ gain(length, True))))))
        assert max(nominal) < 1 and max(radii) < 1 and max(single) < 1
        return {"points": len(lengths), "min_m": lower, "max_m": upper,
                "max_nominal_pole_radius": max(nominal),
                "max_firmware_scaled_pole_radius": max(radii),
                "max_float32_horner_scaled_pole_radius": max(single),
                "worst_scaled_length_m": float(lengths[int(np.argmax(radii))])}

    original = pole_grid(parameters["length_min_m"], parameters["length_max_m"])
    reproduction_error = abs(original["max_nominal_pole_radius"] - metadata["max_linear_pole_radius"])
    assert reproduction_error < 1e-10, "Original MATLAB pole result was not reproduced"
    extension = pole_grid(parameters["length_max_m"], EXTENDED_MAX_M)
    errors = []
    for length in np.unique(np.linspace(parameters["length_max_m"], EXTENDED_MAX_M, 21)):
        a, b = discrete_plant(parameters, length)
        reference, iterations = independent_dlqr(a, b, parameters)
        k = gain(length, True)
        errors.append({"length_m": float(length),
                       "gain_frobenius_relative_error": float(np.linalg.norm(k - reference) / np.linalg.norm(reference)),
                       "row_relative_errors": [float(np.linalg.norm(k[row] - reference[row]) /
                                                     np.linalg.norm(reference[row])) for row in range(2)],
                       "max_absolute_coefficient_error": float(np.max(np.abs(k - reference))),
                       "riccati_iterations": iterations})
    # Explicit screening tolerance for this small extrapolation, not a claim
    # that relative gain error alone certifies closed-loop hardware stability.
    maximum_error = max(item["gain_frobenius_relative_error"] for item in errors)
    assert maximum_error < 0.005, "Fixed-K extrapolation exceeds the declared 0.5% screening tolerance"
    theoretical_max = math.sqrt((parameters["upper_link_m"] + parameters["lower_link_m"]) ** 2 -
                                (parameters["hip_gap_m"] / 2) ** 2)
    assert EXTENDED_MAX_M < theoretical_max
    header = HEADER_PATH.read_text(encoding="utf-8")
    header_max = source_float(header, "LQR_LEG_MAX_M")
    assert header_max == EXTENDED_MAX_M, "Production operating bound differs from the validated 0.15 m limit"
    report = {
        "result": "PASS",
        "scope": "Independent read-only nominal model and float32 check of current production K through 0.15 m",
        "original_fit_sample_domain_m": [parameters["length_min_m"], parameters["length_max_m"]],
        "validated_operating_domain_m": [parameters["length_min_m"], EXTENDED_MAX_M],
        "header_operating_max_m_at_validation": header_max,
        "model_total_mass_kg": parameters["total_mass_kg"],
        "firmware_output_scales": np.diag(output_scale).tolist(),
        "feedback_gain_scale": parameters["feedback_gain_scale"],
        "original_domain": original, "extended_domain": extension,
        "original_matlab_radius_reproduction_error": reproduction_error,
        "gain_reference_screening_tolerance": 0.005,
        "max_gain_reference_relative_error": maximum_error,
        "independent_dlqr_comparison": errors,
        "theoretical_geometry_max_m": theoretical_max,
        "extension_to_full_stretch_margin_m": theoretical_max - EXTENDED_MAX_M,
        "unchanged_input_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in (C_PATH, CSV_PATH, METADATA_PATH)},
        "limitations": [
            "Uses the 1.47 kg centered nominal plant; non-body mass remains the prior estimated 0.35 kg.",
            f"Fit samples span 0.04..{parameters['length_max_m']:.2f}m; no extrapolation is used when the fit includes0.15m.",
            "Float32 Horner evaluation is screened here and host kernel boundary tests run separately; no target-side bitwise ARM fused-rounding equivalence is claimed.",
            "No model of six motor Iq slew/current-loop dynamics, actuator saturation, slip, body COM shift or loaded ground balance.",
            "Geometric reach is ideal; manufacturing tolerances, linkage clearance and near-extension conditioning remain physical constraints.",
        ],
        "hardware_access": False,
    }
    REPORT.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"report": str(REPORT), "result": report["result"],
                      "extension": extension, "maximum_gain_relative_error": maximum_error,
                      "geometry_max_m": theoretical_max}, indent=2))


if __name__ == "__main__":
    main()
