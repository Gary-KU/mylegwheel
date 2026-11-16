"""Copy verified MATLAB polynomial coefficients into the firmware evaluator.

Run models/lqr/generate_lqr_reference.m first. Keeps lqr_control() and its
state/direction convention unchanged. Does not flash or run the robot.
"""
import csv
import json
import math
import re
from pathlib import Path
import shutil
from datetime import datetime


def main():
    root = Path(__file__).resolve().parents[1]
    model = root / "models/lqr"
    metadata = json.loads((model / "lqr_model_metadata.json").read_text(encoding="utf-8"))
    assert metadata["max_linear_pole_radius"] < 1
    params = metadata["parameters"]
    assert params["total_mass_kg"] > params["non_body_mass_kg"] > 0
    assert abs(params["body_mass_kg"] + params["non_body_mass_kg"] - params["total_mass_kg"]) < 1e-12
    with (model / "lqr_coefficients.csv").open(newline="") as source:
        coefficients = [[float(value) for value in row] for row in csv.reader(source)]
    assert len(coefficients) == 12
    assert all(len(row) == 4 and all(math.isfinite(v) for v in row) for row in coefficients)
    target = root / "Control/WheelLeg/Src/lqr_k.c"
    old = target.read_text(encoding="utf-8")
    # 仅替换首个多项式求值器；其后的共享内核、注释与公开入口全部保留。
    evaluator = re.search(r"^void\s+lqr_k\s*\([^)]*\)\s*\{.*?^\}", old, re.MULTILINE | re.DOTALL)
    if evaluator is None:
        raise ValueError(f"No LQR polynomial evaluator found in {target}")
    control = old[evaluator.end():].lstrip("\r\n")
    if re.search(r"^int\s+lqr_control(?:_with_gain)?\s*\(", control, re.MULTILINE) is None:
        raise ValueError(f"No LQR control implementation found in {target}")
    backup = root / "backups" / ("before_lqr_regeneration_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    backup.mkdir(parents=True)
    shutil.copy2(target, backup / "lqr_k.c")
    lines = [
        "/* Generated from models/lqr/generate_lqr_reference.m.",
        f" * Total {params['total_mass_kg']:.3f} kg, body {params['body_mass_kg']:.3f} kg; {params['body_mass_kg']/2:.3f} kg body per side.",
        f" * Wheel/leg split uses the prior geometry ratio, scaled to {params['non_body_mass_kg']:.3f} kg.",
        " * Estimated part masses: see models/lqr/lqr_model_metadata.json.",
        f" * Feedback scales vs nominal DLQR: wheel {params.get('feedback_gain_scale', [1, 1])[0]:.6g}, leg {params.get('feedback_gain_scale', [1, 1])[1]:.6g}.",
        " * MATLAB column-major 2x6; feedback/state conventions unchanged. */",
        '#include "lqr_k.h"',
        "#include <math.h>",
        "void lqr_k(float L0, float K[12])",
        "{",
    ]
    for index, row in enumerate(coefficients):
        c3, c2, c1, c0 = [f"{value:.17e}f" for value in row]
        lines.append(f"    K[{index}] = (({c3}*L0 + ({c2}))*L0 + ({c1}))*L0 + ({c0});")
    lines.extend(["}", "", control.rstrip(), ""])
    target.write_text("\n".join(lines), encoding="utf-8")
    physics = root / "Control/WheelLeg/Inc/lqr_model_config.h"
    constants = {"LQR_BODY_MASS_PER_SIDE_KG":params["body_mass_kg"]/2,
                 "LQR_BODY_COM_HEIGHT_M":params.get("body_com_height_m",0),
                 "LQR_BODY_PITCH_INERTIA_CG":params["body_pitch_inertia_per_side"],
                 "LQR_GRAVITY_M_S2":params["gravity_m_s2"]}
    physics.write_text("/* Generated from LQR model metadata; COM height is an estimate. */\n"
        "#ifndef LQR_MODEL_CONFIG_H\n#define LQR_MODEL_CONFIG_H\n"+
        "".join(f"#define {name} {value:.17e}f\n" for name,value in constants.items())+
        "#endif\n",encoding="utf-8")
    print(f"Updated {target}; backup {backup}")


if __name__ == "__main__":
    main()
