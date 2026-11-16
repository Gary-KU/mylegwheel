"""Host C verification against the supplied scalar MATLAB expressions.

python tests/verify_wheel_leg_math.py --source D:/MATLAB/Robot/WHEEL_LEG/MyProc
Requires gcc on PATH. Does not require MATLAB or third-party Python packages.
"""
import argparse
import ctypes as C
import math
from pathlib import Path
import random
import re
import subprocess
import tempfile


def matlab_function(path):
    text = path.read_text(encoding="utf-8")
    args = re.search(r"function \w+ = \w+\((.*?)\)", text)[1].split(",")
    body = re.sub(r"%[^\n]*", "", text).split("\n", 1)[1]
    def evaluate(*values):
        env = dict(zip(args, values))
        env.update(sin=math.sin, cos=math.cos, sqrt=math.sqrt,
                   atan=math.atan, atan2=math.atan2, real=lambda x: x,
                   imag=lambda x: 0.0, conj=lambda x: x)
        for line in body.splitlines():
            if "=" not in line:
                continue
            lhs, rhs = line.strip().rstrip(";").split("=", 1)
            rhs = rhs.strip().replace(".*", "*").replace("./", "/").replace(".^", "**")
            if rhs.startswith("reshape"):
                return [env[f"mt{i}"][0] for i in range(1, 13)]
            if rhs.startswith("["):
                result = [eval(x, {"__builtins__": {}}, env)
                          for x in rhs[1:-1].split(";")]
            else:
                result = eval(rhs, {"__builtins__": {}}, env)
            env[lhs.strip()] = result
        return result
    return evaluate


def lower_pos(a, b):
    bx, by = .05*math.cos(a), .05*math.sin(a)
    dx, dy = .06+.05*math.cos(b), .05*math.sin(b)
    vx, vy = dx-bx, dy-by
    d = math.hypot(vx, vy)
    h = math.sqrt(.105**2-d*d/4)
    points = [((bx+dx)/2-s*h*vy/d-.03, (by+dy)/2+s*h*vx/d)
              for s in (1, -1)]
    x, y = min(points, key=lambda p: p[1])
    return math.hypot(x, y), math.atan2(y, x)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--lqr-source", type=Path,
                        help="Directory containing the updated lqr_k.m reference")
    opt = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    control = root / "Control/WheelLeg"
    support = root / "tests/support/wheel_leg"
    refs = {name: matlab_function(opt.source / (name+".m"))
            for name in ("leg_pos", "leg_spd", "leg_conv", "lqr_k")}
    lqr_reference = opt.lqr_source
    if lqr_reference is None and (root / "models/lqr/lqr_k.m").is_file():
        lqr_reference = root / "models/lqr"
    if lqr_reference is not None:
        refs["lqr_k"] = matlab_function(lqr_reference / "lqr_k.m")
    # Loading DLLs on Windows prevents deletion until process exit.
    output = Path(tempfile.gettempdir()) / "mylegwheel_math_verify.dll"
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
                    "-shared", "-I", str(control / "Inc"),
                    "-I", str(support / "Inc"),
                    *[str((support if name in ("leg_spd", "leg_conv") else control) /
                          "Src" / (name+".c")) for name in refs],
                    "-o", str(output)], check=True)
    dll = C.CDLL(str(output))
    fp = C.POINTER(C.c_float)
    for name, count in (("leg_pos", 2), ("leg_spd", 4), ("leg_conv", 4),
                        ("leg_conv_3d", 4), ("lqr_k", 1)):
        getattr(dll, name).argtypes = [C.c_float]*count+[fp]
    dll.lqr_control.argtypes = [C.c_float, fp, fp, fp]
    dll.lqr_control.restype = C.c_int
    dll.leg_conv_3d.restype = C.c_int
    class Model(C.Structure):
        _fields_ = [("length", C.c_float), ("angle", C.c_float), ("j", C.c_float*4)]
    dll.leg_model.argtypes = [C.c_float, C.c_float, C.c_int, C.POINTER(Model)]
    dll.leg_model.restype = C.c_int
    maxima = {}
    def close(label, got, expected, atol=2e-5, rtol=3e-4):
        for g, e in zip(got, expected):
            assert math.isfinite(g) and abs(g-e) <= atol+rtol*abs(e), (label,g,e)
            maxima[label] = max(maxima.get(label, 0), abs(g-e))
    def call(name, args, n=2):
        out = (C.c_float*n)()
        getattr(dll,name)(*args,out)
        return list(out)
    rng = random.Random(20261001)
    # Wide angle coverage, avoiding real toggle/coincident-point singularities.
    for _ in range(2000):
        a, b = [C.c_float(rng.uniform(-math.pi,math.pi)).value for _ in range(2)]
        d = math.hypot(.06+.05*math.cos(b)-.05*math.cos(a),
                       .05*math.sin(b)-.05*math.sin(a))
        if d < .008:
            continue
        da, db, F, Tp = .7, -.4, 3.4, .12
        for name, args in (("leg_pos", (a,b)), ("leg_spd", (da,db,a,b)),
                            ("leg_conv", (F,Tp,a,b))):
            close(name, call(name,args), refs[name](*args))
        # Finite differences independently check lower-branch derivatives and wiring.
        h = 1e-6
        j = []
        for aa, bb in ((h,0), (0,h)):
            plus, minus = lower_pos(a+aa,b+bb), lower_pos(a-aa,b-bb)
            j.append(((plus[0]-minus[0])/(2*h),
                      math.remainder(plus[1]-minus[1],2*math.pi)/(2*h)))
        expected = [F*j[i][0]+Tp*j[i][1] for i in (1,0)]
        close("3d_vmc", call("leg_conv_3d",(F,Tp,a,b)), expected)
        pos = call("leg_pos",(a,b))
        spd = call("leg_spd",(da,db,a,b))
        torque = call("leg_conv",(F,Tp,a,b))
        close("virtual_power", [torque[0]*da+torque[1]*db], [F*spd[0]+Tp*spd[1]])
    for i in range(101):
        length = C.c_float(.04+.001*i).value
        close("lqr_k",call("lqr_k",(length,),12),refs["lqr_k"](length),2e-7,2e-6)
        for col in range(6):
            state = (C.c_float*6)(*([0.0]*6))
            reference = (C.c_float*6)(*([.03]*6))
            state[col] = .2
            u = (C.c_float*2)()
            assert dll.lqr_control(length,state,reference,u) == 1
            k = refs["lqr_k"](length)
            close("lqr_control",u,[-sum(k[r+2*j]*(state[j]-reference[j]) for j in range(6)) for r in range(2)])
    # Centered legs: older expanded atan2 derivatives divided by horizontal x=0.
    a, b = math.pi-0.8, 0.8
    for branch in (0,1):
        m = Model()
        assert dll.leg_model(a,b,branch,C.byref(m)) == 1
        assert all(math.isfinite(v) for v in m.j)
    # Coincident B,D; NaN/Inf; out-of-fit LQR inputs.
    for a,b in ((math.acos(.6), math.acos(-.6)), (math.nan,0), (0,math.inf)):
        m = Model()
        assert dll.leg_model(a,b,0,C.byref(m)) == 0
        assert m.length == 0 and all(v == 0 for v in m.j)
        u = (C.c_float*2)(1,1)
        assert dll.leg_conv_3d(3,.1,a,b,u) == 0 and list(u) == [0,0]
        assert all(math.isnan(v) for v in call("leg_pos",(a,b)))
    for length in (.039, .141, math.nan, math.inf):
        u = (C.c_float*2)(1,1)
        assert dll.lqr_control(length,(C.c_float*6)(),(C.c_float*6)(),u) == 0
        assert list(u) == [0,0]
    print("PASS: 2000 angle samples, 101 leg lengths, 606 LQR state/reference cases,")
    print("      3D finite differences, virtual power, centered and invalid inputs.")
    print("Maximum absolute errors:", maxima)


if __name__ == "__main__":
    main()

