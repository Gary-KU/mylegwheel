"""Execute the generated rate estimator; compare its delay with the legacy path."""
from pathlib import Path
import ctypes as C
import math
import re
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "scripts"))
from f103_feedback_patch import configure_feedback_speed

original = (root.parents[1] / "stm32-foc/software/Src/main.c").read_text(encoding="utf-8-sig")
pattern = r"void Motor_SpeedCalcProcess\(\)\s*\{.*?\n\}"
legacy = re.search(pattern, original, re.S).group()
# GCC requires a compile-time bound for the upstream static array.
legacy = legacy.replace("const uint8_t speedLpfLen = 5;", "enum { speedLpfLen = 5 };")
with tempfile.TemporaryDirectory(prefix="f103_rate_") as directory:
    tmp = Path(directory)
    (tmp / "Src").mkdir()
    (tmp / "Src/main.c").write_text(original, encoding="utf-8")
    configure_feedback_speed(tmp)
    patched = (tmp / "Src/main.c").read_text(encoding="utf-8")
    current = re.search(pattern, patched, re.S).group()
    assert "float angle = shaft_angle" in current and "filteredAngle" not in current
    libraries = []
    for name, function in (("current", current), ("legacy", legacy)):
        source = r'''
#include <stdint.h>
#include <math.h>
#define _2PI 6.28318530718f
static float shaft_angle, filteredAngle, speed;
static uint32_t now;
uint32_t HAL_GetTick(void) { return now; }
''' + function + r'''
float update(float angle, uint32_t tick) {
    shaft_angle = filteredAngle = angle; now = tick;
    Motor_SpeedCalcProcess(); return speed;
}
'''
        path = tmp / (name + ".c")
        path.write_text(source)
        dll_path = tmp / (name + ".dll")
        subprocess.run(["gcc", "-shared", "-std=c99", "-O2", "-Wall", "-Wextra",
                        "-Werror", str(path), "-lm", "-o", str(dll_path)], check=True)
        dll = C.CDLL(str(dll_path))
        dll.update.argtypes = [C.c_float, C.c_uint32]
        dll.update.restype = C.c_float
        libraries.append(dll)
    current, legacy = libraries
    current.update(math.nan, 0)
    # A stationary nonzero encoder angle must never create a startup speed burst.
    for t in range(100):
        assert current.update(4.0, t) == 0
    for sign in (1, -1):
        current.update(math.nan, 0)
        for t in range(200):
            rpm = current.update(4.0 + sign * .002 * t, t)
            if t >= 5:
                assert abs(rpm - sign * 120 / (2 * math.pi)) < .001
    # The continuous shaft coordinate may cross 2*pi; the tick may wrap too.
    current.update(math.nan, 0)
    for t in range(100):
        rpm = current.update(6.25 + .001 * t, (0xFFFFFFD0 + t) & 0xFFFFFFFF)
        if t >= 5:
            assert abs(rpm - 60 / (2 * math.pi)) < .001
    assert current.update(7.0, 1000) == 0  # Lost timing history: re-prime.
    for t in range(1001, 1006):
        assert current.update(7.0, t) == 0
    for value in (math.nan, math.inf, -math.inf):
        assert current.update(value, 1100) == 0
    current.update(math.nan, 0)
    q, r, covariance, filtered = .0005, .1, 0.0, 0.0
    frequency = 5.0
    sums = [0j, 0j]
    for tick in range(4000):
        time = tick * .001
        angle = 1.0 + .1 * math.sin(2 * math.pi * frequency * time)
        old_speed = legacy.update(filtered, tick)  # SysTick filters after speed/CAN.
        new_speed = current.update(angle, tick)
        mid = covariance + q
        gain = mid / (mid + r)
        filtered += gain * (angle - filtered)
        covariance = (1 - gain) * mid
        if tick >= 2000:
            phase = complex(math.cos(2 * math.pi * frequency * time),
                            -math.sin(2 * math.pi * frequency * time))
            sums[0] += old_speed * phase
            sums[1] += new_speed * phase
    delays = [-math.atan2(x.imag, x.real) / (2 * math.pi * frequency) * 1000 for x in sums]
    assert 15 < delays[0] < 20 and 2.4 < delays[1] < 2.6
    print(f"PASS: 5 Hz estimator delay legacy={delays[0]:.3f} ms -> current={delays[1]:.3f} ms; excludes CAN/task latency")
    print("PASS: no stationary startup burst, signed RPM, continuous-angle/tick wrap, gap recovery and nonfinite reset")
    if sys.platform == "win32":
        from _ctypes import FreeLibrary
        for library in libraries:
            FreeLibrary(library._handle)
            library._handle = 0
