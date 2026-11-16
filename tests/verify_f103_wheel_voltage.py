"""Execute the generated wheel setter and check voltage profiles against H723."""
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "scripts"))
from f103_wheel_voltage_patch import configure_wheel_voltage

original = root.parents[1] / "stm32-foc/software"
with tempfile.TemporaryDirectory(prefix="wheel_voltage_") as directory:
    tmp = Path(directory)
    for relative in ("USER/motor_config.h", "USER/BLDCMotor.c", "Src/main.c"):
        dest = tmp / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original / relative, dest)
    limit, supply = configure_wheel_voltage(tmp, root / "App/Tasks/Inc/wheel_leg_config.h")
    assert limit == 6 and supply == 12
    main = (tmp / "Src/main.c").read_text(encoding="utf-8")
    assert "#define VOLT_SUPPLY 12.0f" in main
    motor = (tmp / "USER/BLDCMotor.c").read_text(encoding="utf-8")
    setter = re.search(r"void setTargetVotage\(float new_target\)\s*\{.*?\n\}", motor, re.S)
    assert setter
    profile = (tmp / "USER/motor_config.h").read_text(encoding="utf-8")
    assert "#define MOTOR_MAX_VOLT    6.9f" in profile
    harness = r'''
#include "motor_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define _constrain(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
static struct {float q,d;} voltage;
'''+setter.group()+r'''
int main(void) {
 assert(MOTOR_TYPE==MOTOR_WHEEL && MOTOR_POLE_PAIRS==7 && MOTOR_MAX_VOLT==6.0f);
 const float input[]={-INFINITY,-32.768f,-6.001f,-6,-5,0,5,6,6.001f,32.767f,INFINITY,NAN};
 const float expect[]={-6,-6,-6,-6,-5,0,5,6,6,6,6,0};
 for(unsigned i=0;i<sizeof(input)/sizeof(input[0]);i++) {
  setTargetVotage(input[i]);assert(voltage.q==expect[i]);
 }
 puts("PASS: F103 wheel 6 V signed clamp, NaN stop, 12 V model and unchanged 6.9 V joint profile");
}
'''
    source = tmp / "voltage_test.c"
    source.write_text(harness, encoding="utf-8")
    executable = tmp / "voltage_test.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-I", str(tmp / "USER"),
                    str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    # Only generated wheel builds change; shared F103 sources remain untouched.
    assert "#define MOTOR_MAX_VOLT    4.0f" in (original / "USER/motor_config.h").read_text(encoding="utf-8-sig")
