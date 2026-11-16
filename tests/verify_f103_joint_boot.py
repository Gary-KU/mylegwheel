"""Host-execute patched F103 boot paths with valid and invalid saved zeroes."""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "scripts"))
from f103_joint_boot_patch import configure_joint_boot

original = root.parents[1] / "stm32-foc/software"
with tempfile.TemporaryDirectory(prefix="joint_boot_") as directory:
    tmp = Path(directory)
    for relative in ("USER/BLDCMotor.c", "USER/motor_config.h", "Src/main.c"):
        dst = tmp / relative
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes((original / relative).read_bytes())
    configure_joint_boot(tmp)
    assert "#define VOLT_SUPPLY 9.0f" in (tmp / "Src/main.c").read_text(encoding="utf-8")
    assert "#define MOTOR_MAX_VOLT    6.9f" in (tmp / "USER/motor_config.h").read_text(encoding="utf-8")
    motor = (tmp / "USER/BLDCMotor.c").read_text(encoding="utf-8")
    functions = []
    for name in ("Motor_init", "Motor_initFOC"):
        match = re.search(r"void " + name + r"\(void\)\s*\{.*?\n\}", motor, re.S)
        assert match is not None
        functions.append(match.group())
    assert "Beep_PlayNotes(3," not in (tmp / "Src/main.c").read_text(encoding="utf-8")
    harness = r'''
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#define MOTOR_POLE_PAIRS 11
#define CW 1
#define UNKNOWN 0
#define _2PI 6.28318530718
static float voltage_limit=6.9f,voltage_sensor_align=6.9f;
static int pole_pairs;
static long sensor_direction;
static float zero_electric_angle,angle_prev,shaft_angle;
static jmp_buf fault_exit;
static int nv_status,nv_poles,nv_direction,invalid,driver_on,enables,reads,angle_reads,saves;
static float nv_zero;
static void gate(int enabled) {driver_on=enabled;if(enabled) {assert(!invalid);++enables;}}
#define M1_Enable gate(1)
#define M1_Disable gate(0)
static int boot_printf(const char *fmt,...) {(void)fmt;return 0;}
#define printf boot_printf
static void setPhaseVoltage(float q,float d,float a) {
 (void)a;assert(q==0 && d==0); /* Any calibration/beep waveform fails. */
}
static void HAL_Delay(unsigned ms) {
 if(invalid) {assert(ms==100 && !driver_on);longjmp(fault_exit,1);}
 assert(ms==5);
}
static int Flash_ReadMotorParam(int *p,float *z,int *d) {
 ++reads;*p=nv_poles;*z=nv_zero;*d=nv_direction;return nv_status;
}
void Flash_SaveMotorParam(int p,float z,int d) {(void)p;(void)z;(void)d;++saves;}
static float getAngle(void) {++angle_reads;return .3f;}
static float shaftAngle(void) {return .3f;}
'''
    harness += "\n".join(functions)
    harness += r'''
static void run_case(int status,int poles,int direction,float zero,int bad) {
 nv_status=status;nv_poles=poles;nv_direction=direction;nv_zero=zero;invalid=bad;
 driver_on=1;enables=reads=angle_reads=saves=0;
 Motor_init();assert(!driver_on && enables==0);
 int escaped=setjmp(fault_exit);
 if(!escaped) Motor_initFOC();
 assert(reads==1 && saves==0);
 if(bad) {assert(escaped && !driver_on && enables==0 && angle_reads==0);}
 else {
  assert(!escaped && driver_on && enables==1 && angle_reads==1);
  assert(zero_electric_angle==zero && pole_pairs==11 && sensor_direction==CW);
 }
}
int main(void) {
 run_case(0,11,CW,0,0);run_case(0,11,CW,1.25f,0);
 run_case(0,11,CW,6.28f,0);
 run_case(-1,11,CW,1.25f,1);run_case(0,7,CW,1.25f,1);
 run_case(0,11,-1,1.25f,1);run_case(0,11,CW,-.1f,1);
 run_case(0,11,CW,(float)_2PI,1);run_case(0,11,CW,NAN,1);
 run_case(0,11,CW,INFINITY,1);
 puts("PASS: joint boot has zero field/no melody, loads valid zero, rejects invalid calibration, never writes Flash");
}
'''
    source = tmp / "boot_test.c"
    source.write_text(harness, encoding="utf-8")
    exe = tmp / "boot_test.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    str(source), "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
