"""Test the real Mahony C implementation against analytic rotations.

Only the ARM sqrt intrinsic is mapped to host sqrtf; fusion is not mocked.
The host build disables strict aliasing for the legacy inverse-square-root
bit cast, as used by initialization. --arm-check also compiles the unchanged
production source with the project's Cortex-M7/FPU flags. Neither host timing
nor successful ARM compilation measures the H723's 1kHz execution budget.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HARNESS = r'''
#include "mahony_ahrs.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern float q0,q1,q2,q3,twoKi,integralFBx,integralFBy,integralFBz,invSampleFreq;
static const double pi=3.14159265358979323846;
static unsigned checks;
static void check(double actual,double expected,double tolerance,const char *label) {
    double error=fabs(remainder(actual-expected,360.0));
    ++checks;
    if(!isfinite(actual) || error>tolerance) {
        fprintf(stderr,"FAIL %s: got %.10gdeg, expected %.10gdeg, error %.9g > %.9g\n",
                label,actual,expected,error,tolerance);
        exit(1);
    }
}
static void quaternion(double roll,double pitch,double yaw) {
    double r=roll*pi/360.0,p=pitch*pi/360.0,y=yaw*pi/360.0;
    q0=(float)(cos(r)*cos(p)*cos(y)+sin(r)*sin(p)*sin(y));
    q1=(float)(sin(r)*cos(p)*cos(y)-cos(r)*sin(p)*sin(y));
    q2=(float)(cos(r)*sin(p)*cos(y)+sin(r)*cos(p)*sin(y));
    q3=(float)(cos(r)*cos(p)*sin(y)-sin(r)*sin(p)*cos(y));
}
static void known_rotation(double roll,double pitch,double yaw,double tolerance) {
    quaternion(roll,pitch,yaw);
    const float before[4]={q0,q1,q2,q3};
    Mahony_computeAngles();
    check(getRoll(),roll,tolerance,"known quaternion roll");
    check(getPitch(),pitch,tolerance,"known quaternion pitch");
    check(getYaw(),yaw,tolerance,"known quaternion yaw");
    const float after[4]={q0,q1,q2,q3};
    if(memcmp(before,after,sizeof(before))) {
        fputs("FAIL: Euler conversion mutated quaternion\n",stderr);exit(1);
    }
}
static void test_known_rotations(void) {
    const double small[]={-6,-5,-3,-1,-.5,-.01,-.000001,0,.000001,.01,.5,1,3,5,6};
    for(unsigned i=0;i<sizeof(small)/sizeof(small[0]);++i) {
        double angle=small[i];
        known_rotation(angle,0,0,0.0001);
        known_rotation(0,0,angle,0.0001);
        /* A relative bound detects the old 2.76% scale error arbitrarily
         * close to zero, rather than hiding it inside a fixed tolerance. */
        quaternion(angle,0,0);Mahony_computeAngles();
        check(getRoll(),angle,1e-10+fabs(angle)*1e-5,"small roll scale");
        quaternion(0,0,angle);Mahony_computeAngles();
        check(getYaw(),angle,1e-10+fabs(angle)*1e-5,"small yaw scale");
    }
    const double quadrant[]={-180,-179.999,-135,-90,-45,0,45,90,135,179.999,180};
    for(unsigned i=0;i<sizeof(quadrant)/sizeof(quadrant[0]);++i) {
        known_rotation(quadrant[i],0,0,0.0001);
        known_rotation(0,0,quadrant[i],0.0001);
    }
    for(int r=-170;r<=170;r+=34)
        for(int p=-60;p<=60;p+=30)
            for(int y=-170;y<=170;y+=34)
                known_rotation(r,p,y,0.0002);
    known_rotation(0,89,0,0.002);known_rotation(0,-89,0,0.002);
    puts("PASS: analytic unit quaternions, small-angle scale/near-zero, all atan2 quadrants and combined Euler rotations");
}
static void test_axis_integration(void) {
    const float rates[]={-.01f,-.1f,-1.0f,.01f,.1f,1.0f};
    for(unsigned axis=0;axis<3;++axis)
        for(unsigned rate=0;rate<sizeof(rates)/sizeof(rates[0]);++rate) {
            Mahony_Init(1000);
            if(twoKi!=0 || integralFBx!=0 || integralFBy!=0 || integralFBz!=0 ||
               fabsf(invSampleFreq-.001f)>1e-9f) exit(1);
            for(unsigned sample=0;sample<1500;++sample) {
                float gyro[3]={0};gyro[axis]=rates[rate];
                Mahony_update(gyro[0],gyro[1],gyro[2],0,0,0,0,0,0);
            }
            Mahony_computeAngles();
            const double expected=(double)rates[rate]*1.5*180.0/pi;
            check(getRoll(),axis==0?expected:0,.003,"gyro-integrated roll");
            check(getPitch(),axis==1?expected:0,.003,"gyro-integrated pitch");
            check(getYaw(),axis==2?expected:0,.003,"gyro-integrated yaw");
        }
    for(unsigned axis=0;axis<2;++axis) for(int direction=-1;direction<=1;direction+=2) {
        Mahony_Init(1000);
        for(unsigned sample=0;sample<2500;++sample)
            Mahony_update(axis==0?direction*3.0f:0,0,axis==1?direction*3.0f:0,0,0,0,0,0,0);
        Mahony_computeAngles();
        check(axis==0?getRoll():getYaw(),direction*7.5*180.0/pi,.005,"integrated full-turn wrap");
    }
    puts("PASS: real 1kHz Mahony gyro integration, both signs/all axes and full-turn wrapping; original gains retained");
}
static void test_accelerometer_fusion(void) {
    for(int direction=-1;direction<=1;direction+=2) {
        const double angle=direction*5.0*pi/180.0;
        Mahony_Init(1000);
        MahonyAHRSinit(0,(float)(9.8*sin(angle)),(float)(9.8*cos(angle)),0,0,0);
        for(unsigned sample=0;sample<20000;++sample)
            Mahony_update(0,0,0,0,(float)(9.8*sin(angle)),(float)(9.8*cos(angle)),0,0,0);
        Mahony_computeAngles();
        check(getRoll(),direction*5.0,.001,"gravity-aligned roll");
        check(getPitch(),0,.001,"gravity-aligned pitch");
        check(getYaw(),0,.001,"gravity-aligned yaw");
    }
    puts("PASS: real accelerometer initialization and 20s stationary fusion retain known signed5deg roll");
}
int main(void) {
    test_known_rotations();test_axis_integration();test_accelerometer_fusion();
    printf("PASS: %u analytic Mahony angle checks; this is numerical validation, not physical parking or ARM timing proof\n",checks);
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'Algorithms/Attitude/Src/mahony_ahrs.c')
    parser.add_argument('--arm-check', action='store_true')
    args = parser.parse_args()
    include = ROOT/'Algorithms/Attitude/Inc'
    with tempfile.TemporaryDirectory(prefix='mahony_angles_') as directory:
        work = Path(directory)
        harness = work/'test.c'
        harness.write_text(HARNESS, encoding='ascii')
        exe = work/'test.exe'
        subprocess.run(['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-fno-strict-aliasing', '-D__sqrtf=sqrtf', '-I', str(include),
                        str(harness), str(args.source), '-lm', '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        if args.arm_check:
            armclang = Path('D:/app/Keil_v5/ARM/ARMCLANG/bin/armclang.exe')
            subprocess.run([str(armclang), '--target=arm-arm-none-eabi', '-mcpu=cortex-m7',
                            '-mfpu=fpv5-sp-d16', '-mfloat-abi=hard', '-std=c99', '-O2',
                            '-ffunction-sections', '-fdata-sections', '-fshort-enums',
                            '-fshort-wchar', '-D__MICROLIB', '-gdwarf-3', '-I', str(include),
                            '-c', str(args.source), '-o', str(work/'mahony_ahrs.o')], check=True)
            print('PASS: production Cortex-M7/FPv5-SP hard-float ARM Compiler6 compilation; target timing remains unmeasured')


if __name__ == '__main__':
    main()
