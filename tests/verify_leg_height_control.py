"""Host tests of the production bounded height-integral helper.

The 54-case vertical model varies axial load independently of effective mass,
friction and initial length. It checks height recovery, not coupled wheel/body
stability, actual contact force, or free-standing hardware safety. Model cases
start after the existing support feedforward ramp has reached 9 N.
"""
from pathlib import Path
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
harness = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "leg_height_control.h"

static float clip(float value,float lower,float upper) {
    return fmaxf(lower,fminf(upper,value));
}
static float deadband(float value,float width) {
    return copysignf(fmaxf(0,fabsf(value)-width),value);
}
static float candidate(float previous,float error,unsigned ready) {
    return leg_height_integral_candidate(previous,error,80,.005f,4.5f,3,ready);
}
static void scalar_tests(void) {
    assert(fabsf(candidate(0,-.04f,1)+.015f)<1e-7f);
    assert(candidate(4.499f,.04f,1)==4.5f);
    assert(candidate(-4.499f,-.04f,1)==-4.5f);
    assert(candidate(-.8f,.04f,0)==-.8f);
    assert(leg_height_integral_candidate(-.8f,.04f,0,.005f,4.5f,3,0)==0);
    assert(isnan(leg_height_integral_candidate(0,0,-1,.005f,4.5f,3,1)));
    assert(isnan(leg_height_integral_candidate(0,0,80,0,4.5f,3,1)));
    assert(isnan(leg_height_integral_candidate(NAN,0,80,.005f,4.5f,3,1)));
    assert(isnan(leg_height_integral_accept(0,0,9,15,0)));

    /* Upper and lower saturation reject growth and permit unwinding even
     * while the requested force remains outside the available interval. */
    assert(leg_height_integral_accept(2,2.015f,14,0,15)==2);
    assert(leg_height_integral_accept(2,1.985f,14,0,15)==1.985f);
    assert(leg_height_integral_accept(-2,-2.015f,1,0,15)==-2);
    assert(leg_height_integral_accept(-2,-1.985f,1,0,15)==-1.985f);
    assert(leg_height_integral_accept(0,.015f,14.99f,0,15)==0);
    assert(leg_height_integral_accept(0,-.015f,.01f,0,15)==0);
    float integral=0;
    for(unsigned n=0;n<20000;++n)
        integral=leg_height_integral_accept(integral,candidate(integral,.04f,1),14,0,15);
    assert(integral>.97f && integral<=1.0f);
    float previous=integral;
    integral=leg_height_integral_accept(integral,candidate(integral,-.04f,1),14,0,15);
    assert(integral<previous);
    for(unsigned n=0;n<20000;++n)
        integral=leg_height_integral_accept(integral,candidate(integral,-.04f,1),1,0,15);
    assert(integral>=-1.0f && integral<-.97f);
    previous=integral;
    integral=leg_height_integral_accept(integral,candidate(integral,.04f,1),1,0,15);
    assert(integral>previous);
    puts("PASS: integral units, 3N/s slew, +/-4.5N state, ramp freeze, explicit Ki=0, axial saturation/unwind");
}

typedef struct {
    float final_error,tail_error,tail_speed,max_speed,min_force,max_force;
    float min_length,max_length,overshoot,bias;
} Result;

static Result simulate(float load,float mass,float friction,float initial,
                       float force_cap,unsigned enabled) {
    const float plant_dt=.001f,control_dt=.005f;
    float x=initial,v=0,integral=0,reference=0,force=9,applied=9;
    Result result={0,0,0,0,100,-100,initial,initial,0,0};
    for(unsigned step=0;step<2400;++step) {
        float target=clip(5*deadband(.07f-x,.001f),-.04f,.04f);
        reference+=control_dt/(.02f+control_dt)*(target-reference);
        float error=reference-deadband(v,.003f);
        float base=9+(enabled ? 30 : 10)*error;
        float next=enabled ? candidate(integral,error,1) : 0;
        integral=leg_height_integral_accept(integral,next,base,0,force_cap);
        assert(isfinite(integral) && fabsf(integral)<=4.500001f);
        force=clip(base+integral,0,force_cap);
        result.min_force=fminf(result.min_force,force);
        result.max_force=fmaxf(result.max_force,force);
        for(unsigned substep=0;substep<5;++substep) {
            /* First-order 20ms current/force lag; viscous drag 1N.s/m.
             * Coulomb stiction supports any force inside +/-friction at
             * rest. Hard stops exist only to bound infeasible-load cases. */
            applied+=plant_dt/(.02f+plant_dt)*(force-applied);
            float net=applied-load-v;
            if(fabsf(v)<1e-5f && fabsf(net)<=friction) v=0;
            else {
                float acceleration=(net-copysignf(friction,
                    fabsf(v)>1e-5f ? v : net))/mass;
                float next_v=v+acceleration*plant_dt;
                if(v*next_v<0 && fabsf(net)<=friction) next_v=0;
                v=next_v;
            }
            x+=plant_dt*v;
            if(x<.04f) {x=.04f;v=fmaxf(0,v);}
            if(x>.14f) {x=.14f;v=fminf(0,v);}
            result.min_length=fminf(result.min_length,x);
            result.max_length=fmaxf(result.max_length,x);
            result.max_speed=fmaxf(result.max_speed,fabsf(v));
            if(step>=2200) {
                result.tail_error=fmaxf(result.tail_error,fabsf(x-.07f));
                result.tail_speed=fmaxf(result.tail_speed,fabsf(v));
            }
        }
    }
    result.final_error=x-.07f;
    result.overshoot=fmaxf(0,.07f-result.min_length);
    result.bias=integral;
    return result;
}

int main(void) {
    scalar_tests();
    const float loads[3]={6,7.85f,11},masses[3]={.6f,.8f,1.1f};
    const float friction[2]={0,.6f},initial[3]={.09f,.105f,.11f};
    unsigned count=0;float tail=0,speed=0,overshoot=0,minimum=100,maximum=0;
    for(unsigned l=0;l<3;++l)for(unsigned m=0;m<3;++m)
    for(unsigned f=0;f<2;++f)for(unsigned x=0;x<3;++x) {
        Result r=simulate(loads[l],masses[m],friction[f],initial[x],15,1);
        if(r.tail_error>=.002f || r.tail_speed>=.008f ||
           r.min_length<=.04001f || r.max_length>=.13999f)
            fprintf(stderr,"case load=%g mass=%g friction=%g initial=%g tail=%g speed=%g bounds=%g,%g\n",
                loads[l],masses[m],friction[f],initial[x],r.tail_error,r.tail_speed,r.min_length,r.max_length);
        assert(r.tail_error<.002f && r.tail_speed<.008f);
        assert(r.min_length>.04001f && r.max_length<.13999f);
        assert(r.min_force>=0 && r.max_force<=15);
        tail=fmaxf(tail,r.tail_error);speed=fmaxf(speed,r.max_speed);
        overshoot=fmaxf(overshoot,r.overshoot);
        minimum=fminf(minimum,r.min_force);maximum=fmaxf(maximum,r.max_force);
        ++count;
    }
    assert(count==54);
    printf("PASS: %u vertical-model cases; tail error <=%.3fmm; peak target overshoot %.3fmm; peak speed %.4fm/s; force %.3f..%.3fN\n",
        count,tail*1000,overshoot*1000,speed,minimum,maximum);
    Result baseline=simulate(7.85f,.8f,.3f,.105f,15,0);
    Result nominal=simulate(7.85f,.8f,.3f,.105f,15,1);
    assert(baseline.tail_error>.02f && nominal.tail_error<.002f);
    printf("PASS: same nominal plant: Ki=0/Kp10 tail %.2fmm, bounded PI tail %.3fmm, learned bias %.3fN\n",
        baseline.tail_error*1000,nominal.tail_error*1000,nominal.bias);

    Result reduced=simulate(7.85f,.8f,.3f,.105f,7,1);
    assert(reduced.final_error<-.025f && reduced.max_force<=7 && fabsf(reduced.bias)<4.5f);
    Result excessive=simulate(16,.8f,.3f,.105f,15,1);
    assert(excessive.final_error<-.025f && excessive.max_force<=15 && fabsf(excessive.bias)<=4.5f);
    Result too_light=simulate(4,.8f,.3f,.105f,15,1);
    assert(too_light.final_error>.003f && too_light.bias==-4.5f);
    puts("PASS: 7N headroom, 16N load and 4N load expose infeasible recovery without exceeding force/integral bounds");
    puts("Vertical-channel model only: assumes established support, independent axial load, no wheel/body coupling or measured contact-force identification.");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="leg_height_control_") as directory:
    work = Path(directory)
    source = work / "test.c"
    source.write_text(harness, encoding="utf-8")
    exe = work / "test.exe"
    subprocess.run([
        "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "App/Tasks/Inc"), str(source), "-lm", "-o", str(exe)
    ], check=True)
    subprocess.run([str(exe)], check=True)
