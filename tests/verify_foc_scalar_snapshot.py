"""对照旧整块快照，验证标量快照的数值、门禁和临界区语义。

本历史等价性测试显式关闭新增固定限流同步；生产默认3A同步及拒绝门控由
verify_foc_current.py单独覆盖。两种快照仍使用当前角色限流配置。
"""
from pathlib import Path
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
source = (root / "App/Services/Src/motor_foc.c").read_text(encoding="utf-8")


def function(name, return_type="float"):
    start = source.index(return_type + " " + name + "(")
    opening = source.index("{", start)
    depth = 0
    for end in range(opening, len(source)):
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
            if depth == 0:
                return source[start:end + 1]
    raise AssertionError("Function not closed: " + name)


# 固定优化前实现，避免新实现与自身对比；不依赖本地备份或固件产物。
reference = r'''
static float reference_limit(uint8_t id) {
    if(id<1U || id>8U || !selected(id-1U)) return 0;
    unsigned slot=id-1U;MotorFocCalibration c;MotorFocNode n;
    taskENTER_CRITICAL();c=g_motor_foc.calibration[slot];n=g_motor_foc.node[slot];taskEXIT_CRITICAL();
    if(!c.calibration_confirmed || !c.dynamic_response_confirmed ||
       !isfinite(c.minimum_iq_slew_a_per_s) || c.minimum_iq_slew_a_per_s<=0 ||
       !n.slew_verified || !n.voltage_verified || n.remote_iq_slew_a_per_s<c.minimum_iq_slew_a_per_s ||
       !isfinite(c.kt_nm_per_a) || c.kt_nm_per_a<=0 ||
       !isfinite(c.current_limit_a) || c.current_limit_a<=0 || c.current_limit_a>MOTOR_FOC_CURRENT_HARD_LIMIT_A ||
       (c.direction!=1 && c.direction!=-1) || !n.limit_verified) return 0;
    return c.kt_nm_per_a*fminf(c.current_limit_a,n.remote_iq_limit_a);
}
static float reference_current(uint8_t id,float torque) {
    if(id<1U || id>8U || !selected(id-1U) || !isfinite(torque)) return NAN;
    MotorFocCalibration c;
    taskENTER_CRITICAL();c=g_motor_foc.calibration[id-1U];taskEXIT_CRITICAL();
    if(!isfinite(c.kt_nm_per_a) || c.kt_nm_per_a<=0 || (c.direction!=1 && c.direction!=-1)) return NAN;
    return (float)c.direction*torque/c.kt_nm_per_a;
}
'''

prefix = r'''
#include "motor_foc.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
volatile MotorFocDebug g_motor_foc;
static unsigned critical, enters, exits, hook, hook_slot;
static uint32_t seed=0x8261015U;
static unsigned cases;
static int selected(unsigned slot) { return (MOTOR_FOC_NODE_MASK & (1U << slot)) != 0U; }
static void mutate(void) {
    g_motor_foc.calibration[hook_slot].kt_nm_per_a=.66f;
    g_motor_foc.calibration[hook_slot].current_limit_a=.25f;
    g_motor_foc.calibration[hook_slot].direction=-1;
    g_motor_foc.calibration[hook_slot].dynamic_response_confirmed=0U;
    g_motor_foc.node[hook_slot].remote_iq_limit_a=.125f;
    g_motor_foc.node[hook_slot].slew_verified=0U;
}
static void enter(void) {
    assert(!critical);critical=1U;++enters;
    if(hook==1U) mutate();
}
static void leave(void) {
    assert(critical==1U);
    if(hook==2U) mutate();
    critical=0U;++exits;
}
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
static uint32_t random32(void) {
    seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;return seed;
}
static float from_bits(uint32_t bits) {float f;memcpy(&f,&bits,sizeof(f));return f;}
static int same(float a,float b) {
    uint32_t aa,bb;
    if(isnan(a) && isnan(b)) return 1;
    memcpy(&aa,&a,sizeof(aa));memcpy(&bb,&b,sizeof(bb));return aa==bb;
}
static void load(const MotorFocCalibration *c,const MotorFocNode *n) {
    g_motor_foc.calibration[hook_slot]=*c;g_motor_foc.node[hook_slot]=*n;
    enters=exits=critical=0U;
}
'''

checks = r'''
static void compare(uint8_t id,float torque,const MotorFocCalibration *c,const MotorFocNode *n) {
    hook_slot=id>=1U && id<=8U ? id-1U : 0U;
    load(c,n);float expected_limit=reference_limit(id);unsigned locks=enters;
    assert(!critical && exits==locks);
    load(c,n);float actual_limit=MotorFoc_TorqueLimitNm(id);
    assert(!critical && enters==locks && exits==locks && same(actual_limit,expected_limit));
    load(c,n);float expected_current=reference_current(id,torque);locks=enters;
    assert(!critical && exits==locks);
    load(c,n);float actual_current=MotorFoc_CurrentForTorque(id,torque);
    assert(!critical && enters==locks && exits==locks && same(actual_current,expected_current));
    ++cases;
}
int main(void) {
    const float edge[]={-INFINITY,-FLT_MAX,-1.0f,-.001f,-0.0f,0.0f,FLT_MIN,
        .001f,.23f,.8f,1.0f,1.00000012f,2.0f,FLT_MAX,INFINITY,NAN};
    MotorFocCalibration c={0};MotorFocNode n={0};
    /* 穷举五项就绪门禁、正反方向、所有节点/非法 ID，并覆盖锁边界上的数据变化。 */
    for(unsigned gates=0;gates<32;++gates) for(int dir=-2;dir<=2;++dir)
    for(unsigned id=0;id<=9;++id) for(unsigned h=0;h<3;++h) {
        c=(MotorFocCalibration){.kt_nm_per_a=.23f,.current_limit_a=.8f,.direction=dir,
            .calibration_confirmed=gates&1U,.dynamic_response_confirmed=gates&2U,
            .minimum_iq_slew_a_per_s=.5f};
        n=(MotorFocNode){.limit_verified=gates&4U,.voltage_verified=gates&8U,
            .slew_verified=gates&16U,.remote_iq_limit_a=.6f,.remote_iq_slew_a_per_s=1.0f};
        hook=h;
        for(unsigned t=0;t<sizeof(edge)/sizeof(edge[0]);++t) compare((uint8_t)id,edge[t],&c,&n);
    }
    for(unsigned trial=0;trial<250000U;++trial) {
        c=(MotorFocCalibration){0};n=(MotorFocNode){0};
        float *values[]={&c.kt_nm_per_a,&c.current_limit_a,&c.minimum_iq_slew_a_per_s,
                        &n.remote_iq_limit_a,&n.remote_iq_slew_a_per_s};
        for(unsigned i=0;i<5;++i) {
            if(trial%3U==0U) *values[i]=from_bits(random32());
            else if(trial%3U==1U) *values[i]=edge[random32()%16U];
            else *values[i]=.001f+(float)(random32()%2000U)*.001f;
        }
        c.direction=trial%3U==0U ? (int32_t)(random32()%5U)-2 : (random32()&1U ? 1:-1);
        unsigned gates=trial%4U==0U ? random32():31U;
        c.calibration_confirmed=gates&1U;c.dynamic_response_confirmed=gates&2U;
        n.limit_verified=gates&4U;n.voltage_verified=gates&8U;n.slew_verified=gates&16U;
        c.angle_anchor_rad=from_bits(random32());c.angle_anchor_confirmed=random32();
        c.joint_timing_confirmed=random32();n.epoch=random32();n.faults=random32();
        float torque=trial%2U ? from_bits(random32()):edge[random32()%16U];
        hook=trial%7U==0U ? random32()%3U:0U;
        compare((uint8_t)(random32()%12U),torque,&c,&n);
    }
    printf("PASS: %u old/new snapshot cases, mask=0x%x, exact finite/signed-zero results, NaN/Inf, gates and lock-boundary mutation\n",
           cases,MOTOR_FOC_NODE_MASK);
}
'''

with tempfile.TemporaryDirectory(prefix="foc_scalar_snapshot_") as directory:
    work = Path(directory)
    (work / "test.c").write_text(prefix + reference + "\n" +
        function("current_hard_limit") + "\n" +
        function("fixed_iq_limit_ready", "int") + "\n" +
        function("MotorFoc_TorqueLimitNm") + "\n" +
        function("MotorFoc_CurrentForTorque") + checks, encoding="utf-8")
    exe = work / "test.exe"
    base = ["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-DMYLEGWHEEL_A1_CURRENT=1",
            "-DMOTOR_FOC_FIXED_IQ_LIMIT_A=0.0f",
            "-I", str(root / "App/Tasks/Inc"), "-I", str(root / "App/Services/Inc")]
    for mask in ("0x44U", "0x77U"):
        subprocess.run(base + ["-DMOTOR_FOC_NODE_MASK=" + mask, str(work / "test.c"),
                              "-lm", "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
