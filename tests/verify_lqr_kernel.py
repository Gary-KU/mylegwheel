"""主机 LQR 回归：对照优化前内核逐位检查输出，可选记录重复求增益的耗时。

python tests/verify_lqr_kernel.py [--benchmark]
仅需 PATH 中的 gcc；独立控制实现来自优化前快照；冻结系数于2026-11-13按新1.6kg联合DLQR审核更新。
以后有意调整模型系数或有效腿长范围时，应重新审核并更新这份回归基线。
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


# 冻结的旧实现作为独立编译单元，避免用新函数自己验证自己。
BASELINE_SOURCE = r'''
/* Reviewed frozen coefficients from joint1.6kg DLQR generator (2026-11-13).
 * Q=[800,15,200,40,200,80],R=[100000,10000];fit4..15cm; model screening only.
 * Total1.600kg, body1.250kg; .625kg body per side.
 * Wheel/leg split uses the prior geometry ratio, scaled to 0.350 kg.
 * Estimated part masses: see models/lqr/lqr_model_metadata.json.
 * Feedback scales vs nominal DLQR: wheel 1, leg 1.
 * MATLAB column-major 2x6; feedback/state conventions unchanged. */
#include "lqr_k.h"
#include <math.h>
/* 腿长 L0（m）用于调度增益；三次拟合用 Horner 形式求值，适用范围见 lqr_k.h。 */
void baseline_lqr_k(float L0, float K[12])
{
    K[0] = ((-6.36774330485909701e+01f*L0 + (2.75856347654574314e+01f))*L0 + (-5.04824012627304697e+00f))*L0 + (-1.53784814259563720e-02f);
    K[1] = ((8.62324029946065451e+01f*L0 + (-3.10483831522314908e+01f))*L0 + (3.92406256417693333e+00f))*L0 + (2.60089374418339021e-02f);
    K[2] = ((-3.18563224846516757e+00f*L0 + (1.24837356759328189e+00f))*L0 + (-3.34875548612745100e-01f))*L0 + (-6.33193110350154897e-04f);
    K[3] = ((4.66950719747191467e+00f*L0 + (-1.33749832627408405e+00f))*L0 + (6.27093017125283231e-02f))*L0 + (5.26330156735970930e-03f);
    K[4] = ((-2.95386108965965244e+00f*L0 + (1.18579040237773170e+00f))*L0 + (-1.73696859935389719e-01f))*L0 + (-3.15668660384860397e-02f);
    K[5] = ((2.08112309152582569e+01f*L0 + (-7.17369409219590715e+00f))*L0 + (8.01888368328531831e-01f))*L0 + (-3.90278373939434153e-02f);
    K[6] = ((-1.83944554823317130e+00f*L0 + (7.07194727589275596e-01f))*L0 + (-1.14545591128517935e-01f))*L0 + (-4.75519163200269768e-02f);
    K[7] = ((2.55809798344616546e+01f*L0 + (-8.90575784146255955e+00f))*L0 + (1.01905808019909405e+00f))*L0 + (-5.72633107904357702e-02f);
    K[8] = ((-3.00805040334147229e+01f*L0 + (1.39173445405767566e+01f))*L0 + (-2.61511115640672731e+00f))*L0 + (2.73438352074742985e-01f);
    K[9] = ((2.13264545787486526e+01f*L0 + (-1.10608576227753765e+01f))*L0 + (2.19984208701080597e+00f))*L0 + (2.46658873662261913e-01f);
    K[10] = ((-5.65864671106885986e+00f*L0 + (2.43426534705279707e+00f))*L0 + (-3.96604690193417098e-01f))*L0 + (3.57956745074959129e-02f);
    K[11] = ((1.64669289866093500e+01f*L0 + (-6.41162176562489083e+00f))*L0 + (8.88681204503824040e-01f))*L0 + (2.42495536386452097e-02f);
}

int baseline_lqr_control(float L0, const float state[6], const float reference[6], float u[2])
{
    float K[12];
    int i;
    if (!u) return 0;
    u[0] = 0.0f; u[1] = 0.0f;
    if (!state || !reference || !isfinite(L0) ||
        L0 < LQR_LEG_MIN_M || L0 > LQR_LEG_MAX_M) return 0;
    for (i = 0; i < 6; ++i) {
        if (!isfinite(state[i]) || !isfinite(reference[i])) return 0;
    }
    /* 状态次序为 [theta,dtheta,x,dx,phi,dphi]；MATLAB 列优先布局使轮/腿增益交错存放。
     * 负反馈输出 u=[轮力矩 T,虚拟腿摆角力矩 Tp]，实际电机方向与限幅由调用方处理。 */
    baseline_lqr_k(L0,K);
    for (i = 0; i < 6; ++i) {
        float error = state[i]-reference[i];
        u[0] -= K[2*i]*error;
        u[1] -= K[2*i+1]*error;
    }
    if (!isfinite(u[0]) || !isfinite(u[1])) {
        u[0] = 0.0f; u[1] = 0.0f; return 0;
    }
    return 1;
}
'''

HARNESS = r'''

#include "lqr_k.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

void baseline_lqr_k(float, float [12]);
int baseline_lqr_control(float, const float [6], const float [6], float [2]);

static uint32_t random_state = 20261107U;
static unsigned checked, accepted, rejected;
static volatile float benchmark_sink;

static uint32_t random_bits(void)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

static float random_unit(void)
{
    return (float)(random_bits() & 0xffffffU) / 16777215.0f;
}

static void positive_zero(const float *values, unsigned count)
{
    for (unsigned i = 0; i < count; ++i) {
        assert(values[i] == 0.0f && !signbit(values[i]));
    }
}

static void check(float length, const float *state, const float *reference)
{
    float before[2] = {17, -23}, wrapper[2] = {17, -23}, result[2] = {17, -23};
    float gain[12], expected_gain[12], saved_state[6], saved_reference[6];
    for (unsigned i = 0; i < 12; ++i) gain[i] = NAN;
    if (state) memcpy(saved_state, state, sizeof(saved_state));
    if (reference) memcpy(saved_reference, reference, sizeof(saved_reference));
    int expected = baseline_lqr_control(length, state, reference, before);
    int got_wrapper = lqr_control(length, state, reference, wrapper);
    int got = lqr_control_with_gain(length, state, reference, result, gain);
    if (got != expected || got_wrapper != expected ||
        memcmp(before, wrapper, sizeof(before)) || memcmp(before, result, sizeof(before))) {
        fprintf(stderr, "LQR mismatch at sample %u, length %.9g\n", checked, (double)length);
        abort();
    }
    if (expected) {
        baseline_lqr_k(length, expected_gain);
        assert(memcmp(gain, expected_gain, sizeof(gain)) == 0);
        ++accepted;
    } else {
        positive_zero(result, 2);
        positive_zero(wrapper, 2);
        positive_zero(gain, 12);
        ++rejected;
    }
    if (state) assert(memcmp(saved_state, state, sizeof(saved_state)) == 0);
    if (reference) assert(memcmp(saved_reference, reference, sizeof(saved_reference)) == 0);
    ++checked;
}

static void verify(void)
{
    float state[6] = {0}, reference[6] = {0};
    /* Requested runtime extension: include 150 mm, reject its next float. */
    float boundary_u[2];
    assert(lqr_control(.15f, state, reference, boundary_u));
    assert(!lqr_control(nextafterf(.15f, 1.0f), state, reference, boundary_u));
    const float lengths[] = {
        .04f, nextafterf(.04f, 1.0f), .07f, .11f, nextafterf(.14f, 0.0f), .14f,
        nextafterf(.14f, 1.0f), .145f, nextafterf(.15f, 0.0f), .15f,
        nextafterf(.04f, 0.0f), nextafterf(.15f, 1.0f), 0, -1,
        NAN, INFINITY, -INFINITY, FLT_MAX, -FLT_MAX
    };
    for (unsigned n = 0; n < sizeof(lengths) / sizeof(lengths[0]); ++n) {
        check(lengths[n], state, reference);
        for (unsigned column = 0; column < 6; ++column) {
            state[column] = .2f;
            reference[column] = -.03f;
            check(lengths[n], state, reference);
            state[column] = reference[column] = 0;
        }
    }
    /* 正常控制量与宽幅有限浮点输入分开覆盖，后者可触发差值/累加溢出。 */
    for (unsigned sample = 0; sample < 30000; ++sample) {
        float length = .04f + .11f * random_unit();
        for (unsigned i = 0; i < 6; ++i) {
            state[i] = (random_unit() - .5f) * 20.0f;
            reference[i] = (random_unit() - .5f) * 20.0f;
            if (sample >= 20000) {
                uint32_t bits = random_bits();
                if ((bits & 0x7f800000U) == 0x7f800000U) bits ^= 0x00800000U;
                memcpy(&state[i], &bits, sizeof(bits));
                bits = random_bits();
                if ((bits & 0x7f800000U) == 0x7f800000U) bits ^= 0x00800000U;
                memcpy(&reference[i], &bits, sizeof(bits));
            }
        }
        check(length, state, reference);
    }
    memset(state, 0, sizeof(state));
    memset(reference, 0, sizeof(reference));
    const float extremes[] = {NAN, INFINITY, -INFINITY, FLT_MAX, -FLT_MAX, -0.0f, FLT_MIN};
    for (unsigned column = 0; column < 6; ++column) {
        for (unsigned v = 0; v < sizeof(extremes) / sizeof(extremes[0]); ++v) {
            state[column] = extremes[v];
            check(.07f, state, reference);
            state[column] = 0;
            reference[column] = extremes[v];
            check(.07f, state, reference);
            state[column] = -extremes[v];
            check(.07f, state, reference);
            state[column] = reference[column] = 0;
        }
    }
    check(.07f, NULL, reference);
    check(.07f, state, NULL);
    check(.07f, NULL, NULL);
    /* 失败接口不留旧力矩或旧增益；NULL u 保留旧 API 的零返回约定。 */
    float u[2] = {1, -1}, gain[12];
    for (unsigned i = 0; i < 12; ++i) gain[i] = 9;
    assert(!baseline_lqr_control(.07f, state, reference, NULL));
    assert(!lqr_control(.07f, state, reference, NULL));
    assert(!lqr_control_with_gain(.07f, state, reference, NULL, gain));
    positive_zero(gain, 12);
    assert(!lqr_control_with_gain(.07f, state, reference, u, NULL));
    positive_zero(u, 2);
    assert(!lqr_control_with_gain(.07f, NULL, NULL, NULL, NULL));
    /* 兼容旧入口允许 u 指向输入数组的行为：先清零 u，再校验和计算输入。 */
    float old_alias[6] = {.1f, -.2f, .3f, -.4f, .5f, -.6f};
    float new_alias[6], shared_alias[6];
    memcpy(new_alias, old_alias, sizeof(old_alias));
    memcpy(shared_alias, old_alias, sizeof(old_alias));
    int expected = baseline_lqr_control(.07f, old_alias, reference, old_alias);
    assert(lqr_control(.07f, new_alias, reference, new_alias) == expected);
    assert(lqr_control_with_gain(.07f, shared_alias, reference, shared_alias, gain) == expected);
    assert(!memcmp(old_alias, new_alias, sizeof(old_alias)));
    assert(!memcmp(old_alias, shared_alias, sizeof(old_alias)));
    assert(accepted > 20000 && rejected > 100);
    printf("PASS: %u samples, %u valid, %u rejected; bitwise outputs/gains, boundaries, signed zero, overflow and NULL paths.\n",
           checked, accepted, rejected);
}

static void benchmark(void)
{
    const unsigned iterations = 1000000U;
    const float state[6] = {.1f, -.2f, .03f, -.04f, .05f, -.06f};
    const float reference[6] = {0};
    float u[2], gain[12];
    for (unsigned trial = 0; trial < 3; ++trial) {
        clock_t start = clock();
        for (unsigned i = 0; i < iterations; ++i) {
            float length = .04f + (float)(i % 1000U) * .0001f;
            int ok = baseline_lqr_control(length, state, reference, u);
            baseline_lqr_k(length, gain);
            benchmark_sink = ok ? u[0] + gain[(i % 6U) * 2U] : 0;
        }
        double before = (double)(clock() - start) / CLOCKS_PER_SEC;
        start = clock();
        for (unsigned i = 0; i < iterations; ++i) {
            float length = .04f + (float)(i % 1000U) * .0001f;
            int ok = lqr_control_with_gain(length, state, reference, u, gain);
            benchmark_sink = ok ? u[0] + gain[(i % 6U) * 2U] : 0;
        }
        double after = (double)(clock() - start) / CLOCKS_PER_SEC;
        printf("Host-only trial %u, %u calls: old duplicate gain %.6f s, shared gain %.6f s\n",
               trial + 1, iterations, before, after);
    }
}

int main(int argc, char **argv)
{
    verify();
    if (argc > 1 && strcmp(argv[1], "--benchmark") == 0) benchmark();
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark", action="store_true", help="记录主机耗时，不代表 MCU 性能")
    options = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    control = root / "Control/WheelLeg"
    with tempfile.TemporaryDirectory(prefix="mylegwheel_lqr_") as directory:
        work = Path(directory)
        (work / "baseline.c").write_text(BASELINE_SOURCE, encoding="utf-8")
        (work / "test.c").write_text(HARNESS, encoding="utf-8")
        exe = work / "test.exe"
        # 分离编译单元且不开 LTO，让旧路径的两次求增益调用真实保留。
        command = ["gcc", "-std=c99", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror",
                   "-I", str(control / "Inc"), str(work / "baseline.c"), str(work / "test.c"),
                   str(control / "Src/lqr_k.c"), "-lm", "-o", str(exe)]
        subprocess.run(command, check=True)
        subprocess.run([str(exe)] + (["--benchmark"] if options.benchmark else []), check=True)


if __name__ == "__main__":
    main()
