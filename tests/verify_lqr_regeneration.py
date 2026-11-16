"""在临时工程重生成 LQR 多项式，检查新旧控制入口均保留且数学结果不变。"""
from pathlib import Path
import re
import runpy
import shutil
import subprocess
import sys
import tempfile


LEGACY_HARNESS = r'''
#include "lqr_k.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int baseline_lqr_control(float, const float [6], const float [6], float [2]);

static void check(float length, const float state[6], const float reference[6])
{
    float expected[2] = {1, -1}, actual[2] = {1, -1};
    int ok = baseline_lqr_control(length, state, reference, expected);
    assert(lqr_control(length, state, reference, actual) == ok);
    assert(memcmp(expected, actual, sizeof(expected)) == 0);
}

int main(void)
{
    float state[6] = {0}, reference[6] = {0};
    for (unsigned n = 0; n <= 100; ++n) {
        for (unsigned column = 0; column < 6; ++column) {
            state[column] = .2f;
            reference[column] = -.03f;
            check(.04f + (float)n * .001f, state, reference);
            state[column] = reference[column] = 0;
        }
    }
    check(nextafterf(.04f, 0.0f), state, reference);
    check(nextafterf(.14f, 1.0f), state, reference);
    check(NAN, state, reference);
    check(INFINITY, state, reference);
    state[0] = INFINITY;
    check(.07f, state, reference);
    state[0] = FLT_MAX;
    reference[0] = -FLT_MAX;
    check(.07f, state, reference);
    check(.07f, NULL, reference);
    check(.07f, state, NULL);
    assert(!lqr_control(.07f, state, reference, NULL));
    puts("PASS: regenerated legacy-only kernel preserves valid/invalid behavior.");
    return 0;
}
'''


def regenerate(root, project, source, first_entry):
    inputs = (
        "scripts/update_lqr_coefficients.py",
        "models/lqr/lqr_model_metadata.json",
        "models/lqr/lqr_coefficients.csv",
        "Control/WheelLeg/Inc/lqr_k.h",
        "tests/verify_lqr_kernel.py",
    )
    for relative in inputs:
        destination = project / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / relative, destination)
    target = project / "Control/WheelLeg/Src/lqr_k.c"
    target.parent.mkdir(parents=True)
    # 故意留下过期系数，确认生成器确实替换多项式，而不是碰巧保持文件不变。
    stale, replacements = re.subn(r"K\[0\] = [^\n]*;", "K[0] = -999.0f;", source, count=1)
    assert replacements == 1
    target.write_text(stale, encoding="utf-8")
    # 保留整个求值器后缀；共享 helper 即便放在公开入口前也不能被生成器丢弃。
    evaluator_end = source.index("\n}", source.index("void lqr_k(")) + len("\n}")
    expected_suffix = source[evaluator_end:].lstrip("\r\n")
    expected_control = source[source.index(first_entry):]
    subprocess.run([sys.executable, str(project / inputs[0])], check=True)
    generated = target.read_text(encoding="utf-8")
    assert generated[generated.index(first_entry):] == expected_control
    generated_end = generated.index("\n}", generated.index("void lqr_k(")) + len("\n}")
    assert generated[generated_end:].lstrip("\r\n") == expected_suffix
    assert "K[0] = -999.0f;" not in generated
    assert generated.count("int lqr_control(") == 1
    assert generated.count("int lqr_control_with_gain(") == source.count("int lqr_control_with_gain(")
    backups = list((project / "backups").glob("*/lqr_k.c"))
    assert len(backups) == 1 and backups[0].read_text(encoding="utf-8") == stale
    return target


def main():
    root = Path(__file__).resolve().parents[1]
    real_source = root / "Control/WheelLeg/Src/lqr_k.c"
    unchanged = real_source.read_bytes()
    current = real_source.read_text(encoding="utf-8")
    kernel = runpy.run_path(str(root / "tests/verify_lqr_kernel.py"))
    # 使用现有回归中冻结的旧内核，同时覆盖旧工程只有一个入口的迁移场景。
    legacy = kernel["BASELINE_SOURCE"].lstrip("\n").replace("baseline_lqr_k", "lqr_k")
    legacy = legacy.replace("baseline_lqr_control", "lqr_control")
    with tempfile.TemporaryDirectory(prefix="mylegwheel_lqr_regenerate_") as directory:
        work = Path(directory)
        shared_project = work / "shared_gain"
        regenerate(root, shared_project, current, "int lqr_control_with_gain(")
        # 直接复用完整数学回归，在重生成后的临时源码上编译并测试所有异常路径。
        subprocess.run([sys.executable, str(shared_project / "tests/verify_lqr_kernel.py")], check=True)
        legacy_project = work / "legacy"
        target = regenerate(root, legacy_project, legacy, "int lqr_control(")
        (legacy_project / "baseline.c").write_text(kernel["BASELINE_SOURCE"], encoding="utf-8")
        (legacy_project / "test.c").write_text(LEGACY_HARNESS, encoding="utf-8")
        exe = legacy_project / "test.exe"
        subprocess.run([
            "gcc", "-std=c99", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror",
            "-I", str(legacy_project / "Control/WheelLeg/Inc"),
            str(legacy_project / "baseline.c"), str(legacy_project / "test.c"),
            str(target), "-lm", "-o", str(exe),
        ], check=True)
        subprocess.run([str(exe)], check=True)
    assert real_source.read_bytes() == unchanged
    print("PASS: generator preserves complete shared/legacy control implementations and backs up each input; real source untouched.")


if __name__ == "__main__":
    main()
