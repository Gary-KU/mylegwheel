"""用生产 armclang 参数编译并仿真 Cortex-M7 LQR，区分数值与零符号差异。

python tests/verify_lqr_arm.py --deps-dir build/performance_20261107/python_deps
依赖 unicorn、pyelftools 和 Keil ARM Compiler；缺失时明确失败，不自动安装或跳过。
本测试验证 ARM 指令与浮点结果，不测量开发板周期，也不代表实机稳定性验证。
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import runpy
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET


FLASH, RAM, STOP = 0x08000000, 0x20000000, 0x081FF000
STATE, REFERENCE, OUTPUT, GAIN = [RAM + i * 0x1000 for i in range(1, 5)]


def bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


def pack(values):
    return struct.pack("<6I", *values)


class ArmKernel:
    def __init__(self, image, libraries):
        unicorn, self.arm, elf_reader = libraries
        self.cpu = unicorn.Uc(unicorn.UC_ARCH_ARM, unicorn.UC_MODE_THUMB | unicorn.UC_MODE_MCLASS)
        self.cpu.ctl_set_cpu_model(self.arm.UC_CPU_ARM_CORTEX_M7)
        self.cpu.mem_map(FLASH, 0x200000)
        self.cpu.mem_map(RAM, 0x100000)
        with image.open("rb") as stream:
            elf = elf_reader(stream)
            for segment in elf.iter_segments():
                if segment.header["p_type"] == "PT_LOAD":
                    self.cpu.mem_write(segment.header["p_vaddr"], segment.data())
            self.symbols = {s.name: s.entry["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
        self.cpu.reg_write(self.arm.UC_ARM_REG_C1_C0_2, 0xF << 20)
        self.cpu.reg_write(self.arm.UC_ARM_REG_FPEXC, 0x40000000)

    def call(self, function, length, state, reference, fpscr=0, null=(), alias=None):
        cpu, arm = self.cpu, self.arm
        output = STATE if alias == "state" else REFERENCE if alias == "reference" else OUTPUT
        registers = [(arm.UC_ARM_REG_SP, RAM + 0xFF000), (arm.UC_ARM_REG_LR, STOP | 1),
                     (arm.UC_ARM_REG_S0, length), (arm.UC_ARM_REG_FPSCR, fpscr)]
        pointers = [GAIN if function == "lqr_k" else STATE, REFERENCE, output, GAIN]
        for i, register in enumerate([arm.UC_ARM_REG_R0, arm.UC_ARM_REG_R1, arm.UC_ARM_REG_R2, arm.UC_ARM_REG_R3]):
            registers.append((register, 0 if i in null else pointers[i]))
        for register, value in registers:
            cpu.reg_write(register, value)
        cpu.mem_write(STATE, state)
        cpu.mem_write(REFERENCE, reference)
        if alias is None:
            cpu.mem_write(OUTPUT, b"\xCD" * 8)
        cpu.mem_write(GAIN, b"\xCD" * 48)
        cpu.emu_start(self.symbols[function], STOP, count=10000)
        if cpu.reg_read(arm.UC_ARM_REG_PC) != STOP:
            raise RuntimeError(f"{function} did not return within the instruction limit")
        return (cpu.reg_read(arm.UC_ARM_REG_R0), bytes(cpu.mem_read(output, 8)),
                bytes(cpu.mem_read(GAIN, 48)), cpu.reg_read(arm.UC_ARM_REG_FPSCR),
                bytes(cpu.mem_read(STATE, 24)), bytes(cpu.mem_read(REFERENCE, 24)))


def samples():
    rng, zero = random.Random(20261107), [0] * 6
    low, high = bits(.04), bits(.14)
    for length in [low, low + 1, bits(.07), bits(.11), high - 1, high, low - 1, high + 1,
                   0, 0x80000000, 0xBF800000, 0x7F800000, 0xFF800000, 0x7FC00000, 0x7F800001, 0x7F7FFFFF]:
        yield length, pack(zero), pack(zero), (), None
        for i in range(6):
            state, reference = zero.copy(), zero.copy()
            state[i], reference[i] = bits(.2), bits(-.03)
            yield length, pack(state), pack(reference), (), None
    for _ in range(20000):
        yield bits(rng.uniform(.04, .14)), pack([bits(rng.uniform(-3, 3)) for _ in range(6)]), \
            pack([bits(rng.uniform(-3, 3)) for _ in range(6)]), (), None
    for _ in range(5000):
        values = [rng.getrandbits(32) for _ in range(12)]
        values = [v ^ 0x00800000 if v & 0x7F800000 == 0x7F800000 else v for v in values]
        yield bits(rng.uniform(.04, .14)), pack(values[:6]), pack(values[6:]), (), None
    # 保留原始浮点位型，包含次正规数、qNaN/sNaN、正负无穷和相反大值溢出。
    extremes = [0, 0x80000000, 1, 0x80000001, 0x007FFFFF, 0x807FFFFF, 0x00800000,
                0x80800000, 0x7F7FFFFF, 0xFF7FFFFF, 0x7F800000, 0xFF800000, 0x7FC00000, 0x7F800001]
    for length in [low, bits(.07), high]:
        for i in range(6):
            for value in extremes:
                state, reference = zero.copy(), zero.copy()
                state[i] = value
                yield length, pack(state), pack(reference), (), None
                reference[i] = value ^ 0x80000000
                yield length, pack(state), pack(reference), (), None
    for null in [(0,), (1,), (2,), (0, 1, 2)]:
        yield bits(.07), pack(zero), pack(zero), null, None
    # 旧入口没有 restrict 约定；u 与两种输入别名时也必须保持原读取/写入顺序。
    for alias in ["state", "reference"]:
        for _ in range(1000):
            yield bits(rng.uniform(.04, .14)), pack([bits(rng.uniform(-3, 3)) for _ in range(6)]), \
                pack([bits(rng.uniform(-3, 3)) for _ in range(6)]), (), alias


def statistics():
    return dict(samples=0, accepted=0, rejected=0, bitwise_mismatches=0, signed_zero_only=0,
                numeric_mismatches=0, status_mismatches=0, gain_mismatches=0,
                failure_clear_mismatches=0, input_memory_mismatches=0, fpscr_exception_mismatches=0,
                max_abs_error=0.0, max_relative_error=0.0, max_ulp_error=0)


def ordered(value):
    return (~value & 0xFFFFFFFF) if value & 0x80000000 else value | 0x80000000


def compare(stats, expected, actual, expected_gain, null, check_gain):
    stats["samples"] += 1
    stats["accepted" if expected[0] else "rejected"] += 1
    stats["status_mismatches"] += expected[0] != actual[0]
    stats["input_memory_mismatches"] += expected[4:] != actual[4:]
    stats["fpscr_exception_mismatches"] += bool((expected[3] ^ actual[3]) & 0x9F)
    if check_gain and expected[0]:
        stats["gain_mismatches"] += actual[2] != expected_gain
    if not actual[0]:
        stats["failure_clear_mismatches"] += (2 not in null and actual[1] != bytes(8)) or (check_gain and actual[2] != bytes(48))
    if expected[:2] == actual[:2]:
        return
    stats["bitwise_mismatches"] += 1
    ev, av = struct.unpack("<2f", expected[1]), struct.unpack("<2f", actual[1])
    zero_only = expected[0] == actual[0] and ev == av
    stats["signed_zero_only" if zero_only else "numeric_mismatches"] += 1
    for e, a, eb, ab in zip(ev, av, struct.unpack("<2I", expected[1]), struct.unpack("<2I", actual[1])):
        if math.isfinite(e) and math.isfinite(a):
            error = abs(e - a)
            stats["max_abs_error"] = max(stats["max_abs_error"], error)
            stats["max_relative_error"] = max(stats["max_relative_error"], error / max(abs(e), 1e-45))
            stats["max_ulp_error"] = max(stats["max_ulp_error"], 0 if e == a else abs(ordered(eb) - ordered(ab)))


def compile_kernels(root, work, keil, source):
    project_dir = root / "MDK-ARM"
    project = ET.parse(project_dir / "MyLegwheel.uvprojx")
    target = project.find("./Targets/Target")
    controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
    # 与 scripts/build_keil.py 保持相同的生产编译参数，不增加 fast-math 或改变乘加策略。
    flags = ["--target=arm-arm-none-eabi", "-mcpu=cortex-m7", "-mfpu=fpv5-sp-d16", "-mfloat-abi=hard",
             "-std=c99", "-O2", "-ffunction-sections", "-fdata-sections", "-fshort-enums", "-fshort-wchar", "-D__MICROLIB"]
    if target.findtext("./TargetOption/TargetCommonOption/DebugInformation") == "1":
        flags.append("-gdwarf-3")
    flags += ["-D" + value for value in controls.findtext("Define").split(",") if value]
    for include in controls.findtext("IncludePath").split(";"):
        flags += ["-I", str((project_dir / include.replace("\\", "/")).resolve())]
    for component in project.findall("./RTE/components/component"):
        if component.get("Cgroup") == "DSP" and component.get("Cvariant") == "Source":
            package = component.find("package")
            cmsis = keil / "Packs" / package.get("vendor") / package.get("name") / package.get("version") / "CMSIS"
            flags += ["-I", str(cmsis / "Include"), "-I", str(cmsis / "DSP/Include"), "-I", str(cmsis / "DSP/PrivateInclude")]
    baseline = runpy.run_path(str(root / "tests/verify_lqr_kernel.py"))["BASELINE_SOURCE"]
    baseline = baseline.replace("baseline_lqr_control", "lqr_control").replace("baseline_lqr_k", "lqr_k")
    (work / "baseline.c").write_text(baseline, encoding="utf-8")
    (work / "current.c").write_bytes(source.read_bytes())
    toolchain = keil / "ARM/ARMCLANG/bin"
    commands = []
    for name in ["baseline", "current"]:
        commands += [[str(toolchain / "armclang.exe"), *flags, "-c", str(work / (name + ".c")), "-o", str(work / (name + ".o"))],
                     [str(toolchain / "armlink.exe"), "--cpu", "Cortex-M7.fp.sp", "--library_type=microlib", "--entry=lqr_control",
                      "--ro_base=0x08000000", "--rw_base=0x20000000", "--no_remove", str(work / (name + ".o")), "-o", str(work / (name + ".axf"))]]
    for command in commands:
        subprocess.run(command, check=True)
    (work / "commands.json").write_text(json.dumps(commands, indent=2), encoding="utf-8")
    version = subprocess.run([str(toolchain / "armclang.exe"), "--version"], capture_output=True, text=True, check=True).stdout
    return version, flags


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deps-dir", type=Path, default=root / "build/performance_20261107/python_deps")
    parser.add_argument("--keil-root", type=Path)
    parser.add_argument("--output-dir", type=Path, default=root / "build/lqr_arm_verify")
    parser.add_argument("--source", type=Path, default=root / "Control/WheelLeg/Src/lqr_k.c", help="可选的实验 C 源码；默认测试当前生产源码")
    parser.add_argument("--require-bitwise", action="store_true", help="把仅零符号位不同也视为失败")
    args = parser.parse_args()
    build_root, work = (root / "build").resolve(), args.output_dir.resolve()
    if not work.is_relative_to(build_root):
        parser.error("--output-dir must stay under the project's build/ directory")
    sys.path.insert(0, str(args.deps_dir.resolve()))
    try:
        import unicorn
        import unicorn.arm_const as arm
        from elftools.elf.elffile import ELFFile
    except ImportError as error:
        parser.error(f'{error}. Install locally: python -m pip install --target "{args.deps_dir}" unicorn pyelftools')
    candidates = [args.keil_root, os.environ.get("KEIL_ROOT"), "D:/app/Keil_v5", "C:/Keil_v5"]
    keil = next((Path(p) for p in candidates if p and (Path(p) / "ARM/ARMCLANG/bin/armclang.exe").is_file()), None)
    if keil is None:
        parser.error("ARM Compiler not found; pass --keil-root")
    source = args.source.resolve()
    if not source.is_file():
        parser.error(f"Missing LQR source: {source}")
    work.mkdir(parents=True, exist_ok=True)
    version, flags = compile_kernels(root, work, keil, source)
    libraries = unicorn, arm, ELFFile
    old, current = ArmKernel(work / "baseline.axf", libraries), ArmKernel(work / "current.axf", libraries)
    cases, results = list(samples()), {}
    for fpscr in [0, 0x03000000]:
        entries = {"with_gain": statistics(), "legacy_wrapper": statistics()}
        for length, state, reference, null, alias in cases:
            expected = old.call("lqr_control", length, state, reference, fpscr, null, alias)
            gain = old.call("lqr_k", length, state, reference, fpscr)[2] if expected[0] else None
            for label, function in [("with_gain", "lqr_control_with_gain"), ("legacy_wrapper", "lqr_control")]:
                actual = current.call(function, length, state, reference, fpscr, null, alias)
                compare(entries[label], expected, actual, gain, null, label == "with_gain")
        # 新 API 的 NULL gain 没有旧入口可比较，独立检查拒绝及正零输出。
        missing_gain = current.call("lqr_control_with_gain", bits(.07), bytes(24), bytes(24), fpscr, (3,))
        if missing_gain[0] or missing_gain[1] != bytes(8):
            raise SystemExit("FAIL: NULL gain did not reject with zero output")
        results[hex(fpscr)] = entries
        for label, stats in entries.items():
            print(f"FPSCR={fpscr:#010x} {label}: {stats['samples']} samples, "
                  f"numeric/status/gain/clear/FP-flag mismatches="
                  f"{stats['numeric_mismatches']}/{stats['status_mismatches']}/{stats['gain_mismatches']}/"
                  f"{stats['failure_clear_mismatches']}/{stats['fpscr_exception_mismatches']}, "
                  f"signed-zero-only={stats['signed_zero_only']}")
    report = dict(compiler=version, flags=flags, unicorn_version=unicorn.__version__, cpu="Cortex-M7",
                  source=str(source), source_sha256=hashlib.sha256((work / "current.c").read_bytes()).hexdigest(),
                  cases_per_mode=len(cases), results=results,
                  limitations=["Instruction/FP emulation; not hardware timing or robot stability testing.",
                               "Production compile flags with a minimal standalone ELF link map.",
                               "FPSCR is explicitly set to 0 and 0x03000000, not read from the board.",
                               "NaN inputs are rejected and outputs cleared; NaN payload propagation is not promised."])
    (work / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    fields = ["numeric_mismatches", "status_mismatches", "gain_mismatches", "failure_clear_mismatches",
              "input_memory_mismatches", "fpscr_exception_mismatches"]
    if args.require_bitwise:
        fields.append("bitwise_mismatches")
    if any(stats[field] for mode in results.values() for stats in mode.values() for field in fields):
        raise SystemExit(f"FAIL: ARM regression; see {work / 'report.json'}")
    print(f"PASS: production ARM floating-point kernel regression; report {work / 'report.json'}")


if __name__ == "__main__":
    main()
