"""Build the Keil project with ARM Compiler 6, without opening uVision.

python scripts/build_keil.py [--keil-root D:/app/Keil_v5]
Default outputs go to build/keil, the same directory configured in uVision.
Named profiles write HEX/BIN/AXF and build logs under build/<profile>/.
Use --output-dir build/<directory> to keep a separate build of any profile.
"""
import argparse
from pathlib import Path
import os
import subprocess
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keil-root", type=Path, default=None)
    parser.add_argument("--profile", choices=("default", "foc-current", "legacy-voltage", "read-only", "capture", "leg-pd-left", "leg-pd-right", "leg-voltage-left", "leg-voltage-right"), default="default")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="Write firmware, objects and logs to this directory under build/; relative paths use the project root.")
    args = parser.parse_args()
    # 先解析真实路径再检查边界，拒绝通过 .. 或目录链接将构建产物写到 build/ 外。
    build_root = (root / "build").resolve()
    requested_output = (root / args.output_dir).resolve() if args.output_dir is not None else None
    if requested_output is not None and not requested_output.is_relative_to(build_root):
        parser.error("--output-dir must stay under the project's build/ directory.")
    candidates = [args.keil_root, os.environ.get("KEIL_ROOT"), "D:/app/Keil_v5", "C:/Keil_v5"]
    keil = next((Path(path) for path in candidates if path and
                 (Path(path) / "ARM/ARMCLANG/bin/armclang.exe").is_file()), None)
    if keil is None:
        parser.error("ARM Compiler 6 not found; pass --keil-root or set KEIL_ROOT.")
    toolchain = keil / "ARM/ARMCLANG/bin"
    project_dir = root / "MDK-ARM"
    project = ET.parse(project_dir / "MyLegwheel.uvprojx")
    target = project.find("./Targets/Target")
    common = target.find("./TargetOption/TargetCommonOption")
    output = (project_dir / common.findtext("OutputDirectory").replace("\\", "/")).resolve()
    listings = (project_dir / common.findtext("ListingPath").replace("\\", "/")).resolve()
    if not output.is_relative_to(build_root) or not listings.is_relative_to(build_root):
        parser.error("Project output/listing directories must stay under build/.")
    name = common.findtext("OutputName")
    if args.profile != "default":
        output = build_root / args.profile
        listings = output / "listings"
        name = "H723_READ_ONLY" if args.profile == "read-only" else "H723_" + args.profile.replace("-", "_")
    if requested_output is not None:
        # 仅替换本次输出位置；profile 继续决定宏和产物名称，保留默认构建行为。
        output = requested_output
        listings = output / "listings"
    output.mkdir(parents=True, exist_ok=True)
    listings.mkdir(parents=True, exist_ok=True)
    controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
    flags = ["--target=arm-arm-none-eabi", "-mcpu=cortex-m7", "-mfpu=fpv5-sp-d16",
             "-mfloat-abi=hard", "-std=c99", "-O2", "-ffunction-sections", "-fdata-sections",
             "-fshort-enums", "-fshort-wchar", "-D__MICROLIB"]
    # Match this project's uVision 5.34 compiler command (-gdwarf-3).
    # Plain -g defaults to DWARF 4 in armclang 6.16: GDB can read its globals,
    # but this uVision version fails to evaluate their types in Watch.
    # Keep O2; the debug format does not change runtime optimization.
    if common.findtext("DebugInformation") == "1":
        flags.append("-gdwarf-3")
    flags += ["-D" + value for value in controls.findtext("Define").split(",") if value]
    if args.profile == "legacy-voltage":
        flags = [flag for flag in flags if not flag.startswith("-DMYLEGWHEEL_A1_CURRENT=")]
        flags.append("-DMYLEGWHEEL_A1_CURRENT=0")
    if args.profile == "read-only":
        flags += ["-DROBOT_DEFAULT_FEEDBACK_ONLY=1", "-DROBOT_TELEMETRY_READ_ONLY_BUILD=1", "-DMOTOR_COMMAND_TX_ENABLED=0"]
    elif args.profile == "capture":
        flags += ["-DROBOT_CAPTURE_BUILD=1", "-DROBOT_DEFAULT_FEEDBACK_ONLY=0", "-DMOTOR_COMMAND_TX_ENABLED=1"]
    elif args.profile.startswith("leg-"):
        flags.append("-DROBOT_LEG_PD_TEST_SIDE=" + ("0" if args.profile.endswith("-left") else "1"))
        if args.profile.startswith("leg-voltage-"):
            flags.append("-DROBOT_LEG_VOLTAGE_PROBE=1")
    for include in controls.findtext("IncludePath").split(";"):
        directory = (project_dir / include.replace("\\", "/")).resolve()
        if not directory.is_dir():
            parser.error(f"Missing include directory: {directory}")
        flags += ["-I", str(directory)]
    units = [(project_dir / file.findtext("FilePath").replace("\\", "/")).resolve()
             for file in target.findall("./Groups/Group/Files/File")
             if file.findtext("FileType") in ("1", "2")]

    # CMSIS DSP is a Keil RTE component. Resolve its exact pack version, rather
    # than reusing stale DSP objects from another build directory.
    for component in project.findall("./RTE/components/component"):
        if component.get("Cgroup") != "DSP" or component.get("Cvariant") != "Source":
            continue
        package = component.find("package")
        pack = keil / "Packs" / package.get("vendor") / package.get("name") / package.get("version") / "CMSIS"
        if not pack.is_dir():
            parser.error(f"Missing CMSIS DSP pack: {pack}")
        flags += ["-I", str(pack / "Include"), "-I", str(pack / "DSP/Include"),
                  "-I", str(pack / "DSP/PrivateInclude")]
        for folder in sorted((pack / "DSP/Source").iterdir()):
            aggregate = folder / (folder.name + ".c")
            if aggregate.is_file(): units.append(aggregate)

    log_path = output / "build.log"
    objects = []
    seen = set()
    with log_path.open("w", encoding="utf-8") as log:
        def run(arguments):
            result = subprocess.run([str(arg) for arg in arguments], cwd=project_dir,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, errors="replace")
            log.write(subprocess.list2cmdline([str(arg) for arg in arguments]) + "\n" + result.stdout)
            log.flush()
            if result.stdout: print(result.stdout, end="", flush=True)
            if result.returncode:
                raise SystemExit(f"Build failed ({result.returncode}); see {log_path}")

        for source in units:
            if not source.is_file(): parser.error(f"Missing source: {source}")
            obj = output / (source.stem.lower() + ".o")
            if obj.name in seen: parser.error(f"Duplicate object name: {obj.name}")
            seen.add(obj.name)
            objects.append(obj)
            if source.suffix.lower() == ".c":
                run([toolchain / "armclang.exe", *flags, "-c", source, "-o", obj])
            else:
                asm_debug = ["--debug"] if common.findtext("DebugInformation") == "1" else []
                run([toolchain / "armasm.exe", *asm_debug, "--cpu", "Cortex-M7.fp.sp", "--apcs=interwork",
                     "--pd", "__MICROLIB SETA 1", "-o", obj, source])
        scatter = (project_dir / target.findtext("./TargetOption/TargetArmAds/LDads/ScatterFile")).resolve()
        axf = output / (name + ".axf")
        response = output / (name + ".lnp")
        arguments = ["--cpu", "Cortex-M7.fp.sp", *objects, "--library_type=microlib", "--strict",
                     "--scatter", scatter, "--summary_stderr", "--info", "summarysizes", "--map", "--callgraph",
                     "--load_addr_map_info", "--xref", "--symbols", "--info", "sizes,totals,unused,veneers",
                     "--list", listings / (name + ".map"), "-o", axf]
        response.write_text("\n".join('"' + str(arg) + '"' for arg in arguments), encoding="utf-8")
        run([toolchain / "armlink.exe", "--via", response])
        run([toolchain / "fromelf.exe", "--i32combined", "--output", output / (name + ".hex"), axf])
        run([toolchain / "fromelf.exe", "--bin", "--output", output / (name + ".bin"), axf])
    print(f"PASS: rebuilt {len(units)} source files; firmware: {output / (name + '.hex')}")


if __name__ == "__main__":
    main()
