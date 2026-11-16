"""Export verified summaries and build a source-only open-source ZIP."""
from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
import zipfile
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
ROOT = WORKSPACE / "projects" / "wheel-legged-simulation"
OUTPUT = WORKSPACE / "build" / "wheel-legged-simulation_20261011.zip"


def write_validation_document(reports: dict) -> None:
    lines = [
        "# 发布验证", "",
        "2026-10-11 在 MATLAB R2023b / Windows 上，对本仓库运行 `run_validation('all')`。",
        "恢复 MATLAB 默认搜索路径后，在仓库外的当前目录执行；仅加入本仓库入口，验证不依赖作者原工程目录。",
        "**LQR 3 个工况、LMPC 5 个工况全部通过。** 模型文件与来源快照中的保存版 SHA-256 一致。", "",
        "## LQR", "",
        "站立 5 s，初始俯仰 +10° / -10° 各 8 s。要求最后一秒俯仰绝对值小于 1°、速度绝对值小于 0.005 m/s，最终腿长误差小于 5 mm，并检查电机限幅。", "",
        "| 工况 | 最终俯仰 ° | 最终速度 m/s | 最大轮力矩 N·m | 最大关节力矩 N·m | 结果 |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for name, case in reports["lqr"]["cases"].items():
        lines.append(f"| {name} | {case['pitch_final_deg']:.6f} | {case['speed_final_m_s']:.6f} | {case['wheel_peak_nm']:.6f} | {case['joint_peak_nm']:.6f} | 通过 |")
    pole = reports["lqr"]["max_linear_pole_radius"]
    lines.extend([
        "", f"101 个腿长处的离散闭环最大极点半径：**{pole:.9f}**。这只说明所检查的线性化稳定性。",
        "", "![LQR 验证曲线](figures/lqr_validation.png)", "",
        "完整参数与判据摘要：[validation_lqr.json](validation_lqr.json)。", "",
        "## LMPC + Roll PD", "",
        "每个工况 5 s；`braking` 是初始速度 0.15 m/s、零参考的制动恢复。组合扰动同时施加横滚 ±3°、俯仰 ±3° 与初始前进速度 0.15 m/s。", "",
        "要求最后一秒横滚 RMS 小于 0.5°、最终俯仰小于 0.5°、最终速度小于 0.005 m/s、腿长误差小于 5 mm、腿长末段变化小于 2 mm，轮与关节输出不超限，QP 状态正常。", "",
        "| 工况 | 最终俯仰 ° | 最终速度 m/s | 最后 1 s 横滚 RMS ° | 最大轮力矩 N·m | 最大关节力矩 N·m | 结果 |",
        "|---|---:|---:|---:|---:|---:|---|",
    ])
    for name, case in reports["lmpc_roll"]["cases"].items():
        lines.append(f"| {name} | {case['pitch_final_deg']:.6f} | {case['speed_final_m_s']:.6f} | {case['roll_rms_last_s_deg']:.6f} | {max(case['wheel_peak_nm']):.6f} | {max(case['joint_peak_nm']):.6f} | 通过 |")
    lines.extend([
        "", "组合扰动额外检查：横滚反向过冲 <0.3°、超过 ±0.2° 的过零不超过一次、2 s 内保持在 ±0.2° 内。", "",
        "![LMPC 与横滚验证曲线](figures/lmpc_roll_validation.png)", "",
        "完整参数与判据摘要：[validation_lmpc_roll.json](validation_lmpc_roll.json)。", "",
        "## 观察到的诊断与测试范围", "",
        "- 两套模型均报告零延时 Transport Delay 的直接馈通警告；本次测试完成且判据通过，警告保留。",
        "- LMPC 模型部分工况报告初始关节位置目标未完全满足。实际装配由 Simscape 求解，并非所有关节目标都能同时作为独立初值满足。",
        "- 曲线来自各自的物理参数；质量、轮半径和机身宽度不同，不能作为公平的算法性能排名。",
        "- 未测试遥控速度阶跃或 yaw 跟踪、外力时序扰动、载荷变化和地形。这里的恢复性能不能直接等同于实机性能。", "",
        "## 自行复现", "",
        "在仓库根目录执行 `run_validation('all')`。各控制器的 `results/` 会生成完整 MATLAB 数据、曲线和 JSON 摘要，重新运行会覆盖同名结果。", "",
        "发布包只附曲线和摘要；完整时序数据由测试生成，避免分发大体积派生文件。", "",
    ])
    (ROOT / "docs" / "validation.md").write_bytes("\n".join(lines).encode("utf-8"))


def main() -> None:
    figures = ROOT / "docs" / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    reports = {}
    for name, chart in [
        ("lqr", "lqr_validation.png"),
        ("lmpc_roll", "roll_comparison.png"),
    ]:
        results = ROOT / name / "results"
        report_path = results / "validation.json"
        report = json.loads(report_path.read_text(encoding="utf-8"))
        assert report["passed"], f"Validation did not pass: {name}"
        assert all(case["passed"] for case in report["cases"].values())
        reports[name] = report
        shutil.copy2(report_path, ROOT / "docs" / f"validation_{name}.json")
        shutil.copy2(results / chart, figures / f"{name}_validation.png")

    write_validation_document(reports)

    files = sorted(
        set(
            subprocess.check_output(
                ["git", "ls-files", "-c", "-o", "--exclude-standard", "-z"], cwd=ROOT
            )
            .decode("utf-8")
            .split("\0")
        )
        - {"", "SHA256SUMS.txt"}
    )
    forbidden = {".git", ".cache", "slprj", "results", "archive", "backups"}
    for name in files:
        path = Path(name)
        if any(part in forbidden for part in path.parts) and path.name != ".gitkeep":
            raise ValueError(f"Generated/private file in release: {name}")
        if name.endswith((".autosave", ".slxc", ".log", ".zip", ".bak", ".mat")):
            raise ValueError(f"Unexpected generated file: {name}")
        if path.suffix in {".m", ".md", ".json"}:
            text = (ROOT / path).read_text(encoding="utf-8-sig")
            if re.search(r"(?<![A-Za-z])[A-Za-z]:[\\/]", text):
                raise ValueError(f"Absolute local path in release: {name}")
        if path.suffix in {".m", ".md", ".json", ".txt"} or path.name in {
            "LICENSE", ".gitignore", ".gitattributes"
        }:
            # Match .gitattributes so ZIP and a Git checkout have the same bytes.
            text = (ROOT / path).read_text(encoding="utf-8-sig")
            (ROOT / path).write_bytes(text.replace("\r\n", "\n").replace("\r", "\n").encode("utf-8"))

    sums = "".join(
        f"{hashlib.sha256((ROOT / name).read_bytes()).hexdigest()}  {name}\n"
        for name in files
    )
    (ROOT / "SHA256SUMS.txt").write_bytes(sums.encode("utf-8"))
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(OUTPUT, "w", zipfile.ZIP_DEFLATED) as archive:
        for name in files + ["SHA256SUMS.txt"]:
            archive.write(ROOT / name, f"{ROOT.name}/{name}")
    with zipfile.ZipFile(OUTPUT) as archive:
        assert archive.testzip() is None
        for name in files:
            assert archive.read(f"{ROOT.name}/{name}") == (ROOT / name).read_bytes()
    digest = hashlib.sha256(OUTPUT.read_bytes()).hexdigest()
    OUTPUT.with_suffix(".zip.sha256").write_text(f"{digest}  {OUTPUT.name}\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "zip": str(OUTPUT),
                "files": len(files) + 1,
                "bytes": OUTPUT.stat().st_size,
                "sha256": digest,
                "validation_cases": {name: len(r["cases"]) for name, r in reports.items()},
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
