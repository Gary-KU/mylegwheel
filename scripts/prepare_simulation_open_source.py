"""Stage an allowlisted simulation release without changing the source projects."""
from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
DESTINATION = WORKSPACE / "projects" / "wheel-legged-simulation"
SOURCES = {
    "lqr": (
        Path(r"D:\MATLAB\Robot\WHEEL_LEG\MyProc\Total1450g"),
        ["Self_Model_3D.slx", "init_lqr_model.m", "lqr_model_parameters.m"],
    ),
    "lmpc_roll": (
        Path(r"D:\MATLAB\Robot\WHEEL_LEG\MyProc\LMPC_Roll"),
        [
            "Self_Model_3D_LMPC_Roll.slx",
            "init_roll_lmpc.m",
            "lmpc_parameters.m",
            "roll_parameters.m",
            "tests/test_roll_lmpc.m",
            "tests/test_roll_smooth.m",
        ],
    ),
}


def main() -> None:
    manifest = {"snapshot_date": "2026-10-11", "files": []}
    for controller, (source, names) in SOURCES.items():
        for name in names:
            original = source / name
            destination = DESTINATION / controller / name
            if destination.exists():
                raise FileExistsError(f"Refusing to overwrite staged work: {destination}")
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, destination)
            manifest["files"].append(
                {
                    "path": destination.relative_to(DESTINATION).as_posix(),
                    "source_sha256": hashlib.sha256(original.read_bytes()).hexdigest(),
                    "source_saved_at": original.stat().st_mtime,
                }
            )
    documentation = DESTINATION / "docs"
    documentation.mkdir(parents=True, exist_ok=True)
    (documentation / "source_snapshot.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"Staged {len(manifest['files'])} source files at {DESTINATION}")


if __name__ == "__main__":
    main()
