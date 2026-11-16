"""Check source registration, exact path spelling and matching Keil options."""
from pathlib import Path
import json
import re
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
project_dir = root / "MDK-ARM"
project = ET.parse(project_dir / "MyLegwheel.uvprojx")
target = project.find("./Targets/Target")
groups = target.findall("./Groups/Group")
paths = []


def exact_path(path):
    path = Path(path)
    assert path.exists(), f"Missing path: {path}"
    if path.is_relative_to(root):
        parent = root
        for part in path.relative_to(root).parts:
            assert part in {child.name for child in parent.iterdir()}, f"Path case mismatch: {path}"
            parent /= part
    return path


for group in groups:
    for file in group.findall("./Files/File"):
        value = file.findtext("FilePath").replace("\\", "/")
        path = exact_path((project_dir / value).resolve())
        assert path.name == file.findtext("FileName"), f"FileName mismatch: {value}"
        paths.append(path)
assert len(paths) == len(set(paths)), "Duplicate project registration"

custom_roots = ("Core", "App", "BSP", "Algorithms", "Control", "USB_DEVICE")
for directory in custom_roots:
    for file in (root / directory).rglob("*.c"):
        assert file in paths, f"Unregistered application source: {file}"
for directory in ("App", "BSP", "Algorithms", "Control"):
    for file in (root / directory).rglob("*"):
        if file.suffix in (".c", ".h"):
            assert re.fullmatch(r"[a-z][a-z0-9_]*\.[ch]", file.name), f"Nonstandard file name: {file}"
            if file.suffix == ".h":
                text = file.read_bytes().decode("latin-1")
                guard = re.search(r"^#ifndef ([A-Z][A-Z0-9_]+_H)\s*$", text, re.M)
                assert guard and "#define " + guard[1] in text, f"Nonstandard header guard: {file}"

include_paths = target.findtext("./TargetOption/TargetArmAds/Cads/VariousControls/IncludePath").split(";")
for include in include_paths:
    exact_path((project_dir / include.replace("\\", "/")).resolve())
scatter = target.findtext("./TargetOption/TargetArmAds/LDads/ScatterFile")
exact_path((project_dir / scatter).resolve())
for field in ("OutputDirectory", "ListingPath"):
    path = target.findtext("./TargetOption/TargetCommonOption/" + field).replace("\\", "/")
    assert (project_dir / path).resolve().is_relative_to(root / "build"), field

options = ET.parse(project_dir / "MyLegwheel.uvoptx")
option_groups = options.findall("./Group")
assert [g.findtext("GroupName") for g in groups] == [g.findtext("GroupName") for g in option_groups]
for definition, option in zip(groups, option_groups):
    assert [f.findtext("FilePath") for f in definition.findall("./Files/File")] == [
        f.findtext("PathWithFileName") for f in option.findall("./File")], definition.findtext("GroupName")
for file in (root / ".vscode").glob("*.json"):
    json.loads(file.read_text(encoding="utf-8"))
print(f"PASS: {len(groups)} Keil groups, {len(paths)} registered files, include paths, naming/guards, options and editor JSON")
