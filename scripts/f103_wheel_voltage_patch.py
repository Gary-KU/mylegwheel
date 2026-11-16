"""Keep generated F103 wheel voltage configuration aligned with the H723."""
from pathlib import Path
import re
from f103_feedback_patch import configure_feedback_speed


def configure_wheel_voltage(source: Path, host_config: Path):
    config = host_config.read_text(encoding="utf-8-sig")

    def value(name):
        matches = re.findall(r"^#define\s+" + name + r"\s+(\d+(?:\.\d+)?)f\s*$", config, re.M)
        if len(matches) != 1:
            raise ValueError(f"Expected one numeric {name} in {host_config}")
        return float(matches[0])

    limit_v = value("WHEEL_VOLTAGE_LIMIT_MV") / 1000
    supply_v = value("WHEEL_SUPPLY_VOLTAGE_V")
    if not (0 < limit_v <= supply_v / 2):
        raise ValueError("Wheel limit must be positive and at most half the configured supply")

    def replace_once(path, pattern, replacement):
        text = path.read_text(encoding="utf-8-sig")
        text, count = re.subn(pattern, replacement, text, flags=re.S)
        if count != 1:
            raise ValueError(f"Expected one voltage configuration match in {path}, found {count}")
        path.write_text(text, encoding="utf-8")

    profile = source / "USER/motor_config.h"
    replace_once(profile, r"(#define\s+MOTOR_TYPE\s+)MOTOR_JOINT\b", r"\g<1>MOTOR_WHEEL")
    replace_once(profile, r"(#if MOTOR_TYPE == MOTOR_WHEEL\b.*?#define\s+MOTOR_MAX_VOLT\s+)\d+(?:\.\d+)?f",
                 rf"\g<1>{limit_v:.1f}f")
    # Preserve the 6.9 V joint branch; only annotate the wheel profile.
    text = profile.read_text(encoding="utf-8")
    text = re.sub(r"(MOTOR_WHEEL :[^\n]*?限压 )\d+(?:\.\d+)?V", rf"\g<1>{limit_v:.1f}V", text)
    profile.write_text(text, encoding="utf-8")
    replace_once(source / "Src/main.c", r"(#define\s+VOLT_SUPPLY\s+)\d+(?:\.\d+)?(?:f)?",
                 rf"\g<1>{supply_v:.1f}f")
    # The upstream CAN path calls this setter without enforcing voltage_limit.
    replace_once(source / "USER/BLDCMotor.c",
                 r"(void setTargetVotage\(float new_target\)\s*\{\s*)voltage\.q = new_target;",
                 r"\g<1>/* Explicit wheel CAN clamp; NaN commands stop the output. */\n"
                 r"\tvoltage.q = new_target != new_target ? 0.0f :\n"
                 r"\t\t_constrain(new_target, -MOTOR_MAX_VOLT, MOTOR_MAX_VOLT);")
    configure_feedback_speed(source)
    return limit_v, supply_v
