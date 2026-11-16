"""Apply a quiet, stored-calibration-only boot policy to copied F103 joints."""
from pathlib import Path
import re
from f103_feedback_patch import configure_feedback_speed


JOINT_FOC_INIT = r'''void Motor_initFOC(void)
{
    int saved_poles = 0, saved_direction = 0;
    float saved_zero = 0.0f;
    pole_pairs = MOTOR_POLE_PAIRS;
    sensor_direction = CW;
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    M1_Disable;
    /* No boot alignment: assembled legs must not move before host enable.
     * Never drive with a missing, nonfinite or mismatched saved zero. */
    if (Flash_ReadMotorParam(&saved_poles, &saved_zero, &saved_direction) != 0 ||
        saved_poles != MOTOR_POLE_PAIRS || saved_direction != CW ||
        !(saved_zero >= 0.0f && saved_zero < _2PI))
    {
        printf("MOT: Invalid stored calibration; driver disabled.\r\n");
        for (;;)
        {
            setPhaseVoltage(0.0f, 0.0f, 0.0f);
            M1_Disable;
            HAL_Delay(100);
        }
    }
    zero_electric_angle = saved_zero;
    angle_prev = getAngle();
    HAL_Delay(5);
    shaft_angle = shaftAngle();
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    M1_Enable;
}
'''


def configure_joint_boot(source: Path) -> None:
    """Patch the local build copy; leave the shared original project intact."""
    # Joint bus voltage/ceiling now come from the same tuning file as H7.
    config = (Path(__file__).resolve().parents[1] /
              "App/Tasks/Inc/wheel_leg_config.h").read_text(encoding="utf-8-sig")
    def value(name):
        matches = re.findall(r"^#define\s+" + name + r"\s+(\d+(?:\.\d+)?)f\s*$", config, re.M)
        if len(matches) != 1:
            raise ValueError(f"Expected one numeric {name}")
        return float(matches[0])
    supply_v = value("JOINT_DRIVER_SUPPLY_VOLTAGE_V")
    limit_v = value("JOINT_VOLTAGE_LIMIT_MV") / 1000.0
    if not (0 < limit_v <= supply_v):
        raise ValueError("Joint voltage ceiling must be positive and no greater than its supply")
    profile = source / "USER/motor_config.h"
    text = profile.read_text(encoding="utf-8-sig")
    limit_literal = f"{limit_v:.9g}"
    if "." not in limit_literal:
        limit_literal += ".0"
    text, count = re.subn(r"(#else\s*#define\s+MOTOR_POLE_PAIRS\s+11\s*"
                         r"#define\s+MOTOR_MAX_VOLT\s+)\d+(?:\.\d+)?f",
                         rf"\g<1>{limit_literal}f", text)
    if count != 1:
        raise ValueError("Unexpected joint voltage profile; refusing to build")
    profile.write_text(text, encoding="utf-8")
    motor = source / "USER/BLDCMotor.c"
    text = motor.read_text(encoding="utf-8-sig")
    pattern = r"void Motor_initFOC\(void\)\s*\{.*?\n\}\s*(?=/\*+)"
    text, count = re.subn(pattern, lambda _: JOINT_FOC_INIT, text, count=1, flags=re.S)
    if count != 1:
        raise ValueError("Unexpected Motor_initFOC layout; refusing to build")
    old = 'M1_Enable;\n\tprintf("MOT: Enable driver.\\r\\n");'
    if old not in text:
        raise ValueError("Unexpected Motor_init gate; refusing to build")
    text = text.replace(old, 'M1_Disable;\n\tprintf("MOT: Driver waiting for stored calibration.\\r\\n");', 1)
    motor.write_text(text, encoding="utf-8")
    main = source / "Src/main.c"
    text = main.read_text(encoding="utf-8-sig")
    text, count = re.subn(r"(#define\s+VOLT_SUPPLY\s+)\d+(?:\.\d+)?f?",
                         rf"\g<1>{supply_v:.1f}f", text)
    if count != 1:
        raise ValueError("Unexpected joint supply definition; refusing to build")
    pattern = r"Beep_PlayNotes\(3,\(uint16_t\[\]\[2\]\).*?\);[^\n]*"
    text, count = re.subn(pattern, "/* No motor-field boot melody on assembled joints. */", text, count=1)
    if count != 1:
        raise ValueError("Unexpected startup melody; refusing to build")
    main.write_text(text, encoding="utf-8")
    configure_feedback_speed(source)
