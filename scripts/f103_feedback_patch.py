"""Patch copied F103 velocity feedback to avoid delayed-angle differentiation."""
from pathlib import Path
import re

SPEED_FEEDBACK = r'''void Motor_SpeedCalcProcess()
{
    enum { WINDOW = 5 };
    static float angles[WINDOW];
    static uint32_t ticks[WINDOW];
    static unsigned count = 0, next = 0;
    float angle = shaft_angle;
    uint32_t now = HAL_GetTick();
    /* Match CAN angle feedback: use the current continuous shaft angle,
     * not the delayed Kalman angle. Five-ms differencing averages noise. */
    if (!isfinite(angle)) {
        count = next = 0;
        speed = 0;
        return;
    }
    if (count < WINDOW) {
        angles[next] = angle;
        ticks[next] = now;
        next = (next + 1U) % WINDOW;
        ++count;
        speed = 0;
        return;
    }
    uint32_t elapsed = now - ticks[next];
    if (!elapsed || elapsed > 10U) {
        count = next = 0;
        speed = 0;
        return;
    }
    float rpm = (angle - angles[next]) * 60000.0f / ((float)elapsed * _2PI);
    if (!isfinite(rpm)) {
        count = next = 0;
        speed = 0;
        return;
    }
    angles[next] = angle;
    ticks[next] = now;
    next = (next + 1U) % WINDOW;
    speed = rpm;
}
'''



def configure_feedback_speed(source: Path) -> None:
    main = source / "Src/main.c"
    text = main.read_text(encoding="utf-8-sig")
    text, count = re.subn(r"void Motor_SpeedCalcProcess\(\)\s*\{.*?\n\}",
                         lambda _: SPEED_FEEDBACK, text, count=1, flags=re.S)
    if count != 1:
        raise ValueError("Unexpected speed feedback layout; refusing to build")
    if "#include <math.h>" not in text:
        text = text.replace("#include <stdio.h>", "#include <stdio.h>\n#include <math.h>", 1)
    main.write_text(text, encoding="utf-8")
