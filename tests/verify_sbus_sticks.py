"""Verify signed stick shaping and explicit calibration using actual C code."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
harness = r'''
#include "sbus_sticks.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const uint16_t minima[4] = {306, 289, 350, 320};
static const uint16_t centers[4] = {1008, 1017, 1032, 998};
static const uint16_t maxima[4] = {1694, 1710, 1650, 1701};
static void frame_at(SbusFrame *f, const uint16_t values[4]) {
    memset(f, 0, sizeof(*f));
    for (unsigned i = 0; i < 4; ++i) f->channel[i] = values[i];
}
static void sweep(SbusStickCalibration *s, SbusAxisCalibration *c, uint32_t now) {
    SbusFrame f; frame_at(&f, minima);
    SBUS_StickCalibrationFeed(s, c, &f, now, 1);
    frame_at(&f, maxima); SBUS_StickCalibrationFeed(s, c, &f, now + 7, 0);
    assert(s->stage == SBUS_CAL_RANGE && s->samples == 2);
}
static void neutral(SbusStickCalibration *s, SbusAxisCalibration *c, uint32_t now, uint8_t command) {
    SbusFrame f;
    for (unsigned n = 0; n < 101; ++n) {
        frame_at(&f, centers);
        for (unsigned i = 0; i < 4; ++i) f.channel[i] += n % 3 - 1;
        SBUS_StickCalibrationFeed(s, c, &f, now + n * 7, n == 0 ? command : 0);
    }
}
int main(void) {
    SbusAxisCalibration c[4]; SbusStickCalibration s = {0}; SbusFrame f;
    for (unsigned i = 0; i < 4; ++i) {
        c[i] = (SbusAxisCalibration){minima[i], centers[i], maxima[i], 30, 1};
        float previous = -1.0f;
        for (unsigned raw = 0; raw <= 2047; ++raw) {
            float actual = SBUS_NormalizeAxis(raw, &c[i]);
            assert(isfinite(actual) && actual >= -1 && actual <= 1 && actual >= previous);
            int d = (int)raw - centers[i];
            double expected = d > 30 ? (double)(d - 30) / (maxima[i] - centers[i] - 30) :
                d < -30 ? (double)(d + 30) / (centers[i] - minima[i] - 30) : 0;
            if (expected > 1) expected = 1;
            if (expected < -1) expected = -1;
            assert(fabs(actual - expected) < 1e-7); previous = actual;
        }
        assert(SBUS_NormalizeAxis(minima[i], &c[i]) == -1);
        assert(SBUS_NormalizeAxis(maxima[i], &c[i]) == 1);
        for (unsigned raw = centers[i] - 30U; raw <= centers[i] + 30U; ++raw)
            assert(SBUS_NormalizeAxis(raw, &c[i]) == 0);
        assert(SBUS_NormalizeAxis(centers[i] - 31, &c[i]) < 0);
        assert(SBUS_NormalizeAxis(centers[i] + 31, &c[i]) > 0);
        c[i].direction = -1;
        assert(SBUS_NormalizeAxis(minima[i], &c[i]) == 1 && SBUS_NormalizeAxis(maxima[i], &c[i]) == -1);
        c[i].direction = 1;
    }
    SbusAxisCalibration bad = c[0];
    assert(SBUS_NormalizeAxis(1000, NULL) == 0 && SBUS_NormalizeAxis(65535, &bad) == 0);
    bad.center = bad.minimum; assert(!SBUS_AxisCalibrationValid(&bad) && SBUS_NormalizeAxis(0, &bad) == 0);
    bad = c[0]; bad.deadband = bad.maximum - bad.center; assert(!SBUS_AxisCalibrationValid(&bad));
    bad = c[0]; bad.maximum = 2048; assert(!SBUS_AxisCalibrationValid(&bad));
    bad = c[0]; bad.direction = 0; assert(!SBUS_AxisCalibrationValid(&bad));

    sweep(&s, c, 0); neutral(&s, c, 14, 2);
    assert(s.stage == SBUS_CAL_DONE && s.calibrated);
    for (unsigned i = 0; i < 4; ++i) {
        assert(c[i].minimum == minima[i] && c[i].maximum == maxima[i] && c[i].center == centers[i]);
        assert(c[i].deadband == 30 && s.measured_center[i] == centers[i]);
    }
    /* No automatic learning after calibration; driving cannot drag zero. */
    frame_at(&f, maxima);
    for (unsigned n = 0; n < 100; ++n) SBUS_StickCalibrationFeed(&s, c, &f, 1000 + n * 7, 0);
    assert(c[0].center == centers[0]);
    sweep(&s, c, UINT32_MAX - 200); neutral(&s, c, UINT32_MAX - 186, 2);
    assert(s.stage == SBUS_CAL_DONE); /* unsigned timestamp wrap */

    sweep(&s, c, 3000); frame_at(&f, centers);
    SBUS_StickCalibrationFeed(&s, c, &f, 3014, 2);
    for (unsigned n = 1; n < 10; ++n) SBUS_StickCalibrationFeed(&s, c, &f, 3014 + n * 7, 0);
    f.channel[0] += 100; SBUS_StickCalibrationFeed(&s, c, &f, 3084, 0);
    assert(s.stage == SBUS_CAL_CENTER && s.center_samples == 1);
    f.failsafe = 1; SBUS_StickCalibrationFeed(&s, c, &f, 3091, 0); assert(s.center_samples == 0);
    f.failsafe = 0; SBUS_StickCalibrationFeed(&s, c, &f, 4000, 0); assert(s.center_samples == 1);
    SBUS_StickCalibrationFeed(&s, NULL, NULL, 4007, 0); assert(s.center_samples == 0);
    SBUS_StickCalibrationFeed(&s, c, NULL, 4014, 3); assert(s.stage == SBUS_CAL_DONE);

    /* Only a center was swept: reject without replacing known calibration. */
    SbusAxisCalibration before[4]; memcpy(before, c, sizeof(c));
    frame_at(&f, centers); SBUS_StickCalibrationFeed(&s, c, &f, 5000, 1);
    SBUS_StickCalibrationFeed(&s, c, &f, 5007, 0); neutral(&s, c, 5014, 2);
    assert(s.stage == SBUS_CAL_FAILED && !memcmp(before, c, sizeof(c)));
    /* A deflected endpoint cannot be captured as neutral. */
    sweep(&s, c, 6000); frame_at(&f, maxima);
    SBUS_StickCalibrationFeed(&s, c, &f, 6014, 2);
    for (unsigned n = 1; n < 101; ++n) SBUS_StickCalibrationFeed(&s, c, &f, 6014 + n * 7, 0);
    assert(s.stage == SBUS_CAL_FAILED && !memcmp(before, c, sizeof(c)));
    puts("PASS: all 2048 values on four unequal axes, signed/continuous deadband, saturation, direction, invalid config, explicit range/neutral calibration, motion/failsafe/gap/wrap and rejected incomplete sweeps");
}
'''
with tempfile.TemporaryDirectory(prefix="sbus_sticks_") as directory:
    work = Path(directory)
    test = work / "test_sticks.c"
    test.write_text(harness, encoding="utf-8")
    exe = work / "test_sticks.exe"
    subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "App/Protocol/Inc"), str(test),
                    str(root / "App/Protocol/Src/sbus_sticks.c"), "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
