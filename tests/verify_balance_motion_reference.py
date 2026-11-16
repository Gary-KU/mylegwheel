"""Compile and exercise the feedback-gated reference manager; no hardware."""
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = r'''
#include "balance_motion_reference.h"
#include "wheel_leg_config.h"
#include <assert.h>
#include <stdio.h>

/* Keep this older envelope as independent generic-parameter coverage. The
 * actual deployed configuration is exercised separately below. */
static const BalanceMotionReferenceConfig production = {
    .max_speed_m_s = .4f,
    .acceleration_m_s2 = .4f,
    .brake_acceleration_m_s2 = .8f,
    .stop_speed_m_s = .035f,
    .stop_dwell_s = .15f,
    .tracking_window_m = .05f
};
static BalanceMotionReferenceState s;
static BalanceMotionReferenceOutput o;
static const float dt = .005f;

static void step(float x, float measured, float requested, float slewed) {
    float before = s.velocity_m_s;
    assert(BalanceMotionReference_Update(&s, &production, x, measured,
                                        requested, slewed, dt, &o));
    assert(isfinite(o.velocity_m_s) && isfinite(o.goal_m));
    assert(fabsf(o.velocity_m_s) <= production.max_speed_m_s + 1e-6f);
    assert(fabsf(o.velocity_m_s - before) <= production.brake_acceleration_m_s2 * dt + 1e-6f);
}
static void reset(void) {
    BalanceMotionReference_Reset(&s);
    step(2, 0, 0, 0);
    assert(s.phase == BALANCE_MOTION_PARKED && o.position_locked);
    assert(o.goal_m == 2 && o.velocity_m_s == 0 && !o.braking);
}
static void cruise(int sign) {
    reset();
    for (unsigned i = 0; i < 201; ++i) step(2, 0, sign * .4f, sign * .4f);
    assert(s.phase == BALANCE_MOTION_TRACKING && !o.position_locked);
    assert(fabsf(o.velocity_m_s - sign * .4f) < 1e-6f);
    assert(sign * (o.goal_m - 2) >= -1e-6f && fabsf(o.goal_m - 2) <= .050001f);
}
static void settle(float x, float requested, float slewed) {
    for (unsigned i = 0; i < 30; ++i) step(x, 0, requested, slewed);
}
static void zeroed(void) {
    assert(!s.initialized && s.velocity_m_s == 0 && s.goal_m == 0);
    assert(o.velocity_m_s == 0 && o.goal_m == 0 && !o.position_locked && !o.braking);
}

int main(void) {
    /* A neutral request is authoritative even while the upstream ramp is
     * still forward. The parking goal is never rewritten by coasting. */
    cruise(1);
    float goal = o.goal_m;
    for (unsigned i = 0; i < 101; ++i) {
        step(2.2f + i * .002f, .3f, 0, .4f);
        assert(o.braking && o.goal_m == goal && o.velocity_m_s >= 0);
    }
    assert(o.velocity_m_s == 0 && o.position_locked);
    for (unsigned i = 0; i < 300; ++i) {
        step(3, -.12f, 0, .4f);
        assert(o.braking && o.goal_m == goal && o.velocity_m_s == 0);
    }
    settle(3, 0, .4f);
    assert(s.phase == BALANCE_MOTION_PARKED && o.goal_m == goal);
    for (unsigned i = 0; i < 50; ++i) step(4, 0, 0, 0);
    assert(o.goal_m == goal && o.position_locked);
    puts("PASS: neutral intent overrides a stale nonzero ramp; coasting freezes the true goal");

    /* Both signs: an already reversed upstream target cannot release the
     * reverse direction while actual motion is still fast. */
    for (int sign = -1; sign <= 1; sign += 2) {
        cruise(sign); goal = o.goal_m;
        for (unsigned i = 0; i < 240; ++i) {
            step(2 + sign * .4f, sign * .28f, -sign * .4f, -sign * .4f);
            assert(o.braking && o.goal_m == goal && sign * o.velocity_m_s >= 0);
        }
        assert(o.velocity_m_s == 0);
        for (unsigned i = 0; i < 29; ++i) step(2, 0, -sign * .4f, -sign * .4f);
        assert(o.braking && o.velocity_m_s == 0);
        step(2, 0, -sign * .4f, -sign * .4f);
        assert(s.phase == BALANCE_MOTION_TRACKING && o.velocity_m_s == 0 && o.goal_m == goal);
        step(2, 0, -sign * .4f, -sign * .4f);
        assert(fabsf(o.velocity_m_s + sign * .002f) < 1e-7f);
    }
    puts("PASS: measured-speed dwell gates both reversal signs; completion keeps the goal and resumes from zero");

    /* Upstream may leap across zero without ever reporting exactly zero. */
    reset();
    step(2, 0, .4f, .003f);
    assert(o.velocity_m_s > 0);
    goal = o.goal_m;
    step(2, .1f, -.4f, -.017f);
    assert(o.braking && o.velocity_m_s == 0 && o.goal_m == goal);
    for (unsigned i = 0; i < 100; ++i) step(2, .1f, -.4f, -.4f);
    assert(o.braking && o.velocity_m_s == 0);
    puts("PASS: an upstream zero crossing does not substitute for a measured stop");

    /* A single speed excursion resets the continuous dwell. */
    for (unsigned i = 0; i < 29; ++i) step(2, .034f, -.4f, -.4f);
    assert(s.quiet_s > .14f && o.braking);
    step(2, -.036f, -.4f, -.4f);
    assert(s.quiet_s == 0 && o.braking);
    for (unsigned i = 0; i < 29; ++i) step(2, -.034f, -.4f, -.4f);
    assert(o.braking);
    step(2, .034f, -.4f, -.4f);
    assert(s.phase == BALANCE_MOTION_TRACKING && o.velocity_m_s == 0);
    puts("PASS: speed noise outside the threshold restarts the full dwell");

    /* Changing one's mind cannot bypass the brake; completion consumes the
     * latest intent, with no remembered reverse request replayed later. */
    cruise(1); goal = o.goal_m;
    step(2, .2f, -.4f, -.4f);
    for (unsigned i = 0; i < 140; ++i) {
        float request = i % 3 == 0 ? -.4f : i % 3 == 1 ? 0 : .4f;
        step(2.4f, .2f, request, request);
        assert(o.braking && o.goal_m == goal && o.velocity_m_s >= 0);
    }
    settle(2.4f, .4f, .4f);
    assert(s.phase == BALANCE_MOTION_TRACKING && o.velocity_m_s == 0 && o.goal_m == goal);
    step(2.4f, 0, .4f, .4f);
    assert(o.velocity_m_s > 0);
    cruise(1); step(2, .2f, -.4f, -.4f);
    for (unsigned i = 0; i < 110; ++i) step(2, .2f, 0, -.4f);
    settle(2, 0, -.4f);
    assert(s.phase == BALANCE_MOTION_PARKED && o.velocity_m_s == 0);
    for (unsigned i = 0; i < 50; ++i) step(2, 0, 0, -.4f);
    assert(s.phase == BALANCE_MOTION_PARKED && o.velocity_m_s == 0);
    puts("PASS: repeated reversals and cancellation use only current intent after stopping");

    /* A new drive from PARKED explicitly captures once. Starting while
     * already coasting still requires the measured-stop dwell. */
    goal = o.goal_m;
    step(6, .1f, .4f, .4f);
    assert(o.braking && o.goal_m == goal && o.velocity_m_s == 0);
    settle(6, .4f, .4f);
    assert(s.phase == BALANCE_MOTION_TRACKING && o.goal_m == goal);
    reset(); step(7, 0, .4f, .4f);
    assert(s.phase == BALANCE_MOTION_TRACKING && o.goal_m > 7 && o.goal_m < 7.001f);
    step(7.01f, 0, .4f, .4f);
    assert(o.goal_m < 7.001f); /* No per-frame recapture. */
    puts("PASS: parked restart captures once; a coasting restart cannot bypass braking");

    /* OFF/permission loss is the caller's explicit Reset, independent of
     * the motion phase. A later ON cannot retain velocity or an old goal. */
    BalanceMotionReference_Reset(&s);
    step(-4, 0, 0, 0);
    assert(o.position_locked && o.goal_m == -4 && o.velocity_m_s == 0);
    cruise(1); step(2, .2f, -.4f, -.4f);
    BalanceMotionReference_Reset(&s);
    step(-8, 0, 0, 0);
    assert(o.position_locked && o.goal_m == -8 && o.velocity_m_s == 0);
    BalanceMotionReference_Reset(NULL);
    puts("PASS: explicit OFF/permission Reset clears tracking, braking and old origins");

    /* A second configuration and nonuniform periods check the actual slew
     * bound rather than only one canned production trajectory. */
    BalanceMotionReferenceConfig alternate = production;
    alternate.max_speed_m_s = .3f; alternate.acceleration_m_s2 = .23f;
    alternate.brake_acceleration_m_s2 = .41f; alternate.stop_dwell_s = .113f;
    BalanceMotionReference_Reset(&s);
    for (unsigned i = 0; i < 80; ++i) {
        float period = i % 2 ? .003f : .009f;
        float before = s.velocity_m_s;
        assert(BalanceMotionReference_Update(&s, &alternate, 0, 0, .3f, .3f, period, &o));
        assert(o.velocity_m_s >= before && o.velocity_m_s - before <= .23f * period + 1e-7f);
    }
    for (unsigned i = 0; i < 100; ++i) {
        float before = s.velocity_m_s;
        assert(BalanceMotionReference_Update(&s, &alternate, 0, .2f, -.3f, -.3f, .007f, &o));
        assert(o.velocity_m_s >= 0 && before - o.velocity_m_s <= .41f * .007f + 1e-7f);
    }
    assert(o.braking && o.velocity_m_s == 0);
    puts("PASS: configurable acceleration/braking bounds hold with nonuniform periods");

    /* Use the real configuration macros as well as the independent fixture.
     * An excursion outside the speed threshold must restart the entire
     * dwell; the final qualifying tick retains zero and the frozen goal. */
    const BalanceMotionReferenceConfig deployed = {
        ROBOT_MOTION_DEFAULT_FORWARD_M_S, BALANCE_MOTION_ACCELERATION_M_S2,
        BALANCE_MOTION_BRAKE_ACCELERATION_M_S2, BALANCE_MOTION_STOP_SPEED_M_S,
        BALANCE_MOTION_STOP_DWELL_S, ROBOT_MOTION_TRACK_WINDOW_M
    };
    unsigned dwell_frames=(unsigned)ceilf(deployed.stop_dwell_s/dt-1e-5f);
    assert(dwell_frames>0);
    BalanceMotionReference_Reset(&s);
    for(unsigned n=0;n<250;++n)
        assert(BalanceMotionReference_Update(&s,&deployed,0,0,.4f,.4f,dt,&o));
    goal=o.goal_m;
    for(unsigned n=0;n<250;++n) {
        assert(BalanceMotionReference_Update(&s,&deployed,1,.25f,-.4f,-.4f,dt,&o));
        assert(o.braking && o.velocity_m_s>=0 && o.goal_m==goal);
    }
    assert(o.velocity_m_s==0);
    for(unsigned n=0;n+1<dwell_frames;++n) {
        assert(BalanceMotionReference_Update(&s,&deployed,1,
                    .95f*deployed.stop_speed_m_s,-.4f,-.4f,dt,&o));
        assert(o.braking && o.velocity_m_s==0 && o.goal_m==goal);
        assert(fabsf(s.quiet_s-(n+1)*dt)<1e-6f);
    }
    assert(BalanceMotionReference_Update(&s,&deployed,1,
               -1.05f*deployed.stop_speed_m_s,-.4f,-.4f,dt,&o));
    assert(o.braking && s.quiet_s==0);
    for(unsigned n=0;n+1<dwell_frames;++n) {
        assert(BalanceMotionReference_Update(&s,&deployed,1,
                    -.95f*deployed.stop_speed_m_s,-.4f,-.4f,dt,&o));
        assert(o.braking && o.velocity_m_s==0 && o.goal_m==goal);
    }
    assert(BalanceMotionReference_Update(&s,&deployed,1,0,-.4f,-.4f,dt,&o));
    assert(s.phase==BALANCE_MOTION_TRACKING && o.velocity_m_s==0 && o.goal_m==goal);
    assert(BalanceMotionReference_Update(&s,&deployed,1,0,-.4f,-.4f,dt,&o));
    assert(fabsf(o.velocity_m_s+deployed.acceleration_m_s2*dt)<1e-7f);
    printf("PASS: deployed %.3fm/s threshold and %.3fs dwell require continuous measured qualification\n",
           deployed.stop_speed_m_s,deployed.stop_dwell_s);

    const float bad_numbers[] = {NAN, INFINITY, -INFINITY};
    for (unsigned kind = 0; kind < 5; ++kind) {
        for (unsigned j = 0; j < 3; ++j) {
            reset();
            float values[] = {2, 0, 0, 0, dt}; values[kind] = bad_numbers[j];
            assert(!BalanceMotionReference_Update(&s, &production, values[0], values[1],
                                                 values[2], values[3], values[4], &o));
            zeroed();
        }
    }
    const float bad_periods[] = {0, -.005f, .01001f, 1};
    for (unsigned i = 0; i < 4; ++i) {
        reset();
        assert(!BalanceMotionReference_Update(&s, &production, 2, 0, 0, 0, bad_periods[i], &o));
        zeroed();
    }
    for (unsigned field = 0; field < 6; ++field) {
        for (unsigned kind = 0; kind < 3; ++kind) {
            BalanceMotionReferenceConfig c = production;
            float bad = kind == 0 ? NAN : kind == 1 ? 0 : 100;
            if (field == 0) c.max_speed_m_s = bad;
            if (field == 1) c.acceleration_m_s2 = bad;
            if (field == 2) c.brake_acceleration_m_s2 = bad;
            if (field == 3) c.stop_speed_m_s = bad;
            if (field == 4) c.stop_dwell_s = bad;
            if (field == 5) c.tracking_window_m = bad;
            reset();
            assert(!BalanceMotionReference_Update(&s, &c, 2, 0, 0, 0, dt, &o));
            zeroed();
        }
    }
    reset(); assert(!BalanceMotionReference_Update(&s, NULL, 2, 0, 0, 0, dt, &o)); zeroed();
    assert(!BalanceMotionReference_Update(NULL, &production, 2, 0, 0, 0, dt, &o));
    reset(); assert(!BalanceMotionReference_Update(&s, &production, 2, 0, 0, 0, dt, NULL));
    assert(!s.initialized);
    reset(); assert(!BalanceMotionReference_Update(&s, &production, 2, 0, .401f, 0, dt, &o)); zeroed();
    reset(); assert(!BalanceMotionReference_Update(&s, &production, 2, 0, 0, -.401f, dt, &o)); zeroed();
    reset(); s.velocity_m_s = NAN;
    assert(!BalanceMotionReference_Update(&s, &production, 2, 0, 0, 0, dt, &o)); zeroed();
    reset(); s.goal_m = INFINITY;
    assert(!BalanceMotionReference_Update(&s, &production, 2, 0, 0, 0, dt, &o)); zeroed();
    reset(); s.phase = (BalanceMotionReferencePhase)9;
    assert(!BalanceMotionReference_Update(&s, &production, 2, 0, 0, 0, dt, &o)); zeroed();
    puts("PASS: invalid configuration, periods, commands, pointers and state fail closed without NaN output");
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="balance_motion_reference_") as folder:
        tmp = Path(folder)
        source = tmp / "test.c"
        source.write_text(FIXTURE, encoding="utf-8")
        executable = tmp / "test.exe"
        subprocess.run(
            [
                "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "Control/WheelLeg/Inc"), str(source),
                "-I", str(ROOT / "App/Tasks/Inc"),
                "-lm", "-o", str(executable),
            ],
            check=True,
        )
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
