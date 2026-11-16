"""Host-check actual LED encoding/status task with mocked time and outputs.

Requires Python and gcc. Hardware waveform and acoustic checks remain on-board.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "App/Services/Src/board_feedback.c").read_text(encoding="utf-8")


def function(name):
    start = re.search(r"static (?:void|BoardFeedbackStatus) " + name + r"\(", source).start()
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


constants = "\n".join(re.findall(r"^#define (?:LED_\w+|FEEDBACK_PERIOD_MS|IMU_STALE_MS).*", source, re.M))
notes = re.search(r"static const StartupNote startup_notes\[\] = \{.*?\};", source, re.S)[0]
inputs = re.search(r"typedef struct \{\s*uint32_t remote_enable.*?\} FeedbackInputs;", source, re.S)[0]
harness = r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <setjmp.h>
#include "board_feedback.h"
#include "wheel_leg_task.h"
typedef uint32_t TickType_t;
typedef struct { uint16_t frequency_hz, duration_ms; } StartupNote;
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define portTICK_PERIOD_MS 1U
CONSTANTS
NOTES
INPUTS
volatile BoardFeedbackDebug g_board_feedback;
volatile RobotControlDebug g_robot_control;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint8_t led_frame[LED_BUFFER_BYTES];
static uint32_t imu_status, imu_last_sample_tick;
static uint32_t now_tick, boot_tick, iteration, note_count, released, stalled;
static uint32_t alarms;
static uint16_t frequencies[32];
static uint32_t note_times[32];
static jmp_buf task_end;
static TickType_t xTaskGetTickCount(void) { return now_tick; }
static void Buzzer_SetFrequency(uint16_t hz) {
    assert(note_count < 32);
    frequencies[note_count] = hz;
    note_times[note_count++] = now_tick - boot_tick;
}
static void IMU_Task_Start(void) {
    /* First note must be running before sensor initialization is released. */
    assert(note_count == 1 && frequencies[0] != 0);
    ++released;
}
static void LED_Set(uint8_t r, uint8_t g, uint8_t b, TickType_t now) {
    uint32_t ms = now - boot_tick;
    if (alarms) {
        if (ms==160) assert(r>0 && g>0 && b==0 && g_board_feedback.status==BOARD_FEEDBACK_REMOTE_WAIT);
        if (ms==1800) {
            assert(b>0 && r==0 && g==0 && g_board_feedback.status==BOARD_FEEDBACK_PREPARING);
            assert(g_board_feedback.ready_missing_mask==(ROBOT_READY_GAP|ROBOT_READY_LENGTH|ROBOT_READY_POSE));
        }
        if (ms==4000) {
            assert(r>0 && g==0 && b==0 && g_board_feedback.status==BOARD_FEEDBACK_FAULT);
            assert(g_board_feedback.ready_missing_mask==0);
        }
        if (ms==6500) assert(g>0 && r==0 && b==0 && g_board_feedback.status==BOARD_FEEDBACK_BALANCING);
        if (ms==7200) assert(g_board_feedback.status==BOARD_FEEDBACK_IDLE && g_board_feedback.frequency_hz==0);
        return;
    }
    if (ms == 0) assert(b > 0 && r == 0 && g == 0);
    if (ms == 180 || ms == 640) assert(g > 0 && r == 0 && b == 0);
    if (ms == 500 || ms == 1040) assert(r > 0 && g == 0 && b == 0);
}
static void vTaskDelayUntil(TickType_t *last, TickType_t interval) {
    *last += interval;
    now_tick = *last;
    if (stalled && iteration == 8) {
        now_tick += 1000U;
        *last = now_tick;
    }
    if (alarms) {
        if (++iteration>480) longjmp(task_end,1);
        uint32_t ms=now_tick-boot_tick;
        imu_status=ms<160 ? 0xFFU : 0U;imu_last_sample_tick=now_tick;
        g_robot_control=(RobotControlDebug){.remote_enable=1,.remote_status=3};
        if (ms>=1800 && ms<4000) {
            g_robot_control.enable=1;g_robot_control.status=1;g_robot_control.remote_status=4;
            g_robot_control.startup_phase=ROBOT_STARTUP_PREPARE;
            g_robot_control.startup_ready_mask=ROBOT_READY_SPEED|ROBOT_READY_SUPPORT;
        } else if (ms>=4000 && ms<6500) g_robot_control.status=2;
        else if (ms>=6500 && ms<7200) {
            g_robot_control.enable=1;g_robot_control.status=1;g_robot_control.remote_status=4;
            g_robot_control.startup_phase=ROBOT_STARTUP_BALANCE;
        } else if (ms>=7200 && ms<7800) g_robot_control.remote_status=2;
        if (ms>=8900) g_board_feedback.buzzer_enable=0;
        return;
    }
    if (++iteration > 120) longjmp(task_end, 1);
    uint32_t ms = now_tick - boot_tick;
    if (ms < 160) imu_status = 0xFFU;
    else if (ms < 400) { imu_status = 0; imu_last_sample_tick = now_tick; }
    else if (ms < 600) imu_status = 0x100U;
    else if (ms < 1000) { imu_status = 0; imu_last_sample_tick = now_tick; }
    else { imu_status = 0; imu_last_sample_tick = boot_tick + 900U; }
}
ENCODE
READ
CLASSIFY
PATTERN
TASK
static unsigned frame_bit(unsigned bit) {
    return (led_frame[bit / 8] >> (7 - bit % 8)) & 1;
}
static void check_frame(uint8_t r, uint8_t g, uint8_t b) {
    unsigned bit, symbol_bit;
    uint32_t expected = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    LED_Encode(r, g, b);
    for (bit = 0; bit < LED_RESET_BYTES * 8; ++bit) assert(frame_bit(bit) == 0);
    for (bit = 0; bit < 24; ++bit) {
        unsigned high_count = (expected >> (23 - bit)) & 1 ? 6 : 3;
        for (symbol_bit = 0; symbol_bit < 10; ++symbol_bit) {
            assert(frame_bit(LED_RESET_BYTES * 8 + bit * 10 + symbol_bit) ==
                   (symbol_bit < high_count));
        }
    }
    for (bit = (LED_RESET_BYTES + LED_DATA_BYTES) * 8;
         bit < sizeof(led_frame) * 8; ++bit) assert(frame_bit(bit) == 0);
}
static void check_task(uint32_t start, uint32_t stall) {
    static const uint16_t expected_hz[] = {1047, 0, 1319, 0, 1568, 0};
    static const uint32_t expected_ms[] = {0, 140, 180, 320, 360, 620};
    now_tick = boot_tick = start;
    iteration = note_count = released = 0;
    stalled = stall;
    alarms=0;g_board_feedback=(BoardFeedbackDebug){0};g_robot_control=(RobotControlDebug){0};
    imu_status = 0xFFU;
    if (setjmp(task_end) == 0) BoardFeedback_Task(NULL);
    assert(released == 1 && note_count == 6);
    for (unsigned i = 0; i < 6; ++i) {
        assert(frequencies[i] == expected_hz[i]);
        if (!stall) assert(note_times[i] == expected_ms[i]);
    }
    assert(frequencies[note_count - 1] == 0); /* silent; no restart */
}
static void check_classify(void) {
    FeedbackInputs in={.imu_fresh=1,.remote_enable=1,.remote_status=2};
    assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_IDLE);
    in.remote_status=3;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_REMOTE_WAIT);
    in.remote_status=1;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_REMOTE_WAIT);
    in.remote_status=4;in.enable=1;in.phase=ROBOT_STARTUP_PREPARE;
    assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_PREPARING);
    in.phase=ROBOT_STARTUP_RAMP;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_PREPARING);
    in.phase=ROBOT_STARTUP_BALANCE;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_BALANCING);
    for(unsigned s=2;s<=5;s++) {in.status=s;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_FAULT);}
    in.status=0;in.remote_status=5;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_FAULT);
    in.remote_enable=0;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_BALANCING);
    in.imu_fresh=0;assert(Feedback_Classify(&in,10000)==BOARD_FEEDBACK_FAULT);
    in.imu_status=0xFF;assert(Feedback_Classify(&in,4999)==BOARD_FEEDBACK_BOOTING);
    assert(Feedback_Classify(&in,5000)==BOARD_FEEDBACK_FAULT);
    in.imu_status=0x100;assert(Feedback_Classify(&in,100)==BOARD_FEEDBACK_FAULT);
}
static void check_patterns(void) {
    for(unsigned count=1;count<=3;count++) {
        unsigned led_pulses=0,beep_pulses=0,previous_led=0,previous_beep=0;
        for(unsigned ms=0;ms<4000;ms+=20) {
            uint8_t r,g,b;uint16_t hz;
            Feedback_Pattern((BoardFeedbackStatus)count,ms,1,&r,&g,&b,&hz);
            if(ms<2000) {
                unsigned lit=r||g||b;led_pulses+=lit&&!previous_led;previous_led=lit;
                if(lit) assert(count==1 ? r&&g&&!b : count==2 ? b&&!r&&!g : r&&!g&&!b);
            }
            beep_pulses+=(hz!=0)&&!previous_beep;previous_beep=hz!=0;
            if(ms<1000) assert(hz==0);
            Feedback_Pattern((BoardFeedbackStatus)count,ms,0,&r,&g,&b,&hz);assert(hz==0);
        }
        assert(led_pulses==count && beep_pulses==count);
    }
}
static void check_alarms(uint32_t start) {
    static const uint16_t expected_hz[]={1047,0,1319,0,1568,0,2093,0,2093,0,2093,0,1047,0,1047,0,1047,0,2093,0};
    static const uint32_t expected_ms[]={0,140,180,320,360,620,1160,1280,2800,2920,3040,3160,5000,5120,5240,5360,5480,5600,8800,8900};
    now_tick=boot_tick=start;iteration=note_count=released=stalled=0;alarms=1;
    imu_status=0xFF;g_board_feedback=(BoardFeedbackDebug){.buzzer_enable=1};
    g_robot_control=(RobotControlDebug){.remote_enable=1,.remote_status=3};
    if(setjmp(task_end)==0) BoardFeedback_Task(NULL);
    assert(released==1 && note_count==sizeof(expected_hz)/sizeof(expected_hz[0]));
    for(unsigned n=0;n<note_count;n++) {assert(frequencies[n]==expected_hz[n]);assert(note_times[n]==expected_ms[n]);}
    assert(g_board_feedback.frequency_hz==0);
}
int main(void) {
    for (unsigned i = 0; i < 256; ++i) {
        check_frame(i, 0, 0); check_frame(0, i, 0); check_frame(0, 0, i);
    }
    check_frame(0xA5, 0x5A, 0x81);
    check_frame(0xFF, 0xFF, 0xFF);
    check_task(0, 0);
    check_task(0xFFFFFFE0U, 0); /* Tick wrap during first note. */
    check_task(0, 1); /* Task preemption beyond end of startup melody. */
    check_classify();check_patterns();check_alarms(0);check_alarms(0xFFFFF000U);
    puts("PASS: GRB/reset, startup melody, coherent status, 1/2/3 LED/beep groups, priorities, missing readiness, recovery/OFF silence, mute, IMU timeout, tick wrap and preemption");
    return 0;
}
'''
harness = harness.replace("CONSTANTS", constants).replace("NOTES", notes).replace("INPUTS", inputs)
for marker, name in [("ENCODE", "LED_Encode"), ("READ", "Feedback_ReadInputs"),
                     ("CLASSIFY", "Feedback_Classify"), ("PATTERN", "Feedback_Pattern"), ("TASK", "BoardFeedback_Task")]:
    harness = re.sub(r"^" + marker + r"$", lambda _: function(name), harness, flags=re.M)
with tempfile.TemporaryDirectory(prefix="board_feedback_test_") as folder:
    temp = Path(folder)
    test_c = temp / "feedback.c"
    test_c.write_text(harness)
    exe = temp / "verify.exe"
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "App/Services/Inc"), "-I", str(root / "App/Tasks/Inc"),
                    "-I", str(root / "Control/WheelLeg/Inc"), "-I", str(root / "App/Protocol/Inc"),
                    str(test_c), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
