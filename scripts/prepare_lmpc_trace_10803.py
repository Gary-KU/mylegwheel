"""Stage longer observation only; preserve all 10802 control parameters."""
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent / 'H723_LMPC'
STAGE = ROOT / 'build/lmpc_trace_10803'
if STAGE.exists():
    raise SystemExit('Stage already exists')
shutil.copytree(SOURCE, STAGE, ignore=shutil.ignore_patterns('build', 'backups', '__pycache__', 'sim_cache', '*.uvguix.*'))

def edit(name, pairs):
    p = STAGE / name
    s = p.read_text(encoding='utf-8')
    for old, new in pairs:
        assert s.count(old) == 1, old
        s = s.replace(old, new)
    p.write_text(s, encoding='utf-8')

edit('App/Tasks/Inc/wheel_leg_task.h', [
    ('/* Latest 5.12 s at 50 Hz, frozen on disarm;', '/* Latest 51.2 s at 10 Hz, frozen on disarm;'),
    ('* bit2 wheel Nm saturation, bit3 Tp at cap; phase in bits8..15. */',
     '* bit2 wheel Nm saturation, bit3 Tp at cap; phase in bits8..15.\n'
     '                            * bit16 learning; gate mask in bits24..31. */'),
    ('float motion_component_nm[2]; /* wheel dtheta and dphi contributions. */',
     'float parking_lean_rad, parking_filtered_speed_m_s; /* Trace format 10803. */'),
    ('#define WHEEL_BALANCE_TRACE_ROWS 256U', '#define WHEEL_BALANCE_TRACE_ROWS 512U\n#define WHEEL_BALANCE_TRACE_PERIOD_MS 100U'),
])
edit('App/Tasks/Src/wheel_leg_task.c', [
    ('last_tick=state->tick_ms-20U;', 'last_tick=state->tick_ms-WHEEL_BALANCE_TRACE_PERIOD_MS;'),
    ('(state->tick_ms-last_tick)<20U', '(state->tick_ms-last_tick)<WHEEL_BALANCE_TRACE_PERIOD_MS'),
    ('((uint32_t)g_robot_control.startup_phase<<8);',
     '((uint32_t)g_robot_control.startup_phase<<8) |\n'
     '              (g_balance.parking_lean_learning?65536U:0U) |\n'
     '              ((g_parking_learning_debug.gate_mask&255U)<<24);'),
    ('row.motion_component_nm[0]=g_balance.wheel_torque_component_nm[1];\n    row.motion_component_nm[1]=g_balance.wheel_torque_component_nm[5];',
     'row.parking_lean_rad=g_balance.parking_lean_rad;\n    row.parking_filtered_speed_m_s=g_balance.parking_lean_filtered_speed_m_s;'),
])
p=STAGE/'App/Services/Src/robot_telemetry_service.c'
s=p.read_text(encoding='utf-8');assert s.count('10802U')==2
p.write_text(s.replace('10802U','10803U'),encoding='utf-8')
print(STAGE)
