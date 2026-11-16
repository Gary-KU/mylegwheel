from pathlib import Path
import shutil
ROOT=Path(__file__).resolve().parents[1]
source=ROOT.parent/'H723_LMPC';stage=ROOT/'build/lmpc_leg_recovery_20261007'
assert not stage.exists()
shutil.copytree(source,stage,ignore=shutil.ignore_patterns('build','backups','__pycache__','sim_cache'))
(stage/'build').mkdir()
shutil.copy2(ROOT/'diagnostics/lmpc_leg_startup_trace_20261007.json',stage/'tests/data/leg_rearward_10706.json')
p=stage/'App/Tasks/Inc/wheel_leg_config.h';s=p.read_text(encoding='utf-8')
s=s.replace('#define BALANCE_DEFAULT_LEG_SCALE                 0.20f',
 '''/* Use the existing 0.020 Nm virtual-Tp ceiling, not 1/5 of it. */
#define BALANCE_DEFAULT_LEG_SCALE                 1.0f
/* Linear-controller operating envelope, not mechanical joint limits. */
#define LMPC_MAX_THETA_RAD                        0.261799388f
#define LMPC_MAX_PHI_RAD                          0.261799388f
#define LMPC_MAX_RELATIVE_LEG_RAD                 0.349065850f''')
p.write_text(s,encoding='utf-8')
p=stage/'App/Tasks/Inc/wheel_leg_task.h';s=p.read_text(encoding='utf-8')
s=s.replace('ROBOT_FAULT_CPU_OVERTEMP, ROBOT_FAULT_CPU_TEMP_SENSOR',
 'ROBOT_FAULT_CPU_OVERTEMP, ROBOT_FAULT_CPU_TEMP_SENSOR, ROBOT_FAULT_MODEL_REGION')
s=s.replace('/* 0 off, 1 calculated, 2 feedback/range, 3 parameters. Requires dual hold. */',
 '/* 0 off, 1 calculated, 2 feedback/range, 3 parameters, 4 model region. */')
marker='void WheelLeg_Update(void);'
trace='''/* First 640 ms of every CH5 activation, sampled at the 5 ms task rate.
 * Frozen once full/on disarm; a separate fault frame preserves the exit pose. */
#define LMPC_STARTUP_TRACE_ROWS 128U
typedef struct {
    uint32_t tick_ms, active, phase, fault;
    float state[6], reference[6];
    float leg_angle_rad[2], angle_reference_rad[2];
    float lmpc_tp_nm, local_tp_nm[2], sync_tp_nm[2], axial_force_n[2];
    float leg_length_m[2], wheel_torque_nm;
    int16_t voltage_mv[8]; /* CAN slots, same units/order as output owner. */
} LmpcStartupTraceRow;
typedef struct {
    uint32_t count, episode, fault_valid, reserved;
    LmpcStartupTraceRow fault_row, rows[LMPC_STARTUP_TRACE_ROWS];
} LmpcStartupTraceDebug;
extern volatile LmpcStartupTraceDebug g_lmpc_startup_trace;
'''
s=s.replace(marker,trace+marker);p.write_text(s,encoding='utf-8')
p=stage/'App/Tasks/Src/wheel_leg_task.c';s=p.read_text(encoding='utf-8')
marker='''    /* Shared sagittal state, like the reference ESP32 controller.'''
guard='''    /* Both legs must stay inside the local model's operating envelope.
       Do not keep driving a saturating linear controller into a folded pose. */
    for(unsigned side=0;side<2;++side) {
        if(fabsf(state->state[side][0])>LMPC_MAX_THETA_RAD ||
           fabsf(state->state[side][4])>LMPC_MAX_PHI_RAD ||
           fabsf(state->leg[side].angle-1.5707963267948966f)>LMPC_MAX_RELATIVE_LEG_RAD) {
            status=4U;goto publish;
        }
    }
'''
assert marker in s;s=s.replace(marker,guard+marker)
s=s.replace('else if (g_lmpc.status>=2U) reason=ROBOT_FAULT_LMPC;',
 '''else if (g_balance.status==4U) reason=ROBOT_FAULT_MODEL_REGION;
    else if (g_lmpc.status>=2U) reason=ROBOT_FAULT_LMPC;''')
marker='volatile WheelBalanceTraceDebug g_balance_trace;'
trace='''volatile LmpcStartupTraceDebug g_lmpc_startup_trace;
static void startup_trace_capture(const WheelLegDebug *state,uint32_t requested)
{
    static uint32_t was_requested,last_fault;
    if(!requested) {was_requested=0U;return;}
    if(!was_requested) {
        g_lmpc_startup_trace.count=g_lmpc_startup_trace.fault_valid=0U;
        ++g_lmpc_startup_trace.episode;
        last_fault=g_robot_control.last_fault.count;
    }
    was_requested=1U;
    if(g_lmpc_startup_trace.count>=LMPC_STARTUP_TRACE_ROWS &&
       g_robot_control.status==1U)return;
    LmpcStartupTraceRow row={0};
    row.tick_ms=state->tick_ms;
    row.active=g_robot_control.enable && g_robot_control.status==1U;
    row.phase=g_robot_control.startup_phase;
    row.fault=row.active?0U:g_robot_control.last_fault.reason;
    for(unsigned i=0;i<6;++i) {
        row.state[i]=.5f*(state->state[0][i]+state->state[1][i]);
        row.reference[i]=g_balance.reference[i];
    }
    row.lmpc_tp_nm=g_balance.torque_nm[1];
    row.wheel_torque_nm=g_wheel_output.applied_torque_nm;
    for(unsigned side=0;side<2;++side) {
        row.leg_angle_rad[side]=state->leg[side].angle;
        row.angle_reference_rad[side]=g_dual_hold.angle_reference_rad[side];
        row.local_tp_nm[side]=g_dual_hold.angle_hold_torque_nm[side];
        row.sync_tp_nm[side]=g_dual_hold.sync_torque_nm[side];
        row.axial_force_n[side]=g_dual_hold.force_n[side];
        row.leg_length_m[side]=state->leg[side].length;
    }
    for(unsigned i=0;i<8;++i)row.voltage_mv[i]=g_dual_hold.voltage_mv[i];
    if(g_lmpc_startup_trace.count<LMPC_STARTUP_TRACE_ROWS)
        g_lmpc_startup_trace.rows[g_lmpc_startup_trace.count++]=row;
    if(!row.active && (g_robot_control.last_fault.count!=last_fault || row.fault)) {
        g_lmpc_startup_trace.fault_row=row;g_lmpc_startup_trace.fault_valid=1U;
    }
    last_fault=g_robot_control.last_fault.count;
}

'''
assert marker in s;s=s.replace(marker,trace+marker)
s=s.replace('    balance_trace_capture(&out);','    startup_trace_capture(&out,robot_requested);\n    balance_trace_capture(&out);')
p.write_text(s,encoding='utf-8')
p=stage/'App/Services/Src/robot_telemetry_service.c';s=p.read_text(encoding='utf-8').replace('10706','10707');p.write_text(s,encoding='utf-8')
p=stage/'App/Protocol/Inc/robot_telemetry.h';s=p.read_text(encoding='utf-8').replace('ROBOT_TELEMETRY_ANO_PAIRS 27U','ROBOT_TELEMETRY_ANO_PAIRS 28U');p.write_text(s,encoding='utf-8')
p=stage/'App/Protocol/Src/robot_telemetry.c';s=p.read_text(encoding='utf-8')
s=s.replace('else if(pair==26U)',
 '''else if(pair==27U) value=(double)(i ? sample->value[13] :
            .5f*(sample->swing_command_nm[0]+sample->swing_command_nm[1])-
            sample->value[13]-.5f*(sample->value[10]+sample->value[11]))*100000.0;
        else if(pair==26U)''')
p.write_text(s,encoding='utf-8')
config=next(stage.rglob('robot_telemetry_config.h'));print('Telemetry config:',config.relative_to(stage))
s=config.read_text(encoding='utf-8')
s=s.replace('#define ROBOT_TELEMETRY_DEFAULT_ANO_PAIR 20U','#define ROBOT_TELEMETRY_DEFAULT_ANO_PAIR 27U')
# Account for aligned macro whitespace without changing shadow's group 26.
import re
s=re.sub(r'(#define\s+ROBOT_TELEMETRY_DEFAULT_ANO_PAIR\s+)20U',r'\g<1>27U',s)
config.write_text(s,encoding='utf-8')
p=stage/'tests/verify_integration.py';s=p.read_text(encoding='utf-8').replace('<=.004001f','<=.020001f');p.write_text(s,encoding='utf-8')
p=stage/'tests/verify_telemetry.py';s=p.read_text(encoding='utf-8').replace('C.byref(sample),27)==0','C.byref(sample),28)==0')
s+='''\n# Local PD and LMPC remain independently visible on the only two channels.
sample.swing[0]=.015625;sample.swing[1]=.0078125
sample.value[13]=.00390625;sample.value[10]=.001953125;sample.value[11]=-.001953125
assert lib.RobotTelemetry_EncodeAnoPair(frame,136,C.byref(sample),27)==22
assert struct.unpack('<4i',bytes(frame[4:20]))==(781,390,0,0)
print('PASS: ANO 27 local PD/LMPC Tp decomposition, no extra wire fields.')
'''
p.write_text(s,encoding='utf-8')
print('Prepared leg budget restoration, operating-envelope stop and startup capture; no target write or flash.')
