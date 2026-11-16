"""Stage the LQR actuator/sync layer and retained parking gate diagnostics."""
from pathlib import Path
import shutil
import re
ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'build/lmpc_actuation_10802'
SOURCE=ROOT.parent/'H723_LMPC'
def replace(s,a,b):
    if s.count(a)!=1:raise ValueError(a[:100])
    return s.replace(a,b)
if not STAGE.exists():
    shutil.copytree(SOURCE,STAGE,ignore=shutil.ignore_patterns('build','backups','__pycache__','sim_cache','*.uvguix.*'))
    (STAGE/'build').mkdir()
path=STAGE/'App/Tasks/Inc/wheel_leg_config.h';s=path.read_text(encoding='utf-8')
if 'LEG_DEFAULT_SYNC_RECOVERY_PRIORITY' in s:raise SystemExit('Candidate is already modified')
values={'LEG_DEFAULT_SYNC_RATIO':'0.65f','LEG_DEFAULT_SYNC_SPEED_KP':'0.06f',
 'LEG_DEFAULT_SYNC_LIMIT_NM':'0.3f','LEG_DEFAULT_SYNC_RATE_NM_S':'0.4f',
 'BALANCE_DEFAULT_BACK_EMF_RATIO':'0.3f',**{'JOINT_'+joint+'_TORQUE_RATIO':'0.0400f' for joint in ['LF','LR','RF','RR']}}
for key,value in values.items():
    s,n=re.subn(r'(#define\s+'+key+r'\s+)\S+',lambda m:m[1]+value,s)
    assert n==1,key
begin=s.index('/* Runtime validation allows 0..0.2 N*m;')
end=s.index('/* 4. 轮子平衡',begin)
s=s[:begin]+s[begin:end].replace('/* Runtime validation allows 0..0.2 N*m; 0.3 prevents arming. */',
    '/* Match the tested LQR sync; final ratio*limit remains below 0.2 Nm. */')+s[end:]
marker='/* 4. 轮子平衡'
s=replace(s,marker,'#define LEG_DEFAULT_SYNC_RECOVERY_PRIORITY       1U\n'
 '#define LEG_SYNC_RECOVERY_RESERVE_RATIO           0.5f\n'
 '#define LEG_SYNC_RECOVERY_BLEND_RAD               0.087266463f\n\n'+marker)
path.write_text(s,encoding='utf-8')
path=STAGE/'App/Tasks/Inc/wheel_leg_task.h';s=path.read_text(encoding='utf-8')
baseline=(ROOT/'App/Tasks/Inc/wheel_leg_task.h').read_text(encoding='utf-8')
begin=baseline.index('    uint32_t sync_recovery_priority_enable;');end=baseline.index('} DualHoldDebug;',begin)
s=replace(s,'} DualHoldDebug;',baseline[begin:end]+'} DualHoldDebug;')
marker='extern volatile BalanceDebug g_balance;'
s=replace(s,marker,marker+'''
/* Retained on disarm; zeroed only at the next control episode.
 * Gate bits: speed, pitch, body rate, leg split, wheel voltage, wheel Nm,
 * joint voltage, disabled/non-neutral mode. No motor-output ownership. */
typedef struct {
    uint32_t episode,tick_ms,gate_mask,settle_ms,samples,learning_samples;
    uint32_t blocked_samples,gate_count[8],input_limited_samples;
    float filtered_speed_m_s,lean_rad,body_pitch_rad,body_rate_rad_s,leg_split_rad;
} ParkingLearningDebug;
extern volatile ParkingLearningDebug g_parking_learning_debug;
''')
path.write_text(s,encoding='utf-8')
path=STAGE/'App/Tasks/Src/wheel_leg_task.c';s=path.read_text(encoding='utf-8')
s=replace(s,'volatile WheelLegDebug g_wheel_leg;','volatile WheelLegDebug g_wheel_leg;\nvolatile ParkingLearningDebug g_parking_learning_debug;')
s=replace(s,'    .load_rate_n_s = LEG_DEFAULT_LOAD_RATE_N_S',
 '    .load_rate_n_s = LEG_DEFAULT_LOAD_RATE_N_S,\n'
 '    .sync_recovery_priority_enable = LEG_DEFAULT_SYNC_RECOVERY_PRIORITY,\n'
 '    .sync_priority_scale = 1.0f')
s=replace(s,'    if (c->support_only>1U || c->load_enable>1U) return 0;',
 '    if (c->support_only>1U || c->load_enable>1U || c->sync_recovery_priority_enable>1U) return 0;')
s=replace(s,'c->sync_limit_nm>0.2f ||','c->sync_limit_nm>0.3f ||\n        c->sync_ratio*c->sync_limit_nm>0.2f ||')
s=replace(s,'    if (!balance_active) balance_origin=x[2];',
 '''    if (!balance_active) {
        balance_origin=x[2];
        ParkingLearningDebug debug={0};debug.episode=g_parking_learning_debug.episode+1;
        g_parking_learning_debug=debug;
    }''')
begin=s.index('        unsigned healthy=fabsf(x[3])');end=s.index('        if (!healthy)',begin)
s=s[:begin]+'''        uint32_t gates=0;
        if(fabsf(x[3])>BALANCE_PARKING_LEAN_MAX_SPEED_M_S)gates|=1U;
        if(fabsf(state->body_pitch_rad)>BALANCE_PARKING_LEAN_POSE_RAD)gates|=2U;
        if(fabsf(x[5])>BALANCE_PARKING_LEAN_BODY_RATE_RAD_S)gates|=4U;
        if(fabsf(state->leg[0].angle-state->leg[1].angle)>BALANCE_PARKING_LEAN_POSE_RAD)gates|=8U;
        if(g_wheel_output.saturated)gates|=16U;
        if(g_wheel_output.torque_saturated)gates|=32U;
        if(g_joint_output.saturated)gates|=64U;
        /* Exactly the LQR health gates. QP input limits alone do not reject
           an otherwise healthy low-speed equilibrium-learning sample. */
        unsigned healthy=gates==0;
        g_parking_learning_debug.gate_mask=gates;
        if(gates)++g_parking_learning_debug.blocked_samples;
        for(unsigned i=0;i<7;++i)if(gates&(1U<<i))++g_parking_learning_debug.gate_count[i];
        if(c.saturated || fabsf(c.torque_nm[0])>=lmpc_limit[0]*c.wheel_scale-1e-6f)
            ++g_parking_learning_debug.input_limited_samples;
''' +s[end:]
s=replace(s,'    if (!learn_mode) {\n        balance_parking_lean',
 '    if (!learn_mode) {\n        g_parking_learning_debug.gate_mask=128U;\n'
 '        ++g_parking_learning_debug.gate_count[7];\n'
 '        ++g_parking_learning_debug.blocked_samples;\n        balance_parking_lean')
marker='    reference[0]=balance_parking_lean;'
s=replace(s,marker,'''
    ++g_parking_learning_debug.samples;
    if(g_balance.parking_lean_learning)++g_parking_learning_debug.learning_samples;
    g_parking_learning_debug.tick_ms=state->tick_ms;
    g_parking_learning_debug.settle_ms=balance_parking_settle_ms;
    g_parking_learning_debug.filtered_speed_m_s=balance_parking_speed;
    g_parking_learning_debug.lean_rad=balance_parking_lean;
    g_parking_learning_debug.body_pitch_rad=state->body_pitch_rad;
    g_parking_learning_debug.body_rate_rad_s=x[5];
    g_parking_learning_debug.leg_split_rad=state->leg[0].angle-state->leg[1].angle;
'''+marker)
s=replace(s,'    static float sync_error_filtered, sync_applied[2];',
 '    static float sync_error_filtered, sync_applied[2];\n    static float sync_priority_scale=1.0f;')
s=replace(s,'    float sync_difference=0, sync_speed=0, sync_target_speed=0;',
 '    float sync_difference=0, sync_speed=0, sync_target_speed=0;\n'
 '    float sync_used[2]={0},sync_previous[2]={sync_applied[0],sync_applied[1]};\n'
 '    uint32_t sync_priority_active=0U;')
s=s.replace('if (!active) sync_error_filtered=sync_applied[0]=sync_applied[1]=0;',
 'if (!active) {sync_error_filtered=sync_applied[0]=sync_applied[1]=0;sync_priority_scale=1;}')
s=s.replace('if (c.sync_ratio==0) sync_error_filtered=sync_applied[0]=sync_applied[1]=0;',
 'if (c.sync_ratio==0) {sync_error_filtered=sync_applied[0]=sync_applied[1]=0;sync_priority_scale=1;}')
baseline=(ROOT/'App/Tasks/Src/wheel_leg_task.c').read_text(encoding='utf-8')
start=s.index('    for (unsigned side = 0; side < 2; ++side) {\n        /* The shared LQR mean')
end=s.index('        error[side] = c.target_m',start)
block=s[start:end]
block=block[:block.index('        pitch[side]=')]+'''        pitch[side]=angle_hold[side]+lqr_pitch[side];
    }
'''
a=baseline.index('    /* ESP32 sums common LQR and opposite sync directly.')
b=baseline.index('#if ROBOT_LEG_VOLTAGE_PROBE',a)
block+=baseline[a:b]+'''    for(unsigned side=0;side<2;++side) {
        sync_used[side]=sync_priority_scale*sync_applied[side];
        pitch[side]+=sync_used[side];
#if ROBOT_LEG_PD_TEST_SIDE >= 0
        if (side != ROBOT_LEG_PD_TEST_SIDE) angle_hold[side]=pitch[side]=0;
#endif
'''
s=s[:start]+block+s[end:]
s=s.replace('    sync_difference=sync_speed=sync_target_speed=0;',
 '    sync_difference=sync_speed=sync_target_speed=0;\n    sync_used[0]=sync_used[1]=0;sync_priority_scale=1;sync_priority_active=0U;')
s=replace(s,'    g_dual_hold.sync_target_speed_rad_s=sync_target_speed;',
 '    g_dual_hold.sync_target_speed_rad_s=sync_target_speed;\n'
 '    g_dual_hold.sync_priority_scale=sync_priority_scale;g_dual_hold.sync_priority_active=sync_priority_active;')
s=replace(s,'        g_dual_hold.sync_torque_nm[i]=sync_applied[i];',
 '        g_dual_hold.sync_raw_torque_nm[i]=sync_applied[i];\n        g_dual_hold.sync_torque_nm[i]=sync_used[i];')
path.write_text(s,encoding='utf-8')
path=STAGE/'App/Services/Src/robot_telemetry_service.c';s=path.read_text(encoding='utf-8').replace('10801U','10802U');path.write_text(s,encoding='utf-8')
print(STAGE)
