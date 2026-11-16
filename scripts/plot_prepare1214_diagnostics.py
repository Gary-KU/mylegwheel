"""Read-only offline evidence for1214; no debugger or motor access.

The saved first trial and later ring are deliberately analyzed separately.
"""
import copy
import json
import math
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'diagnostics/startup_handoff1214_rear_fall_trial1_20261112.json'
RING_SOURCE=ROOT/'diagnostics/balance_ring1214_rear_fall_20261112.json'
OUT=ROOT/'diagnostics/prepare1214_evidence_20261112'
data=json.loads(SOURCE.read_text(encoding='utf-8'))
active=[r for r in data['records'] if r['active']]
co=np.loadtxt(ROOT/'models/lqr/lqr_coefficients.csv',delimiter=',')
cap=.03
raw=[]
for r in active:
    length=float(np.mean(r['leg_length_m']))
    gain=(co@[length**3,length**2,length,1]).reshape(2,6,order='F')
    error=np.array(r['state'])-np.array(r['reference'])
    error[2]=np.clip(error[2],-.05,.05)
    raw.append(float(-gain[0]@error))
raw=np.array(raw)
limited=np.array([r['wheel_requested_nm'] for r in active])
clipped=np.abs(raw)>cap+1e-6
assert all(r['phase']==1 for r in active)
assert np.max(np.abs(limited-np.clip(raw,-cap,cap)))<1e-7

def first_time(predicate):
    return next((r['elapsed_ms'] for r in active if predicate(r)),None)

def pose_ready(r):
    return abs(r['body_pitch_deg'])<=5 and all(abs(math.degrees(a)-90)<=5 for a in r['leg_angle_rad'])

def references_ready(r):
    # Local target=90deg-pitch trim. The preceding controller frame can
    # differ by0.0025deg while trim slews, much less than the target step.
    target=math.pi/2+r['reference'][4]
    tolerance=.2*r['control_period_ms']*.001+math.radians(.0025)+1e-6
    return max(abs(a-target) for a in r['angle_reference_rad'])<=tolerance

nodes=(1,2,3,5,6,7)
current_stats={}
for i,node in enumerate(nodes):
    rows=[r for r in active if r['elapsed_ms']>=200 and
          r['foc_actual_iq_a'][i] is not None and r['foc_feedback_age_ms'][i]<=10]
    target=np.array([r['foc_applied_iq_a'][i] for r in rows])
    actual=np.array([r['foc_actual_iq_a'][i] for r in rows])
    error=np.abs(actual-target)
    current_stats[str(node)]={'samples':len(rows),'mae_a':float(np.mean(error)),
        'p95_abs_error_a':float(np.percentile(error,95)),
        'max_abs_error_a':float(max(error)),
        'max_feedback_age_ms':max(r['foc_feedback_age_ms'][i] for r in rows)}

local_caps={str(i):{'first_ms':first_time(lambda r:abs(r['angle_pd_nm'][i])>=.16-1e-6),
    'samples':sum(abs(r['angle_pd_nm'][i])>=.16-1e-6 for r in active)} for i in (0,1)}
ring=json.loads(RING_SOURCE.read_text(encoding='utf-8'))
ring_rows=ring['records']
ring_summary={
    'source':str(RING_SOURCE),'separate_run':True,
    'first_tick_ms':ring['first_tick_ms'],'last_tick_ms':ring['last_tick_ms'],
    'phases':sorted({r['phase'] for r in ring_rows}),
    'max_abs_world_theta_deg':max(abs(math.degrees(r['state'][0])) for r in ring_rows),
    'max_abs_body_pitch_deg':max(abs(r['body_pitch_deg']) for r in ring_rows),
    'relative_common_leg_range_deg':[float(v) for v in (
        min(90+math.degrees(r['state'][0]+r['state'][4]) for r in ring_rows),
        max(90+math.degrees(r['state'][0]+r['state'][4]) for r in ring_rows))],
    'max_abs_T_request_nm':max(abs(r['T_request_nm']) for r in ring_rows),
    'max_abs_T_applied_nm':max(abs(r['T_applied_nm']) for r in ring_rows),
    'speed_wire_clip_samples':sum(abs(r['state'][3])>=.8519 for r in ring_rows),
    'limitations':'No speed target, individual leg angles, actual Iq, or position_locked flag in ring. Its position error is a true error only when locked. Do not infer a fixed goal or external-force timing from it.'
}

# Independent nominal small-signal check of actual PREPARE feedback:
# measured theta/x references cancel those two feedback columns; LQR Tp is
# disabled and local relative-angle PD is the second actuator's feedback.
sys.path.insert(0,str(ROOT/'tests'))
from verify_lqr_extended_domain import discrete_plant
p0=json.loads((ROOT/'models/lqr/lqr_model_metadata.json').read_text())['parameters']
pole_screens=[]
for mass in (1.47,1.6):
    p=copy.deepcopy(p0)
    ratio=(mass-p['non_body_mass_kg'])/p['body_mass_kg']
    p['body_mass_kg']*=ratio;p['body_pitch_inertia_per_side']*=ratio
    for length in (.07,.077,.10):
        a,b=discrete_plant(p,length)
        k=(co@[length**3,length**2,length,1]).reshape(2,6,order='F')
        feedback=np.zeros((2,6));feedback[0]=k[0]
        feedback[0,0]=feedback[0,2]=0
        feedback[1]=[.6,.02,0,0,.6,.02]
        eig=np.linalg.eigvals(a-b@feedback)
        pole_screens.append({'mass_kg':mass,'length_m':length,
            'spectral_radius':float(max(abs(eig))),
            'poles':[[float(v.real),float(v.imag)] for v in eig]})

report={
    'source':str(SOURCE),'build_id':data['header']['build_id'],
    'generation':data['header']['generation'],'active_samples':len(active),
    'active_phases':sorted({r['phase'] for r in active}),
    'fault_elapsed_ms':data['summary']['trigger_elapsed_ms'],
    'fault_reason':data['summary']['last_fault_reason'],
    'operator_assistance':'Not established for this1214 trial; do not import1213 no-assistance confirmation.',
    'raw_T_reconstruction':{'cap_nm':cap,'cap_samples':int(sum(clipped)),
        'first_cap_ms':int(active[np.flatnonzero(clipped)[0]]['elapsed_ms']),
        'max_raw_nm':float(max(raw)),'min_raw_nm':float(min(raw)),
        'max_clamp_reconstruction_residual_nm':float(np.max(np.abs(limited-np.clip(raw,-cap,cap)))),
        'recorded_output_saturation_samples':sum(r['wheel_voltage_saturated'] for r in active),
        'theta_reference_equals_measurement_max_error_rad':max(abs(r['state'][0]-r['reference'][0]) for r in active),
        'leg_lqr_Tp_max_abs_nm':max(abs(r['leg_Tp_nm']) for r in active)},
    'readiness':{'pose_ready_samples':sum(pose_ready(r) for r in active),
        'first_both_references_arrived_ms':first_time(references_ready),
        'left_max_actual_angle_deg':max(math.degrees(r['leg_angle_rad'][0]) for r in active),
        'right_max_actual_angle_deg':max(math.degrees(r['leg_angle_rad'][1]) for r in active),
        'local_angle_cap':local_caps},
    'current_tracking_after200ms':current_stats,
    'nominal_prepare_pole_screen':pole_screens,
    'separate_later_ring':ring_summary,
    'causal_limits':[
        'Output is active and actual current broadly follows the limited request.',
        'PREPARE wheel cap binds later, but worsening leg pose and low support commands start earlier.',
        'All preparation pose samples fail; staying PREPARE is consistent with the coded gate.',
        'The nominal unbounded small-signal PREPARE structure has unstable attitude poles at typical leg lengths; this is not a fitted real-robot model.',
        'Virtual support and torque curves are commands, not measured forces/torques.',
        'The40ms sampling cannot measure sub-frame current latency precisely.',
        'Physical fall direction versus signed IMU axes and external assistance are not independently verified for this trial.'
    ]
}
OUT.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

fonts={font.name for font in font_manager.fontManager.ttflist}
font=next((name for name in ('Microsoft YaHei','SimHei','Noto Sans CJK SC') if name in fonts),None)
if font:plt.rcParams['font.sans-serif']=[font]
plt.rcParams['axes.unicode_minus']=False
plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(5,1,figsize=(10.5,11),sharex=True,constrained_layout=True)
fig.suptitle('1214 首轮：一直处于 PREPARE 的证据',fontsize=15,fontweight='bold')
t=np.array([r['elapsed_ms']*.001 for r in active])
teal,purple,orange='#197b84','#72659b','#c45f43'
for side,color,label in ((0,teal,'左'),(1,purple,'右')):
    axes[0].plot(t,[math.degrees(r['leg_angle_rad'][side]) for r in active],color=color,label=label+'腿实际')
    axes[0].plot(t,[math.degrees(r['angle_reference_rad'][side]) for r in active],color=color,ls='--',alpha=.6,label=label+'腿参考')
axes[0].axhspan(85,95,color='#75a671',alpha=.10,label='姿态准入 85–95°')
axes[0].set_ylabel('相对腿角 / °')
axes[0].legend(loc='lower left',ncol=5,frameon=False,fontsize=8)
axes[0].set_title('只含保存首轮 generation13；不混入后续 ring；本轮外力情况未确认',loc='left',fontsize=9,color='#555')

axes[1].plot(t,[r['body_pitch_deg'] for r in active],color=orange,label='机身俯仰（程序符号）')
axes[1].axhspan(-5,5,color='#75a671',alpha=.12,label='姿态准入 ±5°')
axes[1].axhline(35,color='#999',ls=':',label='倾角保护 35°')
axes[1].set_ylabel('机身俯仰 / °')
axes[1].legend(loc='upper left',ncol=3,frameon=False,fontsize=8)

axes[2].plot(t,raw*1000,color='#555',label='原 K 重算：未缩放 raw T')
axes[2].plot(t,limited*1000,color=orange,label='实际轮矩请求（已准备限幅）',lw=1.8)
for sign in (-1,1):axes[2].axhline(sign*30,color='#999',ls=':',lw=.9)
axes[2].scatter(t[clipped],limited[clipped]*1000,color=orange,s=9,zorder=3)
axes[2].set_ylabel('轮矩命令 / mN·m')
axes[2].legend(loc='upper left',frameon=False,ncol=2,fontsize=8)
axes[2].text(.99,.10,'25/75 帧触及准备限幅；输出饱和标志仍为 0',transform=axes[2].transAxes,ha='right',fontsize=8,color='#555')

target=np.array([(-r['foc_applied_iq_a'][2]+r['foc_applied_iq_a'][5])*.5 for r in active])
actual=np.array([(-r['foc_actual_iq_a'][2]+r['foc_actual_iq_a'][5])*.5 for r in active])
axes[3].plot(t,target,color=orange,ls='--',label='共同轮端目标 Iq（方向统一）')
axes[3].plot(t,actual,color=teal,label='共同轮端实际 Iq（方向统一）',lw=1.7)
axes[3].set_ylabel('电流 / A')
axes[3].legend(loc='upper left',frameon=False,ncol=2,fontsize=8)

for side,color,label in ((0,teal,'左'),(1,purple,'右')):
    axes[4].plot(t,[r['support_command_n'][side] for r in active],color=color,label=label+'腿轴向支撑命令')
axes[4].axhline(1.6*9.80665/2,color='#777',ls='--',label='近直立均分承重参考 7.85 N')
axes[4].set_ylabel('轴向力命令 / N')
axes[4].set_xlabel('启动后时间 / s')
axes[4].legend(loc='upper left',frameon=False,ncol=3,fontsize=8)
for ax in axes:
    ax.grid(color='#e5e7e7',lw=.6)
    ax.axvline(2.99,color='#777',ls=':',lw=.9)
    ax.set_xlim(0,3)
fig.savefig(OUT.with_suffix('.png'),dpi=165,facecolor='white')
plt.close(fig)

lines=['# 1214 PREPARE 诊断（分轮分析）','',
    '首轮文件：`startup_handoff1214_rear_fall_trial1_20261112.json`，generation13。75个有效样本均为PREPARE，2990ms因机身倾角故障10停机。本轮外力情况未独立确认，不能沿用1213的无外力声明。','',
    '## 命令链路与限幅','',
    f'- 由原K和state/reference重算raw_T，25/75帧被±0.03Nm限幅，首次1000ms；raw_T最大{max(raw):.8f}Nm。限幅后与记录请求的最大误差{max(abs(limited-np.clip(raw,-cap,cap))):.3g}Nm。',
    '- 所有theta参考等于实际theta，腿LQR Tp为0；准备阶段没有世界腿角位置反馈。限幅已发生在输出层之前，所以记录中的输出饱和标志0不能说明没有限幅。',
    '- 限幅平台对应约±0.3925A，实际约±0.39A。200ms后两轮跟踪MAE约0.012A，95%绝对误差0.043/0.046A；反馈龄最大7/6ms。40ms记录不足以精确辨别短于一帧的延迟。','',
    '## 准入与恢复','',
    '- 两侧参考约1720ms已到89.5°；但实际左腿全程最高74.83°、右腿最高70.03°，从未满足85–95°姿态门槛。因此一直PREPARE与实现一致。',
    f"- 局部PD左腿首次{local_caps['0']['first_ms']}ms、右腿首次{local_caps['1']['first_ms']}ms达到+.16Nm，之后持续对抗偏差。实际腿角仍降低，并不是局部PD提前退出。",
    '- 0.8s时腿长约108/98mm、轴向支撑命令仅1.72/3.81N，远低于近直立时均分1.6kg需要的7.85N。这是虚拟命令，不是测得地面支持力。此时轮端尚未触及准备限幅，因此不能把整个倒伏单因归于轮矩cap。',
    '- 最后有效机身俯仰为程序坐标下+35.37°，不是负值。文件名和用户“后倒”描述不等于物理轴向符号已校验。','',
    '## 结构检查','',
    '- 工程自身的离散线性模型中，按PREPARE实际反馈（轮theta/x反馈取消，腿LQR取消，只留相对腿PD）构造闭环，在7.7cm、1.47kg下谱半径1.005529>1；1.6kg结果见JSON。它不是实车辨识，但说明长时间停留PREPARE不能仅视为稳态欠力。','',
    '## 后续 ring：不同轮次','',
    '- `balance_ring1214_rear_fall_20261112.json`覆盖121593..126693ms，全程BALANCE；说明另外一次运行已经通过准备门槛，不能与首轮混为一谈。',
    '- 后段world theta约+70°、body约+31.5°、共同相对腿角约128.5°，轮请求约+.41Nm、应用钳位+.15286Nm、轮速触及报文±.852m/s。该姿态已远离直立线性化。',
    '- ring未存速度目标、独立腿角、实际Iq及position_locked；不能确定每段外力，也不能把statex−position_error一概认作真实停车锚点。','']
OUT.with_suffix('.md').write_text('\n'.join(lines),encoding='utf-8')
print(json.dumps({'png':str(OUT.with_suffix('.png')),'json':str(OUT.with_suffix('.json')),'md':str(OUT.with_suffix('.md'))},ensure_ascii=False,indent=2))
