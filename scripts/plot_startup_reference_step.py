"""Plot the confirmed no-assistance 1213 trial; no target/hardware access."""
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'diagnostics/common_swing1213_rear_fall_trial1_20261112.json'
OUT=ROOT/'diagnostics/startup_step1213_20261112'
data=json.loads(SOURCE.read_text(encoding='utf-8'))
rows=[r for r in data['records'] if r['elapsed_ms']<=3630]
by_time={r['elapsed_ms']:r for r in rows}


def snapshot(ms):
    r=by_time[ms]
    return {
        'elapsed_ms':ms,'phase':r['phase'],'active':r['active'],
        'theta_actual_deg':math.degrees(r['state'][0]),
        'theta_reference_deg':math.degrees(r['reference'][0]),
        'left_leg_actual_deg':math.degrees(r['leg_angle_rad'][0]),
        'right_leg_actual_deg':math.degrees(r['leg_angle_rad'][1]),
        'left_leg_reference_deg':math.degrees(r['angle_reference_rad'][0]),
        'right_leg_reference_deg':math.degrees(r['angle_reference_rad'][1]),
        'common_leg_target_deg':89.5,
        'wheel_requested_mNm':1000*r['wheel_requested_nm'],
        'wheel_applied_mean_mNm':1000*r['mean_wheel_torque_command_nm'],
        'theta_wheel_component_mNm':1000*r['wheel_components_nm'][0],
        'lqr_Tp_mNm':1000*r['leg_Tp_nm'],
        'left_local_hold_mNm':1000*r['angle_pd_nm'][0],
        'right_local_hold_mNm':1000*r['angle_pd_nm'][1],
        'body_pitch_deg':r['body_pitch_deg'],
    }


points={str(ms):snapshot(ms) for ms in (0,720,760,800)}
before,after=points['720'],points['760']
report={
    'source':str(SOURCE),'build_id':data['header']['build_id'],
    'generation':data['header']['generation'],'record_period_ms':data['header']['period_ms'],
    'operator_confirmation':{'joystick':'neutral','pushed_or_supported_during_trial':False},
    'scope':'First saved trial only; not the later balance ring. No external assistance per user.',
    'snapshots':points,
    'adjacent_frame_delta_720_to_760':{
        key:after[key]-before[key] for key in (
            'theta_actual_deg','theta_reference_deg','wheel_requested_mNm',
            'wheel_applied_mean_mNm','theta_wheel_component_mNm','lqr_Tp_mNm')},
    'findings':[
        'At adjacent720/760ms records, phase changes RAMP(2) to BALANCE(3).',
        'Theta reference jumps to zero while measured theta changes by less than1degree.',
        'Wheel request changes from about-62 to-134mNm; the theta component changes by about-75mNm.',
        'Local absolute-leg PD is zero by760ms although both legs remain far from89.5degrees.',
        'At800ms right leg reference is still about67.7degrees, actual about54.4degrees.',
        'Later the first trial faults on leg pose/split at3630ms. This plot does not establish that the startup step alone caused the later fault.'
    ],
    'limits':'40ms logging does not resolve the exact switching instant. Torque curves are requests/applied command limits, not measured mechanical torque.',
}

fonts={font.name for font in font_manager.fontManager.ttflist}
font=next((name for name in ('Microsoft YaHei','SimHei','Noto Sans CJK SC','Arial Unicode MS') if name in fonts),None)
if font:
    plt.rcParams['font.sans-serif']=[font]
plt.rcParams['axes.unicode_minus']=False
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
t=[r['elapsed_ms']/1000 for r in rows]
fig,axes=plt.subplots(3,1,figsize=(10.5,8),sharex=True,constrained_layout=True)
fig.suptitle('1213 无外力试跑：启动切换附近的参考阶跃',fontsize=15,fontweight='bold')
colors={'actual':'#197b84','reference':'#c45f43','left':'#197b84','right':'#72659b','request':'#c45f43','applied':'#2e5860'}

axes[0].plot(t,[math.degrees(r['state'][0]) for r in rows],color=colors['actual'],label='实际世界腿角 θ',lw=1.9)
axes[0].plot(t,[math.degrees(r['reference'][0]) for r in rows],color=colors['reference'],label='θ 参考',lw=1.8)
axes[0].set_ylabel('世界腿角 / °')
axes[0].annotate(f"参考 {before['theta_reference_deg']:.2f}° → 0°\n实际 {before['theta_actual_deg']:.2f}° → {after['theta_actual_deg']:.2f}°",
                 xy=(.76,0),xytext=(1.05,-8),arrowprops={'arrowstyle':'->','color':'#666'},fontsize=9)
axes[0].legend(loc='lower left',frameon=False,ncol=2)
axes[0].set_title('用户确认：摇杆中立，未推车或扶住；首轮数据，40 ms/帧',loc='left',fontsize=10,color='#555')

for side,name,key in ((0,'左腿','left'),(1,'右腿','right')):
    axes[1].plot(t,[math.degrees(r['leg_angle_rad'][side]) for r in rows],label=f'{name}实际角度',color=colors[key],lw=1.8)
axes[1].axhline(89.5,color='#777',lw=1.2,ls='--',label='最终中立目标 89.5°')
axes[1].set_ylabel('机身相对腿角 / °')
axes[1].annotate('0.8 s：实际 68.0° / 54.4°\n右腿参考仍为 67.7°',
                 xy=(.8,points['800']['right_leg_actual_deg']),xytext=(1.14,54),
                 arrowprops={'arrowstyle':'->','color':'#666'},fontsize=9)
axes[1].legend(loc='upper left',frameon=False,ncol=3,fontsize=9)

axes[2].plot(t,[1000*r['wheel_requested_nm'] for r in rows],color=colors['request'],label='共同轮矩请求',lw=1.7)
axes[2].plot(t,[1000*r['mean_wheel_torque_command_nm'] for r in rows],color=colors['applied'],label='共同轮矩限幅后命令',lw=1.7,ls='--')
for sign in (-1,1):
    axes[2].axhline(sign*152.86,color='#bbb',lw=.9,ls=':')
axes[2].annotate(f"请求 {before['wheel_requested_mNm']:.1f} → {after['wheel_requested_mNm']:.1f} mN·m",
                 xy=(.76,after['wheel_requested_mNm']),xytext=(1.10,-127),
                 arrowprops={'arrowstyle':'->','color':'#666'},fontsize=9)
axes[2].set_ylabel('轮矩命令 / mN·m')
axes[2].set_xlabel('启动后时间 / s')
axes[2].legend(loc='lower left',frameon=False,ncol=2,fontsize=9)
axes[2].set_xlim(0,3.63)
axes[2].text(.99,.96,'3.63 s：腿姿态/分腿保护停机',transform=axes[2].transAxes,ha='right',va='top',fontsize=9,color='#555')

for ax in axes:
    ax.axvspan(.72,.76,color='#e8b85c',alpha=.23,zorder=0)
    ax.axvline(.75,color='#b89451',lw=1,ls=':')
    ax.grid(axis='both',color='#e5e7e7',lw=.6)
    ax.tick_params(axis='both',length=3)
fig.savefig(OUT.with_suffix('.png'),dpi=175,facecolor='white')
plt.close(fig)
OUT.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

lines=['# 1213 无外力试跑：启动参考阶跃','',
       '用户确认：摇杆中立，试跑期间没有推车或扶住。只分析首轮保存记录（generation12），不混用后一次环形记录。','',
       '| 时刻 | 阶段 | 实际 θ | 参考 θ | 左/右腿实际角 | 轮矩请求/应用命令 |',
       '|---|---|---|---|---|---|']
for ms,p in points.items():
    lines.append(f"| {ms} ms | {p['phase']} | {p['theta_actual_deg']:.6f}° | {p['theta_reference_deg']:.6f}° | {p['left_leg_actual_deg']:.6f}° / {p['right_leg_actual_deg']:.6f}° | {p['wheel_requested_mNm']:.6f} / {p['wheel_applied_mean_mNm']:.6f} mN·m |")
lines += ['',
    f"720→760 ms：实际 θ 仅改变 {after['theta_actual_deg']-before['theta_actual_deg']:.6f}°，参考却跳变 {after['theta_reference_deg']-before['theta_reference_deg']:.6f}°；轮矩请求改变 {after['wheel_requested_mNm']-before['wheel_requested_mNm']:.6f} mN·m。其中 θ 反馈项从 {before['theta_wheel_component_mNm']:.6f} 变到 {after['theta_wheel_component_mNm']:.6f} mN·m。",
    '',
    f"初始两腿为 {points['0']['left_leg_actual_deg']:.6f}° / {points['0']['right_leg_actual_deg']:.6f}°。0.8 s 实际仍只有 {points['800']['left_leg_actual_deg']:.6f}° / {points['800']['right_leg_actual_deg']:.6f}°；参考为 {points['800']['left_leg_reference_deg']:.6f}° / {points['800']['right_leg_reference_deg']:.6f}°，最终中立目标89.5°。此时局部绝对角 PD 已退出。",
    '',
    '这是可由记录直接证实的启动切换不连续。记录周期40 ms，无法把精确跳变时刻定位到该窗口内部；约0.75 s为程序阶段切换。轮矩曲线是命令，不是实测机械力矩。后续3.63 s发生腿姿态/分腿保护，但此图不证明启动阶跃是后续故障的唯一原因。','']
OUT.with_suffix('.md').write_text('\n'.join(lines),encoding='utf-8')
print(json.dumps({'png':str(OUT.with_suffix('.png')),'json':str(OUT.with_suffix('.json')),'md':str(OUT.with_suffix('.md')),'font':font},ensure_ascii=False,indent=2))
