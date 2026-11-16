"""Plot the first actual1216 trial; offline, active samples only.

User confirmed wheels grounded and no manual support. Accepted motion target
was zero; raw stick position was not recorded. Requests are not load-cell
measurements, and a clipped speed feedback cannot bound actual vehicle speed.
"""
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT/'diagnostics/support_release1216_trial1_20261113.json'
RING = ROOT/'diagnostics/balance_ring1216_trial1_20261113.json'
OUT = ROOT/'diagnostics/support_release1216_trial1_charts_20261113'


def style():
    fonts = {font.name for font in font_manager.fontManager.ttflist}
    font = next((name for name in ('Microsoft YaHei','SimHei','Noto Sans CJK SC') if name in fonts), None)
    if font:
        plt.rcParams['font.sans-serif'] = [font]
    plt.rcParams.update({'font.size':10.5, 'axes.unicode_minus':False,
                         'axes.spines.top':False, 'axes.spines.right':False,
                         'pdf.fonttype':42})


def main():
    style()
    data = json.loads(DATA.read_text(encoding='utf-8'))
    active = [row for row in data['records'] if row['active']]
    ring = json.loads(RING.read_text(encoding='utf-8'))['records']
    start = data['header']['start_tick_ms']
    t = np.array([row['elapsed_ms']*.001 for row in active])
    tr = np.array([(row['tick_ms']-start)*.001 for row in ring])
    assert data['header']['build_id'] == 2026111216 and data['header']['generation'] == 14
    assert tr[0] == 0 and 5 <= tr[-1] < 5.1
    assert all(row['phase'] == 1 for row in active)
    speed = np.array([row['state'][3] for row in ring])
    clipped = abs(speed) >= .8519
    teal, orange, purple, grey = '#167d8d', '#c95838', '#77659f', '#61696e'
    figure, axes = plt.subplots(4,1,figsize=(11.4,10.4),sharex=True)
    figure.subplots_adjust(top=.88,bottom=.13,left=.105,right=.975,hspace=.24)
    figure.suptitle('1216 实测：支撑已恢复，后段速度仍发散',x=.105,y=.976,
                    ha='left',fontsize=18,fontweight='bold')
    figure.text(.105,.94,'generation 14 · 接地、无手扶（用户确认） · 仅画主动控制，5.065 s 人工停止',color=grey)
    figure.text(.105,.913,'全程准备状态，但本版完整 T/Tp 均已输出；行驶目标为 0，不能将准备状态误解成电机未使能',color=grey,fontsize=9.5)

    axes[0].plot(tr,[row['body_pitch_deg'] for row in ring],color=orange,lw=1.8,label='机身俯仰')
    axes[0].plot(tr,np.degrees([row['state'][0] for row in ring]),color=teal,lw=1.8,label='世界腿倾角 θ')
    axes[0].plot(t,np.degrees([row['reference'][0] for row in active]),color=teal,ls='--',label='θ 参考：捕获后连续到 0')
    axes[0].axhline(0,color='#adb3b7',lw=.8)
    axes[0].set_ylabel('角度 / °')
    axes[0].legend(loc='upper left',ncol=3,frameon=False,fontsize=9)

    axes[1].plot(tr,speed,color=teal,lw=1.8,label='速度反馈（程序前进为正）')
    axes[1].plot(t,[row['forward_target_m_s'] for row in active],color=grey,ls='--',label='已接受行驶目标 = 0')
    for sign in (-1,1):axes[1].axhline(sign*.851955,color='#adb3b7',ls=':',lw=.9)
    axes[1].scatter(tr[clipped],speed[clipped],s=12,color=orange,zorder=3,label='报文量程截幅，真实速度可能更大')
    axes[1].set_ylabel('速度 / m/s')
    axes[1].set_ylim(-1,1)
    axes[1].legend(loc='upper left',ncol=2,frameon=False,fontsize=8.8)

    axes[2].plot(tr,[row['T_request_nm'] for row in ring],color=orange,lw=2,label='轮矩请求')
    axes[2].plot(tr,[row['T_applied_nm'] for row in ring],color=teal,ls='--',lw=1.3,label='应用命令：与请求重合')
    axes[2].plot(tr,[row['Tp_nm'] for row in ring],color=purple,lw=1.3,label='共同腿部 LQR 摆矩 Tp')
    for sign in (-1,1):axes[2].axhline(sign*.15286,color='#adb3b7',ls=':',lw=.9)
    axes[2].text(.99,.05,'虚线边界为当前 2 A 的软件轮矩换算；本轮未碰到该边界',
                 transform=axes[2].transAxes,ha='right',fontsize=8.8,color=grey)
    axes[2].set_ylabel('力矩命令 / N·m')
    axes[2].set_ylim(-.17,.17)
    axes[2].legend(loc='upper left',ncol=3,frameon=False,fontsize=9)

    for side,color,label in ((0,teal,'左'),(1,purple,'右')):
        axes[3].plot(t,[row['support_command_n'][side] for row in active],color=color,lw=1.8,label=label+'腿支撑命令')
    axes[3].plot(t,[row['support_feedforward_n'] for row in active],color=grey,ls='--',label='支撑前馈：9 N')
    axes[3].axhline(8.6,color='#adb3b7',ls=':',lw=.8)
    axes[3].set_ylabel('轴向力命令 / N')
    axes[3].legend(loc='lower right',ncol=3,frameon=False,fontsize=9)
    axes[3].set_xlabel('启动后时间 / s（停止后的约 2 s 下落数据已排除）')
    for axis in axes:
        axis.grid(color='#e5e8e9',lw=.65);axis.set_axisbelow(True);axis.set_xlim(0,5.07)
    figure.text(.105,.027,'角度、速度均按程序坐标。支撑/力矩为命令，实际 Iq 在同期反馈中跟随；本图不代表测得的机械力。\n'
                '姿态合格与停车稳定是不同条件：本轮没有触发故障，但末段后向加速仍然存在。',fontsize=9.3,color=grey,linespacing=1.6)
    figure.savefig(OUT.with_suffix('.png'),dpi=165,facecolor='white')
    plt.close(figure)

    figure, axes = plt.subplots(2,1,figsize=(11.4,7.4),sharex=True)
    figure.subplots_adjust(top=.84,bottom=.15,left=.105,right=.975,hspace=.26)
    figure.suptitle('1216 腿姿态：长度未回目标，右腿持续偏离中立',x=.105,y=.97,ha='left',fontsize=17,fontweight='bold')
    figure.text(.105,.915,'同一轮 generation 14 · 主动控制 0–5.030 s · 40 ms 采样',color=grey)
    for side,color,label in ((0,teal,'左'),(1,purple,'右')):
        axes[0].plot(t,[1000*row['leg_length_m'][side] for row in active],color=color,lw=1.8,label=label+'腿实测长度')
        axes[1].plot(t,np.degrees([row['leg_angle_rad'][side] for row in active]),color=color,lw=1.8,label=label+'腿实际角度')
        axes[1].plot(t,np.degrees([row['angle_reference_rad'][side] for row in active]),color=color,ls='--',alpha=.65,label=label+'腿参考（默认不叠加局部 PD）')
    axes[0].axhline(70,color=grey,ls='--',label='目标 70 mm')
    axes[0].set_ylabel('虚拟腿长 / mm');axes[0].legend(loc='upper left',ncol=3,frameon=False)
    axes[1].axhspan(85,95,color='#75a671',alpha=.10,label='相对腿角就绪范围 85–95°')
    axes[1].set_ylabel('机身相对腿角 / °');axes[1].set_ylim(72,96)
    axes[1].legend(loc='lower left',ncol=2,frameon=False,fontsize=9)
    axes[1].set_xlabel('启动后时间 / s')
    for axis in axes:
        axis.grid(color='#e5e8e9',lw=.65);axis.set_axisbelow(True);axis.set_xlim(0,5.07)
    figure.text(.105,.028,'右腿始终未进入就绪范围，因此行驶许可未放开；完整平衡反馈仍在执行。\n'
                '关闭额外反压恢复了承重请求，但现有高度闭环尚未让腿长收回至 70 mm。',fontsize=9.3,color=grey,linespacing=1.6)
    figure.savefig(OUT.with_name(OUT.name+'_legs').with_suffix('.png'),dpi=165,facecolor='white')
    plt.close(figure)
    print(OUT.with_suffix('.png'))
    print(OUT.with_name(OUT.name+'_legs').with_suffix('.png'))


if __name__ == '__main__':
    main()
