"""Plot the separately retained BALANCE run; offline, no hardware access.

The ring only contains the last5.1seconds. It lacks target velocity, actual
Iq, individual leg angles and external-force timing. Signed software axes
must not be relabeled as independently verified physical forward/backward.
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
SOURCE = ROOT/'diagnostics/balance_ring1214_rear_fall_20261112.json'
OUTPUT = ROOT/'diagnostics/balance1214_tail_charts_20261113'


def main():
    data = json.loads(SOURCE.read_text(encoding='utf-8'))
    rows = data['records']
    assert len(rows) == 256 and all(row['phase'] == 3 for row in rows)
    tick = np.array([row['tick_ms'] for row in rows])
    time = (tick-tick[-1])*.001
    body = np.array([row['body_pitch_deg'] for row in rows])
    theta = np.degrees([row['state'][0] for row in rows])
    speed = np.array([row['state'][3] for row in rows])
    request = np.array([row['T_request_nm'] for row in rows])
    applied = np.array([row['T_applied_nm'] for row in rows])
    limited = np.abs(request-applied) > 1e-6
    clipped = np.abs(speed) >= .8519
    torque_limit = .07643*2.0
    assert np.max(np.abs(applied)) <= torque_limit+1e-6

    fonts = {font.name for font in font_manager.fontManager.ttflist}
    font = next((name for name in ('Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC') if name in fonts), None)
    if font:
        plt.rcParams['font.sans-serif'] = [font]
    plt.rcParams.update({'font.size':11, 'axes.unicode_minus':False,
                         'axes.spines.top':False, 'axes.spines.right':False,
                         'pdf.fonttype':42})
    teal, orange, dark = '#167d8d', '#c95838', '#4b5257'
    figure, axes = plt.subplots(3, 1, figsize=(11.4, 8.8), sharex=True,
                                gridspec_kw={'height_ratios':[1, 1, 1.2]})
    figure.subplots_adjust(top=.87, bottom=.17, left=.105, right=.975, hspace=.22)
    figure.suptitle('1214 后续独立试跑：进入完整平衡后仍然失稳', x=.105, y=.975,
                    ha='left', fontsize=18, fontweight='bold')
    figure.text(.105, .932, '仅保留该轮最后 5.1 秒 · 全程 BALANCE · 与首轮准备阶段记录分开分析',
                color=dark, fontsize=10.5)

    axes[0].plot(time, theta, color=orange, lw=2, label='世界腿倾角 θ')
    axes[0].plot(time, body, color=teal, lw=2, label='机身俯仰角')
    axes[0].axhline(0, color='#a9afb2', lw=.8)
    axes[0].set_ylabel('角度 / °')
    axes[0].legend(loc='upper left', frameon=False, ncol=2)
    axes[0].annotate(f'末帧 θ = {theta[-1]:.1f}°\n机身 = {body[-1]:.1f}°',
                     xy=(time[-1], theta[-1]), xytext=(-1.53, 49),
                     arrowprops={'arrowstyle':'-', 'color':dark}, fontsize=10, color=dark)

    axes[1].plot(time, speed, color=teal, lw=2, label='轮速换算的行驶速度（程序符号）')
    for sign in (-1, 1):
        axes[1].axhline(sign*.851955, color='#8a9297', lw=.9, ls='--',
                        label='CAN 速度量程约 ±0.852 m/s' if sign == 1 else None)
    axes[1].scatter(time[clipped], speed[clipped], s=9, color=orange,
                    label='触及报文量程，真实速度可能更大', zorder=3)
    axes[1].set_ylabel('速度 / m/s')
    axes[1].set_ylim(-1, 1.04)
    axes[1].legend(loc='upper left', frameon=False, fontsize=9)

    axes[2].plot(time, request, color=orange, lw=2, label='轮端请求力矩')
    axes[2].plot(time, applied, color=teal, lw=2, label='限幅后的应用力矩命令')
    axes[2].fill_between(time, request, applied, where=limited, color=orange, alpha=.13,
                         label='被限幅削去的请求')
    for sign in (-1, 1):
        axes[2].axhline(sign*torque_limit, color='#8a9297', ls='--', lw=.9,
                        label='当前 2 A × Kt：约 ±0.153 N·m' if sign == 1 else None)
    axes[2].set_ylabel('力矩命令 / N·m')
    axes[2].set_xlabel('距该轮最后采样的时间 / s（0 为最后一帧，并非启动时刻）')
    axes[2].legend(loc='upper left', frameon=False, fontsize=9, ncol=2)
    axes[2].annotate(f'末帧请求 {request[-1]:.3f}\n应用 {applied[-1]:.3f} N·m',
                     xy=(time[-1], request[-1]), xytext=(-1.75, .29),
                     arrowprops={'arrowstyle':'-', 'color':dark}, fontsize=10, color=dark)
    for axis in axes:
        axis.grid(axis='both', color='#e5e8e9', lw=.7)
        axis.set_axisbelow(True)
        axis.set_xlim(time[0], time[-1]+.05)
    first_limit = time[np.flatnonzero(limited)[0]]
    first_clip = time[np.flatnonzero(clipped)[0]]
    axes[2].axvline(first_limit, color=orange, ls=':', lw=1)
    axes[1].axvspan(first_clip, time[-1], color=orange, alpha=.055)
    figure.text(.105, .025,
                '注：角度/速度正负为程序坐标；应用力矩是限幅后的命令，不是实测机械力矩。\n'
                '本记录没有速度目标、实际电流与外力信息，不能据此单独确定最初后倒原因；1215 尚无实车数据。',
                fontsize=9.5, color=dark, linespacing=1.6)
    for extension in ('png', 'pdf'):
        figure.savefig(OUTPUT.with_suffix('.'+extension), dpi=165, facecolor='white')
    plt.close(figure)

    summary = dict(source=str(SOURCE), phase='BALANCE', samples=len(rows),
                   duration_s=float(time[-1]-time[0]), first=dict(body_deg=float(body[0]),
                   world_leg_deg=float(theta[0]), speed_m_s=float(speed[0])),
                   last=dict(body_deg=float(body[-1]), world_leg_deg=float(theta[-1]),
                   speed_m_s=float(speed[-1]), requested_nm=float(request[-1]),
                   applied_nm=float(applied[-1])), limited_samples=int(sum(limited)),
                   speed_range_samples=int(sum(clipped)), limits=__doc__)
    OUTPUT.with_suffix('.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print('PNG:', OUTPUT.with_suffix('.png'))


if __name__ == '__main__':
    main()
