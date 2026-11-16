"""Build the debugging indexes and verify/export the prepared STM32 source."""
from __future__ import annotations
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile
import re
import difflib

WORKSPACE=Path(__file__).resolve().parents[1]
ROOT=WORKSPACE/'projects/wheel-legged-stm32'

def write(path,text):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(text.encode('utf-8'))

def patches():
    def patch(before,after,target,paths):
        lines=[]
        for rel in paths:
            a,b=before/rel,after/rel
            if not a.is_file() or not b.is_file():continue
            # Retain CRLF in source hunk lines, matching the frozen files.
            old=a.read_bytes().decode('utf-8-sig').splitlines(keepends=True)
            new=b.read_bytes().decode('utf-8-sig').splitlines(keepends=True)
            lines.extend(difflib.unified_diff(old,new,fromfile='a/'+rel,tofile='b/'+rel))
        target.write_bytes(''.join(lines).encode('utf-8'))
    for label,old,new in [('1012_left_gain','overspeed_1005_20261010','left_output_1012_20261011'),
                          ('1013_tracking','left_output_1012_20261011','tracking_1013_20261011'),
                          ('1014_rejected','tracking_1013_20261011','tracking_1014_20261011')]:
        patch(WORKSPACE/'build'/old/'sources',WORKSPACE/'build'/new/'sources',
          ROOT/'docs/changes'/(label+'.patch'),
          ['App/Tasks/Src/wheel_leg_task.c','App/Tasks/Inc/wheel_leg_config.h','App/Services/Inc/lqr_startup_record.h'])
    patch(ROOT,ROOT/'experiments/startup_1015/overlay',ROOT/'experiments/startup_1015/startup_1015.patch',
      ['App/Tasks/Src/wheel_leg_task.c','App/Tasks/Inc/wheel_leg_config.h','App/Services/Inc/lqr_startup_record.h',
       'tests/verify_motion_tracking.py','tests/verify_parking_lean.py','scripts/read_lqr_startup_record.py'])

def index_debugging():
    topics=[
      ('01 反馈、标定与时间基准',
       '先核对真实 CAN 接收增量、反馈年龄、时钟和编码器到模型角的映射；旧 online 标志不能证明当前 CPU 正在运行。',
       ['can_calibration.txt','joint_calibration_editing.txt','f103_feedback_speed_fix.txt'],
       ['can_schematic.png'],
       'App/Tasks/Src/can_task.c；BSP/Peripherals/Src/bsp_fdcan.c；Core/Src/stm32h7xx_hal_timebase_tim.c；App/Services/Src/joint_calibration.c'),
      ('02 启动自抖与真实输出',
       '初期输出、角速度项、采集分组和执行链都经过排查。Nm 与电压是请求量，低幅度动作还需要检查停止原因和实际发送时间。',
       ['lqr_startup_flash_record_20261007.txt','lqr_left_wheel_diag_20261007.txt','leg_voltage_probe_1v_20261007.txt','leg_voltage_probe_2v2s_20261007.txt'],
       ['user_supplied_ano_group22.png','lqr_startup_analysis_20261007_g1.png','lqr_detailed_motion_g7_analysis_20261007.png','lqr_rate05_fall_analysis_20261007.png'],
       'wheel_leg_task.c 的 wheel/shared Tp/voltage 分配；App/Services/Src/motor_command.c；scripts/f103_feedback_patch.py'),
      ('03 腿开叉、同步与支撑',
       '左右平均状态可能掩盖开叉。共享 LQR 摆角、同步、局部 PD 和轴向支撑一起经过雅可比映射，再受每关节输出限制。不要把 0.02 N·m 的共享 Tp 限幅与 0.68 N·m 电机层直接比较。',
       ['leg_sync_cap_pose_guard_20261007.txt','sync_recovery_priority_20261007.txt','sync_priority_g12_analysis_20261007.txt','leg_direction_confirmed_20261007.txt'],
       ['lqr_sync_response_g8_analysis_20261007.png','lqr_sync_response_later_RAM_20261007.png','lqr_sync_priority_g12_analysis_20261007.png'],
       'wheel_leg_task.c 的 shared LQR / sync priority / dual hold；Control/WheelLeg/Src/leg_pos.c'),
      ('04 低头、前倾与 K 的边界',
       '总质量与机身质量、支撑前馈、实体偏载和执行器限制需要分开核对。修改虚拟腿请求上限或增益，不等于已证明重心位置或唯一倒地原因。',
       ['lqr_mass_update.md','common_pitch_gain_20261007.txt','lqr_sync15_nosedown_20261007.txt'],
       ['lqr_common_pitch_g10_analysis_20261007.png','lqr_sync15_nosedown_analysis_20261007.png','lqr_pitch12_analysis_20261007.png'],
       'Control/WheelLeg/Src/lqr_k.c；wheel_leg_config.h 的腿/轮比例和上限；wheel_leg_task.c 的支撑与姿态分配'),
      ('05 停车位置 clip、主动刹车与停机尾段',
       '停车目标与给 LQR 的有界位置误差分开；限误差不会删除 LQR 的其余反馈项。CH5 关闭后的滑行与运行中的主动刹车不同，尾段和触发理由要一起看。',
       ['parking_input_bound_20261007.txt','parking_damping_20261007.txt','post_stop_tail_20261007.txt','parking_bound_g11_analysis_20261007.txt'],
       ['lqr_parking_bound_g11_analysis_20261007.png','lqr_post_stop_g9_analysis_20261007.png'],
       'wheel_leg_task.c 的 parking_target / correction_limit / parking damping；lqr_startup_record.c 的停止后尾段'),
      ('06 低速重心补偿与烧录器偏载',
       '用户观察过烧录器放前后会带来低速漂移。停车补偿通过姿态、速度、限幅和稳定等待条件门控；它学习的是平衡参考补偿，并非直接测得实体重心。',
       ['parking_lean_20261008.txt','parking_lean_slow_20261008.txt','parking_lean_entry_20261008.txt'],
       ['parking_lean_g18_trace_20261008.png','parking_lean_slow_g19_trace_20261008.png'],
       'wheel_leg_task.c 的 balance_parking_lean 与学习门控；wheel_leg_config.h 的停车补偿参数'),
      ('07 遥控方向、跟随和反打减速',
       'CH2 前后、CH1 yaw 必须做端点与反馈方向确认。目标经过斜坡/死区整形；碰墙、堵转或手扶的区间不能当作自由运行的闭环性能。',
       ['remote_motion_control_20261008.txt','remote_motion_direction_20261008.txt','remote_motion_entry_20261008.txt','remote_tracking_0826_20261008.txt','tracking_brake_0828_20261009.txt'],
       ['lqr_remote_0825_g23_20261008.png','tracking_review_0828_20261009/tracking.png'],
       'App/Protocol/Src/sbus_sticks.c；sbus_calibration_config.h；robot_motion_command.c；wheel_leg_task.c 的 motion damping / lean'),
      ('08 yaw 加速与原地转向',
       '原地转向加入有界积分，随后调整目标上限和共同/差动输出余量。原地快转引起的腿姿态保护，不能只从单个轮子力矩字段判断。',
       ['remote_yaw_pi_0827_20261009.txt','yaw_tracking_1002_20261010.txt','yaw_tracking_1003_20261010.txt','turn_tracking_1004_20261010.txt','overspeed_1005_20261010.txt','coast_1006_20261010.txt'],
       ['tracking_review_1002_20261010/tracking.png','tracking_review_1003_20261010/tracking.png','tracking_review_1004_20261010/tracking.png','tracking_review_1005_20261010/tracking.png','tracking_review_1006_20261010/tracking.png','tracking_review_1006_20261010/fault_tail.png'],
       'wheel_leg_task.c 的 motion_turn_applied / yaw PI / wheel headroom；wheel_leg_config.h 的 yaw/保护参数'),
      ('09 左轮断相线与输出增益',
       '用户后来明确发现左轮相线断开，修复后恢复原 1005。此前仅凭转圈推断软件方向错误并不充分。1012 将左轮 Nm/V 命令参数除以 1.05，右轮不变；这是电压换算增益，不是测得力矩增加 5%。',
       ['left_feedback_1011_20261011.txt','left_output_1012_20261011.txt'],
       ['tracking_review_1005_repaired_20261011_100926/response_review.png','tracking_review_left_output_1012_20261011_102834/tracking.png'],
       'wheel_leg_config.h 的轮反馈方向、轮电压方向、左轮 Nm/V；精确改动：../changes/1012_left_gain.patch'),
      ('10 1013 跟随、1014 回退与 1015 假设',
       '1013 增强行驶速度阻尼并条件性加快刹车时同步释放；它仍有跟随与居中启动后退问题。1014 将部分速度项缩到 25% 的试验已撤回。1015 恢复居中时固定停车路径，但尚未实机验证。',
       ['tracking_1013_20261011.txt','tracking_1014_20261011.txt','startup_1015_20261011.txt'],
       ['tracking_review_1013_20261011_104559/tracking.png'],
       'wheel_leg_task.c 的 motion damping / startup recovery；wheel_leg_config.h；../changes/1013_tracking.patch、1014_rejected.patch、../../experiments/startup_1015/'),
    ]
    lines=['# 调试历程：现象、数据、修改位置与结论','',
      '整理于 2026-10-12。覆盖原工程中保存的 2026-10-01～11 调试资料。主工程冻结在 1013；以下案例是历史，不是当前默认参数清单。','',
      '资料入口：[完整踩坑素材](../history/public_account_control_pitfalls_20261011.txt) · [代码地图](../code_map.md) · [图片目录](../figures/index.md) · [数据目录](../data/index.md)。','',
      '## 使用这些记录的规则','',
      '- 先对齐 build ID、generation 和时间窗口，再看曲线。Flash、RAM 和多次启动分别解释。',
      '- 所有力矩/电压都是控制请求；轮速是编码器估计。用户手扶、碰撞和移动载荷属于测试条件。',
      '- 回归/编译通过与实机有效分别标明；静态重放不等于被控对象的闭环预测。',
      '- 图片原样保留；原聊天里没有落盘的截图未被重新生成。','']
    for title,claim,docs,images,code in topics:
        lines += ['## '+title,'',claim,'','**修改位置：** '+code,'',
          '**原记录：** '+' · '.join(f'[{Path(d).stem}](../history/{d})' for d in docs),'',
          '**图片：** '+' · '.join(f'[{Path(i).stem}](../figures/{i})' for i in images),'']
        for image in images:assert (ROOT/'docs/figures'/image).is_file(),image
        for document in docs:assert (ROOT/'docs/history'/document).is_file(),document
    lines += ['## 1012 与 1013 的实测数字','',
      '| 记录 | 速度 MAE m/s | yaw MAE rad/s | 经验速度曲线对齐 s |',
      '|---|---:|---:|---:|']
    for directory,label in [('tracking_review_left_output_1012_20261011_102834','1012'),('tracking_review_1013_20261011_104559','1013')]:
        j=json.loads((ROOT/'docs/data'/directory/'analysis.json').read_text(encoding='utf-8'))
        lines.append(f"| [{label}](../data/{directory}/analysis.json) | {j['metrics']['speed_mae']:.4f} | {j['metrics']['yaw_mae']:.4f} | {j['curve_alignment_estimate']['speed']['lag_s']:.4f} |")
    lines += ['',
      '这两轮操纵输入不同，不是相同目标序列的 A/B 试验；这些数字不能支持“1013 一定比 1012 更好”。经验对齐也不是纯控制延迟。1013 保存段在约 40 秒填满，实际更晚的停机现场要看单独 RAM 导出。','',
      '![1013 原实测曲线](../figures/tracking_review_1013_20261011_104559/tracking.png)','']
    write(ROOT/'docs/debugging/index.md','\n'.join(lines))

def indexes():
    manifest=json.loads((ROOT/'docs/figures/manifest.json').read_text(encoding='utf-8'))
    lines=['# 原诊断图片目录','',f'保留 {len(manifest)} 张落盘图片，未改变图像像素。原路径与 SHA-256 见 [manifest.json](manifest.json)。','',
           '图中版本、统计区间与输出符号按原分析保留；相关判断边界见 [调试索引](../debugging/index.md)。','',
           '| 图片 | 归档路径 |','|---|---|']
    for entry in manifest:
        rel=entry['path'].removeprefix('docs/figures/')
        lines.append(f'| [{Path(rel).stem}]({rel}) | `{rel}` |')
    write(ROOT/'docs/figures/index.md','\n'.join(lines)+'\n')
    entries=json.loads((ROOT/'docs/data/manifest.json').read_text(encoding='utf-8'))
    lines=['# 调试数据导出目录','',f'归档 {len(entries)} 份 JSON/CSV 导出和分析，清单见 [manifest.json](manifest.json)。','',
      'CSV 保留原字节；JSON 只整理缩进并替换本机路径元数据，测量数值不作平滑或修正。原采集 SHA-256 与公开导出 SHA-256 分别记录。JSON 中的 CRC 属于原二进制采集元数据，不是对整理后的 JSON 重新计算的 CRC。','',
      '## 按采集目录查找','']
    directories=sorted({str(Path(e['path']).parent) for e in entries if Path(e['path']).parent.as_posix()!='docs/data'})
    for directory in directories:
        selected=[e for e in entries if str(Path(e['path']).parent)==directory]
        lines += ['### '+Path(directory).name,'', ' · '.join(
           f'[{Path(e["path"]).name}]({Path(e["path"]).relative_to("docs/data").as_posix()})' for e in selected),'']
    lines += ['## 早期单次导出与摘要','', '| 文件 |','|---|']
    for e in entries:
        if Path(e['path']).parent.as_posix()=='docs/data':
            name=Path(e['path']).name;lines.append(f'| [{name}]({name}) |')
    write(ROOT/'docs/data/index.md','\n'.join(lines)+'\n')
    write(ROOT/'docs/changes/README.md',
      '# 保存源码的精确修改补丁\n\n'
      '| 文件 | 比较版本 | 状态 |\n|---|---|---|\n'
      '| [1012_left_gain.patch](1012_left_gain.patch) | 1005 → 1012 | 左轮输出命令增益 +5% 等记录头修改 |\n'
      '| [1013_tracking.patch](1013_tracking.patch) | 1012 → 1013 | 当前基线：增强行驶阻尼、条件性同步释放 |\n'
      '| [1014_rejected.patch](1014_rejected.patch) | 1013 → 1014 | 已撤回，保留作失败案例，未应用到主工程 |\n'
      '| [1015 实验补丁](../../experiments/startup_1015/startup_1015.patch) | 1013 → 1015 | 未实机验证，独立实验目录 |\n\n'
      '补丁来自各自构建时保存的 App/Control 源码快照，不是凭聊天重写。只有对应历史基线才适合应用前三个补丁；查阅它们不需要改变当前主工程。\n')

def validation():
    host=json.loads((ROOT/'build/host_tests/results.json').read_text(encoding='utf-8'))
    assert all(t['passed'] for t in host['suites'])
    original=json.loads((ROOT/'docs/releases/1013_source_manifest.json').read_text(encoding='utf-8'))
    built=ROOT/'build/keil/MyLegwheel.hex'
    digest=hashlib.sha256(built.read_bytes()).hexdigest()
    expected=original['sha256']['MyLegwheel_tracking_1013.hex']
    assert digest==expected,f'Rebuilt HEX differs from frozen 1013: {digest}'
    core={}
    for rel,expected_hash in original['source_sha256'].items():
        if rel.startswith(('App/','Control/')):
            actual=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
            assert actual==expected_hash,rel;core[rel]=actual
    report={'date':'2026-10-12','baseline_build_id':2026101013,
      'compiled_sources':89,'hex_sha256':digest,'matches_frozen_1013_hex':True,
      'controller_source_sha256':core,'host_suites':host['suites'],'board_accessed':False,
      'usb_dependency_in_public_bundle':False}
    write(ROOT/'docs/releases/validation.json',json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    write(ROOT/'docs/releases/validation.md',
      '# 开源包发布验证\n\n2026-10-12 在独立发布目录验证，未访问或烧录主控。\n\n'
      '- Keil 工程检查通过：17 组、106 个注册文件、包含路径、命名与 options 对齐。\n'
      '- ARM Compiler 6.16 重编译 89 个源文件通过。\n'
      '- 生成 HEX 与冻结 1013 **逐字节相同**（通过 SHA-256 核对）。\n'
      '- 四项主机套件通过：wheel leg control、motion tracking、remote motion/decoder、startup recorder。\n'
      '- 主控 App/Control 快照哈希核对通过，默认记录 build ID 为 2026101013。\n\n'
      f'HEX SHA-256：`{digest}`。AXF 含构建位置调试信息，不要求与旧路径下的 AXF 二进制一致。\n\n'
      '验证时 USB 依赖由原库按哈希装入本地忽略目录；公开源包不包含该库实现或 HEX/AXF。\n\n'
      '1015 默认工程未启用；其源码来自已完成历史回归/编译的候选快照，仍需实机验证。1013 居中启动后退与跟随不足为已知问题，未在打包过程中修改控制策略。\n\n'
      '机器可读结果：[validation.json](validation.json)。工程的历史实机数据见 [调试索引](../debugging/index.md)。\n')

def main():
    patches();index_debugging();indexes();validation()
    # Data provenance supplements for the earliest feedback checks.
    # These JSON files are intentionally kept outside the main trace catalog.
    forbidden={'build','.git','__pycache__','DebugConfig'}
    files=sorted(p for p in ROOT.rglob('*') if p.is_file() and
      not any(part in forbidden for part in p.relative_to(ROOT).parts) and
      not p.relative_to(ROOT).is_relative_to('Middlewares/ST/STM32_USB_Device_Library') and
      p.suffix not in ('.pyc','.o','.hex','.axf','.bin','.log','.zip') and
      not p.name.startswith('SHA256SUMS'))
    figure_count=len(json.loads((ROOT/'docs/figures/manifest.json').read_text(encoding='utf-8')))
    assert len([p for p in files if p.suffix=='.png'])==figure_count
    assert not any('ST/STM32_USB_Device_Library/' in p.relative_to(ROOT).as_posix() for p in files)
    links=[]
    for p in files:
        if p.suffix=='.md':
            for ref in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
                if '://' in ref or ref.startswith('#'):continue
                if not (p.parent/ref.split('#')[0]).exists():links.append((str(p.relative_to(ROOT)),ref))
    # Historical prose keeps old relative links as evidence; current docs must resolve.
    new_errors=[e for e in links if not e[0].replace('\\','/').startswith('docs/history/')]
    assert not new_errors,new_errors
    checks=''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.relative_to(ROOT).as_posix()+'\n' for p in files)
    write(ROOT/'SHA256SUMS.txt',checks);files.append(ROOT/'SHA256SUMS.txt')
    output=WORKSPACE/'build/wheel-legged-stm32_1013_20261012.zip'
    with zipfile.ZipFile(output,'w',zipfile.ZIP_DEFLATED) as archive:
        for p in files:archive.write(p,ROOT.name+'/'+p.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(output) as archive:
        assert archive.testzip() is None
        for p in files:assert archive.read(ROOT.name+'/'+p.relative_to(ROOT).as_posix())==p.read_bytes()
    ziphash=hashlib.sha256(output.read_bytes()).hexdigest()
    write(output.with_suffix('.zip.sha256'),ziphash+'  '+output.name+'\n')
    print(json.dumps({'zip':str(output),'files':len(files),'size_mb':round(output.stat().st_size/1024**2,2),
      'sha256':ziphash,'figures':figure_count,'matches_1013_hex':True},indent=2))

if __name__=='__main__':main()
