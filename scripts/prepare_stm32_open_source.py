"""Stage the frozen 1013 source and curate its debugging history."""
from __future__ import annotations
import difflib
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import stat
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'projects' / 'wheel-legged-stm32'
USB = Path('Middlewares/ST/STM32_USB_Device_Library')
FROZEN = ROOT / 'build/tracking_1013_20261011'
EXPERIMENT = ROOT / 'build/startup_1015_20261011'

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if destination.read_bytes()==source.read_bytes():return
        assert destination.resolve().is_relative_to(DEST.resolve())
        destination.chmod(stat.S_IREAD | stat.S_IWRITE)
    shutil.copy2(source, destination)

def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(value.encode('utf-8'))

def scrub(value):
    if isinstance(value, dict):
        return {k: scrub(v) for k, v in value.items()}
    if isinstance(value, list):
        return [scrub(v) for v in value]
    if isinstance(value, str):
        # Remove machine-specific metadata, while retaining measurements.
        return re.sub(r'(?<![A-Za-z])[A-Za-z]:[\\/][^\n\r\"<>]*',
                      lambda m: '<local-path>/' + m[0].replace('\\','/').rsplit('/',1)[-1], value)
    return value

def json_export(source, destination):
    data = json.loads(source.read_text(encoding='utf-8-sig'))
    write(destination, json.dumps(scrub(data), ensure_ascii=False, indent=2) + '\n')

def make_patch(before, after, output, paths):
    diff = []
    for rel in paths:
        a, b = before / rel, after / rel
        if not a.is_file() or not b.is_file():
            continue
        old = a.read_text(encoding='utf-8-sig').splitlines(keepends=True)
        new = b.read_text(encoding='utf-8-sig').splitlines(keepends=True)
        diff.extend(difflib.unified_diff(old,new,fromfile='a/'+rel,tofile='b/'+rel))
    write(output, ''.join(diff).replace('\r\n','\n'))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume',action='store_true',help='Complete a partial staging run without deleting files')
    args=parser.parse_args()
    if DEST.exists() and not args.resume:
        raise FileExistsError('Refusing to overwrite an existing release directory')
    DEST.mkdir(parents=True,exist_ok=True)
    for directory in ('Core','App','BSP','Algorithms','Control','USB_DEVICE'):
        for source in (ROOT/directory).rglob('*'):
            if source.is_file() and source.suffix.lower() in ('.c','.h'):
                copy(source, DEST/source.relative_to(ROOT))
    # Include compiler-registered sources, excluding the separately installed USB library.
    project = ET.parse(ROOT/'MDK-ARM/MyLegwheel.uvprojx')
    for node in project.findall('.//Groups/Group/Files/File'):
        source=(ROOT/'MDK-ARM'/node.findtext('FilePath').replace('\\','/')).resolve()
        rel=source.relative_to(ROOT)
        if not rel.is_relative_to(USB):
            copy(source,DEST/rel)
    for directory in ('Drivers/STM32H7xx_HAL_Driver/Inc','Drivers/CMSIS/Include',
                      'Middlewares/Third_Party/FreeRTOS/Source/include',
                      'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F'):
        for source in (ROOT/directory).rglob('*'):
            if source.is_file() and source.suffix.lower() in ('.c','.h'):
                copy(source,DEST/source.relative_to(ROOT))
    device=Path('Drivers/CMSIS/Device/ST/STM32H7xx')
    for name in ('stm32h7xx.h','stm32h723xx.h','system_stm32h7xx.h'):
        copy(ROOT/device/'Include'/name,DEST/device/'Include'/name)
    for rel in ('Drivers/STM32H7xx_HAL_Driver/LICENSE.txt','Drivers/CMSIS/LICENSE.txt',
                'Drivers/CMSIS/Device/ST/STM32H7xx/LICENSE.txt',
                'MDK-ARM/MyLegwheel.uvprojx','MDK-ARM/MyLegwheel.uvoptx','MDK-ARM/MyLegwheel.sct',
                'MDK-ARM/RTE/_MyLegwheel/RTE_Components.h','MyLegwheel.ioc'):
        copy(ROOT/rel,DEST/rel)
    copy(ROOT.parents[1]/'LICENSE',DEST/'LICENSE')
    # Restore every application snapshot file, not just the version number.
    for source in (FROZEN/'sources').rglob('*'):
        if source.is_file() and source.relative_to(FROZEN/'sources').parts[0] in ('App','Control','tests','scripts'):
            copy(source,DEST/source.relative_to(FROZEN/'sources'))
    for rel in ('scripts/build_keil.py','scripts/check_project_layout.py','scripts/decode_balance_trace_1007.py',
                'scripts/prepare_lqr_startup_record.py','scripts/robot_telemetry_protocol.py',
                'scripts/f103_feedback_patch.py','scripts/f103_wheel_voltage_patch.py','scripts/f103_joint_boot_patch.py'):
        copy(ROOT/rel,DEST/rel)
    # The independent host suite helpers are required by the four release suites.
    for source in (ROOT/'tests/support').rglob('*'):
        if source.is_file() and source.suffix in ('.c','.h','.md'):
            copy(source,DEST/source.relative_to(ROOT))
    for name in ('verify_wheel_leg_math.py','verify_parking_damping.py','verify_robot_telemetry.py',
                 'verify_robot_motion_command.py','verify_can_feedback.py','verify_sbus.py','verify_sbus_sticks.py'):
        if not (DEST/'tests'/name).exists():copy(ROOT/'tests'/name,DEST/'tests'/name)
    manifest=json.loads((FROZEN/'manifest.json').read_text(encoding='utf-8'))
    for rel, expected in manifest['source_sha256'].items():
        p=DEST/rel
        if p.exists():assert digest(p)==expected,rel
    json_export(FROZEN/'manifest.json',DEST/'docs/releases/1013_source_manifest.json')
    json_export(EXPERIMENT/'manifest.json',DEST/'experiments/startup_1015/source_manifest.json')
    for rel in ('App/Tasks/Src/wheel_leg_task.c','App/Tasks/Inc/wheel_leg_config.h',
                'App/Services/Inc/lqr_startup_record.h','tests/verify_motion_tracking.py',
                'tests/verify_parking_lean.py','scripts/read_lqr_startup_record.py'):
        copy(EXPERIMENT/'sources'/rel, DEST/'experiments/startup_1015/overlay'/rel)
    make_patch(FROZEN/'sources',EXPERIMENT/'sources',
               DEST/'experiments/startup_1015/startup_1015.patch',
               ['App/Tasks/Src/wheel_leg_task.c','App/Tasks/Inc/wheel_leg_config.h',
                'App/Services/Inc/lqr_startup_record.h','tests/verify_motion_tracking.py',
                'tests/verify_parking_lean.py','scripts/read_lqr_startup_record.py'])
    # Vendor USB is intentionally an external dependency. Hashes let users
    # supply the same version from their own ST/Cube installation.
    dependencies=[]
    for source in (ROOT/USB).rglob('*'):
        if source.is_file() and source.suffix.lower() in ('.c','.h','.txt'):
            dependencies.append({'path':source.relative_to(ROOT/USB).as_posix(),
                                 'sha256':digest(source)})
    write(DEST/'dependencies/st_usb_device.json',json.dumps({'component':'STM32_USB_Device_Library',
          'license':'SLA0044 Rev5 as found in source component','files':dependencies},indent=2)+'\n')
    copy(ROOT/USB/'LICENSE.txt',DEST/'licenses/ST_USB_SLA0044.txt')
    # Preserve the full FreeRTOS grant separately as well as in its headers.
    text=(ROOT/'Middlewares/Third_Party/FreeRTOS/Source/tasks.c').read_text(encoding='utf-8')
    write(DEST/'licenses/FreeRTOS_MIT.txt',text.split('*/',1)[0]+'*/\n')
    # Historical documents retain their original statements and dates; the
    # curated index distinguishes them from current configuration.
    for source in (ROOT/'docs').iterdir():
        if source.is_file() and source.suffix in ('.txt','.md') and not source.name.startswith('lmpc'):
            raw=source.read_bytes()
            try:text=raw.decode('utf-8-sig')
            except UnicodeDecodeError:text=raw.decode('gb18030')
            text=re.sub(r'(?<![A-Za-z])[A-Za-z]:[\\/][^\n\r\"<>]*',
                        lambda m:'<local-path>/'+m[0].replace('\\','/').rsplit('/',1)[-1],text)
            write(DEST/'docs/history'/source.name,text.replace('\r\n','\n'))
    images=[]
    for source in sorted((ROOT/'diagnostics').rglob('*')):
        if source.suffix.lower() in ('.png','.jpg','.jpeg'):
            rel=source.relative_to(ROOT/'diagnostics')
            copy(source,DEST/'docs/figures'/rel)
            images.append({'source':'diagnostics/'+rel.as_posix(),'path':'docs/figures/'+rel.as_posix(),
                           'sha256':digest(source)})
    write(DEST/'docs/figures/manifest.json',json.dumps(images,ensure_ascii=False,indent=2)+'\n')
    # Import readable exports and analysis, not arbitrary RAM/probe dumps.
    datasets=[]
    for source in sorted((ROOT/'diagnostics').rglob('*')):
        if not source.is_file():continue
        rel=source.relative_to(ROOT/'diagnostics')
        selected=(source.name in ('record.csv','record.json','analysis.json','ram_tail.json') and
                  rel.parts[0].startswith(('tracking_review','spin_review','remote_response','yaw_fall')))
        selected=selected or (len(rel.parts)==1 and source.suffix in ('.json','.csv') and
                 (source.name.startswith(('lqr_','parking_','stable_drift_','wheel_030nm')) and
                  any(k in source.name for k in ('analysis','_trial_','RAM_tail','ram_tail'))))
        if not selected:continue
        target=DEST/'docs/data'/rel
        if source.suffix=='.json':json_export(source,target)
        else:copy(source,target)
        datasets.append({'source':'diagnostics/'+rel.as_posix(),'path':'docs/data/'+rel.as_posix(),
                         'original_sha256':digest(source),'export_sha256':digest(target)})
    write(DEST/'docs/data/manifest.json',json.dumps(datasets,ensure_ascii=False,indent=2)+'\n')
    # Copy portable plot scripts next to their corresponding data exports.
    for source in (ROOT/'diagnostics').glob('tracking_review*/plot_response.py'):
        rel=source.relative_to(ROOT/'diagnostics')
        copy(source,DEST/'docs/data'/rel)
    versions=[('1012_left_gain','overspeed_1005_20261010','left_output_1012_20261011'),
              ('1013_tracking','left_output_1012_20261011','tracking_1013_20261011'),
              ('1014_rejected','tracking_1013_20261011','tracking_1014_20261011')]
    for label,old,new in versions:
        make_patch(ROOT/'build'/old/'sources',ROOT/'build'/new/'sources',
                   DEST/'docs/changes'/(label+'.patch'),
                   ['App/Tasks/Src/wheel_leg_task.c','App/Tasks/Inc/wheel_leg_config.h',
                    'App/Services/Inc/lqr_startup_record.h'])
    print(json.dumps({'destination':str(DEST),'figures':len(images),'data_files':len(datasets),
          'baseline_source_hashes_checked':True,'usb_external_files':len(dependencies)},indent=2))

if __name__=='__main__':main()
