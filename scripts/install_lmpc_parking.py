"""Prepare/review a checksummed LMPC update, then install only that manifest.

Preparation is local. Installation needs filesystem authorization for the
separate H723_LMPC project; it never connects to hardware or flashes a board.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'build/lmpc_parking_20261008'
TARGET=ROOT.parent/'H723_LMPC'
BACKUP=ROOT/'backups/before_lmpc_parking_10801_20261008'
MANIFEST=ROOT/'build/lmpc_parking_10801_install_manifest.json'

SOURCES=[
 'App/Tasks/Inc/wheel_leg_config.h','App/Tasks/Inc/wheel_leg_task.h',
 'App/Tasks/Src/wheel_leg_task.c','App/Services/Src/robot_telemetry_service.c',
 'Control/LMPC/Inc/lmpc_control.h','Control/LMPC/Src/lmpc_control.c',
 'Control/LMPC/Src/lmpc.c','Control/LMPC/Src/lmpc_tables.c',
 'models/lmpc_tables.npz','models/lmpc_parameters.json','scripts/generate_lmpc.py',
 'scripts/run_checks.py','tests/verify_backup_model.py',
 'tests/verify_parking_response.py','tests/verify_parking_supervisor.py',
 'docs/lmpc_parking_10801_20261008.txt','README.md',
]
BASELINE=[
 'Control/LMPC/Inc/lmpc.h','Control/LMPC/Inc/lmpc_tables.h',
 'Control/LMPC/Inc/lmpc_control.h','Control/WheelLeg/Inc/lqr_k.h',
 'Control/LMPC/Src/lmpc.c','Control/LMPC/Src/lmpc_tables.c',
 'Control/LMPC/Src/lmpc_control.c','Control/WheelLeg/Src/lqr_k.c',
]
FIRMWARE=[
 'build/keil/H723_LMPC.hex','build/keil/H723_LMPC.axf',
 'build/keil/build.log','build/keil/listings/H723_LMPC.map',
 'build/read-only/H723_READ_ONLY.hex','build/read-only/H723_READ_ONLY.axf',
 'build/read-only/build.log',
 'build/lmpc-shadow/H723_LMPC_SHADOW.hex','build/lmpc-shadow/H723_LMPC_SHADOW.axf',
 'build/lmpc-shadow/build.log',
 'build/parking_response_validation.json','build/lmpc_host_validation.json',
 'build/lmpc_startup_validation.json','build/lmpc_n10_replay_validation.json',
 'build/backup_model_validation.json',
]
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None

def checked(root,name):
    path=(root/name).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f'Path escaped destination: {path}')
    return path

def baseline():
    for name in BASELINE:
        dst=checked(STAGE/'tests/reference_10708',name)
        if not dst.exists():
            dst.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(checked(TARGET,name),dst)

def prepare():
    baseline()
    if MANIFEST.exists() or BACKUP.exists():
        raise SystemExit('Existing manifest/backup: review rather than overwrite')
    names=SOURCES+['tests/reference_10708/'+n for n in BASELINE]+FIRMWARE
    rows=[]
    for name in names:
        src=checked(STAGE,name);dst=checked(TARGET,name)
        if not src.is_file():raise SystemExit(f'Candidate file missing: {src}')
        old=digest(dst);new=digest(src)
        if old==new:continue
        if dst.is_file():
            saved=checked(BACKUP,name);saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(dst,saved)
            if digest(saved)!=old:raise SystemExit('Backup checksum mismatch')
        rows.append(dict(path=name,before=old,after=new,size=src.stat().st_size))
    report=dict(stage=str(STAGE),target=str(TARGET),backup=str(BACKUP),
        board_flash=False,files=rows)
    MANIFEST.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(manifest=str(MANIFEST),backup=str(BACKUP),
                         files=[r['path'] for r in rows]),indent=2))

def install():
    report=json.loads(MANIFEST.read_text(encoding='utf-8'))
    if report['target']!=str(TARGET) or report['stage']!=str(STAGE):
        raise SystemExit('Unexpected manifest destination')
    for row in report['files']:
        src=checked(STAGE,row['path']);dst=checked(TARGET,row['path'])
        if digest(src)!=row['after'] or digest(dst)!=row['before']:
            raise SystemExit(f'File changed since preparation: {row["path"]}')
        if row['before'] is not None and digest(checked(BACKUP,row['path']))!=row['before']:
            raise SystemExit(f'Backup changed: {row["path"]}')
    for row in report['files']:
        dst=checked(TARGET,row['path']);dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(checked(STAGE,row['path']),dst)
        if digest(dst)!=row['after']:raise SystemExit(f'Installed checksum mismatch: {dst}')
    report['installed']=True
    (ROOT/'diagnostics/lmpc_parking_10801_installed_20261008.json').write_text(
        json.dumps(report,indent=2),encoding='utf-8')
    print(f'PASS: installed and SHA256-verified {len(report["files"])} files; hardware untouched.')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['baseline','prepare','install'],required=True)
    args=parser.parse_args()
    {'baseline':baseline,'prepare':prepare,'install':install}[args.mode]()
