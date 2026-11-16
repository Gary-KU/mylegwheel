"""Export the controller-attributed A1 feedback failure after STOP, read only.

The matching AXF/HEX are mandatory; every application byte is checked before
using DWARF addresses. No halt/reset/target writes, rearm or actuator commands.
RAM only: power/reset or the next accepted active run clears this snapshot.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess

from read_lqr_startup_record import GDB, OCD, check_firmware, read_blocks
from read_foc_failure import GUARDS, IDENTITY_BLOCKS, local_image_info, target_identity

PREFIX_FIELDS=('valid','schema_version','controller_tick_ms','feedback_detail','observer_age_ms')
HEALTH_FIELDS=('valid','motor_id','sample_tick_ms','reject_mask','node_epoch','transport_epoch',
    'status_ms','status_count','status_age_ms','diagnostic_ms','diagnostic_count','diagnostic_age_ms',
    'fast_ms','fast_count','fast_age_ms','flags','diagnostic_flags','faults','board_faults','mode',
    'fast_flags','fast_valid','fast_speed_clipped','anchor_confirmed','timing_confirmed',
    'history_configuration_valid','history_generation','history_count','history_epoch',
    'communication_blocked','recovery_state')
FIELDS=PREFIX_FIELDS+tuple('sample.'+name for name in HEALTH_FIELDS)
LAYOUT=struct.Struct('<'+'I'*len(FIELDS))
REJECT_REASONS=('epoch','status_missing','status_stale','diagnostic_missing','diagnostic_stale',
    'status_sensor_invalid','driver_faults','board_faults','diagnostic_health','fast_invalid',
    'fast_speed_clipped','fast_missing','fast_stale','anchor_unconfirmed','timing_unconfirmed',
    'history_configuration','communication_blocked')
DETAILS={0:'ok',1:'imu',2:'motor_offline',3:'stale',4:'joint_calibration',5:'nonfinite',6:'pair_skew'}

def decode_snapshot(raw):
    if len(raw)!=LAYOUT.size:
        raise RuntimeError('Incomplete MotorFocFeedbackFailureDebug snapshot')
    values=dict(zip(FIELDS,LAYOUT.unpack(raw)))
    if values['valid'] not in (0,1):
        raise RuntimeError('Invalid validity; check matching firmware/layout')
    if values['valid'] and (values['schema_version']!=1 or values['sample.valid']!=1 or
                           not 1<=values['sample.motor_id']<=8):
        raise RuntimeError('Unsupported or inconsistent feedback failure schema')
    sample={name:values['sample.'+name] for name in HEALTH_FIELDS}
    mask=sample['reject_mask']
    return dict(bytes=LAYOUT.size,fields={name:values[name] for name in PREFIX_FIELDS},sample=sample,
        feedback_detail_name=DETAILS.get(values['feedback_detail'],'unknown'),
        online_reject_reasons=[name for bit,name in enumerate(REJECT_REASONS) if mask&(1<<bit)],
        unknown_reject_bits=mask&~((1<<len(REJECT_REASONS))-1),
        snapshot_sha256=hashlib.sha256(raw).hexdigest())

def symbols(axf,gdb=GDB):
    expressions=['p/x &g_motor_foc_feedback_failure','p sizeof(MotorFocFeedbackFailureDebug)']
    expressions+=['p (unsigned long)&((MotorFocFeedbackFailureDebug *)0)->'+field for field in FIELDS]
    expressions+=['p/x &'+expression for _,expression in GUARDS]
    cmd=[gdb,'-nx','-batch',str(axf)]
    for expression in expressions:cmd+=['-ex',expression]
    result=subprocess.run(cmd,capture_output=True,text=True,check=True)
    values=[int(v,0) for v in re.findall(r'\$\d+ = (0x[0-9a-fA-F]+|\d+)',result.stdout)]
    if (len(values)!=len(expressions) or values[1]!=LAYOUT.size or
        values[2:2+len(FIELDS)]!=list(range(0,LAYOUT.size,4))):
        raise RuntimeError('AXF feedback failure field offsets differ from decoder')
    return dict(failure=values[0],guards=dict(zip((name for name,_ in GUARDS),values[2+len(FIELDS):])))

def read_stopped_snapshot(addresses,ocd):
    guard_blocks=[(name,address,1) for name,address in addresses['guards'].items()]
    blocks=guard_blocks+[('failure',addresses['failure'],LAYOUT.size//4),
        ('failure_after',addresses['failure'],LAYOUT.size//4)]
    blocks += [(name+'_after',address,count) for name,address,count in guard_blocks]
    blocks += [(name+'_after',address,count) for name,address,count in IDENTITY_BLOCKS]
    raw=read_blocks(blocks,ocd)
    for name,address,count in guard_blocks:
        if raw[name]!=bytes(4) or raw[name+'_after']!=bytes(4):
            raise RuntimeError('Use CH5 OFF with all owners/storage stopped: '+name)
    if raw['failure']!=raw['failure_after']:
        raise RuntimeError('Feedback failure changed during export; retry after STOP')
    return raw['failure'],{name:0 for name in addresses['guards']},target_identity(raw,'_after')

def collect(axf,hex_path,ocd=OCD,gdb=GDB):
    image=local_image_info(axf,hex_path)
    identity_before=target_identity(read_blocks(IDENTITY_BLOCKS,ocd))
    check_firmware(hex_path,ocd)
    raw,guards,identity_after=read_stopped_snapshot(symbols(axf,gdb),ocd)
    if identity_before!=identity_after:
        raise RuntimeError('Target identity changed while reading feedback failure')
    return dict(source=__doc__,image=image,full_application_hex_verified=True,target=identity_after,
        guards=guards,failure=decode_snapshot(raw),
        semantics='First actual controller feedback fault only. sample fields are the original GetFeedback qualification snapshot; observer_age_ms may be recomputed at the end of the group read. A valid snapshot with reject_mask=0 can describe a later observer STALE/CALIBRATION/NONFINITE failure; inspect feedback_detail and observer_age_ms. An age with count=0 is raw tick arithmetic, not an age of a received packet. Flags/counts/stamps/ages are not reread after recovery.',
        limitations='RAM only, cleared by reset or next accepted active run, retained after STOP/recovery/failed starts. valid=0 means no matching controller-attributed A1 snapshot. The snapshot identifies failed qualification but receive-dequeue stamps cannot by themselves distinguish driver scheduling from CAN receive delay.')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axf',type=Path,required=True)
    parser.add_argument('--hex',type=Path,dest='hex_path')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--openocd',default=OCD)
    parser.add_argument('--gdb',default=GDB)
    args=parser.parse_args()
    report=collect(args.axf,args.hex_path or args.axf.with_suffix('.hex'),args.openocd,args.gdb)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
