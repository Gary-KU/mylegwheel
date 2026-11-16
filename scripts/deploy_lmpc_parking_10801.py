"""Back up stopped LQR hardware, program 10801, verify bytes and OFF defaults."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import struct
import subprocess
import time
from read_lqr_startup_record import GDB,OCD,FLASH,read_blocks,check_firmware,header_decode,record_crc,export

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'build/parking_lean_entry_20261008/MyLegwheel_parking_lean_entry.axf'
NEW=ROOT.parent/'H723_LMPC/build/keil/H723_LMPC.axf'
STAMP=time.strftime('%Y%m%d_%H%M%S')
PREFIX=ROOT/'diagnostics'/('h723_lmpc_parking_10801_'+STAMP)
STATE=ROOT/'build/lmpc_parking_10801_deployment_state.json'
FIELDS=['g_robot_control.enable','g_robot_control.status','g_robot_control.remote_enable',
 'g_robot_control.remote_switch_on','g_robot_control.remote_ch5_raw','g_robot_control.remote_link_ok',
 'g_robot_control.feedback_only','g_dual_hold.enable','g_balance.enable','g_wheel_test.enable',
 'g_wheel_test.zero_request']
OLD_FIELDS=FIELDS+['g_lqr_record.state','g_lqr_record.count','g_lqr_record.storage_busy']
NEW_FIELDS=FIELDS+['g_balance.parking_lean_enable','g_balance.parking_lean_gain_rad_per_m',
 'g_balance.parking_damping_gain_nm_per_m_s','g_balance.parking_correction_limit_m',
 'g_balance.wheel_scale','g_balance.leg_scale','g_balance.parking_lean_rad',
 'g_lmpc_control.mode','g_lmpc.status','SystemCoreClock',
 'g_wheel_leg.period_ms','g_wheel_leg.stack_free_min_bytes']
FLOATS={f for f in NEW_FIELDS if f.startswith('g_balance.') and f not in
 {'g_balance.enable','g_balance.parking_lean_enable'}}

def safe_read(blocks):
    for attempt in range(3):
        try:return read_blocks(blocks,OCD)
        except RuntimeError:
            if attempt==2:raise
            time.sleep(.5)
def verify_image(path):
    for attempt in range(3):
        try:return check_firmware(path,OCD)
        except RuntimeError as error:
            if attempt==2 or 'differs from this HEX' in str(error):raise
            time.sleep(.5)

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def layout(axf,names,arrays=()):
    expressions=['p/x &'+name for name in names]+['p sizeof('+name+')' for name in arrays]
    command=[GDB,'-nx','-batch',str(axf)]
    for expression in expressions:command+=['-ex',expression]
    result=subprocess.run(command,capture_output=True,text=True,check=True,timeout=30)
    values=re.findall(r'\$\d+ = (0x[0-9a-f]+|\d+)',result.stdout)
    if len(values)!=len(expressions):raise RuntimeError(result.stdout+result.stderr)
    return dict(zip(names,[int(v,0) for v in values[:len(names)]])),dict(zip(arrays,[int(v,0) for v in values[len(names):]]))
def snapshot(axf,fields):
    addresses,_=layout(axf,fields+['g_wheel_output.voltage_mv','g_joint_output.voltage_mv'])
    blocks=[(str(i),addresses[name],1) for i,name in enumerate(fields)]
    blocks += [('wheel',addresses['g_wheel_output.voltage_mv'],1),('joint',addresses['g_joint_output.voltage_mv'],2)]
    raw=safe_read(blocks)
    values={name:struct.unpack('<f' if name in FLOATS else '<I',raw[str(i)])[0] for i,name in enumerate(fields)}
    values['wheel_voltage_mv']=list(struct.unpack('<2h',raw['wheel']))
    values['joint_voltage_mv']=list(struct.unpack('<4h',raw['joint']))
    return values
def guard(values):
    for name in ['g_robot_control.enable','g_dual_hold.enable','g_balance.enable',
                 'g_wheel_test.enable','g_wheel_test.zero_request']:
        if values[name]!=0:raise RuntimeError(f'Outputs are active: {name}={values[name]}; close CH5 before flashing')
    if values['g_robot_control.remote_switch_on']!=0:
        raise RuntimeError('CH5 is ON; close CH5 before flashing')
    if any(values['wheel_voltage_mv']+values['joint_voltage_mv']):raise RuntimeError('Nonzero motor voltage; refuse to reset')
    if values.get('g_lqr_record.storage_busy',0):raise RuntimeError('Record Flash save still busy; wait until saved')
    if values.get('g_lqr_record.state')==1:raise RuntimeError('Recorder still running; wait for frozen/saved state')
def ocd(commands,label):
    cfg=PREFIX.with_suffix('.'+label+'.cfg');cfg.write_text('\n'.join(commands)+'\n',encoding='ascii')
    result=subprocess.run([OCD,'-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid',
        '-c','transport select swd','-c','adapter speed 1000','-f','target/stm32h7x.cfg','-f',str(cfg)],
        capture_output=True,text=True,timeout=90)
    output=result.stdout+result.stderr
    PREFIX.with_suffix('.'+label+'.log').write_text(output,encoding='utf-8')
    if result.returncode:raise RuntimeError(output[-3000:])
    return output
def hex_extent(path):
    base=0;start=None;end=0
    for line in path.read_text(encoding='ascii').splitlines():
        row=bytes.fromhex(line[1:])
        if sum(row)&255:raise RuntimeError('Invalid HEX checksum')
        if row[3]==4:base=int.from_bytes(row[4:6],'big')<<16
        elif row[3]==0:
            at=base+int.from_bytes(row[1:3],'big');start=at if start is None else min(start,at);end=max(end,at+row[0])
    if start!=0x08000000 or end>=FLASH:raise RuntimeError('Firmware overlaps reserved log Flash')
    return start,end
def prepare():
    verify_image(OLD.with_suffix('.hex'))
    before=snapshot(OLD,OLD_FIELDS);guard(before)
    start,end=hex_extent(NEW.with_suffix('.hex'))
    manifest=json.loads((ROOT/'diagnostics/lmpc_parking_10801_installed_20261008.json').read_text())
    for suffix in ['.hex','.axf']:
        item=next(v for v in manifest['files'] if v['path']=='build/keil/H723_LMPC'+suffix)
        if digest(NEW.with_suffix(suffix))!=item['after']:raise RuntimeError('Candidate changed since validated installation')
    names=['g_balance_trace','g_lqr_record_data'];addresses,sizes=layout(OLD,names,names)
    raw_log=PREFIX.with_suffix('.preserved_flash.bin')
    raw_code=PREFIX.with_suffix('.previous_firmware.bin')
    raw_log.write_bytes(safe_read([('flash',FLASH,0x40000//4)])['flash'])
    raw_code.write_bytes(safe_read([('code',0x08000000,0x40000//4)])['code'])
    backups={}
    for name in names:
        file=PREFIX.with_suffix('.'+name+'.bin');backups[name]=str(file)
        file.write_bytes(safe_read([('ram',addresses[name],sizes[name]//4)])['ram'])
    after=snapshot(OLD,OLD_FIELDS);guard(after)
    for name in OLD_FIELDS:
        if before[name]!=after[name]:raise RuntimeError(f'State changed during backup: {name}')
    log=raw_log.read_bytes();saved=None
    try:
        header=header_decode(log[:128]);length=header['payload_offset']+header['count']*header['row_bytes']
        config=log[128:header['payload_offset']];rows=log[header['payload_offset']:length]
        if record_crc(log[:128],rows,config)!=header['crc32']:raise RuntimeError('Saved LQR record CRC mismatch')
        output=PREFIX.with_suffix('.previous_record.json');export(header,rows,output,'CRC-verified Flash backup before LMPC deployment',config)
        saved=dict(generation=header['generation'],crc32=header['crc32'],count=header['count'],output=str(output))
    except RuntimeError as error:
        if 'header' not in str(error):raise
    report=dict(prefix=str(PREFIX),old_axf=str(OLD),new_axf=str(NEW),firmware=str(NEW.with_suffix('.hex')),
        firmware_sha256=digest(NEW.with_suffix('.hex')),axf_sha256=digest(NEW),firmware_end=hex(end),
        before=before,preserved_flash=str(raw_log),preserved_flash_sha256=digest(raw_log),
        previous_firmware=str(raw_code),ram_backups=backups,saved_record=saved,prepared=True,
        programmed=False,motors_started_by_agent=False)
    STATE.write_text(json.dumps(report,indent=2),encoding='utf-8')
    PREFIX.with_suffix('.prepare.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
def flash():
    global PREFIX
    report=json.loads(STATE.read_text());PREFIX=Path(report['prefix'])
    if report['programmed']:raise RuntimeError('This deployment is already programmed')
    if digest(NEW)!=report['axf_sha256'] or digest(NEW.with_suffix('.hex'))!=report['firmware_sha256']:
        raise RuntimeError('Validated firmware changed')
    verify_image(OLD.with_suffix('.hex'))
    guard(snapshot(OLD,OLD_FIELDS))
    output=ocd(['init','reset halt',f'program {{{NEW.with_suffix(".hex").as_posix()}}} verify',
                'reset run','shutdown'],'program')
    if 'Verified OK' not in output:raise RuntimeError('OpenOCD did not confirm verification')
    report['programmed']=True;STATE.write_text(json.dumps(report,indent=2),encoding='utf-8')
    verify_image(NEW.with_suffix('.hex'))
    postlog=PREFIX.with_suffix('.after_flash.bin')
    postlog.write_bytes(safe_read([('flash',FLASH,0x40000//4)])['flash'])
    if digest(postlog)!=report['preserved_flash_sha256']:raise RuntimeError('Reserved LQR log Flash changed')
    after=snapshot(NEW,NEW_FIELDS);guard(after)
    for name,expected in [('g_balance.parking_lean_gain_rad_per_m',.05),
                         ('g_balance.parking_damping_gain_nm_per_m_s',.01),
                         ('g_balance.parking_correction_limit_m',.1),
                         ('g_balance.wheel_scale',1),('g_balance.leg_scale',1)]:
        if abs(after[name]-expected)>1e-6:raise RuntimeError(f'Wrong flashed parameter: {name}')
    if after['g_robot_control.feedback_only']!=0 or after['g_balance.parking_lean_enable']!=1:
        raise RuntimeError('Unexpected control profile')
    report.update(full_image_readback_matched=True,reserved_flash_preserved=True,after=after,
                  deployment_complete=True,physical_stability_verified=False)
    STATE.write_text(json.dumps(report,indent=2),encoding='utf-8')
    PREFIX.with_suffix('.flashed.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--mode',choices=['prepare','flash'],required=True)
    args=parser.parse_args();{'prepare':prepare,'flash':flash}[args.mode]()
