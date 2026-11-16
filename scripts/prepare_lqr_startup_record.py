"""Back up the committed record, then prepare the logger while all outputs/CH5 are OFF.

No motor enable, reset, halt, firmware write or Flash erase is issued.
"""
import argparse
import json
from pathlib import Path
import re
import struct
import subprocess
import tempfile

from read_lqr_startup_record import (GDB,OCD,FLASH,HEADER,check_firmware,header_decode,
                                      read_blocks,record_crc,export)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axf',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='New JSON backup destination; must not already exist')
    parser.add_argument('--openocd',default=OCD)
    args=parser.parse_args()
    if any(args.output.with_suffix(ext).exists() for ext in ('.json','.csv','.bin')):
        parser.error('Choose a new backup filename; existing trials will not be overwritten')
    fields=('g_robot_control.enable','g_dual_hold.enable','g_balance.enable','g_wheel_test.enable',
            'g_wheel_test.zero_request','g_robot_control.remote_enable','g_robot_control.remote_switch_on',
            'g_lqr_record.storage_busy','g_lqr_record.state','g_lqr_record.count','g_lqr_record.rearm_request')
    command=[GDB,'-nx','-batch',str(args.axf)]
    for name in fields:command+=['-ex','p/x &'+name]
    result=subprocess.run(command,capture_output=True,text=True,check=True)
    addresses=[int(v,16) for v in re.findall(r'\$\d+ = (0x[0-9a-f]+)',result.stdout)]
    assert len(addresses)==len(fields)
    check_firmware(args.axf.with_suffix('.hex'),args.openocd)
    def snapshot():
        raw=read_blocks([(str(i),at,1) for i,at in enumerate(addresses)],args.openocd)
        return dict(zip(fields,[struct.unpack('<I',raw[str(i)])[0] for i in range(len(fields))]))
    def guard(state):
        assert all(state[name]==0 for name in fields[:5]) and state['g_lqr_record.storage_busy']==0, 'Outputs/storage must be OFF'
        assert not state['g_robot_control.remote_enable'] or not state['g_robot_control.remote_switch_on'], 'CH5 must be OFF'
        assert state['g_lqr_record.state']==3, 'Wait for a successful save; do not discard unsaved/failed RAM records'
    before=snapshot();guard(before)
    first=read_blocks([('header',FLASH,HEADER.size//4)],args.openocd)['header']
    header=header_decode(first)
    assert before['g_lqr_record.count']==header['count'], 'RAM/Flash counts differ; refuse to discard a newer trial'
    length=header['payload_offset']+header['count']*header['row_bytes']
    raw=read_blocks([('record',FLASH,length//4),('header_after',FLASH,HEADER.size//4)],args.openocd)
    assert raw['record'][:128]==first==raw['header_after'], 'Saved record changed during backup'
    config=raw['record'][128:header['payload_offset']]
    rows=raw['record'][header['payload_offset']:]
    assert record_crc(first,rows,config)==header['crc32'], 'CRC mismatch; refuse to rearm'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.with_suffix('.bin').write_bytes(raw['record'])
    export(header,rows,args.output,'committed Flash, CRC verified before rearm',config)
    guard(snapshot())
    request=addresses[-1]
    with tempfile.TemporaryDirectory(prefix='lqr_prepare_') as folder:
        cfg=Path(folder)/'prepare.cfg'
        cfg.write_text(f'init\nwrite_memory {request:#x} 32 {{1}}\nshutdown\n',encoding='ascii')
        result=subprocess.run([args.openocd,'-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid',
            '-c','transport select swd','-c','adapter speed 1000','-f','target/stm32h7x.cfg','-f',str(cfg)],
            capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stdout+result.stderr
    after=snapshot()
    assert all(after[name]==0 for name in fields[:5]) and not after['g_lqr_record.storage_busy']
    assert after['g_lqr_record.state']==0 and after['g_lqr_record.count']==0 and after['g_lqr_record.rearm_request']==0
    assert read_blocks([('header',FLASH,HEADER.size//4)],args.openocd)['header']==first
    status={'before':before,'after':after,'backup':str(args.output.resolve()),
            'preserved_generation':header['generation'],'preserved_crc32':header['crc32'],
            'motor_enable_written':False,'flash_erase_or_program_issued':False}
    args.output.with_suffix('.ready.json').write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
    print('READY: logger state=0/count=0, all owners OFF, saved Flash record retained.')

if __name__=='__main__':main()
