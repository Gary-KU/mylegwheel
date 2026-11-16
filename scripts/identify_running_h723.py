"""Match running flash fragments to local HEX files, without halt/write/reset."""
from pathlib import Path
import json
import re
import struct
import subprocess
ROOT=Path(__file__).resolve().parents[1]
OCD='D:/app/clion/clion+stm32/openocd-20230621/OpenOCD-20230621-0.12.0/bin/openocd.exe'
addresses=[0x08000000,0x08001000,0x0800bfc8,0x08010000]
commands=['init']
for address in addresses:commands += [f'echo FLASH_{address:x}',f'echo [read_memory {address:#x} 32 8]']
commands += ['echo FLASH_END','echo [read_memory 0x08000000 32 8]','shutdown']
result=subprocess.run([OCD,'-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid',
 '-c','transport select swd','-c','adapter speed 1000','-f','target/stm32h7x.cfg','-c','; '.join(commands)],capture_output=True,text=True,check=True)
output=result.stdout+result.stderr
def parse(label):
 match=re.search(label+r'\s*\n([^\n]+)',output);assert match,output
 values=[int(s,16) for s in match[1].split()];assert len(values)==8
 return struct.pack('<8I',*values)
live={a:parse(f'FLASH_{a:x}') for a in addresses}
assert parse('FLASH_END')==live[addresses[0]],'Boot image changed during the read'
def samples(path):
 found={a:bytearray(b'\xff'*32) for a in addresses};base=0
 for line in path.read_text(encoding='ascii').splitlines():
  raw=bytes.fromhex(line[1:]);length=raw[0];offset=int.from_bytes(raw[1:3],'big');kind=raw[3];payload=raw[4:4+length]
  if kind==4:base=int.from_bytes(payload,'big')<<16
  elif kind==2:base=int.from_bytes(payload,'big')<<4
  elif kind==0:
   start=base+offset;end=start+length
   for address in addresses:
    lo=max(address,start);hi=min(address+32,end)
    if lo<hi:found[address][lo-address:hi-address]=payload[lo-start:hi-start]
 return {a:bytes(v) for a,v in found.items()}
files=list((ROOT/'build').rglob('*.hex'))+list((ROOT.parent/'H723_LMPC/build').rglob('*.hex'))
matches=[]
for path in files:
 if samples(path)==live:matches.append(str(path.resolve()))
report=dict(scope='4 flash fragments (128 bytes), not a full-image verification; no halt/reset/write/flash',
 fragments={hex(a):v.hex() for a,v in live.items()},matches=matches)
(ROOT/'diagnostics/running_h723_identity_current.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
