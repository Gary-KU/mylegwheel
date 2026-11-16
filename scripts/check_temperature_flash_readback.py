"""Compare sampled live flash bytes against the old and corrected HEX; no target writes."""
from pathlib import Path
import struct
import json
root=Path(__file__).resolve().parents[1]
live={
    0x080022e0:[0xd9664561,0x70fff64f,0xf04f4281,0xd05f0000,0xb2a41c54,0xbf382c02,0xee01bd10,0xeeb32a10],
    0x08002310:[0x1a02ee21,0x1a04ee81,0x4a2aed9f,0x1a44eeb4,0xfa10eef1,0xed9fdc44,0xeeb44a28,0xeef11a44]
}
def hex_memory(path):
    memory={};base=0
    for line in path.read_text().splitlines():
        record=bytes.fromhex(line[1:]);assert sum(record)%256==0
        kind=record[3];offset=int.from_bytes(record[1:3],'big');payload=record[4:4+record[0]]
        if kind==4:base=int.from_bytes(payload,'big')<<16
        elif kind==0:memory.update({base+offset+i:v for i,v in enumerate(payload)})
    return memory
report={'scope':'sampled running flash bytes read using read_memory; no halt/reset/flash/enable writes',
        'cpu_status':4,'cpu_valid':0,'cpu_samples':0,'cpu_adc_error':0,
        'control_enable':0,'control_fault_reason':11,'lmpc_solves':0,'versions':{}}
for name,path in [('old',root/'build/lmpc_thermal_20261007/build/keil/H723_LMPC.hex'),
                  ('corrected',root.parent/'H723_LMPC/build/keil/H723_LMPC.hex')]:
    memory=hex_memory(path)
    samples={address:list(struct.unpack('<8I',bytes(memory[i] for i in range(address,address+32)))) for address in live}
    matches={hex(address):samples[address]==live[address] for address in live}
    report['versions'][name]={'hex':str(path),'matches':matches,'all_samples_match':all(matches.values())}
assert report['versions']['old']['all_samples_match']
assert not report['versions']['corrected']['all_samples_match']
(root/'diagnostics/remote_red_temperature_flash_readback_20261007.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
