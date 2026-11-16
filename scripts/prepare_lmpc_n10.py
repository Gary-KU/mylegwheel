from pathlib import Path
import json
import re
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent / 'H723_LMPC'
STAGE = ROOT / 'build/lmpc_n10_20261007'
OCD = 'D:/app/clion/clion+stm32/openocd-20230621/OpenOCD-20230621-0.12.0/bin/openocd.exe'

def read_ram():
    reads = [('debug', 0x200063e4, 32), ('lo_first', 0x20008fb4 + 40*4, 2),
             ('hi_first', 0x20006d48 + 40*4, 2), ('fault', 0x20000368, 8)]
    commands = ['init']
    for name, address, count in reads:
        for offset in range(0, count, 8):
            n = min(8, count-offset)
            commands += [f'echo READ_{name}_{offset}', f'echo [read_memory {address+offset*4:#x} 32 {n}]']
    commands += ['shutdown']
    result = subprocess.run([OCD, '-f', 'interface/cmsis-dap.cfg', '-c', 'cmsis_dap_backend hid',
        '-c', 'transport select swd', '-c', 'adapter speed 1000', '-f', 'target/stm32h7x.cfg',
        '-c', '; '.join(commands)], capture_output=True, text=True, check=True)
    output = result.stdout + result.stderr
    words = {}
    for name, address, count in reads:
        values = []
        for offset in range(0, count, 8):
            match = re.search(rf'READ_{name}_{offset}\s*\n([^\n]+)', output)
            assert match, output
            values += [int(s,16) for s in match[1].split()]
        assert len(values) == count
        words[name] = values
    floats = lambda v: list(struct.unpack('<'+'f'*len(v),struct.pack('<'+'I'*len(v),*v)))
    w = words['debug']
    case = dict(version=10704,status=w[0],iterations=w[1],table_index=w[2],
        cycles=w[3],solve_us=w[3]/400,max_us=w[4]/400,solves=w[5],failures=w[6],
        primal=floats(w[7:8])[0],dual=floats(w[8:9])[0],rho_mode=w[11],budget_cycles=w[12],
        polished=w[13],rho=floats(w[14:15])[0],length=floats(w[15:16])[0],
        state=floats(w[16:22]),reference=floats(w[22:28]),limits=floats(w[28:30]),
        reject=w[30],refinements=w[31],lo_first=floats(words['lo_first']),
        hi_first=floats(words['hi_first']),raw=words,
        scope='Running target RAM read only; no halt, reset, write or flash.')
    case['previous_nm'] = [(a+b)*.5*limit for a,b,limit in zip(case['lo_first'],case['hi_first'],[.03,.02])]
    (ROOT/'diagnostics/lmpc_n20_timeout_20261007.json').write_text(json.dumps(case,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in case.items() if k!='raw'},indent=2))
    return case

case=read_ram()
assert not STAGE.exists(),STAGE
shutil.copytree(SOURCE,STAGE,ignore=shutil.ignore_patterns('build','backups','__pycache__'))
(STAGE/'build').mkdir()
shutil.copy2(ROOT/'diagnostics/lmpc_n20_timeout_20261007.json',STAGE/'tests/data/n20_timeout_20261007.json')

p=STAGE/'scripts/generate_lmpc.py'
s=p.read_text(encoding='utf-8').replace('N, NU, NX = 20, 2, 6','N, NU, NX = 10, 2, 6')
s=s.replace("inc.write_text('''", "inc.write_text(f'''")
s=s.replace('#define LMPC_N 20','#define LMPC_N {N}').replace('#define LMPC_VARS 40','#define LMPC_VARS {NU*N}')
s=s.replace('#define LMPC_CONSTRAINTS 80','#define LMPC_CONSTRAINTS {2*NU*N}')
s=s.replace('typedef struct { float H[40][40], F[40][6], G[40][2], W[40][40], WM[40][40], WH[40][40], J[40][40], rho; } LmpcTable;',
            'typedef struct {{ float H[LMPC_VARS][LMPC_VARS], F[LMPC_VARS][6], G[LMPC_VARS][2], W[LMPC_VARS][LMPC_VARS], WM[LMPC_VARS][LMPC_VARS], WH[LMPC_VARS][LMPC_VARS], J[LMPC_VARS][LMPC_VARS], rho; }} LmpcTable;')
s=s.replace("terminal='Q+P at step20; P from nominal DARE; no 1.15 LQR gain scaling'", "terminal=f'Q+P at step{N}; P from nominal DARE; no 1.15 LQR gain scaling'")
s=s.replace("print('Generated 11 length-scheduled 40-variable QPs.')", "print(f'Generated 11 length-scheduled {NU*N}-variable QPs.')")
p.write_text(s,encoding='utf-8')

p=STAGE/'Control/LMPC/Src/lmpc.c';s=p.read_text(encoding='utf-8')
s=s.replace('42+i','LMPC_VARS+2+i').replace('40U','LMPC_VARS').replace('38+(int)axis','(LMPC_VARS-2)+(int)axis')
s=re.sub(r'\b40\b','LMPC_VARS',s);s=re.sub(r'\b80\b','LMPC_CONSTRAINTS',s)
s=re.sub(r'\b39\b','(LMPC_VARS-1)',s);s=re.sub(r'\b38\b','(LMPC_VARS-2)',s)
s=re.sub(r'\b37\b','(LMPC_VARS-3)',s);s=re.sub(r'\b36\b','(LMPC_VARS-4)',s)
p.write_text(s,encoding='utf-8')
p=STAGE/'Control/LMPC/Inc/lmpc.h';s=p.read_text(encoding='utf-8')
s=s.replace('#include <stdint.h>','#include <stdint.h>\n#include "lmpc_tables.h"')
s=s.replace('plan[40],float multipliers[80]','plan[LMPC_VARS],float multipliers[LMPC_CONSTRAINTS]')
p.write_text(s,encoding='utf-8')
for name in ['App/Tasks/Src/wheel_leg_task.c','App/Services/Src/robot_telemetry_service.c']:
    p=STAGE/name;s=p.read_text(encoding='utf-8').replace('10704','10705');p.write_text(s,encoding='utf-8')
for name in ['verify_lmpc.py','verify_lmpc_startup.py']:
    p=STAGE/'tests'/name;s=p.read_text(encoding='utf-8')
    s=s.replace("z=np.load(ROOT/'models/lmpc_tables.npz')", "z=np.load(ROOT/'models/lmpc_tables.npz');NV=z['H'].shape[1];N=NV//2")
    s=s.replace('np.zeros(40)','np.zeros(h.shape[0])').replace('np.ones(40)','np.ones(NV)')
    s=s.replace('step/limit,20','step/limit,N').replace("z['limit'],20","z['limit'],N")
    s=s.replace("z['step']/z['limit'],20","z['step']/z['limit'],N")
    s=s.replace(']*40)',']*NV)').replace(']*80)',']*(2*NV))')
    s=s.replace('iterations<=120','iterations<=160')
    p.write_text(s,encoding='utf-8')
print('Staged horizon 10; no changes to target project or board.')
