from pathlib import Path
import subprocess
import json
import struct
import numpy as np
root=Path(__file__).resolve().parents[1]
regions={'gradient':(0x2000628c,40),'upper':(0x200066b4,2),'lower':(0x2000891c,2),
         'delta_lower':(0x2000891c+160,2),'delta_upper':(0x200066b4+160,2),
         'fault_lengths':(0x20000394,2),'live_state':(0x200061cc,12),
         'solver_debug':(0x20005da4,11),'temperature':(0x200001ec,3)}
cmd=['D:/app/clion/clion+stm32/openocd-20230621/OpenOCD-20230621-0.12.0/bin/openocd.exe',
     '-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend hid','-c','transport select swd',
     '-c','adapter speed 1000','-f','target/stm32h7x.cfg']
commands=['init']
for name,(address,count) in regions.items():
    for offset in range(0,count,8):
        commands += ['echo '+name+'_'+str(offset),f'echo [read_memory {address+4*offset:#x} 32 {min(8,count-offset)}]']
commands+=['shutdown'];cmd+=['-c','; '.join(commands)]
r=subprocess.run(cmd,capture_output=True,text=True,check=True)
lines=(r.stdout+'\n'+r.stderr).splitlines();report={}
for name,(address,count) in regions.items():
    words=[]
    for offset in range(0,count,8):
        label=name+'_'+str(offset);i=lines.index(label)
        words += [int(x,0) for x in lines[i+1].split()]
    report[name]=words if name=='solver_debug' else list(struct.unpack('<'+'f'*count,struct.pack('<'+'I'*count,*words)))
ti=report['solver_debug'][2]
tables=np.load(root.parent/'H723_LMPC/models/lmpc_tables.npz')
f=tables['F'][ti];gradient=np.array(report['gradient'])
error,_,_,_=np.linalg.lstsq(f,gradient,rcond=None)
report['inferred_failed_state_error']=error.tolist()
report['gradient_reconstruction_error']=float(np.max(np.abs(f@error-gradient)))
report['scope']='reads only, control already stopped; F pseudoinverse reconstructs last failed state-reference error (previous actual input zero)'
report['solver_cycles_us']=report['solver_debug'][3]/400.0
report['bounds_nm']=(np.array(report['upper'])*tables['limit']).tolist()
path=root/'diagnostics/lmpc_startup_failure_20261007.json'
path.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
