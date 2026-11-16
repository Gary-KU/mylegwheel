"""Read running shadow metrics; mutate only synthetic input and cold-start selectors."""
from pathlib import Path
import argparse
import json
import re
import socket
import subprocess
import time
import statistics
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--axf',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
gdb='D:/app/clion/clion+stm32/gcc-arm-none-eabi-10.3-2021.10-win32/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gdb.exe'
cmd=[gdb,'-nx','-batch',str(args.axf)]
expressions=['p/x &g_lmpc_shadow','p/x &g_robot_control','p/x &SystemCoreClock',
             'p/x &g_robot_telemetry','p sizeof(g_lmpc_shadow)']
for expression in expressions:cmd+=['-ex',expression]
result=subprocess.run(cmd,capture_output=True,text=True,check=True)
values=[int(v,0) for v in re.findall(r'\$\d+ = (0x[0-9a-f]+|\d+)',result.stdout)]
assert len(values)==5 and values[-1]==124
shadow,master,clock,telemetry,_=values
s=socket.create_connection(('127.0.0.1',6666),3);s.settimeout(5)
def tcl(command):
    s.sendall(command.encode()+b'\x1a');out=b''
    while not out.endswith(b'\x1a'):
        part=s.recv(65536)
        if not part:raise RuntimeError('Debug connection closed')
        out+=part
    return out[:-1].decode()
def read(address,count):
    values=[]
    while count:
        take=min(count,8)
        values.extend(int(v,0) for v in tcl(f'read_memory {address:#x} 32 {take}').split())
        address+=4*take;count-=take
    return values
keys=['input_mode','cold_start','input_valid','solve_status','attempts','skipped',
      'iterations','solve_us','max_solve_us','mean_solve_us','cycle_us','max_cycle_us',
      'overruns','period_ms','zero_frames','zero_failures']
def snapshot():
    # Full 16-word metrics are read in chunks and may cross a 5ms update.
    # Current solve values are published together in one producer copy;
    # distributions and monotone counter differences do not imply atomicity.
    return dict(zip(keys,read(shadow,16)))
def select(mode,cold):
    # No changes to robot enable, feedback_only, voltage, IRQs or clocks.
    assert read(master,2)==[0,0]
    # Initial deployed image snapshots its selector fields into the report.
    # Retry until the new selectors survive a full completed solve cycle.
    # This changes benchmark input only; the compile-time zero-CAN barrier stays.
    for _ in range(50):
        tcl(f'mww {shadow:#x} {mode}')
        tcl(f'mww {shadow+4:#x} {cold}')
        time.sleep(.012)
        if read(shadow,2)==[mode,cold]:
            time.sleep(.15)
            if read(shadow,2)==[mode,cold]:return
    raise RuntimeError('Benchmark selector did not stick')
report={'axf':str(args.axf.resolve()),'core_clock_hz':read(clock,1)[0],
        'SCB_CCR':read(0xe000ed14,1)[0],'master_before':read(master,2),
        'scope':'synthetic-input CPU profiling; no actuation, no halt',
        'phases':[]}
try:
    assert report['master_before']==[0,0]
    for name,mode,cold in [('small_warm',1,0),('small_cold',1,1),('stress_160',2,0)]:
        select(mode,cold);time.sleep(.25)
        start=snapshot();rows=[];seen=set()
        for _ in range(60):
            row=snapshot()
            if row['attempts'] not in seen:
                rows.append(row);seen.add(row['attempts'])
            time.sleep(.04)
        end=snapshot()
        valid=[r for r in rows if r['input_valid'] and r['input_mode']==mode and r['cold_start']==cold]
        assert valid, (name,end)
        report['phases'].append(dict(name=name,samples=valid,
            solve_us_mean=statistics.mean(r['solve_us'] for r in valid),
            solve_us_min=min(r['solve_us'] for r in valid),
            solve_us_max=max(r['solve_us'] for r in valid),
            cycle_us_max=max(r['cycle_us'] for r in valid),
            iterations_min=min(r['iterations'] for r in valid),iterations_max=max(r['iterations'] for r in valid),
            statuses=sorted(set(r['solve_status'] for r in valid)),
            attempts_delta=end['attempts']-start['attempts'],
            overruns_delta=end['overruns']-start['overruns'],
            zero_failures_delta=end['zero_failures']-start['zero_failures']))
    select(1,0);time.sleep(.1)
    report['final']=snapshot();report['master_after']=read(master,2)
    report['telemetry']=read(telemetry,14)
finally:
    try:select(1,0)
    finally:s.close()
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({**{k:v for k,v in report.items() if k not in ('phases','telemetry')},
    'phases':[{k:v for k,v in phase.items() if k!='samples'} for phase in report['phases']]},indent=2))
