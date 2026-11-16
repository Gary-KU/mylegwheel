from pathlib import Path
import ctypes as ct
import json
import subprocess
root=Path(__file__).resolve().parents[1];stage=root/'build/lmpc_startup_fix_20261007'
text=(stage/'Control/LMPC/Src/lmpc.c').read_text()
text='#include <stdio.h>\nstatic unsigned trace_iteration;\n'+text
text=text.replace('static void remove_constraint(unsigned constraint) {',
    'static void remove_constraint(unsigned constraint) {\nif(trace_iteration==160)printf("REMOVE %u weight %.4g\\n",constraint,polish_weight[constraint]);')
text=text.replace('if(*primal>=PRIMAL_TOL) {',
    'if(*primal>=PRIMAL_TOL) {\nif(trace_iteration==160)printf("ADD %u kind %d violation %.5g\\n",violated,violation_kind,*primal);')
text=text.replace('float certified_primal,certified_dual;',
    'trace_iteration=iteration+1;float certified_primal,certified_dual;')
text=text.replace('unsigned old=anchor_constraint[group];',
    'unsigned old=anchor_constraint[group];\nif(trace_iteration==160)printf("CONFLICT old %u new %u protected %d oldVal %.5g newVal %.5g\\n",old,constraint,protected_index,chain_anchor[group],value);')
source=stage/'build/trace_solver.c';source.write_text(text)
dll=stage/'build/trace_solver.dll'
subprocess.run(['gcc','-std=c99','-O2','-shared','-DLMPC_HOST_TEST',str(source),
               str(stage/'Control/LMPC/Src/lmpc_tables.c'),'-I',str(stage/'Control/LMPC/Inc'),'-o',str(dll)],check=True)
lib=ct.CDLL(str(dll));ptr=ct.POINTER(ct.c_float)
lib.Lmpc_Solve.argtypes=[ct.c_float,ptr,ptr,ptr,ptr,ct.c_void_p,ct.c_uint32]
case=json.loads((root/'diagnostics/lmpc_startup_failure_20261007.json').read_text())
vec=lambda v:(ct.c_float*len(v))(*v)
u=vec([0,0]);state=vec([.5*x for x in case['inferred_failed_state_error']])
print('ok:',lib.Lmpc_Solve(.09,state,vec([0]*6),vec([.021,.002]),u,None,0),'input:',list(u))
