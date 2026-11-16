from pathlib import Path
import ast
import json
import sys
import numpy as np
root=Path(__file__).resolve().parents[1]
case=json.loads((root/'diagnostics/lmpc_startup_failure_20261007.json').read_text())
data=np.load(root.parent/'H723_LMPC/models/lmpc_tables.npz')
ti=case['solver_debug'][2]
h,f,c=data['H'][ti],np.array(case['gradient']),data['C'][ti]
f=f*(float(sys.argv[1]) if len(sys.argv)>1 else 1.0)
upper=np.r_[np.tile(case['upper'],20),np.tile(data['step']/data['limit'],20)]
lower=-upper
tree=ast.parse((root.parent/'H723_LMPC/tests/verify_lmpc.py').read_text())
node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='oracle')
ns={'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),'<oracle>','exec'),ns)
exact=ns['oracle'](h,f,c,lower,upper)
candidate=np.linalg.solve(h,-f)
for i in range(40):
    prev=candidate[i-2] if i>=2 else 0
    candidate[i]=np.clip(candidate[i],max(lower[i],lower[40+i]+prev),min(upper[i],upper[40+i]+prev))
stationarity=h@candidate+f
certificate=[]
for axis in range(2):
    next_delta=0
    for i in range(38+axis,axis-1,-2):
        multiplier=next_delta-stationarity[i]
        prev=candidate[i-2] if i>=2 else 0
        if candidate[i]==upper[i] and multiplier>=0: next_delta=0
        elif candidate[i]==lower[i] and multiplier<=0: next_delta=0
        elif candidate[i]==upper[40+i]+prev and multiplier>=0: next_delta=multiplier
        elif candidate[i]==lower[40+i]+prev and multiplier<=0: next_delta=multiplier
        elif abs(multiplier)<.04:next_delta=0
        else:certificate.append((i,float(multiplier)))
base=float(data['rho'][ti])
rows=[]
for factor,alpha in [(1,1),(4,1),(8,1),(16,1),(32,1),(64,1),(128,1),(192,1),(256,1),(384,1),
                     (192,1.6),(256,1.6),(384,1.6),(192,1.8),(256,1.8)]:
    rho=base*factor;w=np.linalg.inv(h+rho*c.T@c)
    v=np.linalg.solve(h,-f);z=np.clip(c@v,lower,upper);y=np.zeros(80)
    for iteration in range(2000):
        v=w@(-f+rho*c.T@(z-y))
        cv=c@v;relaxed=alpha*cv+(1-alpha)*z;nxt=np.clip(relaxed+y,lower,upper)
        primal=float(max(abs(cv-nxt)));dual=float(max(abs(rho*c.T@(nxt-z))))
        y+=relaxed-nxt;z=nxt
        if primal<.001 and dual<.04:break
    out=np.clip(v[:2],np.maximum(lower[:2],lower[40:42]),np.minimum(upper[:2],upper[40:42]))*data['limit']
    rows.append({'factor':factor,'alpha':alpha,'rho':rho,'iterations':iteration+1,'primal':primal,'dual':dual,
                 'input_nm':out.tolist(),'input_error_nm':float(max(abs(out-exact[:2]*data['limit'])))})
report={'rho_base':base,'h_eigen_range':np.linalg.eigvalsh(h)[[0,-1]].tolist(),
        'greedy_candidate_error_vs_oracle':float(max(abs(candidate-exact))),
        'greedy_kkt_failures':certificate,
        'unconstrained_max_violation':float(np.max(np.maximum(c@np.linalg.solve(h,-f)-upper,lower-c@np.linalg.solve(h,-f)))),
        'exact_input_nm':(exact[:2]*data['limit']).tolist(),'results':rows}
(root/'diagnostics/lmpc_startup_rho_study_20261007.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
