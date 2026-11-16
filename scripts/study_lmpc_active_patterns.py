from pathlib import Path
import json
import numpy as np
root=Path(__file__).resolve().parents[1]
case=json.loads((root/'diagnostics/lmpc_startup_failure_20261007.json').read_text())
data=np.load(root.parent/'H723_LMPC/models/lmpc_tables.npz');e=np.array(case['inferred_failed_state_error'])*.5
for ti in (4,5,6):
    h,f,c=data['H'][ti],data['F'][ti]@e,data['C'][ti];rho=16*float(data['rho'][ti])
    upper=np.r_[np.tile(np.array([.021,.00004])/data['limit'],20),np.tile(data['step']/data['limit'],20)]
    inv=np.linalg.inv(h);w=np.linalg.inv(h+rho*c.T@c);v=inv@(-f);z=np.clip(c@v,-upper,upper);y=np.zeros(80)
    rows=[]
    for iteration in range(160):
        v=w@(-f+rho*c.T@(z-y));cv=c@v;nxt=np.clip(cv+y,-upper,upper);y+=cv-nxt;z=nxt
        if (iteration+1)%16:continue
        kinds=np.where((z==upper)&(y>0),1,np.where((z==-upper)&(y<0),-1,0))
        ids=np.flatnonzero(kinds);a=kinds[ids,None]*c[ids];b=upper[ids]
        lam=np.linalg.lstsq(a@inv@a.T,a@inv@(-f)-b,rcond=1e-12)[0]
        polished=inv@(-f-a.T@lam)
        primal=max(abs(c@polished)-upper)
        rows.append({'iteration':iteration+1,'active':len(ids),'primal':float(primal),
                     'worst_lambda':float(min(lam)),'worst_constraint':int(ids[np.argmin(lam)])})
    print(ti,json.dumps(rows))
