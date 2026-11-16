from pathlib import Path
import json
import numpy as np
root=Path(__file__).resolve().parents[1]
case=json.loads((root/'diagnostics/lmpc_startup_failure_20261007.json').read_text())
data=np.load(root.parent/'H723_LMPC/models/lmpc_tables.npz');e=np.array(case['inferred_failed_state_error'])
rows=[]
for ti in (4,5,6):
    h,f,c=data['H'][ti],data['F'][ti],data['C'][ti];base=float(data['rho'][ti])
    factors=np.array([1.,16.,256.]);ws=[np.linalg.inv(h+base*scale*c.T@c) for scale in factors]
    for s in (.5,1,1.1,-1):
        for tp in (.00004,.0004,.002,.004):
            g=f@(s*e);bounds=np.r_[np.tile(np.array([.021,tp])/data['limit'],20),np.tile(data['step']/data['limit'],20)]
            v=np.linalg.solve(h,-g);z=np.clip(c@v,-bounds,bounds);y=np.zeros(80)
            mode=2 if max(abs(g))>50*base else 0;changes=[]
            for iteration in range(160):
                rho=base*factors[mode];v=ws[mode]@(-g+rho*c.T@(z-y));cv=c@v
                nxt=np.clip(cv+y,-bounds,bounds)
                primal=float(max(abs(cv-nxt)));dual=float(max(abs(rho*c.T@(nxt-z))))
                y+=cv-nxt;z=nxt
                if primal<.001 and dual<.04:break
                if (iteration+1)%16==0:
                    new=mode
                    if primal/.001>8*(dual/.04) and mode<2:new=mode+1
                    elif dual/.04>8*(primal/.001) and mode>0:new=mode-1
                    if new!=mode:
                        y*=factors[mode]/factors[new];changes.append((iteration+1,mode,new));mode=new
            rows.append({'table':ti,'scale':s,'tp':tp,'iters':iteration+1,'primal':primal,'dual':dual,'changes':changes})
print(json.dumps({'max_iterations':max(r['iters'] for r in rows),'failed':[r for r in rows if r['primal']>=.001 or r['dual']>=.04],
                  'captured':[r for r in rows if r['table']==5 and r['scale']==1 and r['tp']==.00004]},indent=2))
(root/'diagnostics/lmpc_adaptive_rho_study_20261007.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
