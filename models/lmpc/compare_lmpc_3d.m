function compare_lmpc_3d()
here=fileparts(mfilename('fullpath')); addpath(here); cd(here);
source='D:/MATLAB/Robot/WHEEL_LEG/MyProc'; addpath(source);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'cache'), ...
 'CodeGenFolder',fullfile(here,'codegen'),'createDir',true);
p=lmpc_parameters(); results=struct();
% The LMPC nominal run was saved by the build script.
if isfile('lmpc_initial_run.mat')
 nominal=load('lmpc_initial_run.mat');
else
 load_system(fullfile(here,'Self_Model_3D_LMPC.slx'));
 nominal.out=sim('Self_Model_3D_LMPC','StopTime','5');
 close_system('Self_Model_3D_LMPC',0);
end
results.lmpc_nominal=extract(nominal.out,'lmpc_state_320');
load_system(fullfile(source,'Self_Model_3D.slx')); mdl='Self_Model_3D';
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
b=Simulink.ID.getFullName([mdl ':259']); ph=get_param(b,'PortHandles');
set_param(ph.Outport,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','compare_state');
out=sim(mdl,'StopTime','5','SignalLogging','on','ReturnWorkspaceOutputs','on');
results.lqr_nominal=extract(out,'compare_state');
close_system(mdl,0); % Never save the source.
fprintf('LQR nominal final = ');disp(results.lqr_nominal.x(end,:));
for c=1:2
 load_system(fullfile(here,'Self_Model_3D_LMPC.slx')); mdl='Self_Model_3D_LMPC';
 if c==1
  b=Simulink.ID.getFullName([mdl ':8']);
  set_param(b,'PxVelocityTargetSpecify','on','PxVelocityTargetPriority','High', ...
   'PxVelocityTargetValue','0.15','PxVelocityTargetValueUnits','m/s');
  name='lmpc_braking';
 else
  b=Simulink.ID.getFullName([mdl ':9']);
  set_param(b,'PositionTargetValue','3','PositionTargetValueUnits','deg');
  name='lmpc_pitch_3deg';
 end
 out=sim(mdl,'StopTime','5');
 results.(name)=extract(out,'lmpc_state_320');
 x=results.(name).x;
 fprintf('%s initial = ',name);disp(x(1,:));
 fprintf('%s final = ',name);disp(x(end,:));
 for side=[320 328]
  u=extract(out,sprintf('lmpc_torque_%d',side));
  q=extract(out,sprintf('lmpc_qp_%d',side));
  assert(all(q.x(:)>0),'3-D QP failure');
  assert(all(max(abs(u.x),[],1)<=p.torque_limit_nm+1e-7),'3-D torque limit');
  assert(all(max(abs(diff([zeros(1,2);u.x])),[],1)<=p.torque_step_nm+1e-6),'3-D rate limit');
 end
 results.(name).pass=abs(x(end,3))<0.005 && abs(x(end,4))<0.002 && abs(x(end,5))<pi/360;
 save([name '.mat'],'out');
 close_system(mdl,0);
 fprintf('%s settle_pass = %d\n',name,results.(name).pass);
end
save('lmpc_3d_comparison.mat','results');
f=figure('Visible','off','Position',[100 100 1100 700]);
tiledlayout(2,2);
fields={'lqr_nominal','lmpc_nominal','lmpc_braking','lmpc_pitch_3deg'};
labels={'Source LQR nominal','LMPC nominal','LMPC initial v=0.15 m/s','LMPC initial pitch=3 deg'};
indices=[3 4 5 1]; units={'Position (m)','Velocity (m/s)','Body angle (deg)','Leg angle (deg)'};
for j=1:4
 nexttile; hold on;
 for k=1:4
  d=results.(fields{k}); scale=1; if j>=3,scale=180/pi;end
  plot(d.t,d.x(:,indices(j))*scale);
 end
 xlabel('Time (s)'); ylabel(units{j}); grid on;
 if j==1,legend(labels,'Location','best');end
end
sgtitle('Self Model 3D: original LQR baseline and LMPC checks');
exportgraphics(f,'lmpc_3d_comparison.png'); close(f);
assert(results.lmpc_braking.pass && results.lmpc_pitch_3deg.pass,'3-D settling check failed');
fprintf('PASS 3-D braking, pitch disturbance, torque/rate limits, QP status.\n');
end
function d=extract(out,name)
s=out.logsout.get(name).Values; d=struct('t',s.Time,'x',squeeze(s.Data));
end
