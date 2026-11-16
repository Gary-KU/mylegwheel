function validate_lmpc()
% Reproducible controller tests and same-initial-condition 3-D comparison.
here=fileparts(mfilename('fullpath')); addpath(here); cd(here);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'cache'), ...
 'CodeGenFolder',fullfile(here,'codegen'),'createDir',true);
mpcverbosity('off'); p=lmpc_parameters(); [ctrl,plant,K]=make_lmpc(p);
results=struct();
cases=[0,0,0,0.15,0,0; 0,0,0,0,pi/60,0; 0,0,0.05,0,0,0]';
names={'braking','pitch_3deg','position_5cm'};
for c=1:size(cases,2)
 state=mpcstate(ctrl); x=cases(:,c); N=round(8/p.Ts);
 xx=zeros(N+1,6); uu=zeros(N,2); status=zeros(N,1); elapsed=zeros(N,1);
 xx(1,:)=x';
 for k=1:N
  tick=tic; [u,info]=mpcmove(ctrl,state,x,zeros(1,12));
  elapsed(k)=toc(tick); status(k)=info.Iterations;
  x=plant.A*x+plant.B*u;
  xx(k+1,:)=x'; uu(k,:)=u';
 end
 assert(all(status>0),'QP did not converge.');
 assert(all(max(abs(uu),[],1)<=p.torque_limit_nm+1e-7),'Torque limit failed.');
 assert(all(max(abs(diff([zeros(1,2);uu])),[],1)<=p.torque_step_nm+1e-7),'Rate limit failed.');
 assert(norm(xx(end,:),Inf)<1e-3,'Linear model did not settle.');
 results.(names{c})=struct('state',xx,'torque',uu,'qp',status, ...
  'median_s',median(elapsed(2:end)),'max_s',max(elapsed));
 fprintf('PASS %s final_norm %.6g median_QP_ms %.3f max_QP_ms %.3f\n', ...
  names{c},norm(xx(end,:),Inf),median(elapsed(2:end))*1000,max(elapsed)*1000);
end
% Compare first moves in the unconstrained, negligible-rate-cost limit.
up=p; up.torque_limit_nm=[100 100]; up.torque_step_nm=[100 100];
up.mv_rate_weight=[1e-5 1e-5]; [uc,~,uk]=make_lmpc(up);
trial=[0.001;-0.002;0.003;0.004;-0.001;0.002];
u=mpcmove(uc,mpcstate(uc),trial,zeros(1,12));
results.lqr_equivalence_error=norm(u+uk*trial,Inf);
assert(results.lqr_equivalence_error<1e-6,'Terminal cost / sign check failed.');
save('lmpc_validation.mat','results','p','plant','K');
fprintf('PASS unconstrained LQR match %.6g\n',results.lqr_equivalence_error);
% Plot linear braking case; this is not a hardware execution-time benchmark.
f=figure('Visible','off','Position',[100 100 1100 650]);
t=(0:size(results.braking.state,1)-1)*p.Ts;
tiledlayout(2,2); nexttile; plot(t,results.braking.state(:,3));
xlabel('Time (s)'); ylabel('Position (m)'); grid on;
nexttile; plot(t,results.braking.state(:,4)); xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;
nexttile; plot(t,results.braking.state(:,[1 5])*180/pi); xlabel('Time (s)'); ylabel('Angle (deg)'); legend('Leg','Body'); grid on;
nexttile; stairs(t(1:end-1),results.braking.torque); xlabel('Time (s)'); ylabel('Torque per side (Nm)'); legend('Wheel','Virtual leg'); grid on;
sgtitle('LMPC: linear-model braking from 0.15 m/s');
exportgraphics(f,'lmpc_linear_braking.png'); close(f);
end
