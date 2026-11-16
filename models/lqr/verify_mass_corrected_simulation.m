function verify_mass_corrected_simulation()
here=fileparts(mfilename('fullpath'));stage=fullfile(here,'simulation');
source='D:/MATLAB/Robot/WHEEL_LEG/MyProc';mdl='Self_Model_3D';
if ~isfolder(stage),mkdir(stage);end
copyfile(fullfile(source,[mdl '.slx']),fullfile(stage,[mdl '.slx']));
for name={'lqr_model_parameters.m','init_lqr_model.m'}
 copyfile(fullfile(here,name{1}),fullfile(stage,name{1}));
end
cd(stage);addpath(stage,'-begin');
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'sim_cache'), ...
 'CodeGenFolder',fullfile(here,'sim_cache','codegen'),'createDir',true);
load_system(fullfile(stage,[mdl '.slx']));init_lqr_model;p=lqr_model_parameters();
assert(abs(p.body_mass_kg-1.10)<1e-12 && abs(p.total_mass_kg-1.45)<1e-12);
for sid=[47 41 160 148 85 79 59 53]
 set_param(block(mdl,sid),'rho','lqr_model_config.plate_density_kg_m3');
end
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
set_param(mdl,'SimulationCommand','update');save_system(mdl);close_system(mdl,0);
results=struct();cases={'standing',0,5;'pitch_pos_10',10,8;'pitch_neg_10',-10,8};
for k=1:size(cases,1)
 load_system(fullfile(stage,[mdl '.slx']));
 set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off','SignalLogging','on');
 set_param(block(mdl,9),'PositionTargetValue',num2str(cases{k,2}));
 for sid=[231 232 239 240 277]
  ph=get_param(block(mdl,sid),'PortHandles');
  set_param(ph.Outport,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',sprintf('motor_%d',sid));
 end
 right=[get_param(block(mdl,335),'Parent') '/Wheel motor torque limit'];
 ph=get_param(right,'PortHandles');
 set_param(ph.Outport,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName','motor_right_wheel');
 tic;out=sim(mdl,'StopTime',num2str(cases{k,3}),'CaptureErrors','on','ReturnWorkspaceOutputs','on');elapsed=toc;
 save(fullfile(here,[cases{k,1} '_total1450g.mat']),'out','p','-v7.3');
 assert(isempty(out.ErrorMessage),out.ErrorMessage);
 d=struct();names={'pitch','roll','thetaL','thetaR','lengthL','lengthR','velocity','phi1L','phi4L', ...
  'motor_231','motor_232','motor_239','motor_240','motor_277','motor_right_wheel'};
 for j=1:numel(names)
  s=out.logsout.get(names{j}).Values;d.(names{j})=struct('t',s.Time,'x',double(s.Data(:)));
 end
 d.wheel_peak=max([abs(d.motor_277.x);abs(d.motor_right_wheel.x)]);
 d.joint_peak=max([abs(d.motor_231.x);abs(d.motor_232.x);abs(d.motor_239.x);abs(d.motor_240.x)]);
 assert(d.wheel_peak<=p.wheel_torque_limit_nm+1e-12 && d.joint_peak<=p.joint_torque_limit_nm+1e-12);
 tail=d.pitch.t>=cases{k,3}-1;
 d.pitch_final_deg=d.pitch.x(end)*180/pi;
 d.speed_final=d.velocity.x(end);
 d.lengths_final=[d.lengthL.x(end) d.lengthR.x(end)];
 d.recovered=max(abs(d.pitch.x(tail)))*180/pi<1 && ...
  max(abs(d.velocity.x(d.velocity.t>=cases{k,3}-1)))<.005 && ...
  max(abs(d.lengths_final-p.leg_length_m))<.005;
 results.(cases{k,1})=d;
 fprintf('CASE %s seconds=%.2f recovered=%d pitch=%.6f speed=%.6f lengths=%.6f %.6f wheel=%.6f joint=%.6f\n', ...
  cases{k,1},elapsed,d.recovered,d.pitch_final_deg,d.speed_final,d.lengths_final,d.wheel_peak,d.joint_peak);
 save(fullfile(here,'mass_corrected_simulation.mat'),'results','p');
 if k==1,assert(d.recovered,'Corrected-mass standing failed');end
 close_system(mdl,0);
end
load_system(fullfile(stage,[mdl '.slx']));set_param(mdl,'SimMechanicsOpenEditorOnUpdate','on');save_system(mdl);close_system(mdl,0);
fprintf('PASS corrected 1.45 kg total mass; standing and actuator limits. +/-10 degree assessment recorded.\n');
end
function path=block(mdl,sid)
path=Simulink.ID.getFullName(sprintf('%s:%d',mdl,sid));
end
