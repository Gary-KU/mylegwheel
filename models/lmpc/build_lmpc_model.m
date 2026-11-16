function build_lmpc_model(rebuild)
% Build a separate model. Original Self_Model_3D.slx is never saved.
if nargin<1, rebuild=false; end
here=fileparts(mfilename('fullpath')); addpath(here);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'cache'), ...
 'CodeGenFolder',fullfile(here,'codegen'),'createDir',true);
source='D:/MATLAB/Robot/WHEEL_LEG/MyProc';
original=fullfile(source,'Self_Model_3D.slx');
destination=fullfile(here,'Self_Model_3D_LMPC.slx');
assert(rebuild || ~isfile(destination),'Destination exists; call with true to rebuild.');
if isfile(destination)
 copyfile(destination,fullfile(here,['Self_Model_3D_LMPC_' datestr(now,'yyyymmdd_HHMMSS') '.slx']));
end
load_system(original);
save_system('Self_Model_3D',destination);
close_system('Self_Model_3D',0);
load_system(destination); mdl='Self_Model_3D_LMPC';
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
set_param(mdl,'InitFcn','init_lmpc;');
init_lmpc;
load_system('mpclib');
% Keep sensing, output routing, support, geometry and VMC unchanged.
for item=[320 328]
 sys=Simulink.ID.getFullName(sprintf('%s:%d',mdl,item));
 if item==320
  multiply=257; gain=229; holdIn=259;
 else
  multiply=266; gain=269; holdIn=268;
 end
 mult=Simulink.ID.getFullName(sprintf('%s:%d',mdl,multiply));
 ph=get_param(mult,'PortHandles');
 outLine=get_param(ph.Outport,'Line'); dst=get_param(outLine,'DstPortHandle');
 pos=get_param(mult,'Position');
 delete_line(outLine);
 for ip=ph.Inport(:)'
  ln=get_param(ip,'Line'); if ln~=-1, delete_line(ln); end
 end
 delete_block(mult);
 gb=Simulink.ID.getFullName(sprintf('%s:%d',mdl,gain));
 gp=get_param(gb,'PortHandles');
 ln=get_param(gp.Inport,'Line'); if ln~=-1, delete_line(ln); end
 delete_block(gb);
 b=[sys '/LMPC'];
 add_block('mpclib/MPC Controller',b,'mpcobj','wheel_leg_lmpc', ...
  'md_inport','off','return_qpstatus','on', ...
  'Position',[pos(1),pos(2)-30,pos(1)+100,pos(2)+60]);
 bp=get_param(b,'PortHandles');
 state=Simulink.ID.getFullName(sprintf('%s:%d',mdl,holdIn));
 sp=get_param(state,'PortHandles');
 add_block('simulink/Signal Attributes/Signal Specification',[sys '/Six states'], ...
  'Dimensions','6','Position',[pos(1)-95,pos(2)-25,pos(1)-45,pos(2)+5]);
 spec=get_param([sys '/Six states'],'PortHandles');
 add_line(sys,sp.Outport,spec.Inport,'autorouting','on');
 add_line(sys,spec.Outport,bp.Inport(1),'autorouting','on');
 add_block('simulink/Sources/Constant',[sys '/LMPC Reference'], ...
  'Value','lmpc_reference','Position',[pos(1)-130,pos(2)+60,pos(1)-70,pos(2)+90]);
 ref=get_param([sys '/LMPC Reference'],'PortHandles');
 add_line(sys,ref.Outport,bp.Inport(2),'autorouting','on');
 for d=dst(:)'
  add_line(sys,bp.Outport(1),d,'autorouting','on');
 end
 add_block('simulink/Sinks/Terminator',[sys '/QP status'], ...
  'Position',[pos(1)+180,pos(2)+40,pos(1)+200,pos(2)+60]);
 qp=get_param([sys '/QP status'],'PortHandles');
 add_line(sys,bp.Outport(2),qp.Inport,'autorouting','on');
 set_param(bp.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom', ...
  'DataLoggingName',sprintf('lmpc_torque_%d',item));
 set_param(bp.Outport(2),'DataLogging','on','DataLoggingNameMode','Custom', ...
  'DataLoggingName',sprintf('lmpc_qp_%d',item));
 set_param(sp.Outport,'DataLogging','on','DataLoggingNameMode','Custom', ...
  'DataLoggingName',sprintf('lmpc_state_%d',item));
 % Unused measured leg length is intentionally terminated in fixed LMPC.
 lp=get_param([sys '/length_' char('L'+(item==328)*6)],'PortHandles');
 add_block('simulink/Sinks/Terminator',[sys '/Fixed nominal length'], ...
  'Position',[80 450 100 470]);
 term=get_param([sys '/Fixed nominal length'],'PortHandles');
 add_line(sys,lp.Outport,term.Inport,'autorouting','on');
end
for sid=[233 236]
 set_param(Simulink.ID.getFullName(sprintf('%s:%d',mdl,sid)), ...
  'Value','lmpc_config.leg_length_m');
end
configure_lmpc_model(mdl);
set_param(mdl,'SignalLogging','on','ReturnWorkspaceOutputs','on');
save_system(mdl);
set_param(mdl,'SimulationCommand','update');
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','on');
save_system(mdl);
fprintf('BUILT %s\n',destination);
% Suppress the window only for this batch run; do not save this override.
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
tic; out=sim(mdl,'StopTime','5'); fprintf('LMPC_SECONDS %.3f\n',toc);
save(fullfile(here,'lmpc_initial_run.mat'),'out');
for k=1:out.logsout.numElements
 sig=out.logsout{k};
 if startsWith(sig.Name,'lmpc_')
  fprintf('%s final=',sig.Name);disp(sig.Values.Data(end,:));
  fprintf('min=');disp(min(sig.Values.Data,[],1));
  fprintf('max=');disp(max(sig.Values.Data,[],1));
 end
end
close_system(mdl,0);
end
