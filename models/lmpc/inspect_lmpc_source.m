% Read-only inspection: never saves the source model.
here = fileparts(mfilename('fullpath'));
cd(here);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'cache'), ...
    'CodeGenFolder',fullfile(here,'codegen'),'createDir',true);
addpath('D:/MATLAB/Robot/WHEEL_LEG/MyProc');
load_system('D:/MATLAB/Robot/WHEEL_LEG/MyProc/Self_Model_3D.slx');
mdl='Self_Model_3D';
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
fprintf('MATLAB %s\n',version);
for sid=[21 119 144 8 9 117 118 202 203 247 249 257 266]
 b=Simulink.ID.getFullName(sprintf('%s:%d',mdl,sid));
 fprintf('SID %d PATH %s\n',sid,b);
 if any(sid==[21 119 144])
  p=get_param(b,'DialogParameters');
  names=fieldnames(p);
  for j=1:numel(names)
   if ~isempty(regexp(names{j},'Density|Dimensions|Radius|Length|InertiaType|Mass','once'))
    fprintf('%s = %s\n',names{j},get_param(b,names{j}));
   end
  end
 end
end
load_system('mpclib');
b='mpclib/MPC Controller';
disp(get_param(b,'DialogParameters'));
for sid=[247 249]
 b=Simulink.ID.getFullName(sprintf('%s:%d',mdl,sid));
 p=get_param(b,'PortHandles');
 set_param(p.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom', ...
   'DataLoggingName',sprintf('state_%d',sid));
end
tic;
out=sim(mdl,'StopTime','2','SignalLogging','on','ReturnWorkspaceOutputs','on');
fprintf('BASELINE_SECONDS %.3f\n',toc);
save(fullfile(here,'baseline_inspection.mat'),'out');
for k=1:out.logsout.numElements
 sig=out.logsout{k};
 if startsWith(sig.Name,'state_')
  fprintf('%s final=',sig.Name);disp(sig.Values.Data(end,:));
  fprintf('maxabs=');disp(max(abs(sig.Values.Data),[],1));
 end
end
close_system(mdl,0);
