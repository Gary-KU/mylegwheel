function configure_lmpc_model(mdl)
% Make the saved model find adjacent helpers and use one sampling parameter.
set_param(mdl,'PostLoadFcn',sprintf('addpath(fileparts(get_param(''%s'',''FileName'')));',mdl));
for sid=[259 260 268 270]
 set_param(Simulink.ID.getFullName(sprintf('%s:%d',mdl,sid)), ...
  'SampleTime','lmpc_config.Ts');
end
end
