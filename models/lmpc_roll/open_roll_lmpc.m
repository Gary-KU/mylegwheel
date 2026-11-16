% 日常入口：运行本文件，再点击模型 Run。
here=fileparts(mfilename('fullpath'));
cd(here);
addpath(here,'-begin');
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'.cache'), ...
 'CodeGenFolder',fullfile(here,'.cache','codegen'),'createDir',true);
init_roll_lmpc;
open_system(fullfile(here,'Self_Model_3D_LMPC_Roll.slx'));
set_param('Self_Model_3D_LMPC_Roll','SimMechanicsOpenEditorOnUpdate','on');
set_param('Self_Model_3D_LMPC_Roll','SimulationCommand','update');
fprintf('当前模型: %s\n',get_param('Self_Model_3D_LMPC_Roll','FileName'));
fprintf('机身长 %.0f mm / 宽 %.0f mm / 高 %.0f mm / 质量 %.2f kg\n', ...
 lmpc_config.body_length_m*1000,lmpc_config.body_width_m*1000, ...
 lmpc_config.body_height_m*1000,lmpc_config.body_mass_kg);
