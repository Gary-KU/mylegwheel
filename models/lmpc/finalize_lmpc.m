function finalize_lmpc()
here=fileparts(mfilename('fullpath')); addpath(here); cd(here);
Simulink.fileGenControl('set','CacheFolder',fullfile(here,'cache'), ...
 'CodeGenFolder',fullfile(here,'codegen'),'createDir',true);
load_system(fullfile(here,'Self_Model_3D_LMPC.slx'));
configure_lmpc_model('Self_Model_3D_LMPC');
set_param('Self_Model_3D_LMPC','SimulationCommand','update');
save_system('Self_Model_3D_LMPC'); close_system('Self_Model_3D_LMPC',0);
% Check a fresh load from another folder, without the helper folder on path.
rmpath(here); cd(fileparts(here));
load_system(fullfile(here,'Self_Model_3D_LMPC.slx'));
set_param('Self_Model_3D_LMPC','SimulationCommand','update');
close_system('Self_Model_3D_LMPC',0);
fprintf('PASS standalone model reload and configured sample time.\n');
end
