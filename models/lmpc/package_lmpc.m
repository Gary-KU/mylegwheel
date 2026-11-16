function package_lmpc()
% Export the verified independent copy beside the user's original project.
finalize_lmpc;
here=fileparts(mfilename('fullpath'));
target='D:/MATLAB/Robot/WHEEL_LEG/MyProc/LMPC';
assert(~isfolder(target),'Destination already exists; refusing to overwrite it.');
mkdir(target);
files=[dir(fullfile(here,'*.m'));dir(fullfile(here,'*.txt'));dir(fullfile(here,'*.png'))];
for k=1:numel(files)
 copyfile(fullfile(here,files(k).name),fullfile(target,files(k).name));
end
copyfile(fullfile(here,'Self_Model_3D_LMPC.slx'),target);
copyfile(fullfile(here,'lmpc_validation.mat'),target);
copyfile(fullfile(here,'lmpc_3d_comparison.mat'),target);
addpath(target,'-begin');
load_system(fullfile(target,'Self_Model_3D_LMPC.slx'));
set_param('Self_Model_3D_LMPC','SimulationCommand','update');
close_system('Self_Model_3D_LMPC',0);
fprintf('PACKAGED AND COMPILED %s\n',target);
end
