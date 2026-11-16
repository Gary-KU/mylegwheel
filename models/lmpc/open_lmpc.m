% Run this script to open the standalone LMPC model with its dependencies.
here=fileparts(mfilename('fullpath'));
addpath(here);
init_lmpc;
open_system(fullfile(here,'Self_Model_3D_LMPC.slx'));
set_param('Self_Model_3D_LMPC','SimMechanicsOpenEditorOnUpdate','on');
