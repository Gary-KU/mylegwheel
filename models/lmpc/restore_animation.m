function restore_animation()
% Restore visualization without running a simulation or changing control.
here=fileparts(mfilename('fullpath'));
folders={here,'D:/MATLAB/Robot/WHEEL_LEG/MyProc/LMPC'};
for k=1:numel(folders)
 folder=folders{k}; file=fullfile(folder,'Self_Model_3D_LMPC.slx');
 copyfile(file,fullfile(folder,['Self_Model_3D_LMPC_before_animation_' datestr(now,'yyyymmdd_HHMMSS') '.slx']));
 load_system(file);
 set_param('Self_Model_3D_LMPC','SimMechanicsOpenEditorOnUpdate','on');
 save_system('Self_Model_3D_LMPC');
 close_system('Self_Model_3D_LMPC',0);
 load_system(file);
 assert(strcmp(get_param('Self_Model_3D_LMPC','SimMechanicsOpenEditorOnUpdate'),'on'));
 close_system('Self_Model_3D_LMPC',0);
 % Preserve other edits to the user's opening script.
 script=fullfile(folder,'open_lmpc.m'); content=fileread(script);
 if ~contains(content,'SimMechanicsOpenEditorOnUpdate')
  fid=fopen(script,'a'); assert(fid~=-1);
  fprintf(fid,'\nset_param(''Self_Model_3D_LMPC'',''SimMechanicsOpenEditorOnUpdate'',''on'');\n');
  fclose(fid);
 end
 fprintf('VERIFIED animation enabled: %s\n',file);
end
end
