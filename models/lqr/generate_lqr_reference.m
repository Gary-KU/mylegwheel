function generate_lqr_reference()
here=fileparts(mfilename('fullpath'));
previous=pwd;cleanup=onCleanup(@()cd(previous)); %#ok<NASGU>
cd(here);addpath(here,'-begin');init_lqr_model;
p=lqr_model_parameters();coeff=evalin('base','lqr_gain_coefficients');
radius=evalin('base','lqr_max_linear_pole_radius');
writematrix(coeff,fullfile(here,'lqr_coefficients.csv'));
fid=fopen(fullfile(here,'lqr_k.m'),'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'function K = lqr_k(L0)\n');
fprintf(fid,'%% Generated for total %.3f kg, body %.3f kg; estimated wheel/leg split.\n',p.total_mass_kg,p.body_mass_kg);
fprintf(fid,'%% Feedback row scales: wheel %.6g, leg %.6g relative to nominal DLQR.\n',p.feedback_gain_scale);
fprintf(fid,'t2=L0.^2;\nt3=L0.^3;\n');
for j=1:12
 fprintf(fid,'mt%d=[(%.17e).*t3+(%.17e).*t2+(%.17e).*L0+(%.17e)];\n',j,coeff(j,:));
end
fprintf(fid,'K=reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12],2,6);\nend\n');
fclose(fid);
metadata=struct('parameters',p,'mass_split_estimated',true,'max_linear_pole_radius',radius, ...
 'state_order',{{'theta','dtheta','x','dx','phi','dphi'}}, ...
 'matrix_storage','MATLAB column-major 2x6: K[row+2*column]', ...
 'firmware_feedback','u=-K*(state-reference)', ...
 'simscape_feedback','u=+K*x because actuator coordinates oppose analytical inputs');
fid=fopen(fullfile(here,'lqr_model_metadata.json'),'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(metadata,PrettyPrint=true));fclose(fid);
save(fullfile(here,'lqr_reference.mat'),'p','coeff','radius');
fprintf('GENERATED total=%.6f body=%.6f per-side body=%.6f wheel=%.6f leg=%.6f support=%.6f pole_radius=%.9f\n', ...
 p.total_mass_kg,p.body_mass_kg,p.body_mass_kg/2,p.wheel_mass_per_side_kg,p.leg_mass_per_side_kg,p.support_feedforward_n,radius);
end
