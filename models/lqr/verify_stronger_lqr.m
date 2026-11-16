function verify_stronger_lqr(backup_directory)
% Check the requested gain increase and stability with actual firmware scales.
here=fileparts(mfilename('fullpath'));addpath(here,'-begin');
p=lqr_model_parameters();
old=readmatrix(fullfile(backup_directory,'models','lqr','lqr_coefficients.csv'));
new=readmatrix(fullfile(here,'lqr_coefficients.csv'));
root=fileparts(fileparts(here));
config=fileread(fullfile(root,'App','Tasks','Inc','wheel_leg_config.h'));
wheel=regexp(config,'(?m)^#define\s+BALANCE_DEFAULT_WHEEL_SCALE\s+([0-9.]+)f','tokens','once');
leg=regexp(config,'(?m)^#define\s+BALANCE_DEFAULT_LEG_SCALE\s+([0-9.]+)f','tokens','once');
assert(~isempty(wheel) && ~isempty(leg));
output_scale=[str2double(wheel{1}) str2double(leg{1})];
largest=0;largest_old=0;gain_error=0;
for length=linspace(p.length_min_m,p.length_max_m,101)
 powers=[length^3;length^2;length;1];
 previous=reshape(old*powers,2,6);candidate=reshape(new*powers,2,6);
 expected=diag(p.feedback_gain_scale)*previous;
 gain_error=max(gain_error,max(abs(candidate-expected),[],'all'));
 L=length/2;M=p.body_mass_kg/2;mp=p.leg_mass_per_side_kg;r=p.wheel_radius_m;
 E=[M+mp+p.wheel_mass_per_side_kg+p.wheel_inertia_per_side/r^2,M*length+mp*L,0; ...
    M*length+mp*L,p.leg_inertia_per_side+mp*L^2+M*length^2,0; ...
    0,0,p.body_pitch_inertia_per_side];
 gravity=E\[0;(mp*L+M*length)*p.gravity_m_s2;0];
 actuation=E\[1/r,0;-1,1;0,1];
 A=zeros(6);A(1,2)=1;A(3,4)=1;A(5,6)=1;
 A([4 2 6],1)=gravity;B=zeros(6,2);B([4 2 6],:)=actuation;
 plant=c2d(ss(A,B,eye(6),zeros(6,2)),p.Ts);
 poles=eig(plant.A-plant.B*diag(output_scale)*candidate);
 assert(all(abs(poles)<1),'Scaled firmware feedback is linearly unstable');
 largest=max(largest,max(abs(poles)));
 largest_old=max(largest_old,max(abs(eig(plant.A-plant.B*diag(output_scale)*previous))));
end
assert(gain_error<1e-9,'Gain increase differs from requested row scaling');
report=struct('feedback_gain_scale',p.feedback_gain_scale,'firmware_output_scale',output_scale, ...
 'max_gain_error',gain_error,'max_firmware_pole_radius_old',largest_old, ...
 'max_firmware_pole_radius_new',largest,'leg_lengths_checked',101, ...
 'limitation','Centered nominal mass model; no front battery offset, motor saturation or real hardware motion in this pole check');
fid=fopen(fullfile(here,'stronger_gain_verification.json'),'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('PASS gain scale %.3f/%.3f; max error %.3g; firmware scale %.3f/%.3f; poles old %.9f new %.9f\n', ...
 p.feedback_gain_scale,gain_error,output_scale,largest_old,largest);
end
