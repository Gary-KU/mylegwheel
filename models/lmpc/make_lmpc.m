function [controller, plant, K] = make_lmpc(p)
% Linearization of leg_k_matrix's equations at upright and fixed length.
% Each controller represents one side (half body, one wheel).
% The source Simscape feedback uses +K*x; actuator coordinates are opposite
% the analytical T,Tp convention, hence the explicit minus sign on B.
L=p.leg_length_m/2; L0=p.leg_length_m; r=p.wheel_radius_m;
M=p.body_mass_kg/2; mp=p.leg_mass_kg/2; mw=p.wheel_mass_kg/2;
Iw=p.wheel_inertia/2; Ip=p.leg_inertia/2; Im=p.body_inertia/2;
E=[M+mp+mw+Iw/r^2, M*L0+mp*L, 0; ...
   M*L0+mp*L, Ip+mp*L^2+M*L0^2, 0; 0,0,Im];
gravity=E\[0;(mp*L+M*L0)*9.8;0];
actuation=-(E\[1/r,0;-1,1;0,1]);
A=zeros(6); A(1,2)=1; A(3,4)=1; A(5,6)=1;
A([4 2 6],1)=gravity;
B=zeros(6,2); B([4 2 6],:)=actuation;
plant=c2d(ss(A,B,eye(6),zeros(6,2)),p.Ts);
plant.StateName={'theta','dtheta','position','velocity','pitch','dpitch'};
plant.InputName={'wheel torque','virtual leg torque'};
Q=diag(p.Q); R=diag(p.R);
% Output-cost LQR Riccati terminal penalty follows MathWorks' MPC example.
[K,P]=lqry(plant,Q,R);
augmented=ss(plant.A,plant.B,[eye(6);chol(P)],zeros(12,2),p.Ts);
augmented=setmpcsignals(augmented,'MO',1:6,'UO',7:12);
controller=mpc(augmented,p.Ts,p.prediction_horizon,p.control_horizon);
controller.Weights.OV=[sqrt(p.Q),zeros(1,6)];
controller.Weights.MV=sqrt(p.R);
controller.Weights.MVRate=p.mv_rate_weight;
setterminal(controller,struct('Weight',[zeros(1,6),ones(1,6)]), ...
    struct('Weight',sqrt(p.R)));
setoutdist(controller,'model',ss(zeros(12,1)));
setEstimator(controller,[],eye(6)); % All six states are provided by Simscape.
for j=1:2
 controller.MV(j).Min=-p.torque_limit_nm(j);
 controller.MV(j).Max= p.torque_limit_nm(j);
 controller.MV(j).RateMin=-p.torque_step_nm(j);
 controller.MV(j).RateMax= p.torque_step_nm(j);
end
end
