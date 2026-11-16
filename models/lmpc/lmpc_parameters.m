function p = lmpc_parameters()
% LMPC 仿真调参入口。这里的力矩限制不是经过实测的驱动板额定值。
p.Ts = 0.005;
p.prediction_horizon = 20;
p.control_horizon = 20;
p.leg_length_m = 0.10; % Same as the source 3-D support controller.
p.wheel_radius_m = 0.02; % Source Simscape cylinder radius, NOT firmware 0.026.
% Whole robot values from leg_k_matrix, with wheel geometry corrected.
p.body_mass_kg = 0.325;
p.leg_mass_kg = 0.0248;
p.wheel_mass_kg = 2*pi*0.02^2*0.02*5000;
p.wheel_inertia = p.wheel_mass_kg*p.wheel_radius_m^2/2;
p.leg_inertia = (9.66667e-7+7.9975e-6)*2;
p.body_inertia = 0.000338542;
p.Q = [400 15 20 10 200 10]; % 腿角、腿角速度、位置、速度、俯仰、俯仰角速度
p.R = [1 0.5]; % 轮力矩、虚拟腿力矩的代价；越大越不愿意用力
p.mv_rate_weight = [0.02 0.02]; % 力矩变化惩罚；过大可能响应慢
p.torque_limit_nm = [0.15 0.12]; % Per side: [wheel, virtual leg].
p.torque_step_nm = [0.03 0.03]; % Per control sample, not Nm/s.
p.reference = zeros(6,1); % theta,dtheta,position,velocity,pitch,dpitch
end
