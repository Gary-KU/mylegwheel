function init_lmpc()
% Run after editing lmpc_parameters.m. Model InitFcn runs this automatically.
p=lmpc_parameters();
mpcverbosity('off');
[controller,plant,K]=make_lmpc(p);
assignin('base','lmpc_config',p);
assignin('base','wheel_leg_lmpc',controller);
assignin('base','lmpc_plant',plant);
assignin('base','lmpc_lqr_gain',K);
% Terminal outputs must track chol(P)*reference too.
assignin('base','lmpc_reference',controller.Model.Plant.C*p.reference);
end
