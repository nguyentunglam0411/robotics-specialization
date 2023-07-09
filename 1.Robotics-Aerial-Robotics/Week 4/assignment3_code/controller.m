function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

% =================== Your code ends here ===================
%Thrust controller
Kp = [500;500;500];
Kd = [20;20;20];
rdesdotdot = des_state.acc+Kp.*(des_state.pos-state.pos)+Kd.*(des_state.vel-state.vel);
F = params.mass*(params.gravity+rdesdotdot(3));
if F > params.maxF
    F = params.maxF;
elseif F < params.minF
    F = params.minF;
end

%Moment controller
KpMoment = [100;100;100];
KdMoment = [2;2;2];

psi_des     = des_state.yaw;
phi_des     = (1/params.gravity)*(rdesdotdot(1)*sin(psi_des)-rdesdotdot(2)*cos(psi_des));
theta_des   = (1/params.gravity)*(rdesdotdot(1)*cos(psi_des)+rdesdotdot(2)*sin(psi_des));

rotation_des = [phi_des;theta_des;psi_des];
omega_des = [0;0;des_state.yawdot];

M = KpMoment.*(rotation_des-state.rot)+KdMoment.*(omega_des-state.omega);

end
