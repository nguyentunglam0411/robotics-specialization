function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;

% FILL IN YOUR CODE HERE
delta = s_des - s;
e = delta(1);
edot = delta(2);
kp = 60;
kd = 10;

u = params.mass*(0+kp*e+kd*edot+params.gravity);
if u > params.u_max
    u = params.u_max;
elseif u < params.u_min
    u = params.u_min;
end
end

