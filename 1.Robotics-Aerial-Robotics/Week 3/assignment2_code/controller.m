function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
%Parameter
m = params.mass;
g = params.gravity;
J = params.Ixx;
L = params.arm_length;
minU1 = params.minF;
maxU1 = params.maxF;

%Current state
y = state.pos(1);
z = state.pos(2);
phi = state.rot;
ydot = state.vel(1);
zdot = state.vel(2);
phidot = state.omega;
%Desired states
ydes = des_state.pos(1);
zdes = des_state.pos(2);

ydotdes = des_state.vel(1);
zdotdes = des_state.vel(2);
phidotdes = 0;

ydotdotdes = des_state.acc(1);
zdotdotdes = des_state.acc(2);
phidotdotdes = 0;

%PD Controller
Kpy = 20;
Kvy = 5;
Kpz = 80;
Kvz = 12;
Kpphi = 1000;
Kvphi = 30;

phides = (-1/g)*(ydotdotdes+Kvy*(ydotdes-ydot)+Kpy*(ydes-y));

%Log to test
%fprintf('%f\n', ydes - y);

%Output
u1 = m*(g+zdotdotdes+Kvz*(zdotdes-zdot)+Kpz*(zdes-z));
u2 = J*(phidotdotdes+Kvphi*(phidotdes-phidot)+Kpphi*(phides-phi));

if u1 < minU1
    u1 = minU1;
elseif u1 > maxU1
    u1 = maxU1;
end
end

