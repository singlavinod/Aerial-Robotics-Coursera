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

% robot data
m = params.mass;
Ixx = params.Ixx;
g = params.gravity;

% gains
Kvz = 10;       Kpz = 50;
Kvphi = 20;     Kpphi = 80;
Kvy = 10;       Kpy = 50;

% current state
y = state.pos(1); z = state.pos(2);
y_dot = state.vel(1); z_dot = state.vel(2);
phi = state.rot; phi_dot = state.omega;

% desired state
y_des = des_state.pos(1); z_des = des_state.pos(2);
y_dot_des = des_state.vel(1); z_dot_des = des_state.vel(2);
y_ddot_des = des_state.acc(1); z_ddot_des = des_state.acc(2);

% control equations

phi_c = -(y_ddot_des + Kvy*(y_dot_des - y_dot) + Kpy*(y_des - y))/g;
phi_c_dot = -(Kvy*(y_ddot_des + g*phi) + Kpy*(y_dot_des - y_dot))/g;
u1 = m*(g + z_ddot_des + Kvz*(z_dot_des - z_dot) + Kpz*(z_des - z));
u2 = Ixx*( Kvphi*(phi_c_dot - phi_dot) + Kpphi*(phi_c - phi));

end

