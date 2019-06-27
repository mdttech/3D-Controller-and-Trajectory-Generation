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


g = params.gravity;
m = params.mass;
kd = [5;5;5];
kp = [100;100;100];
kdm = [1;1;1];
kpm = [160;160;160];
accl_command = des_state.acc+(kd.*(des_state.vel-state.vel))+(kp.*(des_state.pos-state.pos));
phi_des = (1/g)*((accl_command(1)*sin(des_state.yaw))-(accl_command(2)*cos(des_state.yaw)));
theta_des = (1/g)*((accl_command(1)*cos(des_state.yaw))+(accl_command(2)*sin(des_state.yaw)));
des_rot = [phi_des;theta_des;des_state.yaw];
des_omega = [0;0;des_state.yawdot];

% Thrust
F = m*(g+accl_command(3));

% Moment
M = (kdm.*(des_omega-state.omega))+(kpm.*(des_rot-state.rot));

end
