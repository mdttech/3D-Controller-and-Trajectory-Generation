function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.



%% Fill in your code here

persistent waypoints0 traj_time d0 coef_x coef_y coef_z
 if nargin > 2
   d = waypoints(:,2:end) - waypoints(:,1:end-1);
   d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
   traj_time = [0, cumsum(d0)];
   waypoints0 = waypoints;
   
    coef_x = getCoef(waypoints0(1,1:end)');
    coef_y = getCoef(waypoints0(2,1:end)');
    coef_z = getCoef(waypoints0(3,1:end)');

desired_state.pos = zeros(3,1);
desired_state.vel = zeros(3,1);
desired_state.acc = zeros(3,1);
desired_state.yaw = 0;
 else
 % provide the trajectory point based on the coefficients
    if(t > traj_time(end))
        t = traj_time(end) - 0.0001;
    end
    
    t_index = find(traj_time >= t,1)-1; %between 1:n
    
    if (t_index == 0)
        t_index = 1;
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = 0*waypoints0(:,1);
        desired_state.acc = 0*waypoints0(:,1);
    else
        %create scaled time, value between 0 to 1
        scale = (t-traj_time(t_index))/d0(t_index);
        
        index = (t_index-1)*8+1:t_index*8;
        
        %calculate position:
        t0 = polyT(8,0,scale)';
        desired_state.pos = [coef_x(index)'*t0; coef_y(index)'*t0; coef_z(index)'*t0];
        
        %calculate velocity:
        t1 = polyT(8,1,scale)';
        desired_state.vel = [coef_x(index)'*t1; coef_y(index)'*t1; coef_z(index)'*t1].*(1/d0(t_index));
        
        %calculate acceleration:
        t2 = polyT(8,2,scale)';
        desired_state.acc = [coef_x(index)'*t2; coef_y(index)'*t2; coef_z(index)'*t2].*(1/d0(t_index)^2);
    end
    
    % leave desired yaw and yawdot at zero
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end
 
 
%__________________________________________________________________________ 
 
% returns the 1xN matrix of coefficients-coefficients vector
 
 
function [T] = polyT(n,k,t)           
T = zeros(n,1);
D = zeros(n,1);
%Init:
for i=1:n
D(i) = i-1;
T(i) = 1;
end
%Derivative:
for j=1:k
    for i=1:n
        T(i) = T(i) * D(i);

        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end
%put t value
for i=1:n
T(i) = T(i) * t^D(i);
end
T = T';
end
%-------------------------------------------------------------------------

% returns the vector of unnknowns 

function [coff, A, b] = getCoef(waypoints)
n = size(waypoints,1)-1; % number of segments P1..n


% Fill A and b matices with values using loops

% Filling of b matrix

b = zeros(1,8*n);
% filling the b vector with Wi and Wi+1 waypoints
for i=1:n
    b(1,i) = waypoints(i);
    b(1,i+n) = waypoints(i+1);
end

% Filling of A matrix we will use polyT(n,k,t) function to fill A matrix

A = zeros(8*n, 8*n);

% Constrain 1) Pi(0) = Wi for all i=1..n

for i=1:n
    A(i,((i-1)*8)+1:i*8) = polyT(8,0,0);
end

% Constrain 2) Pi(1) = Wi+1 for all i=1..n

for i=1:n
    A(i+n,((i-1)*8)+1:i*8) = polyT(8,0,1);
end

% Constrain 3) P1(k)(0)= 0 for all 1<=k<=3

for k=1:3
    A(2*n+k,1:8) = polyT(8,k,0);
end

% Constrain 4) Pn(k)(1) = 0 for all 1<=k<=3

for k=1:3
    A(2*n+3+k,(end-7):end) = polyT(8,k,1);
end

% Constrain 5) Pi-1(k)(1) = Pi(k)(0) for all i=2..n and for all k=1..6

for i=2:n
    for k=1:6
        A(2*n+6+(i-2)*6+k, (i-2)*8+1:((i-2)*8+n*n)) = [polyT(8,k,1) -polyT(8,k,0)];
    end
end

% return the coefficient vector

coff = A\b';

end