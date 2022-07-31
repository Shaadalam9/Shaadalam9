function [psi, ypd_int] = kcs_guidance_waypoint(t, x, params)
% This function must output desired heading angle psi and the derivative of
% the variable yp_int used in ILOS algorithm.

global wp_flag terminate_flag

wp = params.waypoints;

% Tolerance of each waypoint. When the ship is closer than this value to
% the destination waypoint, you may switch to the next waypoint.
R_tol = params.L;

% Lookahead Distance
L = params.L;
d_LA = 2*L;

% Design Parameter for ILOS
k = 0.05;

xn = x(4);
yn = x(5);
yp_int = x(9);      % For implementing ILOS

% Dealing with waypoint switching
%
% wp_flag is an array of size 1 x number of waypoints. This is a global
% variable that keeps track of which waypoints have been reached and which
% have not been. Each element in this array corresponds to one waypoint.
% The element will have a value 1 if the waypoint has been reached and will
% have a value 0 if it has not been reached yet. 
%
% When your vessel reaches within the R_tol distance of the goal waypoint,
% you may switch your goal waypoint to the next one. At this time you must
% update wp_flag corresponding to the reached waypoint to 1. 
%
% When the last waypoint has been reached, update the terminate_flag to 1.
% Then the simulation will terminate when the last waypoint is reached.
%
%**************************************************************************
% Your code to provide guidance for desired yaw angle (psi) based on ILOS
% scheme starts here.
check = 5;
for i = 1:length(wp_flag)
    if wp_flag(i) == 0
        check = i-1;
        break
    end
end

x_current = wp(check,1);
y_current = wp(check,2);
x_next = wp(check+1,1);
y_next = wp(check+1,2);

if (xn - x_next)^2 + (yn-y_next)^2 <= R_tol^2
   wp_flag(check +1) =1; 
end

pi_p = atan2(y_next-y_current , x_next-x_current);
A = [cos(pi_p) sin(pi_p) 0; -sin(pi_p) cos(pi_p) 1; tan(pi_p) -1 0];
b = [cos(pi_p)*xn + sin(pi_p)*yn ; cos(pi_p)*yn - sin(pi_p)*xn ; tan(pi_p)*x_next - y_next];
x1 = A\b;
ye_p = x1(3);
%Drift angle is taken as 0 here.
Kp = 1/d_LA;
psi = pi_p - atan(Kp * ye_p + k * Kp * yp_int);
ypd_int = d_LA * ye_p/ (sqrt((d_LA^2) +(( ye_p +k*yp_int)^2)));
