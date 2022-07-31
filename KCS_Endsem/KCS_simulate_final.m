clear all
close all
clc

set(0,'DefaultAxesFontName','Times New Roman')
set(0,'DefaultAxesFontSize',14)
set(0,'DefaultLineLineWidth',1.5)

scale = 75.5;               % Scale ratio

params = struct;            % Parameter data structure

params.rho = 1000;          % Density of fresh water
params.g = 9.80665;         % Acceleration due to gravity

params.L = 230/scale;       % Length
params.B = 32.2/scale;      % Breadth
params.T = 10.8/scale;      % Draft
params.Cb = 0.651;          % Block coefficient

params.Fn = 0.26;           % Froude number

% Design speed
params.U = params.Fn * sqrt(params.g * params.L);

% Displacement
params.Dsp = params.Cb * params.L * params.B * params.T;

% Mass
params.m = params.rho * params.Dsp;

% Center of gravity
xG = -1.48 * params.L / 100;
yG = 0;
zG = -3.552/75.5;
params.rG = [xG yG zG];

% Yaw mass moment of inertia
params.Iz = params.m * ((0.25 * params.L)^2 + xG^2);

% Mass matrix
params.M_RB = [params.m 0 0; 
    0 params.m -params.m*params.rG(1); 
    0 -params.m*params.rG(1) params.Iz;];

% Added mass matrix
load KCS_hydra.mat w AM
M_A = squeeze(AM(2,[1 2 6],[1 2 6]));
M_A(1,1) = M_A(1,1)/scale^3;
M_A(1,2) = M_A(1,2)/scale^3;
M_A(1,3) = M_A(1,3)/scale^4;
M_A(2,1) = M_A(2,1)/scale^3;
M_A(2,2) = M_A(2,2)/scale^3;
M_A(2,3) = M_A(2,3)/scale^4;
M_A(3,1) = M_A(3,1)/scale^4;
M_A(3,2) = M_A(3,2)/scale^4;
M_A(3,3) = M_A(3,3)/scale^5;
params.M_A = M_A;

% Linear Damping matrix
D_L = zeros(3);
D_L(2,2) = 0.5 * params.rho * params.L * params.T * params.U * (-0.2252);
D_L(3,3) = 0.5 * params.rho * params.L^3 * params.T * params.U * (-0.0465);
D_L(2,3) = 0.5 * params.rho * params.L^2 * params.T * params.U * 0.0398;
D_L(3,2) = 0.5 * params.rho * params.L^2 * params.T * params.U * (-0.1111);
params.D_L = D_L; 

% Propeller Parametes
params.pot = kp505_pot();

% Rudder Hydrodynamic Derivatives for state estimation model
[Yd, Nd] = kcs_rudder_derivatives(params.pot);
params.Yd = Yd;
params.Nd = Nd;

% Maximum rudder and rudder rate
params.delta_max = 35*pi/180;
params.deltad_max = 35*pi/180;

% Rudder time constant
params.rudderTC = 0.25;

% Propeller time constant
params.propTC = 0.1;

% Measurement noise covariance
params.xn_noise_sig = 1;
params.yn_noise_sig = 1;
params.psi_noise_sig = 5*pi/180;

% Waypoints
wp = [
    0 0;
    100 0;
    100 100;
    0 100;
    0 0;
    ];

params.waypoints = wp;

global wp_flag terminate_flag
wp_flag = zeros(1,numel(wp)/2);
wp_flag(1) = 1;

terminate_flag = 0;


% You may update Tmax if your ship is taking more time to reach the last 
% waypoint. Although it should not be needed if your guidance and 
% controller do their job correctly.
Tmax = 300;     
dt = 0.01;
t = 0:dt:Tmax;

% Store time vector to params
params.Tmax = Tmax;
params.dt = dt;
params.t = t;

% Calculate controller gains
[Kp, Kd] = kcs_controller_gains(params);
params.Kp = Kp;
params.Kd = Kd;

% Initial condition and uncertainty
x0 = [params.U 0 0 10 10 -pi/2 0 1 0]';

% Simulate and estimate state in a time loop
options = odeset('Events',@terminate_kcs);
sol = ode45(@(t,x) kcs_ode(t,x,x,params),[0 Tmax], x0, options);

% Time vector
Tmax = min([Tmax sol.xe]);
t = 0:0.01:Tmax;

ss = deval(sol,t);

% Outputs
u = ss(1,:);
v = ss(2,:);
r = ss(3,:);
xn = ss(4,:);
yn = ss(5,:);
psi = ss(6,:);
delta = ss(7,:);
n_prop = ss(8,:);
yint = ss(9,:);

% Desired outputs for comparison
xn_des = wp(:,1);
yn_des = wp(:,2);

% Comparison of actual and desired position
figure()
plot(xn_des,yn_des,'ko',xn,yn,'r',xn(1),yn(1),'bo')
set(gca, 'YDir', 'reverse')
set(gca,'YMinorTick','on')
set(gca,'XMinorTick','on')
axis equal
title('Trajectory Tracking')
xlabel('x in m')
ylabel('y in m')
legend('Desired','Actual','Location','Best')
legend boxoff
box off

% Comparison of actual and desired heading
figure()
plot(t,psi*180/pi,'r')
set(gca,'YMinorTick','on')
set(gca,'XMinorTick','on')
title('Heading angle')
ylabel('\psi(t) in deg')
xlabel('t in sec')
legend('Actual','Location','Best')
legend boxoff
box off

% Comparison of rudder angle
figure()
plot(t,saturate(delta*180/pi,35),'r')
set(gca,'YMinorTick','on')
set(gca,'XMinorTick','on')
title('Rudder Angle')
ylabel('\delta(t) in deg')
xlabel('t in sec')
legend('Actual','Location','Best')
legend boxoff
box off