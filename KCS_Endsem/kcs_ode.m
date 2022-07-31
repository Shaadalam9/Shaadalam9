function xd = kcs_ode(t,x,xh,params)
%KCS_ODE    ODE system for simulating dynamics of KCS Model (scale 1:75.5)
%   KCS_ODE(t, x, xh, params) outputs the 8 x 1 derivative vector xd
%   corresponding to the state vector x at time t. The elements of the
%   state vector x are:
%
%   u = x(1)        Surge velocity of the vessel
%   v = x(2)        Sway velocity of the vessel
%   r = x(3)        Yaw rate of the vessel
%   xn = x(4)       x-position of the vessel in NED frame
%   yn = x(5)       y-position of the vessel in NED frame
%   psi = x(6)      Heading angle of the vessel wrt x-axis of NED frame
%   delta = x(7)    Rudder angle
%   n_prop = x(8)   Propeller speed (RPS)
%
%   params is a data structure that contains the essential parameters to
%   simulate the dynamics of the vessel. xh is the estimated state
%   vector obtained from state estimation. It's size is same as x in this
%   problem.
%
%   Authored by: Abhilash Somayajula 12/03/2022

u = x(1);       % Surge velocity of the vessel
v = x(2);       % Sway velocity of the vessel
r = x(3);       % Yaw rate of the vessel
xn = x(4);      % x-position of the vessel in NED frame
yn = x(5);      % y-position of the vessel in NED frame
psi = x(6);     % Heading angle of the vessel wrt x-axis of NED frame
delta = x(7);   % Rudder angle
n_prop = x(8);  % Propeller speed (RPS)

rho = params.rho;
L = params.L;
U = params.U;
d_em = params.T;
Dsp = params.Dsp;

% Non-dimensional velocities
up = u/U;
vp = v/U;
rp = r*L/U;

% Saturate rudder angle
delta = saturate(delta, params.delta_max);

% Non-dimensional hull force vector
tau_H = kcs_hull_force(up, vp, rp);

% Dimensionalize hull force vector
tau_H(1) = tau_H(1) * (1/2 * rho * U^2 * L * d_em);
tau_H(2) = tau_H(2) * (1/2 * rho * U^2 * L * d_em);
tau_H(3) = tau_H(3) * (1/2 * rho * U^2 * L^2 * d_em);

% Dimensional propeller force vector
tau_P = kp505_model(u, n_prop, params.pot);

% Non-dimensional rudder force vector
tau_R = kcs_rudder_force(u, v, r, delta, n_prop, params);

% Dimensionalize rudder force vector
tau_R(1) = tau_R(1) * (1/2 * rho * U^2 * L * d_em);
tau_R(2) = tau_R(2) * (1/2 * rho * U^2 * L * d_em);
tau_R(3) = tau_R(3) * (1/2 * rho * U^2 * L^2 * d_em);

% Coriolis terms
mp = Dsp / (0.5 * d_em * (L ^ 2));
xGp = params.rG(1) / L;

% Non-dimensional Coriolis forces and moments
Xp_C = mp * vp * rp + mp * xGp * (rp ^ 2);
Yp_C = -mp * up * rp;
Np_C = -mp * xGp * up * rp;

% Non-dimensional Coriolis force vector
tau_C = [Xp_C; Yp_C; Np_C;];

% Dimensionalize Coriolis force vector
tau_C(1) = tau_C(1) * (1/2 * rho * U^2 * L * d_em);
tau_C(2) = tau_C(2) * (1/2 * rho * U^2 * L * d_em);
tau_C(3) = tau_C(3) * (1/2 * rho * U^2 * L^2 * d_em);

% Dimensional total force vector
tau = tau_H + tau_P + tau_R + tau_C;

% Dimensional total mass matrix
M = params.M_RB + params.M_A;

% Derivative of velocities
vel_der = M\tau;

% Derivative of state vector
xd = zeros(8,1);

xd(1:3) = vel_der;
xd(4) = u * cos(psi) - v * sin(psi);
xd(5) = u * sin(psi) + v * cos(psi);
xd(6) = r;

% Desired trajectory from guidance system
[psid, ypd_int] = kcs_guidance_waypoint(t,xh,params);
xd(9) = ypd_int;

% Commanded rudder angle from control system
[delta_c, n_c] = kcs_control(t, psid, xh, params);

% Rudder rate
deltad = saturate((delta_c - delta)/params.rudderTC, params.deltad_max);
xd(7) = deltad;

% Propeller rate
nd = (n_c - n_prop)/params.propTC;
xd(8) = nd;



