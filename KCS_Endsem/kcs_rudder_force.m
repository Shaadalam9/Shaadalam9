function tau_R = kcs_rudder_force(u, v, r, delta, n, params)

% Effective Wake Fraction of the Propeller
wp = 1 - 0.645;

% Propeller diameter
Dp = 7.9/75.5;

% Advance Coefficient
J = u * (1 - wp) / (n * Dp);

% Thrust Coefficient
pot = params.pot;
Kt = polyval(pot.pt,J);

% Parameters
L = params.L;
d_em = params.T;
U = params.U;

% Non-dimensionalize velocities
up = u/U;
vp = v/U;
rp = r*L/U;

% Rudder Force Calculation
A_R = L * d_em / 54.86;
Lamda = 2.164;
f_alp = 6.13 * Lamda / (2.25 + Lamda);

eps = 0.956;
eta = 0.7979;
kappa = 0.633;
xp_P = -0.4565;  % Assuming propeller location is 10 m ahead of AP (Rudder Location)
xp_R = -0.5;

b = atan2(-v,u);
b_p = b - xp_P * rp;

if b_p > 0
    gamma_R = 0.492;
else
    gamma_R = 0.338;
end

lp_R = -0.755;

% Non-dimensional flow velocities at the rudder
up_R = eps * (1 - wp) * up * sqrt(eta * (1 + kappa * (sqrt(1 + 8 * Kt /(pi * (J ^ 2)) ) - 1)) ^ 2 + (1 - eta));

vp_R = gamma_R * (vp + rp * lp_R);

Up_R = sqrt(up_R ^ 2 + vp_R ^ 2);
alpha_R = delta  - atan2(-vp_R, up_R);

F_N = A_R / (L * d_em) * f_alp * (Up_R ^ 2) * sin(alpha_R);

tR = 1 - 0.742;
aH = 0.361;
xp_H = -0.436;

% Non-dimensional forces and moments
Xp_R = - (1 - tR) * F_N * sin(delta);
Yp_R = - (1 + aH) * F_N * cos(delta);
Np_R = - (xp_R + aH * xp_H) * F_N * cos(delta);

% Non-dimensional rudder force vector
tau_R = [Xp_R; Yp_R; Np_R;];