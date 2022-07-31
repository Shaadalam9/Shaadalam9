function [Yd, Nd] = kcs_rudder_derivatives(pot)

rho = 1000;
g = 9.80665;

L = 230/75.5;
d_em = 10.8/75.5;
A_R = 0.0182 * L * d_em;

w = 1 - 0.645;
n = 18.15;
Dp = 7.9 / 75.5;

lambda = 2.164;
eta = 0.7979;
epsilon = 0.956;
kappa = 0.633;

u = 0.26 * sqrt(g * 230/75.5);
J = u * (1 - w) / (n * Dp);

K_T = polyval(pot.pt, J);

uR = u * epsilon * (1 - w) * sqrt(eta * (1 + kappa * (sqrt(1 + 8*K_T/pi/J^2) - 1))^2 + (1 - eta));

f_alpha = 6.13 * lambda / (2.25 + lambda);

FN = 1/2 * rho * f_alpha * A_R * uR^2 ;

xH = -0.436;
aH = 0.361;
xR = -0.5;

Yd = - (1 + aH) * FN;
Nd = -(xR + aH*xH) * L * FN;
