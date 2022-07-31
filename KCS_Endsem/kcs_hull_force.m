function tau_H = kcs_hull_force(up, vp, rp)

b = atan2(-vp, up);

% Surge Hydrodynamic Derivatives in non-dimensional form
X0 = -0.0167;
Xbb = -0.0549;
Xbr_minus_my = -0.1084;
Xrr = -0.0120;
Xbbbb = -0.0417;

% Sway Hydrodynamic Derivatives in non-dimensional form
Yb = 0.2252;
Yr_minus_mx = 0.0398;
Ybbb = 1.7179;
Ybbr = -0.4832;
Ybrr = 0.8341;
Yrrr = -0.0050;

% Yaw Hydrodynamic Derivatives in non-dimensional form
Nb = 0.1111;
Nr = -0.0465;
Nbbb = 0.1752;
Nbbr = -0.6168;
Nbrr = 0.0512;
Nrrr = -0.0387;

% Non-dimensional Surge Hull Hydrodynamic Force
Xp_H = X0 * (up^2) + Xbb * (b^2) + Xbr_minus_my * b * rp ...
    + Xrr * (rp^2) + Xbbbb * (b^4);

% Non-dimensional Sway Hull Hydrodynamic Force
Yp_H = Yb * b + Yr_minus_mx * rp + Ybbb * (b ^ 3) ...
    + Ybbr * (b ^ 2) * rp + Ybrr * b * (rp ^ 2) + Yrrr * (rp ^ 3);

% Non-dimensional Yaw Hull Hydrodynamic Moment
Np_H = Nb * b + Nr * rp + Nbbb * (b^3) + Nbbr * (b^2) * rp ...
    + Nbrr * b * (rp^2) + Nrrr * (rp^3);

% Nondimensional force vector
tau_H = [Xp_H; Yp_H; Np_H;];