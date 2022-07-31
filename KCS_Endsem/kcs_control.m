function [delta_c, n_c] = kcs_control(t, psid, x, params)

n_c = 18.15;    % Do not change this value. Rather only play with rudder angle

psi = x(6);
r = x(3);

% Get the values of the gains from params data structure
Kp = params.Kp;
Kd = params.Kd;

%**************************************************************************
% Your code here to compute commanded rudder angle delta_c
error = psi - psid;
der_err = r;
delta_c = -Kp*ssa(error) - Kd*der_err;