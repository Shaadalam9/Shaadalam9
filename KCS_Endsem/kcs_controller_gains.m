function [Kp, Kd] = kcs_controller_gains(params)
   
%**************************************************************************
% Your code here to compute the gains for the PD controller you are tuning
%Nomoto Time Constant ==> T
T = params.rudderTC;
% Nomoto Gain ==> K
K = params.propTC;

delta_max = params.delta_max;
% ey and zeta can be tuned by us
ey = 2000;
% Since the value of wn should lie between 0.1 to 0.01. Therefore ey should
% lie between 170 to 15000
zeta = 0.25;

Kp = delta_max / ey;
w_psi = sqrt(K*Kp/T);
Kd= ((2*zeta*w_psi*T)-1)/K;