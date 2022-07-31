function tau_P = kp505_model(u, n, pot)
%KP505_MODEL  Calculates propeller thrust and torque
%   KP505_MODEL(u, n, pot) calculates the thrust T and the optional torque
%   Q of the propeller for a specified surge speed u and propeller speed n
%   expressed in RPS. 
%
%   pot denotes a data structure that contains the curve fit information 
%   derived from the propeller open water test.
%
%   The value propulsion parameters like effective wake fraction and thrust 
%   deduction factor are taken from Yoshimura's SIMMAN paper:
%
%   Analysis of steady hydrodynamic force components and prediction of
%   manoeuvering ship motion with KVLCC1, KVLCC2 and KCS (Available on
%   ResearchGate)
%
%   Authored by: Abhilash Somayajula 12/03/2022

% Effective Wake Fraction of the Propeller
wp = 1 - 0.645;

% Thrust Deduction Factor
tp = 1 - 0.793;

% Density of Fresh Water
rho = 1000;

% Propeller diameter
Dp = 7.9/75.5;

% Advance Coefficient
J = u * (1 - wp) / (n * Dp);

% Thrust Coefficient
Kt = polyval(pot.pt,J);

% Thrust
T = (1 - tp) * rho * Kt * Dp^4 * n^2;

% Propeller force vector (dimensional)
tau_P = [T; 0; 0;];