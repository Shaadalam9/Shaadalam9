function pot = kp505_pot()
%KP505_POT  Curve fit propeller open water data
%   KP505_POT() returns the fit parameters for the propller open water test
%   data of the scaled KP505 propeller (scale = 1:75.5). The open water
%   test was performed by NMRI for the SIMMAN 2008.

% Propeller open water data
pot_data = [
    0.000	0.5327	0.7517
0.100	0.4937	0.7058
0.150	0.4719	0.6813
0.200	0.4469	0.6538
0.250	0.4208	0.6232
0.300	0.3922	0.5895
0.350	0.3657	0.5589
0.400	0.3425	0.5314
0.450	0.3143	0.5008
0.500	0.2895	0.4702
0.550	0.2647	0.4396
0.600	0.2407	0.4090
0.650	0.2162	0.3784
0.700	0.1931	0.3478
0.700	0.1943	0.3478
0.750	0.1688	0.3172
0.800	0.1414	0.2805
0.850	0.1148	0.2468
0.900	0.0870	0.2132
0.950	0.0581	0.1704
1.000	0.0293	0.1275
1.050	-0.0033	0.0786];

% Extract data
J = pot_data(:,1);
Kt = pot_data(:,2);
Kq = pot_data(:,3)/10;

% Curve fit using a quadratic polynomial fit using polyfit
pt = polyfit(J,Kt,2);
pq = polyfit(J,Kq,2);

% Output the fit coefficients as a data structure
pot = struct;
pot.pt = pt;
pot.pq = pq;
