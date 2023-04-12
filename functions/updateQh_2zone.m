% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [tclParameters] = updateQh_2zone(tclParameters, m)
%{
Updates the heat gain from the A/C based on the new outdoor temperature and
the ON/OFF mode of each zone.
Input:
    tclParameters: struct
        Info for the TCL population as e.g. coefficients that determine the
        relationship between outdoor temperature and Qh
    m: vector of booleans
        ON/OFF mode of each TCL. For 2-zone houses, equal to 1 if the zone is
        cooling and 0 if not. For single-zone houses, 1 if the compressor
        is ON and 0 if it is OFF.
Output:
    tclParamaters: struct
        Updated struct with the adjusted Qh values based on outdoor temp.
%}

%% update Qh based on outdoor temperature
Btu_per_hr_to_kW = 3412.142^-1;

T_a = tclParameters.T_a;
designTotalCoolingBtu = tclParameters.designTotalCoolingBtu;
latentCooling = tclParameters.latentCooling;
K_0 = tclParameters.K_0;
K_1 = tclParameters.K_1;

coolCapacityBasedOnTout = K_0 + K_1 * (T_a*1.8+32); % T_out assumed to be in in degF
coolingCapacity = (designTotalCoolingBtu .* coolCapacityBasedOnTout).*Btu_per_hr_to_kW;
Q_h = -coolingCapacity/(1+latentCooling); %[kW]

%% Adjust Qh for the 2-zone houses
both_on = m(tclParameters.tcls_2zone_first)==1 & m(tclParameters.tcls_2zone_second)==1;
Q_h(tclParameters.tcls_2zone_first(both_on)) = Q_h(tclParameters.tcls_2zone_first(both_on))/2;
Q_h(tclParameters.tcls_2zone_second(both_on)) = Q_h(tclParameters.tcls_2zone_second(both_on))/2;

%% Store info
P_trans = abs(Q_h); 
tclParameters.coolingCapacity = coolingCapacity;
tclParameters.Q_h = Q_h;
tclParameters.P_trans = P_trans;

end
