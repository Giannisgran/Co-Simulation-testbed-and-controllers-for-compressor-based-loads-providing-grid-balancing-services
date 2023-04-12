% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [Q_s, Q_m] = updateQsQm(irradiance, SHGC, Q_i, f_i, f_s)
%{
This function updates the solar heat gain Qs of each TCL based on the given
irradiance value and afterwards the corresponding internal heat Gains Qm.
Input:
    irradiance: float
        Value of solar irradiance in btu/(hr*ft^2). Assumed the same for all TCLs.
    SHGC: vector of floats
        Solar heat gain coefficient
    Q_i: vector of floats
        Portion of the heat gains the goes to the internal mass
    f_i: float
        Portion of the internal heat gains the goes to the solid mass
    f_s: float
        Portion of the solar heat gains the goes to the solid mass
Output:
    Q_s: vector of floats
        Updated solar heatgains
    Q_m: vector of floats
        Updated heatgains to the solid mass
%}

% variables used during convertion
Btu_per_hr_to_kW = 3412.142^-1;
% compute solar heat gain
Q_s = SHGC.*irradiance; % [btu / hr]
% convert to kW
Q_s = Q_s*Btu_per_hr_to_kW; % [kW]
% scale it down because the value is large otherwise
Q_s = Q_s / 2.5;
% recompute Qm based on the new Qs
Q_m = f_i*Q_i + f_s*Q_s; % [kW]

end

