% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [m_new,logical_index_high,logical_index_low] = thermostatLogic_2zone(tclParameters, T_max, T_min, theta_a_new, m_vect, locked)
%{
This function checks which TCLs are outside their deadband after the last
state update. Lockout constraint is taken into account as well the state of
the other zone of the house.
Input:
    tclParameters: struct
        Contains info regarding the TCLs.
    T_max: vector of floats
        Upper deadband limit of each TCL.
    T_min: vector of floats
        Lower deadband limit of each TCL.
    theta_a_new: vector of floats
        New air temperature of each TCL.
    m_vect: vector of floats
        Former on/off mode of each TCL.
    locked: vector of booleans
        Indicator of whether each TCL is locked or not.
Output:
    m_new: vector of booleans
        Updated on/off modes of the compressor
    logical_index_high: vector of booleans
        Indicator of whether each TCL has exceeded its upper deadband limit
    logical_index_low: vector of booleans
        Indicator of whether each TCL has exceeded its lower deadband limit
%}

%% Load info from struct
% indices corresponding to 1st, 2nd zones and single-zone houses
tcls_2zone_first = tclParameters.tcls_2zone_first;
tcls_2zone_second = tclParameters.tcls_2zone_second;
tcls_1zone = tclParameters.tcls_1zone;
% total number of TCLs
numTCL = length(tclParameters.U_a);

%% Check upper limit of the deadband
% to switch a zone ON, either both zones need to be unlocked or the other zone must already be ON
% check whether the temperatures of the 1st zones are high
logical_index_high_2zone_first = theta_a_new(tcls_2zone_first) > T_max(tcls_2zone_first)...
                                 & ( (locked(tcls_2zone_first) == 0 & locked(tcls_2zone_second) == 0) | (m_vect(tcls_2zone_second) == 1) );
% check whether the temperatures of the 2nd zones are high
logical_index_high_2zone_second = theta_a_new(tcls_2zone_second) > T_max(tcls_2zone_second)...
                                 & ( (locked(tcls_2zone_first) == 0 & locked(tcls_2zone_second) == 0) | (m_vect(tcls_2zone_first) == 1) ); 

% check whether the temperatures of the 1-zone TCLs are high
logical_index_high_1zone = theta_a_new(tcls_1zone) > T_max(tcls_1zone)...
                                 & locked(tcls_1zone) == 0;
                             
%% Check lower limit of the deadband
% to switch a zone OFF, either both zones need to be unlocked or the other zone must already be ON
% check whether the temperatures of the 1st zones are low
logical_index_low_2zone_first = theta_a_new(tcls_2zone_first) < T_min(tcls_2zone_first)...
                                 & ( (locked(tcls_2zone_first) == 0 & locked(tcls_2zone_second) == 0) | (m_vect(tcls_2zone_second) == 1) );
% check whether the temperatures of the 2nd zones are low
logical_index_low_2zone_second = theta_a_new(tcls_2zone_second) < T_min(tcls_2zone_second)...
                                 & ( (locked(tcls_2zone_first) == 0 & locked(tcls_2zone_second) == 0) | (m_vect(tcls_2zone_first) == 1) );

% check whether the temperatures of the 1-zone TCLs are low
logical_index_low_1zone = theta_a_new(tcls_1zone) < T_min(tcls_1zone)...
                                 & locked(tcls_1zone) == 0 ;
                             
%% Collect indices
% collect the indeces for tcls outside deadband, each entry corresponds
% to one TCL, regardless of the zone number
logical_index_high = zeros(numTCL, 1); logical_index_low = zeros(numTCL, 1);
logical_index_high([tcls_2zone_first(logical_index_high_2zone_first) ...
                    tcls_2zone_second(logical_index_high_2zone_second) ...
                    tcls_1zone(logical_index_high_1zone)]) = 1;
logical_index_low([tcls_2zone_first(logical_index_low_2zone_first) ...
                    tcls_2zone_second(logical_index_low_2zone_second) ...
                    tcls_1zone(logical_index_low_1zone)]) = 1; 
logical_index_high = logical(logical_index_high);
logical_index_low = logical(logical_index_low);

%% Carry out thermostat conntrol actions
m_new = m_vect;
m_new(logical_index_high) = 1;
m_new(logical_index_low) = 0;

end

