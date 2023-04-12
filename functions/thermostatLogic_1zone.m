% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [m_vect_new, logical_index_high, logical_index_low] = thermostatLogic_1zone(T_max, T_min, theta_a_new, onOffSwitchValue, locked)
%{
This function checks which TCLs are outside their deadband after the last
state update. Lockout constraint is taken into account.
Input:
    T_max: vector of floats
        Upper deadband limit of each TCL.
    T_min: vector of floats
        Lower deadband limit of each TCL.
    theta_a_new: vector of floats
        New air temperature of each TCL.
    onOffSwitchValue: vector of booleans
        Current ON/OFF status of each TCL
    locked: vector of booleans
        Locked mode of each TCL
Output:
    m_vect_new: vector of booleans
        Updated ON/OFF mode of each TCL after the thermostat's actions
    locked: vector of booleans
        Indicators of 1 when a TCL is locked and 0 when not
    logical_index_high: vector of booleans
        Indicator of whether each TCL has exceeded its upper deadband limit
    logical_index_low: vector of booleans
        Indicator of whether each TCL has exceeded its lower deadband limit
%}

%% Check upper limit of the deadband
% check whether the temperature is too high and the TCL is not locked
logical_index_high = theta_a_new > T_max & locked == 0;

%% Check lower limit of the deadband
% check whether the temperature is too low and the TCL is not locked
logical_index_low = theta_a_new < T_min & locked == 0;

%% Carry out thermostat control
m_vect_new = onOffSwitchValue;
m_vect_new(logical_index_high) = 1;
m_vect_new(logical_index_low) = 0;

end
