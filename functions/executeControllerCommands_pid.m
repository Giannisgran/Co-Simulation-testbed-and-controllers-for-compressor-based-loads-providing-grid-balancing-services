% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [m_vect] = executeControllerCommands_pid(numTCL, onOffSignal, m_vect, logical_index_high, logical_index_low, locked, controllerInformation, useLocalVoltage, tclVoltage)
%{
This function carries out the PID controller commands.
Each TCL first picks the correct probability to act upon, then draws a
random number and decides whether to switch or not.
Input:
    numTCL: int
        Number of TCLs.
    onOffSignal: float
        Switching probability from PID controller. Same for all TCLs.
    m_vect: vector of booleans
        ON/OFF mode of each TCL at the current timestep.
    logical_index_high: vector of booleans
        Indicator of whether each TCL has exceeded its upper deadband limit
    logical_index_low: vector of booleans
        Indicator of whether each TCL has exceeded its lower deadband limit
    controllerInformation: struct
        Info for the controller, includes voltage limits if we want to take
        them into account in the switch actions
    useLocalVoltage: boolean
        1 if we want to take into account voltage in the switch actions
    tclVoltage: vector of floats
        Voltage at 240V level of each TCL
Output:
    m_vect: vector of booleans
        Updated ON/OFF mode for each TCL after taking into account the
        switching probabilities send by the controller.
%}
%% Draw a random number for each TCL
randomValueAtTcl = rand(numTCL,1);
%% same control signal is send to all TCLs
onOffProbability = onOffSignal*ones(numTCL, 1);
%% check probabilities from the controller
if useLocalVoltage == 0 % voltage not take into account
    m_vect(randomValueAtTcl < onOffProbability & 0 < onOffProbability ...
               & logical_index_high==0 & logical_index_low==0 & locked==0) = 1;
    % same as above but using negative values for off switching
    m_vect(randomValueAtTcl < abs(onOffProbability) & 0 > onOffProbability...
               & logical_index_high==0 & logical_index_low==0 & locked==0) = 0; 
else
    % load voltage limits
    vPlus = controllerInformation.upperVoltage;
    vMinus = controllerInformation.lowerVoltage;
    % find TCLs whose voltage is inside the voltage limits
    vFlag = tclVoltage <= vPlus & tclVoltage >= vMinus;
    % switching logic that requires the same as above, except if
    % tcls are over or under voltage they do not respond
    m_vect(randomValueAtTcl < onOffProbability & 0 < onOffProbability ...
               & logical_index_high==0 & logical_index_low==0 & locked==0 & vFlag) = 1;
    % same as above but using negative values for off switching
    m_vect(randomValueAtTcl < abs(onOffProbability) & 0 > onOffProbability...
               & logical_index_high==0 & logical_index_low==0 & locked==0 & vFlag) = 0;    
end

