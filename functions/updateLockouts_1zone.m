% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [timeUntilUnlocked, remainedOn, remainedOff, justTurnedOn, justTurnedOff] = updateLockouts_1zone(m_new, m_vect, tclParameters, timeUntilUnlocked)
%{
This function updates the lockout times of every TCL
Input:
    m_new: vector of booleans
        Contains the new on/off mode after the controller/thermostat actions.
    m_vect: vector of booleans
        Contains the previous on/off mode of each TCL.
    tclParameters: struct
        Contains info for the TCLs, e.g. lockout time etc.
    timeUntilUnlocked: vector of floats
        Time remaining until the lockout of each TCL ends
Output:
    timeUntilUnlocked: vector of floats
        Updated time remaining until the lockout of each TCL ends
    remainedOn: vector of booleans
        Contains 1 to the positions of TCLs that remained ON.
    remainedOff: vector of booleans
        Contains 1 to the positions of TCLs that remained OFF.
    justTurnedOn: vector of booleans
        Contains 1 to the positions of TCLs that switched from OFF to ON.
    justTurnedOff: vector of booleans
        Contains 1 to the positions of TCLs that switched from ON to OFF.
%}


% the appliance just turned off if it is now off, and if it was on at the
% beginning of the time-step
justTurnedOff = m_new == 0 & abs(m_new-m_vect)==1 ;
justTurnedOn = m_new == 1 & abs(m_new-m_vect)==1 ;

% reduce the lockout time of every other unit (that is off) by 1
% check if it remained off
remainedOff = m_new == 0 & abs(m_new-m_vect)==0 ;
remainedOn = m_new == 1 & abs(m_new-m_vect)==0 ;

% reduce the lockout time
timeUntilUnlocked(remainedOff) = timeUntilUnlocked(remainedOff) - 1;
timeUntilUnlocked(remainedOn) = timeUntilUnlocked(remainedOn) - 1;

% set the lockout time of the units that just switched to the max
timeUntilUnlocked(justTurnedOff) = tclParameters.lockoutTimeOffInTimeSteps(justTurnedOff);
timeUntilUnlocked(justTurnedOn) = tclParameters.lockoutTimeOnInTimeSteps(justTurnedOn);

timeUntilUnlocked (timeUntilUnlocked <0) = 0;
end

