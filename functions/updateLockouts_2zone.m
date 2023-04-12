% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [timeUntilUnlocked, remainedOn, remainedOff, justTurnedOn, justTurnedOff] = updateLockouts_2zone(m_new, m_vect, tclParameters, timeUntilUnlocked)
%{
This function updates the lockout times of every TCL
Input:
    m_new: vector of booleans
        Contains the new on/off mode after the controller/thermostat actions.
    m_vect: vector of booleans
        Contains the previous on/off mode of each TCL.
    tclParameters: struct
        Contains info for the TCLs, e.g. which are single or two zone and
        how many timesteps are required for the lockout to end.
    timeUntilUnlocked: vector of floats
        Time until each TCL gets unlocked
Output:
    tclParameters: vector of ints
        Updated lockout times.   
    remainedOn: vector of booleans
        Contains 1 to the positions of TCLs that remained ON.
    remainedOff: vector of booleans
        Contains 1 to the positions of TCLs that remained OFF.
    justTurnedOn: vector of booleans
        Contains 1 to the positions of TCLs that switched from OFF to ON.
    justTurnedOff: vector of booleans
        Contains 1 to the positions of TCLs that switched from ON to OFF.
%}

%% Get the indices corresponding to 1st, 2nd zones and 1-zone TCLs
tcls_2zone_first = tclParameters.tcls_2zone_first;
tcls_2zone_second = tclParameters.tcls_2zone_second;

%% Find which devices switched and which didn't state status
% the appliance just turned off if it is now off, and if it was on at the beginning of the time-step
justTurnedOff = m_new == 0 & abs(m_new-m_vect)==1 ;
justTurnedOn = m_new == 1 & abs(m_new-m_vect)==1 ;
% check witch TCLs didn't change state
remainedOff = m_new == 0 & abs(m_new-m_vect)==0 ;
remainedOn = m_new == 1 & abs(m_new-m_vect)==0 ;

% a TCL has switched OFF if one zone switched OFF, while the other one
% was already OFF or both zones switched OFF at the same time
justTurnedOff_2zone = (justTurnedOff(tcls_2zone_first)==1 & m_new(tcls_2zone_second)==0) ...
                     |(justTurnedOff(tcls_2zone_second)==1 & m_new(tcls_2zone_first)==0) ...
                     |(justTurnedOff(tcls_2zone_first)==1 & justTurnedOff(tcls_2zone_second)==1);
% register only the zone that turned OFF, not both of them - will make lockouts equal further down
justTurnedOff(tcls_2zone_first) = justTurnedOff_2zone & m_new(tcls_2zone_first)==0;
justTurnedOff(tcls_2zone_second) = justTurnedOff_2zone & m_new(tcls_2zone_second)==0;

% a TCL has switched ON if one zone switched ON, while the other one
% was OFF or both zones switched ON at the same time
justTurnedOn_2zone =  (justTurnedOn(tcls_2zone_first)==1 & m_new(tcls_2zone_second)==0) ...
                     |(justTurnedOn(tcls_2zone_second)==1 & m_new(tcls_2zone_first)==0) ...
                     |(justTurnedOn(tcls_2zone_first)==1 & justTurnedOn(tcls_2zone_second)==1);
% register only the zone that turned ON, not both of them - will make lockouts equal further down
justTurnedOn(tcls_2zone_first) = justTurnedOn_2zone & m_new(tcls_2zone_first)==1;
justTurnedOn(tcls_2zone_second) = justTurnedOn_2zone & m_new(tcls_2zone_second)==1;

%% Update lockout times
% reduce the lockout time
timeUntilUnlocked(remainedOff) = timeUntilUnlocked(remainedOff) - 1;
timeUntilUnlocked(remainedOn) = timeUntilUnlocked(remainedOn) - 1;
% NOTE: remainedOff/on is not 100% correct for 2zones and may reduce
% the time until unlock in some zones unnecessarily, but we are
% updating the lock times of tcls that just switched after that

% set the lockout time of the units that just switched to the max
timeUntilUnlocked(justTurnedOff) = tclParameters.lockoutTimeOffInTimeSteps(justTurnedOff);
timeUntilUnlocked(justTurnedOn) = tclParameters.lockoutTimeOnInTimeSteps(justTurnedOn);
% set everything below zero to zero
timeUntilUnlocked (timeUntilUnlocked <0) = 0;

% make the remaining lockout times for each zone equal
% one zone would be zero and the other one would have max lockout after a switch, so we need to correct that
timeUntilUnlocked(tcls_2zone_first) = max(timeUntilUnlocked(tcls_2zone_first), timeUntilUnlocked(tcls_2zone_second));
timeUntilUnlocked(tcls_2zone_second) = max(timeUntilUnlocked(tcls_2zone_first), timeUntilUnlocked(tcls_2zone_second));

end

