% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [tclParameters] = updateTempDelays(tclParameters, remainedOn, remainedOff, justTurnedOff, justTurnedOn, timeStepInSec)
% This function updates the temperature delay times, i.e. the time needed
% for the temperature to start moving towards the opposite direction after
% a switch occurs.
% Input:
%   tclParameters : struct
%       Struct with all the parameters of the TCLs
%   remainedOn : vector
%       Logical value for each TCL that indicates whether it remained ON
%   remainedOff : vector
%       Logical value for each TCL that indicates whether it remained OFF
%   justTurnedOn : vector
%       Logical value for each TCL that indicates whether it switched ON
%   justTurnedOff : vector
%       Logical value for each TCL that indicates whether it switched OFF
%   timeStepInSec : int
%       Timestep of the simulation

timeUntilDelayEnds = tclParameters.timeUntilDelayEnds;
delayed = timeUntilDelayEnds > 0;  % delayed TCLs
% if a TCL switches while already being delayed (as a result of a
% controller), then we don't put it in new delay mode but zero it out
tclsToUpdateOn = justTurnedOn & ~delayed;  % will be ON delayed
tclsToUpdateOff = justTurnedOff & ~delayed;  % will be OFF delayed

% set delay time for the TCLs that have juts switched
if tclParameters.temperatureDelayOption == 4
    % outdoor temp and setpoint for each tcl that switched
    To_off = tclParameters.T_a(tclsToUpdateOff);
    To_on = tclParameters.T_a(tclsToUpdateOn);
    T_sp_off = tclParameters.T_sp(tclsToUpdateOff);
    T_sp_on = tclParameters.T_sp(tclsToUpdateOn);
    % find delay distributions corresponding to the TCLs that switched
    delayDistributions_off = tclParameters.tempDelayObjects(tclParameters.tempDelayDistribution(tclsToUpdateOff));
    delayDistributions_on  = tclParameters.tempDelayObjects(tclParameters.tempDelayDistribution(tclsToUpdateOn));
    % preallocate for delays
    delay_off = zeros(length(delayDistributions_off),1);
    for i_off = 1 : length(delayDistributions_off) % loop through the delay distributions of the TCLs that switched OFF
        d = delayDistributions_off{i_off}{2}; % load object for off delay
        curr_bin = discretize(To_off(i_off),d.res_bin_edges); % current bin according to the outdoor temp
        % generate residual from the Gaussian distribution
        res = normrnd(d.res_bin_mean(curr_bin), d.res_bin_std(curr_bin));
        % delay = fitted value + residual
        if d.num_ind_vars == 1
            delay_off(i_off) = d.delayFit(To_off(i_off)) + res;
        else
            delay_off(i_off) = d.delayFit(To_off(i_off), T_sp_off(i_off)) + res;
        end
    end
    % same proccedure for the ON delays
    delay_on = zeros(length(delayDistributions_on),1);
    for i_on = 1 : length(delayDistributions_on) % loop through the delay distributions of the TCLs that switched ON
        d = delayDistributions_on{i_on}{1}; % load object for ON delay
        curr_bin = discretize(To_on(i_on),d.res_bin_edges); % current bin according to the outdoor temp
        % generate residual from the Gaussian distribution
        res = normrnd(d.res_bin_mean(curr_bin), d.res_bin_std(curr_bin));
        % delay = fitted value + residual
        if d.num_ind_vars == 1
            delay_on(i_on) = d.delayFit(To_on(i_on)) + res;
        else
            delay_on(i_on) = d.delayFit(To_on(i_on), T_sp_on(i_on)) + res;
        end
    end    
    delayTimeOnInTimeSteps = delay_on/timeStepInSec; 
    delayTimeOffInTimeSteps = delay_off/timeStepInSec;
else
    delayTimeOnInTimeSteps = tclParameters.delayTimeOnInTimeSteps(tclsToUpdateOn);    
    delayTimeOffInTimeSteps = tclParameters.delayTimeOffInTimeSteps(tclsToUpdateOff);
end
% delay for the TCLs that just switched
timeUntilDelayEnds(tclsToUpdateOn) = delayTimeOnInTimeSteps;
timeUntilDelayEnds(tclsToUpdateOff) = delayTimeOffInTimeSteps;
% reduce the delay time
timeUntilDelayEnds(remainedOn) = timeUntilDelayEnds(remainedOn) - 1;
timeUntilDelayEnds(remainedOff) = timeUntilDelayEnds(remainedOff) - 1;

% if a TCL was delayed but it was switched (due to a controller) then we
% set its delay time to zero
timeUntilDelayEnds((justTurnedOn | justTurnedOff) & delayed) = 0;

timeUntilDelayEnds(timeUntilDelayEnds <0) = 0;
tclParameters.timeUntilDelayEnds = timeUntilDelayEnds;

% keep generated delays
if ~isempty(delayTimeOnInTimeSteps)
    tclParameters.delays_on_generated = [tclParameters.delays_on_generated;delayTimeOnInTimeSteps];
end
if ~isempty(delayTimeOffInTimeSteps)
    tclParameters.delays_off_generated = [tclParameters.delays_off_generated;delayTimeOffInTimeSteps];
end

end