% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [MDM, SDM, distribution] = DelayModel(CM, numSteps, scenario,timeStepInSec)
rng('shuffle')

% assign mean and standard deviation based on scenarios
if scenario == 2
    % mean of 3 seconds
    % std dev of 1.155
    % uniform distribution
    distribution = 'uniform';
    delayMean = 3;
    delayStdDev = 1.115;
elseif scenario == 3
    % mean 18
    % std dev 3
    % distribution gaussian
    distribution = 'gaussian';
    delayMean = 18;
    delayStdDev = 3;
else
    error('Wrong comm scenario setting')
end


% get the mean of delays on each link based on the link type
MDM = delayMean.*ones(size(CM))./timeStepInSec;
MDM(CM == 0) = 0;

% get the matrix containing standard deviations for each link
SDM = delayStdDev.*ones(size(CM))./timeStepInSec;
SDM(CM == 0) = 0;

end