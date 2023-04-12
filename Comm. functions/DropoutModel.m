% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [DDM] = DropoutModel(IM,CM, numSteps, scenario)

rng('shuffle')

% we want this to be the mean of a bernoulli distribution, so no need to
% generate a std. dev. Just generate the probabilities for each link. 
if scenario == 2
    % 0-5% packet loss rate, so generate uniform random [.95, 1] to
    % describe mean bernoulli rate of successful packet transmission
    noDropoutProbability = rand(size(CM))*.05+.95; 
elseif scenario == 3
    % 5-10% packet loss rate
    noDropoutProbability = rand(size(CM))*.1+.90; 
else
    error('Wrong comm scenario setting')
end

% set the means for the bernouli distributions here
DDM = zeros(size(CM));
DDM(CM ~= 0) = noDropoutProbability(CM ~= 0);

end
