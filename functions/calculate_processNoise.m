% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [Q] = calculate_processNoise(binModel, states_mat, transitionMatrix, i, measPaggNoise, use_full_Q)
%{
This function finds the process noise covariance based on two options.
The first (use_full_Q=1) is to search for a Q that does not lead
to numerical issues with the Kalman filter. If there is ill-conditioning,
it start to prune the error terms considered in the process noise. If no
subset of errors that leads to well-conditioned Q matrix is found, the
diagonal of Q is returned.
The second option (use_full_Q=0) is to return the diagonal of Q.
%}
%% Find the errors
% find process noise, already kept the simulated-measured states
% predictions based on the transition matrix
x_pred = states_mat*transitionMatrix';
% error between prediction and 'measurement' - columns->bins
err = x_pred(1:end-1,:) - states_mat(2:end,:);
%% Case that we don't need the full Q matrix: keep only the diagonal
if ~use_full_Q
    % find the covariance of the error - this will be the process noise
    Q = diag(diag( cov(err) ));
    fprintf('Using only the diagonal of Q for Kalman filter \n')
    return
end
%% Try finding a well-conditioned full Q matrix
% keep number of error terms ignored
err_terms_ignored = 0;
%% check case with output feedback and pseudo-measurement to force estimated states to sum to 1
R = eye(2)*measPaggNoise; 
R(2,2) = 10^(-7);
Q = cov(err);
err_full = err; % keep full errors because we may truncate it 
Caug = [binModel.C{i}; ones(1,binModel.totalBinNumber)];
D = zeros (size(Caug,1), binModel.binNumber);
H = zeros(size(Caug,1),binModel.totalNumber);
plant = ss( binModel.A{i},[ binModel.B binModel.G],Caug,[D H],-1);
while (1)
    try
        Q = cov(err);
        lastwarn(''); % reset warning message
        [kalmf,L,P,M] = kalman(plant,Q,R); % only need the innovation matrix M
        % if there is no singularity warning we're good, else we will start
        % considering less and less error terms
        if isempty(lastwarn) % kalman worked with no errors or warnings
            break
        end                
    catch % error occured in kalman
        warning('Numerical error occurred in Kalman filter');
    end
    if length(err) > 100 % there are still error terms we can truncate
        err = err(2:end,:);
        err_terms_ignored = err_terms_ignored + 1;
    else % can't truncate error terms any more : just consider only the diagonal of Q to make sure it will be well-conditioned
        fprintf('Could not find subset of errors that do not lead to error when calling kalman\n');
        fprintf('Returning diagonal of the covariance matrix corresponding to the full error vector\n');
        Q = diag(diag( cov(err_full) ));
        return 
    end
end
%% check case when state feedback is available
R = eye(1+binModel.totalNumber)*binModel.measStateNoise; % component for state measurements
R(1,1) = measPaggNoise;  % component for power measurement
Caug = [binModel.C{i}; eye(binModel.totalNumber)]; % measurements are [Power; x]
D = zeros (size(Caug,1), binModel.binNumber);
H = zeros(size(Caug,1),binModel.totalNumber);
plant = ss( binModel.A{i},[ binModel.B binModel.G],Caug,[D H],-1);
Q = cov(err);
while (1)
    try
        Q = cov(err);
        lastwarn(''); % reset warning message
        [kalmf,L,P,M] = kalman(plant,Q,R); % only need the innovation matrix M
        % if there is no singularity warning we're good, else we will start
        % considering less and less error terms
        if isempty(lastwarn) % kalman worked with no errors or warnings
            break
        end                
    catch % error occured in kalman
        warning('Numerical error occurred in Kalman filter');
    end
    if length(err) > 100 % there are still error terms we can truncate
        err = err(2:end,:);
        err_terms_ignored = err_terms_ignored + 1;
    else % can't truncate error terms any more : just consider only the diagonal of Q to make sure it will be well-conditioned
        fprintf('Could not find subset of errors that do not lead to error when calling kalman\n');
        fprintf('Returning diagonal of the covariance matrix corresponding to the full error vector\n');
        Q = diag(diag( cov(err_full) ));
        return 
    end
end

end

