% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [transitionMatrix] = compute_theoretical_transition_matrix_delays(transitionMatrix, tclParameters, generalParameters, To, binModel, controller_type)
%{
This function fills the zero columns of the computed A matrix with the
theoretical probabilities, assuming uniformly distributed TCLs over the
bins.
Input: 
    transitionMatrix: matrix
        Matrix with transition probabilities found from the identification
        process on the uncontrolled system.
    tclParameters: struct
        Struct regarding the TCL population.
    generalParameters: struct
        Includes parameters for the simulation.
    To: float
        Constant outdoor temperature for which we are computing the A
        matrix
    binModel: struct
        Current bin model.
    controller_type: string
        Type of the controller that is going to use the bin model.
Output: 
    transitionMatrix: matrix
        Matrix with transition probabilities, fully populated.
%}

numTcl = length(tclParameters.T_a);  % total number of TCLs in the population
sim_timestep = generalParameters.timeStepInSec;  % simulation timestep
binNumber = binModel.binNumber;
zero_cols = any(isnan(transitionMatrix)); % cols for which the sum is zero, i.e. we don't have transitions
zero_cols = find(zero_cols==1);  % keep the indices of the columns that have NaNs
%% Create the mappings between states
% map each state with the corresponding state of the opposite delay and compressor state
state_mapping_delay_comp = [4*binNumber:-1:2*binNumber+1 2*binNumber:-1:1];
% map each state the the corresponding state of the opposite delay 
state_mapping_delay = [2*binNumber+1:4*binNumber 1:2*binNumber];
% map each state with the corresponding state of the next temperature bin
state_mapping_next = [[1:2*binNumber]+1 [2*binNumber+1:4*binNumber]-1]; 
state_mapping_next([binNumber 2*binNumber 2*binNumber+1 3*binNumber+1]) = NaN;  % can't move to the next temperature bin for these since they are at the edges
%% Find the probability of transitioning from delayed to undelayed bins
% Find the delay values corresponding to the current outdoor temperature
delays_on = zeros(numTcl, 1); delays_off = zeros(numTcl, 1);  % preallocate
for i = 1 : numTcl
    current_delay_model = tclParameters.tempDelayObjects(tclParameters.tempDelayDistribution(i));  % current delay model
    current_delay_model = current_delay_model{1};  % unzip the 1x1 cell array
    d_on = current_delay_model{1}; % load object for ON delay
    d_off = current_delay_model{2}; % load object for OFF delay
    delays_on(i) = d_on.delayFit(To);  % find the ON delay value for the given outdoor temperature
    delays_off(i) = d_off.delayFit(To);  % find the OFF delay value for the given outdoor temperature
end
% Probabilities of transitioning from delayed to undelayed
% this assumes that all TCLs are just as likely to be switched, i.e., to appear in the population of delayed TCLs
p_change_on = mean(sim_timestep./delays_on);
p_change_off = mean(sim_timestep./delays_off);
% collect the probabilities of changing delay mode, 
% for the undelayed states there is zero probability of changing delay mode
p_change_vec = [zeros(1, 2*binNumber) p_change_off*ones(1, binNumber) p_change_on*ones(1, binNumber)];
%% Fill the missing probabilities for the delayed/locked states
for i = 1 : length(zero_cols) % loop over the indices of the zero columns
    col = zero_cols(i);  % current zero column index
    p_change = p_change_vec(col);  % probability of becoming undelayed, will be zero if current state corresponds to undelayed state
    % find the probability of moving temperature bin
    if ~isnan(state_mapping_next(col))  % get the value from the opposite delay mode
%         p_temp = transitionMatrix(state_mapping_next(state_mapping_delay(col)), state_mapping_delay(col)) + transitionMatrix(state_mapping_next(state_mapping_delay(state_mapping_delay_comp(col))), state_mapping_delay_comp(col));  
        p_temp = transitionMatrix(state_mapping_next(state_mapping_delay_comp(col)), state_mapping_delay_comp(col));
    else  % at edge state, so there is no next temperature bin
        p_temp = 0;
    end
    % fill out the probabilities
    transitionMatrix(:, col) = zeros(size(transitionMatrix, 1), 1);
    transitionMatrix(col, col) = (1-p_temp)*(1-p_change); % don't change temperature bin and don't change delay state
    transitionMatrix(state_mapping_delay(col), col) = (1-p_temp)*p_change; % don't change temperature bin but change delay state
    if ~isnan(state_mapping_next(col))  % make sure we are not at an edge bin
        transitionMatrix(state_mapping_next(col), col) = p_temp*(1-p_change); % change temperature bin and don't change delay state
    end
    if ~isnan(state_mapping_delay(state_mapping_next(col)))  % make sure we are not at an edge bin
        transitionMatrix(state_mapping_delay(state_mapping_next(col)), col) = p_temp*p_change; % change temperature bin and change delay state
    end
end

end