% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [transitionMatrix] = compute_theoretical_transition_matrix(transitionMatrix, tclParameters, generalParameters, To, binModel, controller_type)
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
%% Find the probability of changing from dalayed/locked to undelayed/unlocked
if strcmp(controller_type, 'Markov controller with lockouts')
    p_change_on = mean(sim_timestep./tclParameters.lockoutTimeOnInTimeSteps);
    p_change_off = mean(sim_timestep./tclParameters.lockoutTimeOffInTimeSteps);
elseif strcmp(controller_type, 'Markov controller with delays')
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
else
    error(['Controller not considered.'])
end
%% Fill the missing probabilities for the delayed/locked states
p_temp = zeros(length(zero_cols), 1);  % initialize, will keep the probabilities here
for i = 1 : length(zero_cols) % loop over the indices of the zero columns
    col = zero_cols(i);  % current zero column index
    if ismember(col, binModel.binsOFF) % check if column corresponds to OFF-delayed/locked state
        temperature_bin = col - 2*binNumber;
        if temperature_bin == 1 % at the lower edge of the deadband
            p_temp(i) = 0;  % the TCL cannot move beyond that. It will stay there in terms of temperature
        else
            if strcmp(controller_type, 'Markov controller with delays')
                pos = 2*binNumber-temperature_bin+1;  % corresponding position for the opposite compressor mode and undelayed state
            elseif  strcmp(controller_type, 'Markov controller with lockouts')
                pos = temperature_bin;  % corresponding position for the same compressor mode and unlocked state
            end
            p_temp(i) = transitionMatrix(pos+1, pos);  % probability of moving to the next temperature bin
        end
        % now we will find the desired column
        if strcmp(controller_type, 'Markov controller with delays')
            transitionMatrix(:, col) = zeros(size(transitionMatrix, 1), 1);
            transitionMatrix(col, col) = (1-p_temp(i))*(1-p_change_off); % don't change temperature bin and don't become undelayed/unlocked
            transitionMatrix(col-1, col) = p_temp(i)*(1-p_change_off); % change temperature bin and don't become undelayed/unlocked
            transitionMatrix(col-2*binNumber, col) = (1-p_temp(i))*p_change_off; % don't change temperature bin but become undelayed/unlocked
            if (col ~= 2*binNumber+1) && (col ~= 3*binNumber+1)  % make sure this is not the first temperature bin, because it cannot change temperature bin from there
                transitionMatrix(col-2*binNumber-1, col) = p_temp(i)*p_change_off; % change temperature bin and become undelayed/unlocked
            end
        elseif strcmp(controller_type, 'Markov controller with lockouts')
            transitionMatrix(:, col) = zeros(size(transitionMatrix, 1), 1);
            transitionMatrix(col, col) = (1-p_temp(i))*(1-p_change_off); % don't change temperature bin and don't become undelayed/unlocked
            transitionMatrix(col-2*binNumber, col) = (1-p_temp(i))*p_change_off; % don't change temperature bin but become undelayed/unlocked
            if (col ~= 3*binNumber) && (col ~= 4*binNumber)  % make sure this is not the last temperature bin, because it cannot change temperature bin from there
                transitionMatrix(col+1, col) = p_temp(i)*(1-p_change_off); % change temperature bin and don't become undelayed/unlocked
                transitionMatrix(col-2*binNumber+1, col) = p_temp(i)*p_change_off; % change temperature bin and become undelayed/unlocked
            end
        end
    else % column corresponds to ON-delayed/locked state
        temperature_bin = 4*binNumber - col+1;
        if temperature_bin == binNumber % at the upper edge of the deadband
            p_temp(i) = 0;  % the TCL cannot move beyond that. It will stay there in terms of temperature
        else
            if strcmp(controller_type, 'Markov controller with delays')
                pos = temperature_bin;  % corresponding position for the opposite compressor mode and undelayed state
            elseif  strcmp(controller_type, 'Markov controller with lockouts')
                pos = col - 2*binNumber;  % corresponding position for the same compressor mode and unlocked state
            end
            p_temp(i) = transitionMatrix(pos+1, pos); % probability of moving to the next temperature bin
        end
        % now we will find the desired column
        if strcmp(controller_type, 'Markov controller with delays')
            transitionMatrix(:, col) = zeros(size(transitionMatrix, 1), 1);
            transitionMatrix(col, col) = (1-p_temp(i))*(1-p_change_on); % don't change temperature bin and don't become undelayed/unlocked
            transitionMatrix(col-1, col) = p_temp(i)*(1-p_change_on); % change temperature bin and don't become undelayed/unlocked
            transitionMatrix(col-2*binNumber, col) = (1-p_temp(i))*p_change_on; % don't change temperature bin but become undelayed/unlocked
            if (col ~= 2*binNumber+1) && (col ~= 3*binNumber+1)  % make sure this is not the first temperature bin, because it cannot change temperature bin from there
                transitionMatrix(col-2*binNumber-1, col) = p_temp(i)*p_change_on; % change temperature bin and become undelayed/unlocked
            end
        elseif strcmp(controller_type, 'Markov controller with lockouts')
            transitionMatrix(:, col) = zeros(size(transitionMatrix, 1), 1);
            transitionMatrix(col, col) = (1-p_temp(i))*(1-p_change_on); % don't change temperature bin and don't become undelayed/unlocked
            transitionMatrix(col-2*binNumber, col) = (1-p_temp(i))*p_change_on; % don't change temperature bin but become undelayed/unlocked
            if (col ~= 3*binNumber) && (col ~= 4*binNumber)  % make sure this is not the last temperature bin, because it cannot change temperature bin from there
                transitionMatrix(col+1, col) = p_temp(i)*(1-p_change_on); % change temperature bin and don't become undelayed/unlocked               
                transitionMatrix(col-2*binNumber+1, col) = p_temp(i)*p_change_on; % change temperature bin and become undelayed/unlocked
            end 
        end
    end  
end


end