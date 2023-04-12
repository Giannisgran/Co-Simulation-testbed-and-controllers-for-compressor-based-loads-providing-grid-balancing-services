% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [transitionMatrix, transitionMatrix_unscaled] = find_markov_matrix(bin_states, totalBinNumber)
%{
This function takes as input a matrix of the bin states of each TCL for
each timestep and outputs the Markov transition matrix.
Input:
    bin_states: matrix
        Matrix with the bin number of each TCL for each timestep. Rows
        correspond to TCLs and columns to timesteps
    totalBinNumber: int
        Number of bins in the bin model that corresponds to either on or off mode.
Output:
    transitionMatrix: matrix
        Markov matrix with the transition probabilities.
    transitionMatrix_unscaled: matrix
        Matrix with the transition counts.
%}

% initialize transition matrix, size is bouble the number of bins
% because we want bins for ON and OFF states, so the first binNumber
% bins correspond to OFF states and the rest to ON states
transitionMatrix_unscaled = zeros(totalBinNumber);
% get the previous state at each timestep and vectorize them
previous_states = bin_states(:,1:end-1)'; previous_states = previous_states(:);
% get the current state at each timestep and vectorize them
current_states = bin_states(:,2:end)'; current_states = current_states(:);
% register the transitions
for j = 1 : length(current_states)
    transitionMatrix_unscaled(current_states(j),previous_states(j)) = transitionMatrix_unscaled(current_states(j),previous_states(j)) + 1;
end
% normalize transition matrix
col_sum = sum(transitionMatrix_unscaled,1);
% repeat column vector with sums as many times as the rows of transitionMatrix
col_sum_mat = repmat(col_sum,size(transitionMatrix_unscaled,1),1);
% normalize using element-wise devision
transitionMatrix = transitionMatrix_unscaled./col_sum_mat; 
end

