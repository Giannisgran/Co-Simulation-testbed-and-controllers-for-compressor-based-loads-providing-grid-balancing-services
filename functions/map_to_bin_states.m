% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [bin_state] =  map_to_bin_states(controller_type, binEdges, binNumber, m, theta_a_norm, timeRemaining)
%{
This function maps each given TCL to the proper bin model state.
Input:
    controller_type: str
        String containing the name of the controller, e.g. Markov controller.
    binEdges: vector
        Contains the edges of the bins.
    binNumber: int
        Contains the number of bins.
    m: vector
        Vector representing the ON/OFF mode of each TCL.
    theta_a_norm: vector
        Vector with the normalized air temperatures of all TCLs.
    timeRemaining: vector
        Vector containing the remaining time until each TCL gets unlocked
        or stops being delayed. Interpretation depends on the controller
        type.
Output:
    current_bins: vector
        Vector that contains the state of each TCL in the bin model.
%}

% check controller type and map to the bin model states accordingly
if strcmp(controller_type, 'Markov controller')
    bin_state = (1-2*m).*discretize(theta_a_norm,binEdges)+m*(2*binNumber+1);
elseif strcmp(controller_type, 'Markov controller with lockouts')
    % first 2*binNumber bins are for unlocked TCL and the remaining 2*binNumber are for locked ones
    locked = timeRemaining > 0;
    bin_state = (1-2*m).*discretize(theta_a_norm,binEdges)+m*(2*binNumber+1) + locked*(2*binNumber);
elseif strcmp(controller_type, 'Markov controller with delays')
    % first 2*binNumber bins are for unlocked TCL and the remaining 2*binNumber are for locked ones
    delayed = timeRemaining > 0;
    bin_state = (1-2*m).*discretize(theta_a_norm,binEdges)+m*(2*binNumber+1) + delayed*(2*binNumber);
elseif strcmp(controller_type, 'Markov controller 2 zone') %This does not consider lockout at all 
    bin_1_edges = binEdges(1,:);  % bin edges for zone 1
    bin_2_edges = binEdges(2,:);  % bin edges for zone 2
    
    % q is the call for cool or on/off mode for each zone - each vector has
    % equal number of elements as the number of 2zone houses
    q_1 = m(1:2:end-1,:); %Zone 1
    q_2 = m(2:2:end,:); %Zone 2
    % get the normalized temperatures for each zone - each vector has as
    % many elements as the number of 2zone houses
    Ta_normal_1 = theta_a_norm(1:2:end-1,:); %Zone 1
    Ta_normal_2 = theta_a_norm(2:2:end,:); %Zone 2

    % group air temperature in both zones based on ON or OFF state - zeroes out the entries that correspond to the other mode of what each vector represents
    Ta_1_on_normal = q_1.*Ta_normal_1;
    Ta_1_off_normal = (1-q_1).*Ta_normal_1;
    Ta_2_on_normal = q_2.*Ta_normal_2;
    Ta_2_off_normal = (1-q_2).*Ta_normal_2;

    % bin indexing based on ON or OFF bin matrix - zeroes out the entries that correspond to the other mode of what each vector represents
    bin_idx_Ta_1_on = q_1.*discretize(Ta_1_on_normal,bin_1_edges);
    bin_idx_Ta_1_off = (1-q_1).*discretize(Ta_1_off_normal,bin_1_edges);
    bin_idx_Ta_2_on = q_2.*discretize(Ta_2_on_normal,bin_2_edges);
    bin_idx_Ta_2_off = (1-q_2).*discretize(Ta_2_off_normal,bin_2_edges);

    % assign each normalized TCL's state to the aggregate state
    aggr_state_00 = (1-q_1).*(1-q_2).*((bin_idx_Ta_1_off - 1)*binNumber + bin_idx_Ta_2_off); %Zone 1 OFF Zone 2 OFF
    aggr_state_01 = (1-q_1).*(q_2).*(binNumber*binNumber + (bin_idx_Ta_1_off - 1)*binNumber + bin_idx_Ta_2_on); %Zone 1 OFF Zone 2 ON
    aggr_state_10 = (q_1).*(1-q_2).*(2*binNumber*binNumber + (bin_idx_Ta_1_on - 1)*binNumber + bin_idx_Ta_2_off); %Zone 1 ON Zone 2 OFF
    aggr_state_11 = (q_1).*(q_2).*(3*binNumber*binNumber + (bin_idx_Ta_1_on - 1)*binNumber + bin_idx_Ta_2_on); %Zone 1 ON Zone 2 ON

    % construct the total aggregate (bin) state
    bin_state = aggr_state_00 + aggr_state_01 + aggr_state_10 + aggr_state_11;
    
elseif strcmp(controller_type, 'Markov controller 2 zone with lockouts') %This considers both ON and OFF compressor lockout
    bin_1_edges = binEdges(1,:);  % bin edges for zone 1
    bin_2_edges = binEdges(2,:);  % bin edges for zone 2
    q_1 = m(1:2:end-1,:); %Zone 1
    q_2 = m(2:2:end,:); %Zone 2
    Ta_normal_1 = theta_a_norm(1:2:end-1,:); %Zone 1
    Ta_normal_2 = theta_a_norm(2:2:end,:); %Zone 2

    %AC lockout status
    q_lock_prev = timeRemaining(1:2:end-1,:);
    q_lock_prev(q_lock_prev ~= 0) = 1;

    % group air temperature in both zones based on ON or OFF state
    Ta_1_on_normal = (1-q_lock_prev).*q_1.*Ta_normal_1;
    Ta_1_off_normal = (1-q_lock_prev).*(1-q_1).*Ta_normal_1;
    Ta_2_on_normal = (1-q_lock_prev).*q_2.*Ta_normal_2;
    Ta_2_off_normal = (1-q_lock_prev).*(1-q_2).*Ta_normal_2;
    Ta_1_off_lock_normal = q_lock_prev.*(1-q_1).*Ta_normal_1;
    Ta_2_off_lock_normal = q_lock_prev.*(1-q_2).*Ta_normal_2;
    Ta_1_on_lock_normal = q_lock_prev.*q_1.*Ta_normal_1;
    Ta_2_on_lock_normal = q_lock_prev.*q_2.*Ta_normal_2;

    % bin indexing based on ON or OFF bin matrix
    % for MTM considering lockout
    bin_idx_Ta_1_on = (1-q_lock_prev).*q_1.*discretize(Ta_1_on_normal,bin_1_edges);
    bin_idx_Ta_1_off = (1-q_lock_prev).*(1-q_1).*discretize(Ta_1_off_normal,bin_1_edges);
    bin_idx_Ta_2_on = (1-q_lock_prev).*q_2.*discretize(Ta_2_on_normal,bin_2_edges);
    bin_idx_Ta_2_off = (1-q_lock_prev).*(1-q_2).*discretize(Ta_2_off_normal,bin_2_edges);
    bin_idx_Ta_1_off_lock = q_lock_prev.*(1-q_1).*discretize(Ta_1_off_lock_normal,bin_1_edges);
    bin_idx_Ta_2_off_lock = q_lock_prev.*(1-q_2).*discretize(Ta_2_off_lock_normal,bin_2_edges);
    bin_idx_Ta_1_on_lock = q_lock_prev.*q_1.*discretize(Ta_1_on_lock_normal,bin_1_edges);
    bin_idx_Ta_2_on_lock = q_lock_prev.*q_2.*discretize(Ta_2_on_lock_normal,bin_2_edges);

    % assign each normalized TCL's state to the aggregate state
    % for MTM considering lockout
    aggr_state_00 = (1-q_lock_prev).*(1-q_1).*(1-q_2).*((bin_idx_Ta_1_off - 1)*binNumber + bin_idx_Ta_2_off); %Zone 1 OFF Zone 2 OFF
    aggr_state_01 = (1-q_lock_prev).*(1-q_1).*(q_2).*(binNumber*binNumber + (bin_idx_Ta_1_off - 1)*binNumber + bin_idx_Ta_2_on); %Zone 1 OFF Zone 2 ON
    aggr_state_10 = (1-q_lock_prev).*(q_1).*(1-q_2).*(2*binNumber*binNumber + (bin_idx_Ta_1_on - 1)*binNumber + bin_idx_Ta_2_off); %Zone 1 ON Zone 2 OFF
    aggr_state_11 = (1-q_lock_prev).*(q_1).*(q_2).*(3*binNumber*binNumber + (bin_idx_Ta_1_on - 1)*binNumber + bin_idx_Ta_2_on); %Zone 1 ON Zone 2 ON
    aggr_state_00_lock = q_lock_prev.*(1-q_1).*(1-q_2).*(4*binNumber*binNumber + (bin_idx_Ta_1_off_lock - 1)*binNumber + bin_idx_Ta_2_off_lock); %Zone 1 OFF Zone 2 OFF locked
    aggr_state_01_lock = q_lock_prev.*(1-q_1).*(q_2).*(5*binNumber*binNumber + (bin_idx_Ta_1_off_lock - 1)*binNumber + bin_idx_Ta_2_on_lock); %Zone 1 OFF Zone 2 ON locked
    aggr_state_10_lock = q_lock_prev.*(q_1).*(1-q_2).*(6*binNumber*binNumber + (bin_idx_Ta_1_on_lock - 1)*binNumber + bin_idx_Ta_2_off_lock); %Zone 1 ON Zone 2 OFF locked
    aggr_state_11_lock = q_lock_prev.*(q_1).*(q_2).*(7*binNumber*binNumber + (bin_idx_Ta_1_on_lock - 1)*binNumber + bin_idx_Ta_2_on_lock); %Zone 1 ON Zone 2 ON locked

    % construct the total aggregate state
    % for MTM considering lockout
    bin_state = aggr_state_00 + aggr_state_01 + aggr_state_10 + aggr_state_11 ...
        + aggr_state_00_lock + aggr_state_01_lock + aggr_state_10_lock + aggr_state_11_lock;
else
    error('Invalid controller.')
end


end

