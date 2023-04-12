% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [m_vect] = executeControllerCommands_markov(controllerInformation, numAcs, timeUntilUnlocked, timeUntilDelayEnds, T_min, T_max, onOffSignal, theta_a, m_vect, logical_index_high, logical_index_low, locked, useLocalVoltage, tclVoltage)
%{
This function carries out the controller commands.
Each TCL first picks the correct probability to act upon, then draws a
random number and decides whether to switch or not.
Input:
    controllerInformation: struct
        Contains controller information such as the bin model, controller
        type etc.
    numAcs: int
        Number of A/Cs in the current aggregation.
    timeUntilUnlocked: vector of ints
        Remaining timesteps until lockout ends for each TCL.
    timeUntilDelayEnds: vector of ints
        Remaining timesteps until temperature delay ends for each TCL.
    Tmin: vector of floats
        Lower edge of the deadband for each TCL.
    Tmax: vector of floats
        Upper edge of the deadband for each TCL.
    onOffSignal: vector
        Probabilities corresponding to the subset of bins needed. E.g. for 
        the Markov controller, if we want to switch OFF, only probabilities
        corresponding to half of the bins will be given as input.
    theta_a: vector of floats
        Air temperature of each TCL at the current timestep.
    m_vect: vector of booleans
        ON/OFF mode of each TCL at the current timestep.
    logical_index_low: vector of booleans
        Indicator of whether each TCL has exceeded its lower deadband limit
    useLocalVoltage: boolean
        1 if we want to avoid switching TCLs whose voltage is outside 
        certain values.
    tclVoltage: vector of floats
        Voltage at each TCL (240V level).
Output:
    m_vect: vector of booleans
        Updated ON/OFF mode for each TCL after taking into account the
        switching probabilities send by the controller.
%}

%% Load variables from structs
binsON = controllerInformation.binModel.binsON;
controller_type = controllerInformation.Type;
binEdges = controllerInformation.binModel.binEdges;
binNumber = controllerInformation.binModel.binNumber;
%% Draw a random number for each TCL
randomValueAtTcl = rand(numAcs,1);
%% Assign the correct probability to each TCL
% normalize first according to the deadband
theta_a_norm = (theta_a-T_min)./(T_max-T_min);
% assign each tcl to the corresponding bin depending on the initial temp 
% and state. Also, create the vector of relative probabilities for all 
% states, even those not available for the controller at the current timestep
if strcmp(controller_type, 'Markov controller')
    current_bins = map_to_bin_states(controller_type, binEdges, binNumber, m_vect, theta_a_norm, timeUntilUnlocked);
    u_rel_full = [any(onOffSignal > 0)*onOffSignal;any(onOffSignal < 0)*onOffSignal];
elseif strcmp(controller_type, 'Markov controller with lockouts')
    current_bins = map_to_bin_states(controller_type, binEdges, binNumber, m_vect, theta_a_norm, timeUntilUnlocked);
    u_rel_full = [onOffSignal;zeros(2*binNumber, 1)];
    % make the probabilities meant to switch OFF negative 
    u_rel_full(binsON) = -u_rel_full(binsON);
elseif strcmp(controller_type, 'Markov controller with delays')
    current_bins = map_to_bin_states(controller_type, binEdges, binNumber, m_vect, theta_a_norm, timeUntilDelayEnds);
    u_rel_full = onOffSignal;
    % make the probabilities meant to switch OFF negative 
    u_rel_full(binsON) = -u_rel_full(binsON);
end
% assign the right probabilities to TCLs based on their normalized temp
onOffProbability = u_rel_full(current_bins);

%% Switch decision
if useLocalVoltage == 0 % voltage not take into account
    % if not locked and within the deadband then switch if the drawn random
    % number is less than the corresponding switching probability
    m_vect(randomValueAtTcl < onOffProbability & 0 < onOffProbability ...
               & logical_index_high==0 & logical_index_low==0 & locked==0) = 1;
    % same as above but using negative values for off switching
    m_vect(randomValueAtTcl < abs(onOffProbability) & 0 > onOffProbability...
               & logical_index_high==0 & logical_index_low==0 & locked==0) = 0; 
else
    % load voltage limits
    vPlus = controllerInformation.upperVoltage;
    vMinus = controllerInformation.lowerVoltage;
    % find TCLs whose voltage is inside the voltage limits
    vFlag = tclVoltage <= vPlus & tclVoltage >= vMinus;
    % switching logic that requires the same as above, except if
    % tcls are over or under voltage they do not respond
    m_vect(randomValueAtTcl < onOffProbability & 0 < onOffProbability ...
               & logical_index_high==0 & logical_index_low==0 & locked==0 & vFlag) = 1;
    % same as above but using negative values for off switching
    m_vect(randomValueAtTcl < abs(onOffProbability) & 0 > onOffProbability...
               & logical_index_high==0 & logical_index_low==0 & locked==0 & vFlag) = 0; 
end
    
end

