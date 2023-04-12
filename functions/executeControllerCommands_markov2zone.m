% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [m_vect] = executeControllerCommands_markov2zone(controllerInformation, num_zones, timeUntilUnlocked, T_min, T_max, onOffSignal, theta_a, m_vect, logical_index_high, logical_index_low, locked, useLocalVoltage, tclVoltage)
%{
This function carries out the controller commands based on the Markov model for two-zone houses.
Each TCL first picks the correct probability to act upon, then draws a
random number and decides whether to switch or not.
Input:
    controllerInformation: struct
        Contains controller information such as the bin model, controller
        type etc.
    num_zones : int
        Total number of zones in the considered TCLs. 
    timeUntilUnlocked: vector
        Contains the time until lockout ends for each zone. Zones
        corresponding to the same compressor/house have identical times.
    T_min: vector of ints
        Lower edge of the deadband.
    T_max: vector of ints
        Upper edge of the deadband.
    onOffSignal: vector
        Probabilities corresponding to the subset of bins needed. E.g. for 
        the Markov controller, if we want to switch OFF, only probabilities
        corresponding to half of the bins will be given as input.
    theta_a: vector of floats
        Air temperature of each TCL at the current timestep.
    m_vect: vector of booleans
        ON/OFF mode of each TCL at the current timestep.
    logical_index_high: vector of booleans
        Indicator of whether each TCL has exceeded its upper deadband limit
    logical_index_low: vector of booleans
        Indicator of whether each TCL has exceeded its lower deadband limit
    useLocalVoltage: boolean
        1 if we want to take into avoid switching TCLs whose voltage is
        outside certain values
    tclVoltage: vector of floats
        Voltage at each TCL (240V level)
Output:
    m_vect: vector of booleans
        Updated ON/OFF mode for each TCL after taking into account the
        switching probabilities send by the controller.
%}

%% Load variables from structs
controller_type = controllerInformation.Type;
binNumber = controllerInformation.binModel.binNumber;
N1 = binNumber;
N2 = binNumber;
bin_1_edges = controllerInformation.binModel.bin_1_edges; 
bin_2_edges = controllerInformation.binModel.bin_2_edges; 
N_houses = num_zones/2;  % only 2 zone houses considered here, hence the number of tcls is half the number of zones

%% Draw a random number for each TCL
randomValueAtTcl = rand(N_houses,1);

%% Assign the correct probability to each TCL
% normalize first according to the deadband
theta_a_norm = (theta_a-T_min)./(T_max-T_min);
% assign each tcl to the corresponding bin depending on the initial temp and state
current_bins = map_to_bin_states(controller_type,[bin_1_edges;bin_2_edges], binNumber, m_vect, theta_a_norm, timeUntilUnlocked);
prev_aggr_state = current_bins;
% locked compressor
islocked = locked(1:2:end-1);
%Map control to their corresponding bins
u2_ON_val = onOffSignal(1:N1*N2);
u3_ON_val = onOffSignal(N1*N2+1:2*N1*N2);
u4_ON_val = onOffSignal(2*N1*N2+1:3*N1*N2);
u2_OFF_val = onOffSignal(3*N1*N2+1:4*N1*N2);
u3_OFF_val = onOffSignal(4*N1*N2+1:5*N1*N2);
u4_OFF_val = onOffSignal(5*N1*N2+1:6*N1*N2);

if any(u2_ON_val+u3_ON_val+u4_ON_val)
    dPowerIdx = 1; % will turn TCLs ON
else
    dPowerIdx = -1; % will turn TCLs OFF
end
%% Switch decision
%Construct calling based on external control signal and dead-band control
q_ctrl = zeros(num_zones,1);
q1_ctrl = zeros(N_houses,1);
q2_ctrl = zeros(N_houses,1);
q1_ctrl_temp = -ones(N_houses,1); %This is r in the notebook
q2_ctrl_temp = -ones(N_houses,1); %This is r in the notebook
q1_prev = m_vect(1:2:end-1);
q2_prev = m_vect(2:2:end);
%Not yet vectorized
if dPowerIdx == 1 %TCLs need to consume more power
    for j = 1:N_houses
        %Command due to external control
        if (1 <= prev_aggr_state(j)) && (prev_aggr_state(j) <= N1*N2)  %Check if the TCL is OFF
            if ismember(prev_aggr_state(j),controllerInformation.binModel.rq2_OFF_idx) %Switch to 1 OFF 2 ON
               if (randomValueAtTcl(j) <= u2_ON_val(prev_aggr_state(j))) && ((q1_prev(j) == 0) && (q2_prev(j) == 0)) %Switch if the random value is greater
                   q1_ctrl_temp(j) = 0;
                   q2_ctrl_temp(j) = 1;
               end
            elseif ismember(prev_aggr_state(j),controllerInformation.binModel.rq3_OFF_idx) %Switch to 1 ON 2 OFF
               if (randomValueAtTcl(j) <= u3_ON_val(prev_aggr_state(j))) && ((q1_prev(j) == 0) && (q2_prev(j) == 0)) %Switch if the random value is greater
                   q1_ctrl_temp(j) = 1;
                   q2_ctrl_temp(j) = 0;
               end
            elseif ismember(prev_aggr_state(j),controllerInformation.binModel.rq4_OFF_idx) %Switch to 1 ON 2 ON
               if (randomValueAtTcl(j) <= u4_ON_val(prev_aggr_state(j))) && ((q1_prev(j) == 0) && (q2_prev(j) == 0)) %Switch if the random value is greater
                   q1_ctrl_temp(j) = 1;
                   q2_ctrl_temp(j) = 1;
               end                    
            end
        end
        %Final command for Zone 1
        if (q1_ctrl_temp(j) == -1) && (q1_prev(j) == 0) && (islocked(j) == 0) && logical_index_high((j-1)*2+1) %(Ta1_prev(j) >= theta1_h_v(j))
            q1_ctrl(j) = 1;
        elseif (q1_ctrl_temp(j) == -1) && (q1_prev(j) == 1) && (islocked(j) == 0) && logical_index_low((j-1)*2+1) %(Ta1_prev(j) <= theta1_l_v(j))
            q1_ctrl(j) = 0;
        elseif (q1_ctrl_temp(j) == 1) && (q1_prev(j) == 0) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+1) && ~logical_index_low((j-1)*2+1) %(Ta1_prev(j) > theta1_l_v(j)) && (Ta1_prev(j) < theta1_h_v(j))
            q1_ctrl(j) = 1;
        elseif (q1_ctrl_temp(j) == 0) && (q1_prev(j) == 1) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+1) && ~logical_index_low((j-1)*2+1) %(Ta1_prev(j) > theta1_l_v(j)) && (Ta1_prev(j) < theta1_h_v(j))
            q1_ctrl(j) = 0;
        else
            q1_ctrl(j) = q1_prev(j);
        end                
         %Final command for Zone 2
        if (q2_ctrl_temp(j) == -1) && (q2_prev(j) == 0) && (islocked(j) == 0) && logical_index_high((j-1)*2+2) %(Ta2_prev(j) >= theta2_h_v(j))
            q2_ctrl(j) = 1;
        elseif (q2_ctrl_temp(j) == -1) && (q2_prev(j) == 1) && (islocked(j) == 0) && logical_index_low((j-1)*2+2) %(Ta2_prev(j) <= theta2_l_v(j))
            q2_ctrl(j) = 0;
        elseif (q2_ctrl_temp(j) == 1) && (q2_prev(j) == 0) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+2) && ~logical_index_low((j-1)*2+2) %(Ta2_prev(j) > theta2_l_v(j)) && (Ta2_prev(j) < theta2_h_v(j))
            q2_ctrl(j) = 1;
        elseif (q2_ctrl_temp(j) == 0) && (q2_prev(j) == 1) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+2) && ~logical_index_low((j-1)*2+2) %(Ta2_prev(j) > theta2_l_v(j)) && (Ta2_prev(j) < theta2_h_v(j))
            q2_ctrl(j) = 0;
        else
            q2_ctrl(j) = q2_prev(j);
        end 
    end
else %TCLs need to consume less power
    for j = 1:N_houses
        %Command due to external control
        if (N1*N2+1 <= prev_aggr_state(j)) && (prev_aggr_state(j) <= 2*N1*N2)  %Check if the TCL is 1 OFF and 2 ON
            if ismember(prev_aggr_state(j)-N1*N2,controllerInformation.binModel.rq1_ON_FROM_Q2_idx) % check if the current temperature bin for (0,1) is controllable
               if (randomValueAtTcl(j) <= u2_OFF_val(prev_aggr_state(j)-N1*N2)) && ((q1_prev(j) == 1) || (q2_prev(j) == 1)) %Switch if the random value is greater
                   q1_ctrl_temp(j) = 0;
                   q2_ctrl_temp(j) = 0;
               end
            end
        elseif (2*N1*N2+1 <= prev_aggr_state(j)) && (prev_aggr_state(j) <= 3*N1*N2)  %Check if the TCL is 1 ON and 2 OFF
            if ismember(prev_aggr_state(j)-2*N1*N2,controllerInformation.binModel.rq1_ON_FROM_Q3_idx) % check if the current temperature bin for (1,0) is controllable
               if (randomValueAtTcl(j) <= u3_OFF_val(prev_aggr_state(j)-2*N1*N2)) && ((q1_prev(j) == 1) || (q2_prev(j) == 1)) %Switch if the random value is greater
                   q1_ctrl_temp(j) = 0;
                   q2_ctrl_temp(j) = 0;
               end
            end
        elseif (3*N1*N2+1 <= prev_aggr_state(j)) && (prev_aggr_state(j) <= 4*N1*N2)  %Check if the TCL is 1 ON and 2 ON
            if ismember(prev_aggr_state(j)-3*N1*N2,controllerInformation.binModel.rq1_ON_FROM_Q4_idx) % check if the current temperature bin for (1,1) is controllable
               if (randomValueAtTcl(j) <= u4_OFF_val(prev_aggr_state(j)-3*N1*N2)) && ((q1_prev(j) == 1) || (q2_prev(j) == 1)) %Switch if the random value is greater
                   q1_ctrl_temp(j) = 0;
                   q2_ctrl_temp(j) = 0;
               end
            end
        end
        %Final command for Zone 1
        if (q1_ctrl_temp(j) == -1) && (q1_prev(j) == 0) && (islocked(j) == 0) && logical_index_high((j-1)*2+1) %(Ta1_prev(j) >= theta1_h_v(j))
            q1_ctrl(j) = 1;
        elseif (q1_ctrl_temp(j) == -1) && (q1_prev(j) == 1) && (islocked(j) == 0) && logical_index_low((j-1)*2+1) %(Ta1_prev(j) <= theta1_l_v(j))
            q1_ctrl(j) = 0;
        elseif (q1_ctrl_temp(j) == 1) && (q1_prev(j) == 0) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+1) && ~logical_index_low((j-1)*2+1) %(Ta1_prev(j) > theta1_l_v(j)) && (Ta1_prev(j) < theta1_h_v(j))
            q1_ctrl(j) = 1;
        elseif (q1_ctrl_temp(j) == 0) && (q1_prev(j) == 1) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+1) && ~logical_index_low((j-1)*2+1) %(Ta1_prev(j) > theta1_l_v(j)) && (Ta1_prev(j) < theta1_h_v(j))
            q1_ctrl(j) = 0;
        else
            q1_ctrl(j) = q1_prev(j);
        end                
         %Final command for Zone 2
        if (q2_ctrl_temp(j) == -1) && (q2_prev(j) == 0) && (islocked(j) == 0) && logical_index_high((j-1)*2+2) %(Ta2_prev(j) >= theta2_h_v(j))
            q2_ctrl(j) = 1;
        elseif (q2_ctrl_temp(j) == -1) && (q2_prev(j) == 1) && (islocked(j) == 0) && logical_index_low((j-1)*2+2) %(Ta2_prev(j) <= theta2_l_v(j))
            q2_ctrl(j) = 0;
        elseif (q2_ctrl_temp(j) == 1) && (q2_prev(j) == 0) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+2) && ~logical_index_low((j-1)*2+2) %(Ta2_prev(j) > theta2_l_v(j)) && (Ta2_prev(j) < theta2_h_v(j))
            q2_ctrl(j) = 1;
        elseif (q2_ctrl_temp(j) == 0) && (q2_prev(j) == 1) && (islocked(j) == 0) && ~logical_index_high((j-1)*2+2) && ~logical_index_low((j-1)*2+2) %(Ta2_prev(j) > theta2_l_v(j)) && (Ta2_prev(j) < theta2_h_v(j))
            q2_ctrl(j) = 0;
        else
            q2_ctrl(j) = q2_prev(j);
        end  
    end
end
q_ctrl(1:2:end-1) = q1_ctrl;
q_ctrl(2:2:end) = q2_ctrl;
m_vect = q_ctrl;
end