% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [theta_a_new, theta_m_new] = advanceTemps_2zone(tclParameters, theta_a, theta_m, m_vect)
%{
This function computes the air and mass temperatures of all TCLs for the
next timestep, given the corresponding values at the current timestep.
Input:
    tclParameters: struct
        Contains info regarding the TCLs, e.g. thermal parameters etc.
    theta_a: vector of floats
        Current air temperatures of all TCLs.
    theta_m: vector of floats
        Current mass temperatures of all TCLs.
    m_vect: vector of booleans
        Current on/off mode of all TCLs.
Output:
    theta_a_new:
        New air temperatures of all TCLs.
    theta_m_new:
        New mass temperatures of all TCLs.
%}

%% Load usefull variables from structs
% number of TCLs
numTCL = tclParameters.numAcs;
% number of TCLs that correspond to single and two-zone houses
num2zone = tclParameters.numHouses_2zone;
num1zone = tclParameters.numHouses_1zone;
% number of houses with an A/C
numHousesWithAc = num2zone + num1zone;
% indices of 1 and 2 zone houses that correspond to tclParameters
tcls_2zone_first = tclParameters.tcls_2zone_first;
tcls_2zone_second = tclParameters.tcls_2zone_second;
tcls_1zone = tclParameters.tcls_1zone;
% get the 3-d arrays with the A,B, and H matrices stacked
A_discrete_dynamics = tclParameters.A_discrete_dynamics;
B_discrete_dynamics = tclParameters.B_discrete_dynamics;
H_discrete_dynamics = tclParameters.H_discrete_dynamics;

%% Create distrubance vector
% disturbance_extended = [T_a_extended Q_i_extended Q_s_extended Q_m_extended Q_i_extended_other Q_s_extended_other Q_m_extended_other];
disturbance_extended = zeros(numHousesWithAc*4, 7);
% outdoor temperature - assume the same for both zones of the house
disturbance_extended(:,1) = reshape([repmat(tclParameters.T_a(tcls_2zone_first),1,4)' repmat(tclParameters.T_a(tcls_1zone),1,4)'],[],1);
% internal heatgains Qi, first zone
disturbance_extended(:,2) = reshape([repmat(tclParameters.Q_i(tcls_2zone_first),1,4)' repmat(tclParameters.Q_i(tcls_1zone),1,4)'],[],1);
% solar heatgains Qs, first zone
disturbance_extended(:,3) = reshape([repmat(tclParameters.Q_s(tcls_2zone_first),1,4)' repmat(tclParameters.Q_s(tcls_1zone),1,4)'],[],1);
% heatgains to the mass Qm, first zone
disturbance_extended(:,4) = reshape([repmat(tclParameters.Q_m(tcls_2zone_first),1,4)' repmat(tclParameters.Q_m(tcls_1zone),1,4)'],[],1);
% internal heatgains Qi, second zone
disturbance_extended(:,5) = reshape([repmat(tclParameters.Q_i(tcls_2zone_second),1,4)' repmat(tclParameters.Q_i(tcls_1zone),1,4)'],[],1);
% internal heatgains Qs, second zone
disturbance_extended(:,6) = reshape([repmat(tclParameters.Q_s(tcls_2zone_second),1,4)' repmat(tclParameters.Q_s(tcls_1zone),1,4)'],[],1);
% internal heatgains Qm, second zone
disturbance_extended(:,7) = reshape([repmat(tclParameters.Q_m(tcls_2zone_second),1,4)' repmat(tclParameters.Q_m(tcls_1zone),1,4)'],[],1);

%% Create state vector
% create the state matrices for element-wise multiplication with A
% x_n_extended = [airTemperature_extended massTemperature_extended airTemperature_other_zone_extended massTemperature_other_zone_extended];
x_n_extended = zeros(numHousesWithAc*4, 4);
% air temperature of the first zone
x_n_extended(:,1) = reshape([repmat(theta_a(tcls_2zone_first),1,4)' repmat(theta_a(tcls_1zone),1,4)'],[],1);
% mass temperature of the first zone
x_n_extended(:,2) = reshape([repmat(theta_m(tcls_2zone_first),1,4)' repmat(theta_m(tcls_1zone),1,4)'],[],1);
% air temperature of the corresponding second zone
x_n_extended(:,3) = reshape([repmat(theta_a(tcls_2zone_second),1,4)' repmat(zeros(num1zone,1),1,4)'],[],1);
% mass temperature of the corresponding second zone
x_n_extended(:,4) = reshape([repmat(theta_m(tcls_2zone_second),1,4)' repmat(zeros(num1zone,1),1,4)'],[],1);

%% Create heat gain from the A/C vector
% A/C heat gains, first column corresponds to the 1st zone and 2nd column
% to the 2nd zone. For single-zone the 2nd column is zero.
Q_h_extended = reshape( [repmat(tclParameters.Q_h(tcls_2zone_first),1,4)'...
                         repmat(tclParameters.Q_h(tcls_1zone),1,4)' ...
                         repmat(tclParameters.Q_h(tcls_2zone_second),1,4)'...
                         repmat(zeros(num1zone,1),1,4)'], [],2);

%% Create on/off mode vectors                   
% same with on/off modes
m_vect_extended = reshape( [repmat(m_vect(tcls_2zone_first),1,4)'...
                         repmat(m_vect(tcls_1zone),1,4)' ...
                         repmat(m_vect(tcls_2zone_second),1,4)'...
                         repmat(zeros(num1zone,1),1,4)'], [],2);   

%% Compute new states using element-wise calculations
% consider temperature delays
if tclParameters.temperatureDelayOption ~= 1
    % set a logical that flags whether the TCL is in delay (=1) or not (=0)
    delayed = tclParameters.timeUntilDelayEnds > 0;
    % if delayed=1 then m_n is reversed, since the dynamics are not
    % reversed yet. if delayed=0 then m_n remains as it is
    delayed_m_n = (1-delayed).*m_vect + delayed.*(1-m_vect);
    delayed_m_n_extended = reshape( [repmat(delayed_m_n(tcls_2zone_first),1,4)'...
                                     repmat(delayed_m_n(tcls_1zone),1,4)' ...
                                     repmat(delayed_m_n(tcls_2zone_second),1,4)'...
                                     repmat(zeros(num1zone,1),1,4)'], [],2);    
    % this is the state update using element-wise operations
    x_np1 = sum(A_discrete_dynamics .* x_n_extended,2) + sum(B_discrete_dynamics .* Q_h_extended .* delayed_m_n_extended, 2) + (H_discrete_dynamics .* disturbance_extended)*ones(7,1);
else
    x_np1 = sum(A_discrete_dynamics .* x_n_extended,2) + sum(B_discrete_dynamics .* Q_h_extended .* m_vect_extended, 2) + (H_discrete_dynamics .* disturbance_extended)*ones(7,1);
end

% for x_npl, each tcl has 4 corresponding entries, with the first being
% its own air temp, the second its mass temp and the remaining two
% corresponing to the other zone. For single-zone houses we don't care
% about the last 2 entries of each block.
theta_a_new = [x_np1(1:2:num2zone*4);x_np1(num2zone*4+1:4:end)];
theta_m_new = [x_np1(2:2:num2zone*4);x_np1(num2zone*4+2:4:end)];
% add extra noise in the TCL state updates
theta_a_new = theta_a_new + normrnd(tclParameters.noiseMean,tclParameters.noiseSd,numTCL,1);
theta_m_new = theta_m_new + normrnd(tclParameters.noiseMean,tclParameters.noiseSd,numTCL,1);

end

