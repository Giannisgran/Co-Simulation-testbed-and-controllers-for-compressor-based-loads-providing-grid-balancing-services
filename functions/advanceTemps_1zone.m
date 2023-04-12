% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [theta_a_new, theta_m_new] = advanceTemps_1zone(tclParameters, theta_a, theta_m, m_vect)
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

%% Load useful variables from structs
% total number of TCLs
numTCL = length(tclParameters.U_a);
% get the 3-d arrays with the A,B, and H matrices stacked
A_discrete_dynamics = tclParameters.A_discrete_dynamics;
B_discrete_dynamics = tclParameters.B_discrete_dynamics;
H_discrete_dynamics = tclParameters.H_discrete_dynamics;

%% Advance air and mass temperatures
% disturbance - preallocate to speed up computation
% disturbance_extended = [T_a_extended Q_i_extended Q_s_extended Q_m_extended];
disturbance_extended = zeros(2*numTCL, 4);
% outdoor temperature - assume the same for both zones of the house
disturbance_extended(:,1) = reshape(repmat(tclParameters.T_a,1,2)',[],1);
% internal heatgains Qi, first zone
disturbance_extended(:,2) = reshape(repmat(tclParameters.Q_i,1,2)',[],1);
% solar heatgains Qs, first zone
disturbance_extended(:,3) = reshape(repmat(tclParameters.Q_s,1,2)',[],1);
% heatgains to the mass Qm, first zone
disturbance_extended(:,4) = reshape(repmat(tclParameters.Q_m,1,2)',[],1);

% create the state matrices for element-wise multiplication with A
% x_n_extended = [airTemperature_extended massTemperature_extended];
x_n_extended = zeros(2*numTCL, 2);
% air temperature of the first zone
x_n_extended(:,1) = reshape(repmat(theta_a,1,2)',[],1);
% mass temperature of the first zone
x_n_extended(:,2) = reshape(repmat(theta_m,1,2)',[],1);

% same with A/C heat gains
Q_h_extended = reshape(repmat(tclParameters.Q_h,1,2)',[],1);
                     
% same with on/off modes
m_vect_extended = reshape(repmat(m_vect,1,2)',[],1);   

% consider temperature delays
if tclParameters.temperatureDelayOption ~= 1
    % set a logical that flags whether the TCL is in delay (=1) or not (=0)
    delayed = tclParameters.timeUntilDelayEnds > 0;
    % if delayed=1 then m_n is reversed, since the dynamics are not
    % reversed yet. if delayed=0 then m_n remains as it is
    delayed_m_n = (1-delayed).*m_vect + delayed.*(1-m_vect);
    delayed_m_n_extended = reshape(repmat(delayed_m_n,1,2)',[],1);    
    % this is the state update using element-wise operations
    x_np1 = sum(A_discrete_dynamics .* x_n_extended,2) + (B_discrete_dynamics .* Q_h_extended .* delayed_m_n_extended) + (H_discrete_dynamics .* disturbance_extended)*ones(4,1);
else
    x_np1 = sum(A_discrete_dynamics .* x_n_extended,2) + sum(B_discrete_dynamics .* Q_h_extended .* m_vect_extended, 2) + (H_discrete_dynamics .* disturbance_extended)*ones(4,1);
end

% for x_npl, each tcl has 4 corresponding entries, with the first being
% its own air temp, the second its mass temp and the remaining two
% corresponing to the other zone. For single-zone houses we don't care
% about the last 2 entries of each block.
theta_a_new = x_np1(1:2:end);
theta_m_new = x_np1(2:2:end);
% add extra noise in the TCL state updates
theta_a_new = theta_a_new + normrnd(tclParameters.noiseMean,tclParameters.noiseSd,numTCL,1);
theta_m_new = theta_m_new + normrnd(tclParameters.noiseMean,tclParameters.noiseSd,numTCL,1);
end

