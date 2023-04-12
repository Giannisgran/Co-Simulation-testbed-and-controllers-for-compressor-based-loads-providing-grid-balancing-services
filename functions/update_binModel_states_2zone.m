% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [binModel] = update_binModel_states_2zone(binModel, tclParameters, theta_a, m, tclsThisAgg, u, n, controllerInformation)
%{
Updates the state vector of the bin model either by using full-state info
or by using a Kalman filter
Input:
    binModel: struct
        Existing struct for the bin model.
    tclParameters: struct
        Information for the TCLs is stored there.
    theta_a: vector of floats
        Current indoor temperature for each TCL. Only used if state
        measurement is available.
    m: vector of ints
        Compressor mode for each TCL (1 if ON 0 if OFF).
    tclsThisAgg: vector of ints
        TCLs of the current aggregator.
    u: struct
        Control vector sent to the TCLs.
    n: int
        Current timestep.
    controllerInformation: struct
        Information for the controller.
Output:
    binModel: struct
        Updated bin model struct.
%}

%% Define some needed variables
% current timestep in terms of controller calls
n_control = controllerInformation.timeStep;
% total number of houses
numHouses = length(tclsThisAgg)/2;
% number of bins that discretize the temperature range
binNumber = binModel.binNumber;
% total number of bins, some multiple of the above based on the model
totalBinNumber = binModel.totalBinNumber;
% type of the controller e.g. Markov controller or Markov controller with lockouts
controller_type = controllerInformation.Type;
% load full state info option
if n ~= 0 
    fullStateInfo = binModel.fullStateInfo;
else % we know the initial states at the beginning of the simulation
    fullStateInfo = 1;
end
% load output adaptation gain from controller
Ka = controllerInformation.Ka;
% load state adaptation gain from controller
Kwt = controllerInformation.Kwt;
% load number of timesteps to be used for state adaptation
Nwt = controllerInformation.Nwt;
% load integral error from controller 
e_int = controllerInformation.e_int;
% if at the start of the simulation then we know the initial point
if mod(n_control, controllerInformation.measRecieveTimestep)==0
    receivedStateMeas = 1;
else
    receivedStateMeas = 0;
end

%% Find the state matrices corresponding to the current outdoor temperature
% find the first binModel temeprature greater than the current outdoor temp
% assumed that all TCLs have the same ambient temperature
% haven't taken into account if temps are lower than 20 or higher than
% 40 which are the edge temperatures considered in the binModel right now
if length(binModel.T_a) == 1  % bin model based on only one outdoor temp, nothing to search for
	T = 1;
	A = binModel.A{1};
	Pon_av = binModel.Pon_av(1);
    Ptotal_av = binModel.Ptotal_av(1);
elseif any(tclParameters.T_a(1) == binModel.T_a)  % more than one bin models, check if the outdoor temperature is the same
    T = find(tclParameters.T_a(1) == binModel.T_a);
    A = binModel.A{T};
    Pon_av = binModel.Pon_av(T);
    Ptotal_av = binModel.Ptotal_av(T);
else  % more than one bin models but did not find the same outdoor temperature, use linear interpolation
    T = find(binModel.T_a>=tclParameters.T_a(1));
    T = max(1,T(1)-1); % keep only the first one - don't want it to be zero
    T_higher = binModel.T_a(T+1);
    T_lower = binModel.T_a(T);
    if T == length(binModel.Pon_av) % last bin
        % use the last bin's variables
        A = binModel.A{T};
        Pon_av = binModel.Pon_av(T);
        Ptotal_av = binModel.Ptotal_av(T);
    else
        % find the relative position of current temp between the above two
        a = (tclParameters.T_a(1)-T_lower)/(T_higher-T_lower);
        % Interpolate the transition matrices
        A = binModel.A{T}*a + (1-a)*binModel.A{T+1};
        Pon_av = binModel.Pon_av(T)*a + (1-a)*binModel.Pon_av(T+1);
        Ptotal_av = binModel.Ptotal_av(T)*a + (1-a)*binModel.Ptotal_av(T+1);
    end
end
% store current Pon_av to use it in generateControlSignal
binModel.Pon_av_current = Pon_av;
% store current Ptotal_av for potential use
binModel.Ptotal_av_current = Ptotal_av;
% store estimated Pon_av for post-processing after the simulation is over
if n_control > 0 % make sure we are not at the onInit 
    binModel.Pon_av_est(n_control) = Pon_av;
end
%% Power measurement
if fullStateInfo
    Pagg = m(tclsThisAgg)'*tclParameters.P_power_draw(tclsThisAgg);
elseif receivedStateMeas
    R_Pagg = binModel.measPaggNoise{T}; % measurement noise for power, set during create_binModel
    Pagg = m(tclsThisAgg)'*tclParameters.P_power_draw(tclsThisAgg);    
else  % no perfect power measurements
    R_Pagg = binModel.measPaggNoise{T}; % measurement noise for power, set during create_binModel
    % we measure the total power draw: true + noise
    Pagg =  m(tclsThisAgg)'*tclParameters.P_power_draw(tclsThisAgg) + normrnd(0,sqrt(R_Pagg));
end
%% determine C based on outdoor temperature
if strcmp(controller_type, 'Markov controller 2 zone with lockouts')
    C = Pon_av*numHouses*[zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber) zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber)];
else
    C = Pon_av*numHouses*[zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber)];
end
%% Update states
if (fullStateInfo) % assume full state info, hence air temp and state known
    %% Full-state information
    % normalize first according to the deadband
    theta_a_norm = (theta_a(tclsThisAgg)-tclParameters.T_min(tclsThisAgg))./(tclParameters.T_max(tclsThisAgg)-tclParameters.T_min(tclsThisAgg));
    % assign each tcl to the corresponding bin depending on the initial temp and state
    current_bins = map_to_bin_states(controller_type, [binModel.bin_1_edges;binModel.bin_2_edges], binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilUnlocked(tclsThisAgg));
    % x is the percentage of tcls within each bin, preallocate first
    x = zeros(totalBinNumber,1);
    % count the number of TCLs in each bin --- depends on the type of the TCL
    % keep the percentage of each bin that is available for control, i.e.
    % not locked
    bin_availability = zeros(totalBinNumber,1);
    for i = 1 : numHouses
        x(current_bins(i)) = x(current_bins(i)) + 1;
        isAvailable = ~(tclParameters.timeUntilUnlocked(tclsThisAgg(2*i-1))>0);
        bin_availability(current_bins(i)) = bin_availability(current_bins(i)) + isAvailable;
    end
    % percentage of the population in each bin that is available
    x_available = bin_availability/numHouses;
    % convert number of available TCLs into a percentage of the bin
    bin_availability = bin_availability./x;
    % deal with nan values resulting from empty states
    bin_availability(isnan(bin_availability)) = 1;
    % avoid the zeros because they will go to the denominator
    bin_availability(bin_availability==0) = 1;
    binModel.bin_availability = bin_availability;
    % normalize, find the percentage of tcls in each bin
    binModel.x = x/numHouses;
else % need to estimate the states   
    %% Output feedback only
    % gain for the Kalman innovation gain
    Km = controllerInformation.Km;
    % measurement noise for state info, from create_binModel
    R_state = binModel.measStateNoise; 
%   general form of the plant: x(k+1) = Ax(k) + Bu(k) + Gw(k), y(k) = Cx(k) + Du(k) + Hw(k) + v(k)
    Q = binModel.processNoise{T}; % process covariance, comes from create_binModel
    D = binModel.D; H = binModel.H; G = binModel.G; %B = binModel.B; 
    if (receivedStateMeas) % received state measurements at this timestep
        %% State measurement at the current timestep
        % measurement covariance
        R = eye(1+totalBinNumber)*R_state; % component for state measurements, +1 to account for the power measurement
        R(1,1) = R_Pagg;  % component for power measurement
        Caug = [C; eye(totalBinNumber)]; % measurements are [Power; x]
        % measurements of states
        % normalize first according to the deadband
        theta_a_norm = (theta_a(tclsThisAgg)-tclParameters.T_min(tclsThisAgg))./(tclParameters.T_max(tclsThisAgg)-tclParameters.T_min(tclsThisAgg));
        % assign each tcl to the corresponding bin depending on the initial temp and state
        current_bins = map_to_bin_states(controller_type, [binModel.bin_1_edges;binModel.bin_2_edges], binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilUnlocked(tclsThisAgg));
        % count the number of TCLs in each bin --- depends on the type of the TCL
        % keep the percentage of each bin that is available for control, i.e.
        % not locked
        x_meas = zeros(totalBinNumber,1);
        bin_availability = zeros(totalBinNumber,1);
        for i = 1 : numHouses
            x_meas(current_bins(i)) = x_meas(current_bins(i)) + 1;
            isAvailable = ~(tclParameters.timeUntilUnlocked(tclsThisAgg(2*i-1))>0);
            bin_availability(current_bins(i)) = bin_availability(current_bins(i)) + isAvailable;
        end
        % percentage of the population in each bin that is available
        x_available = bin_availability/numHouses;
        % convert number of available TCLs into a percentage of the bin
        bin_availability = bin_availability./x_meas;
        % deal with nan values resulting from empty states
        bin_availability(isnan(bin_availability)) = 1;
        % avoid the zeros because they will go to the denominator
        bin_availability(bin_availability==0) = 1;
        binModel.bin_availability = bin_availability;
        % transform into a percentage of the total population
        x_meas = x_meas/numHouses;
        % add noise to the state measurements 
        x_meas = x_meas + normrnd(0,sqrt(R_state),size(x_meas)); 
        meas_vec = [Pagg;x_meas];
    else % no state measurements in this timestep - use only output measurements
        %% No state measurement at the current timestep
        Caug = [C; ones(1,totalBinNumber)]; % 2nd row incorporates the states summing to 1
        R = eye(2)*R_Pagg; R(2,2) = 10^(-7);
        meas_vec = [Pagg;1]; % 1 to make estimated states sum to 1
    end
    %Redefine number of bins
    N1 = binNumber;
    N2 = binNumber;
    %Map control to their corresponding bins
    u2_ON = u(1:N1*N2); % corresponding to (0,0)->(0,1)
    u3_ON = u(N1*N2+1:2*N1*N2); % corresponding to (0,0)->(1,0)
    u4_ON = u(2*N1*N2+1:3*N1*N2);  % corresponding to (0,0)->(1,1)
    u2_OFF = u(3*N1*N2+1:4*N1*N2);  % corresponding to (0,1)->(0,0)
    u3_OFF = u(4*N1*N2+1:5*N1*N2); % corresponding to (1,0)->(0,0)
    u4_OFF = u(5*N1*N2+1:6*N1*N2); % corresponding to (1,1)->(0,0)
    %Determine the B(u) matrix
    if strcmp(controller_type, 'Markov controller 2 zone with lockouts')
        Bu = zeros(8*binNumber*binNumber,8*binNumber*binNumber);
        Bu(1:N1*N2,1:N1*N2) = -diag(u2_ON + u3_ON + u4_ON);
        Bu(5*N1*N2+1:6*N1*N2,1:N1*N2) = diag(u2_ON);
        Bu(6*N1*N2+1:7*N1*N2,1:N1*N2) = diag(u3_ON);
        Bu(7*N1*N2+1:8*N1*N2,1:N1*N2) = diag(u4_ON);
        Bu(N1*N2+1:2*N1*N2,N1*N2+1:2*N1*N2) = -diag(u2_OFF);
        Bu(2*N1*N2+1:3*N1*N2,2*N1*N2+1:3*N1*N2) = -diag(u3_OFF);
        Bu(3*N1*N2+1:4*N1*N2,3*N1*N2+1:4*N1*N2) = -diag(u4_OFF);
        Bu(4*N1*N2+1:5*N1*N2,N1*N2+1:2*N1*N2) = diag(u2_OFF);
        Bu(4*N1*N2+1:5*N1*N2,2*N1*N2+1:3*N1*N2) = diag(u3_OFF);
        Bu(4*N1*N2+1:5*N1*N2,3*N1*N2+1:4*N1*N2) = diag(u4_OFF);
        size_n = 8*N1*N2;
    else
        Bu = zeros(4*binNumber*binNumber,4*binNumber*binNumber);
        Bu(1:N1*N2,1:N1*N2) = -diag(u2_ON + u3_ON + u4_ON);
        Bu(N1*N2+1:2*N1*N2,1:N1*N2) = diag(u2_ON);
        Bu(2*N1*N2+1:3*N1*N2,1:N1*N2) = diag(u3_ON);
        Bu(3*N1*N2+1:4*N1*N2,1:N1*N2) = diag(u4_ON);
        Bu(N1*N2+1:2*N1*N2,N1*N2+1:2*N1*N2) = -diag(u2_OFF);
        Bu(2*N1*N2+1:3*N1*N2,2*N1*N2+1:3*N1*N2) = -diag(u3_OFF);
        Bu(3*N1*N2+1:4*N1*N2,3*N1*N2+1:4*N1*N2) = -diag(u4_OFF);
        Bu(1:N1*N2,N1*N2+1:2*N1*N2) = diag(u2_OFF);
        Bu(1:N1*N2,2*N1*N2+1:3*N1*N2) = diag(u3_OFF);
        Bu(1:N1*N2,3*N1*N2+1:4*N1*N2) = diag(u4_OFF);
        size_n = 4*N1*N2;
    end
    
    %Previous values
    x_pre = binModel.x;
    P_pre = binModel.P_pre;
    
    %New A matrix for Kalman Filter
    A_new = (Bu + eye(size_n))*A;

    %Time update
    x_new_min = A_new*x_pre;
    Pk_min = A_new*P_pre*A_new' + Q;

    %Measurement Update
    Kk = Pk_min*Caug'/(Caug*Pk_min*Caug' + R);
    binModel.x = x_new_min + Kk*(meas_vec-Caug*x_new_min);
    binModel.P_pre = (eye(size_n)-Kk*Caug)*Pk_min;

    % filter values below zero and above 1
    binModel.x(binModel.x<0) = 0;
    binModel.x(binModel.x>1) = 1;  
    
    % percentage of each bin that is avaialble for control - TO DO
    binModel.bin_availability = ones(totalBinNumber,1);
    % percantage of population in each bin that is available 
    x_available = binModel.x;
end

%% Update the C matrix if using varying Pon_av
% compute Pon_av based on the current timestep's data
if controllerInformation.adjust_Pon_av_option
    num_on = sum(m(tclParameters.tcls_2zone_first,:) | m(tclParameters.tcls_2zone_second,:));
    Pon_av = Pagg/num_on;
    binModel.Pon_av_current = Pon_av;
    binModel.Pon_av_est(max(1,n_control)) = Pon_av;
    if strcmp(controller_type, 'Markov controller 2 zone with lockouts')
        C = Pon_av*numHouses*[zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber) zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber)];
    else
        C = Pon_av*numHouses*[zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber)];
    end
end

%% Update structs
% update population availability for ON and OFF commands
binModel.available_tcls_ON(n_control+1) = sum(x_available(binModel.conBinsON)); % available percentage of population for ON commands
binModel.available_tcls_OFF(n_control+1) = sum(x_available(binModel.conBinsOFF)); % available percentage of population for OFF commands

% keep the estimates inside binModel for post-processing
binModel.estimatedStates(n_control+1,:) = binModel.x';
% current estimated power draw
binModel.estimatedPagg(n_control+1) = C*binModel.x;
% store prediction for the power draw of the next timestep
binModel.x_proj = A*binModel.x;
binModel.Ppred(n_control+1) = C*binModel.x_proj; 

end