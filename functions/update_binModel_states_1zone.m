% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [binModel] = update_binModel_states_1zone(binModel, tclParameters, theta_a, m, tclsThisAgg, u, n, controllerInformation)
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
% total number of TCLs
numTcl = length(theta_a(tclsThisAgg));
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
    Pagg = m(tclsThisAgg)'*tclParameters.P_power_draw(tclsThisAgg);
    R_Pagg = binModel.measPaggNoise{T}; % measurement noise for power, set during create_binModel  
else  % no perfect power measurements  
    R_Pagg = binModel.measPaggNoise{T}; % measurement noise for power, set during create_binModel  
    % we measure the total power draw: true + noise
    Pagg =  m(tclsThisAgg)'*tclParameters.P_power_draw(tclsThisAgg) + normrnd(0,sqrt(R_Pagg));
end

%% Determine the C matrix
% determine C based on outdoor temperature
if strcmp(controllerInformation.Type, 'Markov controller')
    C = Pon_av*numTcl*[zeros(1,binNumber) ones(1,binNumber)];
elseif strcmp(controllerInformation.Type, 'Markov controller with lockouts')
    C = Pon_av*numTcl*[zeros(1,binNumber) ones(1,binNumber) zeros(1,binNumber) ones(1,binNumber)];
elseif strcmp(controllerInformation.Type, 'Markov controller with delays')
    C = Pon_av*numTcl*[zeros(1,binNumber) ones(1,binNumber) zeros(1,binNumber) ones(1,binNumber)];
end

%% Output adaptation of A
% output adaptation for A if Ka not 0
if Ka~=0 && n_control>0
    Aad = binModel.Aad; % made if -1 and 1 at the right places
    A = A + Ka*Aad*e_int(n_control);
end

%% State adaptation of A
% if not yet reached the Nwt timesteps, don't do anything
% need to be a timestep that we receive state measurements
if Kwt ~= 0 && n >= Nwt && (receivedStateMeas || fullStateInfo)  
    bin_states = binModel.binStates_mat(:,n-Nwt+1:n);
    % find the Markov transition matrix
    Awt = find_markov_matrix(bin_states, binModel.totalBinNumber);
    % replace the nan entries with zeros
    Awt(isnan(Awt)) = 0;
    % keep adaptive A in the bin model
    binModel.Awt = Awt;
end
% adapt A matrix according to the known past states 
if Kwt ~= 0 && n >= Nwt
    A = Kwt*A + (1-Kwt)*binModel.Awt;
end
% keep state of each TCL if we want to use state adaptation
if Kwt ~= 0 
    theta_a_norm = (theta_a(tclsThisAgg)-repmat(tclParameters.T_min(tclsThisAgg),1,size(theta_a(tclsThisAgg),2))) ./ (repmat(tclParameters.T_max(tclsThisAgg),1,size(theta_a(tclsThisAgg),2))-repmat(tclParameters.T_min(tclsThisAgg),1,size(theta_a(tclsThisAgg),2)));
    if strcmp(controller_type, 'Markov controller') || strcmp(controller_type, 'Markov controller with lockouts')
        binModel.binStates_mat(:,n+1) = map_to_bin_states(controller_type, binModel.binEdges, binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilUnlocked(tclsThisAgg));
    elseif strcmp(controller_type, 'Markov controller with delays')
        binModel.binStates_mat(:,n+1) = map_to_bin_states(controller_type, binModel.binEdges, binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilDelayEnds(tclsThisAgg));
    end
end
% store the A matrix corresponding to the unforced system
binModel.Au = A;

%% Update states
if (fullStateInfo) % assume full state info, hence air temp and state known
    %% Full-state information
    % normalize first according to the deadband
    theta_a_norm = (theta_a(tclsThisAgg)-tclParameters.T_min(tclsThisAgg))./(tclParameters.T_max(tclsThisAgg)-tclParameters.T_min(tclsThisAgg));
    % assign each tcl to the corresponding bin depending on the initial temp and state
    if strcmp(controller_type, 'Markov controller') || strcmp(controller_type, 'Markov controller with lockouts')
        current_bins = map_to_bin_states(controller_type, binModel.binEdges, binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilUnlocked(tclsThisAgg));
    elseif strcmp(controller_type, 'Markov controller with delays')
        current_bins = map_to_bin_states(controller_type, binModel.binEdges, binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilDelayEnds(tclsThisAgg));
    end
    % x is the percentage of tcls within each bin, preallocate first
    x = zeros(totalBinNumber,1);
    % keep the percentage of each bin that is available for control, i.e.
    % not locked
    bin_availability = zeros(totalBinNumber,1);
    % count the number of TCLs in each bin
    for i = 1 : numTcl
        x(current_bins(i)) = x(current_bins(i)) + 1;
        isAvailable = ~(tclParameters.timeUntilUnlocked(tclsThisAgg(i))>0);
        bin_availability(current_bins(i)) = bin_availability(current_bins(i)) + isAvailable;
    end
    % percentage of the population in each bin that is available
    x_available = bin_availability/numTcl;
    % convert number of available TCLs into a percentage of the bin
    bin_availability = bin_availability./x;
    % deal with nan values resulting from empty states
    bin_availability(isnan(bin_availability)) = 1;
    % avoid the zeros because they will go to the denominator
    bin_availability(bin_availability==0) = 1;
    binModel.bin_availability = bin_availability;
    % normalize, find the percentage of tcls in each bin
    binModel.x = x/numTcl;
else % need to estimate the states   
    %% Output feedback only
    % gain for the Kalman innovation gain
    Km = controllerInformation.Km;
    % measurement noise for state info, from create_binModel
    R_state = binModel.measStateNoise;    
    if strcmp(controllerInformation.Type, 'Markov controller')
        if any(u<0) % we wanted to turn tcls OFF
            u = flip(u(binModel.binsON)); % only care about TCLs on ON state
        else % we wanted to turn tcls ON
            u = u(binModel.binsOFF); % only care about TCLs on OFF state
        end
    end
    
%   general form of the plant: x(k+1) = Ax(k) + Bu(k) + Gw(k), y(k) = Cx(k) + Du(k) + Hw(k) + v(k)
    Q = binModel.processNoise{T}; % process covariance, comes from create_binModel
    B = binModel.B; G = binModel.G;   
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
        if strcmp(controller_type, 'Markov controller') || strcmp(controller_type, 'Markov controller with lockouts')
            current_bins = map_to_bin_states(controller_type, binModel.binEdges, binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilUnlocked(tclsThisAgg));
        elseif strcmp(controller_type, 'Markov controller with delays')
            current_bins = map_to_bin_states(controller_type, binModel.binEdges, binNumber, m(tclsThisAgg), theta_a_norm, tclParameters.timeUntilDelayEnds(tclsThisAgg));
        end
        % count the number of TCLs in each bin --- depends on the type of the TCL
        % keep the percentage of each bin that is available for control, i.e.
        % not locked
        x_meas = zeros(totalBinNumber,1);
        bin_availability = zeros(totalBinNumber,1);
        for i = 1 : numTcl
            x_meas(current_bins(i)) = x_meas(current_bins(i)) + 1;
            isAvailable = ~(tclParameters.timeUntilUnlocked(tclsThisAgg(i))>0);
            bin_availability(current_bins(i)) = bin_availability(current_bins(i)) + isAvailable;
        end
        % percentage of the population in each bin that is available
        x_available = bin_availability/numTcl;
        % convert number of available TCLs into a percentage of the bin
        bin_availability = bin_availability./x_meas;
        % deal with nan values resulting from empty states
        bin_availability(isnan(bin_availability)) = 1;
        % avoid the zeros because they will go to the denominator
        bin_availability(bin_availability==0) = 1;
        binModel.bin_availability = bin_availability;
        % transform into a percentage of the total population
        x_meas = x_meas/numTcl;
        % add noise to the state measurements 
        x_meas = x_meas + normrnd(0,sqrt(R_state),size(x_meas)); 
        meas_vec = [Pagg;x_meas];
    else % no state measurements in this timestep - use only output measurements
        %% No state measurement at the current timestep
        Caug = [C; ones(1,totalBinNumber)]; % 2nd row incorporates the states summing to 1
        R = eye(2)*R_Pagg; R(2,2) = R_state;
        meas_vec = [Pagg;1]; % 1 to make estimated states sum to 1
    end

    % check if A or C have changed, C will also change when state
    % measurement is received, A will be constantly changing if
    % output-based adaptive strategy is used
    if isequal(A,binModel.Aprev) && isequal(Caug,binModel.Cprev)
        M = binModel.M;
    else
        lastwarn('');
        plant = ss(A,[B G],Caug,0,-1);
        [kalmf,L,P,M] = kalman(plant,Q,R); % only need the innovation matrix M
        if ~isempty(lastwarn) 
            fprintf('warning for singular matrix in kalman filter at timestep %d \n', n)
        else
            lastwarn('');
        end
        binModel.Cprev = Caug;
        binModel.Aprev = A;
        binModel.M = M;
    end
    % a priori estimate: x^(k|k-1) = Ax^(k-1|k-1) + Bu(k-1)
    x_apriori = A*binModel.x + B*u;
    % a posteriori estimate: x^(k|k) = x^(k|k-1) + M*( Pagg(k)-C*x^(k|k-1) )   
    binModel.x = x_apriori + Km*M*(meas_vec-Caug*x_apriori);
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
    num_on = sum(binModel.x(binModel.binsON))*numTcl;
    Pon_av = Pagg./num_on;
    binModel.Pon_av_current = Pon_av;
    binModel.Pon_av_est(max(1,n_control)) = Pon_av;
    if strcmp(controllerInformation.Type, 'Markov controller')
        C = Pon_av*numTcl*[zeros(1,binNumber) ones(1,binNumber)];
    elseif strcmp(controllerInformation.Type, 'Markov controller with lockouts')
        C = Pon_av*numTcl*[zeros(1,binNumber) ones(1,binNumber) zeros(1,binNumber) ones(1,binNumber)];
    elseif strcmp(controllerInformation.Type, 'Markov controller with delays')
        C = Pon_av*numTcl*[zeros(1,binNumber) ones(1,binNumber) zeros(1,binNumber) ones(1,binNumber)];
    end
end

%% Update structs
% update the available TCLs 
binModel.available_tcls_ON(n_control+1) = sum(x_available(binModel.conBinsON)); % available percentage of population for ON commands
binModel.available_tcls_OFF(n_control+1) = sum(x_available(binModel.conBinsOFF)); % available percentage of population for OFF commands

% keep the estimates inside binModel for post-processing
binModel.estimatedStates(n_control+1,:) = binModel.x';
% current estimated power draw
binModel.estimatedPagg(n_control+1) = C*binModel.x;
% store prediction for the power draw of the next timestep
binModel.Ppred(n_control+1) = C*(binModel.Au*binModel.x);

end