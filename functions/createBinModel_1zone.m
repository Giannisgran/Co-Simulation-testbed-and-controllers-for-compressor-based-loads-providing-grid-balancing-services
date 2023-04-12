% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [binModel] = createBinModel_1zone(tclParameters,generalParameters,binNumber,fullStateInfo,tclsThisAgg, Ta_binModel, controller_type)
% Function that creates the bin model that is used by the Markov controller.
% Input:
%   tclParameters: struct 
%       Includes the characteristics of the TCL population. e.g. thermal parameters, power draws etc.
%   generalParameters: struct
%       General info regarding the simulation. e.g. timesteps, feeder, TCL population size etc.
%   binNumber: int
%       Number of bins.
%   fullStateInfo: int
%       Option to have full state info (1) or not (0)
%   tclsThisAgg: vector
%       TCL indices that are assigned to the current aggregator
%   Ta_binModel: vector
%       Outdoor temperature values based on which the bin model will be
%       formed.
%   controller_type: str
%       Type of the controller, Markov controller or Markov controller with
%       lockouts
% Notes:
%   A: matrix of aggregate model {x(k+1)= Ax(k)+Bu(k), y = Cx} one such matrix for each considered outdoor temperature
%   B: matrix of aggregate model {x(k+1)= Ax(k)+Bu(k), y = Cx} for the assumed structure of matrix A
%   C: matrix of aggregate model {x(k+1)= Ax(k)+Bu(k), y = Cx}
%   T_a: outdoor temperatures considered
%   Pon_av: average consumption of ON TCLs, computed each time for different outdoor temperatures
%   binEdges: edges of the bins considered
%   x: percentage of TCL population at each bin
%   outdoorTemperature: vector of outdoor temps we want to find the transition matrix for
%   It will not be returning tclParameters, so any change made here doesn't
%   affect the struct in the initial main function

%% Set some options
simDuration = 5*3600; % duration of each simulation in seconds
simDurationIC = 5*3600; % duration of simulation for initial conditions/steady state in seconds
use_full_Q = 0; % 1 if we want to try using the whole Q in the Kalman filter, 0 to just use the diagonal

%% Modify tcl struct to include only the TCLs of interest - won't be returned by the function
if length(tclsThisAgg) ~= tclParameters.numAcs  % we care about a subset of the population here
    tclParameters = reduce_tcl_struct(tclParameters, tclsThisAgg, 1);
end
%% Create variables needed
% we want the bin model even without the Markov controller in order to get
% base power consumption. If PID or PEM controller are used instead, create
% the bin model like if we were using the Markov controller
if ~strcmp(controller_type, 'Markov controller') && ~strcmp(controller_type, 'Markov controller with lockouts') && ~strcmp(controller_type, 'Markov controller with delays')
    controller_type = 'Markov controller';
end
% total number of TCL population, could have picked any other field of tclParameters
numTCL = length(tclsThisAgg); 
% simulation time step, same as in the intial main function
h = generalParameters.timeStepInSec;
% number of iterations for each simulation
numSteps_train = round(simDuration/h); 
binModel.numSteps_train = numSteps_train;
% number of iterations for each simulation for initial conditions/steady-state
numStepsIC = round(simDurationIC/h); 
% number of control steps that will be executed in the actual simulation - needed for preallocation
numSteps = ceil(generalParameters.numSteps * (generalParameters.timeStepInSec/generalParameters.controlTimeStep));
% Vector of ambient temps we want to find transition matrices for
binModel.T_a = Ta_binModel; 
% calculate the maximum and minimum temperatures 
T_max = tclParameters.T_max;
T_min = tclParameters.T_min;
% keep full state info option
binModel.fullStateInfo = fullStateInfo;
% keep number of temperature bins
binModel.binNumber = binNumber;
% number of outdoor temperature intervals - if only one element exists
% then we essentially want the bin model for that specific outdoor temperature
num_Ta_bins = max(1, length(Ta_binModel)-1);

%% Modify general parameter struct as it will be used in the calculation of initial conditions/steady-state
generalParameters.Pop = numTCL;
generalParameters.numStepsIC = numStepsIC;
generalParameters.timeStepInSec = h;
generalParameters.timeStepInHr = h/3600;

%% Form the bins
% keep total number of bins
if strcmp(controller_type, 'Markov controller') % two cases: on or off
    binModel.totalBinNumber = 2*binNumber;
elseif strcmp(controller_type, 'Markov controller with lockouts') % 4 cases: on/off with and w/o lockout
    binModel.totalBinNumber = 4*binNumber;
elseif strcmp(controller_type, 'Markov controller with delays') % 4 cases: on/off with and w/o delays
    binModel.totalBinNumber = 4*binNumber;
end

% determine the edges of the bins for each TCL according to the 
% desired number of states
bin_edges = linspace(0,1,binNumber+1);
bin_edges(1) = -100;  % modify first bin in order to include anything below T_min
bin_edges(end) = 100; % modify last bin in order to include anything above T_max
binModel.binEdges = bin_edges; % store into the binModel struct

%% Determine the uncontrollable bins
% uncontrollable temperature bins
unc_perc_ON = 0.5; % percentage of OFF bins that will be uncontrollable for ON commands
unc_perc_OFF = 0.5; % percentage of ON bins that will be uncontrollable for OFF commands
binModel.uncBinsON = [1:ceil(binNumber*unc_perc_ON)]; % not controlled when turning ON
binModel.uncBinsOFF = binNumber + [1:ceil(binNumber*unc_perc_OFF)]; % not controlled when turning OFF

% determine bins that correspond to ON and OFF states separately. To be
% used during the control signal generation
if strcmp(controller_type, 'Markov controller')
    binModel.binsOFF = [1 : binNumber];
    binModel.binsON = [binNumber+1 : 2*binNumber]; 
elseif strcmp(controller_type, 'Markov controller with lockouts')
    binModel.binsOFF = [[1 : binNumber] [2*binNumber+1 : 3*binNumber]];
    binModel.binsON = [[binNumber+1 : 2*binNumber] [3*binNumber+1 : 4*binNumber]];
    binModel.uncBinsON = [binModel.uncBinsON [2*binNumber+1 : 3*binNumber]]; % OFF-locked TCLs aren't controllable
    binModel.uncBinsOFF = [binModel.uncBinsOFF [3*binNumber+1 : 4*binNumber]]; % ON-locked TCLs aren't controllable
elseif strcmp(controller_type, 'Markov controller with delays')
    % same as above, these are used for only the unlocked TCLs 
    binModel.binsOFF = [[1 : binNumber] [2*binNumber+1 : 3*binNumber]];
    binModel.binsON = [[binNumber+1 : 2*binNumber] [3*binNumber+1 : 4*binNumber]];
    binModel.uncBinsON = [binModel.uncBinsON [2*binNumber+1 : 3*binNumber]]; % don't control off,delayed because they will most likely be locked
    binModel.uncBinsOFF = [binModel.uncBinsOFF [3*binNumber+1 : 4*binNumber]]; % don't control on,delayed because they will most likely be locked
    warning('Delayed bins considered uncontrollable.');
end

% store bins that are controllable
binModel.conBinsOFF = setdiff(binModel.binsON, binModel.uncBinsOFF); % controllable bins for OFF commands
binModel.conBinsON = setdiff(binModel.binsOFF, binModel.uncBinsON); % controllable bins for ON commands

%% Create state-space matrices + matrices for Kalman filter
% preallocate for A,C - these will be created from simulation
binModel.A = cell(1,num_Ta_bins); % will include the Markov matrices for each outdoor temperature interval
binModel.A_unscaled = cell(1,num_Ta_bins); % will include the matrices with the transition counts for each outdoor temperature interval
binModel.C = cell(1,num_Ta_bins); % will include the C matrices for each outdoor temperature interval
% create matrix B in x(k+1)=Ax(k)+Bu(k)
if strcmp(controller_type, 'Markov controller')
    binModel.B = [-eye(binNumber);fliplr(eye(binNumber))];
elseif strcmp(controller_type, 'Markov controller with lockouts')
    binModel.B = [-eye(binNumber)        zeros(binNumber)       zeros(binNumber)       fliplr(eye(binNumber));
                  zeros(binNumber)       -eye(binNumber)        fliplr(eye(binNumber)) zeros(binNumber);
                  zeros(binNumber)       fliplr(eye(binNumber)) -eye(binNumber)        zeros(binNumber);
                  fliplr(eye(binNumber)) zeros(binNumber)       zeros(binNumber)       -eye(binNumber)];
elseif strcmp(controller_type, 'Markov controller with delays')
    binModel.B = [-eye(binNumber)        zeros(binNumber)       zeros(binNumber)       fliplr(eye(binNumber));
                  zeros(binNumber)       -eye(binNumber)        fliplr(eye(binNumber)) zeros(binNumber);
                  zeros(binNumber)       fliplr(eye(binNumber)) -eye(binNumber)        zeros(binNumber);
                  fliplr(eye(binNumber)) zeros(binNumber)       zeros(binNumber)       -eye(binNumber)];
end

% create the matrix which will be used for the output adaptive update
binModel.Aad = diag([repelem(-1,binNumber) zeros(1,binNumber)])+...
      diag([zeros(1,binNumber) repelem(1,binNumber)])+...
      diag([repelem(1,binNumber) zeros(1,binNumber-1)],-1)+...
      diag([zeros(1,binNumber) repelem(-1,binNumber-1)],-1);
binModel.Aad(1,binModel.totalBinNumber) = -1;
% initialize the matrix for the output adaptation of A
binModel.Awt = zeros(binModel.totalBinNumber);

if ~(fullStateInfo) % will use Kalman filter
    % measurement noise variance for state measurements
    binModel.measStateNoise = 10^(-10);
    % process noise that will be calculated in this function
    binModel.processNoise = cell( 1,num_Ta_bins );
    binModel.G = eye(binModel.totalBinNumber);
end
%% Preallocations
% errors of the kalman filter estimation for each bin at every timestep
binModel.estimatedStates = zeros(numSteps+1,binModel.totalBinNumber);
% preallocate inside the struct
binModel.Pon_av = zeros(1,num_Ta_bins);
binModel.Ptotal_av = zeros(1,num_Ta_bins);
% will be used to store the most recent A and C 
binModel.Aprev = zeros(binModel.totalBinNumber);
binModel.Cprev = zeros(1,binModel.totalBinNumber);
% predictions for next timesteps
binModel.Ppred = zeros(numSteps+1, 1);
% predictions for current timesteps
binModel.estimatedPagg = zeros(numSteps+1, 1);
% matrix to keep estimates of the Pon_av quantity during the simulation
binModel.Pon_av_est = zeros(1,numSteps+1);
% vectors that we will keep the estimated available TCLs for ON and OFF commands
binModel.available_tcls_ON  = zeros(1, numSteps+1);
binModel.available_tcls_OFF = zeros(1, numSteps+1);
% preallocate for the states of the bin model at each timestep
binModel.binStates_mat = zeros(numTCL, numSteps);
theta_a_norm = (tclParameters.theta_a0-repmat(T_min,1,size(tclParameters.theta_a0,2))) ./ (repmat(T_max,1,size(tclParameters.theta_a0,2))-repmat(T_min,1,size(tclParameters.theta_a0,2)));
if ~strcmp(controller_type, 'Markov controller with delays')
    binModel.binStates_mat(:,1) = map_to_bin_states(controller_type, bin_edges, binNumber, tclParameters.m0, theta_a_norm, tclParameters.timeUntilUnlocked);
else
    binModel.binStates_mat(:,1) = map_to_bin_states(controller_type, bin_edges, binNumber, tclParameters.m0, theta_a_norm, tclParameters.timeUntilDelayEnds);
end

% preallocate for the structs that will be used in upcoming simulation
airTemp_mat = zeros(numTCL, numSteps_train);
onOff_mat = zeros(numTCL, numSteps_train);
realPowerDraws_mat = zeros(numTCL, numSteps_train);
timeUntilUnlocked_mat = zeros(numTCL, numSteps_train); 
timeUntilDelayEnds_mat = zeros(numTCL, numSteps_train); 
% matrices where we are going to keep the total power and number of ON devices per timestep during training
binModel.sum_realPowerDraws_train = zeros(num_Ta_bins, numSteps_train);
binModel.sum_onOff_train = zeros(num_Ta_bins, numSteps_train);

%% Simulate the uncontrolled population for each desired outdoor temperature
% loop over the ambient temps, create matrices for each of those
for i = 1 : num_Ta_bins  
    if length(Ta_binModel) == 1 % only one outdoor temperature
        % set the outdoor temperature for all TCLs equal to the one current one
        tclParameters.T_a(:) = Ta_binModel(i);
    else % more than one intervals based on outdoor temperature
        % randomly sample the outdoor temperatures within the current bin
        tclParameters.T_a = unifrnd(Ta_binModel(i), Ta_binModel(i+1), size(tclParameters.T_a));
        % simulate for steady state, as we may be using a different outdoor
        % temperature here than what the population was initialized on
        [tclParameters] = calculateInitialConditions(generalParameters, tclParameters);
    end
    % start for the initial values obtained from the steady-state
    m = tclParameters.m0; % initial on/off states
    theta_a = tclParameters.theta_a0; % initial air temps
    theta_m = tclParameters.theta_m0; % initial mass temps  
    timeUntilUnlocked_mat(:,1) = tclParameters.timeUntilUnlocked;
    timeUntilDelayEnds_mat(:,1) = tclParameters.timeUntilDelayEnds;
    % initial values which come from the initialization script
    airTemp_mat(:,1) = tclParameters.theta_a0;
    onOff_mat(:,1) = tclParameters.m0;
    realPowerDraws_mat(:,1) = onOff_mat(:,1).*tclParameters.P_power_draw;
    % recompute the Q_h for each new temperature, then repeat each element in
    % the vector twice
    tclParameters = updateQh_1zone(tclParameters);

    % simulate all TCLs for a particular ambient temperature
    for n = 1:numSteps_train-1 % simulate the TCL's for the given simulation time
        % create struct with controller info
        [controllerInformation,tclParameters] = controllerSetup('none',tclParameters, generalParameters);
        controllerInformation.timeStep = n;
        controllerInformation.Pop = numTCL;
        controllerInformation.tclsAssigned = 1:numTCL;


        % set placeholders for potential control actions
        controlActions.deltaDeadband = zeros(numTCL, 1);
        controlActions.deltaSetpoint = zeros(numTCL, 1);
        controlActions.onOffSignal = zeros(numTCL, 1);
        % set 0 = no override, 1 = override off, 2 = override on
        controlActions.overRide = zeros(numTCL, 1);
        simulatedTclData = [];

        [controlActions, ~] = generateControlSignal(generalParameters, controllerInformation, simulatedTclData, tclParameters, m, theta_a, binModel, controlActions, theta_m);

        % update the TCLs
        simulationOptions.NumberOfAggregators = 0; % need this to simulate things here
        [m, theta_a, theta_m, tclParameters]= TCL_pop_sim_3state(m, theta_a, theta_m, generalParameters, tclParameters, controlActions, controllerInformation, simulationOptions);

        % store historical values at each timestep
        airTemp_mat(:,n+1) = theta_a;
        onOff_mat(:,n+1) = m;
        realPowerDraws_mat(:, n+1) =  m.*tclParameters.P_power_draw;
        timeUntilUnlocked_mat(:, n+1) = tclParameters.timeUntilUnlocked;
        timeUntilDelayEnds_mat(:, n+1) = tclParameters.timeUntilDelayEnds;
    end
    %% aggregate results to form transition matrix for the current outdoor temperature
    % keep info for later processing (fusing real and virtual transitions)
    binModel.sum_realPowerDraws_train(i,:) = sum(realPowerDraws_mat,1);
    binModel.sum_onOff_train(i,:) = sum(onOff_mat,1);
    % normalized temperatures according to the deadband
    airTemp_normed_mat = (airTemp_mat-repmat(T_min,1,size(airTemp_mat,2))) ./ (repmat(T_max,1,size(airTemp_mat,2))-repmat(T_min,1,size(airTemp_mat,2)));
    % find the state of each TCL at every timestep
    if ~strcmp(controller_type, 'Markov controller with delays')
        bin_states = map_to_bin_states(controller_type, bin_edges, binNumber, onOff_mat, airTemp_normed_mat, timeUntilUnlocked_mat);
    else
        bin_states = map_to_bin_states(controller_type, bin_edges, binNumber, onOff_mat, airTemp_normed_mat, timeUntilDelayEnds_mat);
    end
    [transitionMatrix, transitionMatrix_unscaled] = find_markov_matrix(bin_states, binModel.totalBinNumber);
    % replace the nan entries
    if any(isnan(transitionMatrix), 'all') && (strcmp(controller_type, 'Markov controller with delays') || strcmp(controller_type, 'Markov controller with lockouts'))
        warning('Some transitions were never observed during the creation of the bin matrix with delays/lockouts.');
        if strcmp(controller_type, 'Markov controller with delays')
            transitionMatrix = compute_theoretical_transition_matrix_delays(transitionMatrix, tclParameters, generalParameters, Ta_binModel(i), binModel, controller_type);
        elseif strcmp(controller_type, 'Markov controller with lockouts')
            transitionMatrix = compute_theoretical_transition_matrix_lockouts(transitionMatrix, tclParameters, generalParameters, Ta_binModel(i), binModel, controller_type);
        end
        disp('Using the theoritical A matrix.')
        if any(isnan(transitionMatrix), 'all')
            warning('A matrix still has NaN values even after considering the matrix resulting from controlled population.')
            transitionMatrix(isnan(transitionMatrix)) = 0;
        end
    elseif any(isnan(transitionMatrix), 'all') && strcmp(controller_type, 'Markov controller')
        warning('NaN values detected in the standard A matrix.')
        transitionMatrix(isnan(transitionMatrix)) = 0;
    end
    % store matrix into the binModel struct
    binModel.A{i} = transitionMatrix; 
    % store the matrix that includes the transitions 
    binModel.A_unscaled{i} = transitionMatrix_unscaled; 
    % store mean Pon_av for the current ambient temperature
    binModel.Pon_av(i) = nanmean(sum(realPowerDraws_mat,1)./sum(onOff_mat,1));
    % store mean average total consumption for the current ambient temperature
    binModel.Ptotal_av(i) = nanmean(sum(realPowerDraws_mat,1)); 

    % create C in y(k)=Cx(k) - different C for different T_a due to Pon_av being temp. dependent
    if strcmp(controller_type, 'Markov controller')
        binModel.C{i} = binModel.Pon_av(i)*numTCL*[zeros(1,binNumber) ones(1,binNumber)];
    elseif strcmp(controller_type, 'Markov controller with lockouts')
        binModel.C{i} = binModel.Pon_av(i)*numTCL*[zeros(1,binNumber) ones(1,binNumber) zeros(1,binNumber) ones(1,binNumber)];
    elseif strcmp(controller_type, 'Markov controller with delays')
        binModel.C{i} = binModel.Pon_av(i)*numTCL*[zeros(1,binNumber) ones(1,binNumber) zeros(1,binNumber) ones(1,binNumber)];        
    end
    
    if ~(fullStateInfo)
        % keep the percentage of the population at each bin for every timestep   
        x = bin_states;
        xc = mat2cell(x', ones(1,size(x,2)), size(x,1));                 % Split Matrix Into Cells By Row
        [hcell,~] = cellfun(@(x) histcounts(x,[1:binModel.totalBinNumber+1]), xc, 'Uni',0);   % Do ‘histcounts’ On Each Column
        states_mat = cell2mat(hcell)/numTCL; 
        % measurement noise variance for total power - needed if we want to
        % try using the full Q matrix
        % assume it is the same as the average consumption of the population for the current outdoor temperature
        binModel.measPaggNoise{i} = (0.05*binModel.Ptotal_av(i))^2; % this is variance=sd^2
        % find process noise and check whether a numerical issue arises
        % when using the Kalman filter
        binModel.processNoise{i} = calculate_processNoise(binModel, states_mat, transitionMatrix, i, binModel.measPaggNoise{i}, use_full_Q);
    end
end

end