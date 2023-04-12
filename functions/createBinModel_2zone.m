% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [binModel] = createBinModel_2zone(tclParameters,generalParameters,binNumber,fullStateInfo,tclsThisAgg, Ta_binModel, controller_type)
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
simDuration = 5*3600; % duration of each simulation in seconds %Reduced for coding purpose
simDurationIC = 10*3600; % duration of simulation for initial conditions/steady state in seconds
use_full_Q = 0; % 1 if we want to try using the whole Q in the Kalman filter, 0 to just use the diagonal

%% Modify tcl struct to include only the TCLs of interest - won't be returned by the function
if length(tclsThisAgg) ~= tclParameters.numAcs  % we care about a subset of the population here
    tclParameters = reduce_tcl_struct(tclParameters, tclsThisAgg, 2);
end
%% Create variables needed
% we want the bin model even without the Markov controller in order to get
% base power consumption. If PID or PEM controller are used instead, create
% the bin model like if we were using the Markov controller
if ~strcmp(controller_type, 'Markov controller') && ~strcmp(controller_type, 'Markov controller with lockouts') ...
        && ~strcmp(controller_type, 'Markov controller 2 zone') && ~strcmp(controller_type, 'Markov controller 2 zone with lockouts')
    controller_type = 'Markov controller 2 zone';
end
% total number of TCL population, could have picked any other field of tclParameters
numTCL = length(tclsThisAgg); 
%Number of houses for two zone
numHouses = tclParameters.numHouses_2zone; 
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
if strcmp(controller_type, 'Markov controller 2 zone with lockouts')
    totalBinNumber = 8*binNumber*binNumber; % 8 cases with ON and OFF compressor lockout
else %2 Zone and no lockout
    totalBinNumber = 4*binNumber*binNumber; % 4 cases without compressor lockout
end
binModel.totalBinNumber = totalBinNumber;

% determine the edges of the bins for each TCL according to the 
% desired number of states
bin_1_edges = linspace(0,1,binNumber+1);
bin_1_edges(1) = -100;  % modify first bin in order to include anything below T_min
bin_1_edges(end) = 100; % modify last bin in order to include anything above T_max
bin_2_edges = linspace(0,1,binNumber+1);
bin_2_edges(1) = -100;  % modify first bin in order to include anything below T_min
bin_2_edges(end) = 100; % modify last bin in order to include anything above T_max
binModel.bin_1_edges = bin_1_edges; % store into the binModel struct
binModel.bin_2_edges = bin_2_edges; % store into the binModel struct

%Create indexing for the N1xN2 grid for two-zone control purposes
try
    %The index corresponds to each quadrant
    %Warning: N1 and N2 should be EVEN numbers
    %Not yet coded for ODD numbers
    % replace variables
    N1 = binNumber;
    N2 = binNumber;
    
    % The variables are binNumber X binNumber matrices with the columns
    % corresponding to the temp of zone 2 and the rows to zone 1.
    % Temperature increases as we going down or right.
    % Wherever there is a 1, the corresponding state is allowed to switch
    % Quadrants: Q1:(0,0), Q2:(0,1), Q3:(1,0), Q4:(1,1)
    
    
    % Targeting OFF bins (0,0) with the goal of switching ON. The Q means
    % quadrant and represents where we will be sending them 
    
    % Q1: from 1 OFF & 2 OFF to 1 OFF & 2 OFF - hence no control
    Q1_OFF = zeros(N1,N2);
    % Q2: from 1 OFF & 2 OFF to 1 OFF & 2 ON
    Q2_OFF = zeros(N1,N2);
    for i = 1:N1/2
        for j = i+1:N2
            Q2_OFF(i,j) = 1;
        end
    end
    %Q3: from 1 OFF & 2 OFF to 1 ON & 2 OFF
    Q3_OFF = Q2_OFF';
    %Q4: from 1 OFF & 2 OFF to 1 ON & 2 ON
    Q4_OFF = zeros(N1,N2);
    for i = (N1/2)+1:N1
        for j = (N2/2)+1:N2
            Q4_OFF(i,j) = 1;
        end
    end
    Q4_OFF = Q4_OFF + ~(Q2_OFF + Q3_OFF + Q4_OFF);
    % The following are aimed to the ON bins that we want to switch OFF.
    % Hence targets any of (0,1), (1,0), (1,1)
    
    % any bin that is switched is sent to Q1: (0,0)
    Q1_ON_FROM_Q2 = ones(N1,N2); % from (0,1) to (0,0)
    Q1_ON_FROM_Q3 = ones(N1,N2); % from (1,0) to (0,0)
    Q1_ON_FROM_Q4 = ones(N1,N2); % from (1,1) to (0,0)

    
    % set the uncontrollable bins
    if binNumber <= 6
        uncTempBinON = 1;
        uncTempBinOFF = 1;
    elseif binNumber <= 12
        uncTempBinON = 2;
        uncTempBinOFF = 2;
    else
        uncTempBinON = 3;
        uncTempBinOFF = 3;
    end
    % Q2_OFF: (0,0)->(0,1) hence need to constrain the lower zone 2 bins
    Q2_OFF(:,1:uncTempBinON) = 0;
    % Q3_OFF: (0,0)->(1,0) hence need to constrain the lower zone 1 bins
    Q3_OFF(1:uncTempBinON,:) = 0;
    % Q4_OFF: (0,0)->(1,1) hence need to constrain the lower bins
    Q4_OFF(1:uncTempBinON,:) = 0;
    Q4_OFF(:,1:uncTempBinON) = 0;
    % Q1_ON_FROM_Q2: (0,1)->(0,0) hence need to constrain the upper bins of zone 2
    Q1_ON_FROM_Q2(:,binNumber:-1:binNumber-uncTempBinOFF+1) = 0;
    % Q1_ON_FROM_Q3: (1,0)->(0,0) hence need to constrain the upper bins of zone 1
    Q1_ON_FROM_Q3(binNumber:-1:binNumber-uncTempBinOFF+1,:) = 0;
    % Q1_ON_FROM_Q4: (1,1)->(0,0) hence need to constrain the upper bins of both zones
    Q1_ON_FROM_Q4(binNumber:-1:binNumber-uncTempBinOFF+1,:) = 0;
    Q1_ON_FROM_Q4(:,binNumber:-1:binNumber-uncTempBinOFF+1) = 0;

    % Map the above matrices to vectors:
    % Batches of binNumber number of elements corresponding to the same
    % bin of zone 1. Within the batch, the bin of zone 2 changes.
    % e.g. for binNumber=6, element 13 correspond to bin 3 for zone 1 and
    % bin 1 for zone 2.
    binModel.q1_OFF = zeros(N1*N2,1);
    binModel.q2_OFF = zeros(N1*N2,1);
    binModel.q3_OFF = zeros(N1*N2,1);
    binModel.q4_OFF = zeros(N1*N2,1);
    binModel.q1_ON_FROM_Q2 = zeros(N1*N2,1);
    binModel.q1_ON_FROM_Q3 = zeros(N1*N2,1);
    binModel.q1_ON_FROM_Q4 = zeros(N1*N2,1);
    for i = 1:N1
        for j = 1:N2
            idx = (i-1)*N2 + j;
            binModel.q1_OFF(idx,1) = Q1_OFF(i,j);
            binModel.q2_OFF(idx,1) = Q2_OFF(i,j);
            binModel.q3_OFF(idx,1) = Q3_OFF(i,j);
            binModel.q4_OFF(idx,1) = Q4_OFF(i,j);
            binModel.q1_ON_FROM_Q2(idx,1) = Q1_ON_FROM_Q2(i,j);
            binModel.q1_ON_FROM_Q3(idx,1) = Q1_ON_FROM_Q3(i,j);
            binModel.q1_ON_FROM_Q4(idx,1) = Q1_ON_FROM_Q4(i,j);
        end
    end

    %Get indices
    [binModel.rq1_OFF_idx,~] = find(binModel.q1_OFF == 1);
    [binModel.rq2_OFF_idx,~] = find(binModel.q2_OFF == 1);
    [binModel.rq3_OFF_idx,~] = find(binModel.q3_OFF == 1);
    [binModel.rq4_OFF_idx,~] = find(binModel.q4_OFF == 1);
    [binModel.rq1_ON_FROM_Q2_idx,~] = find(binModel.q1_ON_FROM_Q2 == 1);
    [binModel.rq1_ON_FROM_Q3_idx,~] = find(binModel.q1_ON_FROM_Q3 == 1);
    [binModel.rq1_ON_FROM_Q4_idx,~] = find(binModel.q1_ON_FROM_Q4 == 1);
catch
    error('Clusters for two-zone are not created.')
end

% determine bins that correspond to ON and OFF states separately. To be
% used during the control signal generation
if strcmp(controller_type, 'Markov controller 2 zone with lockouts')
    binModel.binsOFF = [1:N1*N2,4*N1*N2+1:5*N1*N2];
    binModel.binsON = [N1*N2+1:4*N1*N2,5*N1*N2+1:8*N1*N2]; 
    binModel.binsUnlockedOFF = [1:N1*N2];
    binModel.binsLockedOFF = [4*N1*N2+1:5*N1*N2];
    binModel.binsUnlockedON = [N1*N2+1:4*N1*N2];
    binModel.binsLockedON = [5*N1*N2+1:8*N1*N2];
else %2 Zone and no lockout
    binModel.binsOFF = [1:N1*N2];
    binModel.binsON = [N1*N2+1 : 4*N1*N2]; 
end

% store controllable and uncontrollable bins 
binModel.conBinsOFF = [binModel.rq1_ON_FROM_Q2_idx+N1*N2;binModel.rq1_ON_FROM_Q3_idx+2*N1*N2;binModel.rq1_ON_FROM_Q4_idx+3*N1*N2]';
binModel.uncBinsOFF = setdiff(binModel.binsON, binModel.conBinsOFF);
binModel.conBinsON = sort([binModel.rq2_OFF_idx;binModel.rq3_OFF_idx;binModel.rq4_OFF_idx])';
binModel.uncBinsON = setdiff(binModel.binsOFF, binModel.conBinsON);

%% Create state-space matrices + matrices for Kalman filter
% preallocate for A,C - these will be created from simulation
binModel.A = cell(1,num_Ta_bins); % will include the Markov matrices for each outdoor temperature interval
binModel.A_unscaled = cell(1,num_Ta_bins); % will include the matrices with the transition counts for each outdoor temperature interval
binModel.C = cell(1,num_Ta_bins); % will include the C matrices for each outdoor temperature interval

if ~(fullStateInfo) % will use Kalman filter
    % measurement noise variance for state measurements
    binModel.measStateNoise = 10^(-10);
    % process noise that will be calculated in this function
    binModel.processNoise = cell(1,num_Ta_bins);
    binModel.D = zeros (1,binModel.totalBinNumber/2);
    binModel.H = zeros(1,binModel.totalBinNumber);
    binModel.G = eye(binModel.totalBinNumber);
    % initialize parameters for Kalman Filter
    binModel.P_pre = 1e-4*eye(totalBinNumber);
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
binModel.binStates_mat = zeros(numHouses, numSteps+1); % edited for two zone

theta_a_norm = (tclParameters.theta_a0-repmat(T_min,1,size(tclParameters.theta_a0,2))) ./ (repmat(T_max,1,size(tclParameters.theta_a0,2))-repmat(T_min,1,size(tclParameters.theta_a0,2)));
binModel.binStates_mat(:,1) = map_to_bin_states(controller_type, [bin_1_edges;bin_2_edges], binNumber, tclParameters.m0, theta_a_norm, tclParameters.timeUntilUnlocked); 

% preallocate for the structs that will be used in upcoming simulation
airTemp_mat = zeros(numTCL, numSteps_train);
onOff_mat = zeros(numTCL, numSteps_train); %This is q in the ACC paper
realPowerDraws_mat = zeros(numTCL, numSteps_train);
timeUntilUnlocked_mat = zeros(numTCL, numSteps_train);
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
    % start for the initial values again
    m = tclParameters.m0; % initial on/off states. This is cooling request q in the ACC paper
    theta_a = tclParameters.theta_a0; % initial air temps
    theta_m = tclParameters.theta_m0; % initial mass temps 
    timeUntilUnlocked_mat(:,1) = tclParameters.timeUntilUnlocked;
    % initial values which come from the initialization script
    airTemp_mat(:,1) = tclParameters.theta_a0;
    onOff_mat(:,1) = tclParameters.m0;
    realPowerDraws_mat(:,1) = onOff_mat(:,1).*tclParameters.P_power_draw;        
    % recompute the Q_h for each new temperature, then repeat each element in
    % the vector twice
    tclParameters = updateQh_2zone(tclParameters, m);

    % simulate all TCLs for a particular ambient temperature
    for n = 1:numSteps_train-1 % simulate the TLC's for the given simulation time
        % create struct with controller info
        [controllerInformation,tclParameters] = controllerSetup('none',tclParameters, generalParameters);
        controllerInformation.timeStep = n;
        controllerInformation.Pop = numel(tclParameters.T_a);
        controllerInformation.tclsAssigned = 1:numel(tclParameters.T_a);


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
    end 
    %% aggregate results to form transition matrix for the current outdoor temperature
    % keep info for later processing (fusing real and virtual transitions)
    binModel.sum_realPowerDraws_train(i,:) = sum(realPowerDraws_mat,1);
    binModel.sum_onOff_train(i,:) = sum(onOff_mat,1);
    % normalized temperatures according to the deadband
    airTemp_normed_mat = (airTemp_mat-repmat(T_min,1,size(airTemp_mat,2))) ./ (repmat(T_max,1,size(airTemp_mat,2))-repmat(T_min,1,size(airTemp_mat,2)));
    % find the state of each TCL at every timestep
    bin_states = map_to_bin_states(controller_type, [bin_1_edges;bin_2_edges], binNumber, onOff_mat, airTemp_normed_mat, timeUntilUnlocked_mat);
    % find the Markov transition matrix
    [transitionMatrix, transitionMatrix_unscaled] = find_markov_matrix(bin_states, binModel.totalBinNumber);
    if any(isnan(transitionMatrix), 'all')
        warning('Some states were never visited during the identification of the A matrix for the 2-zone bin model.')
        % replace the nan entries with zeros
        transitionMatrix(isnan(transitionMatrix)) = 0;
    end

    % store matrix into the binModel struct
    binModel.A{i} = transitionMatrix; 
    % store the matrix that includes the transitions 
    binModel.A_unscaled{i} = transitionMatrix_unscaled; 
    % store mean Pon_av for the current ambient temperature
    numOn = sum(onOff_mat(tclParameters.tcls_2zone_first,:) | onOff_mat(tclParameters.tcls_2zone_second,:));
    binModel.Pon_av(i) = nanmean(sum(realPowerDraws_mat,1)/numOn);
    % store mean average total consumption for the current ambient temperature
    binModel.Ptotal_av(i) = nanmean(sum(realPowerDraws_mat,1)); 

    % create C in y(k)=Cx(k) - different C for different T_a due to Pon_av being temp. dependent
    if strcmp(controller_type, 'Markov controller 2 zone with lockouts')
        binModel.C{i} = binModel.Pon_av(i)*numHouses*[zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber) zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber)];
    else
        binModel.C{i} = binModel.Pon_av(i)*numHouses*[zeros(1,binNumber*binNumber) ones(1,3*binNumber*binNumber)];
    end
    % keep the percentage of the population at each bin for every timestep   
    x = bin_states;
    xc = mat2cell(x', ones(1,size(x,2)), size(x,1));                 % Split Matrix Into Cells By Row
    [hcell,~] = cellfun(@(x) histcounts(x,[1:binModel.totalBinNumber+1]), xc, 'Uni',0);   % Do ?histcounts? On Each Column
    states_mat = cell2mat(hcell)/numHouses; 

    if ~(fullStateInfo)
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