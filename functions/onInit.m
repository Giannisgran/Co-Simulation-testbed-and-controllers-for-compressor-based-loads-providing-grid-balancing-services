% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.


% if this doesn't exist, we're running it from GLD and need to set this to
% proceed
if ~exist('simulationOptions','var') || ~isfield(simulationOptions,'useGLD')
   simulationOptions.useGLD = 1; 
end

% if we're running with GLD, load the saved info, otherwise it already
% exists in the workspace
if simulationOptions.useGLD
    % load variables that we pass from the main script, or that we need when
    % running gridlab-d
    load('thisRunData.mat')
end
    
% if we're running with gld, add the necessary folders to the path
if simulationOptions.useGLD == 1
    % gets the folder that contains the mfile, add that folder plus its
    % subfolders to the path, then make that filder the current directory
    [thisFolder]  = fileparts(mfilename('fullpath'));
    addpath(genpath(thisFolder));
    cd(thisFolder);
    addpath(genpath('../../functions'));
    addpath(genpath('../../Comm. functions'));
    addpath(genpath('../../dataFiles'));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GET PARAMETERS FOR THIS CASE

simulationOptions.regSignal = parameterSweepSimulation.regulationSignalTypeValues(idx,:)';
simulationOptions.regNumCycles = parameterSweepSimulation.regulationSignalNumCyclesValues(idx,:)';
simulationOptions.regAmpl = parameterSweepSimulation.regulationSignalAmplitudeValues(idx,:)';
simulationOptions.percentTclOption = parameterSweepSimulation.percentTclOptionSet(idx);
simulationOptions.feederCapScaling = parameterSweepSimulation.feederCapacityScalingOptionSet(idx);
simulationOptions.backgroundDemandScaling = parameterSweepSimulation.backgroundDemandScalingFactorOptionSet(idx);
simulationOptions.feederFile = parameterSweepSimulation.setOfTaxFiles{idx};
simulationOptions.weather = parameterSweepSimulation.weatherSet(idx);
simulationOptions.buildingParameter = parameterSweepSimulation.buildingParameterSet(idx);
simulationOptions.tclHeterogeneity = parameterSweepSimulation.tclHeterogeneitySet(idx);
simulationOptions.lockoutOn = parameterSweepSimulation.lockoutTimeOnSet(idx);
simulationOptions.lockoutOff = parameterSweepSimulation.lockoutTimeOffSet(idx);
simulationOptions.commNetwork = parameterSweepSimulation.CommNetwork(idx);
simulationOptions.voltageRegSet = parameterSweepSimulation.voltageRegSet(idx);


% enable communication network
% 0: no comm network , 1: consider comm network
if simulationOptions.commNetwork  == 1 
    simulationOptions.considerCommNetwork = 0;
else
    simulationOptions.considerCommNetwork = 1;
end

% END CASE PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GET INFO SPECIFIC TO THIS SIMULATION

% get the data for the scenario that we calculated before and pull the
% field names to create them as variables.
[houseParameters, generalParameters, feederParameters, ...
    tclParameters, outdoorTemperature, irradiance] = ...
    getGeneratedData(simulationOptions);
outdoorTemperature = outdoorTemperature(1:simulationOptions.simLength*3600/generalParameters.timeStepInSec + 1);
irradiance = irradiance(1:simulationOptions.simLength*3600/generalParameters.timeStepInSec + 1);


% now that we have the general parameter structure, set the comm
% network value
generalParameters.considerCommNetwork = simulationOptions.considerCommNetwork;
% option to include delay for the temperature to move towards opposite
% direction after a switch has been made
tclParameters.temperatureDelayOption = temperatureDelayOption;

% set the number of steps in the simulation
generalParameters.simLength = simulationOptions.simLength;
generalParameters.numSteps = simulationOptions.simLength/generalParameters.timeStepInHr;

% assign TCLs to the available aggregators. 
tclParameters.aggregatorAssignment = ...
    getAggregatorAssignment(simulationOptions, houseParameters, tclParameters);

% set controller options for each aggregator
controllerInformation.controlTimeStep = simulationOptions.controlTimeStep; % [sec]
generalParameters.controlTimeStep = simulationOptions.controlTimeStep; % [sec]
for aggIdx = 1:simulationOptions.NumberOfAggregators

    aggString = ['agg' num2str(aggIdx)]';
    
    
    % set the population in under this aggregator
    controllerInformation.(aggString).Pop = ...
        sum(tclParameters.aggregatorAssignment== aggIdx);
    
    % get the tcl indices for this aggregator
    controllerInformation.(aggString).tclsAssigned = find(tclParameters.aggregatorAssignment == aggIdx);
    
    controllerInformation.(aggString).controlTimeStepInSec = simulationOptions.controlTimeStep(aggIdx); 
    
    % create struct with controller info
    controllerInformation.(aggString).feeder = simulationOptions.feederFile;
    controllerInformation.(aggString).useLocalVoltage = simulationOptions.useLocalVoltage; 
    controllerInformation.(aggString).usePEMTempDeadband = simulationOptions.usePEMTempDeadband;

    [controllerInformation.(aggString),tclParameters] = controllerSetup(simulationOptions.controllerType{aggIdx},tclParameters,generalParameters, controllerInformation.(aggString), numTempIntervals(aggIdx), numTempIntervals2zone(aggIdx));
    
    % set placeholders for potential control actions
    controlActionsFromController.deltaDeadband = zeros(generalParameters.Pop,1);
    controlActionsFromController.deltaSetpoint = zeros(generalParameters.Pop,1);
    controlActionsFromController.onOffSignal = zeros(controllerInformation.agg1.controlSignalLength,1);
    % set 0 = no override, 1 = override off, 2 = override on
    controlActionsFromController.overRide = zeros(generalParameters.Pop,1);
    
    % get the bin model
    tclsThisAgg = controllerInformation.(aggString).tclsAssigned;
    Ta_binModel  = (min(outdoorTemperature) : 1 : max(outdoorTemperature));
    % make sure the max temp is captured in T_a
    if Ta_binModel(end) < max(outdoorTemperature)
        Ta_binModel = [Ta_binModel max(outdoorTemperature)];
    end
    if 1 == 1
        if tclParameters.perc_2zone == 0  % only single-zone houses
            [binModel] = createBinModel_1zone(tclParameters,generalParameters,numTempIntervals(aggIdx),fullStateInfo,tclsThisAgg, Ta_binModel, controllerInformation.(aggString).Type);
        elseif tclParameters.perc_2zone == 1  % only 2-zone houses
            [binModel] = createBinModel_2zone(tclParameters,generalParameters,numTempIntervals2zone(aggIdx),fullStateInfo,tclsThisAgg, Ta_binModel, controllerInformation.(aggString).Type);
        else % population includes both single and 2-zone houses
            tclsThisAgg_1zone = intersect(tclsThisAgg, tclParameters.tcls_1zone); % single-zone TCLs assigned to the current aggregator
            tclsThisAgg_2zone = setdiff(tclsThisAgg, tclsThisAgg_1zone); % 2-zone TCLs assigned to the current aggregator
            [binModel.binModel_1zone] = createBinModel_1zone(tclParameters,generalParameters,numTempIntervals(aggIdx),fullStateInfo,tclsThisAgg_1zone, Ta_binModel, controllerInformation.(aggString).controllerInformation_1zone.Type);
            [binModel.binModel_2zone] = createBinModel_2zone(tclParameters,generalParameters,numTempIntervals2zone(aggIdx),fullStateInfo,tclsThisAgg_2zone, Ta_binModel, controllerInformation.(aggString).controllerInformation_2zone.Type);            
            binModel.Ptotal_av = binModel.binModel_1zone.Ptotal_av + binModel.binModel_2zone.Ptotal_av;  % assumes that the training times of the two bin models are equal
            binModel.T_a = binModel.binModel_1zone.T_a;  % keep the outdoor temperature that the bin model is based on              
        end
        save(['case' num2str(idx) '_' aggString(:)' '_binModel'],'binModel')
    else
        if simulationOptions.useGLD == 1
            load(['../../case' num2str(idx) '_' aggString(:)' '_binModel'],'binModel')
        else
            load(['case' num2str(idx) '_' aggString(:)' '_binModel'],'binModel')
        end
    end
    controllerInformation.(aggString).binModel = binModel;
   
    % get the average Pon value for the aggregator's tcl population
    temp = mean(tclParameters.T_a(controllerInformation.(aggString).tclsAssigned));

    if numel(binModel.T_a) > 2
        tclPopBaseConsumption = interp1(binModel.T_a,[binModel.Ptotal_av binModel.Ptotal_av(end)],temp);
    else
        tclPopBaseConsumption = binModel.Ptotal_av;
    end
   
    % construct the regulation signal based on the options provided
    regType = simulationOptions.regSignal(aggIdx);
    regCycles = simulationOptions.regNumCycles(aggIdx);
    regAmpl = simulationOptions.regAmpl(aggIdx); 
    controllerInformation.(aggString).Ptotal_des = ...
        generateRegulationSignal(simulationOptions, regType, regAmpl, regCycles, generalParameters, tclPopBaseConsumption);
    % keep some info in the case of mixed-zone controller
    if strcmp(controllerInformation.(aggString).Type, 'Markov controller mixed zone')
        controllerInformation.(aggString).controllerInformation_1zone.Ptotal_des = controllerInformation.(aggString).Ptotal_des; % keep the tracking signal in the single-zone struct
        controllerInformation.(aggString).controllerInformation_2zone.Ptotal_des = controllerInformation.(aggString).Ptotal_des; % keep the tracking signal in the 2-zone struct
        controllerInformation.(aggString).controllerInformation_1zone.binModel = controllerInformation.(aggString).binModel.binModel_1zone;  % keep the bin model in the individual struct because it will be needed later
        controllerInformation.(aggString).controllerInformation_2zone.binModel = controllerInformation.(aggString).binModel.binModel_2zone;  % keep the bin model in the individual struct because it will be needed later
    end 
    regulationSignalInfo.(aggString) = struct(...
        'type',simulationOptions.regSignal, ...
        'regAmpl',simulationOptions.regAmpl,...
        'regNumCycles',simulationOptions.regNumCycles,...
        'values',controllerInformation.(aggString).Ptotal_des ...
        );
end

% END SIMULATION SPECIFIC DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DO SOME SETUP BASED ON THINGS WE GENERATED USING MAIN.M

% initialize some things...
onOffValueVector = tclParameters.m0;
airTemperatureVector = tclParameters.theta_a0;
massTemperatureVector = tclParameters.theta_m0;
rng(generalParameters.rngSeed);


% create numNodes x numTcls matrix of zeros and ones to sum
% power draws using matrix multiplication. The transpose of this will map
% voltages at nodes to voltages at TCLs
tclIncidenceMatrix = zeros(size(feederParameters.houseAndNodeList,1), numel(tclParameters.acNodeAssignment));
idxOfOnes = sub2ind(size(tclIncidenceMatrix),tclParameters.acNodeAssignment, (1:numel(tclParameters.acNodeAssignment))');
tclIncidenceMatrix(idxOfOnes) = 1;

% create numNodes x numHouses matrix of zeros and ones to sum
% background power draws using matrix multiplication
houseIncidenceMatrix = zeros(size(feederParameters.houseAndNodeList,1), houseParameters.totalHouses);
idxOfOnesHouses = sub2ind(size(houseIncidenceMatrix),houseParameters.nodeConnectedTo, (1:houseParameters.totalHouses)');
houseIncidenceMatrix(idxOfOnesHouses) = 1;

%%%%%%%%%%
% set some strings that we will use within eval() calls to set and get
% variables to and from GLD

% create the string that we will evaluate to set the node voltages
% at each timestep
setNodeVoltage = '';
getNodeVoltage = 'allNodalVoltages = [';
% Set gridlabD voltages for use within onSync
for k = 1:size(feederParameters.houseAndNodeList,1)
    voltageMeasurement = 240; % just set it to nominal
    
    % set the values that would come from GLD
    setNodeVoltage = [setNodeVoltage, ...    
        'node' num2str(k) '_voltageMeasurement = ', ...
        num2str(voltageMeasurement), '; '];
    
    getNodeVoltage = [getNodeVoltage, ' node' num2str(k) '_voltageMeasurement;'];
end
getNodeVoltage = [getNodeVoltage '];'];

% create the string that we will evaluate to set the node power
% at each timestep
setNodePower = '';
setNodePowerToGLD = '';
getNodePower = 'allMeasuredNodePower = [';
for k = 1:size(feederParameters.houseAndNodeList,1) % loop through nodes
    
    % set the values that would come from GLD
    setNodePower = [setNodePower, 'node' num2str(k) ...
        '_powerMeasurement = allNodesPower(', num2str(k), '); '];
    
    setNodePowerToGLD = [setNodePowerToGLD, 'node' ...
        num2str(k) '_powerCalculation = allNodesPower(', num2str(k), '); '];
    
    getNodePower = [getNodePower ' node' num2str(k) ...
        '_powerMeasurement;'];
end
getNodePower = [getNodePower '];'];

% end make strings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make structures and strings to store grid variable information

gridData = [];
% get regulator tap and voltage
for idx = 1:feederParameters.numberOfRegulators
    gridData.(['regulator' num2str(idx)]).tapPositions = nan(3,generalParameters.numSteps);
    gridData.(['regulator' num2str(idx)]).voltages = nan(3,generalParameters.numSteps);
    gridData.(['regulator' num2str(idx)]).nominalVoltages = feederParameters.nominalVoltages.regs(idx);
end
    
% get cap bank switch value and voltage
for idx = 1:feederParameters.numberOfCapacitors
    gridData.(['capacitor' num2str(idx)]).switchPositions = nan(3,generalParameters.numSteps);
    gridData.(['capacitor' num2str(idx)]).voltages = nan(3,generalParameters.numSteps);
    gridData.(['capacitor' num2str(idx)]).nominalVoltages = feederParameters.nominalVoltages.caps(idx);
end

% get comm and split-phase load values and voltages
for idx = 1:feederParameters.numberOfCommLoads
    gridData.(['commLoad' num2str(idx)]).powerDraw= nan(3,generalParameters.numSteps);
    gridData.(['commLoad' num2str(idx)]).voltages = nan(3,generalParameters.numSteps);
    gridData.(['commLoad' num2str(idx)]).nominalVoltages = feederParameters.nominalVoltages.commLoads(idx);
end
for idx = 1:feederParameters.numberOfSplitPhaseNodes
    gridData.(['splitLoad' num2str(idx)]).powerDraw= nan(1,generalParameters.numSteps);
    gridData.(['splitLoad' num2str(idx)]).voltages = nan(1,generalParameters.numSteps);
    gridData.(['splitLoad' num2str(idx)]).nominalVoltages = feederParameters.nominalVoltages.SpLoads(idx);
end

% get transformer power flows
for idx = 1:size(feederParameters.trafoNameRatings,1)
    gridData.(['transformer' num2str(idx)]).powerFlow = nan(1,generalParameters.numSteps);
    gridData.(['transformer' num2str(idx)]).powerRating = feederParameters.trafoNameRatings{idx,2};
end

% create a string that we will evaluate to get the grid data from GLD and
% populate the gridData structure that we will store after the simulation. 
getGridData = [];

% get regulator tap and voltage
for idx = 1:feederParameters.numberOfRegulators
    getGridData = [getGridData ...
    'gridData.regulator' num2str(idx) '.tapPositions(1,n) = regulator' num2str(idx) '_tapA;' ...
    'gridData.regulator' num2str(idx) '.tapPositions(2,n) = regulator' num2str(idx) '_tapB;' ...
    'gridData.regulator' num2str(idx) '.tapPositions(3,n) = regulator' num2str(idx) '_tapC;' ...
    'gridData.regulator' num2str(idx) '.voltages(1,n) = regulator' num2str(idx) '_voltageA;' ...
    'gridData.regulator' num2str(idx) '.voltages(2,n) = regulator' num2str(idx) '_voltageB;' ...
    'gridData.regulator' num2str(idx) '.voltages(3,n) = regulator' num2str(idx) '_voltageC;' ...
    ];
end
% get capacitor switch position and voltage
for idx = 1:feederParameters.numberOfCapacitors
    getGridData = [getGridData ...
    'gridData.capacitor' num2str(idx) '.switchPositions(1,n) = cap' num2str(idx) '_switchA;' ...
    'gridData.capacitor' num2str(idx) '.switchPositions(2,n) = cap' num2str(idx) '_switchB;' ...
    'gridData.capacitor' num2str(idx) '.switchPositions(3,n) = cap' num2str(idx) '_switchC;' ...
    'gridData.capacitor' num2str(idx) '.voltages(1,n) = cap' num2str(idx) '_voltageA;' ...
    'gridData.capacitor' num2str(idx) '.voltages(2,n) = cap' num2str(idx) '_voltageB;' ...
    'gridData.capacitor' num2str(idx) '.voltages(3,n) = cap' num2str(idx) '_voltageC;' ...
    ];
end
% get comm and split-phase load values and voltages
for idx = 1:feederParameters.numberOfCommLoads
    getGridData = [getGridData ...
    'gridData.commLoad' num2str(idx) '.powerDraw(1,n) = commercialLoad' num2str(idx) '_powerA;' ...
    'gridData.commLoad' num2str(idx) '.powerDraw(2,n) = commercialLoad' num2str(idx) '_powerB;' ...
    'gridData.commLoad' num2str(idx) '.powerDraw(3,n) = commercialLoad' num2str(idx) '_powerC;' ...
    'gridData.commLoad' num2str(idx) '.voltages(1,n) = commercialLoad' num2str(idx) '_voltageA;' ...
    'gridData.commLoad' num2str(idx) '.voltages(2,n) = commercialLoad' num2str(idx) '_voltageB;' ...
    'gridData.commLoad' num2str(idx) '.voltages(3,n) = commercialLoad' num2str(idx) '_voltageC;' ...
    ];
end
for idx = 1:feederParameters.numberOfSplitPhaseNodes
    getGridData = [getGridData ...
    'gridData.splitLoad' num2str(idx) '.powerDraw(n) = node' num2str(idx) '_powerMeasurement;' ...
    'gridData.splitLoad' num2str(idx) '.voltages(n) = node' num2str(idx) '_voltageMeasurement;' ...
    ];
end

for idx = 1:size(feederParameters.trafoNameRatings,1)
    getGridData = [getGridData ...
    'gridData.transformer' num2str(idx) '.powerFlow(n) = transformer' num2str(idx) '_powerMeasurement;'...     
    ];
end


% if running a simulation without GLD, set the values that would come from
% GLD
if simulationOptions.useGLD == 0
    
    % create a string that we will evaluate to get the grid data from GLD and
    % populate the gridData structure that we will store after the simulation.
    setGridData = [];
    
    % get regulator tap and voltage
    for idx = 1:feederParameters.numberOfRegulators
        setGridData = [setGridData ...
            'regulator' num2str(idx) '_tapA = 0;' ...
            'regulator' num2str(idx) '_tapB = 0;' ...
            'regulator' num2str(idx) '_tapC = 0;' ...
            'regulator' num2str(idx) '_voltageA = 1;' ...
            'regulator' num2str(idx) '_voltageB = 1;' ...
            'regulator' num2str(idx) '_voltageC = 1;' ...
            ];
    end
    % get capacitor switch position and voltage
    for idx = 1:feederParameters.numberOfCapacitors
        setGridData = [setGridData ...
            'cap' num2str(idx) '_switchA = 0;' ...
            'cap' num2str(idx) '_switchB = 0;' ...
            'cap' num2str(idx) '_switchC = 0;' ...
            'cap' num2str(idx) '_voltageA = 1;' ...
            'cap' num2str(idx) '_voltageB = 1;' ...
            'cap' num2str(idx) '_voltageC = 1;' ...
            ];
    end
    % get comm and split-phase load values and voltages
    for idx = 1:feederParameters.numberOfCommLoads
        setGridData = [setGridData ...
            'commercialLoad' num2str(idx) '_powerA = 0;' ...
            'commercialLoad' num2str(idx) '_powerB = 0;' ...
            'commercialLoad' num2str(idx) '_powerC = 0;' ...
            'commercialLoad' num2str(idx) '_voltageA = 1;' ...
            'commercialLoad' num2str(idx) '_voltageB = 1;' ...
            'commercialLoad' num2str(idx) '_voltageC = 1;' ...
            ];
    end
    
	% set transformer power flows
    for idx = 1:size(feederParameters.trafoNameRatings,1)
        setGridData = [setGridData ...
            'transformer' num2str(idx) '_powerMeasurement = 1;' ...
            ];
    end

    
    
    eval(setGridData) % evaluate to set the data
    
end

% END SETUP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PREALLOCATE ARRAYS TO STORE DATA FOR POSTPROCESSING
timeVector = nan(generalParameters.numSteps,1);

% initialize it as an empty struct to erase anything from prior sim
simulatedTclData = struct();
simulatedTclData.airTemp  = nan(generalParameters.Pop,generalParameters.numSteps);
simulatedTclData.onOff = nan(generalParameters.Pop,generalParameters.numSteps);
simulatedTclData.massTemp = nan(generalParameters.Pop,generalParameters.numSteps); 

simulatedTclData.tclParameters = tclParameters;
simulatedTclData.m0 = tclParameters.m0;
simulatedTclData.theta_m0 = tclParameters.theta_m0;
simulatedTclData.theta_a0 = tclParameters.theta_a0;

% store the voltages
simulatedTclData.voltage240 = nan(generalParameters.Pop,generalParameters.numSteps);

simulatedTclData.realPowerDraws= nan(generalParameters.Pop,generalParameters.numSteps);
simulatedTclData.reactivePowerDraws = nan(generalParameters.Pop,generalParameters.numSteps);
simulatedTclData.timeUntilUnlocked = nan(generalParameters.Pop,generalParameters.numSteps);
simulatedTclData.timeUntilDelayEnds = nan(generalParameters.Pop,generalParameters.numSteps);

% store the control actions
% set placeholders for potential control actions
simulatedTclData.deltaDeadband = zeros(generalParameters.Pop,generalParameters.numSteps);
simulatedTclData.deltaSetpoint = zeros(generalParameters.Pop,generalParameters.numSteps);
if strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type,'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type,'Markov controller with delays') || strcmp(controllerInformation.(aggString).Type,'Markov controller mixed zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone with lockouts') %For Markov Controller, Control signal length is bin length not tcl pop
    simulatedTclData.onOffSignal = zeros(controllerInformation.agg1.controlSignalLength,generalParameters.numSteps);  % control signal received by the TCLs
    simulatedTclData.onOffSignalFromController = zeros(controllerInformation.agg1.controlSignalLength,generalParameters.numSteps); % control signal sent by the controller. May not be the same as the one received from TCLs if the communication netowrk is imperfect
else
    simulatedTclData.onOffSignal = zeros(generalParameters.Pop,generalParameters.numSteps);
    simulatedTclData.onOffSignalFromController = zeros(generalParameters.Pop,generalParameters.numSteps); % control signal sent by the controller. May not be the same as the one received from TCLs if the communication netowrk is imperfect
end
simulatedTclData.overRide = zeros(generalParameters.Pop,generalParameters.numSteps);

% store aggregate power information
simulatedTclData.totalRealPowerDemand = nan(simulationOptions.NumberOfAggregators,generalParameters.numSteps+1);
simulatedTclData.totalReactivePowerDemand = nan(simulationOptions.NumberOfAggregators,generalParameters.numSteps+1);
simulatedTclData.totalComplexPowerDemand = nan(simulationOptions.NumberOfAggregators,generalParameters.numSteps+1);

% initialize aggregate values
for aggIdx = 1:simulationOptions.NumberOfAggregators
    aggString = ['agg' num2str(aggIdx)]';    
    tclsThisAgg = controllerInformation.(aggString).tclsAssigned;
    
    P_power_draw = onOffValueVector(tclsThisAgg)'*simulatedTclData.tclParameters.P_power_draw(tclsThisAgg);
    Q_power_draw = onOffValueVector(tclsThisAgg)'*simulatedTclData.tclParameters.Q_power_draw(tclsThisAgg);
    S_power_draw = P_power_draw + 1j * Q_power_draw;
    
    simulatedTclData.totalRealPowerDemand(aggIdx,1) = P_power_draw;
    simulatedTclData.totalReactivePowerDemand(aggIdx,1) = Q_power_draw;
    simulatedTclData.totalComplexPowerDemand(aggIdx,1) = S_power_draw;
end

%Pre-allocating placeholder for controlactions received after impact of
    %communication network
    controlActionsReceived = zeros(generalParameters.Pop,generalParameters.numSteps);
    
% END PREALLOCATE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DO COMM NETWORK PREPROCESSING

% % modified this to preprocess everything. Matrices for incidence and
% % connectivity are generated, all dropouts and delays are generated, given
% % the number of time-steps in the simulation, and so on. 
if simulationOptions.considerCommNetwork == 1
    % modified to generate connectivity and indicidence matrices here and
    % only once. Generates all random values in one big draw versus at
    % every time-step
    [CM,DDM,MDM, SDM, distribution, IM] = CommNetworkCharacteristics(generalParameters.Pop,simulationOptions.nc, generalParameters.numSteps, simulationOptions.commNetwork,generalParameters.timeStepInSec);
else
    CM = [];
    DDM = [];
    MDM = [];
    SDM = [];
    distribution = [];
    IM = [];
end
commNetworkData.CM = CM;
commNetworkData.DDM = DDM;
commNetworkData.MDM = MDM;
commNetworkData.SDM = SDM;
commNetworkData.distribution = distribution;
commNetworkData.IM = IM;
commNetworkData.nc = simulationOptions.nc;
commNetworkData.scenario = simulationOptions.commNetwork; 
if commNetworkData.scenario == 3 && (strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type,'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type,'Markov controller with delays') || strcmp(controllerInformation.(aggString).Type,'Markov controller mixed zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone with lockouts')) %For Markov Controller, Control signal length is bin length not tcl pop
    commNetworkData.controlActionsFromController = cell(generalParameters.Pop,generalParameters.numSteps+1);
    commNetworkData.controlActionsFromController(:,:) = {zeros(1,controllerInformation.agg1.controlSignalLength)};
    commNetworkData.controlActionsToTcls = cell(generalParameters.Pop,generalParameters.numSteps+1);
    commNetworkData.controlActionsToTcls(:,:) = {zeros(1,controllerInformation.agg1.controlSignalLength)};
elseif commNetworkData.scenario == 1 && (strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type,'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type,'Markov controller with delays') || strcmp(controllerInformation.(aggString).Type,'Markov controller mixed zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone with lockouts'))
    commNetworkData.controlActionsFromController = zeros(controllerInformation.agg1.controlSignalLength, generalParameters.numSteps+1);
    commNetworkData.controlActionsToTcls = zeros(controllerInformation.agg1.controlSignalLength, generalParameters.numSteps+1);
else
    commNetworkData.controlActionsFromController = zeros(generalParameters.Pop, generalParameters.numSteps+1);
    commNetworkData.controlActionsToTcls = zeros(generalParameters.Pop, generalParameters.numSteps+1);
end
% END COMM NETWORK PREPROCESSING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SETUP FOR ITERATION AT EACH TIMESTEP

% set placeholders for potential control actions
controlActionsWithComms.deltaDeadband = zeros(generalParameters.Pop,1);
controlActionsWithComms.deltaSetpoint = zeros(generalParameters.Pop,1);
if commNetworkData.scenario == 1 && (strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type,'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type,'Markov controller with delays')) || strcmp(controllerInformation.(aggString).Type,'Markov controller mixed zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone with lockouts')
    controlActionsWithComms.onOffSignal = zeros(controllerInformation.agg1.controlSignalLength,1);
else
    controlActionsWithComms.onOffSignal = zeros(generalParameters.Pop,1);
end
% set 0 = no override, 1 = override off, 2 = override on
controlActionsWithComms.overRide = zeros(generalParameters.Pop,1);
% initialize some fields needed for the mixed zone controller
if generalParameters.perc_2zone < 1 && generalParameters.perc_2zone > 0
    controlActionsWithComms.onOffSignal_1zone = [];
    controlActionsWithComms.onOffSignal_2zone = [];
    controlActionsFromController.onOffSignal_1zone = zeros(size(controllerInformation.agg1.controllerInformation_1zone.u_rel_mat,1), 1);
    controlActionsFromController.onOffSignal_2zone = zeros(controllerInformation.agg1.controlSignalLength_2zone, 1);
elseif generalParameters.perc_2zone == 0 && (strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type, 'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type, 'Markov controller with delays'))
    % need to initialize u, which is different than onOffSignal for the Markov controller
    controlActionsWithComms.u = [];
end

% set controller time-step (which could be different from
% simulation time-step)
% set things for n = 0
controllerInformation.(['agg' num2str(aggIdx)]).timeStep = 0;
controllerInformation.(['agg' num2str(aggIdx)]).timeStepSim = 0;
controlNumSteps = controllerInformation.controlTimeStep./generalParameters.timeStepInSec;
[controllerInformation, simulatedTclData, tclParameters] = ...
    updateStructures(0, controlNumSteps,...
    controllerInformation,tclParameters,simulatedTclData, ...
    airTemperatureVector,onOffValueVector,massTemperatureVector,outdoorTemperature,irradiance,controlActionsWithComms);

% store the state of the tcls
simulatedTclData.airTemp(:,1) = airTemperatureVector;
simulatedTclData.onOff(:,1) = onOffValueVector;
simulatedTclData.massTemp(:,1) = massTemperatureVector;
simulatedTclData.realPowerDraws(:,1) =  onOffValueVector.*tclParameters.P_power_draw ;
simulatedTclData.reactivePowerDraws(:,1) =  onOffValueVector.*tclParameters.Q_power_draw ;
simulatedTclData.timeUntilUnlocked(:,1) =  tclParameters.timeUntilUnlocked;
simulatedTclData.timeUntilDelayEnds(:,1) = tclParameters.timeUntilDelayEnds;
simulatedTclData.voltage240(:,1) = tclParameters.voltage240;
% store control actions
simulatedTclData.deltaDeadband(:,1) = controlActionsWithComms.deltaDeadband;
simulatedTclData.deltaSetpoint(:,1) = controlActionsWithComms.deltaSetpoint;
if commNetworkData.scenario == 3 && (strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type,'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type,'Markov controller with delays') || strcmp(controllerInformation.(aggString).Type,'Markov controller mixed zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone with lockouts')) %For Markov Controller, Control signal length is bin length not tcl pop
    simulatedTclData.onOffSignal(:,1) = (commNetworkData.controlActionsFromController{1,1})';
    simulatedTclData.onOffSignalFromController(:,1) = (commNetworkData.controlActionsFromController{1,1})';
else
    simulatedTclData.onOffSignal(:,1) = controlActionsWithComms.onOffSignal;
    simulatedTclData.onOffSignalFromController(:,1) = controlActionsWithComms.onOffSignal;
end
simulatedTclData.overRide(:,1) = controlActionsWithComms.overRide;

timeStepAndIterationCounter = [];
n = 1; % Initialize time step

%PEM Initialization - Initial TCL requests
locked = tclParameters.timeUntilUnlocked > 0;
if strcmp(controllerInformation.(aggString).Type, 'PEM_E-T controller')
    [onRequests, offRequests, controllerInformation.(aggString)] = Extended_PEM(airTemperatureVector,controllerInformation.(aggString).T_min,controllerInformation.(aggString).T_max,controllerInformation.(aggString).MTTR,controllerInformation.(aggString).MTTRoff,generalParameters.timeStepInSec,onOffValueVector,locked,controllerInformation.(aggString).tclsAssigned,n,controllerInformation.(aggString));
end

simTimer = tic;
%sim start time
sim_start_time = datetime;
% print that event started 
disp(append("Started simulation at ", string(sim_start_time)))

% code onSync will execute now based on the time-step and information 
% sent from gld