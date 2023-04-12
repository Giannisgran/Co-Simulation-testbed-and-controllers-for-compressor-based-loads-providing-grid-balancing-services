% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function parameterSweepSimulation = generateSimulationParameterSweepArrays(simulationOptionSet)
% calls either the exhaustive set design or the two-factor design based on
% the option. Both functions are anonymous functions below. 
if simulationOptionSet.useTwoFactorCaseDesign == 1
    parameterSweepSimulation = doTwoFactorDesign(simulationOptionSet);
elseif simulationOptionSet.useTwoFactorCaseDesign == 0
    parameterSweepSimulation = doExhaustiveDesign(simulationOptionSet);
else
   error('Unknown parameter setting used ') 
end

end

function parameterSweepSimulation = doTwoFactorDesign(simulationOptionSet)

% get the values from the settings
regulationSignalType = simulationOptionSet.regulationSignalType;
regulationSignalNumCycles = simulationOptionSet.regulationSignalNumCycles;
regulationSignalAmplitude = simulationOptionSet.regulationSignalAmplitude;
percentTclOptionSet2 = simulationOptionSet.percentTclOptionSet; % no units
feederCapacityScalingOptionSet2 = simulationOptionSet.feederCapacityScalingOptionSet; % no units
backgroundDemandScalingFactorOptionSet2 = simulationOptionSet.backgroundDemandScalingFactorOptionSet;
setOfTaxFiles = simulationOptionSet.setOfTaxFiles;
weatherSet2 = simulationOptionSet.weatherSet; % [-]
buildingParameterSet2 = simulationOptionSet.buildingParameterSet; % [-]
tclHeterogeneitySet2 = simulationOptionSet.tclHeterogeneitySet; %[-]
lockoutTimeOnSet2 = simulationOptionSet.lockoutTimeOnSet; % [sec]
lockoutTimeOffSet2 = simulationOptionSet.lockoutTimeOffSet; % [min]
commNetworkSet = simulationOptionSet.commScenario; 
voltageRegSettings = simulationOptionSet.voltageRegulatorSetting;


% only find the number of settings for the first feeder, since they just
% get repeated for the other
feeder = setOfTaxFiles{1};
feeder2 = strrep(feeder,'.glm','');
feeder3 = strrep(feeder2,'.','');
feeder4 = strrep(feeder3,'-','_');

% load feeder data for this feeder
fileList = dir(['./' feeder4 '/feederData.mat']);
load(['./' feeder4 '/' fileList.name],'feederData')

% gets the field names of the structure, which contains all the
% iterations of the feeder we're interested in
thisFeederFieldNames = fieldnames(feederData);

scalingSetting = zeros(numel(thisFeederFieldNames), 1);
for idx = 1:numel(thisFeederFieldNames)
    settings = regexp(thisFeederFieldNames{idx},'\d*','Match');
    scalingSetting(idx) = str2double(settings{end});
end

% 16 variations for each feeder. 
numCases = numel(setOfTaxFiles) * 16;
% get the number of aggregators
numAggs = size(regulationSignalNumCycles,1);


% initialize arrays to hold values
regulationSignalTypeValues = zeros(numCases,numAggs);
regulationSignalNumCyclesValues = zeros(numCases,numAggs);
regulationSignalAmplitudeValues = zeros(numCases,numAggs);
percentTclOptionSet = zeros(numCases,1);
feederCapacityScalingOptionSet = zeros(numCases,1);
backgroundDemandScalingFactorOptionSet = zeros(numCases,1);
setOfTaxFilesOut = cell(numCases,1);
weatherSet = zeros(numCases,1);
buildingParameterSet = zeros(numCases,1);
tclHeterogeneitySet = zeros(numCases,1);
lockoutTimeOnSet = zeros(numCases,1);
lockoutTimeOffSet = zeros(numCases,1);
CommNetwork = zeros(numCases,1); 
voltageRegSet = zeros(numCases,1);


for feeder = 1:numel(setOfTaxFiles)
% Column definitions) 1: ISO Signal type;	2: ISO Signal amplitude;	
% 3: Weather; 4: Building Parameters; 5: Voltage setting; 
% 6: Comms Network; 7: TCL Heterogeneity

% Case definitions 

% set a value that lets us set cases based on the feeder we are looking at
feederIdx = (feeder-1)*16;

% Base case: All "-1" values
caseIdx = 1;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 1, 5, 7
% 1: ISO Signal type; 5: Voltage Setting; 7: TCL Heterogeneity
caseIdx = 2;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 2, 5, 6
% 2: ISO Signal amplitude; 5: Voltage Setting; 6: Comms Network;
caseIdx = 3;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

% "+1" values for column 1, 2, 6, 7
% 1: ISO Signal type; 2: ISO Signal amplitude; 6: Comms Network; 
% 7: TCL Heterogeneity
caseIdx = 4;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

% "+1" values for column 3, 5, 6, 7
% 3: Weather; 5: Voltage Setting; 
% 6: Comms Network; 7: TCL Heterogeneity
caseIdx = 5;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

% "+1" values for column 1, 3, 6
% 1: ISO Signal type; 3: Weather; 6: Comms Network;
caseIdx = 6;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

% "+1" values for column 2, 3, 7
% 2: ISO Signal amplitude; 3: Weather; 7: TCL Heterogeneity
caseIdx = 7;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 1, 2, 3, 5
% 1: ISO Signal type;	2: ISO Signal amplitude; 3: Weather; 
% 5: Voltage Setting; 
caseIdx = 8;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(1);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 4, 6, 7
% 4: Building Parameters; 6: Comms Network; 7: TCL Heterogeneity
caseIdx = 9;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

% "+1" values for column 1, 4, 5, 6
% 1: ISO Signal type; 4: Building Parameters; 5: Voltage Setting; 
% 6: Comms Network;
caseIdx = 10;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

% "+1" values for column 2, 4, 5, 7
% 2: ISO Signal amplitude; 4: Building Parameters; 5: Voltage Setting; 
% 7: TCL Heterogeneity
caseIdx = 11;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 1, 2, 4
% 1: ISO Signal type;	2: ISO Signal amplitude; 4: Building Parameters;
caseIdx = 12;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(1);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 3, 4, 5
% 3: Weather; 4: Building Parameters; 5: Voltage Setting; 
caseIdx = 13;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 1, 3, 4, 7
% 1: ISO Signal type; 3: Weather; 4: Building Parameters; 
% 7: TCL Heterogeneity
caseIdx = 14;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,1);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(1); 

% "+1" values for column 2, 3, 4, 6
% 2: ISO Signal amplitude;	
% 3: Weather; 4: Building Parameters; 6: Comms Network
caseIdx = 15;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,1);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(1);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(1);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

% All "+1" values
caseIdx = 16;
regulationSignalTypeValues(feederIdx+caseIdx,:) = regulationSignalType(:,2);
regulationSignalNumCyclesValues(feederIdx+caseIdx,:) = regulationSignalNumCycles(:,1);
regulationSignalAmplitudeValues(feederIdx+caseIdx,:) = regulationSignalAmplitude(:,2);
percentTclOptionSet(feederIdx+caseIdx) = percentTclOptionSet2(1);
feederCapacityScalingOptionSet(feederIdx+caseIdx) = feederCapacityScalingOptionSet2(1);
backgroundDemandScalingFactorOptionSet(feederIdx+caseIdx) = backgroundDemandScalingFactorOptionSet2(1);
voltageRegSet(feederIdx+caseIdx) = voltageRegSettings(2);
setOfTaxFilesOut{feederIdx+caseIdx} = setOfTaxFiles{feeder};
weatherSet(feederIdx+caseIdx) = weatherSet2(2);
buildingParameterSet(feederIdx+caseIdx) = buildingParameterSet2(2);
tclHeterogeneitySet(feederIdx+caseIdx) = tclHeterogeneitySet2(2);
lockoutTimeOnSet(feederIdx+caseIdx) = lockoutTimeOnSet2(1);
lockoutTimeOffSet(feederIdx+caseIdx) = lockoutTimeOffSet2(1);
CommNetwork(feederIdx+caseIdx) = commNetworkSet(2); 

end


% populate arrays
parameterSweepSimulation.regulationSignalTypeValues = regulationSignalTypeValues;
parameterSweepSimulation.regulationSignalNumCyclesValues = regulationSignalNumCyclesValues;
parameterSweepSimulation.regulationSignalAmplitudeValues = regulationSignalAmplitudeValues;
parameterSweepSimulation.percentTclOptionSet = percentTclOptionSet;
parameterSweepSimulation.feederCapacityScalingOptionSet = feederCapacityScalingOptionSet;
parameterSweepSimulation.backgroundDemandScalingFactorOptionSet = backgroundDemandScalingFactorOptionSet;
parameterSweepSimulation.setOfTaxFiles = setOfTaxFilesOut;
parameterSweepSimulation.weatherSet = weatherSet;
parameterSweepSimulation.buildingParameterSet = buildingParameterSet;
parameterSweepSimulation.tclHeterogeneitySet = tclHeterogeneitySet;
parameterSweepSimulation.lockoutTimeOnSet = lockoutTimeOnSet;
parameterSweepSimulation.lockoutTimeOffSet = lockoutTimeOffSet;
parameterSweepSimulation.CommNetwork = CommNetwork; 
parameterSweepSimulation.voltageRegSet = voltageRegSet;


end

function parameterSweepSimulation = doExhaustiveDesign(simulationOptionSet)

regulationSignalType = simulationOptionSet.regulationSignalType;
regulationSignalNumCycles = simulationOptionSet.regulationSignalNumCycles;
regulationSignalAmplitude = simulationOptionSet.regulationSignalAmplitude;
percentTclOptionSet = simulationOptionSet.percentTclOptionSet; % no units
feederCapacityScalingOptionSet = simulationOptionSet.feederCapacityScalingOptionSet; % no units
backgroundDemandScalingFactorOptionSet = simulationOptionSet.backgroundDemandScalingFactorOptionSet;
setOfTaxFiles = simulationOptionSet.setOfTaxFiles;
weatherSet = simulationOptionSet.weatherSet; % [-]
buildingParameterSet= simulationOptionSet.buildingParameterSet; % [-]
tclHeterogeneitySet = simulationOptionSet.tclHeterogeneitySet; %[-]
lockoutTimeOnSet= simulationOptionSet.lockoutTimeOnSet; % [sec]
lockoutTimeOffSet= simulationOptionSet.lockoutTimeOffSet; % [min]
commNetworkSet = simulationOptionSet.commScenario; 
voltageRegSet = simulationOptionSet.voltageRegulatorSetting;


% get the number of aggregators
numAggs = size(regulationSignalNumCycles,1);

% get number of parameters in each set
num(1) = size(regulationSignalType,2);
num(2) = size(regulationSignalNumCycles,2);
num(3) = size(regulationSignalAmplitude,2);
num(4) = numel(percentTclOptionSet);
num(5) = numel(feederCapacityScalingOptionSet);
num(6) = numel(backgroundDemandScalingFactorOptionSet);
num(7) = numel(setOfTaxFiles);
num(8) = numel(weatherSet);
num(9) = numel(buildingParameterSet);
num(10) = numel(tclHeterogeneitySet);
num(11) = numel(lockoutTimeOnSet);
num(12) = numel(commNetworkSet);
num(13) = numel(voltageRegSet);

% get total number of cases
numCases = prod(num);

% initialize arrays to hold values
parameterSweepSimulation.regulationSignalTypeValues = zeros(numCases,numAggs);
parameterSweepSimulation.regulationSignalNumCyclesValues = zeros(numCases,numAggs);
parameterSweepSimulation.regulationSignalAmplitudeValues = zeros(numCases,numAggs);
parameterSweepSimulation.percentTclOptionSet = zeros(numCases,1);
parameterSweepSimulation.feederCapacityScalingOptionSet = zeros(numCases,1);
parameterSweepSimulation.backgroundDemandScalingFactorOptionSet = zeros(numCases,1);
parameterSweepSimulation.setOfTaxFiles = cell(numCases,1);
parameterSweepSimulation.weatherSet = zeros(numCases,1);
parameterSweepSimulation.buildingParameterSet = zeros(numCases,1);
parameterSweepSimulation.tclHeterogeneitySet = zeros(numCases,1);
parameterSweepSimulation.lockoutTimeOnSet = zeros(numCases,1);
parameterSweepSimulation.lockoutTimeOffSet = zeros(numCases,1);
parameterSweepSimulation.CommNetwork = zeros(numCases,1); 
parameterSweepSimulation.voltageRegSet = zeros(numCases,1);
% counter for indexing
caseNum =1;

% loop through different parameters to define a list of parameter options
% for each case
for idx1 = 1:num(1)
for idx2 = 1:num(2)
for idx3 = 1:num(3)
for idx4 = 1:num(4)
for idx5 = 1:num(5)
for idx6 = 1:num(6)
for idx7 = 1:num(7)
for idx8 = 1:num(8)
for idx9 = 1:num(9)
for idx10 = 1:num(10)
for idx11 = 1:num(11)
for idx12 = 1:num(12)
for idx13 = 1:num(13)
    
% populate arrays
parameterSweepSimulation.regulationSignalTypeValues(caseNum,:) = regulationSignalType(:,idx1);
parameterSweepSimulation.regulationSignalNumCyclesValues(caseNum,:) = regulationSignalNumCycles(:,idx2);
parameterSweepSimulation.regulationSignalAmplitudeValues(caseNum,:) = regulationSignalAmplitude(:,idx3);
parameterSweepSimulation.percentTclOptionSet(caseNum) = percentTclOptionSet(idx4);
parameterSweepSimulation.feederCapacityScalingOptionSet(caseNum) = feederCapacityScalingOptionSet(idx5);
parameterSweepSimulation.backgroundDemandScalingFactorOptionSet(caseNum) = backgroundDemandScalingFactorOptionSet(idx6);
parameterSweepSimulation.setOfTaxFiles{caseNum} = setOfTaxFiles{idx7};
parameterSweepSimulation.weatherSet(caseNum) = weatherSet(idx8);
parameterSweepSimulation.buildingParameterSet(caseNum) = buildingParameterSet(idx9);
parameterSweepSimulation.tclHeterogeneitySet(caseNum) = tclHeterogeneitySet(idx10);
parameterSweepSimulation.lockoutTimeOnSet(caseNum) = lockoutTimeOnSet(idx11);
parameterSweepSimulation.lockoutTimeOffSet(caseNum) = lockoutTimeOffSet(idx11);
parameterSweepSimulation.CommNetwork(caseNum) = commNetworkSet(idx12); 
parameterSweepSimulation.voltageRegSet(caseNum) = voltageRegSet(idx13);
    % iterate counter
    caseNum = caseNum + 1;

end
end
end
end
end
end
end
end
end
end
end
end
end

end 
