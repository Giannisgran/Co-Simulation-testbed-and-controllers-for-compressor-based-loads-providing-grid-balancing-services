% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function parameterSweepFeeder = generateFeederParameterSweepArrays(feederOptions)
% function that takes the set of feeder options, and creates arrays for
% each of them that allow each option to be set while automatically
% constructing the feeders. Outputs the parameter arrays that can be
% indexed by a for loop to construct feeders in batch.

% get set of options
setOfTaxFiles = feederOptions.setOfTaxFiles;
percentTclOptionSet = feederOptions.percentTclOptionSet;
feederCapacityScalingOptionSet = feederOptions.feederCapacityScalingOptionSet;
backgroundDemandScalingFactorOptionSet = feederOptions.backgroundDemandScalingFactorOptionSet;
voltageRegulatorOptionSet = feederOptions.voltageRegulatorOptionSet;


% find number of values in each parameter
num1 = numel(setOfTaxFiles);
num2 = numel(percentTclOptionSet);
num3 = numel(feederCapacityScalingOptionSet);
num4 = numel(backgroundDemandScalingFactorOptionSet);
num5 = numel(voltageRegulatorOptionSet); 

% get total number of cases
numCases = num1 * num2 * num3 * num4 * num5;

% initialize arrays to hold values
percentTclSweepValues = zeros(numCases,1);
feederCapacitySweepValues = zeros(numCases,1);
feederValues = cell(numCases,1);
backgroundDemandScalingFactor = zeros(numCases,1);
voltageRegulatorOptions = zeros(numCases,1);

% counter for indexing
caseNum =1;

% nested loops to loop through the different values and set the values
for ind1 = 1:num1 % feeder
for ind2 = 1:num2 % percent TCL
for ind3 = 1:num3 % feeder capacity
for ind4 = 1:num4
for ind5 = 1:num5
    
   feederValues{caseNum} =  setOfTaxFiles{ind1};
   percentTclSweepValues(caseNum) = percentTclOptionSet(ind2);
   feederCapacitySweepValues(caseNum) = feederCapacityScalingOptionSet(ind3);
   backgroundDemandScalingFactor(caseNum) = backgroundDemandScalingFactorOptionSet(ind4);
   voltageRegulatorOptions(caseNum) = voltageRegulatorOptionSet(ind5);
   % iterate counter
   caseNum = caseNum + 1;

end
end
end
end
end

% set output structure
parameterSweepFeeder.feederValues = feederValues;
parameterSweepFeeder.percentTcl = percentTclSweepValues;
parameterSweepFeeder.feederCapacity = feederCapacitySweepValues;
parameterSweepFeeder.backgroundDemandScalingFactor = backgroundDemandScalingFactor;
parameterSweepFeeder.voltageRegulatorSettings = voltageRegulatorOptions;
end
