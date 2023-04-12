% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function parameterSweepTcls = generateTclParameterSweepArrays(tclOptions)

% calls either the exhaustive set design or the two-factor design based on
% the option. Both functions are anonymous functions below. 
if tclOptions.useTwoFactorCaseDesign == 1
    parameterSweepTcls = doTwoFactorDesign(tclOptions);
elseif tclOptions.useTwoFactorCaseDesign == 0
    parameterSweepTcls = doExhaustiveDesign(tclOptions);
else
   error('Unknown parameter setting used ') 
end

end

function     parameterSweepTcls = doTwoFactorDesign(tclOptions)

% get different paramter sets
feederValues = tclOptions.setOfTaxFiles;
weatherValues = tclOptions.weatherSet; % [-]
buildingValues = tclOptions.buildingParameterSet; % [-]
heterogeneityValues = tclOptions.tclHeterogeneitySet; %[-]
onLockoutValues = tclOptions.lockoutTimeOnSet; % [sec]
offLockoutValues = tclOptions.lockoutTimeOffSet; % [min]

% only find the number of settings for the first feeder, since they just
% get repeated for the other
feeder = feederValues{1};
feeder2 = strrep(feeder,'.glm','');
feeder3 = strrep(feeder2,'.','');
feeder4 = strrep(feeder3,'-','_');

% load feeder data for this feeder
fileList = dir(['./' feeder4 '/feederData.mat']);
load(['./' feeder4 '/' fileList.name],'feederData')

% gets the field names of the structure, which contains all the
% iterations of the feeder we're interested in
thisFeederFieldNames = fieldnames(feederData);

VregSetting = zeros(numel(thisFeederFieldNames), 1);
for idx = 1:numel(thisFeederFieldNames)
    settings = regexp(thisFeederFieldNames{idx},'\d*','Match');
    VregSetting(idx) = str2double(settings{1});
end
   
numCases = numel(feederValues) * 16; % 16 variations for each feeder. 

% initialize arrays to hold values
feederSweepValues = cell(numCases,1);
feederDirectorySweepValues = cell(numCases,1);
weatherSweepValues = zeros(numCases,1);
buildingSweepValues = zeros(numCases,1);
heterogeneitySweepValues = zeros(numCases,1);
onLockoutSweepValues = zeros(numCases,1);
offLockoutSweepValues = zeros(numCases,1);
vRegValues = zeros(numCases,1);

for feeder = 1:numel(feederValues)
% Column definitions) 1: ISO Signal type;	2: ISO Signal amplitude;	
% 3: Weather; 4: Building Parameters; 5: Background Demand; 
% 6: Comms Network; 7: TCL Heterogeneity

% Case definitions 

feederIterationValues = {};
feederDirectoryValues = {};

feederTemp = feederValues{feeder};
feeder2 = strrep(feederTemp,'.glm','');
feeder3 = strrep(feeder2,'.','');
feeder4 = strrep(feeder3,'-','_');

% load feeder data for this feeder
fileList = dir(['./' feeder4 '/feederData.mat']);
load(['./' feeder4 '/' fileList.name],'feederData')

% gets the field names of the structure, which contains all the
% iterations of the feeder we're interested in
thisFeederFieldNames = fieldnames(feederData);

% store the feeder iterations
feederIterationValues = [feederIterationValues; thisFeederFieldNames];

thisDirectory = {['./' feeder4 '/']};
feederDirectoryValues = [feederDirectoryValues; ...
    repmat(thisDirectory, numel(thisFeederFieldNames), 1 )];


% set a value that lets us set cases based on the feeder we are looking at
feederIdx = (feeder-1)*16;

% Base case: All "-1" values
caseIdx = 1;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 1, 5, 7
% 1: ISO Signal type; 5: Background Demand; 7: TCL Heterogeneity
caseIdx = 2;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);


% "+1" values for column 2, 5, 6
% 2: ISO Signal amplitude; 5: Background Demand; 6: Comms Network;
caseIdx = 3;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);



% "+1" values for column 1, 2, 6, 7
% 1: ISO Signal type; 2: ISO Signal amplitude; 6: Comms Network; 
% 7: TCL Heterogeneity
caseIdx = 4;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 3, 5, 6, 7
% 3: Weather; 5: Background Demand; 
% 6: Comms Network; 7: TCL Heterogeneity
caseIdx = 5;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 1, 3, 6
% 1: ISO Signal type; 3: Weather; 6: Comms Network;
caseIdx = 6;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 2, 3, 7
% 2: ISO Signal amplitude; 3: Weather; 7: TCL Heterogeneity
caseIdx = 7;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);


% "+1" values for column 1, 2, 3, 5
% 1: ISO Signal type;	2: ISO Signal amplitude; 3: Weather; 
% 5: Background Demand; 
caseIdx = 8;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(1);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 4, 6, 7
% 4: Building Parameters; 6: Comms Network; 7: TCL Heterogeneity
caseIdx = 9;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 1, 4, 5, 6
% 1: ISO Signal type; 4: Building Parameters; 5: Background Demand; 
% 6: Comms Network;
caseIdx = 10;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 2, 4, 5, 7
% 2: ISO Signal amplitude; 4: Building Parameters; 5: Background Demand; 
% 7: TCL Heterogeneity
caseIdx = 11;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 1, 2, 4
% 1: ISO Signal type;	2: ISO Signal amplitude; 4: Building Parameters;
caseIdx = 12;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(1);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 3, 4, 5
% 3: Weather; 4: Building Parameters; 5: Background Demand; 
caseIdx = 13;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 1, 3, 4, 7
% 1: ISO Signal type; 3: Weather; 4: Building Parameters; 
% 7: TCL Heterogeneity
caseIdx = 14;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% "+1" values for column 2, 3, 4, 6
% 2: ISO Signal amplitude;	
% 3: Weather; 4: Building Parameters; 6: Comms Network
caseIdx = 15;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{1};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{1};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(1);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

% All "+1" values
caseIdx = 16;
feederSweepValues{feederIdx+caseIdx} = feederIterationValues{2};
feederDirectorySweepValues{feederIdx+caseIdx} = feederDirectoryValues{2};
weatherSweepValues(feederIdx+caseIdx) = weatherValues(2);
buildingSweepValues(feederIdx+caseIdx) = buildingValues(2);
heterogeneitySweepValues(feederIdx+caseIdx) = heterogeneityValues(2);
onLockoutSweepValues(feederIdx+caseIdx) = onLockoutValues(1);
offLockoutSweepValues(feederIdx+caseIdx) = offLockoutValues(1);

end

% populate arrays
parameterSweepTcls.feederSweepValues = feederSweepValues;
parameterSweepTcls.feederDirectorySweepValues = feederDirectorySweepValues;
parameterSweepTcls.weatherSweepValues = weatherSweepValues;
parameterSweepTcls.buildingSweepValues = buildingSweepValues;
parameterSweepTcls.heterogeneitySweepValues = heterogeneitySweepValues;
parameterSweepTcls.onLockoutSweepValues = onLockoutSweepValues;
parameterSweepTcls.offLockoutSweepValues = offLockoutSweepValues;


end



function parameterSweepTcls = doExhaustiveDesign(tclOptions)
% get different paramter sets
feederValues = tclOptions.setOfTaxFiles;
weatherValues = tclOptions.weatherSet; % [-]
buildingValues = tclOptions.buildingParameterSet; % [-]
heterogeneityValues = tclOptions.tclHeterogeneitySet; %[-]
onLockoutValues = tclOptions.lockoutTimeOnSet; % [sec]
offLockoutValues = tclOptions.lockoutTimeOffSet; % [min]

% get number of parameters in each set
num1 = numel(feederValues);
num2 = numel(weatherValues);
num3 = numel(buildingValues);
num4 = numel(heterogeneityValues);
num5 = numel(onLockoutValues);

feederIterationValues = {};
feederDirectoryValues = {};
for idx1 = 1:num1
   feeder = feederValues{idx1}; 
   feeder2 = strrep(feeder,'.glm','');
   feeder3 = strrep(feeder2,'.','');
   feeder4 = strrep(feeder3,'-','_');
   
   % load feeder data for this feeder
   fileList = dir(['./' feeder4 '/feederData.mat']);
   load(['./' feeder4 '/' fileList.name],'feederData')
   
   % gets the field names of the structure, which contains all the
   % iterations of the feeder we're interested in
   thisFeederFieldNames = fieldnames(feederData);
   
   % store the feeder iterations
   feederIterationValues = [feederIterationValues; thisFeederFieldNames];

   thisDirectory = {['./' feeder4 '/']};
   feederDirectoryValues = [feederDirectoryValues; ...
       repmat(thisDirectory, numel(thisFeederFieldNames), 1 )];
end

% calculate number of feeder iterations based on above
numFeederVersions = numel(feederIterationValues);


% get total number of cases
numCases = numFeederVersions * num2 * num3 * num4 * num5;

% initialize arrays to hold values
feederSweepValues = cell(numCases,1);
feederDirectorySweepValues = cell(numCases,1);
weatherSweepValues = zeros(numCases,1);
buildingSweepValues = zeros(numCases,1);
heterogeneitySweepValues = zeros(numCases,1);
onLockoutSweepValues = zeros(numCases,1);
offLockoutSweepValues = zeros(numCases,1);

% counter for indexing
caseNum =1;

% loop through different parameters to define a list of parameter options
% for each case
for idx1 = 1:numFeederVersions
for idx2 = 1:num2
for idx3 = 1:num3
for idx4 = 1:num4
for idx5 = 1:num5
    
    % populate arrays
    feederSweepValues{caseNum} = feederIterationValues{idx1};
    feederDirectorySweepValues{caseNum} = feederDirectoryValues{idx1};
    weatherSweepValues(caseNum) = weatherValues(idx2);
    buildingSweepValues(caseNum) = buildingValues(idx3);
    heterogeneitySweepValues(caseNum) = heterogeneityValues(idx4);
    onLockoutSweepValues(caseNum) = onLockoutValues(idx5);
    offLockoutSweepValues(caseNum) = offLockoutValues(idx5);

    % iterate counter
    caseNum = caseNum + 1;
end
end
end
end 
end

% populate arrays
parameterSweepTcls.feederSweepValues = feederSweepValues;
parameterSweepTcls.feederDirectorySweepValues = feederDirectorySweepValues;
parameterSweepTcls.weatherSweepValues = weatherSweepValues;
parameterSweepTcls.buildingSweepValues = buildingSweepValues;
parameterSweepTcls.heterogeneitySweepValues = heterogeneitySweepValues;
parameterSweepTcls.onLockoutSweepValues = onLockoutSweepValues;
parameterSweepTcls.offLockoutSweepValues = offLockoutSweepValues;



end
