% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script will be used to do any pre-processing that is needed before
% running actual simulations. This includes parameterizing the TCLs,
% initializing the TCLs, etc. You need to be in the
% directory where the m file is located to run it properly. 

% Inputs : Feeder data and options structures created and saved during
%           feeder creation; options defined below

% Outputs : Initialized TCL data; selected weather data

% In this script, the parameters that we're interested in varying are
% listed below. Two-factor case construction is an option here, and details
% are given below. Note that you also need to select the relevent feeder(s)
% that you want to construct cases for as you did in the pre1_... file.
%   Parameters: 
%       simulationOptionSet.weatherSet = [1, 2]; % [-]
%       simulationOptionSet.buildingParameterSet = [1, 2]; % [-]
%       simulationOptionSet.tclHeterogeneitySet = [1, 2]; %[-]
%   In the above, weatherSet sets the outdoor temperature for the 
%   simulation (1 = 90degF; 2=100degF), buildingParameterSet 
%   sets the thermal parameters
%   of the TCL population (1=normal parameters; 2 = half the thermal
%   capacity of case 1, meaning faster temperature evolution in the
%   houses), tclHeterogeneitySet controls the spread of parameters for the
%   TCLs (1 = normal heterogeneity; 2 = a homogeneous population). 
%
%   To run two-factor case study setup, the seven parameters that we can
%   change above each need to have two options. It's also possible to
%   generate single cases or exhaustive conbinations of all parameters by
%   setting the two-factor flag to 0. 

fprintf('Copyright (C) 2023 The Regents of the University of Michigan\n')
clearvars; clc; 
addpath(genpath(pwd)) % add folders and subfolders of the directory to path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SELECT OPTIONS (we don't sweep over these)

% set the length of the simulation in pre-processing for determining TCL
% initial conditions
simLengthForICs = 24/2; % [h]

% Guassian noise used in the TCL state updates, meant to capture small
% distrurbances in the houses
noiseMean = 0;       % mean
noiseSd = 5*10^(-4); % standard deviation

% Choose whether to use teh same square footage value for all houses
% 1: All houses have the same square footage of 2500 ft^2
% 0: Each house inherits the square footage of the house it's been assigned
sfOption = 1;

% TCL temperature delays option that control the time delay between the
% switching of the A/C and the temperature starting to move towards the
% opposite direction
% 1: no temperature delay
% 2: constant low temperature delay
% 3: constant high temperature delay
% 4: temperature delay distribution
temperatureDelayOption = 1;
% END SELECT OPTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET PARAMETER SWEEPS


% switch that determines how we generate the simulation case settings. This
% can be done either in an exhaustive search, or using the two-factor case
% design. 
% 0 = exhaustive search
% 1 = two factor design
tclOptionSet.useTwoFactorCaseDesign = 0;
%%%%%% Set this to 1 if we want to define the full set of cases, which
%%%%%% might cause it to throw errors if there aren't enough options in the
%%%%%% parameters that we sweep over, or select 0 if you want to be able to
%%%%%% just design a specific case, or if you want to create every
%%%%%% combination of parameter setting that you selected.

% Integers that control different weather time series options. 
% 1 = constant temp at 90 degF
% 2 = constant temp at 100 degF
% 3 = constant temp at 70 degF %%%% Not in use for cases, but it works
% 4 = linear 
tclOptionSet.weatherSet = [1]; % [-]

% Integers that control different building parameter options.
% Option is used to control TCL build parameter
% values/distributions in the TCL parameterization.
% Option 1 is GLD default values as the means, and options 2 is a lower RC
% type setting that preserves the average duty cycles of the population
tclOptionSet.buildingParameterSet = [1]; % [-]

% TCL heterogeneity option that controls the probability distributions of
% the potential TCL parameters. This defines cases that control the
% paramter distribution range
% 1 = 0.2 range of values
% 2 = 0 standard deviation (homogeneous population
tclOptionSet.tclHeterogeneitySet = [1]; %[-]


% Values > 0 that indicate the lockout time when the TCL switches on and
% off respectively. These are used to set lockout times in the TCL
% parameterization. These options are treated as a tuple, meaning that the
% first element of each is used, the second element of each is used, and so
% on. As a result, these need to each contain the same number of elements.
tclOptionSet.lockoutTimeOnSet = 180; % [sec]
tclOptionSet.lockoutTimeOffSet = 3; % [min]

% Determine the range of the possible setpoints and deadbands
Tsp_range = [20, 24]; % edge values between which to draw random setpoints [20, 24]
db_range = [1, 2];  % edge values between which to draw random deadband widths, e.g. [1, 2]

% option to include inrush 
inrushOption = 1;
%%%%%%%%%%%%%% DO NOT CHANGE %%%%%%%%%% ... these are set to reflect what
%%%%%%%%%%%%%% we'll see in the field tests. You can change them if you
%%%%%%%%%%%%%% want to investigate their impact, but they aren't part of
%%%%%%%%%%%%%% the cases

% This controls the directories that we will look into to populate the TCL
% models according to the above options. It populates EVERY iteration of
% the feeder within each of the directories. 
tclOptionSet.setOfTaxFiles = {%'R5-12.47-4.glm'; ...
    %'R5-12.47-1.glm'; ...
    'R5-25.00-1.glm'; ...
    %'R2-12.47-2.glm';...
    %'R2-12.47-3.glm';...
    %'R2-25.00-1.glm';...
    %'R5-12.47-5.glm';...
    %'R5-35.00-1.glm'...
    };

% END PARAMETER SWEEPS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Determine where your m-file's folder is.
folder = fileparts(which(mfilename)); 
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));

% function that determines option sequences given parameter sets above and
% set of different feeders that have been constructed. Either uses
% two-factor design or does not, depending on the option. 
parameterSweepTcls = generateTclParameterSweepArrays(tclOptionSet);

% calculate the number of cases from the option sequences determined above
numCases = numel(parameterSweepTcls.onLockoutSweepValues);


% initialize empty feederDataStructure
feederAndTclData = struct();
savePaths = {};

% loop through each case to select the weather data, parameterize TCLs, and
% initialize TCLs
for idx = 1:numCases

disp(['Populating TCLs on feeder ' num2str(idx) ' of ' num2str(numCases)])
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BEGIN OPTION SELECTION

% feeder
tclOptions.feederDataFile = parameterSweepTcls.feederSweepValues{idx};
tclOptions.feederDirectory = parameterSweepTcls.feederDirectorySweepValues{idx};
% weather
tclOptions.weatherOption = parameterSweepTcls.weatherSweepValues(idx); % figure out a way to define different weather signals

% building parameters
tclOptions.buildingParameterOption = parameterSweepTcls.buildingSweepValues(idx); % options to control house paramters

% get tcl heterogeneity
tclOptions.heterogeneityOption = parameterSweepTcls.heterogeneitySweepValues(idx);

% get lockout times
tclOptions.lockTimeOnInSec = parameterSweepTcls.onLockoutSweepValues(idx);
tclOptions.lockTimeOffInMin = parameterSweepTcls.offLockoutSweepValues(idx);


% END OPTION SELECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GET FEEDER DATA

% loads the feeder data and previously constucted option sets

% load feeder data for this feeder
dataFile = [tclOptions.feederDirectory 'feederData.mat'];
load(dataFile,'feederData')
thisFeederData = feederData.(tclOptions.feederDataFile);

% extract fields of the structure and copy them to variables
names = fieldnames(thisFeederData);
for i=1:length(names)
eval([names{i} '=thisFeederData.' names{i} ';']);
end

% add option to option set for the simulation
generalParameters.simLengthForICs = simLengthForICs;
generalParameters.numStepsIC = round(simLengthForICs/generalParameters.timeStepInHr);

% END GET FEEDER DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CHECK IF WE CAN USE DATA FOR OTHER VOLTAGE REGULATOR CASE

feederField = strrep(strrep(tclOptions.feederDirectory,'.',''),'/','');

% if the feeder field doesn't exist, make it
if ~isfield(feederAndTclData,feederField)
    feederAndTclData.(feederField) = [];
    savePaths = [savePaths; ['./' feederField '/']];
end

% get the feeder data
feederDataOptions = tclOptions.feederDataFile;

% set options being used to generate TCLs
presentOptions = ['optionsTCL_Weath_' num2str(tclOptions.weatherOption) ...
    '_Build_' num2str(tclOptions.buildingParameterOption) ...
    '_Het_' num2str(tclOptions.heterogeneityOption) ...
    '_LockOn_' num2str(tclOptions.lockTimeOnInSec)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE VALUES FROM SELECTED OPTIONS

% conver lockout times to time-steps
tclOptions.lockoutTimeOffInTimeSteps = tclOptions.lockTimeOffInMin*60/(generalParameters.timeStepInSec);
tclOptions.lockTimeOnInTimeSteps = tclOptions.lockTimeOnInSec/generalParameters.timeStepInSec;

% calculate that number of time-steps in calculating
% the initial conditions
tclOptions.numStepsIC = round(simLengthForICs/generalParameters.timeStepInHr)+1;

% ranges based on which setpoint and deadband will be generated
tclOptions.Tsp_range = Tsp_range; % [Tsp_min, Tsp_max] 
tclOptions.db_range = db_range; % [deadband_min, deadband_max]

% Guassian noise used in the TCL state updates
tclOptions.noiseMean = noiseMean;  % mean
tclOptions.noiseSd = noiseSd; % standard deviation

% Option for square footage 
tclOptions.sfOption = sfOption;

% Option for temperature delays
tclOptions.temperatureDelayOption = temperatureDelayOption;

% Option for inrush power
tclOptions.inrushOption = inrushOption;

% END CALCULATE VALUES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GET WEATHER DATA

% generate outdoor temp in degC based on time series option
outdoorTemperature = getOutdoorTemperature(tclOptions);
% generate irradiance in btu/hr.ft2 based on time series option
irradiance = getIrradiance(tclOptions);
% END WEATHER DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERIZE TCLS

[tclParameters] = GenerateUpdatedRandomTclParameters(generalParameters, houseParameters, tclOptions, outdoorTemperature, irradiance);

% END PARAMETERIZE TCLS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INITIALIZE TCLS

% set the initial conditions of the tcl's (e.g., on/off mode and internal
% temperatures) based on the simulation parameters and the tcl parameters
% calculated in the feeder generator. 
[tclParameters] = calculateInitialConditions...
                        (generalParameters, tclParameters);
% END INITIALIZE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% create a field for the given option set and store options
feederAndTclData.(feederField).(feederDataOptions).(presentOptions).houseParameters = houseParameters;
feederAndTclData.(feederField).(feederDataOptions).(presentOptions).generalParameters = generalParameters;
feederAndTclData.(feederField).(feederDataOptions).(presentOptions).feederParameters = feederParameters;
feederAndTclData.(feederField).(feederDataOptions).(presentOptions).tclOptions = tclOptions;
feederAndTclData.(feederField).(feederDataOptions).(presentOptions).tclParameters = tclParameters;
feederAndTclData.(feederField).(feederDataOptions).(presentOptions).outdoorTemperature = outdoorTemperature;
feederAndTclData.(feederField).(feederDataOptions).(presentOptions).irradiance = irradiance;

end


% save the values generated in the preprocessing script to a data file to
% be loaded during TCL parameterization
tempFeederAndTclData = feederAndTclData;
for idx = 1:numel(savePaths)
    
    thisSavePath = savePaths{idx};

    % uses the save path to pull the data only for the specific feeder
    feederAndTclData = tempFeederAndTclData.(strrep(strrep(thisSavePath,'/',''),'.',''));
    save([thisSavePath 'feederAndTclData.mat'], 'feederAndTclData') 

end