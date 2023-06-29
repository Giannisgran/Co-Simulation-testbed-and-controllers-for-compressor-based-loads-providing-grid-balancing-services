% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script takes a set of parameters for simulating a gridlab-d PNNL
% feeder with MATLAB-based house and appliance models. It takes a PNNL
% feeder and some user-defined parameters, and populates the feeder with
% houses and based on the given paramters. It also sets up the
% necessary files and functions for running the simulation (e.g., glm file,
% link file, etc.).

% Set cd to the directory containing this script before running it. The
% path to this file gets used in setting other paths. 

% In this file, the options that we care about modifying for the test cases
% are as listed below. 
%   Parameters: 
%       simulationOptionSet.voltageRegulatorSetting = [1, 2];
%   In the above, voltageRegulatorSetting
%   sets the voltage regulator setpoint for the feeder (1 = set so there
%   are no issues; 2 = set for each feeder to try and induce voltage
%   issues).
% 
%   To run two-factor case study setup, both options need to be used in
%   this script to construct the necessary files. You don't need to specify
%   two-factor case study construction here. 

% clear the workspace and command interface
clc; clearvars;
addpath(genpath(pwd)) % add folders and subfolders of the directory to path
fprintf('Copyright (C) 2023 The Regents of the University of Michigan\n')
rng(4234234)  % this will keep the total house number the same, since it is determined using Bernouli trials
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SELECT PARAMETERS AND SETTINGS

% This section sets user-defined parameters that are used throughout the
% simulation and pre-processing

% set the time-step of the simulation in seconds. 
timeStepInSec = 2; % [s]

% percentage of houses with 2-zones and two compressors
perc_2zone = 0; % must be between [0,1]

% Directory that contains the taxonomy feeders listed above
tax_dir = './Taxonomy_Feeders-Master/';

% The PNNL feeders have different weather zones (6 of them, I think) for 
% different areas of the US. This sets the region that we'll be using. 
% Note that this overrides the region given in the feeder name.
region = 5; % setting it for Austin, TX;

% sets the random number seed for the simulation after preprocessing is
% done
rngSeed = 105;



%%%%%%%% OPTIONS FOR TEST SCENARIOS NEED IN FEEDER POPULATION
% sets the range of parameters for the percentage of houses with tcls and
% the scaling used to determine the number of houses on the feeder 

% value from 0 to 1 that controls the percentage of houses that contain a
% TCL. Manipulates the number of TCLs simulated for a given number of
% houses.
feederOptions.percentTclOptionSet = 0.5; % no units
%%%%%% DO NOT CHANGE %%%%% ...it just isn't an option we use for cases


% value greater than 0 that helps determine the number of houses on the
% feeder. The population script will scale the power capacity being
% allocated to houses by the factor given below. For example 1 populates
% the feeder normally, 0.9 populates the feeder with 90% capacity, and 1.1
% populates the feeder with 110% capacity. 
feederOptions.feederCapacityScalingOptionSet = 1; % no units
%%%%%% DO NOT CHANGE %%%%% ...it just isn't an option we use for cases

% value greater than 0 that scales the non-TCL demand of a house. Used to
% increase non-TCL demand without changing TCL or house numbers.
feederOptions.backgroundDemandScalingFactorOptionSet = 0.75;
%%%%%% DO NOT CHANGE %%%%% ...it just isn't an option we use for cases

% value to change the voltage regulator's voltage that it regulates to in
% order to push voltages on the feeder upper or lower
% 1 = nominal case that doesn't induce issues on the power grid
% 2 = case defined for each feeder that pushes voltages to upper or lower
% edge of allowable range
feederOptions.voltageRegulatorOptionSet = [1];

% feeder files that we want to populate, and the directory that contains
% those file. Look in "Taxonomy_Feeders" folder for possible feeders.
feederOptions.setOfTaxFiles = {%'R5-12.47-4.glm'; ...
    %'R5-12.47-1.glm'; ...
    'R5-25.00-1.glm'; ...
    %'R2-12.47-2.glm';...
    %'R2-12.47-3.glm';...
    %'R2-25.00-1.glm';...
    %'R5-12.47-5.glm';...
    %'R5-35.00-1.glm'...
    };
    
% set the default start and end dates to be populated into the glm file
% that gets constructed. These get overwritten as needed at the actual
% simulation script generation. 
defaultStartDate = '2018-07-15 00:00:00';
defaultEndDate = '2018-07-15 01:00:00';
%%%%%% DO NOT CHANGE %%%%% ...it just isn't an option we use for cases and
%%%%%% we handle them automatically when building the simulations

% END SELECT PARAMETERS AND SETTINGS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

totalFeederConstructionStartTime = tic;

% generate the set of parameters that we will loop through to create the
% set of feeders needed for the test scenarios. 
parameterSweepFeeder = generateFeederParameterSweepArrays(feederOptions);

% initialize empty feederDataStructure
feederData = struct();
savePaths = {};
% loop through the feeders and options in the list of feeders above
for feederNumber = 1:numel(parameterSweepFeeder.feederValues)

% get the population parameters
percentHousesWithTCls = parameterSweepFeeder.percentTcl(feederNumber);
feederCapacityScalingFactor = parameterSweepFeeder.feederCapacity(feederNumber);
backgroundDemandScalingFactor = parameterSweepFeeder.backgroundDemandScalingFactor(feederNumber);
voltageRegulatorSetting  = parameterSweepFeeder.voltageRegulatorSettings(feederNumber);

% get the feeder from the list
tax_file =   parameterSweepFeeder.feederValues{feederNumber};
thisFeederStartTime =  tic;
disp(['Programmatically constructing feeder ' num2str(feederNumber)])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE VALUES FROM SELECTABLE OPTIONS

% This section of the code calculates parameters and settings that are
% derived from the settings in the section above. 

% convert time step from (s) to (hr)
timeStepInHr        = timeStepInSec/3600;

% MATLAB directory for running the gridlabd simulation
% this sets and saves the folder that the link file will use to set its
% working directory as the file that this script is in, hence the need to
% put them in the same directory. 
folderForOnInit = fileparts(which(mfilename)); 

% Use a gridlab d function (regionalization) that was modified for this
% project (regionalizationGSL) to determine regionalization data based on
% the region indicated above. The original gridlabd script can be found in 
% '.\Taxonomy_Feeders\PopulationScript'
regional_data = regionalizationGSL(region);

% Use gridlab d function (TaxFeederData) that was modified for this project
% (+ GSL suffix) to determine information about the specified feeder. The 
% original gridlabd script can be found in 
% '.\Taxonomy_Feeders\PopulationScript'
taxonomy_data = TaxFeederDataGSL(tax_file,region,voltageRegulatorSetting);

% END CALCULATE VALUES FROM SELECTABLE OPTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET UP STRUCTURES

% This section of the code creates structures with the various fields such
% that a large set of parameters can be passed to functions using only the
% structure variable name. It simplifies function calls. 

% make  structure with general parameters
generalParameters = struct(...
    'folderForOnInit', folderForOnInit, ...
    'rngSeed', rngSeed, ...
    'timeStepInSec',timeStepInSec ,...
    'timeStepInHr',timeStepInHr ,...
    'defaultStartDate', defaultStartDate , ...
    'defaultEndDate', defaultEndDate, ...
    'perc_2zone', perc_2zone ...
    );


% make  structure with feeder parameters
feederParameters = struct(...
    'tax_dir', tax_dir, ...
    'percentHousesWithTCls', percentHousesWithTCls, ...
    'feederCapacityScalingFactor', feederCapacityScalingFactor ,...
    'backgroundDemandScalingFactor', backgroundDemandScalingFactor, ...
    'voltageRegulatorSetting', voltageRegulatorSetting, ...
    'region', region, ...
    'taxonomy_data', taxonomy_data, ...
    'regional_data', regional_data, ...
    'feederFile', tax_file ...
    );

% END SET UP STRUCTURES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% run the feeder generator script to determine house and tcl parameters
% based on feeder parameters, to create gld functions (onInit, onSync, and
% onTerm) based on simulation parameters, and to create the matlab link
% file based on feeder and simulation information. 
[feederParameters, houseParameters, generalParameters] = ...
    generateFeeder ...
        (generalParameters, feederParameters);
    
% get feeder name in propper format
feederField = strrep(strrep(strrep(tax_file,'.glm',''),'.',''),'-','_');

% if the feeder field doesn't exist, make it
if ~isfield(feederData,feederField)
    feederData.(feederField) = [];
    savePaths = [savePaths; ['./' feederField '/']];
end

presentOptions = ['options_Vset_' num2str(voltageRegulatorSetting) '_' num2str(floor(percentHousesWithTCls*100)) ...
    '_' num2str(floor(feederCapacityScalingFactor*100)) ...
    '_' num2str(floor(backgroundDemandScalingFactor*100))];

% create a field for the given option set and store options
feederData.(feederField).(presentOptions).houseParameters = houseParameters;
feederData.(feederField).(presentOptions).generalParameters = generalParameters;
feederData.(feederField).(presentOptions).feederParameters = feederParameters;

% clean up any open files that weren't closed
fclose('all');

disp(['        Feeder constructed in ' num2str(toc(thisFeederStartTime),'%.2f') ' seconds'])

end % end of loop for feeders in list of feeders

% save the values generated in the preprocessing script to a data file to
% be loaded during TCL parameterization
tempFeederData = feederData;
for idx = 1:numel(savePaths)
    
    thisSavePath = savePaths{idx};

    % uses the save path to pull the data only for the specific feeder
    feederData = tempFeederData.(strrep(strrep(thisSavePath,'/',''),'.',''));
    save([thisSavePath 'feederData.mat'], 'feederData') 

end


disp(['All feeders constructed in ' num2str(toc(totalFeederConstructionStartTime),'%.2f') ' seconds'])
