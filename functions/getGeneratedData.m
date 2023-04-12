% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [houseParameters, generalParameters, feederParameters, ...
    tclParameters, outdoorTemperature, irradiance] = getGeneratedData(simulationOptions)

    % load the data for the feeder options that we generated before
    feederFile = simulationOptions.feederFile;
    folder = strrep(strrep(strrep(feederFile,'.glm',''),'-','_'),'.','');
    
    % if running from within gld, need to go up a level based on where we
    % run the script from
    if simulationOptions.useGLD == 1
        thisData = load(['..\..\' folder '\feederAndTclData.mat' ]);
    else
        thisData = load(['.\' folder '\feederAndTclData.mat' ]);
    end
    
    percentTclOption = simulationOptions.percentTclOption;
    feederCapScaling = simulationOptions.feederCapScaling;
    backgroundDemandScaling = simulationOptions.backgroundDemandScaling;
    voltageRegSetting = simulationOptions.voltageRegSet;
    
    % put the parameters into the same format used to generate the field
    % names and get that field
    fieldToGet = ['options_Vset_' num2str(voltageRegSetting) ...
        '_' num2str(floor(percentTclOption*100)) ...
        '_' num2str(floor(feederCapScaling*100)) ...
        '_' num2str(floor(backgroundDemandScaling*100)) ];    
    feederData = thisData.feederAndTclData.(fieldToGet);
    
    % do the same as above to get the next sub-field in the structure that
    % has the TCL specific information
    weather = simulationOptions.weather;
    buildingParameter = simulationOptions.buildingParameter;
    tclHeterogeneity = simulationOptions.tclHeterogeneity;
    lockoutOn = simulationOptions.lockoutOn;
    
    fieldToGet2 = ['optionsTCL_Weath_' num2str(weather) ...
        '_Build_' num2str(buildingParameter) ...
        '_Het_' num2str(tclHeterogeneity)  ...
        '_LockOn_' num2str(lockoutOn)];
    
    priorData = feederData.(fieldToGet2);
    
    % set the outputs that will be extracted below
    houseParameters = [];
    generalParameters = []; 
    feederParameters = [];
    tclParameters = [];
    outdoorTemperature = [];
    irradiance = [];
    
    % extract fields of the structure and copy them to variables
    names = fieldnames(priorData);
    for i=1:length(names)
        eval([names{i} '=priorData.' names{i} ';']);
    end

    
end