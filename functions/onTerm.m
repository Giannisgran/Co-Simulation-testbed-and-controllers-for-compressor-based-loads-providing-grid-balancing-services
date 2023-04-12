% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

% create structure to store data around the simulation for post processing
% and analysis

% get the simulation time and other time-based values
simulationData.simulationTime = toc(simTimer);
simulationData.timeStepAndIterationCounter = timeStepAndIterationCounter;
simulationData.timeVector = timeVector;

% get the various options and parameters used to construct the case
simulationData.simulationOptions = simulationOptions;
simulationData.houseParameters = houseParameters;
simulationData.generalParameters = generalParameters;
simulationData.feederParameters = feederParameters;
simulationData.tclParameters = tclParameters;
simulationData.outdoorTemperature = outdoorTemperature;
simulationData.regulationSignal = regulationSignalInfo;

% get the tcl information and the inputs sent to them
simulationData.simulatedTclData = simulatedTclData;

% get the grid information
simulationData.gridData = gridData;

% get the controller information
simulationData.controllerInformation = controllerInformation;

% save data stored throughout the simulation
saveTime =  datestr(now, 'yyyymmdd_HHMM');

% With bad comms and the mixed zone controller, if the comm struct is
% combined with the simulationData struct then the resulted file is too
% large to save (more than 2GB). Instead, we can save them separately and
% reduce the size significantly.
if simulationOptions.commNetwork ~= 1 && strcmp(controllerInformation.agg1.Type, 'Markov controller mixed zone')
    saveFileName_comms = ['commNetworkData_' saveTime '.mat'];
    if simulationOptions.useGLD == 1
       % if running from GLD, will just save it to the simulation folder
       save(saveFileName_comms, 'simulationData') 
    else
       % if running from MATLAB, will save it to simulationData directory
       save(['./simulationData/' saveFileName_comms], 'simulationData')
    end
else
    % probably need to get comm network info as well
    simulationData.commNetworkData = commNetworkData;
end

saveFileName = ['simulationData_' saveTime '.mat'];
if simulationOptions.useGLD == 1
   % if running from GLD, will just save it to the simulation folder
   save(saveFileName, 'simulationData') 
else
   % if running from MATLAB, will save it to simulationData directory
   save(['./simulationData/' saveFileName], 'simulationData')
end
