% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [tclParameters] = calculateInitialConditions(generalParameters, tclParameters)
%{
This function initialized the TCLs randomly and then simulates for a
specified horizon in order for the population to reach steady-state. The
resulting final states will be the starting point for the simulation
with control that we are interested in.
Input:
    generalParameters: struct
        Contains various information regarding the simulation we want to
        run, e.g. population size, timestep, initialization duration, etc.
    tclParameters: struct
        Contains info for the TCL population, as for example thermal
        parameters etc.
Output:
    tclParameters: struct
        Initialized population which has reached steady state.
%}
    
% set the number of time-steps in steady-state (zero input) simulation to
% the number used to generate the A matrix
numTimeSteps = generalParameters.numStepsIC;

% set the rng for the random draws below
rng(200021) 

% initialize TCL air and mass temperatures somewhere within their deadband
airTemperature = tclParameters.T_min + tclParameters.deadband.*rand(generalParameters.Pop,1);
massTemperature = tclParameters.T_min + tclParameters.deadband.*rand(generalParameters.Pop,1);

% initialize TCL on/off values randomly
onOffSwitchValue = randi(2,generalParameters.Pop,1)-1;

% set up some arrays to hold historical values and initialize them as the
% present values
hisoricalAirTemp = zeros(size(airTemperature,1), numTimeSteps);
hisoricalMassTemp = zeros(size(airTemperature,1), numTimeSteps);
hisoricalAirTemp(:,1) = airTemperature;
hisoricalMassTemp(:,1) = massTemperature;
hisoricalOnOff = zeros(size(airTemperature,1), numTimeSteps);
hisoricalOnOff(:,1) = onOffSwitchValue;

% simulate the TLC's for the given simulation time
for n = 1:numTimeSteps-1
    % create struct with controller info
    [controllerInformation,tclParameters] = controllerSetup('none',tclParameters,generalParameters);
    controllerInformation.timeStep = n;
    controllerInformation.Pop = numel(tclParameters.T_a);
    controllerInformation.tclsAssigned = 1:numel(tclParameters.T_a);

    % set placeholders for potential control actions
    controlActions.deltaDeadband = zeros(generalParameters.Pop,1);
    controlActions.deltaSetpoint = zeros(generalParameters.Pop,1);
    controlActions.onOffSignal = zeros(generalParameters.Pop,1);
    % set 0 = no override, 1 = override off, 2 = override on
    controlActions.overRide = zeros(generalParameters.Pop,1);
    
    % This function will generate the control signals to be sent to the
    % TCLs. TCL models in TCL_pop_sim_3state are extended to allow
    % temperature setpont changes, deadband changes, and user override to
    % go along with on/off control.
    binModel = [];
    simulatedTclData = [];
    
    [controlActions, ~] = generateControlSignal(generalParameters, controllerInformation, simulatedTclData, tclParameters, onOffSwitchValue,airTemperature,binModel,controlActions, massTemperature);

    % update the TCLs
    simulationOptions.NumberOfAggregators = 0; % need this to simulate things here
    [onOffSwitchValue, airTemperature, massTemperature, tclParameters]= TCL_pop_sim_3state(onOffSwitchValue, airTemperature, massTemperature, generalParameters, tclParameters, controlActions,controllerInformation,simulationOptions);
    
    % keeps track of tcls thast switched on for inrush calculations
    justTurnedOn = tclParameters.justTurnedOn;
    % update the power draw of the device based on the voltage, the outdoor
    % temeperature, the on/off mode, and whether the unit just turned on
    if tclParameters.perc_2zone == 0  % only single-zone houses
        tclParameters = updatePowerDraw_1zone(tclParameters, justTurnedOn, onOffSwitchValue);
    else
        tclParameters = updatePowerDraw_2zone(tclParameters, justTurnedOn, onOffSwitchValue);
    end
    % update A/C heatgain based on the new outdoor temperature
    if tclParameters.perc_2zone == 0  % only single-zone houses
        tclParameters = updateQh_1zone(tclParameters);
    else  % 2-zone houses exist in the population
        tclParameters = updateQh_2zone(tclParameters, onOffSwitchValue);
    end
    % store historical values at each timestep
    hisoricalAirTemp(:,n+1) = airTemperature;
    hisoricalMassTemp(:,n+1) = massTemperature;
    hisoricalOnOff(:,n+1) = onOffSwitchValue;
    
end

onOffDiff = diff(hisoricalOnOff,1,2);

% these lines find the last time that a tcl switched on and off
[val,lastSwitchOn] = max(  fliplr(onOffDiff==1),  [],2);
lastSwitchOn(val==0)=nan;
[val2,lastSwitchOff] = max(  fliplr(onOffDiff==-1),  [],2);
lastSwitchOff(val2==0)=nan;


% store the initial conditions in the parameter structure and pass updated
% parameters out of the function
tclParameters.theta_a0        = airTemperature; % initial air temperature
tclParameters.theta_m0        = massTemperature; % initial mass temperature
tclParameters.m0         = onOffSwitchValue; % initial on/off mode

% these store the number of time-steps that is has been on
tclParameters.timestepsSinceLastSwitchOff = lastSwitchOff;
tclParameters.timestepsSinceLastSwitchOn = lastSwitchOn;

end
