% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [m_vect_new, theta_a_new, theta_m_new, tclParameters, controllerInformation] = TCL_pop_sim_3state(onOffSwitchValue, airTemperature, massTemperature, generalParameters, tclParameters, controlActions, controllerInformation, simulationOptions)
%{
This function updates the TCL states as well as the power draw of the 
devices.
Input:
    onOffSwitchValue: vector of floats
        ON/OFF mode of each TCL
    airTemperature: vector of floats
        Air temperature of each TCL
    massTemperature: vector of floats
        Mass temperature of each TCL
    generalParameters: struct
        Contains info for the simulation as for example the size of the
        population.
    tclParameters: struct
        Contains info for the TCLs like thermal parameters etc.
    controlActions: struct
        Control actions, e.g. switching probabilities
    controllerInformation: struct
        Info for the controller e.g. gains, bin model etc.
    simulationOptions: struct
        Simulation options, e.g. option to use local voltage for controller
Output:
    m_vect_new: vector of booleans
        New ON/OFF mode for each TCL
    theta_a: vector of floats
        New air temperatures of each TCL
    theta_m: vector of floats
        New mass temperatures of each TCL
    tclParameters: struct
        Updated with new lockouts, delays etc.
    controllerInformation: struct
        Updated from PEM
%}

%% Advance air and mass temperatures according to the 3-state TCL model
if tclParameters.perc_2zone == 0  % only single-zone houses
    [theta_a_new, theta_m_new] = advanceTemps_1zone(tclParameters, airTemperature, massTemperature, onOffSwitchValue);
else  % some houses are two-zone
    [theta_a_new, theta_m_new] = advanceTemps_2zone(tclParameters, airTemperature, massTemperature, onOffSwitchValue);
end

%% Update the on/off mode setting 
% set a logical that flags whether the TCLis locked (=1) or not (=0)
locked = tclParameters.timeUntilUnlocked > 0;
% check which TCLs have violated their deadband and should be switched subject to lockout constraints
if tclParameters.perc_2zone == 0  % only single-zone houses
    [m_vect_new, logical_index_high,logical_index_low] = ...
        thermostatLogic_1zone(tclParameters.T_max, tclParameters.T_min, theta_a_new, onOffSwitchValue, locked);
else  % some houses are two-zone
    [m_vect_new, logical_index_high,logical_index_low] = ...
        thermostatLogic_2zone(tclParameters, tclParameters.T_max, tclParameters.T_min, theta_a_new, onOffSwitchValue, locked);
end
 
% controller actions
for idx = 1:simulationOptions.NumberOfAggregators
    controllerInformationTemp = controllerInformation.(['agg' num2str(idx)]);
    controlNumSteps = controllerInformationTemp.controlTimeStepInSec./generalParameters.timeStepInSec; % convert control timesteps to number of steps between execution
    % Control commands are issued at the 1st time instant of the simulation and then after every controlNumSteps
    % if we are not currently in a timestep where control commands are executed, skip this section
    if controllerInformationTemp.timeStepSim ~= 1 && mod(controllerInformationTemp.timeStepSim-1,controlNumSteps) ~= 0 
        continue
    end
    theseTcls = controllerInformationTemp.tclsAssigned;
    if strcmp(controllerInformationTemp.Type , 'Markov controller') || strcmp(controllerInformationTemp.Type , 'Markov controller with lockouts') || strcmp(controllerInformationTemp.Type , 'Markov controller with delays')
        if simulationOptions.commNetwork == 1
            m_vect_new(theseTcls) = executeControllerCommands_markov(controllerInformationTemp, length(theseTcls), tclParameters.timeUntilUnlocked(theseTcls), tclParameters.timeUntilDelayEnds(theseTcls), tclParameters.T_min(theseTcls), tclParameters.T_max(theseTcls), controlActions.onOffSignal, theta_a_new(theseTcls), m_vect_new(theseTcls), logical_index_high(theseTcls), logical_index_low(theseTcls), locked(theseTcls), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls));
        elseif simulationOptions.commNetwork == 2 || simulationOptions.commNetwork == 3
            m_vect_new(theseTcls) = executeControllerCommands_markov_badComms(controllerInformationTemp, length(theseTcls), tclParameters.timeUntilUnlocked(theseTcls), tclParameters.timeUntilDelayEnds(theseTcls), tclParameters.T_min(theseTcls), tclParameters.T_max(theseTcls), controlActions.onOffSignal, theta_a_new(theseTcls), m_vect_new(theseTcls), logical_index_high(theseTcls), logical_index_low(theseTcls), locked(theseTcls), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls));
        end
    elseif strcmp(controllerInformationTemp.Type , 'Markov controller 2 zone') || strcmp(controllerInformationTemp.Type , 'Markov controller 2 zone with lockouts')
        if simulationOptions.commNetwork == 1
            m_vect_new(theseTcls) = executeControllerCommands_markov2zone(controllerInformationTemp, length(theseTcls), tclParameters.timeUntilUnlocked(theseTcls), tclParameters.T_min(theseTcls), tclParameters.T_max(theseTcls), controlActions.onOffSignal, theta_a_new(theseTcls), m_vect_new(theseTcls), logical_index_high(theseTcls), logical_index_low(theseTcls), locked(theseTcls), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls));    
        elseif simulationOptions.commNetwork == 2 || simulationOptions.commNetwork == 3
            m_vect_new(theseTcls) = executeControllerCommands_markov2zone_badComms(controllerInformationTemp, length(theseTcls), tclParameters.timeUntilUnlocked(theseTcls), tclParameters.T_min(theseTcls), tclParameters.T_max(theseTcls), controlActions.onOffSignal, theta_a_new(theseTcls), m_vect_new(theseTcls), logical_index_high(theseTcls), logical_index_low(theseTcls), locked(theseTcls), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls));    
        end
    elseif strcmp(controllerInformationTemp.Type , 'Markov controller mixed zone')
        theseTcls_1zone = theseTcls(~tclParameters.is2zone(theseTcls));  % current single-zone TCLs
        theseTcls_2zone = theseTcls(tclParameters.is2zone(theseTcls));  % current 2-zone TCLs
        if simulationOptions.commNetwork == 1
            m_vect_new(theseTcls_1zone) = executeControllerCommands_markov(controllerInformationTemp.controllerInformation_1zone, length(theseTcls_1zone), tclParameters.timeUntilUnlocked(theseTcls_1zone), tclParameters.timeUntilDelayEnds(theseTcls_1zone), tclParameters.T_min(theseTcls_1zone), tclParameters.T_max(theseTcls_1zone), controlActions.onOffSignal(1:controllerInformationTemp.controlSignalLength_1zone), theta_a_new(theseTcls_1zone), m_vect_new(theseTcls_1zone), logical_index_high(theseTcls_1zone), logical_index_low(theseTcls_1zone), locked(theseTcls_1zone), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls_1zone));
            m_vect_new(theseTcls_2zone) = executeControllerCommands_markov2zone(controllerInformationTemp.controllerInformation_2zone, length(theseTcls_2zone), tclParameters.timeUntilUnlocked(theseTcls_2zone), tclParameters.T_min(theseTcls_2zone), tclParameters.T_max(theseTcls_2zone), controlActions.onOffSignal(controllerInformationTemp.controlSignalLength_1zone+1:end), theta_a_new(theseTcls_2zone), m_vect_new(theseTcls_2zone), logical_index_high(theseTcls_2zone), logical_index_low(theseTcls_2zone), locked(theseTcls_2zone), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls_2zone));          
        elseif simulationOptions.commNetwork == 2 || simulationOptions.commNetwork == 3
            m_vect_new(theseTcls_1zone) = executeControllerCommands_markov_badComms(controllerInformationTemp.controllerInformation_1zone, length(theseTcls_1zone), tclParameters.timeUntilUnlocked(theseTcls_1zone), tclParameters.timeUntilDelayEnds(theseTcls_1zone), tclParameters.T_min(theseTcls_1zone), tclParameters.T_max(theseTcls_1zone), controlActions.onOffSignal(theseTcls_1zone,1:controllerInformationTemp.controlSignalLength_1zone), theta_a_new(theseTcls_1zone), m_vect_new(theseTcls_1zone), logical_index_high(theseTcls_1zone), logical_index_low(theseTcls_1zone), locked(theseTcls_1zone), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls_1zone));
            m_vect_new(theseTcls_2zone) = executeControllerCommands_markov2zone_badComms(controllerInformationTemp.controllerInformation_2zone, length(theseTcls_2zone), tclParameters.timeUntilUnlocked(theseTcls_2zone), tclParameters.T_min(theseTcls_2zone), tclParameters.T_max(theseTcls_2zone), controlActions.onOffSignal(theseTcls_2zone,controllerInformationTemp.controlSignalLength_1zone+1:end), theta_a_new(theseTcls_2zone), m_vect_new(theseTcls_2zone), logical_index_high(theseTcls_2zone), logical_index_low(theseTcls_2zone), locked(theseTcls_2zone), simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls_2zone));          
        end
    elseif strcmp(controllerInformationTemp.Type , 'PID')
        m_vect_new(theseTcls) = executeControllerCommands_pid(length(theseTcls), controlActions.onOffSignal, m_vect_new(theseTcls), logical_index_high(theseTcls), logical_index_low(theseTcls), locked(theseTcls), controllerInformationTemp, simulationOptions.useLocalVoltage, tclParameters.voltage240(theseTcls));
    elseif strcmp(controllerInformationTemp.Type, 'PEM_E-T controller')
        [m_vect_new(theseTcls),controllerInformation.(['agg' num2str(idx)])] = executeControllerCommands_PEM(controllerInformationTemp,controlActions,logical_index_high(theseTcls), logical_index_low(theseTcls), locked(theseTcls),m_vect_new(theseTcls),controllerInformationTemp.timeStepSim,airTemperature,generalParameters.timeStepInSec);
    elseif strcmp(controllerInformationTemp.Type, 'none')
        
    else 
        warning('Controller not recognized, and so no action taking effect.');
    end   
end

%% Update lockouts
% load previous lockout remaining time
timeUntilUnlocked = tclParameters.timeUntilUnlocked;
if tclParameters.perc_2zone == 0  % only single-zone houses
    [tclParameters.timeUntilUnlocked, remainedOn, remainedOff, justTurnedOn, justTurnedOff] = ...
        updateLockouts_1zone(m_vect_new, onOffSwitchValue, tclParameters, timeUntilUnlocked);
else  % some houses are two-zone
    [tclParameters.timeUntilUnlocked, remainedOn, remainedOff, justTurnedOn, justTurnedOff] = ...
        updateLockouts_2zone(m_vect_new, onOffSwitchValue, tclParameters, timeUntilUnlocked);
end

%% Update delay time
if tclParameters.temperatureDelayOption ~= 1
    tclParameters = updateTempDelays(tclParameters, remainedOn, remainedOff, justTurnedOff, justTurnedOn, generalParameters.timeStepInSec);
end

%% Update power draw
% update the power draw of the device based on the voltage, the outdoor
% temeperature, the on/off mode, and whether the unit just turned on
if tclParameters.perc_2zone == 0  % only single-zone houses
    tclParameters = updatePowerDraw_1zone(tclParameters, justTurnedOn, m_vect_new);
else
    tclParameters = updatePowerDraw_2zone(tclParameters, justTurnedOn, m_vect_new);
end

%% Store which TCLs turned ON
% we're storing this so that we can use it when calling the power draw
% update each time the voltage updates from GLD without updating the state
tclParameters.justTurnedOn = justTurnedOn;
tclParameters.justTurnedOff = justTurnedOff;

end