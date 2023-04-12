% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [controllerInformation, simulatedTclData, tclParameters] = ...
         updateStructures(n, controlNumSteps,...
            controllerInformation,tclParameters,simulatedTclData, ...
            airTemperatureVector,onOffValueVector,massTemperatureVector,outdoorTemperature, irradiance, controlActions)
% This function updates several structs at every timestep.
% Input:
%   n : int
%       Current simulation timestep
%   controlNumSteps: int
%       Number of steps between execution (controlTimestep/simulationTimestep)
%   controllerInformation: struct 
%       Includes info regarding the controller. e.g. type, gains, TCLs assigned etc.
%   tclParameters: struct 
%       Includes the characteristics of the TCL population. e.g. thermal parameters, power draws etc.
%   simulatedTclData: struct
%       Stored data, results of the simulation e.g. history of ON/OFF modes and air temperatures
%   airTemperatureVector: vector 
%       Current values of the regulated air temperature at each TCL
%   onOffValueVector: vector 
%       Current ON/OFF (1 or 0) mode of each TCL
%   massTemperatureVector: vector 
%       Current values of the mass temperature at each TCL
%   outdoorTemperature: float
%       Current value of the outdoor temperature
%   irradiance: float
%       Current value of the solar irradiance
%   controlActions: struct
%       Includes current control signals, e.g. switching probabilities

%% Update power draws
% keeps track of tcls thast switched on for inrush calculations
justTurnedOn = tclParameters.justTurnedOn;
% update the power draw of the device based on the voltage, the outdoor
% temeperature, the on/off mode, and whether the unit just turned on
if tclParameters.perc_2zone == 0  % only single-zone houses
    tclParameters = updatePowerDraw_1zone(tclParameters, justTurnedOn, onOffValueVector);
else
    tclParameters = updatePowerDraw_2zone(tclParameters, justTurnedOn, onOffValueVector);
end

%% Update bin model and store power draws
% loop through aggregators to compute some aggregator-specific information
for aggIdx = 1:numel(controlNumSteps)
    tclsThisAgg = controllerInformation.(['agg' num2str(aggIdx)]).tclsAssigned;
    controller_type = controllerInformation.(['agg' num2str(aggIdx)]).Type;
    % update current state vector of binModel (percentages of tcls in each bin)
    % if the current timestep matches the control timestep minus 1
    % we want to update the bin model right before sending a command
    if (n==0 || mod(n,controlNumSteps(aggIdx))==0) && (strcmp(controller_type, 'Markov controller') || strcmp(controller_type, 'Markov controller with lockouts') || strcmp(controller_type, 'Markov controller with delays') || strcmp(controller_type, 'Markov controller 2 zone') || strcmp(controller_type, 'Markov controller 2 zone with lockouts') || strcmp(controller_type, 'Markov controller mixed zone'))
        % get the bin model and the tcls for this aggregator
        binModel = controllerInformation.(['agg' num2str(aggIdx)]).binModel;
        % update the states of the bin model
        if tclParameters.perc_2zone == 0 % only single-zone houses
            binModel = update_binModel_states_1zone(binModel,tclParameters,airTemperatureVector,onOffValueVector,tclsThisAgg,controlActions.u,n,controllerInformation.(['agg' num2str(aggIdx)]));
        elseif tclParameters.perc_2zone == 1 % only 2-zone houses
            binModel = update_binModel_states_2zone(binModel,tclParameters,airTemperatureVector,onOffValueVector,tclsThisAgg,controlActions.onOffSignal,n,controllerInformation.(['agg' num2str(aggIdx)]));
        else % both single and 2-zone houses in the population, will be using the mixed controller
            tclsAssigned_1zone = controllerInformation.(['agg' num2str(aggIdx)]).controllerInformation_1zone.tclsAssigned;  % single-zone TCLs of the current aggregator
            tclsAssigned_2zone = controllerInformation.(['agg' num2str(aggIdx)]).controllerInformation_2zone.tclsAssigned;  % 2-zone TCLs of the current aggregator
            % update the current control timesteps for each sub struct
            controllerInformation.(['agg' num2str(aggIdx)]).controllerInformation_1zone.timeStep = controllerInformation.(['agg' num2str(aggIdx)]).timeStep;
            controllerInformation.(['agg' num2str(aggIdx)]).controllerInformation_2zone.timeStep = controllerInformation.(['agg' num2str(aggIdx)]).timeStep;
            binModel.binModel_1zone = update_binModel_states_1zone(binModel.binModel_1zone,tclParameters,airTemperatureVector,onOffValueVector,tclsAssigned_1zone,controlActions.onOffSignal_1zone,n,controllerInformation.(['agg' num2str(aggIdx)]).controllerInformation_1zone);
            binModel.binModel_2zone = update_binModel_states_2zone(binModel.binModel_2zone,tclParameters,airTemperatureVector,onOffValueVector,tclsAssigned_2zone,controlActions.onOffSignal_2zone,n,controllerInformation.(['agg' num2str(aggIdx)]).controllerInformation_2zone);         
        end           
        controllerInformation.(['agg' num2str(aggIdx)]).binModel = binModel;
    end

    % store the aggregate demand of the tcls
    powerDraws = simulatedTclData.tclParameters.P_power_draw(tclsThisAgg);
    reactivePowerDraws = simulatedTclData.tclParameters.Q_power_draw(tclsThisAgg);
    simulatedTclData.totalRealPowerDemand(aggIdx,n+1) =  onOffValueVector(tclsThisAgg)'*powerDraws ;
    simulatedTclData.totalReactivePowerDemand(aggIdx,n+1) =  onOffValueVector(tclsThisAgg)'*reactivePowerDraws ;

    realPow = sum(onOffValueVector(tclsThisAgg).*powerDraws);
    reacPow = sum(onOffValueVector(tclsThisAgg).*reactivePowerDraws);
    acPower = realPow + 1j*reacPow;

    simulatedTclData.totalComplexPowerDemand(aggIdx,n+1) = acPower;

    controllerInformation.(['agg' num2str(aggIdx)]).totalRealPowerDemand(n+1) = realPow;
end

%% Update irradiance, outdoor temperature and heat gains

% update the outdoor tempeature in the tclParameters
if n ~=0
    tclParameters.T_a = tclParameters.T_a.*0 + outdoorTemperature(n);
    tclParameters.irradiance = tclParameters.irradiance.*0 + irradiance(n);
end
% update A/C heatgain based on the new outdoor temperature
if tclParameters.perc_2zone == 0  % only single-zone houses
    tclParameters = updateQh_1zone(tclParameters);
else  % 2-zone houses exist in the population
    tclParameters = updateQh_2zone(tclParameters, onOffValueVector);
end        
% update internal and solar heat gains based on the new solar
% irradiance value - assume same irradiance for all TCLs
[tclParameters.Q_s, tclParameters.Q_m] = updateQsQm(tclParameters.irradiance, tclParameters.SHGC, tclParameters.Q_i, tclParameters.mass_internal_gain_fraction, tclParameters.mass_solar_gain_fraction);
% store struct
simulatedTclData.tclParameters = tclParameters;
    
end