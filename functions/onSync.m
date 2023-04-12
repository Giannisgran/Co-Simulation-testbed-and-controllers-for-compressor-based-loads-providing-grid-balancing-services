% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.


% this function/script gets called each time gridlabd executes a new
% iteration/pass. There are two parts of it, the first is when it a new
% time-step and loads need to be updated. The second is when gridlabd has
% iterated within the same time-step and loads' voltage-dependent power
% draw needs to be updated. Convergence is calculated when the power change
% between iterations is small

% get the time within the simulation from gridlabd. This is seconds from
% 1970 or something like that.
timeStep = gld.global.clock;

% if it's a new time-step (i.e., make sure that the time gld sends isn't
% already within the list of times that we have received
if ~sum(timeStep == timeVector)      
    
    % store the new time step in the list of historical times
    timeVector(n) = timeStep;
    
    % figure out how much time has elapsed in the simulation
    timeElapsed(n) = timeStep - timeVector(1);
    
    % initialize iterations for this timestep to 1 and record new timestep;
    % iteration is a counter used to keep track of how many times gld is
    % called for each timestep. timeStepAndIterationCounter stores the
    % number of iterations spent on each timestep for us to look at after
    % the simulation
    iteration = 1;
    timeStepAndIterationCounter = [timeStepAndIterationCounter; timeElapsed(n) iteration];

    % initialize convered, which indicates whether the load models in
    % matlab satisfy the convergence criteria ( = 1) or not ( = 0)
    converged = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % simulate the physical representation of the TCLs

    
    % convert control timesteps to number of steps between execution
    controlNumSteps = controllerInformation.controlTimeStep./generalParameters.timeStepInSec;

    % set controller time-step (which could be different from
    % simulation time-step)
    n_control = max(1,ceil(n/(controlNumSteps(aggIdx)))); % current controller timestep - used max to account for when n=1
    controllerInformation.(['agg' num2str(aggIdx)]).timeStep = n_control; % store current control timestep
    controllerInformation.(['agg' num2str(aggIdx)]).timeStepSim = n; % store current simulation timestep
    % Control commands are issued at the 1st time instant of the simulation and then after every controlNumSteps
    % check to see whether any of the controllers need to execute
    for aggIdx = 1:numel(controlNumSteps) 
        % check if the controller needs to execute
        if n == 1 || mod(n-1,controlNumSteps(aggIdx)) ==0                        
            % This function will generate the control signals to be sent to the
            % TCLs. TCL models in TCL_pop_sim_3state are extended to allow
            % temperature setpoint changes, deadband changes, and user override to
            % go along with on/off control.
            [controlActionsFromController, controllerInformation.(['agg' num2str(aggIdx)])] = generateControlSignal(generalParameters, controllerInformation.(['agg' num2str(aggIdx)]), simulatedTclData, tclParameters, onOffValueVector,airTemperatureVector,controllerInformation.(['agg' num2str(aggIdx)]).binModel,controlActionsFromController, massTemperatureVector);
        end
    end
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Taking control information, imposing
    % communication issues and network characteristics, then having
    % messages arrive at TCLs
    nc = simulationOptions.nc;
    if generalParameters.considerCommNetwork == 1
    controlActionsSent(:,n) = onOffValueVector; % controlActions
    [controlActionsWithComms, commNetworkData] = Comm(n,nc,generalParameters,controlActionsFromController, controllerInformation.(['agg' num2str(aggIdx)]).Type,commNetworkData);
    
    else % with no comms network, just set things as needed
        controlActionsWithComms = controlActionsFromController;
        commNetworkData.controlActionsFromController(:,n) =controlActionsFromController.onOffSignal;
        commNetworkData.controlActionsToTcls(:,n) =controlActionsFromController.onOffSignal;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % set the random numbers at the TCLs for this time-step, so that
    % they can figure out whether they need to switch or not.
    tclParameters.randomValueAtTcl = rand(generalParameters.Pop,1);

    % get the set of nodal voltages and map them to the TCLs using a
    % prevously generated incidence matrix
    allNodalVoltages = [];
    eval(getNodeVoltage);
    tclParameters.voltage240 = tclIncidenceMatrix' * abs(allNodalVoltages);

    % update the TCLs
    [onOffValueVector, airTemperatureVector, massTemperatureVector, tclParameters, controllerInformation] = ...
            TCL_pop_sim_3state(onOffValueVector, airTemperatureVector, massTemperatureVector, generalParameters, tclParameters, controlActionsWithComms,controllerInformation,simulationOptions);
        
    % update structures and store some data
    [controllerInformation, simulatedTclData, tclParameters] = ...
         updateStructures(n, controlNumSteps,...
            controllerInformation,tclParameters,simulatedTclData, ...
            airTemperatureVector,onOffValueVector,massTemperatureVector,outdoorTemperature,irradiance,controlActionsFromController);
    % store the state of the tcls - moved outside function to make it faster
    simulatedTclData.airTemp(:,n+1) = airTemperatureVector;
    simulatedTclData.onOff(:,n+1) = onOffValueVector;
    simulatedTclData.massTemp(:,n+1) = massTemperatureVector;
    simulatedTclData.realPowerDraws(:,n+1) =  onOffValueVector.*tclParameters.P_power_draw ;
    simulatedTclData.reactivePowerDraws(:,n+1) =  onOffValueVector.*tclParameters.Q_power_draw ;
    simulatedTclData.timeUntilUnlocked(:,n+1) =  tclParameters.timeUntilUnlocked;
    simulatedTclData.timeUntilDelayEnds(:,n+1) = tclParameters.timeUntilDelayEnds;
    simulatedTclData.voltage240(:,n+1) = tclParameters.voltage240;
    % store control actions
    simulatedTclData.deltaDeadband(:,n+1) = controlActionsWithComms.deltaDeadband;
    simulatedTclData.deltaSetpoint(:,n+1) = controlActionsWithComms.deltaSetpoint;
    if commNetworkData.scenario == 3 && (strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type,'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type,'Markov controller with delays') || strcmp(controllerInformation.(aggString).Type,'Markov controller mixed zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone with lockouts'))
        simulatedTclData.onOffSignal(:,n+1) = (commNetworkData.controlActionsFromController{1,1})';
        simulatedTclData.onOffSignalFromController(:,n+1) = (commNetworkData.controlActionsFromController{1,1})';
    else
        simulatedTclData.onOffSignal(:,n+1) = controlActionsWithComms.onOffSignal;
        simulatedTclData.onOffSignalFromController(:,n+1) = commNetworkData.controlActionsFromController(:,n);
    end
    simulatedTclData.overRide(:,n+1) = controlActionsWithComms.overRide;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Assign Nodal Powers for GridlabD
    
    % get the apparent power draw of each tcl at this time-step
    powerDraws = 1e3.*(simulatedTclData.tclParameters.P_power_draw);
    reactivePowerDraws = 1e3.*(simulatedTclData.tclParameters.Q_power_draw);    
    actualAcPowerDrawsReal = onOffValueVector.*powerDraws;
    actualAcPowerDrawsReac = onOffValueVector.*reactivePowerDraws;
    actualAcPowerDrawsApp = actualAcPowerDrawsReal + 1j*actualAcPowerDrawsReac;
    
    % use matrix multiplication to calculate the power at each
    % node, using incidence matrices calculated in onInit
    acPowerAllNodes = tclIncidenceMatrix*actualAcPowerDrawsApp;
    backgroundPowerAllNodes = houseIncidenceMatrix * houseParameters.backgroundLoad;
    allNodesPower = acPowerAllNodes + backgroundPowerAllNodes;
    eval(setNodePowerToGLD)
    
    % % END Assign Nodal Powers for GridlabD
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % set the next gld timestep to the same timestep so that we can check
    % convergence of the loads below.
    ans=gld.global.clock;

else % i.e., if it isn't a new timestep
    
    % if our loads have converged, then skip to the else, don't recalculate
    % the loads, and set the time-step as the next value. If the loads
    % havent converged for this time-step then update the ZIPW loads.
    if exist('converged','var') && converged ~= 1
       
        % keep track of number of iterations on each time-step
        iteration = iteration + 1;
        timeStepAndIterationCounter = [timeStepAndIterationCounter; timeElapsed(n) iteration];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % UPDATE VOLTAGES IN TCL PARAMETER STRUCTURE
        
        % get the set of nodal voltages and map them to the TCLs using a
        % prevously generated incidence matrix
        allNodalVoltages = [];
        eval(getNodeVoltage);
        tclParameters.voltage240 = tclIncidenceMatrix' * abs(allNodalVoltages);

        
        % END UPDATE VOLTAGES
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % update structures and store some data
        
        controlNumSteps = controllerInformation.controlTimeStep./generalParameters.timeStepInSec;
        [controllerInformation, simulatedTclData, tclParameters] = ...
            updateStructures(n, controlNumSteps,...
            controllerInformation,tclParameters,simulatedTclData, ...
            airTemperatureVector,onOffValueVector,massTemperatureVector,outdoorTemperature,irradiance,controlActionsFromController);
            
        % store the state of the tcls
        simulatedTclData.airTemp(:,n+1) = airTemperatureVector;
        simulatedTclData.onOff(:,n+1) = onOffValueVector;
        simulatedTclData.massTemp(:,n+1) = massTemperatureVector;
        simulatedTclData.realPowerDraws(:,n+1) =  onOffValueVector.*tclParameters.P_power_draw ;
        simulatedTclData.reactivePowerDraws(:,n+1) =  onOffValueVector.*tclParameters.Q_power_draw ;
        simulatedTclData.timeUntilUnlocked(:,n+1) =  tclParameters.timeUntilUnlocked;
        simulatedTclData.timeUntilDelayEnds(:,n+1) = tclParameters.timeUntilDelayEnds;
        simulatedTclData.voltage240(:,n+1) = tclParameters.voltage240;
        % store control actions
        simulatedTclData.deltaDeadband(:,n+1) = controlActionsWithComms.deltaDeadband;
        simulatedTclData.deltaSetpoint(:,n+1) = controlActionsWithComms.deltaSetpoint;
        if commNetworkData.scenario == 3 && (strcmp(controllerInformation.(aggString).Type, 'Markov controller') || strcmp(controllerInformation.(aggString).Type,'Markov controller with lockouts') || strcmp(controllerInformation.(aggString).Type,'Markov controller with delays') || strcmp(controllerInformation.(aggString).Type,'Markov controller mixed zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone') || strcmp(controllerInformation.(aggString).Type,'Markov controller 2 zone with lockouts'))
            simulatedTclData.onOffSignal(:,n+1) = (commNetworkData.controlActionsFromController{1,1})';
            simulatedTclData.onOffSignalFromController(:,n+1) = (commNetworkData.controlActionsFromController{1,1})';
        else
            simulatedTclData.onOffSignal(:,n+1) = controlActionsWithComms.onOffSignal;
            simulatedTclData.onOffSignalFromController(:,n+1) = commNetworkData.controlActionsFromController(:,n);
        end
        simulatedTclData.overRide(:,n+1) = controlActionsWithComms.overRide;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Assign Nodal Powers for GridlabD
        
        % use matrix multiplication to calculate the power at each
        % node, using incidence matrices calculated in onInit
        acPowerAllNodes = tclIncidenceMatrix*actualAcPowerDrawsApp;
        backgroundPowerAllNodes = houseIncidenceMatrix * houseParameters.backgroundLoad;
        allNodesPower = acPowerAllNodes + backgroundPowerAllNodes;
        eval(setNodePowerToGLD)
             
        % get the measured load power at the nodes to check the
        % convergence below
        allMeasuredNodePower = [];
        eval(getNodePower) % creates appropriate variable name
       
        % if the load changes by less than 1 W then set it to converged
        if all(abs(allNodesPower) - abs(allMeasuredNodePower) <= 1)
            converged = converged || 1;
        end
        
        % % END Assign Nodal Powers for GridlabD
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % set the next timestep that needs to be calculated (2 time-steps from
        % now if the loads have converged or else the present time-step again
        % if loads have not converged
        if converged == 1           
            % store the grid values for the time-step
            eval(getGridData)
            
            n = n + 1;
            ans = gld.global.clock + generalParameters.timeStepInSec;
        else
            ans = gld.global.clock;
        end
        
    % in this case, we've done our part and just need gld to decide to
    % advance to the next time-step
    else
        ans = gld.global.clock + generalParameters.timeStepInSec;
    end
end