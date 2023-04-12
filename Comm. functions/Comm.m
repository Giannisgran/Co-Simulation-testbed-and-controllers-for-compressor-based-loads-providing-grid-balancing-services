% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [controlActionsWithComms, commNetworkData] = Comm(n,nc,generalParameters,controlActionsSent, controllerType,commNetworkData)

if nc == 0 %% Centralized topology
    % implement delays by shifting where the control actions get places in
    % the arrival matrix based on the realized delays. 
    CM = commNetworkData.CM;
    MDM = commNetworkData.MDM;
    SDM = commNetworkData.SDM;
    if strcmp(commNetworkData.distribution,'uniform')
        % sqrt(SDM*12) converts std dev of uniform dist. into range
        DM = rand(size(CM(1,2:end),1), size(CM(1,2:end),2)) .* ...
            sqrt(SDM(1,2:end)*12) + MDM(1,2:end)-sqrt(SDM(1,2:end)*12)/2;
    else
        DM = normrnd(0,1,size(CM(1,2:end),1), size(CM(1,2:end),2)) .* ...
            SDM(1,2:end) + MDM(1,2:end);
    end
    DM = ceil(DM); % added this in to make sure delays are integers
    DM(DM<0) = 0;
    delaysToTcls = DM;
    
    % implement dropouts from controller to nodes by setting controll
    % actions to 0 where ever dropout occurs.
        
    DDM = commNetworkData.DDM(1,2:end)';
    DDR = rand(size(CM(1,2:end)',1), size(CM(1,2:end)',2)) <= DDM;
    dropoutsToTcls = DDR;

    %store sampled delay and dropout realizations used in each timestep
    commNetworkData.delaysToTcls(:,n)=delaysToTcls;
    commNetworkData.dropoutsToTcls(:,n)=dropoutsToTcls;
    
    % initialize version of the control action vector with all entries 
    % (except overRides) set to 0 that we will then assign below with the
    % comm network effects. 
    controlActionsWithComms.deltaDeadband =  0.*controlActionsSent.deltaDeadband;
    controlActionsWithComms.deltaSetpoint = 0.*controlActionsSent.deltaSetpoint;
    controlActionsWithComms.onOffSignal =  0.*controlActionsSent.onOffSignal;
    controlActionsWithComms.overRide = controlActionsSent.overRide; % exclude this because local to house
    
    % throw warning if we're using control actions that we haven't build
    % into the comms system yet
    if any(controlActionsSent.deltaDeadband) && any(controlActionsSent.deltaSetpoint)
        warning('Deadband and setpoint adjustments not built into comms yet')
    end
        
    % store the original signal for analysis
    if strcmp(controllerType, 'Markov controller') || strcmp(controllerType,'Markov controller with lockouts') || strcmp(controllerType,'Markov controller with delays') || strcmp(controllerType,'Markov controller mixed zone') || strcmp(controllerType,'Markov controller 2 zone') || strcmp(controllerType,'Markov controller 2 zone with lockouts') %For Markov Controller, Control signal length is bin length not tcl pop
        commNetworkData.controlActionsFromController(:,n) = {controlActionsSent.onOffSignal'};
        controlActionsFromControlleronOffSignal = commNetworkData.controlActionsFromController(:,n);
    else
        commNetworkData.controlActionsFromController(:,n) = ones(generalParameters.Pop,1).*controlActionsSent.onOffSignal;
        controlActionsFromControlleronOffSignal = commNetworkData.controlActionsFromController(:,n);
    end
    
    % control actions received has a row for each TCL, and a column for
    % each time-step, and so we just need to assign each entry in the
    % control with dropout into the appropriate column for each TCL. Again,
    % overwritting is fine becasue we want to use the most recent control
    % action for a time-step    
    
    rows = (1:generalParameters.Pop)';
    columns = (n+delaysToTcls)'; % pick the delays from controller to TCLs
    
    % get rows where there is no dropout and where the delay will not make
    % the control action take effect after the simulation boundary
    rowsNoDropoutsAndInbounds = rows(dropoutsToTcls ==1 & ...
        columns<=repmat(size(commNetworkData.controlActionsFromController,2),numel(columns),1));

    % convert subscripts to indices (1-d selection), then remove the
    % indices that correspond to out of bounds and dropout entries
    delayedIndices = rows + (columns-1)*generalParameters.Pop;
    delayedIndicesNoDropoutsAndInbounds = delayedIndices(rowsNoDropoutsAndInbounds);
    
    % now we use the linear indices to put the relevant control actions into the
    % matrix according to when they arrive, where we exclude control
    % actions that never arrive. 
    if strcmp(controllerType, 'Markov controller') || strcmp(controllerType,'Markov controller with lockouts') || strcmp(controllerType,'Markov controller with delays') || strcmp(controllerType,'Markov controller mixed zone') || strcmp(controllerType,'Markov controller 2 zone') || strcmp(controllerType,'Markov controller 2 zone with lockouts') %For Markov Controller, Control signal length is bin length not tcl pop
        commNetworkData.controlActionsToTcls(delayedIndicesNoDropoutsAndInbounds) = ...
        controlActionsFromControlleronOffSignal(rowsNoDropoutsAndInbounds);
        controlActionsWithComms.onOffSignal = cell2mat(commNetworkData.controlActionsToTcls(:,n));
    else
        commNetworkData.controlActionsToTcls(delayedIndicesNoDropoutsAndInbounds) = ...
        controlActionsFromControlleronOffSignal(rowsNoDropoutsAndInbounds);
        controlActionsWithComms.onOffSignal = commNetworkData.controlActionsToTcls(:,n);
    end 
    
else %% Tree topology
    warning('GL did not update tree topology comm networks')    
    %Implementing Communication Network Effects
    x = round(DM(1,1:nc+1)); %Delay vector containing delays between CC and
    %collection nodes at each timestep
    y = round(DM(1:nc+1,:)); %Delay matrix containing delays between
    %Collection nodes and TCLs attached to them
    for j = 2:nc+1
        if x(j) ~= 0 && DDR(1,j) == 0 %No dropout btw CC and
            %Collection node but there is delay
            for t = nc+2:generalParameters.Pop+nc+1
                if y(j,t) == 0&& DDR(j,t) == 0 && CM(j,t)~=0
                    %No dropout and no delay between Collection Nodes
                    %and TCLs & Connection exists btw Collection node and TCL
                    if n + x(j) <= generalParameters.numSteps %Ensuring we stay within total nos of timesteps
                        controlActionsWithComms(t-nc-1,n+x(j)) = controlActionsSent(t-nc-1,n);
                    end
                    %Signal is assigned to the same timestep
                elseif y(j,t)~=0 && DDR(j,t) == 0 %No dropout btw Collection Node and TCL but delay exists
                    if n + x(j)+y(j,t) <= generalParameters.numSteps %Ensuring we stay within total nos of timesteps
                        controlActionsWithComms(t-nc-1,n+x(j)+y(j,t)) = controlActionsSent(t-nc-1,n); %Implementing Delay
                    end
                end
            end
        elseif x(j) == 0 && DDR(1,j) == 0 %No dropout and no delay between CC and collection nodes
            for t = nc+2:generalParameters.Pop+nc+1
                if y(j,t) == 0&& DDR(j,t) == 0 && CM(j,t)~=0 %No dropout and no delay between Collection Nodes and TCLs & Connection exists btw Collection node and TCL
                    controlActionsWithComms(t-nc-1,n) = controlActionsSent(t-nc-1,n); %Signal is assigned to the same timestep
                elseif y(j,t)~=0 && DDR(j,t) == 0 %No dropout btw Collection Node and TCL but delay exists
                    if n+ y(j,t) <= generalParameters.numSteps %Ensuring we stay within total nos of timesteps
                        controlActionsWithComms(t-nc-1,n+y(j,t)) = controlActionsSent(t-nc-1,n); %Implementing Delay
                    end
                end
            end
        end
    end
end

end