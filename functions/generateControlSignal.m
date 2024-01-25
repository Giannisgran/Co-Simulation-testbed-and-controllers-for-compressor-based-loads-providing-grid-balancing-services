% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [controlActions, controllerInformation] = generateControlSignal(generalParameters, controllerInformation, simulatedTclData, tclParameters, onOffValueVector,airTemperatureVector, binModel,controlActions, massTemperatureVector)

% This function implements the control signal that will be send to the TCLs
% according to the desired controller type
% Input:
%   generalParameters: struct
%       General info regarding the simulation. e.g. timesteps, feeder, TCL population size etc.
%   controllerInformation: struct 
%       Includes info regarding the controller. e.g. type, gains, TCLs assigned etc.
%   simulatedTclData: struct
%       Stored data, results of the simulation e.g. history of ON/OFF modes and air temperatures
%   tclParameters: struct 
%       Includes the characteristics of the TCL population. e.g. thermal parameters, power draws etc.
%   onOffValueVector: vector 
%       Current ON/OFF (1 or 0) mode of each TCL
%   airTemperatureVector: vector 
%       Current values of the regulated air temperature at each TCL
%   binModel: struct
%       Contains information regarding the binModel. e.g. A matrices, bin edges etc.
%   controlActions: struct
%       Includes current control signals, e.g. switching probabilities
%   massTemperatureVector: vector 
%       Current values of the mass temperature at each TCL

% current control timestep
n = controllerInformation.timeStep;
% get the number of TCLs in the population and the indices for those tcl
numTcls = controllerInformation.Pop;
tclIndices = controllerInformation.tclsAssigned;

% rename some variables for some cases below
theta_a = airTemperatureVector;
theta_m = massTemperatureVector;
m = onOffValueVector;


% actions according to the desired controller type
if strcmp(controllerInformation.Type , 'PID')
    % load controller info
    Kp = controllerInformation.Kp;  % proportional gain
    Kint = controllerInformation.Kint; % integral gain
    Kd = controllerInformation.Kd;  % derivative gain
    Dt = generalParameters.timeStepInSec; % controller timestep
    Pset = controllerInformation.Ptotal_des(n); % setpoint
    Pon_av = controllerInformation.Pon_av; % average consumption of ON TCLs
    % current measurement
    Pmeas = sum(simulatedTclData.realPowerDraws(tclIndices,n));
    % current error
    e = Pset - Pmeas;
    % previous error, make sure it work for n=1
    e_prev = controllerInformation.e(max(n-1, 1));
    % store current error
    controllerInformation.e(n) = e;
    % form PID control term
    u = Kp*e + Kint*sum(e) + Kd*(e-e_prev)/Dt;
    % transform into probability
    controlActions.onOffSignal = u/(numTcls*Pon_av);

elseif strcmp(controllerInformation.Type , 'Markov controller') || strcmp(controllerInformation.Type , 'Markov controller with lockouts') || strcmp(controllerInformation.Type , 'Markov controller with delays')    
    % load Pon_av determined in update_binModel_states according to current
    % outdoor temeprature
    Pon_av = binModel.Pon_av_current;
    % predicted consumption for next time step
    Ppred = binModel.Ppred(n);
    % gain of the controller
    Kp = controllerInformation.Kp;
    % integral gain
    Kint = controllerInformation.Kint;
    % total power signal we need to track
    Ptotal_des = controllerInformation.Ptotal_des(n);  
    % update the error for the integrator with the previous timestep info
    if (n>1 && controllerInformation.Kint~=0)
        % desired power draw from the last timestep
        Pdes_prev = controllerInformation.Ptotal_des(n-1);
        if (binModel.fullStateInfo)
            % measured power draw of the assigned TCLs in the previous step
            Pmeas_prev = sum(simulatedTclData.realPowerDraws(tclIndices,n));
            % update the integral error 
            controllerInformation.e_int(n) = controllerInformation.e_int(n-1) + (Pmeas_prev-Pdes_prev); 
        else
            % predicted power draw
            Ppred_prev = binModel.estimatedPagg(n);
            % update the integral error 
            controllerInformation.e_int(n) = controllerInformation.e_int(n-1) + (Ppred_prev-Pdes_prev);
        end
    end
    % load integration error
    e_int = controllerInformation.e_int(n);

    vPlus = controllerInformation.upperVoltage;
    vMinus = controllerInformation.lowerVoltage;
    % load uncontrolled bins - determined during createBinModel
    if Ptotal_des-Ppred > 0 % TCLs will be turned ON
        uncontrolledBins = binModel.uncBinsON;
        uselessBins = binModel.binsON;
        usefullBins = binModel.binsOFF;
    else % TCLs will be turned OFF
        uncontrolledBins = binModel.uncBinsOFF; 
        uselessBins = binModel.binsOFF;
        usefullBins = binModel.binsON;
    end 
    
    % don't take into account TCLs in bins that we are not controlling
    available_tcls = numTcls*( 1-sum(binModel.x([uncontrolledBins uselessBins])) );
    
    % if we're using local voltages, remove the devices that are 
    % outside the voltage range
    if controllerInformation.useLocalVoltage == 1
        numTclsOutsideVoltageRange = sum(simulatedTclData.voltage240(:,n) > vPlus  | simulatedTclData.voltage240(:,n) < vMinus );
    end

    u_goal = (1/(length(usefullBins)-length(uncontrolledBins)))*(Kp*(Ptotal_des-Ppred)-Kint*e_int)/(numTcls*Pon_av); 
	controllerInformation.des_pop_change(n) = (Kp*(Ptotal_des-Ppred)-Kint*e_int)/(numTcls*Pon_av);
    if controllerInformation.availability_adjustment == 0
        u_rel = sign(u_goal)*min(abs(u_goal)./binModel.x,1); % takes care of empty bins also
    else
        u_rel = sign(u_goal)*min(abs(u_goal)./binModel.x,1)./binModel.bin_availability; 
    end
    u_rel(uncontrolledBins) = 0; % zero probability at bins we don't control
    u_rel(uselessBins) = 0; % zero probability at bins that are already at the state we want
    
    % vector of switching probabilities for the desired bin states
    if strcmp(controllerInformation.Type, 'Markov controller')
        controlActions.onOffSignal = u_rel(usefullBins);
        % keep the probabilities with respect to the bins This is used in Kalman instead of u_rel
        if controllerInformation.controlInputAssign == 1
            controlActions.u = u_goal*ones(binModel.totalBinNumber,1);
        elseif controllerInformation.controlInputAssign == 2
            controlActions.u = u_goal*binModel.x;
        end
    elseif strcmp(controllerInformation.Type, 'Markov controller with lockouts')
        controlActions.onOffSignal = abs(u_rel);
        u_goal = abs(u_goal); % don't want negative probs for this one
        % keep the probabilities with respect to the bins This is used in Kalman instead of u_rel
        if controllerInformation.controlInputAssign == 1
            controlActions.u = u_goal*ones(binModel.totalBinNumber,1);
        elseif controllerInformation.controlInputAssign == 2
            controlActions.u = u_goal*binModel.x;
        end
    elseif strcmp(controllerInformation.Type, 'Markov controller with delays')
        controlActions.onOffSignal = abs(u_rel);
        u_goal = abs(u_goal); % don't want negative probs for this one
        % keep the probabilities with respect to the bins This is used in Kalman instead of u_rel
        if controllerInformation.controlInputAssign == 1
            controlActions.u = u_goal*ones(binModel.totalBinNumber,1);
        elseif controllerInformation.controlInputAssign == 2
            controlActions.u = u_goal*binModel.x;
        end
    end
    controlActions.u(uncontrolledBins) = 0; % zero probability at bins we don't control
    controlActions.u(uselessBins) = 0;

    % store u_rel
    controllerInformation.u_rel_mat(:,n) = u_rel;
    
elseif strcmp(controllerInformation.Type , 'Markov controller 2 zone') || strcmp(controllerInformation.Type, 'Markov controller 2 zone with lockouts')    % bin number
    binNumber = binModel.binNumber;
    % redefine variables
    N1 = binNumber;
    N2 = binNumber;
    % total number of TCLs
    numTcl = length(airTemperatureVector);
    % load Pon_av determined in update_binModel_states according to current outdoor temeprature
    Pon_av = binModel.Pon_av_current;
    % predicted consumption for next time step
    Ppred = binModel.Ppred(n);
    % gain of the controller
    Kp = controllerInformation.Kp;
    % total power signal we need to track
    Ptotal_des = controllerInformation.Ptotal_des(n); 
    % difference in desired and projected aggregate power
    dPower = Ptotal_des - Ppred;
    % compute control signal using uniform controller
    if dPower > 0 % TCLs will be turned ON
        %Compute control action (a vector of switching probability)
        x_aggr_temp = binModel.x_proj(1:N1*N2).*(binModel.q2_OFF + binModel.q3_OFF + binModel.q4_OFF); % this has 0 or 1 depending on if the corresponding bin of (0,0) is controlled
        u_ON_val = (Kp*dPower/(Pon_av*numTcl))/sum(x_aggr_temp);
        u_ON_val = u_ON_val*ones(N1*N2,1);
        %Map control to their corresponding 
        u2_ON_val = binModel.q2_OFF.*u_ON_val;
        u3_ON_val = binModel.q3_OFF.*u_ON_val;
        u4_ON_val = binModel.q4_OFF.*u_ON_val;
        %Create vector u_rel
        u_rel = [u2_ON_val;u3_ON_val;u4_ON_val; zeros(3*N1*N2,1)];
        % option to adjust control vector based on bin availability
        % no difference when using the lockout model
        u_rel(1:3*N1*N2) = u_rel(1:3*N1*N2)./repmat(binModel.bin_availability(1:N1*N2),3,1); 
    else % TCLs will be turned OFF
        % auxillary variable: basically includes the values for the states
        % that will be controlled
        x_aggr_temp = binModel.x_proj(N1*N2+1:2*N1*N2).*binModel.q1_ON_FROM_Q2 + ...
                      binModel.x_proj(2*N1*N2+1:3*N1*N2).*binModel.q1_ON_FROM_Q3 + ...
                      binModel.x_proj(3*N1*N2+1:4*N1*N2).*binModel.q1_ON_FROM_Q4;
        %Compute control action
        u_OFF_val = -(Kp*dPower/(Pon_av*numTcl))/sum(x_aggr_temp);
        u_OFF_val = u_OFF_val*ones(N1*N2,1);
        %Map control to their corresponding 
        u1_OFF_FROM_2_val = binModel.q1_ON_FROM_Q2.*u_OFF_val;
        u1_OFF_FROM_3_val = binModel.q1_ON_FROM_Q3.*u_OFF_val;
        u1_OFF_FROM_4_val = binModel.q1_ON_FROM_Q4.*u_OFF_val;
        %Create vector u_rel
        u_rel = [zeros(3*N1*N2,1); u1_OFF_FROM_2_val;u1_OFF_FROM_3_val;u1_OFF_FROM_4_val];
        % option to adjust control vector based on bin availability
        if controllerInformation.availability_adjustment == 1 && strcmp(controllerInformation.Type , 'Markov controller 2 zone') 
            u_rel(3*N1*N2+1:end) = u_rel(3*N1*N2+1:end)./binModel.bin_availability(N1*N2+1:end); 
        elseif controllerInformation.availability_adjustment == 1 && strcmp(controllerInformation.Type , 'Markov controller 2 zone with lockouts')
            u_rel(3*N1*N2+1:end) = u_rel(3*N1*N2+1:end)./binModel.bin_availability(N1*N2+1:5*N1*N2);
        end
    end 
    % vector of probabilities that correspond to states, not tcls
    controlActions.onOffSignal = u_rel;
    % store u_rel
    controllerInformation.u_rel_mat(:,n) = u_rel;

elseif strcmp(controllerInformation.Type , 'Markov controller mixed zone')
    % load the variable that indicates which entries correspond to 2-zones
    is2zone = tclParameters.is2zone;
    % update the timestep in the individual bin models
    controllerInformation.controllerInformation_1zone.timeStep = n;
    controllerInformation.controllerInformation_2zone.timeStep = n;
    % store the total power adjustment needed by the whole population
    controllerInformation.dP(n) = controllerInformation.Ptotal_des(n) - binModel.binModel_1zone.Ppred(n) - binModel.binModel_2zone.Ppred(n);
    % reduce the desired power for each bin model in order to account for the power of the other one
    % When the function is called for each model: dPower = Ptotal_des - Ppred_binModel_1zone - Ppred_binModel_2zone
    % the allocation portion is incorporated in Kp later in the function call for each model
    controllerInformation.controllerInformation_1zone.Ptotal_des(n) = controllerInformation.Ptotal_des(n) - binModel.binModel_2zone.Ppred(n);
    controllerInformation.controllerInformation_2zone.Ptotal_des(n) = controllerInformation.Ptotal_des(n) - binModel.binModel_1zone.Ppred(n);
    % determine how the control input will be allocated to each population
    if controllerInformation.weight_option == 1  % allocation based on percentage of total population
        controllerInformation.alloc_1zone(n) = 1-tclParameters.perc_2zone;
        controllerInformation.alloc_2zone(n) = tclParameters.perc_2zone;      
    elseif controllerInformation.weight_option == 2  % allocation based on average power draw of each group
        controllerInformation.alloc_1zone(n) = binModel.binModel_1zone.Ptotal_av_current/(binModel.binModel_1zone.Ptotal_av_current + binModel.binModel_2zone.Ptotal_av_current);
        controllerInformation.alloc_2zone(n) = binModel.binModel_2zone.Ptotal_av_current/(binModel.binModel_1zone.Ptotal_av_current + binModel.binModel_2zone.Ptotal_av_current);                
    elseif controllerInformation.weight_option == 3  % allocation based on available power increase/decrease of each group
        % determine availability for each population based on the sign of the desired power change
        avail_1zone = (controllerInformation.dP(n)>=0)* binModel.binModel_1zone.available_tcls_ON(n) + (controllerInformation.dP(n)<0)* binModel.binModel_1zone.available_tcls_OFF(n);
        avail_2zone = (controllerInformation.dP(n)>=0)* binModel.binModel_2zone.available_tcls_ON(n) + (controllerInformation.dP(n)<0)* binModel.binModel_2zone.available_tcls_OFF(n);
        controllerInformation.alloc_1zone(n) = avail_1zone*binModel.binModel_1zone.Ptotal_av_current/(avail_1zone*binModel.binModel_1zone.Ptotal_av_current + avail_2zone*binModel.binModel_2zone.Ptotal_av_current);
        controllerInformation.alloc_2zone(n) = avail_2zone*binModel.binModel_2zone.Ptotal_av_current/(avail_1zone*binModel.binModel_1zone.Ptotal_av_current + avail_2zone*binModel.binModel_2zone.Ptotal_av_current);                
    else
        error('Invalid weight option for Markov controller with mixed population.')
    end 
    controllerInformation.controllerInformation_1zone.Kp = controllerInformation.alloc_1zone(n)*controllerInformation.Kp_1zone;
    controllerInformation.controllerInformation_2zone.Kp = controllerInformation.alloc_2zone(n)*controllerInformation.Kp_2zone;
    % store the power total adjustment for each group - this is just for post-processing
    controllerInformation.dP1(n) = controllerInformation.dP(n) * controllerInformation.alloc_1zone(n);  % required power change for the 1zone population
    controllerInformation.dP2(n) = controllerInformation.dP(n) * controllerInformation.alloc_2zone(n);  % required power change for the 2zone population
    % generate control vectors for each bin model
    [controlActions_1zone, controllerInformation.controllerInformation_1zone] = generateControlSignal(generalParameters, controllerInformation.controllerInformation_1zone, simulatedTclData, tclParameters, onOffValueVector(~is2zone),airTemperatureVector(~is2zone), binModel.binModel_1zone,controlActions, massTemperatureVector(~is2zone));
    [controlActions_2zone, controllerInformation.controllerInformation_2zone] = generateControlSignal(generalParameters, controllerInformation.controllerInformation_2zone, simulatedTclData, tclParameters, onOffValueVector(is2zone),airTemperatureVector(is2zone), binModel.binModel_2zone,controlActions, massTemperatureVector(is2zone));
    controlActions.onOffSignal = [controlActions_1zone.onOffSignal;controlActions_2zone.onOffSignal]; % combine the signals
    controlActions.onOffSignal_1zone = controlActions_1zone.u;
    controlActions.onOffSignal_2zone = controlActions_2zone.onOffSignal;
    controllerInformation.u_rel_mat(:,n) = [controllerInformation.controllerInformation_1zone.u_rel_mat(:,n);controllerInformation.controllerInformation_2zone.u_rel_mat(:,n)];

elseif strcmp(controllerInformation.Type, 'PEM_E-T controller') %Extended PEM scheme

    %%%%% Get PEM deadband info
    usePEMTempDeadband = controllerInformation.usePEMTempDeadband;
    if usePEMTempDeadband == 1
        pemT_max = controllerInformation.T_max;
        pemT_min = controllerInformation.T_min;
    elseif usePEMTempDeadband == 0
        pemT_max = tclParameters.T_max;
        pemT_min = tclParameters.T_min;
    end
    
    %Aggregator request-evaluation & decision broadcast
    if controllerInformation.TCLRequest(1,n) == 0
        ControllableTCLs = [];
    else
        ControllableTCLs = controllerInformation.TCLRequest(:,n);
        ControllableTCLs = ControllableTCLs(ControllableTCLs ~= 0);
    end

    if controllerInformation.TCLOffRequest (1,n) == 0
        TurnOffTcls = [];
    else
        TurnOffTcls = controllerInformation.TCLOffRequest(:,n);
        TurnOffTcls = TurnOffTcls(TurnOffTcls~=0);
    end
    TPD =  onOffValueVector(tclIndices)'*tclParameters.P_power_draw(tclIndices); %Total Power Draw
    gridcapacity = controllerInformation.Ptotal_des(n) - TPD; %Reference Signal - Total Power Draw
    controllerInformation.gridcapacity(n) = gridcapacity; %Store gridcapacity at each timestep

    if gridcapacity > 0 %Approve TCL Energy Requests
        gridTCLcapacity = ceil(gridcapacity/controllerInformation.APD); %Nos of TCLs to be approved based on Average Power Draw
        if length(ControllableTCLs) >= gridTCLcapacity
            turnonindex = randperm(length(ControllableTCLs),gridTCLcapacity); %randomly select TCLs to be turned ON amongst reuesting TCLs
        else %gridTCLcapacity is more than controllableTCLs/ramp-rate, so turn all on
            turnonindex = randperm(length(ControllableTCLs),length(ControllableTCLs));
        end
        if ~isempty(turnonindex) > 0
            controllerInformation.turnonindex(1:length(turnonindex),n) = turnonindex;
            controlActions.onOffSignal(ControllableTCLs(turnonindex)) = 1;
        end
    elseif gridcapacity < 0 %Approve Turn-off requests to shave power
        gridTCLcapacity = ceil(abs(gridcapacity/controllerInformation.APD));
        if length(TurnOffTcls) >= abs(gridTCLcapacity)
            turnoffindex = randperm(length(TurnOffTcls),abs(gridTCLcapacity)); %randomly select TCLs to be turned off amongst TCLs which just turned on
        else %need to turn off more tcls than availble turn-off tcls, ramp-down rate
            turnoffindex = randperm(length(TurnOffTcls),length(TurnOffTcls));
        end
        if ~isempty(turnoffindex)
            controllerInformation.turnoffindex(1:length(turnoffindex),n) = turnoffindex;
            controlActions.onOffSignal(TurnOffTcls(turnoffindex)) = -1;
        end
    else
        gridTCLcapacity = 0; %power balanced, deny all turn-on/off requests
    end
    controllerInformation.gridTCLcapacity(n) = gridTCLcapacity; %Storing gridTCLcapacity at each timestep

elseif strcmp(controllerInformation.Type , 'none')

end

end