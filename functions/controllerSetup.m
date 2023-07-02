% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [controllerInformation,tclParameters] = controllerSetup(type,tclParameters,generalParameters,controllerInformation,numTempIntervals,numTempIntervals2zone)
% Preliminary controller setup depending on the controller type given as input
% Input:
%   type : str
%       Desired type of the controller to be used. Determined in main_runSim
%   tclParameters: struct 
%       Includes the characteristics of the TCL population. e.g. thermal parameters, power draws etc.
%   generalParameters: struct
%       General info regarding the simulation. e.g. timesteps, feeder, TCL population size etc.
%   controllerInformation: struct 
%       Includes info regarding the controller. e.g. TCLs assigned, population size etc.
%   numTempIntervals: int
%       Number of bins for the binModel. Determined in main_runSim.
%   numTempIntervals2zone: int
%       Number of bins for the 2-zone binModel. Determined in main_runSim.
% Output: 
%   controllerInformation: struct
%       Inlcludes the controller specifications and necessary structs.
%   tclParameters: struct
%       For a specific case with the PEM controllers, may have potentially 
%       altered lockout variables .

if strcmp(type,'PID')
    % keep field with name of controller
    controllerInformation.Type = 'PID'; 
    % voltage controller bounds
    controllerInformation.upperVoltage = 1.04*240;
    controllerInformation.lowerVoltage = 0.96*240; 
    % coefficient for the proportional term
    controllerInformation.Kp = 0.5;  
    % coefficient for the integral term
    controllerInformation.Kint = 0.005; 
    % coefficient for the derivative term
    controllerInformation.Kd = 0; 
    % number of control steps that will be executed in the actual simulation - needed for preallocation
    numSteps = ceil(generalParameters.numSteps * (generalParameters.timeStepInSec/generalParameters.controlTimeStep));
    % initialize error term
    controllerInformation.e = zeros(1, numSteps+1);
    % average consumption of ON TCLs, needed for converting into probabilities
    % use the current state of the population to determine it
    controllerInformation.Pon_av = ( tclParameters.m0'*tclParameters.P_power_draw(controllerInformation.tclsAssigned) )/sum(tclParameters.m0);
    % length of the generated control signal
    controllerInformation.controlSignalLength = 1;
    
elseif strcmp(type,'Markov controller') || strcmp(type, 'Markov controller with lockouts') || strcmp(type, 'Markov controller with delays')
    % voltage controller bounds
    controllerInformation.upperVoltage = 1.04*240;
    controllerInformation.lowerVoltage = 0.96*240;    
    % controller gain
    controllerInformation.Kp = 1;
    % Kalman innovation gain
    controllerInformation.Km = 1;
    % gain for the integrator, set to 0 to deactivate
    controllerInformation.Kint = 0; % 0.01
    % time in seconds needed to receive a new measurement
    time_between_meas = 0; % set to zero for no updates  
    % output adaptation option - set to zero if not needed
    % a=Ka*e_int will be added to the A matrix for correction
    controllerInformation.Ka = 0; % 3*10^(-6)
    % state adaptation: Anew = Kwt*A + (1-Kwt)*Awt
    controllerInformation.Kwt = 0; % weight factor 
    controllerInformation.Nwt = 150; % number of seconds to take into account for Awt
    controllerInformation.availability_adjustment = 0; % adjusts the switching probabilities based on the available TCLs in each bin
    controllerInformation.adjust_Pon_av_option = 1; % recomputes Pon_av at every timestep based on the available measurements
    % end of Markov controller options

    % keep field with name of controller
    controllerInformation.Type = type;   
    
    % number of control steps that will be executed in the actual simulation - needed for preallocation
    numSteps = ceil(generalParameters.numSteps * (generalParameters.timeStepInSec/generalParameters.controlTimeStep));

    % controller name to be passed through the mqqt
    % length of the generated control signal and u_rel preallocation
    if strcmp(type, 'Markov controller') % control signal corresponds only to on or off states
        controllerInformation.controlSignalLength = numTempIntervals;
        controllerInformation.u_rel_mat = zeros(2*numTempIntervals, numSteps);
    elseif strcmp(type, 'Markov controller with lockouts') % control signal includes entries for both on and off states, but only for unlocked tcls 
        controllerInformation.controlSignalLength = 4*numTempIntervals;
        controllerInformation.u_rel_mat = zeros(4*numTempIntervals, numSteps);
    elseif strcmp(type, 'Markov controller with delays') % control signal includes on/off and delayed/not delayed
        controllerInformation.controlSignalLength = 4*numTempIntervals;
        controllerInformation.u_rel_mat = zeros(4*numTempIntervals, numSteps);
    end
    % error for the integrator
    controllerInformation.e_int = zeros(1, numSteps+1);
    % translate measurement period according to the simulation and control timesteps
    controllerInformation.measRecieveTimestep = ceil(time_between_meas/generalParameters.timeStepInSec/generalParameters.controlTimeStep);
    % keep number of bins and bin model edges
    controllerInformation.binNumber = numTempIntervals;
	% store u_goal values
    controllerInformation.des_pop_change = zeros(1, numSteps);
 
    % do some checks to make sure simulation options are compatible with the controller type
    if strcmp(type, 'Markov controller with delays') && tclParameters.temperatureDelayOption == 1
        error('In order to use Markov controller with delays, temperature delay option should not be 1.')
    elseif strcmp(type, 'Markov controller with lockouts') && (tclParameters.lockoutTimeOffInTimeSteps(1) == 0 || tclParameters.lockoutTimeOnInTimeSteps(1) == 0)
        error('In order to use Markov controller with lockouts, ON/OFF lockout need to be non-zero.')
    end
    assert((controllerInformation.Kwt == 0 && generalParameters.controlTimeStep~=generalParameters.timeStepInSec) ||  generalParameters.controlTimeStep==generalParameters.timeStepInSec, 'Need same control and simulation timesteps for state adaptation.');

elseif strcmp(type,'Markov controller 2 zone') || strcmp(type, 'Markov controller 2 zone with lockouts') % controller for exclusively 2-zone population
    % keep field with name of controller
    controllerInformation.Type = type; 
    % voltage controller bounds
    controllerInformation.upperVoltage = 1.04*240;
    controllerInformation.lowerVoltage = 0.96*240;    
    % controller gain
    controllerInformation.Kp = 1;
    % Kalman innovation gain
    controllerInformation.Km = 1;
    % time in seconds needed to receive a new measurement
    time_between_meas = 0; % set to zero for no updates  
    % output adaptation option - set to zero if not needed
    % a=Ka*e_int will be added to the A matrix for correction
    controllerInformation.Ka = 0; % 3*10^(-6)
    % control, adding that to the original A: Anew = Kwt*A + (1-Kwt)*Awt
    controllerInformation.Kwt = 0; % weight factor 
    controllerInformation.Nwt = 150; % number of seconds to take into account for Awt
    controllerInformation.availability_adjustment = 0; % adjusts the switching probabilities based on the available TCLs in each bin
    controllerInformation.adjust_Pon_av_option = 0; % recomputes Pon_av at every timestep based on the available measurements
    assert(~controllerInformation.availability_adjustment || ~strcmp(type, 'Markov controller 2 zone with lockouts'), 'Cannot use availability adjustment with bin model that includes lockouts. Set controllerInformation.availability_adjustment to 0.')
    assert(controllerInformation.Kwt == 0, 'State adaptation not yet implemented for 2-zone controller.')

    % end of Markov controller options
    
    % length of the generated control signal
    controllerInformation.controlSignalLength = 6*numTempIntervals2zone*numTempIntervals2zone;
    % keep number of temperature bins that the deadband is divided into
    controllerInformation.binNumber = numTempIntervals2zone;
    % number of control steps that will be executed in the actual simulation - needed for preallocation
    numSteps = ceil(generalParameters.numSteps * (generalParameters.timeStepInSec/generalParameters.controlTimeStep));
    % error for the integrator
    controllerInformation.e_int = zeros(1, numSteps+1);
    % store u send out 
    controllerInformation.u_rel_mat = zeros(controllerInformation.controlSignalLength, numSteps);
    % translate measurement period according to the simulation and control timesteps
    controllerInformation.measRecieveTimestep = ceil(time_between_meas/generalParameters.timeStepInSec/generalParameters.controlTimeStep);
    % store u_goal values
    controllerInformation.des_pop_change = zeros(1, numSteps);
    
elseif strcmp(type,'Markov controller mixed zone') % controller for population with both single and 2-zone houses
    % Options
    controller_type_1zone = 'Markov controller';  % type of the controller for the single-zone part of the population
    controller_type_2zone = 'Markov controller 2 zone';  % type of the controller for the 2-zone part of the population
    % method to allocate the control input, i.e. adjust the gains of each controller
    % if 1, then the input is allocated based on the percentage of the population in each group
    % if 2, then the input is allocated based on the average power draw of each group
    % if 3, then the input is allocated based on the available power increase/decrease which is computed according to the average power and bin availability
    controllerInformation.weight_option = 3; 
    
    % number of control steps that will be executed in the actual simulation - needed for preallocation
    numSteps = ceil(generalParameters.numSteps * (generalParameters.timeStepInSec/generalParameters.controlTimeStep));
    % Create one controller struct for each population
    [controllerInformation.controllerInformation_1zone,~] = controllerSetup(controller_type_1zone,tclParameters,generalParameters,controllerInformation,numTempIntervals,numTempIntervals2zone);
    [controllerInformation.controllerInformation_2zone,~] = controllerSetup(controller_type_2zone,tclParameters,generalParameters,controllerInformation,numTempIntervals,numTempIntervals2zone);
    % correct the population size for the two controller structs
    controllerInformation.controllerInformation_1zone.Pop = tclParameters.numHouses_1zone;
    controllerInformation.controllerInformation_2zone.Pop = 2*tclParameters.numHouses_2zone;  % doublecount the zones
    % correct the assigned TCLs in the two controller structs
    controllerInformation.controllerInformation_1zone.tclsAssigned = controllerInformation.tclsAssigned(~tclParameters.is2zone(controllerInformation.tclsAssigned));
    controllerInformation.controllerInformation_2zone.tclsAssigned = controllerInformation.tclsAssigned(tclParameters.is2zone(controllerInformation.tclsAssigned));
    % length of the generated control signal - concat the control vectors
    % aimed to the single and 2-zone populations
    controllerInformation.controlSignalLength = controllerInformation.controllerInformation_1zone.controlSignalLength + controllerInformation.controllerInformation_2zone.controlSignalLength;    
    % keep the control vector lengths for each group
    controllerInformation.controlSignalLength_1zone = controllerInformation.controllerInformation_1zone.controlSignalLength;
    controllerInformation.controlSignalLength_2zone = controllerInformation.controllerInformation_2zone.controlSignalLength;
    % store u send out 
    controllerInformation.u_rel_mat = zeros(size(controllerInformation.controllerInformation_1zone.u_rel_mat,1)+size(controllerInformation.controllerInformation_2zone.u_rel_mat,1), numSteps);
    % keep field with name of controller
    controllerInformation.Type = type;
    % keep the gain for each controller in case we want to change it later
    controllerInformation.Kp_1zone = controllerInformation.controllerInformation_1zone.Kp;
    controllerInformation.Kp_2zone = controllerInformation.controllerInformation_2zone.Kp;
    % will store the power adjustment needed to match the reference signal at each timestep
    controllerInformation.dP = zeros(1, numSteps); 
    controllerInformation.dP1 = zeros(1, numSteps); % for the single-zone part
    controllerInformation.dP2 = zeros(1, numSteps); % for the 2-zone part
    % will store the percentage allocations
    controllerInformation.alloc_1zone = zeros(1, numSteps); % for the single-zone part
    controllerInformation.alloc_2zone = zeros(1, numSteps); % for the 2-zone part
    % Do some checks
    assert(tclParameters.perc_2zone > 0 && tclParameters.perc_2zone <1, 'Population should have both single and 2-zone houses for this controller.');
    
elseif strcmp(type,'PEM_E-T controller') %Extended PEM controller    
    % keep field with name of controller
    controllerInformation.Type = 'PEM_E-T controller';
    controllerInformation.MTTR = 600; %Turn-on MTTR in seconds
    controllerInformation.MTTRoff = 1; %Turn-off MTTR in seconds
    controllerInformation.minepoch = 180; %Min epoch length in seconds
    controllerInformation.epoch = 600; %Max Epoch length in seconds
    tclParameters.lockoutTimeOnInTimeSteps = (controllerInformation.minepoch/generalParameters.timeStepInSec) * ones(generalParameters.Pop,1); %minEpoch = lockoutTimeOn
    %tclParameters.lockoutTimeOffInTimeSteps = (0) * ones(generalParameters.Pop,1);
    controllerInformation.pop = generalParameters.Pop;  %Nos of ACs
    controllerInformation.dt = generalParameters.timeStepInSec; %Controller timestep
    controllerInformation.APD = mean(tclParameters.P_power_draw); %Average Power Draw of TCL Population

    %%%%% Voltage deadband limits
    controllerInformation.upperVoltage = 1.04*240;
    controllerInformation.lowerVoltage = 0.96*240;

    % set a value for PEM dadband constriction (how much we want to reduce the
    % deadband by)
    controllerInformation.deadbandReduction = 0.8;

    % calculate the new deadband width, and calculate new upper and lower
    % limits for control
    restrictedDeadband = (tclParameters.deadband*controllerInformation.deadbandReduction);
    if controllerInformation.usePEMTempDeadband == 1
        controllerInformation.T_max = tclParameters.T_sp + restrictedDeadband/2;
        controllerInformation.T_min = tclParameters.T_sp - restrictedDeadband/2;
    else
        controllerInformation.T_max = tclParameters.T_max;
        controllerInformation.T_min = tclParameters.T_min;
    end

    % length of the generated control signal
    controllerInformation.controlSignalLength = generalParameters.Pop;
    
elseif strcmp(type,'none')
    controllerInformation.Type = 'none';
    % length of the generated control signal
    controllerInformation.controlSignalLength = generalParameters.Pop;
    % if the population has both single and 2-zone houses, keep one separate struct for each
    if tclParameters.perc_2zone > 0 && tclParameters.perc_2zone < 1
        controllerInformation.controllerInformation_1zone = controllerInformation;
        controllerInformation.controllerInformation_2zone = controllerInformation;
    end

end

end
