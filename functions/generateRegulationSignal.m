% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function  regulationSignal = generateRegulationSignal(simulationOptions, regSignal, regAmpl, regNumCycles, generalParameters, tcl_base_cons)
% This function creates tracking signals that will be used to test
% the implemented controllers
% Output: signals-> each row correspodns to one tracking signal
% Input: simuationOptions, generalParameters
% The switch below has the following cases:
%        regSignal = 1; triangular_option
%        regSignal = 2; sinusoid_option
%        regSignal = 3; pjm_option
%        regSignal = 4; square signal
%        regSignal = 4; none -> no signal

timeStepInSec = simulationOptions.controlTimeStep;
numSteps = floor(generalParameters.simLength*3600/timeStepInSec);

switch regSignal
    case 1 % triangular_option
        t = linspace(0,(numSteps-1)*timeStepInSec,numSteps);        
        % max deviation as a percentage of base TCL consumption
        amplitude_tr = regAmpl;
        % number of triangles in the tracking signal
        cycles_tr = regNumCycles;
        % Define some parameters that define the triangle wave.
        elementsPerHalfPeriod = ceil(numSteps/(cycles_tr*2)); % Number of elements in each rising or falling section.
        amplitude = tcl_base_cons*amplitude_tr*2; % Peak-to-peak amplitude.
        verticalOffset = tcl_base_cons-tcl_base_cons*amplitude_tr; % Also acts as a phase shift.
        numberOfPeriods = cycles_tr; % How many replicates of the triangle you want.
        % % Construct one cycle, up and down.
        risingSignal = linspace(0, amplitude, elementsPerHalfPeriod+1);
        fallingSignal = linspace(amplitude, 0, elementsPerHalfPeriod+1);
        % % Combine rising and falling sections into one single triangle.
        oneCycle = [risingSignal(1:end-1), fallingSignal(1:end-1)] + verticalOffset;
        % now replicate this cycle several (numberOfPeriods) times.
        triangleWaveform = [oneCycle(1) repmat(oneCycle, [1 numberOfPeriods])];
        signals_tr =triangleWaveform(1:numel(t)+1);
        % set the output of the function
        regulationSignal = signals_tr;
        
    case 2 % sinusoid_option
        % max deviation as a percentage of base TCL consumption
        amplitude_sin = regAmpl;
        % number of cycles in the tracking signal
        cycles_sin = regNumCycles;
        % create triangular waves for each combination of amplitude and number of triangles
        x = -pi : 2*pi/numSteps : pi;
        if mod(cycles_sin,2)~=0
            signals_sin = tcl_base_cons*(1+amplitude_sin*sin(cycles_sin*x));
        else
            signals_sin = tcl_base_cons*(1+amplitude_sin *sin(-cycles_sin*x));
        end
        % set the function output
        regulationSignal = signals_sin;

    case 3 % pjm_option
        % import pjm signals from excel
        if simulationOptions.useGLD == 1
            pjm_signalData = load('../../dataFiles/pjmRegulationSignalData2.mat');
        else
            pjm_signalData = load('./dataFiles/pjmRegulationSignalData2.mat');
        end 
        pjm_unscaled  = pjm_signalData.pjm_signalData2;
        % regulation capacity: percent of rated TCL power that's available
        reg_cap = regAmpl*tcl_base_cons; 
        % scale the pjm signal according to base load and regulation capacity
        signals_pjm = (pjm_unscaled(1:timeStepInSec:end)*reg_cap)+tcl_base_cons;
        signals_pjm = signals_pjm(1:numSteps+1);
        % set the function output
        regulationSignal = signals_pjm(:);

    case 4 % step function
        % number of timesteps before changing value
        portion = ceil(numSteps/regNumCycles/6);  % number of timesteps
        % single period: 0 -> 1 -> 0 -> 0 -> -1 -> 0
        step_unscaled_single = [zeros(portion,1); ones(portion,1); zeros(portion,1); zeros(portion,1); -1* ones(portion,1); zeros(portion,1)];
        % repeat as many times needed in order to cover the whole
        % simulation horizon
        step_unscaled_extended = repmat(step_unscaled_single, ceil(numSteps/portion/6), 1);
        % keep only the part needed for the desired simulation duration
        step_unscaled = [0; step_unscaled_extended(1:numSteps)];
        % offered regulation capacity
        reg_cap = regAmpl*tcl_base_cons;
        % resulting scaled regulation signal
        regulationSignal = (step_unscaled*reg_cap)+tcl_base_cons;
        
    case 5 % none
        regulationSignal = [];

end

end