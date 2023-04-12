% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [] = compare_cycling(simulationData)
% Compare cyling bahavior of the given dataset with a baseline.
[duty_cycles, To_dc, durations, durations_on, durations_off] = find_pop_cycling_info(simulationData.simulatedTclData.airTemp, simulationData.simulatedTclData.realPowerDraws, simulationData.outdoorTemperature, simulationData.houseParameters.floorArea(simulationData.tclParameters.tcl_house_mapping), simulationData.tclParameters);  
end

function [dc, To_dc, durations, durations_on, durations_off] = duty_cycles_process(P, To)
%{
This function determines the duty cycles and the duration of the cycles
given the input timeseries of compressor power draws.

Input:
    P : vector of floats
        Power draw values of the compressor
    To : vector of floats
        Outdoor air temperature values.
Output:
    dc : vector of floats
        Duty cycles detected.
    To_dc : vector of floats
        Mean outdoor temperatures of the cycles that correspond to dc.
    durations : vector of ints
        Durations of the detected cycles.
    durations_on : vector of ints
        Durations of the ON part of the detected cycles.
    durations_off : vector of ints
        Durations of the OFF part of the detected cycles.
                                        
Notes:
- all input arguments should have the same number of elements.  
- all output arguments have the same number of elements.                                      
%}
%% find duty cycles dc as well as indices of their start and end (initcross and finalcross)
[dc,initcross,finalcross,nextcross,midlev] = dutycycle(P, 'StateLevels', [0,0.5]);
% make the start and ends of the cycles integers
initcross = int64(initcross); finalcross = int64(finalcross);
%% find indicative quantities of each cycle
% total number of cycles
num_cycles = length(dc);
% initialize
To_dc = zeros(num_cycles-1, 1); 
durations = zeros(num_cycles-1, 1); 
durations_on = zeros(num_cycles-1, 1); % duration of the ON part of the cycle
durations_off = zeros(num_cycles-1, 1);% duration of the OFF part of the cycle
for i = 1 : num_cycles-1
    cycle_start = initcross(i); cycle_end = finalcross(i);
    % find the corresponding temperature of the cycle
    To_dc(i) = mean( To(cycle_start:cycle_end) );
    % find the corresponding durations of the cycle
    durations(i) = initcross(i+1)-initcross(i);
    durations_on(i) = finalcross(i)-initcross(i);
    durations_off(i) = initcross(i+1)-finalcross(i);
end
%% Reshape into column vectors and align
% make duty cycles vector equal in length with the durations
dc = dc(1:end-1);
% reshape into a column vector
dc = reshape(dc, [], 1);
end

function [duty_cycles, To_dc, durations, durations_on, durations_off] = find_pop_cycling_info(airTemp_mat, realPowerDraws_mat, To, floorArea, tclParameters)
% vectors that will include the cycling information from simualted TCLs
duty_cycles = []; To_dc = []; durations = []; 
durations_on = []; durations_off = [];
% vector to keep TCL number corresponding to the entries of the above
tcl_num = [];
for i = 1 : size(airTemp_mat,1) % loop through each TCL
    % process duty cycles from the data just simulated
    [duty_cycles_curr, To_dc_curr, durations_curr, durations_on_curr, durations_off_curr] = duty_cycles_process(realPowerDraws_mat(i,:),To);
    durations = [durations;durations_curr];
    durations_on = [durations_on;durations_on_curr];
    durations_off = [durations_off;durations_off_curr];
    duty_cycles = [duty_cycles;duty_cycles_curr];
    To_dc = [To_dc;To_dc_curr];
    tcl_num = [tcl_num;i*ones(length(durations_curr),1)];
end
%% Plots
figure()
histogram(duty_cycles, 20,'Normalization', 'probability' )    
ylabel('Probability')
xlabel('Duty cycle')
figure()
histogram(durations/60, 20,'Normalization', 'probability' )    
ylabel('Probability')
xlabel('Cycle Duration (min)')
figure()
histogram(durations_on/60, 20,'Normalization', 'probability' )    
ylabel('Probability')
xlabel('Cycle ON Duration (min)')
figure()
histogram(durations_off/60, 20,'Normalization', 'probability' )    
ylabel('Probability')
xlabel('Cycle OFF Duration (min)')
% scatter plots of duty cycle and cycle duration
figure()
scatter(tcl_num, duty_cycles)    
xlabel('TCL number')
ylabel('Duty cycle')
figure()
scatter(tcl_num, durations/60)    
xlabel('TCL number')
ylabel('Cycle duration (min)')
end