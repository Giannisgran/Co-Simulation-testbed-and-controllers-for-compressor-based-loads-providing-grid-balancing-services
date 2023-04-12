% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [] = generatePlots_anyController(simulationData)
% This function generates some useful plots for understanding system
% behaviour. Usable only for the Markov controller.
% 
% Input: 
%   simulationData -> struct containing most simulation info.
% Output: 
%   plots for the simulation given as input
 
close all;
%% Load needed variables from the simulation struct
timeVector = simulationData.timeVector;
timeStepInSec = simulationData.generalParameters.timeStepInSec;
controlTimestepInSec = simulationData.controllerInformation.controlTimeStep;
t=(simulationData.timeVector-simulationData.timeVector(1))./3600; %time vector
timeStepRatio = floor(controlTimestepInSec/timeStepInSec);
t_control = t(2:timeStepRatio:end);
realPowerDraws = simulationData.simulatedTclData.realPowerDraws(:,2:timeStepRatio:end-1);  % real power draw of each TCL at every timestep
totalRealPowerDemand = simulationData.controllerInformation.agg1.totalRealPowerDemand(2:timeStepRatio:end-1)'; % we start from the 2nd entry because this is when tracking starts, the first point corresponds to the initial condition and should not be included in the comparison with the tracking signal
Ptotal_des = reshape(simulationData.controllerInformation.agg1.Ptotal_des(1:length(t_control)), [], 1);
P_power_draw = simulationData.tclParameters.P_power_draw;
airTemp_mat = simulationData.simulatedTclData.airTemp(:,1:length(t));
PopSize = simulationData.generalParameters.Pop;
numSteps = simulationData.generalParameters.numSteps;
simLength = simulationData.generalParameters.simLength ;
T_max = simulationData.tclParameters.T_max;
T_min = simulationData.tclParameters.T_min;
timeUntilUnlocked_mat = simulationData.simulatedTclData.timeUntilUnlocked(:,1:length(t));
perc_2zone = simulationData.tclParameters.perc_2zone; % percentage of houses with A/C that are 2-zone
if perc_2zone > 0 
    tcls_2zone_first = simulationData.tclParameters.tcls_2zone_first;
    tcls_2zone_second = simulationData.tclParameters.tcls_2zone_second;
    tcls_2zone = sort([tcls_2zone_first tcls_2zone_second]);
    tcls_1zone = simulationData.tclParameters.tcls_1zone;
    is2zone = simulationData.tclParameters.is2zone; % bool vector with 1 if the tcl is 2-zone
    numHouses_1zone = simulationData.tclParameters.numHouses_1zone; % number of single-zone houses
    numHouses_2zone = simulationData.tclParameters.numHouses_2zone; % number of 2-zone houses
end
try 
    timeUntilDelayEnds_mat = simulationData.simulatedTclData.timeUntilDelayEnds(:,1:length(t));
catch
    warning('Time until delay ends not stored.')
end
onOff_mat = simulationData.simulatedTclData.onOff(:,1:length(t));
lockoutTimeOnInTimeSteps = simulationData.tclParameters.lockoutTimeOnInTimeSteps;
lockoutTimeOffInTimeSteps = simulationData.tclParameters.lockoutTimeOffInTimeSteps;
numCycles = simulationData.regulationSignal.agg1.regNumCycles;
freq = numCycles/(simLength*3600);
ampl = simulationData.regulationSignal.agg1.regAmpl;

%% Print normalized RMS error between resulting total power output and tracking signal
% rms of deviations from the tracking signal: sqrt( errors^2 )
rms_error = rms(totalRealPowerDemand-Ptotal_des);
% normalize according to max-min value of tracking signal
norm_rms_error = rms_error/(mean(Ptotal_des))*100;
% print result
% disp(['Regulation signal amplitude:' num2str(ampl*100,'%.1f') '%'])
% disp(['Regulation signal frequency:' num2str(freq,'%.6f') 'Hz'])
disp(['Normalized RMS error: ' num2str(norm_rms_error,'%.3f') '%'])
%% Compute total number of switches
num_switches = sum(abs(diff(onOff_mat)), 'all');
disp(['Total number of switches: ' num2str(num_switches)])
%% Plots the desired trajectory along with the achieved one 
figure()
plot(t_control,Ptotal_des)
hold on 
plot(t_control,totalRealPowerDemand(1:length(t_control)))
hold off
% vertical line shows the max power draw if everything was one constantly 
%(assumes constant outdoor temperature and thereby constant rated power draw)
% yline( sum(P_power_draw), '--' ); 
xlabel('Time (h)')
ylabel('Total Real Power Demand (kW)')
legend('Regulation Signal','Output Signal')

%% Plot the temperature trajectories for all TCLs
% normalized air temperature
airTemp_normed_mat = (airTemp_mat-repmat(T_min,1,size(airTemp_mat,2))) ./ (repmat(T_max,1,size(airTemp_mat,2))-repmat(T_min,1,size(airTemp_mat,2)));
figure()
plot(t,airTemp_normed_mat)
yline(0, 'LineWidth', 2);
yline(1, 'LineWidth', 2);
xlabel('Time (h)')
ylabel('Normalized air temperature')
%% Plot the number of locked TCLs
figure()
plot(t,sum( (timeUntilUnlocked_mat>0),1 )/PopSize*100)
xlabel('Time (h)')
ylabel('Percentage of the population that is locked (%)')

%% Plot availability
% controllable TCLs for power increase
conUp   = onOff_mat==0 & timeUntilUnlocked_mat==0;
% controllable TCLs for power decrease
conDown = onOff_mat==1 & timeUntilUnlocked_mat==0;

figure()
subplot(2,1,1)
plot(t, 100*sum(conUp)/PopSize)
xlabel('Time (h)')
ylabel('Availability for switching ON (%)')
subplot(2,1,2)
plot(t, 100*sum(conDown)/PopSize)
xlabel('Time (h)')
ylabel('Availability for switching OFF (%)')

%% Plots for the case of mixed-zone population
if perc_2zone > 0 && perc_2zone < 1 % mixed zones
    % plot the total power for each group
    totalRealPowerDraw_1zone = sum(realPowerDraws(~is2zone,:));
    totalRealPowerDraw_2zone = sum(realPowerDraws(is2zone,:));
    figure() 
    plot(t_control, totalRealPowerDraw_1zone)
    hold on 
    plot(t_control, totalRealPowerDraw_2zone)
    hold off
    xlabel('Time (h)')
    ylabel('Total Real Power Demand (kW)')   
    legend('Single-zone', '2-zone')
    
    % plot the number of ON TCLs for each group
    on_1zone = sum(onOff_mat(tcls_1zone,:),1); % number of single zone TCLs that are ON at every timestep
    on_2zone = sum(onOff_mat(tcls_2zone_first,:) | onOff_mat(tcls_2zone_second,:),1);  % consumes power if either zone is ON
    figure() 
    plot(t, 100*on_1zone/numHouses_1zone)
    hold on 
    plot(t, 100*on_2zone/numHouses_2zone)
    hold off
    xlabel('Time (h)')
    ylabel('Percentage of ON TCLs')   
    legend('Single-zone', '2-zone')
    
    % plot the percentage of locked TCLs for each group
    locked_1zone = sum(timeUntilUnlocked_mat(tcls_1zone,:)>0,1); % number of locked single-zone TCLs
    locked_2zone = sum(timeUntilUnlocked_mat(tcls_2zone_first,:)>0 | timeUntilUnlocked_mat(tcls_2zone_second,:)>0, 1);  % number of locked 2-zone TCLs
    figure() 
    plot(t, 100*locked_1zone/numHouses_1zone)
    hold on 
    plot(t, 100*locked_2zone/numHouses_2zone)
    hold off
    xlabel('Time (h)')
    ylabel('Percentage of locked TCLs (%)')   
    legend('Single-zone', '2-zone')
end


end
