% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

% Script to plot results from batch simulations coupled with GridLab-D
% Just fill out the variables allFiles, batchSim and simulationCount

clearvars 
close all
%% Parameters you need to change
%use this if you want output names of figures to include batch ID (e.g. PlottingFn_Figure3-batchSimulation_20201207-simulation5.fig
outputNamesIncludeBatch = false;

%This is where your autogenerated batchSimulation file lives
allFiles = "";

%which specific autogenerated batchSimulation file you are creating plots
%from
batchSim = "";

%how many simulations are in this batchSimulation
simulationCount = 10;
% preallocate for the matrix that will contain performance info
perfMatrix = zeros(simulationCount,13);
%% WARNING DO NOT CLOSE ANY FIGURES AS THEY ARE CREATED. THERE IS A LAG BETWEEN CREATING THEM AND SAVING THEM DUE TO CALCULATIONS!

%% A simple loop through the files to plot from
disp(strcat('Plotting started for ', batchSim));
disp(' ');
    
for simNum = 1:simulationCount
    %creates the name for the simulation # folder in the
    %batchSimulation_... file
    simFolder = join([allFiles, batchSim, strcat('simulation', mat2str(simNum))],"\");
    
    %returns the data on the files in the simFolder
    simFolderData = struct2cell(dir(simFolder));
    %drops everything but the file names
    simFolderData = simFolderData(1,:);
    
    %finds the simulation data .mat file in the folder & flags if there
    %isn't one or there are multiple ones
    dataLocation = 0;
    for names = 1:length(simFolderData)
        if startsWith(simFolderData{names},"simulationData_") && endsWith(simFolderData{names}, ".mat") && dataLocation == 0
            dataLocation = names;
        elseif startsWith(simFolderData{names},"simulationData_") && endsWith(simFolderData{names}, ".mat") && dataLocation ~= 0
            dataLocation = -1;
        end
    end
    
    if dataLocation == 0
        warning(strcat('No simulation data was found for simulation', mat2str(simNum), ". Plotting not performed for this simulation!"));
    elseif dataLocation < 0
        warning(strcat('Multiple simulation data files were found for simulation', mat2str(simNum), ". Plotting not performed for this simulation!"));
    else
        %call the function for plotting
        perfMatrix(simNum,:) = plotSimData(join([simFolder,simFolderData{dataLocation}] , "\"),outputNamesIncludeBatch);
        % store performance metrics of current simulation
        disp(strcat('Plotting complete for simulation', mat2str(simNum)));
    end
end
disp(' ');
disp(strcat('Plotting complete for ', batchSim));
% save(join([allFiles, batchSim,'perfMatrix.mat']),'perfMatrix')

%% The plotting function
function metrics = plotSimData(fileName,outputNamesIncludeBatch)
    %% assign variables
    filePieces = split(fileName, "\");
    fileLocation = join(filePieces(1:end-1), "\");
    load(fileName);
    %% assign variables
    % in some cases there are 1800 voltage elements and in some others
    % 1801, while timevector has 1801
    len = size(simulationData.gridData.commLoad3.voltages,2);
    t=(simulationData.timeVector(1:len)-simulationData.timeVector(1))./3600; %time vector
    regulatortap=simulationData.gridData.regulator1.tapPositions; %regulator tap positions
    timestep = simulationData.generalParameters.timeStepInSec; % simulation timestep in seconds

    capswitch = [];
    for idx = 1:simulationData.feederParameters.numberOfCapacitors
        capString = ['capacitor' num2str(idx)];
        capswitch = [capswitch; simulationData.gridData.(capString).switchPositions]; %capacitor switch positions
    end

    gridElements = fieldnames(simulationData.gridData);
    voltageElements = gridElements(contains(fieldnames(simulationData.gridData),'transformer')==0);
    underVoltageElements = {};
    numUvElements = 0;
    voltagemagTemp=[]; %3-phase voltage mag at all load, cap, etc nodes, in pu
    voltageangTemp=[]; %3-phase voltage ang at all load, cap, etc nodes, in deg
    voltagemag3=[]; %3-phase voltage mag at all load, cap, etc nodes, in pu - 3 phase nodes only
    voltageang3=[]; %3-phase voltage ang at all load, cap, etc nodes, in deg - 3 phase nodes only
    for idx = 1:numel(voltageElements)
        voltageString = voltageElements{idx};
        theseVoltages = simulationData.gridData.(voltageString).voltages;
        nominalVoltage = simulationData.gridData.(voltageString).nominalVoltages; 
        if contains(voltageString,'split')
            nominalVoltage = nominalVoltage*2;
        end
        if contains(voltageString,'comm')
            nominalVoltage = nominalVoltage/sqrt(3);
        end
        
        if all(abs(theseVoltages)./nominalVoltage < .8)
            numUvElements = numUvElements + 1;
            underVoltageElements{numUvElements,1} = voltageString;
        end
        voltagemagTemp=[voltagemagTemp; abs(theseVoltages)./nominalVoltage];
        voltageangTemp=[voltageangTemp; angle(theseVoltages)./pi.*180];
        
        %re-record just 3 phase node voltages for unbalance assessment
        if size(theseVoltages,1)==3  
            voltagemag3=[voltagemag3; abs(theseVoltages)./nominalVoltage];
            voltageang3=[voltageang3; angle(theseVoltages)./pi.*180];
        end
    end

    % remove rows that have all zeros, which means there actually wasn't a
    % conductor
    noPhase = find(~all(voltagemagTemp==0,2));
    voltagemag = voltagemagTemp(noPhase,:);
    voltageang = voltageangTemp(noPhase,:);

    transformers = gridElements(contains(fieldnames(simulationData.gridData),'transformer')==1);
    transpower=[]; %three-phase power going through transformer normalized to transformer power rating
    for idx = 1:numel(transformers)
    transString = transformers{idx};

    thisTransPower = abs(simulationData.gridData.(transString).powerFlow)./1000;
    thisTransRating = simulationData.gridData.(transString).powerRating;
    transpower = [transpower; thisTransPower./thisTransRating]; 
    end

    reference=simulationData.controllerInformation.agg1.Ptotal_des(1:length(t)); %grid signal
    reference = reshape(reference, [], 1);
    aggpower=simulationData.controllerInformation.agg1.totalRealPowerDemand(2:end); %aggregate power consumption of controlled ACs
    aggpower = reshape(aggpower, [], 1);
    aggpowerReactive=simulationData.simulatedTclData.totalReactivePowerDemand(1:length(t)); %aggregate power consumption of controlled ACs
    
    temps=(simulationData.simulatedTclData.airTemp(:,2:end)-simulationData.tclParameters.T_min)./(simulationData.tclParameters.T_max-simulationData.tclParameters.T_min); %temperature trajectories of all ACs, normalized

    %% PLOTS FOR MILESTONE DELIVERABLE

    %tracking performance: plot reference and aggregate power
    figure('Name', strcat(filePieces(end-2),"-", filePieces(end-1)))
    subplot(311)
    plot(t,reference)
    hold on
    plot(t,aggpower)
    legend('reference', 'AC power')
    title('Tracking Performance')
    xlabel('time')
    ylabel('power (kW)')

    %voltages: plot voltages and limits
    subplot(312)
    plot(t,voltagemag)
    hold on
    plot(t,1.05*ones(1,length(t)), 'k--');
    plot(t,0.95*ones(1,length(t)), 'k--');
    title('Voltage Magnitudes')
    xlabel('time')
    ylabel('voltage (pu)')

    %transformers: plot transformer powers and ratings - normalized
    subplot(313)
    plot(t,transpower)
    hold on
    plot(t,ones(1, length(t)), 'k--')
    title('Transformer Power Flow')
    xlabel('time')
    ylabel('power (normalized to transformer rating')


    %% PLOTS FOR OUR ANALYSIS

    %%FIG 2 is TCL-specific data
    figure('Name', strcat(filePieces(end-2),"-", filePieces(end-1)))

    %tracking performance: plot reference and aggregate power
    subplot(211)
    plot(t,reference)
    hold on
    plot(t,aggpower)
    legend('reference', 'AC power')
    title('Tracking Performance')
    xlabel('time')
    ylabel('power (kW)')

    %compute + output RMSE in kW
    rmsetracking=rms(reference-aggpower');

    %compute PJM score
    [passOrFail, pjmScore, accScore, delayScore, precScore] = pjmScoringPassFailv5(reference, aggpower, mean(aggpower), 2);

    %temperature trajectories so we can check if we're going out of dead-band
    %and temperature synhchronization
    subplot(212)
    plot(t,temps)
    hold on
    plot(t, ones(1,length(t)), 'k--') %normalized temp limit - high
    plot(t, zeros(1,length(t)), 'k--') %normalized temp limit - low
    ylim([min(min(temps))-.1, max(max(temps))+.1])
    title('AC Temperatures')
    xlabel('time')
    ylabel('temperature, normalized')


    %% FIG 3 is network-specific data
    figure('Name', strcat(filePieces(end-2),"-", filePieces(end-1)))

    %regulator tapping
    subplot(521)
    plot(t,regulatortap)
    title('Regulator Tapping')
    xlabel('time')
    ylabel('tap position')

    %compute + output total tapping
    totaltapping=sum(diff(regulatortap')',2);

    %capacitor switching
    subplot(522)
    plot(t,capswitch)
    title('Capacitor Switching')
    xlabel('time')
    ylabel('tap position')

    %compute + output total switching
    totalcapswitching=sum(abs(diff(capswitch')'),2);

    %voltages and limits
    subplot(523)
    plot(t,voltagemag)
    hold on
    plot(t,1.05*ones(1,length(t)), 'k--');
    plot(t,0.95*ones(1,length(t)), 'k--');
    title('Voltage Magnitudes')
    xlabel('time')
    ylabel('voltage (pu)')

    %change of voltages and limits
    subplot(524)
    plot(t(2:end),diff(voltagemag')')
    title('Change of Voltage Magnitude')
    xlabel('time')
    ylabel('voltage (pu)')

    %compute + plot voltage unbalance
    vuf=zeros(size(voltagemag3,1)/3,length(t));
    for n = 1:size(voltagemag3,1)/3
        voltagemag3this=voltagemag3(3*n-2:3*n,:);
        voltageang3this=voltageang3(3*n-2:3*n,:);
        if sum(find(voltagemag3this<.5))>0 %check if phase missing -- if so, return nan for vuf
            vuf(n,:)=NaN(1,length(t));
        else
            voltagephasor=voltagemag3this.*(cosd(voltageang3this)+j*sind(voltageang3this));
            posseq=(voltagephasor(1,:)+(cosd(120)+j*sind(120))*voltagephasor(2,:)+(cosd(120)+j*sind(120))^2*voltagephasor(3,:))/3;
            negseq=(voltagephasor(1,:)+(cosd(120)+j*sind(120))^2*voltagephasor(2,:)+(cosd(120)+j*sind(120))*voltagephasor(3,:))/3;
            vuf(n,:)=abs(negseq)./abs(posseq)*100;
        end
    end
    subplot(525)
    plot(t,vuf)
    hold on
    plot(t,2*ones(1,length(t)))
    plot(t,3*ones(1,length(t)))
    legend('VUF', '2% unbalance', '3% unbalance')
    title('Voltage Unbalance')
    xlabel('time')
    ylabel('vuf %')

    %historgram of overvoltages
    subplot(526)
    overvolt=histogram(voltagemag(voltagemag>1.05));
    title('Histogram of Overvoltages')
    xlabel('voltage (pu)')
    ylabel('incidence')

    %historgram of undervoltages
    subplot(527)
    undervolt=histogram(voltagemag(voltagemag<0.95));
    title('Histogram of Undervoltages')
    xlabel('voltage (pu)')
    ylabel('incidence')

    %transformer powers and ratings - normalized
    subplot(528)
    plot(t,transpower)
    hold on
    plot(t,ones(1, length(t)), 'k--')
    title('Transformer Power Flow')
    xlabel('time')
    ylabel('power (normalized to transformer rating')

    %histogram of power over rating -- magnitude over rating
    subplot(529)
    histogram(transpower(transpower>1));
    title('Hist of Trans Power-Over-Rating (Magnitude)')
    xlabel('normalized power')
    ylabel('incidence')

    %histogram of power over rating -- time over rating
    subplot(5,2,10)
    overrate=sum(((transpower-1)>0))>0; %entry is 1 if there is over-power in any of three phases
    %count consecutive ones + make vector of number of consecutive timesteps over rating -- this is
    %pretty brute force, I'm sure some clever matlab-ing can get you a more
    %elegent solution. Also needs to be scaled by timestep to get time in (s).
    overratetime=zeros(length(t));
    count=0;
    for n=1:length(t)
        if overrate(n)==1
            count=count+1;
        elseif n>1 && overrate(n-1)==1
            overratetime(n)=count;
            count=0;
        end
    end
    %get last count
    if count>0
        overratetime(n)=count;
    end
    overratetime=overratetime(overratetime>0); %remove zeros
    histogram(overratetime)
    title('Hist of Trans Power-Over-Rating (Time)')
    xlabel('length of time (measured in timesteps)')
    ylabel('incidence')


    if outputNamesIncludeBatch
        savefig(1, strcat(fileLocation,"\","PlottingFn_Figure1_",filePieces(end-2),"-", filePieces(end-1),".fig"));
        savefig(2, strcat(fileLocation,"\","PlottingFn_Figure2_",filePieces(end-2),"-", filePieces(end-1),".fig"));
        savefig(3, strcat(fileLocation,"\","PlottingFn_Figure3_",filePieces(end-2),"-", filePieces(end-1),".fig"));
    else
        savefig(1, strcat(fileLocation,"\","PlottingFn_Figure1.fig"));
        savefig(2, strcat(fileLocation,"\","PlottingFn_Figure2.fig"));
        savefig(3, strcat(fileLocation,"\","PlottingFn_Figure3.fig"));
    end    
    close all;


    %compute + output RMSE in kW
    rmsetracking=rms(reference-aggpower);
    nrmsetracking = rmsetracking/mean(reference);

    %compute PJM score
    [passOrFail, pjmScore, accScore, delayScore, precScore] = pjmScoringPassFailv5(reference, aggpower, mean(aggpower), 2);
    
    % keep performance metrics in a vector to return
    numOverVoltages = length(voltagemag(voltagemag>1.05));
    numUnderVoltages = length(voltagemag(voltagemag<0.95));
    % keep number of temperature excursions
    numTempViolations = sum(temps>1.02, 'all') + sum(temps<-0.02, 'all');
    % total number of switches
    num_switches = sum(abs(diff(simulationData.simulatedTclData.onOff, 1, 2)), 'all');
    % metrics for transformer power flows
    overflow_trans_max = max(transpower, [], 'all'); % max power flow
    overflow_total_duration = timestep*sum(transpower>1.01, 'all'); % duration in sec of transformer overflows
    max_overratetime = timestep*max(overratetime); % max number of consecutive seconds that a transformer rating was exceeded

    metrics = [rmsetracking,nrmsetracking,pjmScore, accScore, delayScore, precScore,numOverVoltages,numUnderVoltages,numTempViolations,overflow_trans_max,overflow_total_duration,max_overratetime,num_switches];
    
    disp(['RMSE: ' num2str(rmsetracking)] )
    disp(['NRMSE: ' num2str(nrmsetracking)] )
    fprintf(['PJM scores: ' num2str(pjmScore) ...
        '\n  PJM accuracy score: ' num2str(accScore) ...
        '\n  PJM delay score: ' num2str(delayScore) ...
        '\n  PJM precision score: ' num2str(precScore) '\n'] )
    
    save(strcat(fileLocation,"\",'plottingoutputs_',filePieces(end-2),"-", filePieces(end-1),'.mat'),'rmsetracking','totaltapping','totalcapswitching', 'pjmScore', 'accScore', 'delayScore', 'precScore');
end