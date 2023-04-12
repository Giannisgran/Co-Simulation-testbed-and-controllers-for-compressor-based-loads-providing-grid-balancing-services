% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script either runs simulations without gridlabd (if 
% simulationOptions.useGLD == 0), or it sets up simulations to be run from
% a command line interface that includes the gridlabd feeder models as well
% as the ARPA-E house models and such (if simulationOptions.useGLD == 1). 
% 
% Some preprocessing is needed before setting up batch simulations that run
% with gridlabd from this script:
% 1) run pre1_... with the feeder options that you are interested
%       exploring. pre1_... sets up the feeder models and determines the
%       number of houses that are placed onto the feeder. It creates files
%       in directories like "R5_1247_1" that contains the feeder
%       information. See comments in pre1_... for details. 
% 2) run pre2_... with the tcl options that you are interested in
%       exploring. This script takes the previously generated feeder files
%       and allows you to populate TCLs onto the feeder and initialize
%       them. It adds a .mat file that contains the feeder and TCL
%       information for each of the cases. See comments in pre2_... for
%       details.
% 3) run this script (main_...) to set up batch simulations that need to be
%       executed from the command line. This script takes the previously
%       generated data files for feeders and tcls and add in some run-time
%       selection of options--controllers, controller time-step, regulation
%       signal, communication scenario, simulation length, etc. To set this
%       script up to run, first determine whether you want to run batch 
%       simulations with GLD or single simulations without and set 
%       simulationOptions.useGLD. Set the simulation length in hours by
%       setting simulationOptions.simLength. Set the control timestep using 
%       simulationOptions.controlTimeStep (units are seconds) and the
%       specific controller using simulationOptions.controllerType. 
%
%       Next, step the options that we want to sweep over, which is a bit
%       further down in the script. Start by setting whether we want to do
%       the two-factor case design or the exhaustive search by setting 
%       simulationOptionSet.useTwoFactorCaseDesign. Set the regulation
%       signal type, communication network settings, regulation signal
%       amplitude, which will generate different regulation signals and
%       have different effects during the simulation. Below this, set the
%       other parameters to values that you generated using pre1_... and
%       pre2_... that will be used to load data from those initializations.
%       
%       Run this script once all options are selected, and it will set up a
%       directory with name batchSimulation_yyyymmdd_HHMM that contains a
%       batch file, and a number of sub-directories for each of the
%       simulations that correspond to the different parameter combinations
%       generated. The batch file calls gridlabd to run each of the
%       simulations sequentially, back-to-back. To run the batch file, in
%       the command line set the current directory to the directory
%       containing the batch file. The command is cd <directory path> in
%       windows. Then copy and paste the name of the batch file over into
%       the command line and execute; it will run through the commands
%       sequentially in the command line to execute the simulations. After
%       the simulations, a file simulationData_<yyyymmdd>_<HHMM> where the
%       date and time correspond to the time the simulation was completed
%       is saved into the directory within the batch file. A plotting
%       function will also be generated that plots the necessary values and
%       saves them in the same place as the simlationData file for analysis
%       of the results. 
%   To run two-factor case study setup, the seven parameters that we can
%   change above each need to have two options. It's also possible to
%   generate single cases or exhaustive conbinations of all parameters by
%   setting the two-factor flag to 0. 

clc; clearvars;
addpath(genpath(pwd)) % add folders and subfolders of the directory to path
fprintf('Copyright (C) 2023 The Regents of the University of Michigan\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET OPTIONS ABOUT WHAT FEEDER, DATA, AND CONTROL TO USE

%%%%%%%%%
% These aren't parameters that we sweep across

% decide whether to simulate with GLD or to simulate independent of GLD
% 0 = simulate w/o GLD
% 1 = simulate w GLD 
%       NOTE: option 1 only sets up the directories and the cmd file for
%       running the simulations from the command prompt sequentially,
%       without user intervention. When using this option, run this script,
%       wait for it to signal that you should run the batch file that was
%       created, then run that file from the command line. 
simulationOptions.useGLD = 0;

% option that you set to zero or one to control whether TCLs (and
% controllers) take the voltage at the TCLs into account to determine
% whether they are available or not. 
simulationOptions.useLocalVoltage = 0;

% option that you set to zero or one to decide whether to use original temp
% deadbands or PEM temp deadband, a % of the original; 0 for default
% deadband, 1 for PEM temp deadband
simulationOptions.usePEMTempDeadband = 0;

% set the length of the simulation in hours 
% this is not a parameter we sweep across
simulationOptions.simLength = 1; % [h]

% column vector of length NumberOfAggregators that defines how frequently
% an updated control signal will be generated for each aggregator
simulationOptions.controlTimeStep = [2]; % [sec]

% vector of cells of length equal to the number of aggregators that defines
% the controller that each aggregator is using. 
simulationOptions.controllerType = {'Markov controller'}; 
% 'PID'; 'Markov controller'; 'Markov controller with lockouts'; 
% 'Markov controller with delays'; 
% 'Markov controller 2 zone'; 'Markov controller 2 zone with lockouts';
% 'Markov controller mixed zone'; 
% 'PEM_E-T controller';

%%%%%% BELOW ARE OPTIONS THAT YOU DON'T NEED TO WORRY ABOUT CHANGING %%%%%


% takes values from 0-N, where 0 is no control, and where N is the number
% of independent "controllers" influencing loads on the feeder
simulationOptions.NumberOfAggregators = 1; 

% This defines the method used to assign houses/TCLs to the aggregators.
% 'random' just means that each tcl is randomly assigned. 
simulationOptions.aggregatorAssignment = 'random';

% PDF of probability of an AC being assigned to an aggregator. First
% element of the pdf corresponds to the first aggregator, second element to
% the probability of being assigned to the second aggregator, and so on.
simulationOptions.aggregatorAssignmentProb = [1];

% enable communication network
% 0: no comm network , 1: consider comm network
simulationOptions.considerCommNetwork = 1;

% Comm neetwork parameters used if simulating the communication network
simulationOptions.nc = 0; %number of collection nodes

rng(3453089235)

%%%%%% END THINGS TO LEAVE %%%%%

% end non-sweep parameters
%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% SWEEP PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% switch that determines how we generate the simulation case settings. This
% can be done either in an exhaustive search, or using the two-factor case
% design. 
% 0 = exhaustive search
% 1 = two factor design
simulationOptionSet.useTwoFactorCaseDesign = 0;

%%%%%%%% SET RUN-TIME OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set the communication network case, where case 1 (no comm network issues)
% results in  use setting considerCommNetwork to 0 to increase speed.
% 1 = no comm network issues
% 2 = moderate %%%% not being used for deliverables.
% 3 = severe 
simulationOptionSet.commScenario = [1];
% e.g., of definition = [1, 3]

% Set the regulation signal for each aggregator. Rows correspond to each 
% aggregator and columns correspond to different cases. 
% 1; triangular_option->enables the generation of triangular signals
% 2; sinusoid_option->enables the generation of sinusoid signals
% 3; pjm_option->enables the generation of PJM regulation signals based on scaling the signals in PJM_july_2019.xls
% 4; step response option -> enables step response signal
% 5; none -> no signal
simulationOptionSet.regulationSignalType = [3];
% e.g., of single aggregator definition = [3, 4]


% Defines the number of cycles in the sine and triangle waves. Should
% probably be changed to reguation signal period

simulationOptionSet.regulationSignalNumCycles = [1];
%%%%% DO NOT CHANGE %%%%% ...doesn't impact the regulation signals we're
%%%%% using for the deliverable. 

% fraction of the ss demand of the TCL population. Defined as the percent
% of the rated TCL capacity, NOT the portion of the steady-state demand at
% a given temperature. 
% .1 and .3 are the desired values. 
simulationOptionSet.regulationSignalAmplitude = [.1];
% e.g., of single aggregator definition = [.1, .3]

%%%%%%%% END RUN-TIME OPTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% OPTIONS FOR FEEDER DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% same options that were used in pre1_...; make sure that the options used
% here are a subset of the options used to generate the files. 
simulationOptionSet.percentTclOptionSet = 0.5; % no units
simulationOptionSet.feederCapacityScalingOptionSet = 1; % no units
simulationOptionSet.backgroundDemandScalingFactorOptionSet = 0.75; %GL note...changed this in redefining cases 
simulationOptionSet.voltageRegulatorSetting = [1];
simulationOptionSet.setOfTaxFiles = {%'R5-12.47-4.glm'; ...
    %'R5-12.47-1.glm'; ...
    'R5-25.00-1.glm'; ...
    %'R2-12.47-2.glm';...
    %'R2-12.47-3.glm';...
    %'R2-25.00-1.glm';...
    %'R5-12.47-5.glm';...
    %'R5-35.00-1.glm'...
    };

% END FEEDER DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BEGIN TCL OPTION SELECTION
% Same options used in pre2_...; make sure the options used here are a
% subset of the options used to generate the data files when running the
% pre2_... script directly. 
simulationOptionSet.weatherSet = [1]; % [-]
simulationOptionSet.buildingParameterSet = [1]; % [-]
simulationOptionSet.tclHeterogeneitySet = [1]; %[-]
simulationOptionSet.lockoutTimeOnSet = 180; % [sec]
simulationOptionSet.lockoutTimeOffSet = 3; % [min]

% END TCL OPTION SELECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%% END SWEEP PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BEGIN MODEL OPTION SELECTION

% can define options here that control what model data gets loaded for use
% within the controllers, if needed

% number of temperature intervals in the state bin model
numTempIntervals = 20*ones(simulationOptions.NumberOfAggregators,1); 
% number of temperature intervals for the 2-zone bin model
numTempIntervals2zone = 6*ones(simulationOptions.NumberOfAggregators,1);
% 0 for kalman estimator, 1 for full state info - used in the binModel
fullStateInfo = 1;
% TCL temperature delays option that control the time delay between the
% switching of the A/C and the temperature starting to move towards the
% opposite direction - make sure its the same as the option from pre2
% 1: no temperature delay
% 2: constant low temperature delay
% 3: constant high temperature delay
% 4: temperature delay distribution
temperatureDelayOption = 1;
% END MODEL OPTION SELECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% END SET OPTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% You shouldn't have to look below this point if you just want to set up
% and run simulations. 




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND PARAMETER SWEEPS

% function that determines option sequences given parameter sets above
parameterSweepSimulation = generateSimulationParameterSweepArrays(simulationOptionSet);

% calculate the number of cases from the option sequences determined above
numCases = numel(parameterSweepSimulation.weatherSet);

% END PARAMETER SWEEPS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% loop through the different simulations within the set
for idx = 1:numCases

    % keep the total time the simulation will run
    stime = tic;
    
    % if we want to run with GLD, execute this to set up directories and
    % batch file.
    if simulationOptions.useGLD
        
        % Create a directory to hold all files needed to run the batch
        if idx == 1
            thisDate = datestr(datetime('now'),'yyyymmdd_HHMM');
            batchDirectory = ['batchSimulation_' thisDate];
            mkdir(batchDirectory);
            
            % create batch file that goes in this directory to run the set
            % of simulations, which we will add lines to as we process each
            % simulation below
            batchFile = ['./' batchDirectory '/' batchDirectory '.bat'];
            fidBatch = fopen(batchFile, 'wt' );
            fclose(fidBatch);
        end
        
        % create a directory within the one above that stores files for
        % each simulation
        simulationDirectory = ['simulation' num2str(idx) ];
        mkdir(['./' batchDirectory '/' simulationDirectory]);
        
        % copy core gld files over to the simulation directory
        standardFiles = {'onInit.m'; 'onSync.m'; 'onTerm.m'};
        for fileIdx = 1:numel(standardFiles)
            source = fullfile('./functions',standardFiles{fileIdx});
            destination = fullfile(['./' batchDirectory '/' simulationDirectory]);
            
            destination = strrep(destination ,'\','/');
            source = strrep(source ,'\','/');            
            
            copyfile(source,destination)
        end
        
        % copy over GLM file and link file
        thisGLM = parameterSweepSimulation.setOfTaxFiles{idx};
        thisGLM2 = strrep(strrep(strrep(thisGLM,'.glm',''),'-','_'),'.','');
        percentTcl = parameterSweepSimulation.percentTclOptionSet(idx);
        feederCap = parameterSweepSimulation.feederCapacityScalingOptionSet(idx);
        regSetting = parameterSweepSimulation.voltageRegSet(idx);
        brDemand = parameterSweepSimulation.backgroundDemandScalingFactorOptionSet(idx);
        thisGLM3 = [thisGLM2 '_' num2str(regSetting) ...
            '_' num2str(floor(percentTcl*100)) ...
            '_' num2str(floor(feederCap*100)) '_' ...
            num2str(floor(brDemand*100)) '.glm'];
        source = fullfile(['./' thisGLM2],thisGLM3);
        source = strrep(source ,'\','/');            

        % update timestep and sim length in glm file
        fid = fopen(source );
        cac = textscan(fid,'%s','Delimiter','\n','CollectOutput',true);
        cac = cac{1};
        fclose(fid);
        fid = fopen([destination '/' thisGLM3],'w');
        
        % loop through the lines in the base file and replace cd(...)
        for jj = 1 : length(cac)
            % if its the line that sets the start-time, get start date and
            % time info
            if ~isempty(strfind(cac{jj},'starttime'))
                loc1 = strfind(cac{jj},'starttime');
                glmStartTime = strrep(strrep(strrep(cac{jj},'starttime',''),';',''),'''','');
            
                fprintf(fid, '%s\n', cac{jj});

            % if it's the line that sets the stop-time, set the stop time
            % according to start time and the simulation length
            elseif ~isempty(strfind(cac{jj},'stoptime'))
                
                stopDateTime = datetime( glmStartTime, 'InputFormat', 'yyyy-MM-dd HH:mm:ss' );
                glmStopTime = datestr(stopDateTime  + hours(simulationOptions.simLength),'yyyy-mm-dd HH:MM:SS');
                           
                loc1 = strfind(cac{jj},'''');
                loc2  = strfind(cac{jj},';');
                writeString = [cac{jj}(1:loc1) glmStopTime cac{jj}(loc2-1:end)];
                
                fprintf(fid, '%s\n', writeString);

                % else copy the line over
            else
                
                fprintf(fid, '%s\n', cac{jj});
                
            end
        end
        fclose( fid );
    
        
        
        thisLink = ['matlabLink_' strrep(thisGLM3,'.glm','.link')];
        source = fullfile(['./' thisGLM2],thisLink);
        source = strrep(source ,'\','/');            
            
        % update cd in link file
        % open a new file that will use the link information
        % open the base link file and work from that
        fid = fopen(source );
        cac = textscan(fid,'%s','Delimiter','\n','CollectOutput',true);
        cac = cac{1};
        fclose(fid);
        fid = fopen([destination '/' thisLink],'w');
        
        % loop through the lines in the base file and replace cd(...)
        for jj = 1 : length(cac)
            if ~isempty(strfind(cac{jj},'cd(')) && isempty(strfind(cac{jj},'#'))
                loc1 = strfind(cac{jj},'cd(');
                loc2  = strfind(cac{jj},')');
               
                % get full path to add to link file
                thisPath = what;
                linkPath = [thisPath.path destination(2:end)]; 
                linkPath = strrep(linkPath,'\','/');            
                writeString = [cac{jj}(1:loc1+2) '''' linkPath '''' cac{jj}(loc2:end)];
                
                fprintf(fid, '%s\n', writeString);
                
                % else copy the line over
            else
                
                fprintf(fid, '%s\n', cac{jj});
                
            end
        end
        fclose( fid );
         
        % save files to thisRun directory
        save([destination '/thisRunData.mat'],'parameterSweepSimulation','idx','simulationOptions','numTempIntervals','fullStateInfo', 'temperatureDelayOption', 'numTempIntervals2zone')

        % add lines to the batch file to run this simulation
        fidBatch = fopen(batchFile, 'at+' );
        thisPath = what;
        pathToGLM = [thisPath.path destination(2:end)];
       	pathToGLM = strrep(pathToGLM,'\','/');                 
        myCmd = ['cd ' pathToGLM ' & gridlabd ' thisGLM3 ' > simOutput.txt & '];        
        fprintf(fidBatch,'%s\n',myCmd);
        fclose(fidBatch);

    % if we want to run without GLD, execute this
    else
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % do some setup
        
        % call the initialization script within the feeder folder above
        onInit;
        
        % END do some setup
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % RUN THE SIMULATION
        
        % this is the output of the GLD simulation time; we'll use it directly in
        % this sim
        gld.global.clock = 0;
        
        % this is an indicator for the loop below on when to exit the loop after
        % the last time-step has been executed
        endOfSimulation = 0 ;
        
        % while not at end of simulation
        while endOfSimulation == 0
            
            % set the node voltages by evaluating the command constrcuted
            % above
            eval(setNodeVoltage);
            
            % do the script that gets executed at each iteration in Gridlab-D
            onSync;
            
         
            % get the apparent power draw of each tcl at this time-step
            powerDraws = simulatedTclData.tclParameters.P_power_draw;
            reactivePowerDraws = simulatedTclData.tclParameters.Q_power_draw;
            actualAcPowerDrawsReal = onOffValueVector.*powerDraws;
            actualAcPowerDrawsReac = onOffValueVector.*reactivePowerDraws;
            actualAcPowerDrawsApp = actualAcPowerDrawsReal + 1j*actualAcPowerDrawsReac;        
            
            % use matrix multiplication to calculate the power at each
            % node, using incidence matrices calculated in onInit
            acPowerAllNodes = tclIncidenceMatrix*actualAcPowerDrawsApp;
            backgroundPowerAllNodes = houseIncidenceMatrix * houseParameters.backgroundLoad;  
            allNodesPower = acPowerAllNodes + backgroundPowerAllNodes;
            eval(setNodePower)
            
            % ans gets set in onSync and determines whether to move to the next
            % timestep or not.
            gld.global.clock = ans;
            
            % check if the next time-step is going to be after the end of the
            % simulation duration that was set in the parameters
            % subtract a time-step to make sure that indices work out
            if gld.global.clock > generalParameters.simLength * 3600 - generalParameters.timeStepInSec
                endOfSimulation = 1;
            end
        end
        
        % END RUN THE SIMULATION
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % run the termination script within the feeder folder that we are
        % simulating
        onTerm;
        
    end
    
    %sim end time
    sim_end_time = datetime;

    % provide some output
    disp(['Simulation Completed in ' num2str(toc(stime),'%.2f') ' seconds'])
    
%     generatePlots_anyController(simulationData)
    
end

if simulationOptions.useGLD==1
    f = msgbox({'Setup GLD files. Run the batch file from the command line.';
        'To do this, set the current directory of the command line environment';
        'to the directory containing the batch file, then copy and paste the';
        'batch file name over to the command line and execute.'});
end