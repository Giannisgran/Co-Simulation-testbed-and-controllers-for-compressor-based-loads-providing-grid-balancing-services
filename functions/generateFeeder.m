% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [feederParameters, houseParameters, generalParameters] = ...
            generateFeeder...
                  (generalParameters, feederParameters)
% COPY PNNL FEEDER OVER TO NEW FILE

% This section of code takes the original PNNL feeder file and preprs a new
% file for generation from that and the additional details we are adding to
% the feeder

% pull data from the feeder parameters that will be used below
regional_data = feederParameters.regional_data;
taxonomy_data = feederParameters.taxonomy_data;

% get the path to the original feeder file that we will be manipulating
file_to_extract = feederParameters.feederFile;
taxonomy_directory = feederParameters.tax_dir;
extraction_file = [taxonomy_directory, file_to_extract];

% manipulate the filename to extract by removing .glm, .'s, and replacing -
% with '_'; add _arpaE suffix and .glm extension
file2 = strrep(file_to_extract,'.glm','');
file3 = strrep(file2,'.','');
file4 = strrep(file3,'-','_');
[~,idx] = intersect(file4,'R','stable');
file5 = file4; %strrep(file4,['R' origRegion],'R6');
tech_file = [file5 '_' ...
    num2str(feederParameters.voltageRegulatorSetting) '_' ...
    num2str(floor(feederParameters.percentHousesWithTCls*100)) '_' ...
    num2str(floor(feederParameters.feederCapacityScalingFactor*100)) '_' ...
    num2str(floor(feederParameters.backgroundDemandScalingFactor*100))];
filename = [tech_file,'.glm'];

% read the file that we have designated so that we can process the
% information contained in it below
read_file = fopen(extraction_file,'r');

% function parses the original glm file and creates an array of
% configurations and an array of glm things to be copied over to a new glm
% file that we are constructing
[glm_final, config_final, mm, nn, swing_node] =  generateConfigArrays(read_file);

% set the write file name, and create the write directory if it doesn't
% exist already
fid_name = ['./', file5, '/', filename];
if ~exist(strrep(fid_name,['/', filename],''), 'dir')
       mkdir(strrep(fid_name,['/', filename],''))
end
write_file = fopen(fid_name,'w');

% start writing things to the new glm file. 
fprintf(write_file,'//Input feeder information for Taxonomy Feeders with different cases.\n');
fprintf(write_file,'//Started on 10/21/09. This version created %s.\n',datestr(now));
fprintf(write_file,'// This version was modified by Greg Ledva.\n\n');

% set the timing of the simulation using the timezone that was specified
% and the start and end date and time
fprintf(write_file,'clock {\n');
fprintf(write_file,'     timezone %s;\n',regional_data.timezone);
fprintf(write_file,'     starttime ''%s'';\n',generalParameters.defaultStartDate);
fprintf(write_file,'     stoptime ''%s'';\n',generalParameters.defaultEndDate);
fprintf(write_file,'}\n\n');

% set the minimum timestep of the simulation using the timestep that was
% set at the top of main and point to the relevant matlab link file that
% will be constructed.
fprintf(write_file,'#set minimum_timestep=%s;\n',num2str(generalParameters.timeStepInSec));
fprintf(write_file,'#set profiler=1;\n');
fprintf(write_file,'#set relax_naming_rules=1;\n\n');
fprintf(write_file,'link "matlabLink_%s.link";\n\n',strrep(filename,'.glm',''));

fprintf(write_file,'module tape;\n');

% set things related to the powerflow module below, removed a feeder
% parameter allowing other solvers to be used, and so we only use the
% default GLD solver. 
fprintf(write_file,'module powerflow {\n');
fprintf(write_file,'     solver_method NR;\n');
fprintf(write_file,'     NR_iteration_limit 1;\n');
fprintf(write_file,'     default_maximum_voltage_error 1e-6;\n');
fprintf(write_file,'};\n\n');

% area where we are starting to import the different confurations used in
% the feeder model
fprintf(write_file,'//Configurations\n\n');

p = 0; % variable used to do some looping within main loop

% copy over the configurations that were collected in the 
% generateConfigArrays function into the file that were are creating. 
for i=1:(mm-1)
    
    if p >= i
       continue; 
    end
    
    if (strcmp(char(config_final{1}{i}),'}') == 1)
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n\n',char(config_final{1}{i}),char(config_final{2}{i}),char(config_final{3}{i}),char(config_final{4}{i}),char(config_final{5}{i}),char(config_final{6}{i}),char(config_final{7}{i}),char(config_final{8}{i}));
    elseif (strcmp(char(config_final{2}{i}),'regulator_configuration') ~= 0)
        % write the first line
        
        % iterate through rest of the object block
        p = i;
        while (strcmp(char(config_final{1}{p}),'}') == 0)
            if (strcmp(char(config_final{2}{p}),'band_center') ~= 0)
                param = [num2str(taxonomy_data.voltage_regulation{1}) ';'];
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{p}),char(config_final{2}{p}),param,char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));                        
            elseif (strcmp(char(config_final{2}{p}),'band_width') ~= 0)
                param = [num2str(taxonomy_data.voltage_regulation{2}) ';'];
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{p}),char(config_final{2}{p}),param,char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));                        
            elseif (strcmp(char(config_final{2}{p}),'time_delay') ~= 0)
                param = [num2str(taxonomy_data.voltage_regulation{3}) ';'];
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{p}),char(config_final{2}{p}),param,char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));                        
            elseif (strcmp(char(config_final{2}{p}),'raise_taps') ~= 0)
                param = [num2str(taxonomy_data.voltage_regulation{4}) ';'];
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{p}),char(config_final{2}{p}),param,char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));                        
            elseif(strcmp(char(config_final{2}{p}),'lower_taps') ~= 0)
                param = [num2str(taxonomy_data.voltage_regulation{4}) ';'];
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{p}),char(config_final{2}{p}),param,char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));                        
            elseif(strcmp(char(config_final{2}{p}),'regulation') ~= 0)
                param = [num2str(taxonomy_data.voltage_regulation{5}) ';'];
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{p}),char(config_final{2}{p}),param,char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));                        
            else
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{p}),char(config_final{2}{p}),char(config_final{3}{p}),char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));                        
            end
            
            % increment the loop variable
            p = p + 1;
        end
        % print the line that exited the loop
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n\n',char(config_final{1}{p}),char(config_final{2}{p}),char(config_final{3}{p}),char(config_final{4}{p}),char(config_final{5}{p}),char(config_final{6}{p}),char(config_final{7}{p}),char(config_final{8}{p}));
    else
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(config_final{1}{i}),char(config_final{2}{i}),char(config_final{3}{i}),char(config_final{4}{i}),char(config_final{5}{i}),char(config_final{6}{i}),char(config_final{7}{i}),char(config_final{8}{i}));
    end
end

% create a transformer configuration that corresponds to the feeder head;
% voltage and feeder rating are used below to define this configuration
% GSL NOTE: modify feeder data in TaxFeederDataGSL to change below fields
% that are filled in programmatically
fprintf(write_file,'object transformer_configuration {\n');
fprintf(write_file,'     name trans_config_to_feeder;\n');
fprintf(write_file,'     connect_type WYE_WYE;\n');
fprintf(write_file,'     install_type PADMOUNT;\n');
fprintf(write_file,'     primary_voltage 132790.56;\n');
fprintf(write_file,'     secondary_voltage %.3f;\n',taxonomy_data.nom_volt);
fprintf(write_file,'     power_rating %.1f MVA;\n',taxonomy_data.feeder_rating);
fprintf(write_file,'     impedance 0.00033+0.0022j;\n');
fprintf(write_file,'}\n\n');

% Create the feeder
% Starts creating the actal feeder (nodes, line, and configurations) using
% the configurations copied over above

% create the swing bus at the feeder head
fprintf(write_file,'//Start actual feeder\n\n');
fprintf(write_file,'object meter {\n');
fprintf(write_file,'     name network_node;\n');
fprintf(write_file,'     bustype SWING;\n');
fprintf(write_file,'     nominal_voltage 132790.56;\n');
fprintf(write_file,'     phases ABCN;\n');
fprintf(write_file,'}\n\n');

% use the transformer configuration above to set up the transformer from 
% the substation to the actual feeder
fprintf(write_file,'object transformer {\n');
fprintf(write_file,'     name substation_transformer;\n');
fprintf(write_file,'     from network_node;\n');
fprintf(write_file,'     to %s\n',swing_node);
fprintf(write_file,'     phases ABCN;\n');
fprintf(write_file,'     configuration trans_config_to_feeder;\n');
fprintf(write_file,'}\n\n');

% set some variables that will be used below when looking through the glm
% file. 
m = 0; % secondary loop index that keeps track of when the logic has moved further into the file
cap_n = 0; % number of capacitors
reg_n = 0; % number of regulators
house_no_S = 0; % number of split phase nodes
totalSplitPhaseLoad = 0; % total split phase load
loads_number = 0; % number of large loads
nodes_number = 0; % number of nodes
bigLoads = 0; % power of the large loads
allNodesList = {}; % keeps track of node names and phases
commLoadNodeList = {}; % keeps track of the commercial loads and nodes


% set variables that will keep track of the number of triplex lines, the
% number of underground lines, and the number of overhead lines. 
no_triplines = 0;
no_ugls = 0;
no_ohls = 0;

% for each line in glm_final
for j=1:(nn-1)
    
    % if m is greater than j, then skip to the next iteration because we
    % handled the line somewhere in the logic below
    if (m >= j)
        continue; % this is used to skip over certain lines
    end
    
    % if the line in the glm file corresponds to 'name', a 'to' node, or a
    % 'from' node, copy the line over to the write file. 
    if (strcmp(char(glm_final{2}{j}),'name') ~= 0) % copy name line over
        named_object = char(glm_final{3}{j});
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
    elseif (strcmp(char(glm_final{2}{j}),'to') ~= 0) % copy to line over
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
    elseif (strcmp(char(glm_final{2}{j}),'from') ~= 0) % copy from line over
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        %         elseif (strcmp(char(glm_final{2}{j}),'triplex_node') ~= 0)
        %             fprintf(write_file,'object triplex_meter {\n');
    
    
    % if the row in the glm file corresponds to a capacitor, loop through
    % to get it's name, iterate number of capacitors (cap_n), and add the
    % capacitor's name to the capacitor list
    elseif (strcmp(char(glm_final{2}{j}),'capacitor') ~= 0) % get and print capacitor name
        c = j;
        while (strcmp(char(glm_final{1}{c}),'}') == 0) % get the capacitor name
            c = c+1;
            if(strcmp(char(glm_final{2}{c}),'name') ~= 0)
                cap_n = cap_n+1;
                cap_name = strrep(glm_final{3}{c},';','');
                capacitor_list{cap_n} = char(cap_name);
                break;
            end
        end
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
    
    % do the same for regulators as you did for capacitors
    elseif (strcmp(char(glm_final{2}{j}),'regulator') ~= 0)
        r = j;
        
        % track regulator number, get regulator name from glm file
        while (strcmp(char(glm_final{1}{r}),'}') == 0) % while not the end of the code block
            r = r+1;
            if(strcmp(char(glm_final{2}{r}),'name') ~= 0)
                reg_n = reg_n+1;
                reg_name = strrep(glm_final{3}{r},';','');
                regulator_list{reg_n} = char(reg_name);
            elseif (strcmp(char(glm_final{2}{r}),'to') ~= 0)
                regulator_senseNode{reg_n} = strrep(char(glm_final{3}{r}),';','');
            end
        end
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
    
    % if you get to a line that indicates the parents of an object, store
    % the parent and copy the line over
    elseif (strcmp(char(glm_final{2}{j}),'parent') ~= 0) % get parent and write to glm
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        parent_object = char(glm_final{3}{j});
    
    % This seems like it just copies over transformer information, but I'm
    % not deleting the logic because I don't quite know what it's supposed
    % to be doing 
    elseif (strcmp(char(glm_final{2}{j}),'transformer') ~= 0)
        m = j;
        if (0 && use_flags.use_commercial == 1)
            while (strcmp(char(glm_final{1}{m}),'}') == 0)
                m = m+1;
                if(strcmp(char(glm_final{2}{m}),'WYE_WYE') ~= 0)
                    
                end
            end
        else
            fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
            fprintf(write_file,'      groupid Distribution_Trans;\n');
        end
    
    % if it's a triplex line, copy the line over and update the number of
    % triplex lines
    elseif (strcmp(char(glm_final{2}{j}),'triplex_line') ~= 0)
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        fprintf(write_file,'      groupid Triplex_Line;\n');
        no_triplines = no_triplines + 1;
    
    % if it's an overhead line, copy the line over and update the number of
    % overhead lines
    elseif (strcmp(char(glm_final{2}{j}),'overhead_line') ~= 0)
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        fprintf(write_file,'      groupid Distribution_Line;\n');
        no_ohls = no_ohls + 1;
    
    % if it's an underground line, copy the line over and update the number
    % of underground lines
    elseif (strcmp(char(glm_final{2}{j}),'underground_line') ~= 0)
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        fprintf(write_file,'      groupid Distribution_Line;\n');
        no_ugls = no_ugls + 1;
        
    % if it's a line indicating what phases something pertains to, copy
    % them over
    elseif (strcmp(char(glm_final{2}{j}),'phases') ~= 0) % copy phases over
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        phase = char(glm_final{3}{j});
    
    % if it's a non-residential load, create ZIP models for the load
    elseif (strcmp(char(glm_final{2}{j}),'load') ~= 0)
        % creates commercial ZIP loads
        [m, parent_name, parent_phase, totalLoad] = generateCommercialZipLoads(j, glm_final, taxonomy_data, write_file, feederParameters);

        
        bigLoads = bigLoads + totalLoad;
        loads_number = loads_number + 1;
        
        idx2 = 1;
        nodeName = [];
        nodePhases = [];
        nodeLoad = abs(totalLoad);
        while (strcmp(char(glm_final{1}{j+idx2}),'}') ~= 1)                        
           if(strcmp(char(glm_final{2}{j+idx2}),'name') ~= 0)
               nodeName = strrep(char(glm_final{3}{j+idx2}),';','');
           elseif (strcmp(char(glm_final{2}{j+idx2}),'phases') ~= 0)
               nodePhases = strrep(strrep(char(glm_final{3}{j+idx2}),';',''),'N','');
           end
               
           idx2 = idx2 + 1;
            
        end
        
        commLoadNodeList = [commLoadNodeList; {nodeName} {nodePhases} {nodeLoad}];
        
    % if we have a residential load, indicated by the options in the logic,
    % then we copy the line over to the write file, set the number of
    % houses attached to that node based on the load size, and store some
    % information that will be used below all of this logic that we will
    % use to create the actual houses
    elseif (strcmp(char(glm_final{2}{j}),'power_1') ~= 0 || strcmp(char(glm_final{2}{j}),'power_12') ~= 0)
        
        % copy the line over
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));

        % scaling factor used to increase or decrease the planning load
        % (and so the number of houses) on the feeder
        scalingFactor = feederParameters.feederCapacityScalingFactor;
        splitPhaseLoad = abs(str2num(glm_final{3}{j}));
        
        
        % get real, imag, and magnitude of load
        bb_real = real(str2num(glm_final{3}{j}))*scalingFactor;
        bb_imag = imag(str2num(glm_final{3}{j}))*scalingFactor;
        bb = sqrt(bb_real^2 + bb_imag^2);
        
        % get the number of houses this corresponds to based on the average
        % house size of the taxonomy data
        
        %no_of_houses = round(bb/feederParameters.averageHouseSizeInKw);
        no_of_houses = round(bb/taxonomy_data.avg_house);
        
        % get object name
        name = named_object;
        
        house_no_S = house_no_S + 1;

        % store the house info for use below when we build the actual
        % houses and TCLs
        phase_S_houses{house_no_S,1} = num2str(no_of_houses);
        phase_S_houses{house_no_S,2} = name;
        phase_S_houses{house_no_S,3} = phase;
        phase_S_houses{house_no_S,4} = parent_object;
        phase_S_houses{house_no_S,5} = splitPhaseLoad;
        
        totalSplitPhaseLoad = totalSplitPhaseLoad + splitPhaseLoad;
    
    % if it's the last line in an "object block" in the glm (i.e., the }
    % ends a section of the code), copy the line over
    elseif (strcmp(char(glm_final{2}{j}),'}') ~= 0)
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        
    % if it's the first line, which has '}', add blank line into feeder
    % glm file
    elseif (strcmp(char(glm_final{1}{j}),'}') ~= 0) && (j == 1) %Very special case to clear out line that isn't needed.
        fprintf(write_file,'        \n');
    
    % if its a fuse, set the replacement time to avoid throwing warniings
    elseif (strcmp(char(glm_final{2}{j}),'fuse') ~= 0)
        fuseString = 'mean_replacement_time 3600;';
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        fprintf(write_file,'%s %s \n',char('     '),char(fuseString));

    % if it's a capacitor bank (I think)...
    elseif (strcmp(char(glm_final{2}{j}),'pt_phase') ~= 0)
        % check the object is capacitor bank
        foundBrace = 0; % flag if found beginning of object
        jBack = j -1; % index to look backwards in file
        isCap = 0; % flag for a capacitor
        while foundBrace == 0
            % if it's a cap, flag it
            if strcmp(char(glm_final{2}{jBack}), 'capacitor')
                isCap = 1;
                break
            end
            % if you find the beginning of the object, exit while
            if strcmp(char(glm_final{3}{jBack}),'{')
                foundBrace = 1;
            end
            jBack = jBack - 1;
        end
        
        % now find the connection attribute
        foundBrace = 0; % reset this
        jForward = j + 1; % forward iterator
        phases = char(glm_final{3}{j}); % set to original value
        while foundBrace ==0 && isCap
            % set phases to phases_connected if you find it
            if strcmp(char(glm_final{2}{jForward}), 'phases_connected')
                phases = char(glm_final{3}{jForward});
                break;
            end
            if strcmp(char(glm_final{2}{jForward}), '}')
                foundBrace = 1;
            end
            jForward = jForward + 1;
        end
        
        % print the new parameter for pt_phases to file
        fprintf(write_file, '%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),phases,char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));

    % get the list of all nodes
    elseif (strcmp(char(glm_final{2}{j}),'node') ~= 0)
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
        
        idx2 = 1;
        nodeName = [];
        nodePhases = [];
        while (strcmp(char(glm_final{1}{j+idx2}),'}') ~= 1)            
           if(strcmp(char(glm_final{2}{j+idx2}),'name') ~= 0)
               nodeName = strrep(char(glm_final{3}{j+idx2}),';','');
           elseif (strcmp(char(glm_final{2}{j+idx2}),'phases') ~= 0)
               nodePhases = strrep(strrep(char(glm_final{3}{j+idx2}),';',''),'N','');
           end
               
           idx2 = idx2 + 1;
            
        end
        
        allNodesList = [allNodesList; {nodeName} {nodePhases}];
        nodes_number = nodes_number + 1;
        
    % copy any other miscellaneous lines over
    else 
        fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{j}),char(glm_final{2}{j}),char(glm_final{3}{j}),char(glm_final{4}{j}),char(glm_final{5}{j}),char(glm_final{6}{j}),char(glm_final{7}{j}),char(glm_final{8}{j}));
    end
end % end loop through glm_final

% store some values about the feeder
feederParameters.allNodesList = allNodesList;
feederParameters.numberOfNodes = nodes_number;
feederParameters.capacitorList = capacitor_list;
feederParameters.numberOfCapacitors = cap_n;
feederParameters.regulatorList = regulator_list;
feederParameters.numberOfRegulators = reg_n;
feederParameters.regulatorSenseNodes = regulator_senseNode;
feederParameters.commercialLoadList = commLoadNodeList;
feederParameters.numberOfCommLoads = loads_number;
feederParameters.splitPhaseNodeList = phase_S_houses;
feederParameters.numberOfSplitPhaseNodes = house_no_S;
feederParameters.totalSplitPhaseLoads = totalSplitPhaseLoad;
feederParameters.totalCommLoads = bigLoads;

% END COPY PNNL FEEDER OVER TO NEW FILE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BUILD HOUSES AND TCLS
  
% now that we have gone through the entire GLM file and collected
% information about the nodes that will have houses connected to them, we
% need to actually build the houses and TCLs within them.

% create a cell array that will hold each node, its complex power
% value, and the number of houses GLD says should be there
houseAndNodeList =  cell(size(phase_S_houses(:,1),1),3);

% This is the AC penetration for the region, which determines the
% probability of an AC being in a house
acPenetration = feederParameters.percentHousesWithTCls;

% keeps track of how many nodes we have found to add loads to
nodeCount = 0;
% keeps track of total houses on feeder
total_no_houses = 0;
% keeps track of the total number of ACs on the feeder
numAcs = 0;
% keep the indices of houses that will be 2zone, single zone
houses_2zone = []; houses_1zone = [];
 % keep house indices that will contain an A/C
housesWithAc = [];

% loop through the file again and get some information that I didn't get
% above. Could probably just merge with that part of the code if you wanted
% to simplify things.
for j=1:(nn-1)
    if (strcmp(char(glm_final{2}{j}),'name') ~= 0) % copy name line over
        % gets the name of the object
        named_object = char(glm_final{3}{j});
    elseif (strcmp(char(glm_final{2}{j}),'power_1') ~= 0 || strcmp(char(glm_final{2}{j}),'power_12') ~= 0)
        
        % scaling factor used to increase or decrease the planning load
        % (and so the number of houses) on the feeder
        scalingFactor = feederParameters.feederCapacityScalingFactor;
        
        % get real, imag, and magnitude of loads
        bb_real = real(str2num(glm_final{3}{j}))*scalingFactor;
        bb_imag = imag(str2num(glm_final{3}{j}))*scalingFactor;
        
        % set the node name in column 1
        houseAndNodeList{nodeCount+1,1} = named_object(1:end-1);
        % set the load information in column 2
        houseAndNodeList{nodeCount+1,2} = bb_real + 1j*bb_imag;
        % set the number of houses on the node in column 3
        houseAndNodeList{nodeCount+1,3} = str2num(char(phase_S_houses(nodeCount+1,1)));
        
        houseIndicesThisNode = total_no_houses + (1:houseAndNodeList{nodeCount+1,3});
        % set the house indices for each node in column 4
        
        houseAndNodeList{nodeCount+1,4} = houseIndicesThisNode;
        
        % set the air conditioner indices for each node in column 5       
        getsAC = binornd(1,acPenetration, size(houseIndicesThisNode));       
        houseAndNodeList{nodeCount+1,5} = getsAC.*houseIndicesThisNode;
        if sum(getsAC) > 0 % if at least one house in the current node contains an A/C
            current_houses_with_ac = houseIndicesThisNode(logical(getsAC));
            housesWithAc = [housesWithAc current_houses_with_ac];
        end
        numAcs = numAcs + sum(getsAC);
        
        % determine whether the ac will be single or 2 zone
        if generalParameters.perc_2zone > 0 && sum(getsAC) > 0
            is2zone = binornd(1, generalParameters.perc_2zone, [1, sum(getsAC)]); 
            if length(is2zone) > 1 || is2zone == 1  % won't get in if there is only 1 house that is not 2-zone
                houses_2zone = [houses_2zone current_houses_with_ac(logical(is2zone))];
                houses_1zone = [houses_1zone setdiff(current_houses_with_ac, current_houses_with_ac(logical(is2zone)))];
            else
                houses_1zone = [houses_1zone current_houses_with_ac];
            end
            numAcs = numAcs + sum(is2zone);
        end
        
        % set the node number for each node in column 6 (essentially just
        % the row number)
        houseAndNodeList{nodeCount+1,6} = nodeCount+1;
        
        % add houses on this node to the total number of houses on the
        % feeder
        total_no_houses = total_no_houses + houseAndNodeList{nodeCount+1,3};

        
        % get the total power draw of the houses that we have attached to the
        % feeder
        total_no_houses_power = houseAndNodeList(:,2);

        
        nodeCount = nodeCount + 1;
        
    end
end


%%%%%%%%%%%%%%%
% find the nominal voltages of nodes of interest

nominalVoltages.regs = [];
nominalVoltages.caps = [];
nominalVoltages.commLoads = [];
nominalVoltages.SpLoads = [];

numCaps = 0;
numRegs = 0;
numSpLoads = 0;
numCommLoads = 0;
numTrafos = 0;
m = 0;

if numel(feederParameters.regulatorSenseNodes) > 1
    warning('More than one regulator not handled in nominal voltages')
end

for j=1:(nn-1)
    % if m is greater than j, then skip to the next iteration because we
    % handled the line somewhere in the logic below
    if (m >= j)
        continue; % this is used to skip over certain lines
    end
    
    % do the same for regulators as you did for capacitors
    if strcmp(char(glm_final{2}{j}),'name') && (strcmp(char(glm_final{3}{j}),[feederParameters.regulatorSenseNodes{1} ';']) ~= 0)
        foundBrace = 0; % reset this
        jForward = j + 1; % forward iterator
        while foundBrace ==0
            % set phases to phases_connected if you find it
            if strcmp(char(glm_final{2}{jForward}), 'nominal_voltage')
                nomVoltage = str2double(strrep(char(glm_final{3}{jForward}),';',''));
                break;
            end
            if strcmp(char(glm_final{2}{jForward}), '}')
                foundBrace = 1;
            end
            jForward = jForward + 1;
        end
        numRegs = numRegs + 1;
        nominalVoltages.regs = [nominalVoltages.regs; nomVoltage];
        m = jForward;
    elseif (strcmp(char(glm_final{2}{j}),'capacitor') ~= 0)
        foundBrace = 0; % reset this
        jForward = j + 1; % forward iterator
        while foundBrace ==0
            % set phases to phases_connected if you find it
            if strcmp(char(glm_final{2}{jForward}), 'nominal_voltage')
                nomVoltage = str2double(strrep(char(glm_final{3}{jForward}),';',''));
                break;
            end
            if strcmp(char(glm_final{2}{jForward}), '}')
                foundBrace = 1;
            end
            jForward = jForward + 1;
        end
        numCaps = numCaps + 1;
        nominalVoltages.caps = [nominalVoltages.caps; nomVoltage];
        m = jForward;
        
    elseif (strcmp(char(glm_final{2}{j}),'transformer') ~= 0)
        stuff = 0;
    % if it's a non-residential load, create ZIP models for the load
    elseif (strcmp(char(glm_final{2}{j}),'load') ~= 0)
        foundBrace = 0; % reset this
        jForward = j + 1; % forward iterator
        while foundBrace ==0
            % set phases to phases_connected if you find it
            if strcmp(char(glm_final{2}{jForward}), 'nominal_voltage')
                nomVoltage = str2double(strrep(char(glm_final{3}{jForward}),';',''));
                break;
            end
            if strcmp(char(glm_final{2}{jForward}), '}')
                foundBrace = 1;
            end
            jForward = jForward + 1;
        end
        numCommLoads = numCommLoads + 1;
        nominalVoltages.commLoads = [nominalVoltages.commLoads; nomVoltage];
        m = jForward;
    % get split phase node
    elseif (strcmp(char(glm_final{2}{j}),'power_1') ~= 0 || strcmp(char(glm_final{2}{j}),'power_12') ~= 0)
        foundBrace = 0; % reset this
        jForward = j + 1; % forward iterator
        while foundBrace ==0
            % set phases to phases_connected if you find it
            if strcmp(char(glm_final{2}{jForward}), 'nominal_voltage')
                nomVoltage = str2double(strrep(char(glm_final{3}{jForward}),';',''));
                break;
            end
            if strcmp(char(glm_final{2}{jForward}), '}')
                foundBrace = 1;
            end
            jForward = jForward + 1;
        end
        numSpLoads = numSpLoads + 1;
        nominalVoltages.SpLoads = [nominalVoltages.SpLoads; nomVoltage];
        m = jForward;
    end
end
feederParameters.nominalVoltages = nominalVoltages;

numTrafoConfig = 0;
trafoConfigRatings = {};
m =0;
% get transformer ratings and whatever we need to pull from GLD
for j = 1:size(config_final{1},2)-1
    if (m >= j)
        continue; % this is used to skip over certain lines
    end
    
    if (strcmp(char(config_final{2}{j}),'transformer_configuration') ~= 0)
        foundBrace = 0; % reset this
        jForward = j + 1; % forward iterator
        while foundBrace ==0
            
            if strcmp(char(config_final{2}{jForward}), 'name')
                configName = strrep(char(config_final{3}{jForward}),';','');
                
            elseif strcmp(char(config_final{2}{jForward}), 'power_rating')
                power_rating = str2double(strrep(char(config_final{3}{jForward}),';',''));
                break;
            end
            if strcmp(char(config_final{2}{jForward}), '}')
                foundBrace = 1;
            end
            jForward = jForward + 1;
        end
        numTrafoConfig = numTrafoConfig + 1;
        trafoConfigRatings{numTrafoConfig,1} = configName;
        trafoConfigRatings{numTrafoConfig,2} = power_rating;
        m = jForward;
    end
end
  
m =0;
trafoNameRatings = {};
numTrafo = 0;
for j=1:(nn-1)

    % do the same for regulators as you did for capacitors
    if (strcmp(char(glm_final{2}{j}),'transformer') ~= 0)
        foundBrace = 0; % reset this
        jForward = j + 1; % forward iterator
        
        trafoName = '';
        trafoRating = 0;
        
        while foundBrace ==0
            if strcmp(char(glm_final{2}{jForward}), 'name')
                trafoName= strrep(char(glm_final{3}{jForward}),';','');
                
            elseif strcmp(char(glm_final{2}{jForward}), 'configuration')
                config = strrep(char(glm_final{3}{jForward}),';','');
                idx = find(ismember({trafoConfigRatings{:,1}}, config));

                try
                trafoRating = trafoConfigRatings{idx,2};
                catch
                    stuff =0;
                end 
            end
            if strcmp(char(glm_final{1}{jForward}), '}')
                foundBrace = 1;
            end
            jForward = jForward + 1;
        end
        numTrafo = numTrafo + 1;
        trafoNameRatings{numTrafo,1} = trafoName;
        trafoNameRatings{numTrafo,2} = trafoRating;
        
        m = jForward;
        
    end
end
%%%%%%%%%%%%%%%
% store the transformer information
feederParameters.trafoNameRatings = trafoNameRatings;

% store the node and load information in feeder parameters
feederParameters.houseAndNodeList = houseAndNodeList;

% set some values in the general parameter structure based on the things we
% have done thus far

% number of houses
generalParameters.houseTotal = total_no_houses; 
% planning load on the feeder
generalParameters.totalPlanningPower = sum(cell2mat(total_no_houses_power));
% the feeder name we have given
generalParameters.feeder = tech_file;
% where the feeder information gets written
generalParameters.writeFileDirectory = ['./', file5, '/'];
% number of ACs attached to the feeder
generalParameters.Pop = numAcs;
% enable communication network
% 0: no comm network , 1: consider comm network
generalParameters.considerCommNetwork = 0;

% generates the link file pulling nodal information from the feeder
generateLinkFile(generalParameters, houseAndNodeList, feederParameters);

% call the function that actually generates the houses and the TCLs within
% the houses.
[houseParameters, generalParameters] = GenerateRandomHouseParameters(generalParameters, houseAndNodeList, feederParameters, housesWithAc, houses_2zone, houses_1zone);

% END BUILD HOUSES AND TCLS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end



function generateLinkFile(parameterStructure, houseAndNodeList, feederParameters)
% This function creates the .link file for this specific feeder realization
% The link file allows the matlab-based house and tcl models to run within
% a gridlabd simulation. 

% get the feeder structure
feeder = parameterStructure.feeder;

% display something so we know what's happening in the script
disp('        Constructing link file')

% set the link file's name
filenameLink = strcat('matlabLink_',feeder,'.link');

% open the base link file and work from that
fid = fopen('matlabLink_baseGSL.link' );
cac = textscan(fid,'%s','Delimiter','\n','CollectOutput',true);
cac = cac{1};
fclose(fid);

% open a new file that will use the link information
fid = fopen([parameterStructure.writeFileDirectory, filenameLink],'w');

% loop through the lines in the base file
for jj = 1 : length(cac)
    
    % if the line contains 'addCdHere' and doesn't contain '#' (a comment
    % in gld files), then set the path for the cd in onInit
    if ~isempty(strfind(cac{jj},'addCdHere')) && isempty(strfind(cac{jj},'#'))
        loc1 = strfind(cac{jj},'addCdHere');
        loc2  = strfind(cac{jj},';');
        
        writeDirectory = parameterStructure.writeFileDirectory;
        writeDirectory = strrep(writeDirectory, '.', '');
        writeDirectory = strrep(writeDirectory(1:end-1), '/', '\');
        folderForInit = [parameterStructure.folderForOnInit writeDirectory];
        
        % if it's a mac, change the slashes
        if ismac == 1
            folderForInit = strrep(folderForInit,'\','/');
        end
        
        writeString = [cac{jj}(1:loc1-1), 'cd(''', folderForInit, ''')', cac{jj}(loc2:end)];
        
        fprintf(fid, '%s\n', writeString);
    
    % else copy the line over
    else
        
        fprintf(fid, '%s\n', cac{jj});
        
    end
end

% create regulator variables to get from gld
numberOfRegulators = feederParameters.numberOfRegulators;
regulatorList = feederParameters.regulatorList;
regulatorSenseNodes = feederParameters.regulatorSenseNodes;

% get regulator tap positions and sense voltage
for idx = 1:numberOfRegulators
    % export line (sending regulator tap positions to matlab)
    exLine1 = ['export ' regulatorList{idx} '.tap_A   regulator' num2str(idx) '_tapA'];
    exLine2 = ['export ' regulatorList{idx} '.tap_B   regulator' num2str(idx) '_tapB'];
    exLine3 = ['export ' regulatorList{idx} '.tap_C   regulator' num2str(idx) '_tapC'];
        
    % print the lines to the file
    fprintf(fid, '%s\n', exLine1);
    fprintf(fid, '%s\n', exLine2);
    fprintf(fid, '%s\n', exLine3);
     
    % TODO: get regulator voltage
    exLine1 = ['export ' regulatorSenseNodes{idx} '.voltage_A   regulator' num2str(idx) '_voltageA'];
    exLine2 = ['export ' regulatorSenseNodes{idx} '.voltage_B   regulator' num2str(idx) '_voltageB'];
    exLine3 = ['export ' regulatorSenseNodes{idx} '.voltage_C   regulator' num2str(idx) '_voltageC'];
    
    % print the lines to the file
    fprintf(fid, '%s\n', exLine1);
    fprintf(fid, '%s\n', exLine2);
    fprintf(fid, '%s\n', exLine3);
    
end


% create cap bank variables t get from gld
numberOfCapacitors = feederParameters.numberOfCapacitors;
capacitorList = feederParameters.capacitorList;

% get capacitor switch positions and voltages
for idx = 1:numberOfCapacitors
    % export line (sending cap switch positions to matlab)
    exLine1 = ['export ' capacitorList{idx} '.switchA   cap' num2str(idx) '_switchA' ];
    exLine2 = ['export ' capacitorList{idx} '.switchB   cap' num2str(idx) '_switchB'];
    exLine3 = ['export ' capacitorList{idx} '.switchC   cap' num2str(idx) '_switchC'];
        
    % print the lines to the file
    fprintf(fid, '%s\n', exLine1);
    fprintf(fid, '%s\n', exLine2);
    fprintf(fid, '%s\n', exLine3);
     
    %get capacitor voltages
    exLine1 = ['export ' capacitorList{idx} '.voltage_A   cap' num2str(idx) '_voltageA' ];
    exLine2 = ['export ' capacitorList{idx} '.voltage_B   cap' num2str(idx) '_voltageB'];
    exLine3 = ['export ' capacitorList{idx} '.voltage_C   cap' num2str(idx) '_voltageC'];
    
    % print the lines to the file
    fprintf(fid, '%s\n', exLine1);
    fprintf(fid, '%s\n', exLine2);
    fprintf(fid, '%s\n', exLine3);
    
    % get nominal voltage
    nomVexLine = ['export ' capacitorList{idx} '.cap_nominal_voltage   cap' num2str(idx) '_nominalVoltage'];

    % print the lines to the file
    fprintf(fid, '%s\n', nomVexLine);
    
end


% create ability to set commercial loads and get their voltages
numberOfCommLoads= feederParameters.numberOfCommLoads;
commercialLoadList = feederParameters.commercialLoadList;

% get commercial loads and voltages
for idx = 1:numberOfCommLoads
       
    % export line ( sending commload values to GLD)
    exLine1 = ['export ' commercialLoadList{idx,1} '.power_A   commercialLoad' num2str(idx) '_powerA' ];
    exLine2 = ['export ' commercialLoadList{idx,1} '.power_B   commercialLoad' num2str(idx) '_powerB'];
    exLine3 = ['export ' commercialLoadList{idx,1} '.power_C   commercialLoad' num2str(idx) '_powerC'];
        
    % print the lines to the file
    fprintf(fid, '%s\n', exLine1);
    fprintf(fid, '%s\n', exLine2);
    fprintf(fid, '%s\n', exLine3);
    
    % TODO: (maybe) set load values from MATLAB
    
    
    % get load voltages
    exLine1 = ['export ' commercialLoadList{idx,1} '.voltage_A   commercialLoad' num2str(idx) '_voltageA' ];
    exLine2 = ['export ' commercialLoadList{idx,1} '.voltage_B   commercialLoad' num2str(idx) '_voltageB'];
    exLine3 = ['export ' commercialLoadList{idx,1} '.voltage_C   commercialLoad' num2str(idx) '_voltageC'];
    
    % print the lines to the file
    fprintf(fid, '%s\n', exLine1);
    fprintf(fid, '%s\n', exLine2);
    fprintf(fid, '%s\n', exLine3);

    % get nominal voltage
    nomVexLine = ['export ' commercialLoadList{idx,1} '.nominal_voltage   commercialLoad' num2str(idx) '_nominalVoltage'];

    % print the lines to the file
    fprintf(fid, '%s\n', nomVexLine );
    
end

% create ability to set split phase loads and get their voltages
% after going through the base file, create import and export variables in
% the link file based on the node name and based on the node number
for kk=1:size(houseAndNodeList,1)

    % export line (sending power to matlab)
    exLine = ['export ' houseAndNodeList{kk,1} '.power_12   node' num2str(kk) '_powerMeasurement'];
    % send the voltage of the node to matlab
    vexLine = ['export ' houseAndNodeList{kk,1} '.voltage_12   node' num2str(kk) '_voltageMeasurement'];
    % send the nominal voltage of the node to matlab
    %nomVexLine = ['export ' houseAndNodeList{kk,1} '.nominal_voltage   node' num2str(kk) '_nominalVoltage'];
    % set the power draw at the node based on matlab's values
    imLine = ['import ' houseAndNodeList{kk,1} '.power_12   node' num2str(kk) '_powerCalculation'];
    
    % print the lines to the file
    fprintf(fid, '%s\n', exLine);
    fprintf(fid, '%s\n', vexLine);
    %fprintf(fid, '%s\n', nomVexLine);
    fprintf(fid, '%s\n', imLine);
end

% get the transformer powers
for kk = 1:size(feederParameters.trafoNameRatings, 1)
   exLine = ['export ' feederParameters.trafoNameRatings{kk,1} '.power_in   transformer' num2str(kk) '_powerMeasurement'];
   fprintf(fid, '%s\n', exLine); 
end

fclose( fid );

end



% This function are all things that the original GLD feeder generator did,
% and I have moved them into another function mainly because they are not
% critical for what we're doing.
function [glm_final, config_final, mm, nn, swing_node] =  generateConfigArrays(read_file)

test = textscan(read_file,'%s %s %s %s %s %s %s %s');
[~,d] = size(test);
[a,~] = size(test{1});

% Split up the configurations and feeder information into 2 different
% arrays
for i=1:a
    % GSL: loop goes through each line of the original feeder glm
    
    %TODO: add in catches for new objects (relays, etc.)
    % -- Definitely reclosers
    
    % Fill in the last line, cuz the array isn't almost never the same size
    if (i == a)
        if (strfind(test{1}{i},'}') ~= 0)
            for jindex = 2:d
                test{jindex}{i} = '';
            end
        end
        break;
    end
    
    % GSL: if you get to the name of an object, store it. Doesn't seem
    % to be used in this loop
    if (strfind(char(test{1}{i}),'name') ~= 0)
        my_name = char(test{2}{i});
    end
    
    % Remove id references and replace with names as neccessary
    if (strfind(char(test{2}{i}),'fuse:') ~= 0)
        test{2}{i} = 'fuse';
        %     elseif (strfind(char(test{1}{i}),'current_limit') ~= 0)
        %         test{2}{i} = '1000;';
    elseif (strfind(char(test{2}{i}),'load:') ~= 0)
        test{2}{i} = 'load';
    elseif (strfind(char(test{2}{i}),'triplex_meter:') ~= 0)
        test{2}{i} = 'triplex_meter';
    elseif (strfind(char(test{2}{i}),'meter:') ~= 0)
        test{2}{i} = 'meter';
    elseif (strfind(char(test{2}{i}),'triplex_node:') ~= 0)
        test{2}{i} = 'triplex_node';
    elseif (strfind(char(test{2}{i}),'node:') ~= 0)
        test{2}{i} = 'node';
    elseif (strfind(char(test{2}{i}),'switch:') ~= 0)
        test{2}{i} = 'switch';
    elseif (strfind(char(test{2}{i}),'recloser:') ~= 0)
        test{2}{i} = 'recloser';
    elseif (strfind(char(test{2}{i}),'overhead_line:') ~= 0)
        test{2}{i} = 'overhead_line';
    elseif (strfind(char(test{2}{i}),'regulator:') ~= 0)
        test{2}{i} = 'regulator';
    elseif (strfind(char(test{2}{i}),'transformer:') ~= 0)
        test{2}{i} = 'transformer';
    elseif (strfind(char(test{2}{i}),'capacitor:') ~= 0)
        test{2}{i} = 'capacitor';
    elseif (strfind(char(test{2}{i}),'triplex_line_conductor:') ~= 0)
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            
            % GL comment: if it's a triplex conductor, loop through the
            % object definition and copy over to the config variable
            % while replacing entries with blanks
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            
            % GL comment: make sure the last line is a '}' with nothing
            % else
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        elseif (strfind(char(test{1}{i}),'configuration') ~= 0)
            temp = strrep(test{2}{i},'triplex_line_conductor:','');
            temp2 = strcat('triplex_line_conductor_',temp);
            test{2}{i} = temp2;
        else
            disp('I missed something. Check line 75ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'triplex_line:') ~= 0)
        test{2}{i} = 'triplex_line';
    elseif (strfind(char(test{2}{i}),'underground_line:') ~= 0)
        test{2}{i} = 'underground_line';
        % Move all of the configuration files into a different array
    elseif (strfind(char(test{2}{i}),'triplex_line_configuration:') ~= 0)
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            
            % take configuration data and port it from test to config
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        elseif (strfind(char(test{1}{i}),'configuration') ~= 0)
            temp = strrep(test{2}{i},'triplex_line_configuration:','');
            temp2 = strcat('triplex_line_configuration_',temp);
            test{2}{i} = temp2;
        else
            disp('I missed something. Check line 50ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'line_configuration:') ~= 0)
        
        % if it is a line configuration, port it from test to config
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        elseif (strfind(char(test{1}{i}),'configuration') ~= 0)
            temp = strrep(test{2}{i},'line_configuration:','');
            temp2 = strcat('line_configuration_',temp);
            test{2}{i} = temp2;
        else
            disp('I missed something. Check line 131ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'line_spacing:') ~= 0)
        
        % take line spacing data and port it from test to config
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        else
            disp('I missed something. Check line 152ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'overhead_line_conductor:') ~= 0)
        
        % take over head line conductor info and port it over to config
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        else
            disp('I missed something. Check line 173ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'underground_line_conductor:') ~= 0)
        
        % take underground line conductor info and port it to config
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        else
            disp('I missed something. Check line 194ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'regulator_configuration:') ~= 0)
        
        % take regulator configuration info and port it over to config
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            
            % GSL: copy regulator configuration over to config and
            % erase from test.
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        elseif (strfind(char(test{1}{i}),'configuration') ~= 0)
            temp = strrep(test{2}{i},'regulator_configuration:','');
            temp2 = strcat('regulator_configuration_',temp);
            test{2}{i} = temp2;
        else
            disp('I missed something. Check line 220ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'transformer_configuration:') ~= 0)
        
        %take transformer configuration info and port it over to
        % config
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                for r=1:d
                    config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'}') == 1)
                config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
        elseif (strfind(char(test{1}{i}),'configuration') ~= 0)
            temp = strrep(test{2}{i},'transformer_configuration:','');
            temp2 = strcat('transformer_configuration_',temp);
            test{2}{i} = temp2;
        else
            disp('I missed something. Check line 245ish, i = %d',i);
        end
    elseif (strfind(char(test{2}{i}),'SWING') ~= 0)
        for kk=1:d
            test{kk}{i}='';
        end
        
        swing_node = my_name;
    elseif (strfind(char(test{2}{i}),'recorder') ~= 0)
        if (strfind(char(test{1}{i}),'object') ~= 0)
            m = 0;
            while (strcmp(char(test{1}{i+m}),'};') ~= 1)
                for r=1:d
                    %config{r}{i+m} = test{r}{i+m};
                    test{r}{i+m} = '';
                end
                m = m+1;
            end
            if (strcmp(char(test{1}{i+m}),'};') == 1)
                %config{1}{i+m} = test{1}{i+m};
                for p=2:d
                    test{p}{i+m}= '';
                    %config{p}{i+m}= '';
                end
                test{1}{i+m} = '';
            end
            
            if (m == 0) %do the same without the ';'
                while (strcmp(char(test{1}{i+m}),'}') ~= 1)
                    for r=1:d
                        %config{r}{i+m} = test{r}{i+m};
                        test{r}{i+m} = '';
                    end
                    m = m+1;
                end
                if (strcmp(char(test{1}{i+m}),'}') == 1)
                    %config{1}{i+m} = test{1}{i+m};
                    for p=2:d
                        test{p}{i+m}= '';
                        %config{p}{i+m}= '';
                    end
                    test{1}{i+m} = '';
                end
            end
        end
    end
    
    
    % GSL: removes rows that start with the key value, and sometimes
    % the row after them. Applies mainly to the beginning of the feeder
    % file.
    if (strfind(char(test{1}{i}),'//') ~= 0)
        for n=1:d
            test{n}{i} = '';
        end
    elseif (strfind(char(test{1}{i}),'clock') ~= 0)
        for n=1:d
            test{n}{i} = '';
        end
    elseif (strfind(char(test{1}{i}),'solver_method') ~= 0)
        for n=1:d
            test{n}{i} = '';
        end
    elseif (strfind(char(test{1}{i}),'module') ~= 0)
        for n=1:d
            test{n}{i} = '';
        end
    elseif (strfind(char(test{1}{i}),'timestamp') ~= 0)
        for n=1:d
            test{n}{i} = '';
        end
    elseif (strfind(char(test{1}{i}),'stoptime') ~= 0)
        for n=1:d
            test{n}{i} = '';
        end
    elseif (strfind(char(test{1}{i}),'#set') ~= 0)
        for n=1:d
            test{n}{i} = '';
        end
    elseif (strfind(char(test{1}{i}),'timezone') ~= 0)
        for n=1:d
            test{n}{i} = '';
            test{n}{i+1} = '';
        end
    elseif (strfind(char(test{1}{i}),'default_maximum_voltage_error') ~= 0)
        for n=1:d
            test{n}{i} = '';
            test{n}{i+1} = '';
        end
    end
end

% Get rid of all those pesky spaces in the "feeder" file
%GSL: copy lines over to the final feeder file if they aren't blank
nn=1;
for i=1:a % loop through rows of test
    test_me = 0;
    for j=1:d % loop through columns of test
        if (strcmp(char(test{j}{i}),'')~=0)
            test_me = test_me+1; % keeps track of the number of blanks
        end
    end
    if (test_me == d)
        %do nothing cuz the lines are blank
    elseif (strfind(char(test{3}{i}),'{') ~= 0) % if its a { copy the line
        for j=1:d
            glm_final{j}{nn} = test{j}{i};
        end
        nn = nn + 1;
    elseif (strfind(char(test{1}{i}),'}') ~= 0) % if its a } copy the line
        for j=1:d
            glm_final{j}{nn} = test{j}{i};
        end
        nn = nn + 1;
    else % insert a tab and copy over the rest of the line
        for j=1:d
            if (j==1)
                glm_final{j}{nn} = '     ';
            else
                glm_final{j}{nn} = test{j-1}{i};
            end
        end
        nn = nn + 1;
    end
    
end

% Get rid of those nasty blank lines in the configuration file and re-name
% the configurations to get rid of all id references and use names
mm=1;
[~,y]=size(config{1});
for i=1:y % loop through the cols of the config array
    
    % GSL: check to see how many of the line components are blank
    test_me = 0;
    for j=1:d
        if (strcmp(char(config{j}{i}),'')~=0)
            test_me = test_me+1;
        end
    end
    
    % GSL: if all are blank, skip the line
    if (test_me == d)
        %do nothing cuz the lines are blank
    elseif (strfind(char(config{2}{i}),'triplex_line_configuration:') ~= 0)
        for j=1:d
            if (j==3)
                config_final{j}{mm} = config{j}{i};
                temp = strrep(config{2}{i},'triplex_line_configuration:','');
                temp2 = strcat('triplex_line_configuration_',temp,';');
                config_final{j}{mm+1} = temp2;
            elseif (j==2)
                config_final{2}{mm} = 'triplex_line_configuration';
                config_final{2}{mm+1} = 'name';
            elseif (j==1)
                config_final{j}{mm} = config{j}{i};
                config_final{j}{mm+1} ='     ';
            else
                config_final{j}{mm} = config{j}{i};
                config_final{j}{mm+1} ='';
            end
        end
        mm = mm + 2;
    elseif (strfind(char(config{2}{i}),'line_configuration:') ~= 0)
        for j=1:d
            if (j==3)
                config_final{j}{mm} = config{j}{i};
                temp = strrep(config{2}{i},'line_configuration:','');
                temp2 = strcat('line_configuration_',temp,';');
                config_final{j}{mm+1} = temp2;
            elseif (j==2)
                config_final{2}{mm} = 'line_configuration';
                config_final{2}{mm+1} = 'name';
            elseif (j==1)
                config_final{j}{mm} = config{j}{i};
                config_final{j}{mm+1} ='     ';
            else
                config_final{j}{mm} = config{j}{i};
                config_final{j}{mm+1} ='';
            end
        end
        mm = mm + 2;
    elseif (strfind(char(config{2}{i}),'transformer_configuration:') ~= 0)
        for j=1:d
            if (j==3)
                config_final{j}{mm} = config{j}{i};
                temp = strrep(config{2}{i},'transformer_configuration:','');
                temp2 = strcat('transformer_configuration_',temp,';');
                config_final{j}{mm+1} = temp2;
            elseif (j==2)
                config_final{2}{mm} = 'transformer_configuration';
                config_final{2}{mm+1} = 'name';
            elseif (j==1)
                config_final{j}{mm} = config{j}{i};
                config_final{j}{mm+1} ='     ';
            else
                config_final{j}{mm} = config{j}{i};
                config_final{j}{mm+1} ='';
            end
        end
        mm = mm + 2;
    elseif (strfind(char(config{1}{i}),'}') ~= 0)
        for j=1:d
            config_final{j}{mm} = config{j}{i};
        end
        mm = mm + 1;
    elseif (strfind(char(config{2}{i}),'triplex_line_conductor:') ~= 0)
        for j=1:d
            if (j==2)
                config_final{j}{mm} = 'triplex_line_conductor';
            else
                config_final{j}{mm} = config{j}{i};
            end
        end
        mm = mm + 1;
    elseif (strfind(char(config{2}{i}),'underground_line_conductor:') ~= 0)
        if (strfind(char(config{1}{i}),'object') ~= 0)
            for j=1:d
                if (j==2)
                    config_final{j}{mm} = 'underground_line_conductor';
                    config_final{j}{mm+1} = 'name';
                elseif (j==1)
                    config_final{j}{mm} = config{j}{i};
                    config_final{j}{mm+1} = '     ';
                elseif (j==3)
                    config_final{j}{mm} = config{j}{i};
                    temp = strrep(config{2}{i},'underground_line_conductor:','');
                    temp2 = strcat('underground_line_conductor_',temp,';');
                    config_final{j}{mm+1} = temp2;
                else
                    config_final{j}{mm} = config{j}{i};
                end
            end
            mm = mm + 1;
        else
            for j=1:d
                if (j==3)
                    temp = strrep(config{2}{i},'underground_line_conductor:','');
                    temp2 = strcat('underground_line_conductor_',temp);
                    config_final{j}{mm} = temp2;
                elseif (j==1)
                    config_final{j}{mm} = '     ';
                else
                    config_final{j}{mm} = config{j-1}{i};
                end
            end
        end
        mm = mm + 1;
    elseif (strfind(char(config{2}{i}),'overhead_line_conductor:') ~= 0)
        if (strfind(char(config{1}{i}),'object') ~= 0)
            for j=1:d
                if (j==2)
                    config_final{j}{mm} = 'overhead_line_conductor';
                    config_final{j}{mm+1} = 'name';
                elseif (j==1)
                    config_final{j}{mm} = config{j}{i};
                    config_final{j}{mm+1} = '     ';
                elseif (j==3)
                    config_final{j}{mm} = config{j}{i};
                    temp = strrep(config{2}{i},'overhead_line_conductor:','');
                    temp2 = strcat('overhead_line_conductor_',temp,';');
                    config_final{j}{mm+1} = temp2;
                else
                    config_final{j}{mm} = config{j}{i};
                end
            end
            mm = mm + 1;
        else
            for j=1:d
                if (j==3)
                    temp = strrep(config{2}{i},'overhead_line_conductor:','');
                    temp2 = strcat('overhead_line_conductor_',temp);
                    config_final{j}{mm} = temp2;
                elseif (j==1)
                    config_final{j}{mm} = '     ';
                else
                    config_final{j}{mm} = config{j-1}{i};
                end
            end
        end
        mm = mm + 1;
    elseif (strfind(char(config{2}{i}),'regulator_configuration:') ~= 0)
        for j=1:d
            if (j==2)
                config_final{j}{mm} = 'regulator_configuration';
                config_final{j}{mm+1} = 'name';
            elseif (j==1)
                config_final{j}{mm} = config{j}{i};
                config_final{j}{mm+1} = '     ';
            elseif (j==3)
                config_final{j}{mm} = config{j}{i};
                temp = strrep(config{2}{i},'regulator_configuration:','');
                temp2 = strcat('regulator_configuration_',temp,';');
                config_final{j}{mm+1} = temp2;
            else
                config_final{j}{mm} = config{j}{i};
            end
        end
        mm = mm + 2;
    elseif (strfind(char(config{2}{i}),'line_spacing:') ~= 0)
        if (strfind(char(config{1}{i}),'spacing') ~= 0)
            for j=1:d
                if (j==3)
                    temp = strrep(config{2}{i},'line_spacing:','');
                    temp2 = strcat('line_spacing_',temp);
                    config_final{j}{mm} = temp2;
                elseif (j==1)
                    config_final{j}{mm} = '     ';
                else
                    config_final{j}{mm} = config{j-1}{i};
                end
            end
            mm = mm + 1;
        else
            for j=1:d
                if (j==3)
                    temp = strrep(config{2}{i},'line_spacing:','');
                    temp2 = strcat('line_spacing_',temp,';');
                    config_final{j}{mm+1} = temp2;
                    config_final{j}{mm} = config{j}{i};
                elseif (j==1)
                    config_final{j}{mm+1} = '     ';
                    config_final{j}{mm} = config{j}{i};
                elseif (j==2)
                    config_final{j}{mm+1} = 'name';
                    config_final{j}{mm} = 'line_spacing';
                else
                    config_final{j}{mm} = config{j}{i};
                    config_final{j}{mm+1} = '';
                end
            end
            mm = mm + 2;
        end
    else
        for j=1:d
            if (j==1)
                config_final{j}{mm} = '     '; %#ok<*SAGROW>
            else
                config_final{j}{mm} = config{j-1}{i};
            end
        end
        mm = mm + 1;
    end
    
end

end

% This is another function that I created to separate out some non-critical
% code from the original script. It creates ZIP loads from non-residential
% loads.
function [m, parent_name, parent_phase, totalLoad] = generateCommercialZipLoads(j, glm_final, taxonomy_data, write_file, feederParameters)

% These are parameters to determine ZIP load parameters of commercial 
% buildings. These could be moved to the top of the main script, but we
% move this here as this is where they are used. 
tech_data.c_z_pf = .95; % .87 used bc that's how they size the transformer in taxfeederdata 
tech_data.c_i_pf = .95;
tech_data.c_p_pf = .95;
tech_data.c_zfrac = 0.2;
tech_data.c_ifrac = 0.4;
tech_data.c_pfrac = 1 - tech_data.c_zfrac - tech_data.c_ifrac;


        fprintf(write_file,'\nobject load {\n');
        m = j;
        
        load_A = 0;
        load_B = 0;
        load_C = 0;
        
        % Add up all of the loads at that node on each phase
        while (strcmp(char(glm_final{1}{m}),'}') == 0)
            if (strcmp(char(glm_final{2}{m}),'constant_power_A') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                load_A = load_A + abs(bb_real + 1i*bb_imag);
            elseif (strcmp(char(glm_final{2}{m}),'constant_power_B') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                load_B = load_B + abs(bb_real + 1i*bb_imag);
            elseif (strcmp(char(glm_final{2}{m}),'constant_power_C') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                load_C = load_C + abs(bb_real + 1i*bb_imag);
            elseif (strcmp(char(glm_final{2}{m}),'constant_impedance_A') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                S_A = abs(taxonomy_data.nom_volt2^2 / (bb_real * 1i*bb_imag));
                
                load_A = load_A + S_A;
            elseif (strcmp(char(glm_final{2}{m}),'constant_impedance_B') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                S_B = abs(taxonomy_data.nom_volt2^2 / (bb_real * 1i*bb_imag));
                
                load_B = load_B + S_B;
            elseif (strcmp(char(glm_final{2}{m}),'constant_impedance_C') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                S_C = abs(taxonomy_data.nom_volt2^2 / (bb_real * 1i*bb_imag));
                
                load_C = load_C + S_C;
            elseif (strcmp(char(glm_final{2}{m}),'constant_current_A') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                S_A = abs(taxonomy_data.nom_volt2 * (bb_real * 1i*bb_imag));
                
                load_A = load_A + S_A;
            elseif (strcmp(char(glm_final{2}{m}),'constant_current_B') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                S_B = abs(taxonomy_data.nom_volt2 * (bb_real * 1i*bb_imag));
                
                load_B = load_B + S_B;
            elseif (strcmp(char(glm_final{2}{m}),'constant_current_C') ~= 0)
                bb_real = real(str2num(glm_final{3}{m}));
                bb_imag = imag(str2num(glm_final{3}{m}));
                
                S_C = abs(taxonomy_data.nom_volt2 * (bb_real * 1i*bb_imag));
                
                load_C = load_C + S_C;
            elseif (strcmp(char(glm_final{2}{m}),'name') ~= 0)
                parent_name = char(glm_final{3}{m});
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{m}),char(glm_final{2}{m}),char(glm_final{3}{m}),char(glm_final{4}{m}),char(glm_final{5}{m}),char(glm_final{6}{m}),char(glm_final{7}{m}),char(glm_final{8}{m}));
            elseif (strcmp(char(glm_final{2}{m}),'phases') ~= 0)
                parent_phase = char(glm_final{3}{m});
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{m}),char(glm_final{2}{m}),char(glm_final{3}{m}),char(glm_final{4}{m}),char(glm_final{5}{m}),char(glm_final{6}{m}),char(glm_final{7}{m}),char(glm_final{8}{m}));
            elseif (strcmp(char(glm_final{2}{m}),'nominal_voltage') ~= 0)
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{m}),char(glm_final{2}{m}),char(glm_final{3}{m}),char(glm_final{4}{m}),char(glm_final{5}{m}),char(glm_final{6}{m}),char(glm_final{7}{m}),char(glm_final{8}{m}));
            elseif (strcmp(char(glm_final{2}{m}),'parent') ~= 0)
                fprintf(write_file,'%s %s %s %s %s %s %s %s\n',char(glm_final{1}{m}),char(glm_final{2}{m}),char(glm_final{3}{m}),char(glm_final{4}{m}),char(glm_final{5}{m}),char(glm_final{6}{m}),char(glm_final{7}{m}),char(glm_final{8}{m}));
            end
            
            m = m + 1;
        end
        
        scalingFactor = feederParameters.backgroundDemandScalingFactor;
        %scalingFactor = 0.75;
        %warning('scalingFactor hardcoded')
        
        % Re-make the loads using the specified ZIP fractions & pf
        if (load_A > 0)
            fprintf(write_file,'      base_power_A %f;\n',load_A*scalingFactor);
            fprintf(write_file,'      power_pf_A %f;\n',tech_data.c_p_pf);
            fprintf(write_file,'      current_pf_A %f;\n',tech_data.c_i_pf);
            fprintf(write_file,'      impedance_pf_A %f;\n',tech_data.c_z_pf);
            fprintf(write_file,'      power_fraction_A %f;\n',tech_data.c_pfrac);
            fprintf(write_file,'      current_fraction_A %f;\n',tech_data.c_ifrac);
            fprintf(write_file,'      impedance_fraction_A %f;\n',tech_data.c_zfrac);
        end
        if (load_B > 0)
            fprintf(write_file,'      base_power_B %f;\n',load_B*scalingFactor);
            fprintf(write_file,'      power_pf_B %f;\n',tech_data.c_p_pf);
            fprintf(write_file,'      current_pf_B %f;\n',tech_data.c_i_pf);
            fprintf(write_file,'      impedance_pf_B %f;\n',tech_data.c_z_pf);
            fprintf(write_file,'      power_fraction_B %f;\n',tech_data.c_pfrac);
            fprintf(write_file,'      current_fraction_B %f;\n',tech_data.c_ifrac);
            fprintf(write_file,'      impedance_fraction_B %f;\n',tech_data.c_zfrac);
        end
        if (load_C > 0)
            fprintf(write_file,'      base_power_C %f;\n',load_C*scalingFactor);
            fprintf(write_file,'      power_pf_C %f;\n',tech_data.c_p_pf);
            fprintf(write_file,'      current_pf_C %f;\n',tech_data.c_i_pf);
            fprintf(write_file,'      impedance_pf_C %f;\n',tech_data.c_z_pf);
            fprintf(write_file,'      power_fraction_C %f;\n',tech_data.c_pfrac);
            fprintf(write_file,'      current_fraction_C %f;\n',tech_data.c_ifrac);
            fprintf(write_file,'      impedance_fraction_C %f;\n',tech_data.c_zfrac);
        end
        fprintf(write_file,'}\n\n');

totalLoad = (load_A + load_B + load_C)*scalingFactor;
end