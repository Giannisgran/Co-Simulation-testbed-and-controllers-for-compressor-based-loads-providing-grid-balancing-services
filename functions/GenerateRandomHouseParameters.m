% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [houseParameters, generalParameters] = GenerateRandomHouseParameters(generalParameters, houseAndNodeList, feederParameters, housesWithAc, houses_2zone, houses_1zone)

fprintf('Generating house parameters\n')

% set the random number generator seed
rng(2598)

% get the total number of houses that were populated onto the feeder
% previously
totalHouses = generalParameters.houseTotal;

% get the number of houses connected to each split-phase node
numHouses = [houseAndNodeList{:,3}]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GENERATE RANDOM FLOOR AREAS
if generalParameters.perc_2zone == 0  % only single-zone houses
    houseParameters = generateFloorArea_1zone(generalParameters);
else  % some houses are two-zone
    houseParameters = generateFloorArea_2zone(generalParameters.perc_2zone, generalParameters.Pop, totalHouses, houses_2zone, houses_1zone);
end
% END GENERATE RANDOM FLOOR AREAS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% keep the 2-zone percentage in the house struct as well
houseParameters.perc_2zone = generalParameters.perc_2zone; 
% set this for outputing from the function
houseParameters.totalHouses = totalHouses; 
houseParameters.housesWithAc = reshape(housesWithAc, [], 1);
% houseParameters.floorArea = floorAreaHouses;

% allocate space for storing background load values and which node each
% house is connected to. Populated below
houseParameters.backgroundLoad = zeros(totalHouses,1);
houseParameters.nodeConnectedTo = zeros(totalHouses,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GENERATE NON-TCL LOAD INFORMATION


% generate the background load based on house size from here until end of
% block of code

% keep track of the overall house
houseIterator = 1;
        

% loop through each triplex/split-phase node
for nodeNumber = 1:length(numHouses)
    
    % if the load was large enough to put a house on the node
    if numHouses(nodeNumber)>0
        
        % initialize the running sum of background load on the node
        realBackgroundLoad = 0; % in W
        reactiveBackgroundLoad = 0; % in VAR
        
        scalingFactor = feederParameters.backgroundDemandScalingFactor;

        % for each house connected to the node
        for houseCounter = 1:numHouses(nodeNumber)
            
            % set the background load using this function from gld. Note
            % that GSL isn't sure where this code is in GLD, but this is
            % what Steph based her code on. 
            realBLthisHouse = 1e3*(324.9/8907)*houseParameters.floorArea(houseIterator)^0.442; % formula based on Feeder_Generator.m script
            reacBLthisHouse = realBLthisHouse*tan(acos(0.95));
            
            % scale the base load by the scaling factor
            realBLthisHouse = realBLthisHouse * scalingFactor;
            reacBLthisHouse = reacBLthisHouse * scalingFactor;
            
            % set the BL load for each house and its node
            houseParameters.backgroundLoad(houseIterator) = complex(realBLthisHouse,reacBLthisHouse);
            houseParameters.nodeConnectedTo(houseIterator) = nodeNumber;
            
            % accumulate the load of each house for the node
            realBackgroundLoad = realBackgroundLoad + realBLthisHouse; % in W
            reactiveBackgroundLoad = reactiveBackgroundLoad + reacBLthisHouse; % in VAR
        
            houseIterator = houseIterator + 1;
        
        end
        
        % set the background load within the cell structure to the complex
        % background load. 
        houseAndNodeList{nodeNumber,4} = complex(realBackgroundLoad,reactiveBackgroundLoad);

    else
        % set the background load within the cell structure to the planning
        % load for that node
        houseAndNodeList{nodeNumber,4} =  houseAndNodeList{nodeNumber,2};
    end
    
end

% END GENERATE NON-TCL LOAD INFORMATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

function [houseParameters] = generateFloorArea_1zone(generalParameters)
% This function links simulated houses with PSI houses in order to inherit
% the square footage values.

houseParameters.numAcs = generalParameters.Pop;
% read files with square footage on the 1st column and probabilities on the second
% values_probabilities = readtable('./Data/square_footage_probability.csv');
% load sq ft info
psi_dataport = readtable('./dataFiles/PSI_dataID_squareFootage.csv');
% load the house ids of the houses processed for the delays
% houseID_1thermostat_processed = load('houseIDs_delayObjects_1thermostat_2020_08_13__2020_08_31.mat').houseID_processed;
houseID_1thermostat_processed = load('./dataFiles/houseIDs_delayObjects_1thermostat_onlyTo2bins_2020_08_13__2020_08_31.mat').houseID_processed;
% initialize for the sq ft of the processed houses - can't preallocate
% because we don't know how many houses will be found in the dataport csv
values_1floor = [];
% vectors in which we will keep the indices that correspond to houses for
% which we know the sq ft - some are not included in the csv
valid_ind_1floor = [];
% keep a list of the house IDs not found in the dataport csv
houseIDs_not_found = [];
for i = 1 : length(houseID_1thermostat_processed) % find the square ft of the processed houses with a single zone
    % seach for the house ID in the dataport csv
    found = psi_dataport.dataid==houseID_1thermostat_processed(i);    
    if any(found) && ~isnan(psi_dataport.total_square_footage(found)) % check if house sq ft info is in csv
        valid_ind_1floor = [valid_ind_1floor i];
        values_1floor = [values_1floor psi_dataport.total_square_footage(found)];
    else % house id wasn't found in the dataport csv
        houseIDs_not_found = [houseIDs_not_found houseID_1thermostat_processed(i)];
        values_1floor = [values_1floor NaN]; % put nan in order to keep same number of elements as in delayObject
    end   
end

% randomly pick with replacement indices that correspond to the processed houses
house_ind = datasample(valid_ind_1floor, generalParameters.houseTotal);
% match the sq ft with the house indeces drawn to create the population
floorArea = values_1floor(house_ind);

%% Update struct with floor area and indeces of real houses
houseParameters.floorArea = floorArea;
% do the same with the indices of the real houses
houseParameters.house_ind = reshape(house_ind, 1,[]);
% also keep real house ID
houseID = houseID_1thermostat_processed(house_ind);
houseParameters.realHouseID = reshape(houseID, 1,[]);

end

function [houseParameters] = generateFloorArea_2zone(perc_2zone, pop, totalHouses, houses_2zone, houses_1zone)
% load sq ft info
psi_dataport = readtable('./dataFiles/PSI_dataID_squareFootage.csv');
% load the house ids of the houses processed for the delays
houseID_2thermostats_processed = load('houseIDs_delayObjects_2thermostats_onlyTo2bins_2020_08_13__2020_08_31_no_outliers.mat').houseID_processed;
% houseID_1thermostat_processed = load('houseIDs_delayObjects_1thermostat_2020_08_13__2020_08_31.mat').houseID_processed;
houseID_1thermostat_processed = load('houseIDs_delayObjects_1thermostat_onlyTo2bins_2020_08_13__2020_08_31.mat').houseID_processed;
% load sf values and corresponding probabilities for houses without A/C
values_probabilities_noAC = readtable('./dataFiles/square_footage_probability.csv'); % use the csv with mixed houses
% keep vector that has 0 if a tcl is not part of a 2-zone house and 1 if it is
houseParameters.is2zone = false(totalHouses, 1);
houseParameters.is2zone(houses_2zone) = 1;
% house indices that don't have an A/C
houses_noAC = setdiff([1:totalHouses], [houses_2zone houses_1zone]);

% number of ACs that correspond to single and 2-zone houses
num2zones = length(houses_2zone);
num1zones = length(houses_1zone);
% store the new number of ACs, assume 2-zone houses have two ACs even
% though it is not true in reality
houseParameters.numAcs = pop;
% store number of single and 2 zone houses
houseParameters.num2zone = num2zones;
houseParameters.num1zone = num1zones;
% number of houses without AC
houseParameters.numHousesWithouAC = length(houses_noAC);
% store houses zones and acs
houseParameters.houses_2zone = houses_2zone;
houseParameters.houses_1zone = houses_1zone;
houseParameters.houses_noAC = houses_noAC;

% initialize for the sq ft of the processed houses - can't preallocate
% because we don't know how many houses will be found in the dataport csv
values_2floor_1st = []; values_2floor_2nd = []; values_1floor = [];
% vectors in which we will keep the indices that correspond to houses for
% which we know the sq ft - some are not included in the csv
valid_ind_2floor = []; valid_ind_1floor = [];
% keep a list of the house IDs not found in the dataport csv
houseIDs_not_found = [];
for i = 1 : length(houseID_2thermostats_processed) % find the square ft of the processed houses with 2 zones 
    % seach for the house ID in the dataport csv
    found = psi_dataport.dataid==houseID_2thermostats_processed(i);
    if any(found) % check if house sq ft info is in csv
        valid_ind_2floor = [valid_ind_2floor i];
        values_2floor_1st = [values_2floor_1st psi_dataport.first_floor_square_footage(found)];
        values_2floor_2nd = [values_2floor_2nd psi_dataport.second_floor_square_footage(found)];     
    else % house id wasn't found in the dataport csv
        houseIDs_not_found = [houseIDs_not_found houseID_2thermostats_processed(i)];
        values_2floor_1st = [values_2floor_1st NaN]; % put nan in order to keep same number of elements as in delayObject
        values_2floor_2nd = [values_2floor_2nd NaN]; % put nan in order to keep same number of elements as in delayObject
    end
end
for i = 1 : length(houseID_1thermostat_processed) % find the square ft of the processed houses with a single zone
    % seach for the house ID in the dataport csv
    found = psi_dataport.dataid==houseID_1thermostat_processed(i);    
    if any(found) && ~isnan(psi_dataport.total_square_footage(found)) % check if house sq ft info is in csv
        valid_ind_1floor = [valid_ind_1floor i];
        values_1floor = [values_1floor psi_dataport.total_square_footage(found)];
    else % house id wasn't found in the dataport csv
        houseIDs_not_found = [houseIDs_not_found houseID_1thermostat_processed(i)];
        values_1floor = [values_1floor NaN]; % put nan in order to keep same number of elements as in delayObject
    end   
end
% Print a message with the houses not found
disp(['Houses (' num2str(houseIDs_not_found) ') were not found in dataport and were therefore skipped.' ])

% randomly pick with replacement indices that correspond to the processed houses
ind_2floor = datasample(valid_ind_2floor, num2zones);
ind_1floor = datasample(valid_ind_1floor, houseParameters.num1zone);
% match the sq ft with the house indices drawn to create the population
floorArea_2floor_1st = values_2floor_1st(ind_2floor);
floorArea_2floor_2nd = values_2floor_2nd(ind_2floor);
floorArea_1floor = values_1floor(ind_1floor);

[floorArea_noAC,~] = sample_from_distribution(values_probabilities_noAC, houseParameters.numHousesWithouAC);

% Update struct with floor area and indeces of real houses
floorArea = zeros(totalHouses, 1);
floorArea(houses_2zone) = floorArea_2floor_1st + floorArea_2floor_2nd;
floorArea(houses_1zone) = floorArea_1floor;
floorArea(houses_noAC) = floorArea_noAC;
houseParameters.floorArea = floorArea;
% do the same with the indices of the real houses
house_ind = zeros(totalHouses, 1);
house_ind(houses_2zone) = ind_2floor;
house_ind(houses_1zone) = ind_1floor;
house_ind(houses_noAC) = 0*floorArea_noAC; % don't have houses without a/c in psi data
houseParameters.house_ind = reshape(house_ind, 1,[]);
% also keep real house ID
houseID = zeros(totalHouses, 1);
houseID(houses_1zone) = houseID_1thermostat_processed(ind_1floor);
houseID(houses_2zone) = houseID_2thermostats_processed(ind_2floor);
houseID(houses_noAC) = 0*floorArea_noAC; % don't have houses without a/c in psi data
houseParameters.realHouseID = reshape(houseID, 1,[]);

end