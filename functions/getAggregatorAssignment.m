% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function aggregatorAssignment = getAggregatorAssignment(simulationOptions,houseParameters, tclParameters)
% This function takes some parameters and options, then outputs which
% aggregator each TCL is assigned to. 

% switch through different methods of assigning aggregators.
switch simulationOptions.aggregatorAssignment
    case 'random'
        
        % get the aggregator distribution and check that it sums to 1.
        aggDist = cumsum(simulationOptions.aggregatorAssignmentProb);
        aggDist = [0; aggDist(:)];
        if aggDist(end) ~=1
            error('The probability distribution assigning TCLs to aggregators does not sum to 1')
        end
        
        %generate random values for TCLs
        rand_vals = rand(numel(houseParameters.housesWithAc),1);  %spans zero to one
        
        %look into CDF to see which index the rand val corresponds to, then store
        %values in TCL parameters
        out_val = interp1(aggDist,[0:1/(length(aggDist)-1):1],rand_vals); %spans zero to one
        ind = ceil(out_val*length(simulationOptions.aggregatorAssignmentProb));
        if houseParameters.perc_2zone == 0  % only single-zone houses
            aggregatorAssignment = ind;
        else  % some 2-zone houses exist, hence number of houses in not the same as the number of TCLs
            aggregatorAssignment = zeros(houseParameters.numAcs, 1);  % will keep which TCLs are managed by the current aggregator
            for i = 1 : houseParameters.numAcs  % loop through the TCLs 
                curr_house = tclParameters.tcl_house_mapping(i);
                % find the position of the house in the list with houses 
                % that include an A/C, same position as the ind vector
                pos = find(houseParameters.housesWithAc == curr_house); 
                % if house that corresponds to the current TCL is managed 
                % by the aggregator then add that to the list
                if ind(pos) == 1  
                    aggregatorAssignment(i) = 1;
                end
            end
        end
    otherwise
        error('invalid aggregator assignment method given')
end

end

