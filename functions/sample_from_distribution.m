% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [sampled_values,ind] = sample_from_distribution(sizeInfo, num_samples)
%{
This function takes as input a table, which includes values on the
first column and the corresponding probabilities on the second column, and
generates num_samples based on the given distribution. Also returns the
indeces picked.
%}
sizeValues = table2array(sizeInfo(:,1)); % get the set of sizes in the data
sizeDistribution = table2array(sizeInfo(:,2)); % get probability of each size

% normalize the distribution to 1, then remove any bins that have zero
% probability, then convert the pdf to a cdf, the do a random draw
distribution = sizeDistribution./sum(sizeDistribution); % normalize
distributionNoZeros = distribution(distribution ~= 0); % remove zero probabilities
valuesNoZeros = sizeValues(distribution~=0); % remove values with zero probability
CDF = cumsum(distributionNoZeros); % create a CDF from the PDF
CDF = [0; CDF(:)]; % add 0 as the first entry to make valid CDF
rand_vals = rand(num_samples,1);  % get a random draw from zero to one for each house
        
% convert random draw to elements of the cdf, then get the indices for each
% house's value, then populate the actual square footage values
out_val = interp1(CDF,[0:1/(length(CDF)-1):1],rand_vals); %spans zero to one
ind = ceil(out_val*length(valuesNoZeros));
sampled_values = valuesNoZeros(ind);

end

