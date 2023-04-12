% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [outdoorTemperature]=getOutdoorTemperature(tclOptions)
% Function that takes the simulation parameters and outputs a time series
% of outdoor temperature from the PSI data that is linearly interpolated
% from 1 hour intervals to the intervals of the simulation

switch tclOptions.weatherOption   
    case 1
        outdoorTemperatureInF = 90*ones(43201,1); % in F, 2 sec per time-step
    case 2
        outdoorTemperatureInF = 100*ones(43201,1);% in F, 2 sec per time-step       
    case 3 
        outdoorTemperatureInF = 70*ones(43201,1);% in F, 2 sec per time-step 
    case 4
        outdoorTemperatureInF = interp1([1,43201],[90,150],[1:43201]);
    otherwise 
        error('Option for weather/temperature data did not correspond to switch value')
end

% convert to degC
outdoorTemperature = (outdoorTemperatureInF-32)./1.8;  

end