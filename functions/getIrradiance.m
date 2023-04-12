% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [irradiance]=getIrradiance(tclOptions)
% Function that takes the simulation parameters and outputs a time series
% of irradiance from the PSI data that is linearly interpolated
% from 1 hour intervals to the intervals of the simulation

switch tclOptions.weatherOption   
    case 1
        irradiance = 10*ones(43201,1); % in btu/hr.ft2
    case 2
        irradiance = 25*ones(43201,1);% in btu/hr.ft2
    case 3 
        irradiance = 0*ones(43201,1);% in btu/hr.ft2  
    case 4
        irradiance = interp1([1,43201],[10,25],[1:43201]);
    otherwise 
        error('Option for weather/temperature data did not correspond to switch value')
end

end