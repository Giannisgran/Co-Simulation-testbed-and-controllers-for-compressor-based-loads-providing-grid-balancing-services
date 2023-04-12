% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [tclParameters]=GenerateUpdatedRandomTclParameters(parameterStructure, houseParameters, tclOptions, outdoorTemperature, irradiance)
%{
This function takes the previously generated house parameters and builds
TCL models within the relevant houses. Thermal parameters, dynamic matrices
for state updates, power draws, lockouts, temperature delays and node 
assignment are all carried out here.
Input:
    parameterStructure: struct
        Includes the timestep for the simulation.
    houseParameters: struct
        Info for the houses, e.g. number of ACs, houses that have an AC.
    tclOptions: struct
        Specified options for the current population that is being
        constructed.
    outdoorTemperature: vector of floats
        Outdoor temperature timeseries.
    irradiance: vector of floats
        Irradiance timeseries.
Output:
    tclParameters: struct
        Includes info for the TCLs, as for example thermal parameters,
        dynamic matrices, initial conditions etc.
%}
fprintf('Updating TCL parameters \n')

% set the random number generator seed
rng(2548)

% pull some parameters from the structure
numAcs = houseParameters.numAcs;
numHousesWithAcs = length(houseParameters.housesWithAc);
timeStepInHr = parameterStructure.timeStepInHr;
timeStepInSec = parameterStructure.timeStepInSec;

% create mapping between houses and TCLs through a vector that has as many
% elements as the total number of TCLs and each entry corresponds to the
% house that the TCL is located at
if houseParameters.perc_2zone == 0  % only single-zone houses
    tclParameters.tcl_house_mapping = houseParameters.housesWithAc;
else
    tclParameters.tcl_house_mapping = [repelem(houseParameters.houses_2zone, 1, 2) houseParameters.houses_1zone]; % have 2zones first then single-zone houses
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET LOCKOUT AND OUTDOOR TEMPERATURE VALUES

lockTimeOnInTimeSteps = tclOptions.lockTimeOnInTimeSteps; % sets the lockout time
lockoutTimeOffInTimeSteps = tclOptions.lockoutTimeOffInTimeSteps; % sets the lockout time
initialTa = outdoorTemperature(1); % sets initial Ta based on time series
initialIrradiance = irradiance(1); % sets initial irradiance based on time series
% Set the lockout times that limit cycling when the unit switches on and
% off, and also set the time remaining in the lockout
tclParameters.lockoutTimeOffInTimeSteps = lockoutTimeOffInTimeSteps .*ones(numAcs,1);
tclParameters.lockoutTimeOnInTimeSteps = lockTimeOnInTimeSteps .*ones(numAcs,1);
tclParameters.timeUntilUnlocked = 0 .* ones(numAcs,1);

% store the percentage of 2-zone houses in the aggregation
tclParameters.perc_2zone = houseParameters.perc_2zone;

% store the current outdoor tempeature seen at each TCL
tclParameters.T_a = initialTa*ones(numAcs,1); % degC % ambient temperature
% store the current solar irradiance seen at each TCL
tclParameters.irradiance = initialIrradiance*ones(numAcs,1);
% option to include inrush
tclParameters.inrushOption = tclOptions.inrushOption;
% END LOCKOUT ...
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET POWER DRAW CHARACERISTICS

tclParameters = generatePowerDrawCharacteristics(tclParameters, houseParameters, numAcs);

% END POWER DRAWS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET THERMAL PARAMETERS

% uses options passed into the function to determine the variability of the
% TCL thermal parameters
switch tclOptions.heterogeneityOption
    case 1
        variability = .1;
    case 2 
        variability = 0;
    otherwise
        error('Invalid heterogeneity option')
end

buildingParameterOption = tclOptions.buildingParameterOption;
sfOption = tclOptions.sfOption;
% tclParameters = generateThermalParameters(tclParameters, houseParameters, numAcs, variability, buildingParameterOption, initialIrradiance);
if houseParameters.perc_2zone == 0  % only single-zone houses
    tclParameters = generateThermalParameters_1zone(tclParameters, houseParameters, numAcs, variability, buildingParameterOption, sfOption, initialIrradiance);
else  % some houses are two-zone
    tclParameters = generateThermalParameters_2zone(tclParameters, houseParameters, variability, buildingParameterOption, sfOption, initialIrradiance);
end
% END SET THERMAL PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAKE STATE UPDATE MATRICES

if houseParameters.perc_2zone == 0  % only single-zone houses
    tclParameters = generateDynamicMatrices_1zone(tclParameters, numAcs, timeStepInHr);
else  % some houses are two-zone
    tclParameters = generateDynamicMatrices_2zone(tclParameters, timeStepInHr);
end
% END MAKE STATE UPDATE MATRICES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAKE SETPOINTS AND DEADBANDS
if houseParameters.perc_2zone == 0  % only single-zone houses
    tclParameters = generate_deadbands_1zone(tclParameters, tclOptions.Tsp_range, tclOptions.db_range, numAcs);
else  % some houses are two-zone
    tclParameters = generate_deadbands_2zone(tclParameters, tclOptions.Tsp_range, tclOptions.db_range, numAcs);
end
% END MAKE SETPOINTS AND DEADBANDS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Guassian noise used in the TCL state updates
tclParameters.noiseMean = tclOptions.noiseMean;  % mean
tclParameters.noiseSd = tclOptions.noiseSd; % standard deviation

% initialize and fill a vector that contains the node that each AC
% corresponds to
acNodeAssignment = zeros(numAcs,1);
for thisAc = 1:numAcs
    thisHouse = tclParameters.tcl_house_mapping(thisAc);
    acNodeAssignment(thisAc) = houseParameters.nodeConnectedTo(thisHouse);
end
    
tclParameters.acNodeAssignment = acNodeAssignment;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAKE TEMPERATURE DELAY TIMES
if houseParameters.perc_2zone == 0  % only single-zone houses
    tclParameters = generateTemperatureDelayValues_1zone(tclParameters, numAcs, variability, timeStepInSec, tclOptions.temperatureDelayOption);
else  % some houses are two-zone
    tclParameters = generateTemperatureDelayValues_2zone(tclParameters, numAcs, variability, timeStepInSec, tclOptions.temperatureDelayOption);
end    
% END MAKE TEMPERATURE DELAY TIMES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

function distributionValuesAndCdf = converPdfToCdf (inputDistribution, inputValues)

% normalize the distribution to 1, then remove any bins that have zero
% probability, then convert the pdf to a cdf
distribution = inputDistribution./sum(inputDistribution);

% remove zero entries from the distribution
distributionNoZeros = distribution(distribution ~= 0);
% remove values that corresponded to zero probabilities
values = inputValues;
valuesNoZeros = values(distribution~=0);

% form the cdf
CDF = cumsum(distributionNoZeros);
CDF = [0; CDF(:)];

% create array with cdf values in first column and realization values in
% second column. Add NaN at the end to make them the same length
distributionValuesAndCdf = [CDF [valuesNoZeros; NaN]];
end

function tclParameters = generatePowerDrawCharacteristics(tclParameters, houseParameters, numAcs)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Map the houses with an A/C to the corresponding entries of the regression
% coefficients based on the real underlying house ID
ind_regression = find_regression_houseIDs(houseParameters, tclParameters.tcl_house_mapping);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET POWER DRAW CHARACTERISTICS
% Get the multiple linear regression coefficients that resulted from
% analyzing the PSI one second data. The form of the model is the
% following:
%  P or Q =a1*temp+a2*voltage+a3 (temp in F, voltage is in 240)
coefficients_QvsTinFandVin240 = table2array(readtable('./dataFiles/coefficients_reactive_multiple_regression.csv', 'ReadVariableNames',false));
coefficients_PvsTinFandVin240 = table2array(readtable('./dataFiles/coefficients_real_multiple_regression.csv', 'ReadVariableNames',false));

% allocate arrays to store the regression coefficients for each air conditioner
K_p_0 = zeros(numAcs,1); % p, intercept
K_p_temp = zeros(numAcs,1); % p, temp coeff
K_p_voltage = zeros(numAcs,1); % p, voltage coeff
K_q_0 = zeros(numAcs,1); % q, intercept
K_q_temp = zeros(numAcs,1); % q, temp coeff
K_q_voltage = zeros(numAcs,1); % q, voltage coeff

% allocate cell arrays to store the inrush distributions for each AC. They
% are cell arrays because the inrush distributions are not all the same
% dimension
inrushDistributionP = cell(numAcs,1); 
inrushDistributionQ = cell(numAcs,1);

% loop through each of the air conditioners
for whatHouseData = 1:numAcs 
    % pull a house number from the set of available houses (with the data)
    houseNumber = ind_regression(whatHouseData);

    % set the coefficients for real power regression model
    K_p_0(whatHouseData) = coefficients_PvsTinFandVin240(houseNumber, 3); % p, intercept
    K_p_temp(whatHouseData) = coefficients_PvsTinFandVin240(houseNumber, 1);  % p, slope wrt temp
    K_p_voltage(whatHouseData) = coefficients_PvsTinFandVin240(houseNumber, 2);  % p, slope wrt voltage
    
    % set the coefficients for the reactive power regression model
    K_q_0(whatHouseData) = coefficients_QvsTinFandVin240(houseNumber,3); % q, intercept
    K_q_temp(whatHouseData) = coefficients_QvsTinFandVin240(houseNumber,1); % q, slope    
    K_q_voltage(whatHouseData) = coefficients_QvsTinFandVin240(houseNumber,2); % q, slope    
    
    % find inrush distributions
    if tclParameters.inrushOption
        % get the inrush distribution information from the csv
        inrushInfo = readtable(['./dataFiles/inrushP/house' num2str(houseNumber) '_P_inrushStatistics.csv'], 'HeaderLines',1);
        inrushDistribution = table2array(inrushInfo(:,1));
        inrushValues= table2array(inrushInfo(:,2));

        % convert the probability distirbution into a set of values and
        % probabilities
        distributionValuesAndCdf = converPdfToCdf (inrushDistribution, inrushValues);

        % store the probabilities and values in the cell array
        inrushDistributionP{whatHouseData} = distributionValuesAndCdf;

        % Do the same for the Q inrush
        inrushInfo = readtable(['./dataFiles/inrushQ/house' num2str(houseNumber) '_Q_inrushStatistics.csv'], 'HeaderLines',1);
        inrushDistribution = table2array(inrushInfo(:,1));
        inrushValues= table2array(inrushInfo(:,2));
        distributionValuesAndCdf = converPdfToCdf (inrushDistribution, inrushValues);
        inrushDistributionQ{whatHouseData} = distributionValuesAndCdf; % CFD of the values below
    end
end

% set a vector of voltages and temperatures to set power draw values
voltage240 = 240.*ones(numAcs,1); % 240-based voltage to be updated in simulation
TaInF = tclParameters.T_a*1.8 + 32; % degF
P_power_draw = K_p_0 + TaInF .* K_p_temp + voltage240.* K_p_voltage; %kW
Q_power_draw = K_q_0 + TaInF .* K_q_temp + voltage240.* K_q_voltage; % kVAR

% generate rated power of each device for 95 degF and 240V
ratedRealPower = K_p_0 + 95 .* K_p_temp + voltage240.* K_p_voltage;

% END SET POWER DRAW CHARACTERISTICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


tclParameters.P_power_draw = P_power_draw; %active power draw when TCL is on
tclParameters.Q_power_draw = Q_power_draw; % reactive power draw when TCL is on
tclParameters.voltage240 = voltage240;
tclParameters.K_p_0 = K_p_0;
tclParameters.K_p_temp = K_p_temp;
tclParameters.K_p_voltage= K_p_voltage;
tclParameters.K_q_0 = K_q_0;
tclParameters.K_q_temp = K_q_temp;
tclParameters.K_q_voltage = K_q_voltage;
tclParameters.ratedReadPower = ratedRealPower;

% do this separate because it acts weird if you don't
tclParameters.inrushDistributionP = inrushDistributionP;
tclParameters.inrushDistributionQ = inrushDistributionQ;

end

function ids_powerDraw_regression = find_regression_houseIDs(houseParameters, tcl_house_mapping)
% Find for each TCL the corresponding index for the power draw regression coefficients
ids_powerDraw_regression_coef = readtable('./dataFiles/ids_coefficients_multiple_regression.csv', 'ReadVariableNames', false);
numAcs = houseParameters.numAcs;
ids_powerDraw_regression = zeros(numAcs, 1);  % preallocate
for i = 1 : numAcs
    curr_house = tcl_house_mapping(i);
    curr_realHouseID = houseParameters.realHouseID(curr_house);
    j = find(ids_powerDraw_regression_coef.Var1 == curr_realHouseID);
    if isempty(j) % did not find the house, pick something with similar footage
        switch curr_realHouseID
            case 8292
                j = 11;
            case 7541
                j = 7;
            case 6990
                j = 2;
            case 3310
                j = 6;
            case 7875
                j = 20;
            case 171
                j = 25;
            case 974
                j = 21;
            case 2129
                j = 10;
            case 6390
                j = 2;
            case 4213
                j = 31;
            case 3009
                j = 4; 
            otherwise
                error('House %d not taken into account for the regression coefficient house matching', houseParameters.realHouseID(i));
        end
    end
    ids_powerDraw_regression(i) = j;
end

end

function tclParameters = generateThermalParameters_1zone(tclParameters, houseParameters, numAcs, variability, buildingParameterOption, sfOption, ISR)

tclParameters.numAcs = numAcs;
% sets the range for the uniform distributions
parameter_variability = variability;
% load floor areas of the houses
A = houseParameters.floorArea(houseParameters.housesWithAc); % [ft^2]
A = reshape(A, [], 1);  % make sure it is a column vector
if variability == 0 || sfOption == 1 % homogenous population
    A(:) = 2500; % [ft^2]
end
% store floor areas in struct
tclParameters.floorAreas = A;
% conversion factors
Btu_per_hr_to_kW = 3412.142^-1;
Btu_to_kWh = 0.00029307107;
degF_over_degC = 1.8;

%% Default parameters taken from gridlabd
n = 1; % number of stories for each house
R = 1.5; % floor aspect ratio (measurement of a building’s floor area in relation to the size of the lot/parcel that the building is located on)
h = 8; % ceiling height in ft
ECR = 1; % Exterior ceiling, fraction of total (ECR)
EFR = 1; % Exterior floor, fraction of total (EFR)
EWR = 1; % Exterior wall, fraction of total (EWR)
WWR = 0.07; % Window/exterior wall area ratio (WWR)
nd = 4; % number of doors
A1d = 19.5; % (ft^2) area of one door
WET = 0.6; % Window, exterior transmission coefficient (WET)
Rw = 19; % (F.ft^2.hr/Btu) R-value, walls
Rc = 30; % (F.ft^2.hr/Btu) R-value, ceilings
Rf = 22; % (F.ft^2.hr/Btu) R-value, floors
Rd = 5; % (F.ft^2.hr/Btu) R-value, doors
I = 0.5; % (1/hr) Infiltration volumetric air exchange rate (I)
IWR = 1.5; % Interior/exterior wall surface ratio (IWR)
hs = 1.46; % (Btu/hr.°F.ft^2) Interior surface heat transfer coefficient
mf = 2; % (Btu/°F.ft^2) Total thermal mass, per unit floor area
fs = 0.5; % Solar gain fraction to mass
fi = 0.5; % Internal gain fraction to mass
fm = 0; % HVAC delivered fraction to mass
Ug = 0.47; % (Btu / hr.°F.ft^2) window U-value
SHGC_nom = 0.67; % solar heat gain coefficient
SHOC = 400; % [btu / (hr*occupant) ] sensible heat from each occupant
NOC = 4; % number of occupants
K = 3412; % [btu / (hr*kW) ] convertion factor of background load to internal heat gain
foc = 0; % household occupancy factor, from Tesfatsion notes

%% Calculate needed parameters
% needed areas
Awt = 2*n*h*(1+R)*sqrt(A/n/R); % (ft^2) gross exterior wall area
Ag = WWR*Awt*EWR; % (ft^2) gross window area
Ad = nd*A1d; % (ft^2) total door area
% Aw = (Awt-Ag-Ad)*EWR; % (ft^2) net exterior wall area
Aw = Awt*EWR-Ag-Ad; % (ft^2) net exterior wall area, corrected from source code, webpage is wrong
Ac = A/n*ECR; % (ft^2) net exterior ceiling area
Af = A/n*EFR; % (ft^2) net exterior floor area
SHGC = SHGC_nom*WET*Ag;
Pback = (324.9/8907)*A.^0.442*tan(acos(0.95)); % [kW] from testbed script that generates house parameters

%% Compute thermal parameters except Qh
U_a = Ag*Ug + Ad/Rd + Aw/Rw + Ac/Rc + Af/Rf + 0.018*A*h*I; % [Btu / (hr*degF)]
% Hm = hs*Aw/EWR + Awt*IWR + Ac*n/ECR; % webpage is wrong on this one
H_m = hs*(Awt-Ag-Ad + Awt*IWR + Ac*n/ECR); % [Btu /(hr*degF)], corrected from source code, webpage is wrong
C_a = 3*0.018*A*h; % [Btu / degF]
C_m = A*mf - 2*0.018*A*h; % [Btu / degF]
Q_s = SHGC*ISR; % [btu / hr]
Q_i = K*Pback + SHOC*NOC*foc; % [btu / hr]
Q_m = fi*Q_i + fs*Q_s; % [btu / hr]

% uses the options passed into the function to determine how to construct
% the buildings. 
switch buildingParameterOption
    case 1
       % Do nothing because the defaults are built in above. 
    case 2
        % reduce the thermal masses to half their value to make temperature
        % evolutions more dynamic. 
        C_a = C_a/2;
        C_m = C_m/2;
    otherwise
        error('Invalid building parameter option')
end


% calculate the design cooling capacity based on U_a and square footage

% set some defaults basesedon 
% http://gridlab-d.shoutwiki.com/wiki/Residential_module_user%27s_guide#Sizing_Calculations
latentCooling = .35; % fraction of latent cooling
designOutdoorTemp = 100; % design outdoor temperature [degF]
designIndoorTemp = 75; % [degF] the default that they give

 % internal gains based on floor area
designInternalGains = 167.09 .* A.^0.442; %[Btu/hr]

% set the design solar gains 
% set the design solar gains (GLD website is wrong because it has SHGC)
ISR_design = 195; % [Btu/(hr*ft^2)] incident solar radiation, from for GLD webpage
designSolarGains = ISR_design*Ag*SHGC_nom*WET; % [Btu/hr]

% oversizing for the unit
Oversizing_factor = 0; % don't add extra capacity

% calculate the design cooling load
designSensibleCooling = U_a * (designOutdoorTemp-designIndoorTemp) + designInternalGains + designSolarGains;
designTotalCooling = (1+latentCooling) * designSensibleCooling;

% calculate the unit load assuming 6000 Btu/hr increments
% [Btu/hr]
designTotalCoolingBtu = round( (designTotalCooling * (1 + Oversizing_factor) + 3000) / 6000 ) * 6000 ;

% calculate the Cooling capacity based on the outdoor temperature
K_0 = 1.48924533;
K_1 = -0.00514995;

T_a = tclParameters.T_a;
coolCapacityBasedOnTout = K_0 + K_1 * (T_a*1.8+32); % T_out assumed to be in in degF
designTotalCoolingBtu = designTotalCoolingBtu .* (1+(rand(numAcs,1)-.5).* parameter_variability);
coolingCapacity = (designTotalCoolingBtu .* coolCapacityBasedOnTout).*Btu_per_hr_to_kW;

Q_h = -coolingCapacity/(1+latentCooling); %[kW]
P_trans = abs(Q_h);

% randomize the parameters and create a vector of them that stores a value
% for each TCL
Q_i = Q_i .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_per_hr_to_kW;  % [kW]
Q_s = Q_s .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_per_hr_to_kW;  % [kW]
Q_m = Q_m * Btu_per_hr_to_kW;  % [kW]
U_a = U_a .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_per_hr_to_kW * degF_over_degC; % [kW / degC]
C_a = C_a .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_to_kWh * degF_over_degC; % [kWh / degC]
H_m = H_m .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_per_hr_to_kW * degF_over_degC; % [kW/ degC]
C_m = C_m .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_to_kWh * degF_over_degC; % [kWh / degC]

tclParameters.parameter_variability = parameter_variability; % range in uniform distributions for parameters
tclParameters.U_a = U_a; % the following bunch of parameters to TCL specific values
tclParameters.C_a = C_a;
tclParameters.H_m = H_m;
tclParameters.C_m = C_m;
tclParameters.Q_i = Q_i;
tclParameters.Q_s = Q_s;
tclParameters.Q_m = Q_m;
tclParameters.Q_h = Q_h;
tclParameters.SHGC = SHGC;
tclParameters.mass_internal_gain_fraction = fi;
tclParameters.mass_solar_gain_fraction = fs;
tclParameters.P_trans = P_trans; %energy transfer ...
tclParameters.coolingCapacity = coolingCapacity;
tclParameters.K_0 = K_0;
tclParameters.K_1 = K_1;
tclParameters.designTotalCoolingBtu = designTotalCoolingBtu;
tclParameters.latentCooling = latentCooling;
% keep the house that each TCL corresponds to
tclParameters.house_ind = houseParameters.house_ind(tclParameters.tcl_house_mapping);
tclParameters.designInternalGains = designInternalGains;
tclParameters.designSolarGains = designSolarGains;

end

function tclParameters = generateThermalParameters_2zone(tclParameters, houseParameters, parameter_variability, buildingParameterOption, sfOption, ISR)
%{
This function generates the thermal parameters for the TCL population,
including 2-zone houses based on the square footage values.
Input:
    tclParameters : struct
        Struct with info for the TCL, e.g. the houses that they correspond
    houseParameters: struct
        Struct with information regarding the houses, e.g. floor area
    parameter_variability: float
        Number that indicates the desired variability we want to have in
        the parameters between the TCLs
    buildingParameterOption: int
        If 1, keeps the same Ca, Cm, if 2 it uses the half of them
    ISR: float
        Initial value for the solar irradiance in btu/hr.ft2.
Output:
    tclParameters: struct
        Struct with various parameters of the TCL population
%}
rng(114234);
%% Load from house struct
% number of AC units
numAcs = houseParameters.numAcs;
tclParameters.numAcs = numAcs;
% load floor areas of the houses
A = houseParameters.floorArea(tclParameters.tcl_house_mapping); % [ft^2]
A = reshape(A, [], 1);  % make sure it is a column vector
if parameter_variability == 0 || sfOption == 1 % homogenous population
    A(:) = 2500; % [ft^2]
end
% store floor areas in struct
tclParameters.floorAreas = A;
% background load
Pback = (324.9/8907)*A.^0.442*tan(acos(0.95)); % [kW] from testbed script that generates house parameters
% zones
first_zone = [1:2:2*houseParameters.num2zone]; % 1st floor of houses that are 2-zone
second_zone = [2:2:2*houseParameters.num2zone]; % 2nd floor of houses that are 2-zone
houses_1_zone = [2*houseParameters.num2zone+1:houseParameters.numAcs]; % houses with only one zone/floor

%% Default parameters taken from gridlabd
n = 2; % number of stories for each house
R = 1.5; % floor aspect ratio (measurement of a building’s floor area in relation to the size of the lot/parcel that the building is located on)
h = 8; % ceiling height in ft
ECR = 1; % Exterior ceiling, fraction of total (ECR)
EFR = 1; % Exterior floor, fraction of total (EFR)
EWR = 1; % Exterior wall, fraction of total (EWR)
WWR = 0.07; % Window/exterior wall area ratio (WWR)
nd = 4; % number of doors
A1d = 19.5; % (ft^2) area of one door
WET = 0.6; % Window, exterior transmission coefficient (WET)
Rw = 19; % (F.ft^2.hr/Btu) R-value, walls
Rc = 30; % (F.ft^2.hr/Btu) R-value, ceilings
Rf = 22; % (F.ft^2.hr/Btu) R-value, floors
Rd = 5; % (F.ft^2.hr/Btu) R-value, doors
I = 0.5; % (1/hr) Infiltration volumetric air exchange rate (I)
IWR = 1.5; % Interior/exterior wall surface ratio (IWR)
hs = 1.46; % (Btu/hr.°F.ft^2) Interior surface heat transfer coefficient
mf = 2; % (Btu/°F.ft^2) Total thermal mass, per unit floor area
fs = 0.5; % Solar gain fraction to mass
fi = 0.5; % Internal gain fraction to mass
fm = 0; % HVAC delivered fraction to mass
Ug = 0.47; % (Btu / hr.°F.ft^2) window U-value
SHGC_nom = 0.67; % solar heat gain coefficient
SHOC = 400; % [btu / (hr*occupant) ] sensible heat from each occupant
NOC = 4; % number of occupants
K = 3412; % [btu / (hr*kW) ] convertion factor of background load to internal heat gain
foc = 0; % household occupancy factor, from Tesfatsion notes

%% conversion factors
Btu_per_hr_to_kW = 3412.142^-1;
Btu_to_kWh = 0.00029307107;
degF_over_degC = 1.8;
% square footage of the whole house. When 2-zones exist, sum them and
% assign the resulting value to both zones
A_whole = A;
A_whole(first_zone) = A_whole(first_zone) + A_whole(second_zone);
A_whole(second_zone) = A_whole(first_zone) + A_whole(second_zone);

%% Specify extra parameters
ISR_design = 195; % [Btu/(hr*ft^2)] incident solar radiation, from for GLD webpage
latentCooling = .35; % fraction of latent cooling
designIndoorTemp = 75; % [degF] the default that they give
designOutdoorTemp = 95; % design outdoor temperature [degF]
Oversizing_factor_1zone = 0; % oversizing for the 1-zone units
Oversizing_factor_2zone = 0; % oversizing for the 2-zone units
% coefficients for Cooling capacity based on the outdoor temperature
K_0 = 1.48924533; % 1.48924533*0.6;
K_1 = -0.00514995; % 0.00514995;

%% Calculate needed parameters
% needed areas
Awt = 2*n*h*(1+R)*sqrt(A/n/R); % (ft^2) gross exterior wall area
Ag = WWR*Awt*EWR; % (ft^2) gross window area
Ad = nd*A1d; % (ft^2) total door area
Aw = Awt*EWR-Ag-Ad; % (ft^2) net exterior wall area, corrected from source code, webpage is wrong
Ac = A/n*ECR; % (ft^2) net exterior ceiling area
Af = A/n*EFR; % (ft^2) net exterior floor area
% areas that correspond to the whole house, used with 2-zone house cases
Awt_whole = 2*n*h*(1+R)*sqrt(A_whole/n/R); % (ft^2) gross exterior wall area
Ag_whole = WWR*Awt_whole*EWR; % (ft^2) gross window area
Ad_whole = nd*A1d; % (ft^2) total door area
Aw_whole = Awt_whole*EWR-Ag_whole-Ad_whole; % (ft^2) net exterior wall area, corrected from source code, webpage is wrong
Ac_whole = A_whole/n*ECR; % (ft^2) net exterior ceiling area
Af_whole = A_whole/n*EFR; % (ft^2) net exterior floor area
SHGC = SHGC_nom*WET*Ag;

%% Compute thermal parameters except Qh
U_a = Ag*Ug + Ad/Rd + Aw/Rw + Ac/Rc + Af/Rf + 0.018*A*h*I; % [Btu / (hr*degF)]
U_a_whole = Ag_whole*Ug + Ad_whole/Rd + Aw_whole/Rw + Ac_whole/Rc + Af_whole/Rf + 0.018*A_whole*h*I; % [Btu / (hr*degF)]
H_m = hs*(Awt-Ag-Ad + Awt*IWR + Ac*n/ECR); % [Btu /(hr*degF)], corrected from source code, webpage is wrong
C_a = 3*0.018*A*h; % [Btu / degF]
C_m = A*mf - 2*0.018*A*h; % [Btu / degF]
Q_s = SHGC*ISR; % [btu / hr]
% Q_i = 0*ones(length(A),1); % [btu / hr]
Q_i = K*Pback + SHOC*NOC*foc; % [btu / hr]
Q_m = fi*Q_i + fs*Q_s; % [btu / hr]
H_z = 0.1*ones(numAcs, 1);
H_z(houses_1_zone) = 0; % for single-zone houses H_z should be zero

% uses the options passed into the function to determine how to construct
% the buildings. 
switch buildingParameterOption
    case 1
       % Do nothing because the defaults are built in above. 
    case 2
        % reduce the thermal masses to half their value to make temperature
        % evolutions more dynamic. 
        C_a = C_a/2;
        C_m = C_m/2;
    otherwise
        error('Invalid building parameter option')
end

%% Compute Qh
% oversizing factors, can be different for 1-zone and 2-zone TCLs
Oversizing_factor = Oversizing_factor_1zone*ones(length(A), 1);
Oversizing_factor(first_zone) = Oversizing_factor_2zone;
Oversizing_factor(second_zone) = Oversizing_factor_2zone;
% internal gains based on floor area of the whole house
designInternalGains = 167.09 .* A_whole .^0.442; %[Btu/hr]

% set the design solar gains (GLD website is wrong because it has SHGC)
designSolarGains = ISR_design*Ag_whole*SHGC_nom*WET; % [Btu/hr]

% calculate the design cooling load
designSensibleCooling = U_a_whole * (designOutdoorTemp-designIndoorTemp) + designInternalGains + designSolarGains;
designTotalCooling = (1+latentCooling) * designSensibleCooling;

% calculate the unit load assuming 6000 Btu/hr increments
% [Btu/hr]
designTotalCoolingBtu = round( (designTotalCooling .* (1 + Oversizing_factor) + 3000) / 6000 ) * 6000 ;

coolCapacityBasedOnTout = K_0 + K_1 * (tclParameters.T_a*1.8+32); % T_out assumed to be in degF
designTotalCoolingBtu = designTotalCoolingBtu .* (1+(rand(numAcs,1)-.5).* parameter_variability);
designTotalCoolingBtu(second_zone) = designTotalCoolingBtu(first_zone);
coolingCapacity = (designTotalCoolingBtu .* coolCapacityBasedOnTout).*Btu_per_hr_to_kW;

Q_h = -coolingCapacity/(1+latentCooling); %[kW]
P_trans = abs(Q_h);

%% Unit conversion into kW and degC
U_a = U_a .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_per_hr_to_kW * degF_over_degC; % [kW / degC]
C_a = C_a .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_to_kWh * degF_over_degC; % [kWh / degC]
H_m = H_m .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_per_hr_to_kW * degF_over_degC; % [kW/ degC]
C_m = C_m .* (1+(rand(numAcs,1)-.5)* parameter_variability) * Btu_to_kWh * degF_over_degC; % [kWh / degC]
Q_i = Q_i .* (1+(rand(numAcs,1)-.5).* parameter_variability)*Btu_per_hr_to_kW;  % [kW]
Q_s = Q_s .* (1+(rand(numAcs,1)-.5).*parameter_variability)*Btu_per_hr_to_kW;  % [kW]
Q_m = Q_m .* (1+(rand(numAcs,1)-.5).* parameter_variability)*Btu_per_hr_to_kW;  % [kW]

%% Store parameters in struct
tclParameters.parameter_variability = parameter_variability; % range in uniform distributions for parameters
tclParameters.U_a = U_a; % the following bunch of parameters to TCL specific values
tclParameters.C_a = C_a;
tclParameters.H_m = H_m;
tclParameters.C_m = C_m;
tclParameters.Q_i = Q_i;
tclParameters.Q_s = Q_s;
tclParameters.Q_m = Q_m;
tclParameters.Q_h = Q_h;
tclParameters.H_z = H_z;
tclParameters.SHGC = SHGC;
tclParameters.mass_internal_gain_fraction = fi;
tclParameters.mass_solar_gain_fraction = fs;
tclParameters.P_trans = P_trans; %energy transfer ...
tclParameters.coolingCapacity = coolingCapacity;
tclParameters.K_0 = K_0;
tclParameters.K_1 = K_1;
tclParameters.designTotalCoolingBtu = designTotalCoolingBtu;
tclParameters.latentCooling = latentCooling;

% store zone links, first column correspond to the first zone of the houses
% that have 2 zones and the second column to the second zone
tclParameters.tcls_2zone_first = first_zone;
tclParameters.tcls_2zone_second = second_zone;
tclParameters.tcls_1zone = houses_1_zone;
tclParameters.numHouses_1zone = length(houses_1_zone);  % number of houses that are single-zone
tclParameters.numHouses_2zone = length(first_zone);  % number of houses that are 2-zone
tclParameters.is2zone = houseParameters.is2zone(tclParameters.tcl_house_mapping);
tclParameters.house_ind = houseParameters.house_ind(tclParameters.tcl_house_mapping);

end

function [tclParameters] = generate_deadbands_1zone(tclParameters, Tsp_range, db_range, numAcs)
%{
Creates deadbands for each TCL randomly within the limits specified.
Input:
    tclParameters: struct
        Contains info for the TCLs as for example their thermal parameters.
    Tsp_range: vector
        Vector that contains the min and max edges of the setpoints that
        will be randomly generated.
    db_range: vector
        Vector that contains the min and max edges of the deadband widths.
    numAcs: int
        Number of A/C units.
Output:
    tclParameters: struct
        Updated struct with new deadbands, setpoints.
%}

% Generate deadbands and setpoints
T_sp = (Tsp_range(2)-Tsp_range(1))*rand(numAcs,1) + Tsp_range(1);
deadband = (db_range(2)-db_range(1))*rand(numAcs,1) + db_range(1);
T_min = T_sp-deadband./2; %in deg C
T_max = T_sp+deadband./2; %in deg C
% save to struct
tclParameters.T_sp = T_sp;
tclParameters.T_max = T_max;
tclParameters.T_min = T_min;
tclParameters.deadband = deadband;
tclParameters.Tsp_range = Tsp_range;
tclParameters.db_range = db_range;

end

function [tclParameters] = generate_deadbands_2zone(tclParameters, Tsp_range, db_range, numAcs)
%{
Creates deadbands for each TCL randomly within the limits specified.
Input:
    tclParameters: struct
        Contains info for the TCLs as for example their thermal parameters.
    Tsp_range: vector
        Vector that contains the min and max edges of the setpoints that
        will be randomly generated.
    db_range: vector
        Vector that contains the min and max edges of the deadband widths.
    numAcs: int
        Number of A/C units.
Output:
    tclParameters: struct
        Updated struct with new deadbands, setpoints.
%}

% Generate deadbands and setpoints
T_sp = (Tsp_range(2)-Tsp_range(1))*rand(numAcs,1) + Tsp_range(1);
deadband = (db_range(2)-db_range(1))*rand(numAcs,1) + db_range(1);
% make the setpoint, deadband width for 2nd zone equal to the 1st zone
T_sp(tclParameters.tcls_2zone_second) = T_sp(tclParameters.tcls_2zone_first); 
deadband(tclParameters.tcls_2zone_second) = deadband(tclParameters.tcls_2zone_first); 
T_min = T_sp-deadband./2; %in deg C
T_max = T_sp+deadband./2; %in deg C
% save to struct
tclParameters.T_sp = T_sp;
tclParameters.T_max = T_max;
tclParameters.T_min = T_min;
tclParameters.deadband = deadband;
tclParameters.Tsp_range = Tsp_range;
tclParameters.db_range = db_range;

end

function tclParameters = generateTemperatureDelayValues_1zone(tclParameters, numAcs, variability, timeStepInSec, temperatureDelayOption)
% Create the distributions for the temperature delays

tclParameters.temperatureDelayOption = temperatureDelayOption; % load option
if temperatureDelayOption == 4
    delayObjects = load('./dataFiles/delayObjects_1thermostat_onlyTo2bins__2020_08_13_2020_08_31').delayObjects; % this one only has To as independent variable
    tclParameters.tempDelayObjects = delayObjects;
    % load from house struct to make the following commands more clear
    house_ind = tclParameters.house_ind;
    % assign each TCL to a delay distribution
    if variability == 0 % homegenuous case
        tclParameters.tempDelayDistribution = ones(numAcs,1);
    else
        % match the temperature delays with the real houses
        tclParameters.tempDelayDistribution = house_ind;
    end
else
    switch temperatureDelayOption
        case 1 % zero delays
            delay_off = 0;
            delay_on = 0;
        case 2 % low constant delays
            delay_off = 50;
            delay_on = 150;   
        case 3 % high constant delays
            delay_off = 50*2;
            delay_on = 150*3;
    end
    % Set the time needed for the temperature to change direction after a
    % switching occurs
    tclParameters.delayTimeOffInTimeSteps = delay_off/timeStepInSec;
    tclParameters.delayTimeOnInTimeSteps = delay_on/timeStepInSec;
    % add variability
    tclParameters.delayTimeOffInTimeSteps = tclParameters.delayTimeOffInTimeSteps * (1+(rand(numAcs,1)-.5).* variability);
    tclParameters.delayTimeOnInTimeSteps = tclParameters.delayTimeOnInTimeSteps * (1+(rand(numAcs,1)-.5).* variability);    
end
tclParameters.timeUntilDelayEnds = 0 .* ones(numAcs,1);

% construct fields to keep the delays that will be generated
tclParameters.delays_off_generated = []; 
tclParameters.delays_on_generated = [];

end

function tclParameters = generateTemperatureDelayValues_2zone(tclParameters, numAcs, variability, timeStepInSec, temperatureDelayOption)
% Create the distributions for the temperature delays

tclParameters.temperatureDelayOption = temperatureDelayOption;
if temperatureDelayOption == 4
    delayObjects_1zone = load('./dataFiles/delayObjects_1thermostat_onlyTo2bins__2020_08_13_2020_08_31').delayObjects; % this one only has To as independent variable
    delayObjects_2zone = load('./dataFiles/delayObjects_2thermostats_onlyTo2bins_2020_08_13__2020_08_31_no_outliers.mat').delayObjects; % this one only has To as independent variable    
    tclParameters.tempDelayObjects_1zone = delayObjects_1zone;
    tclParameters.tempDelayObjects_2zone = delayObjects_2zone;
    tclParameters.tempDelayObjects = [delayObjects_2zone delayObjects_1zone];    % load from house struct to make the following commands more clear
    % load from house struct to make the following commands more clear
    house_ind = tclParameters.house_ind;

    if variability == 0 % homegenuous case
        tclParameters.tempDelayDistribution = ones(numAcs,1);
    else
        % match the temperature delays with the real houses
        tclParameters.tempDelayDistribution = house_ind;
    end
else
    switch temperatureDelayOption
        case 1 % zero delays
            delay_off = 0;
            delay_on = 0;
        case 2 % low constant delays
            delay_off = 50;
            delay_on = 150;   
        case 3 % high constant delays
            delay_off = 50*2;
            delay_on = 150*3;
    end
    % Set the time needed for the temperature to change direction after a
    % switching occurs
    tclParameters.delayTimeOffInTimeSteps = delay_off/timeStepInSec;
    tclParameters.delayTimeOnInTimeSteps = delay_on/timeStepInSec;
    % add variability
    tclParameters.delayTimeOffInTimeSteps = tclParameters.delayTimeOffInTimeSteps * (1+(rand(numAcs,1)-.5).* variability);
    tclParameters.delayTimeOnInTimeSteps = tclParameters.delayTimeOnInTimeSteps * (1+(rand(numAcs,1)-.5).* variability);    
end
tclParameters.timeUntilDelayEnds = 0 .* ones(numAcs,1);

% construct fields to keep the delays that will be generated
tclParameters.delays_off_generated = []; 
tclParameters.delays_on_generated = [];

end