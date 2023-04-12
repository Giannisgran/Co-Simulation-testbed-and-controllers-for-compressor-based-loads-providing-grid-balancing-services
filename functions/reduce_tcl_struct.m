% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [tclParameters] = reduce_tcl_struct(tclParameters, inds, option)
%{
Reduces the tclParameters struct in order to correspond to the specified
indices only. Initally the struct corresponds to a mixed house population.
The final struct will only consist of single or 2-zone TCLs.
Input:
    tclParameters: struct
        Info for all TCLs that we want to reduce.
    inds: vector of ints
        Indices of the TCLs that we are interested in and want to keep.
    option: int
        If 1, then the reduced population will be consisting only of
        single-zone house. If equal to 2, only by 2-zone houses.
Output:
    tclParameters: struct
        Reduced struct.
%}

if option == 1
    numHouses_2zone = tclParameters.numHouses_2zone;  % keep the actual number of 2-zone houses
    tclParameters.perc_2zone = 0;  % we only consider the single-zone houses here
    tclParameters.numHouses_2zone = 0;
    tclParameters.numHouses_1zone = length(inds); % may have removed some houses because they correspond to real devices
    tclParameters.numAcs = length(inds);
    tclParameters.tcls_2zone_first = [];
    tclParameters.tcls_2zone_second = [];
    tclParameters.tcls_1zone = 1:length(inds);
    tclParameters.is2zone = tclParameters.is2zone(inds);
    tclParameters.house_ind = tclParameters.house_ind(inds);
    tclParameters.tcl_house_mapping = tclParameters.tcl_house_mapping(inds);
    tclParameters.lockoutTimeOffInTimeSteps = tclParameters.lockoutTimeOffInTimeSteps(inds);
    tclParameters.lockoutTimeOnInTimeSteps = tclParameters.lockoutTimeOnInTimeSteps(inds);
    tclParameters.irradiance = tclParameters.irradiance(inds);    
    tclParameters.m0 = tclParameters.m0(inds);
    tclParameters.theta_a0 = tclParameters.theta_a0(inds);
    tclParameters.theta_m0 = tclParameters.theta_m0(inds);
    tclParameters.T_max = tclParameters.T_max(inds);
    tclParameters.T_min = tclParameters.T_min(inds);
    tclParameters.T_sp = tclParameters.T_sp(inds);
    tclParameters.deadband = tclParameters.deadband(inds);
    tclParameters.timeUntilUnlocked = tclParameters.timeUntilUnlocked(inds);
    tclParameters.timeUntilDelayEnds = tclParameters.timeUntilDelayEnds(inds);
    tclParameters.P_power_draw = tclParameters.P_power_draw(inds);
    tclParameters.Q_power_draw = tclParameters.Q_power_draw(inds);
    tclParameters.T_a = tclParameters.T_a(inds);
    tclParameters.voltage240 = tclParameters.voltage240(inds);
    tclParameters.K_p_0 = tclParameters.K_p_0(inds);
    tclParameters.K_p_temp = tclParameters.K_p_temp(inds);
    tclParameters.K_p_voltage = tclParameters.K_p_voltage(inds);
    tclParameters.K_q_0 = tclParameters.K_q_0(inds);
    tclParameters.K_q_temp = tclParameters.K_q_temp(inds);
    tclParameters.K_q_voltage = tclParameters.K_q_voltage(inds);  
    tclParameters.ratedReadPower = tclParameters.ratedReadPower(inds);
    tclParameters.inrushDistributionP = tclParameters.inrushDistributionP(inds);
    tclParameters.inrushDistributionQ = tclParameters.inrushDistributionQ(inds);
    tclParameters.U_a = tclParameters.U_a(inds);
    tclParameters.C_a = tclParameters.C_a(inds);
    tclParameters.H_m = tclParameters.H_m(inds);
    tclParameters.C_m = tclParameters.C_m(inds);
    tclParameters.Q_i = tclParameters.Q_i(inds);
    tclParameters.Q_s = tclParameters.Q_s(inds);
    tclParameters.Q_m = tclParameters.Q_m(inds);
    tclParameters.Q_h = tclParameters.Q_h(inds);
    tclParameters.H_z = tclParameters.H_z(inds);
    tclParameters.SHGC = tclParameters.SHGC(inds);
    tclParameters.P_trans = tclParameters.P_trans(inds);
    tclParameters.coolingCapacity = tclParameters.coolingCapacity(inds);
    tclParameters.designTotalCoolingBtu = tclParameters.designTotalCoolingBtu(inds);
    tclParameters.acNodeAssignment = tclParameters.acNodeAssignment(inds);
    tclParameters.delayTimeOffInTimeSteps = tclParameters.delayTimeOffInTimeSteps(inds);
    tclParameters.delayTimeOnInTimeSteps = tclParameters.delayTimeOnInTimeSteps(inds);
    tclParameters.justTurnedOn = tclParameters.justTurnedOn(inds);
    tclParameters.timestepsSinceLastSwitchOff = tclParameters.timestepsSinceLastSwitchOff(inds);
    tclParameters.timestepsSinceLastSwitchOn = tclParameters.timestepsSinceLastSwitchOn(inds);
    tclParameters.aggregatorAssignment = tclParameters.aggregatorAssignment(inds);
    % donwsample the dynamic matrices in order to pick the entries that
    % correspond to the single-zone TCLs
    % each house has 4 entries, and the first portion corresponds to 2-zone houses
    aux_var = [4*numHouses_2zone+1:4:4*(numHouses_2zone+tclParameters.numHouses_1zone)];  % starting index for each single-zone house
    aux_var_mat = [aux_var' aux_var'+1]; % each row has the 4 indices that correspond to each house, but for single-zone houses we only need 2
    inds_downsample = reshape(aux_var_mat', 1, []);  % pick the entries that correspond to the single-zone houses
    tclParameters.A_discrete_dynamics = tclParameters.A_discrete_dynamics(inds_downsample, 1:2);
    tclParameters.B_discrete_dynamics = tclParameters.B_discrete_dynamics(inds_downsample, 1);
    tclParameters.H_discrete_dynamics = tclParameters.H_discrete_dynamics(inds_downsample, 1:4);    

elseif option == 2
    tclParameters.perc_2zone = 1;  % we only consider the single-zone houses here
    tclParameters.numHouses_1zone = 0;
    tclParameters.numHouses_2zone = length(inds)/2; % may have removed some houses because they correspond to real devices
    tclParameters.numAcs = length(inds);
    tclParameters.tcls_1zone = [];
    tclParameters.tcls_2zone_first = 1:2:length(inds);
    tclParameters.tcls_2zone_second = 2:2:length(inds);
    tclParameters.is2zone = tclParameters.is2zone(inds);
    tclParameters.house_ind = tclParameters.house_ind(inds);
    tclParameters.tcl_house_mapping = tclParameters.tcl_house_mapping(inds);
    tclParameters.lockoutTimeOffInTimeSteps = tclParameters.lockoutTimeOffInTimeSteps(inds);
    tclParameters.lockoutTimeOnInTimeSteps = tclParameters.lockoutTimeOnInTimeSteps(inds);
    tclParameters.irradiance = tclParameters.irradiance(inds);    
    tclParameters.m0 = tclParameters.m0(inds);
    tclParameters.theta_a0 = tclParameters.theta_a0(inds);
    tclParameters.theta_m0 = tclParameters.theta_m0(inds);
    tclParameters.T_max = tclParameters.T_max(inds);
    tclParameters.T_min = tclParameters.T_min(inds);
    tclParameters.T_sp = tclParameters.T_sp(inds);
    tclParameters.deadband = tclParameters.deadband(inds);
    tclParameters.timeUntilUnlocked = tclParameters.timeUntilUnlocked(inds);
    tclParameters.timeUntilDelayEnds = tclParameters.timeUntilDelayEnds(inds);
    tclParameters.P_power_draw = tclParameters.P_power_draw(inds);
    tclParameters.Q_power_draw = tclParameters.Q_power_draw(inds);
    tclParameters.T_a = tclParameters.T_a(inds);
    tclParameters.voltage240 = tclParameters.voltage240(inds);
    tclParameters.K_p_0 = tclParameters.K_p_0(inds);
    tclParameters.K_p_temp = tclParameters.K_p_temp(inds);
    tclParameters.K_p_voltage = tclParameters.K_p_voltage(inds);
    tclParameters.K_q_0 = tclParameters.K_q_0(inds);
    tclParameters.K_q_temp = tclParameters.K_q_temp(inds);
    tclParameters.K_q_voltage = tclParameters.K_q_voltage(inds);  
    tclParameters.ratedReadPower = tclParameters.ratedReadPower(inds);
    tclParameters.inrushDistributionP = tclParameters.inrushDistributionP(inds);
    tclParameters.inrushDistributionQ = tclParameters.inrushDistributionQ(inds);
    tclParameters.U_a = tclParameters.U_a(inds);
    tclParameters.C_a = tclParameters.C_a(inds);
    tclParameters.H_m = tclParameters.H_m(inds);
    tclParameters.C_m = tclParameters.C_m(inds);
    tclParameters.Q_i = tclParameters.Q_i(inds);
    tclParameters.Q_s = tclParameters.Q_s(inds);
    tclParameters.Q_m = tclParameters.Q_m(inds);
    tclParameters.Q_h = tclParameters.Q_h(inds);
    tclParameters.H_z = tclParameters.H_z(inds);
    tclParameters.SHGC = tclParameters.SHGC(inds);
    tclParameters.P_trans = tclParameters.P_trans(inds);
    tclParameters.coolingCapacity = tclParameters.coolingCapacity(inds);
    tclParameters.designTotalCoolingBtu = tclParameters.designTotalCoolingBtu(inds);
    tclParameters.acNodeAssignment = tclParameters.acNodeAssignment(inds);
    tclParameters.delayTimeOffInTimeSteps = tclParameters.delayTimeOffInTimeSteps(inds);
    tclParameters.delayTimeOnInTimeSteps = tclParameters.delayTimeOnInTimeSteps(inds);
    tclParameters.justTurnedOn = tclParameters.justTurnedOn(inds);
    tclParameters.timestepsSinceLastSwitchOff = tclParameters.timestepsSinceLastSwitchOff(inds);
    tclParameters.timestepsSinceLastSwitchOn = tclParameters.timestepsSinceLastSwitchOn(inds);
    tclParameters.aggregatorAssignment = tclParameters.aggregatorAssignment(inds);
    % donwsample the dynamic matrices in order to pick the entries that
    % correspond to the single-zone TCLs
    % each house has 4 entries, and the first portion corresponds to 2-zone houses
    inds_downsample = [1:4*tclParameters.numHouses_2zone];
    tclParameters.A_discrete_dynamics = tclParameters.A_discrete_dynamics(inds_downsample, :);
    tclParameters.B_discrete_dynamics = tclParameters.B_discrete_dynamics(inds_downsample, :);
    tclParameters.H_discrete_dynamics = tclParameters.H_discrete_dynamics(inds_downsample, :);
else 
    error('Invalid option.')
end

