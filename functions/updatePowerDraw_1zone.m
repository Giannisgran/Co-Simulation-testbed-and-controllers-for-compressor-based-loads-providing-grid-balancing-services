function [tclParameters] = updatePowerDraw_1zone(tclParameters, switchedOnIndicator, isOn)
%{
Updates the real and reactice power draws for all TCLs. 
Assumes that only single-zone TCLs exist in the population
---
Input:
    tclParameters: struct
        Contains all the info for the TCL population.
    switchedOnIndicator: vector of bools
        For each TCL, equals 1 if the compressor just switched ON.
    isOn: vector of bools
        For each TCL, equals 1 if the compressor is ON
Output:
    tclParameters: struct
        Updated real and reactive power draws.
%}
P_power_draw = calculatePversusTV(tclParameters, switchedOnIndicator, isOn);
Q_power_draw = calculateQversusTV(tclParameters, switchedOnIndicator, isOn);

tclParameters.P_power_draw = P_power_draw;
tclParameters.Q_power_draw = Q_power_draw;

end

function P_power_draw = calculatePversusTV(tclParameters, switchedOnIndicator, isOn)

K_0 = tclParameters.K_p_0;
K_temp = tclParameters.K_p_temp;
K_voltage = tclParameters.K_p_voltage;
T_a = tclParameters.T_a.*1.8+32; % convert to F
voltage = tclParameters.voltage240;

% compute inrush
inrushMultiplier = ones(numel(switchedOnIndicator),1); % initialize to 1
if any(switchedOnIndicator) && tclParameters.inrushOption
    tclIndices = find(switchedOnIndicator==1);
    for row = 1:numel(tclIndices)
        tclIndex = tclIndices(row);
 
        % get distribution, values, and a random draw from 0 to 1
        inrushDistributionP = tclParameters.inrushDistributionP{tclIndex}(:,1);
        inrushValuesP = tclParameters.inrushDistributionP{tclIndex}(:,2);
        rand_vals = rand(1,1);  %spans zero to one
        
        % get the actual value from the distribution for the random draw
        out_val = interp1(inrushDistributionP,[0:1/(length(inrushDistributionP)-1):1],rand_vals); %spans zero to one
        ind = ceil(out_val*(length(inrushValuesP)-1));
        
        inrushMultiplier(tclIndex) = 1+inrushValuesP(ind)/100;
    end
end

P_power_draw = (K_0(:) + K_temp(:).*T_a + K_voltage(:).*voltage).*inrushMultiplier;


end

function Q_power_draw = calculateQversusTV(tclParameters, switchedOnIndicator, isOn)

K_0 = tclParameters.K_q_0;
K_temp = tclParameters.K_q_temp;
K_voltage = tclParameters.K_q_voltage;
T_a = tclParameters.T_a.*1.8+32; % convert to F
voltage = tclParameters.voltage240;

% compute inrush 
inrushMultiplier = ones(numel(switchedOnIndicator),1);
if any(switchedOnIndicator) && tclParameters.inrushOption
    tclIndices = find(switchedOnIndicator==1);
    for row = 1:numel(tclIndices)
        tclIndex = tclIndices(row);

        % get distribution, values, and a random draw from 0 to 1
        inrushDistributionQ = tclParameters.inrushDistributionQ{tclIndex}(:,1);
        inrushValuesQ = tclParameters.inrushDistributionQ{tclIndex}(:,2);
        rand_vals = rand(1,1);  %spans zero to one
        
        % get the actual value from the distribution for the random draw
        out_val = interp1(inrushDistributionQ,[0:1/(length(inrushDistributionQ)-1):1],rand_vals); %spans zero to one
        ind = ceil(out_val*(length(inrushValuesQ)-1));
        
        inrushMultiplier(tclIndex) = 1+inrushValuesQ(ind)/100;
    end
end 

Q_power_draw = (K_0(:) + K_temp(:).*T_a + K_voltage(:).*voltage).*inrushMultiplier;

end