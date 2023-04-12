% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [violation_duration,violation_max] = analize_temp_violations(airTemp_mat, tclParameters, timestepInSec)
%{
This function aims to output some metrics to quantify the amount of
temperature deadband violations of the whole TCL population.
%}

% calculate normalized temperatures based on the corresponding deadbands
airTemp_normed_mat = (airTemp_mat-repmat(tclParameters.T_min,1,size(airTemp_mat,2))) ./ (repmat(tclParameters.T_max,1,size(airTemp_mat,2))-repmat(tclParameters.T_min,1,size(airTemp_mat,2)));
% locate temperature deadband violations, up for Tmax violation and down for Tmin
violations_up = airTemp_normed_mat > 1;
violations_down = airTemp_normed_mat < 0;
% duration that the air temperature violates the deadband
violation_up_duration = sum(violations_up(:))*timestepInSec; % sec
violation_down_duration = sum(violations_down(:))*timestepInSec; % sec
violation_duration = violation_up_duration + violation_down_duration; % sec
% max and min magnitudes of temperature deadband violations
violation_max = max([airTemp_normed_mat(violations_up)-1;abs(airTemp_normed_mat(violations_down))]);
% do some printing
disp(['Total duration of the violations:' num2str(violation_duration,'%d')])
disp(['Maximum deadband violation:' num2str(violation_max,'%.4f')])
end

