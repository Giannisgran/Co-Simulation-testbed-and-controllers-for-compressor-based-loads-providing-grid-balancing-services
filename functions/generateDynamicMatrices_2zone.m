% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function tclParameters = generateDynamicMatrices_2zone(tclParameters, timeStepInHr)
%{
This function generates the matrices that will be used during the
computation of the air and mass temperatures in a vectorized fashion.
The idea is the following:
    Whether something is 2-zone house or a 1-zone house, we
    treat it the same, as a block. For a zone in the 2-zone case, this block 
    has 4 entries, air temp and mass temp of current zone and then air temp
    and mass temp of the other zone. For 1-zone we just copy the values.
    The entries that correspond to the second zone are computed from the
    block of the 1st zone. 
Input:
    tclParameters: struct
        Includes info about the TCLs like thermal parameters etc.
    timeStepInHr: float
        Simulation timestep in hours.
Output:
    tclParameters: struct
        Updated with the matrices to be used for vectorized state updates.
%}

%% Load variables from structs
% number of houses
num_houses = length(tclParameters.tcls_2zone_first) + length(tclParameters.tcls_1zone);
% load the specified fractions of the heat gains that go to the mass
fi = tclParameters.mass_internal_gain_fraction;
fs = tclParameters.mass_solar_gain_fraction;
% get the indices corresponding to 1st, 2nd zones and 1-zone TCLs
tcls_2zone_first = tclParameters.tcls_2zone_first;
tcls_2zone_second = tclParameters.tcls_2zone_second;
tcls_1zone = tclParameters.tcls_1zone;
%% Get thermal parameters for each zone
% make cell arrays from the individual TCL arrays for each of the thermal
% parameters
U_a_cell = mat2cell(tclParameters.U_a([tcls_2zone_first tcls_1zone]),ones(num_houses,1));
H_m_cell = mat2cell(tclParameters.H_m([tcls_2zone_first tcls_1zone]),ones(num_houses,1));
C_a_cell = mat2cell(tclParameters.C_a([tcls_2zone_first tcls_1zone]),ones(num_houses,1));
C_m_cell = mat2cell(tclParameters.C_m([tcls_2zone_first tcls_1zone]),ones(num_houses,1));
H_z_cell = mat2cell(tclParameters.H_z([tcls_2zone_first tcls_1zone]),ones(num_houses,1));

% create arrays for the parameters corresponding to the second zones, for
% single-zone houses, they will be multiplied by zero temperatures later
U_a_other_cell = mat2cell(tclParameters.U_a([tcls_2zone_second tcls_1zone]),ones(num_houses,1));
H_m_other_cell = mat2cell(tclParameters.H_m([tcls_2zone_second tcls_1zone]),ones(num_houses,1));
C_a_other_cell = mat2cell(tclParameters.C_a([tcls_2zone_second tcls_1zone]),ones(num_houses,1));
C_m_other_cell = mat2cell(tclParameters.C_m([tcls_2zone_second tcls_1zone]),ones(num_houses,1));

%% Construct continuous matrices
% set up the continuous dynamics using cell functions
% d[T_air T_mass T_air_other_zone T_mass_other_zone]' = A * [T_air T_mass T_air_other_zone T_mass_other_zone]' + B * Q_h * m + H * [T_a Q_i Q_s Q_m]
A_continuous_dynamics = cellfun(@(a,b,c,d,e,f,g,h,i) [-(a + b + e)/c b/c e/c 0; b/d -b/d 0 0; e/h 0 -(f + g + e)/h g/h;0 0 g/i -g/i], ...
                                 U_a_cell, H_m_cell, C_a_cell, C_m_cell, H_z_cell, U_a_other_cell, H_m_other_cell, C_a_other_cell, C_m_other_cell,'UniformOutput', 0);
% continuous B is set up so that the input vector is [Q_h*m Q_h_2*m_2 T_a Q_i Q_s Q_m Q_i_other Q_s_other Q_m_other]'
% We will split it into the actual B matrix and the H matrix below
B_continuous_dynamics = cellfun(@(a,b,c,d,e,f) [ 1/b 0 a/b 1/b*(1-fi) 1/b*(1-fs) 0 0 0 0; 0 0 0 0 0 1/c 0 0 0; 0 1/e d/e 0 0 0 1/e*(1-fi) 1/e*(1-fs) 0; 0 0 0 0 0 0 0 0 1/f],...
                                 U_a_cell, C_a_cell, C_m_cell, U_a_other_cell, C_a_other_cell, C_m_other_cell,  'UniformOutput', 0);

%% Construct discretized matrices
% turn the timestep into an approriately sized cell matrix
delta_t = mat2cell(ones(num_houses,1) * timeStepInHr, ones(num_houses,1));
% create the discretized A matrix for each TCL
A_discrete_dynamics_cell = cellfun(@(a, b) expm(a .* b), A_continuous_dynamics, delta_t, 'UniformOutput', 0);
% ccompute discretization factor used to compute B
discretization_factor = cellfun(@(a, b) a^-1 * (b-eye(4)), A_continuous_dynamics, A_discrete_dynamics_cell , 'UniformOutput', 0);
% create discretized B matrix
B_discrete_dynamics_combined = cellfun(@(a, b) a * b, discretization_factor, B_continuous_dynamics, 'UniformOutput', 0);
% split the discretized B matrix so that B_d * Q_h *m + H_d * [T_a Q_i Q_s Q_m]
H_discrete_dynamics_cell = cellfun(@(a) a(:,3:end) , B_discrete_dynamics_combined, 'UniformOutput', 0);
B_discrete_dynamics_cell = cellfun(@(a) a(:,1:2) , B_discrete_dynamics_combined, 'UniformOutput', 0);

%% Reforming cell arrays to 3-dimensional arrays for vectorization purposes
% H is the disturbance input matrix
H_discrete_dynamics = cat(1,H_discrete_dynamics_cell{:});
% B is the input matrix (for the discrete- 0/1 - state)
B_discrete_dynamics = cat(1,B_discrete_dynamics_cell{:});
% state-update equation
A_discrete_dynamics = cat(1,A_discrete_dynamics_cell{:});

%% Store generated matrices to the struct 
tclParameters.broadcastInput = []; % sets a field that will be used to store inputs
tclParameters.A_discrete_dynamics = A_discrete_dynamics; % A-matrix for individual 3-State TCL (state are temperatures)
tclParameters.B_discrete_dynamics = B_discrete_dynamics; % B matrix for individual 3-State TCL (input is on/off value)
tclParameters.H_discrete_dynamics = H_discrete_dynamics;  % disturbance input matrix for individual 3-State TCL 

end