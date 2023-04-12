% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function tclParameters = generateDynamicMatrices_1zone(tclParameters, numAcs, timeStepInHr)
% load the specified fractions of the heat gains that go to the mass
fi = tclParameters.mass_internal_gain_fraction;
fs = tclParameters.mass_solar_gain_fraction;
% make cell arrays from the individual TCL arrays for each of the thermal
% parameters
U_a_cell = mat2cell(tclParameters.U_a,ones(numAcs,1));
H_m_cell = mat2cell(tclParameters.H_m,ones(numAcs,1));
C_a_cell = mat2cell(tclParameters.C_a,ones(numAcs,1));
C_m_cell = mat2cell(tclParameters.C_m,ones(numAcs,1));

% set up the continuous dynamics using cell functions
% d[T_air T_mass]' = A * [T_air T_mass]' + B * Q_h * m + H * [T_a Q_i Q_s Q_m]

% set up the continuous time state matrices
A_continuous_dynamics = cellfun(@(a,b,c,d) [-(a + b)/c b/c; b/d -b/d], U_a_cell, H_m_cell, C_a_cell, C_m_cell,'UniformOutput', 0);

% continuous B is set up so that the input vector is [Q_h*m T_a Q_i Q_s Q_m]'
% We will split it into the actual B matrix and the H matrix below
B_continuous_dynamics = cellfun(@(a,b,c) [ 1/b a/b (1-fi)/b (1-fs)/b 0; 0 0 0 0 1/c], U_a_cell, C_a_cell, C_m_cell,  'UniformOutput', 0);

% discretize the above dynamics

% turn the timestep into an approriately sized cell matrix
delta_t = mat2cell(ones(numAcs,1) * timeStepInHr, ones(numAcs,1));

% create the discretized A matrix for each TCL
A_discrete_dynamics_cell = cellfun(@(a, b) expm(a .* b), A_continuous_dynamics, delta_t, 'UniformOutput', 0);

% create discretized B matrix
discretization_factor = cellfun(@(a, b) a^-1 * (b- eye(2)), A_continuous_dynamics, A_discrete_dynamics_cell , 'UniformOutput', 0);
B_discrete_dynamics_combined = cellfun(@(a, b) a * b, discretization_factor, B_continuous_dynamics, 'UniformOutput', 0);

% split the discretized B matrix so that B_d * Q_h *m + H_d * [T_a Q_i Q_s Q_m]
H_discrete_dynamics_cell = cellfun(@(a) a(:,2:end) , B_discrete_dynamics_combined, 'UniformOutput', 0);
B_discrete_dynamics_cell = cellfun(@(a) a(:,1) , B_discrete_dynamics_combined, 'UniformOutput', 0);

% convert the discretized cell arrays into three dimensional arrays to
% vectorize the state update equation for each TCL. 

% H is the disturbance input matrix
H_discrete_dynamics = cat(1,H_discrete_dynamics_cell{:});
% B is the input matrix (for the discrete- 0/1 - state)
B_discrete_dynamics = cat(1,B_discrete_dynamics_cell{:});
% state-update equation
A_discrete_dynamics = cat(1,A_discrete_dynamics_cell{:});

% create the tclParameters stucture that contains the values that we
% generated within this function. 
tclParameters.broadcastInput = []; % sets a field that will be used to store inputs
tclParameters.A_discrete_dynamics = A_discrete_dynamics; % A-matrix for individual 3-State TCL (state are temperatures)
tclParameters.B_discrete_dynamics = B_discrete_dynamics; % B matrix for individual 3-State TCL (input is on/off value)
tclParameters.H_discrete_dynamics = H_discrete_dynamics;  % disturbance input matrix for individual 3-State TCL 
end