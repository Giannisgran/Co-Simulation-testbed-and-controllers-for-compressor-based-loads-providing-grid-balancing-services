% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [CM,DDM,MDM, SDM, distribution, IM] = CommNetworkCharacteristics(pop, nc, numSteps,scenario, timeStepInSec)

IM = IncidenceMatrix(pop,nc); %Generates nodal connection matrix using the 
%IncidenceMatrix function

CM = ConnectivityMatrix(pop,nc,IM); %Generates connectivity matrix 

[DDM] = DropoutModel(IM,CM, numSteps,scenario); %Obtain Dropout Distribution Matrix 
%(DDM), Dropout Distribution Realization(DDR) and Dropout Effect (DE) 
%from DropoutModel function

[MDM, SDM, distribution] = DelayModel(CM, numSteps,scenario, timeStepInSec); %Obtain Delay Matrix from DelayModel function


end