% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [IM] = IncidenceMatrix(n,nc)

% IM for Centralized topology
if nc == 0
    IM = zeros(n+1,n+1); %n+1 = n TCLs/nodes + central controller
    for i = 1:n+1
        IM(1,i) = 1; %connection between CC and TCL/nodes
        IM(i,1) = 1; %connection between CC and TCL/nodes
        for j = 1:n+1
            if i == j
                IM(i,j) = 1; %Diagonals, assuming self conection = computation time
            end
        end
    end
end
%IM for Tree topology
if nc ~= 0
    
    IM = zeros(n+nc+1,n+nc+1); %n+nc+1 = TCLs + collection nodes + central controller
    
    for i = 1:nc+1
    IM(1,i) = 1; %connection between CC and collection nodes
    IM(i,1) = 1; %connection between CC and collection nodes
        for j = 1:n+nc+1 
            for k = 1:n+nc+1
                if j == k
                IM(j,k) = 1; %Diagonals, assuming self conection = computation time
                end
            end
            p = round(n/nc); %number of TCLs/sub-nodes per collection nodes
            o = nc+2:1:n+nc+1; %set of sub-nodes/TCL indices
            
                for q = 1 : floor(n/p)  
                    s = rem(n,p); %extra sub-nodes after evenly distributing sub-nodes amongst collection nodes
                    r = (p*(q-1))+1 :1: p*q; %set of sub-node indices corresponding to each collection node                                                      
                            IM(q+1,o(1,r)) = 1; %connection between collection node and sub node
                            IM(o(1,r),q+1) = 1; %connection between collection node and sub node                                    
                end
                if s ~=0 
                    r =  p*floor(n/p)+1 :1: p*floor(n/p)+s; %set of sub-node indices for 'extra' sub-nodes
                    IM(nc+1,o(1,r)) = 1; %assigning connection between 'extra' sub nodes and last collection node
                    IM(o(1,r),nc+1) = 1; %assigning connection between 'extra' sub nodes and last collection node
                end
        end
    end
    
end
end