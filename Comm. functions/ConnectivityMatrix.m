% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [CM] = ConnectivityMatrix(n,nc,IM)

if nc == 0 %centralized topology
    rng(105); %To ensure, the same CM is generated given the same IM
    CM = randi([1 3],n+1,n+1); %Connection Matrix (CM)
else %tree topology
    rng(105); %To ensure, the same CM is generated given the same IM
    CM = randi([1 3],n+nc+1,n+nc+1); %Connection Matrix (CM)
end

%No nodal connection,therefore no connection matrix representation
if nc == 0 %centralized topology
    for i =1:n+1
        for k = 1:n+1
            if IM(i,k) == 0
                CM(i,k) = 0;
            end
        end
    end
else %tree topology
    for i =1:n+nc+1
        for k = 1:n+nc+1
            if IM(i,k) == 0
                CM(i,k) = 0;
            end
        end
    end
end
end