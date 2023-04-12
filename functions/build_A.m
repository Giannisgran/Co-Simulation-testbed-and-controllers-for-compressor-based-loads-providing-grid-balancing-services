% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [A] = build_A(u, N, Au)
%{
This function constructs the Ac matrix in x(t+1) = A*x(t) = Ac*Au*x(t),
where Ac is the matrix that depends on the control actions of the
aggregator and Au models the internal-natural transitions.
Input:
    u: vector
        Control signal from the aggregator. Length depends on controller.
    N: int
        Number of bins for discretizing [0,1]
    Au: matrix
        Matrix that models the natural transitions of TCLs.
Output:
    A: matrix
        Resulting A matrix.
%}

% identity matrix
I = eye(N);
% create A matrix corresponding to the control signal
Ac = [I-diag(u(1:N))     zeros(N)              zeros(N) zeros(N);
      zeros(N)           I-diag(u(N+1:2*N))    zeros(N) zeros(N);
      zeros(N)           fliplr(u(2*N:-1:N+1))    I     zeros(N);
      fliplr(u(N:-1:1))  zeros(N)              zeros(N)    I
      ];
% final A matrix
A = Ac*Au;
end

