% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [transitionMatrix] = fill_A_matrix_heuristic_2zone(transitionMatrix, controller_type, binNumber)
%{
Some states may have not been accessed during the A matrix identification
proccess. This results in some entries of A being zero, and its columns not
summing to 1. This function fills out these entries using pre-specified
probabilities.
Input:
    transitionMatrix: matrix
        Markov transition matrix. Some columns do not sum to 1 as they
        should.
    controller_type: str
        Type of the controller being used.
    binNumber: int
        Number of bins that the normalized deadband [0,1] is discretized
        into.
Output:
    transitionMatrix: matrix
        Markov matrix with every column summing to 1.
%}

%Column sum
col_sum_normal = sum(transitionMatrix,1)';
% Address zero entries in the transitionMatrix using heuristics
if strcmp(controller_type, 'Markov controller 2 zone with lockouts') %This is just a heuristic method based on guesses
    %Append the nonzero column on the OFF unlocked (0,0) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 1:binNumber*binNumber
        if col_sum_normal(ij) == 0
            transitionMatrix(ij,ij) = 0.85; %Heuristic
            transitionMatrix(ij+1,ij) = 0.05;
            transitionMatrix(ij+binNumber,ij) = 0.05;
            transitionMatrix(ij+binNumber+1,ij) = 0.05;
        end
    end
    %Manage zeros on the (0,1) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 1*binNumber*binNumber+1:2*binNumber*binNumber 
        if col_sum_normal(ij) == 0 
            if false %sum(transitionMatrix_nolck(1:end,ij)) > 0 %using the unlocked MTM
                transitionMatrix(1:4*binNumber*binNumber,ij) = transitionMatrix_nolck(1:end,ij);
            else
                transitionMatrix(ij,ij) = 0.90; %Heuristic
                transitionMatrix(ij-1,ij) = 0.025;
                transitionMatrix(ij+binNumber,ij) = 0.025;
                transitionMatrix(ij+binNumber-1,ij) = 0.025;
                transitionMatrix(ij+3*binNumber*binNumber,ij) = 0.025; %to (0,0)L
            end
        end
    end
    %Manage zeros on the (1,0) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 2*binNumber*binNumber+1:3*binNumber*binNumber 
        if col_sum_normal(ij) == 0 
            if false %sum(transitionMatrix_nolck(1:end,ij)) > 0 %using the unlocked MTM
                transitionMatrix(1:4*binNumber*binNumber,ij) = transitionMatrix_nolck(1:end,ij);
            else
                transitionMatrix(ij,ij) = 0.90; %Heuristic
                transitionMatrix(ij-binNumber,ij) = 0.025;
                transitionMatrix(ij-binNumber+1,ij) = 0.025;
                transitionMatrix(ij+1,ij) = 0.025;
                transitionMatrix(ij+2*binNumber*binNumber,ij) = 0.025; %to (0,0)L
            end
        end
    end
    %Manage zeros on the (1,1) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 3*binNumber*binNumber+1:4*binNumber*binNumber 
        if col_sum_normal(ij) == 0 
            if false %sum(transitionMatrix_nolck(1:end,ij)) > 0 %using the unlocked MTM
                transitionMatrix(1:4*binNumber*binNumber,ij) = transitionMatrix_nolck(1:end,ij);
            else
                transitionMatrix(ij,ij) = 0.90;
                transitionMatrix(ij-binNumber,ij) = 0.025;
                transitionMatrix(ij-1,ij) = 0.025;
                transitionMatrix(ij-binNumber-1,ij) = 0.025;
                transitionMatrix(ij+1*binNumber*binNumber,ij) = 0.025; %to (0,0)L
            end
        end
    end
    %Manage zeros on the (0,0)L entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 4*binNumber*binNumber+1:5*binNumber*binNumber
        if col_sum_normal(ij) == 0
            transitionMatrix(ij,ij) = 0.85;
            transitionMatrix(ij+1,ij) = 0.05; %Fractions that go to higher temperature bins
            transitionMatrix(ij-4*binNumber*binNumber,ij) = 0.1; %Fractions that go to unlocked (0,0) bins
        end
    end
    %Manage zeros on the (0,1)L entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 5*binNumber*binNumber+1:6*binNumber*binNumber
        if col_sum_normal(ij) == 0
            transitionMatrix(ij,ij) = 0.90;
            transitionMatrix(ij-1,ij) = 0.025; %Fractions that go to other locked (0,1) bins
            transitionMatrix(ij+binNumber,ij) = 0.025; %Fractions that go to other locked (0,1) bins
            transitionMatrix(ij+binNumber-1,ij) = 0.025; %Fractions that go to other locked (0,1) bins
            transitionMatrix(ij-4*binNumber*binNumber,ij) = 0.025; %Fractions that go to unlocked (0,1) bins
        end
    end
    %Manage zeros on the (1,0)L entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 6*binNumber*binNumber+1:7*binNumber*binNumber
        if col_sum_normal(ij) == 0
            transitionMatrix(ij,ij) = 0.90;
            transitionMatrix(ij-binNumber+1,ij) = 0.025; %Fractions that go to other locked (1,0) bins
            transitionMatrix(ij+1,ij) = 0.025; %Fractions that go to other locked (1,0) bins
            transitionMatrix(ij-binNumber,ij) = 0.025; %Fractions that go to other locked (1,0) bins
            transitionMatrix(ij-4*binNumber*binNumber,ij) = 0.025; %Fractions that go to unlocked (1,0) bins
        end
    end
    %Manage zeros on the (1,1)L entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 7*binNumber*binNumber+1:8*binNumber*binNumber
        if col_sum_normal(ij) == 0
            transitionMatrix(ij,ij) = 0.90;
            transitionMatrix(ij-binNumber,ij) = 0.025; %Fractions that go to other locked (1,1) bins
            transitionMatrix(ij-1,ij) = 0.025; %Fractions that go to other locked (1,1) bins
            transitionMatrix(ij-binNumber-1,ij) = 0.025; %Fractions that go to other locked (1,1) bins
            transitionMatrix(ij-4*binNumber*binNumber,ij) = 0.025; %Fractions that go to unlocked (1,1) bins
        end
    end
    %Sanity check
    %Column sum
    col_sum_normal_new = sum(transitionMatrix,1)';
    %Warning of there is one zero column
    if (nnz(~col_sum_normal_new) == 0) && (sum(col_sum_normal_new) == 8*binNumber*binNumber)
        disp("Succeed: no zero column found");
    else
        disp("Failed: at least one zero column");
    end
else
    %Append the nonzero column on the OFF unlocked (0,0) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 1:binNumber*binNumber
        if col_sum_normal(ij) == 0
            transitionMatrix(ij,ij) = 0.85; %Heuristic
            transitionMatrix(ij+1,ij) = 0.05;
            transitionMatrix(ij+binNumber,ij) = 0.05;
            transitionMatrix(ij+binNumber+1,ij) = 0.05;
        end
    end
    %Manage zeros on the (0,1) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 1*binNumber*binNumber+1:2*binNumber*binNumber 
        if col_sum_normal(ij) == 0 
            if false %sum(transitionMatrix_nolck(1:end,ij)) > 0 %using the unlocked MTM
                transitionMatrix(1:4*binNumber*binNumber,ij) = transitionMatrix_nolck(1:end,ij);
            else
                transitionMatrix(ij,ij) = 0.90; %Heuristic
                transitionMatrix(ij-1,ij) = 0.025;
                transitionMatrix(ij+binNumber,ij) = 0.025;
                transitionMatrix(ij+binNumber-1,ij) = 0.025;
                transitionMatrix(ij-1*binNumber*binNumber,ij) = 0.025; %to (0,0)
            end
        end
    end
    %Manage zeros on the (1,0) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 2*binNumber*binNumber+1:3*binNumber*binNumber 
        if col_sum_normal(ij) == 0 
            if false %sum(transitionMatrix_nolck(1:end,ij)) > 0 %using the unlocked MTM
                transitionMatrix(1:4*binNumber*binNumber,ij) = transitionMatrix_nolck(1:end,ij);
            else
                transitionMatrix(ij,ij) = 0.90; %Heuristic
                transitionMatrix(ij-binNumber,ij) = 0.025;
                transitionMatrix(ij-binNumber+1,ij) = 0.025;
                transitionMatrix(ij+1,ij) = 0.025;
                transitionMatrix(ij-2*binNumber*binNumber,ij) = 0.025; %to (0,0)
            end
        end
    end
    %Manage zeros on the (1,1) entries of the transition matrix
    disp("Fixing the nonzero columns");
    for ij = 3*binNumber*binNumber+1:4*binNumber*binNumber 
        if col_sum_normal(ij) == 0 
            if false %sum(transitionMatrix_nolck(1:end,ij)) > 0 %using the unlocked MTM
                transitionMatrix(1:4*binNumber*binNumber,ij) = transitionMatrix_nolck(1:end,ij);
            else
                transitionMatrix(ij,ij) = 0.90;
                transitionMatrix(ij-binNumber,ij) = 0.025;
                transitionMatrix(ij-1,ij) = 0.025;
                transitionMatrix(ij-binNumber-1,ij) = 0.025;
                transitionMatrix(ij-3*binNumber*binNumber,ij) = 0.025; %to (0,0)
            end
        end
    end

    %Sanity check
    %Column sum
    col_sum_normal_new = sum(transitionMatrix,1)';
    %Warning of there is one zero column
    if (nnz(~col_sum_normal_new) == 0) && (sum(col_sum_normal_new) == 4*binNumber*binNumber)
        disp("Succeed: no zero column found");
    else
        disp("Failed: at least one zero column");
    end
end
end

