% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function[onRequests, offRequests, controllerInformation] = Extended_PEM(airTemperatureVector,T_min,T_max,MTTR,MTTRoff,dt,m,locked,tclIndices,n,controllerInformation)
% This function implements the extended PEM logic to generate turn-on and
% turn-off requests
% Inputs: airTemperature, Tmin, Tmax, MTTR, dt(timestep duration),ON/OFF
%         mode, lock-out mode, tclIndices,timestep
% Outputs:onRequests, offRequests, controllerInformation (storing
%         TCL request probabilities and other relevant variables)

%Identifying TCLs available to make PEM probability requests
m = logical(m);
offUnlockedIndices = find(m==0 & locked == 0);
onUnlockedIndices = find(m==1 & locked == 0);
%OnProbability - if tcl is off and unlocked:
controllerInformation.mu(offUnlockedIndices,n) = ((airTemperatureVector(offUnlockedIndices) - T_min(offUnlockedIndices))./(T_max(offUnlockedIndices) - airTemperatureVector(offUnlockedIndices))) .* (1/MTTR);
controllerInformation.onOffProbability(offUnlockedIndices,n) = 1 - exp(-controllerInformation.mu(offUnlockedIndices,n) .* dt);    
%OffProbability - if tcl is on and unlocked: 
controllerInformation.muofft(onUnlockedIndices,n) = ((T_max(onUnlockedIndices) - airTemperatureVector(onUnlockedIndices))./(airTemperatureVector(onUnlockedIndices) - T_min(onUnlockedIndices))) .* (1/MTTRoff);
controllerInformation.OffProbability(onUnlockedIndices,n) = 1 - exp(-controllerInformation.muofft(onUnlockedIndices,n) .* dt);

% do the random draw
onOffProbability = controllerInformation.onOffProbability(:,n);
onRequests = binornd(1,onOffProbability);
if isfield(controllerInformation,'OffProbability')
    [rownum, colnum] = size(controllerInformation.OffProbability);
    if colnum == n
        OffProbability = controllerInformation.OffProbability(:,n);
    else
        OffProbability = [];
    end
    if ~isempty(OffProbability)
        offRequests = binornd(1,OffProbability);
        if rownum < length(tclIndices)
            controlActions.OffSignal(tclIndices(1:rownum),1) = offRequests;
            controlActions.OffSignal(tclIndices(rownum+1:length(tclIndices)),1) = 0;
        else
            controlActions.OffSignal(tclIndices,1) = offRequests;
        end
    end
    %storage for analysis
    controllerInformation.offRequests(:,n) = controlActions.OffSignal;
    
end

% get the TCLs whose requests were accepted
onIndices = find(onRequests==1);
TCLRequest = tclIndices(onIndices); % i is tcl Number

if exist('offRequests','var')
    offIndices = find(offRequests==1);
    TCLOffRequest=tclIndices(offIndices);
    %store TCLs requesting to turn off
    if ~isempty(TCLOffRequest)
        iSub = 1 : length(TCLOffRequest);
        controllerInformation.TCLOffRequest(iSub,n) = TCLOffRequest; %Storing IDs of TCL which are within deadband, not locked and had successful turn-off bernoulli draws at each timestep
    else
        controllerInformation.TCLOffRequest(1,n) = 0;
    end
end

% store TCLs that requested power
if ~isempty(TCLRequest)
    iSub = 1 : length(TCLRequest);
    controllerInformation.TCLRequest(iSub,n) = TCLRequest; %Storing IDs of TCL which are within deadband, not locked and had successful bernoulli draws at each timestep
else
    controllerInformation.TCLRequest(1,n) = 0;
end

% get the final approved set of requests
[rownum2, colnum2] = size(onRequests);
if rownum2 < length(tclIndices)
    onRequests(tclIndices(1:rownum2),1)=onRequests;
    onRequests(tclIndices(rownum2+1:length(tclIndices)),1) = 0;
end

if rownum < length(tclIndices)
    offRequests(tclIndices(1:rownum),1)=offRequests;
    offRequests(tclIndices(rownum+1:length(tclIndices)),1) = 0;
end

%controlActions.onOffSignal(tclIndices) = onRequests;
% store for analysis
controllerInformation.onRequests(:,n) = onRequests;