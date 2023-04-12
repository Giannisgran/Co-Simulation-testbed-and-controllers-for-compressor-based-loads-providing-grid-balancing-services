% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [ passOrFail, pjmScore, accScoreM, delayScoreM, precScoreM ] = pjmScoringPassFailv5(referenceSignal, responseSignal, avgDemand, deltaTsec)

% Formed to align with PJM Manual and Regulation Market Issues
% Senior Task Force 
% https://www.pjm.com/committees-and-groups/closed-groups/rmistf

%% Flags
%This is a flag for operating with a 10 second delay (true = delay, false =
%no delay). Other groups add this delay to comply with the PJM manual's
%requirement "allows a 10 second latency for signal propagation delay for 
%regulating resources" Section 4.5.6 PJM Manual 12
propagationDelayFlag = true;

%"If the standard deviation of the regulation signal is less than a 
%threshold value..." -- PJM Manual 12 Section 4.4.6 pg 48
STD_THRESHOLD=0 ; % there is no indication in the PJM manual to what this threshold should be set to
%% Down Sample
% This down samples from 2sec measurements to the 10second values we need
% for the PJM score.

% down sample to 10 sec intervals
stepsPer10sec = 10/deltaTsec;

if rem(stepsPer10sec,1) ~= 0
   warning('Use time-steps that work with the PJM scoring!') 
end

reference10s = referenceSignal(1:stepsPer10sec:end);
avgReference10s = mean(reference10s);
response10s = responseSignal(1:stepsPer10sec:end);
timesteps = length(reference10s); 
correlationTime = 30; %5 minutes
if (timesteps-1)/6 ~= 60
   warning('PJM Scoring function currently designed for exactly 1 hour!') 
end

%% Output Variables Setup
% set the arrays to hold values for each time-step
delayScore = zeros(timesteps,1);
accScore = zeros(timesteps,1);
precScore = zeros(timesteps,1);
instant_pjmScore = zeros(timesteps,1);
tempCorrelation = zeros(correlationTime,1);
tempDelay = zeros(correlationTime,1);
    
%% Iteration through Regulation Signal
%starts at t+10 interval
for t = 2:timesteps
    %% Accuracy Score & Delay Score
    %if there are enough time steps left, examine the next 5 min of the
    %reference signal. If not, pull the rest available.
    if t+correlationTime <= timesteps
        refTemp = reference10s(t:t+correlationTime);
    else
        refTemp = reference10s(t:end);
    end
    %iterate over the next 5 minutes of the response sigal, if not a
    %full 5 min, take what you can and cut down the reference signal too.
    %If you're at a k timestep where you would exceed the number of
    %timesteps left, zero out your correlation & delay and move on
    for k = 1:correlationTime
        if t+correlationTime+(k-1) <= timesteps
            respTemp = response10s(t+(k-1):t+correlationTime+(k-1));
        elseif t+correlationTime+(k-1) > timesteps && t+(k-1) <= timesteps
            respTemp = response10s(t+(k-1):end);
            refTemp = refTemp(1:length(respTemp));
        else
            tempCorrelation(k) = 0;
            tempDelay(k) = 0;
            continue
        end
        
        %"If the standard deviation of the regulation signal is less than a 
        %threshold value, then the Correlation shall be calculated as
        %(1 – abs(slope of the regulation signal - the slope of the response)). The
        %performance score for Correlation and Delay will be calculated by using
        %linear regression to find the slopes of the regulation signal and the
        %resource response." -- PJM Manual 12 Section 4.4.6 pg 48
        threshold = STD_THRESHOLD; % there is no indication in the PJM manual to what this threshold should be set to
        if std(response10s) < threshold
            slope_ref = polyfit(1:length(refTemp),refTemp,1);
            slope_resp = polyfit((t+(k-1):length(respTemp)),respTemp,1);
            tempCorrelation = 1-abs(slope_ref(1) - slope_resp(1));
        elseif refTemp == respTemp
            tempCorrelation(k) = 1;
        else
            tempMat = corrcoef(refTemp, respTemp);
            if all(diff(refTemp)==0)
                tempCorrelation(k) = 0;
            elseif isnan(tempMat)
                tempCorrelation(k) = NaN;
            elseif isscalar(tempMat)
                tempCorrelation(k) = 0;
            else
                tempCorrelation(k) = tempMat(1,2);
            end
        end
        
        if propagationDelayFlag% && (t+1) < timesteps && (k+1) <= correlationTime
             tempDelay(k) = abs(((k)-correlationTime)/correlationTime);
        else
            tempDelay(k) = abs(((k-1)-correlationTime)/correlationTime);
        end
    end
    
    %"Correlation and Delay are determined together by finding the 10 second
    %interval with the highest coincident Correlation and Delay score. The 
    %10 second interval that will determine Correlation and Delay for each 
    %scoring period is: max_[\delta = 0 to 5 min] (Delay Score + Corrlation
    %score)" -- PJM Manual 12 Section 4.4.6 pg 48
    [~, tempI] = max(tempCorrelation + tempDelay);
    accScore(t) = tempCorrelation(tempI);
    delayScore(t) = tempDelay(tempI);

    %% Precision Score
    if propagationDelayFlag && (t+1) < timesteps
        error = abs((response10s(t+1) - reference10s(t))/avgReference10s);
    else
        error = abs((response10s(t) - reference10s(t))/avgReference10s);
    end
    %this is not the precision score yet, it's just the error
    precScore(t) = 1 - mean(error);
    
    instant_pjmScore(t) = 1/3*accScore(t)+ 1/3*delayScore(t) + 1/3*precScore(t);
end






%% PJM Performance Score
% calculate overall score

% max(min(XXX,1),0) = forces any score calculation above 1 to be 1 or below
% 0 to be 0
accScore = max(min(accScore,1),0);
delayScore = max(min(delayScore,1),0);
precScore = max(min(precScore,1),0);

%scoring starts at t+10 time interval
accScore = accScore(2:end);
delayScore = delayScore(2:end);
precScore = precScore(2:end);


accScoreM = mean(accScore);
delayScoreM = mean(delayScore);

precScoreM = mean(precScore);

test = 1/3*accScoreM + 1/3*delayScoreM + 1/3*precScoreM;
pjmScore = mean(instant_pjmScore(2:end));

if pjmScore >= .75
   passOrFail = 1; 
else
   passOrFail = 0;
end


end