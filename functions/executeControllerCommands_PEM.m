% Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services
% Copyright (C) 2023 The Regents of the University of Michigan
% This is a free software and comes with ABSOLUTELY NO WARRANTY; for details see the license in the license.txt file.

function [m_vect, controllerInformationTemp] = executeControllerCommands_PEM(controllerInformationTemp, controlActions, logical_index_high, logical_index_low, locked, m_vect,n,airTemperatureVector,timeStepInSec)
%{
This function carries out the PEM controller commands.

Input:
    controllerInformationTemp: struct
        Info for the controller, includes voltage limits if we want to take
        them into account in the switch actions
    onOffSignal: bool
        Packet requests from PEM controller for all TCLs. 0 if TCL isn't requesting
        and 1 for an energy packet request
    logical_index_high: vector of booleans
        Indicator of whether each TCL has exceeded its upper deadband limit
    logical_index_low: vector of booleans
        Indicator of whether each TCL has exceeded its lower deadband limit
    locked: vector of booleans
        Indicator of whether each TCL is locked or unlocked
    m_vect: vector of booleans
        ON/OFF mode of each TCL at the current timestep.
Output:
    m_vect: vector of booleans
        Updated ON/OFF mode for each TCL after taking into account the
        request acceptance rate of the controller.
    controllerInformation: struct
        Updated with controllable TCLs,
%}
%% PEM Logic
    theseTcls = controllerInformationTemp.tclsAssigned;
    T_min = controllerInformationTemp.T_min;
    T_max = controllerInformationTemp.T_max;
    MTTR = controllerInformationTemp.MTTR;
    
    %ON/OFF switching based on Aggregator turn-on/turn-off commands
    m_vect(theseTcls((controlActions.onOffSignal(theseTcls) == 1 ...
        & logical_index_high(theseTcls) ==0 & logical_index_low(theseTcls) ==0 & locked(theseTcls) ==0))) = 1;

    m_vect(theseTcls((controlActions.onOffSignal(theseTcls) == -1 ...
        & logical_index_high(theseTcls) ==0 & logical_index_low(theseTcls) ==0 & locked(theseTcls) ==0))) = 0;

    MTTRoff = controllerInformationTemp.MTTRoff;

    %TCL Local PEM Request evaluation
    [onRequests, offRequests, controllerInformationTemp] = Extended_PEM(airTemperatureVector,T_min,T_max,MTTR,MTTRoff,timeStepInSec,m_vect,locked,theseTcls,n+1,controllerInformationTemp);
end
