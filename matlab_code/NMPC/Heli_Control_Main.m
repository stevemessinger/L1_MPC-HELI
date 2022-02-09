%{
Main function for performing control of the helicopter:

Processes State data from VICON tracking system, computes the NMPC signal,
applies L1 adaptive element, and streams control information to hardware
%}

%% Adaption Element Setup