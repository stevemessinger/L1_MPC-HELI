%{
AERSP 596: Autonomous Acrobatic Helicopter Project
Steve Messinger 
AERSP 597 Optimal Control Final Project: Model Predictive Control of a Small Un-occupied helicopter
Helicopter Dynamics from the Handbook of Unmanned Aerial Vehicles: Dynamic Model for a Miniature Aerobatic Helicopter
%}

clear all 
close all 
clc

%assign params
heli_params; 

dt=0.05; % timestep 
t=0:dt:10; % start -> endtime 

dyn='helicopter'; % define the model
% allocate space for the arrays
x=nan(17,length(t));
xdot=nan(17,length(t));

x0 = zeros(17,1); 
u = [0 0 0 0 0 0]; 

% assign initial conditions
x(:,1)=x0;

for k=1:length(t)-1  
    [x(:,k+1),xdot(:,k)]= RK4_zoh(dyn,x(:,k),u,t(k),heli,dt);
end