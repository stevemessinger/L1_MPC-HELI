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

dt=0.001; % timestep 
t=0:dt:10; % start -> endtime 

dyn='helicopter'; % define the model
% allocate space for the arrays
x=nan(18,length(t));
xdot=nan(18,length(t));
euler=nan(3,length(t));

x0 = zeros(18,1); 
x0(7) = 1; 
x0(16) = 167; 
u = [0.5 0 0 0 0 0.2]; 

% assign initial conditions
x(:,1)=x0;

for k=1:length(t)-1  
    [x(:,k+1),xdot(:,k)]= RK4_zoh(dyn,x(:,k),u,t(k),heli,dt);
    euler(:,k) = quat2eul(x(7:10,k+1)')*(180/pi); 
end

figure(1) 
subplot(3,2,1)
plot(t,x(1,:))
hold on
plot(t,x(2,:))
ylim([-20,20])
legend('N_{pos}','E_{pos}','location','southwest')
subplot(3,2,3)
plot(x(2,:),x(1,:))
ylabel('N_{pos}')
xlabel('E_{pos}')
ylim([-40,40])
xlim([-40,40])
subplot(3,2,5)
plot(t,-x(3,:))
ylim([-1,10])
ylabel('h')
subplot(3,2,2)
plot(t,euler(3,:))
ylabel('\phi')
subplot(3,2,4)
plot(t,euler(2,:))
ylabel('\theta')
subplot(3,2,6)
plot(t,euler(1,:))
ylabel('\psi')

