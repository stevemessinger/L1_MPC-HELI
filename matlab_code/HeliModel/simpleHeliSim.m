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
simpleHeliParams; 

dt=0.01; % timestep 
t=0:dt:10; % start -> endtime 

dyn='simpleHeli'; % define the model
% allocate space for the arrays
x=nan(13,length(t)); %[x y z w x y z vx vy vz p q r]
xdot=nan(13,length(t));
euler=nan(3,length(t));

x0 = zeros(13,1); 
x0(4) = 1; 
u = [0; 0; 0; 1]*pi/180; %u = [collective command, roll cyclic, pitch cyclic, rudder]

% assign initial conditions
x(:,1)=x0;

for k=1:length(t)-1  
    [x(:,k+1),xdot(:,k)]= RK4_zoh(dyn,x(:,k),u,t(k),heli,dt);
    euler(:,k) = quat2eul(x(4:7,k+1)')*(180/pi); 
end

figure(1) 
subplot(3,2,1)
plot(t,x(1,:))
hold on
plot(t,x(2,:))
legend('N_{pos}','E_{pos}','location','southwest')

subplot(3,2,3)
plot(x(2,:),x(1,:))
ylabel('N_{pos}')
xlabel('E_{pos}')

subplot(3,2,5)
plot(t,-x(3,:))
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

figure(2)
subplot(3,1,1)
plot(t, x(11,:))
ylabel('roll rate')

subplot(3,1,2)
plot(t,x(12,:))
ylabel('pitchrate')

subplot(3,1,3)
plot(t, x(13,:))
ylabel('yaw rate')

