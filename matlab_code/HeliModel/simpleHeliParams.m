heli.cT = 0.05; % 1/(rad/s)^2
heli.omega = 167; %rad/s
heli.mass = 8.2;% kg
heli.I = diag([0.18; 0.34; 0.28]); %kg*m^2
heli.drag = [0; 0; 0];
heli.mainRotorInertia = 0.038; %main rotor MoI
heli.IdriveTrain = 2.5*heli.mainRotorInertia;
heli.h = [0, 0, 0];
heli.g = 9.81; %m/s^2