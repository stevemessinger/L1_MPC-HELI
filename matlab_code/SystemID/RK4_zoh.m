function [xnew,xdot]= RK4_zoh(fdyn,x,u,t,params,dt)
%{
Performs 4th order Runge-Kutta Integration of the equations xdot=f(t,x,u).
This version uses a zero-order hold on the inputs.

fdyn:   dynamic model
u:      input (held constant over time interval dt)
time:   time
dt:     time step
x:      x(t)

output:
xnew:   x(t+dt)
xdot:   xdot(t)

fdyn must be set up as

xdot=fdyn(x,u,t,params)

where params is a structure (or vector) that contains all the parameters of
the dynamic model (e.g. mass, moment of inertia, aero coefficients, etc.)

Modified from Joe Horn's rk4.m by JWL, Feb. 2 2021

%}

xd=feval(fdyn,x,u,t,params);
xdot=xd;
k1=dt*xd;
xd=feval(fdyn,x+0.5*k1,u,t+0.5*dt,params);
k2=dt*xd;
xd=feval(fdyn,x+0.5*k2,u,t+0.5*dt,params);
k3=dt*xd;
xd=feval(fdyn,x+k3,u,t+dt,params);
k4=dt*xd;

xnew=x+k1/6+k2/3+k3/3+k4/6;

return;