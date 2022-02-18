function xdot = simpleHeli(x, u, t, params)
%{
Description: this function describes the dynamics of the helicoter in the
NED frame

x = state of the vehicle in the NED frame
u = [collective command, roll cyclic, pitch cyclic, rudder]
params = struct containing vehicle parameters

%}
%% initialize important variables
xdot = nan(13,1);

Cbn = [(x(4)^2 + x(5)^2 - x(6)^2 - x(7)^2), 2*(x(5)*x(6)+x(4)*x(7)), 2*(x(5)*x(7)-x(4)*x(6));
       2*(x(5)*x(6)-x(4)*x(7)), (x(4)^2 - x(5)^2 + x(6)^2 - x(7)^2), 2*(x(6)*x(7)+x(4)*x(5));
       2*(x(5)*x(7)+x(4)*x(6)), 2*(x(6)*x(7)-x(4)*x(5)), (x(4)^2 - x(5)^2 - x(6)^2 + x(7)^2)]; % DCM from intertial to body frame


%% propogate linear kinematics
xdot(1:3) = x(8:10);

%% propogate angular kinematics
p = x(11);
q = x(12);
r = x(13);

wBN = [p, q, r]'; % angular velocity of the vehicle with respect to the (assumed inertial) NED frame, expressed in the body frame

omegaBN = [0, -p, -q, -r;
           p, 0, r, -q;
           q, -r, 0, p;
           r, q, -p, 0];
 
xdot(4:7) = 0.5 * omegaBN * x(4:7); % coriollis angular acceleration

F = [0; 0; -params.cT * params.omega^2 * sin(u(1))] -  params.drag .* x(8:10).^2; % forces on the vehicle in the body frame

xdot(8:10) = ((1/params.mass) * Cbn' * F) + [0; 0; params.g]; % linear acceleration in the NED frame

M = [params.cT * params.omega^2 * sin(u(2)), params.cT * params.omega^2 * sin(u(3)), u(4)]'; % moments on the vehicle in the body frame

xdot(11:13) = params.I\(M - cross(wBN, (params.I*wBN + params.h'))); % resulting momements on the vehicle including internal angular momentum

end