
close all;
clc

BEGIN_ACADO;
    %% Setup the problem
    acadoSet('problemname','simpleHeliMPC_LIVE');
    
    x0 = acado.MexInputVector;
    inputStartTime = acado.MexInput;
    refTraj = acado.MexInputMatrix;
    
    %% setup the differential equation
    f = acado.DifferentialEquation();
    
    DifferentialState x y z qw qx qy qz xd yd zd p q r;
    
    Control col roll pitch yaw;
    
    tau = 0.05;
    K = 1000/tau * pi/180;
    mass = 1;
    Kcol = 5*mass/tau;
    
    Cbn = [(qw^2 + qx^2 - qy^2 - qz^2), 2*(qx*qy+qw*qz), 2*(qx*qz-qw*qy);
           2*(qx*qy-qw*qz), (qw^2 - qx^2 + qy^2 - qz^2), 2*(qy*qz+qw*qx);
           2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), (qw^2 - qx^2 - qy^2 + qz^2)]';
    
    f.add(dot(x) == xd);
    f.add(dot(y) == yd);
    f.add(dot(z) == zd);
    f.add(dot(qw) == 0.5 * (-qx*p - qy*q -qz*r));
    f.add(dot(qx) == 0.5 * (qw*p + qy*r - qz*q));
    f.add(dot(qy) == 0.5 * (qw*q - qx*r + qz*p));
    f.add(dot(qz) == 0.5 * (qw*r + qx*q - qy*p));
    f.add(dot(xd) == 1/mass * Cbn(1,3) * (-1/tau + Kcol * col));
    f.add(dot(yd) == 1/mass * Cbn(2,3) * (-1/tau + Kcol * col));
    f.add(dot(zd) == 1/mass * Cbn(3,3) * (-1/tau + Kcol * col) + 9.81);
    f.add(dot(p) == -1/tau * p + K*roll);
    f.add(dot(q) == -1/tau * q + K*pitch);
    f.add(dot(r) == -1/tau * r + K*yaw);
    
    %% OCP
    startTime = 0;
    endTime = 3;
    freq = 10;
    
    ocp = acado.OCP(startTime, endTime, endTime*freq);
    
    h = {x, y, z, qw, qx, qy, qz, xd, yd, zd, p, q, r};
    
    Q = eye(13);
    
    Q(4,4) = 5;
    Q(5,5) = 5;
    Q(6,6) = 5;
    Q(7,7) = 5;
    Q(8,8) = 1/20;
    Q(9,9) = 1/20;
    Q(10,10) = 1/20;
    Q(11,11) = 1;
    Q(12,12) = 1;
    Q(13,13) = 1;
    
    ref = zeros(1,13);
    ref(4) = 1;
    
    ocp.minimizeLSQ(Q, h, ref);
    
    ocp.subjectTo(f);
    
    % formulated to be equivelent to stick inputs
    colMax = 1;
    rollMax = 1;
    pitchMax = 1;
    yawMax = 1;
    
    ocp.subjectTo(-colMax <= col <= colMax);
    ocp.subjectTo(-rollMax <= roll <= rollMax);
    ocp.subjectTo(-pitchMax <= pitch <= pitchMax);
    ocp.subjectTo(-yawMax <= yaw <= yawMax);

    %% Optimization Algorithm
    algo = acado.RealTimeAlgorithm(ocp, 0.1); % Set up the optimization algorithm
    
    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-6);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-4 );
    
    algo.set('MAX_NUM_ITERATIONS', 3);

    algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );  % Example setting hessian approximation
    %algo.set( 'HESSIAN_APPROXIMATION', 'CONSTANT_HESSIAN' );  % Other possible settings
    %algo.set( 'HESSIAN_APPROXIMATION', 'FULL_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'BLOCK_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON_WITH_BLOCK_BFGS' );

    %% Define Stand Alone Controller
    reference = acado.StaticReferenceTrajectory(refTraj);
    
    controller = acado.Controller(algo, reference);
    
    controller.init(inputStartTime, x0);
    controller.step(inputStartTime, x0);

END_ACADO;

%% Run a simulation
load('result.mat')
trajectory = zeros(size(result));
trajectory(:,1:8) = result(:,1:8);

dt=0.1; % time step
endTime = 50;
t=0:dt:endTime; % have space for 300 seconds (5 minutes of simulation)

f_dyn = 'heliDynamics';
params.mass = 1;
params.tau = 0.05;
params.Kcol = 5*params.mass/params.tau;
params.K = 1000/params.tau * pi/180;

x = nan(length(t), 13);
xdot = x;
u = nan(length(t), 4);

x0 = trajectory(1,2:end);
x(1,:) = x0;

k = 1;
while t(k)<endTime
    
    tic;
    out = simpleHeliMPC_LIVE_RUN(x(k,:), t(k), trajectory);
    toc;
    
    u(k,:) = out.U;
    
    [x(k+1,:),xdot(k,:)]= RK4_zoh(f_dyn, x(k,:), u(k,:), t, params, dt);
    
    k = k + 1;
end


%vis.time = t;
%vis.signals.values = x(:,2:end);

%% plotting
figure('name', 'position');
subplot(3,1,1);
plot(t, x(:,1), trajectory(:,1), trajectory(:,2));
ylabel('x pos (m)');
legend('state', 'reference');

subplot(3,1,2);
plot(t, x(:,2), trajectory(:,1), trajectory(:,3));
ylabel('y pos (m)');
legend('state', 'reference');

subplot(3,1,3);
plot(t, x(:,3), trajectory(:,1), trajectory(:,4));
ylabel('z pos (m)');
xlabel('time (sec)');
legend('state', 'reference');

euler = quat2eul(x(:,4:7), 'XYZ');
eulerRef = quat2eul(trajectory(:,5:8), 'XYZ');
figure('name', 'angles');
subplot(3,1,1);
plot(t, euler(:,1)*180/pi, trajectory(:,1), eulerRef(:,1)*180/pi);
ylabel('rol (deg)');

subplot(3,1,2);
plot(t, euler(:,2)*180/pi, trajectory(:,1), eulerRef(:,2)*180/pi);
ylabel('pitch (deg)');

subplot(3,1,3);
plot(t, euler(:,3)*180/pi, trajectory(:,1), eulerRef(:,3)*180/pi);
ylabel('yaw (deg)');
xlabel('time (sec)');

figure('name', 'quat')
subplot(4,1,1)
plot(t, x(:,4), trajectory(:,1), trajectory(:,5));
ylabel('qw')

subplot(4,1,2)
plot(t, x(:,5), trajectory(:,1), trajectory(:,6));
ylabel('qx')

subplot(4,1,3)
plot(t, x(:,6), trajectory(:,1), trajectory(:,7));
ylabel('qy')

subplot(4,1,4)
plot(t, x(:,7), trajectory(:,1), trajectory(:,8));
ylabel('qz')
xlabel('time (sec)')

figure('name', 'rates')
subplot(3,1,1);
plot(t, x(:,11) * 180/pi, trajectory(:,1), trajectory(:,12)*180/pi);
ylabel('p (m)');

subplot(3,1,2);
plot(t, x(:,12) * 180/pi, trajectory(:,1), trajectory(:,13)*180/pi);
ylabel('q (m)');

subplot(3,1,3);
plot(t, x(:,13) * 180/pi, trajectory(:,1), trajectory(:,14)*180/pi);
ylabel('r (m)');
xlabel('time (sec)');


figure('name', 'inputs');
subplot(4,1,1)
plot(t, u(:,1));
ylabel('collective');

subplot(4,1,2)
plot(t, u(:,2));
ylabel('roll');

subplot(4,1,3)
plot(t, u(:,3));
ylabel('pitch');

subplot(4,1,4)
plot(t, u(:,4));
ylabel('yaw');
xlabel('time (sec)');











