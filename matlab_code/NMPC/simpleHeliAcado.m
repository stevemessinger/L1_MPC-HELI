clearvars;
close all;
clc

BEGIN_ACADO;
    %% Setup the problem
    acadoSet('problemname','simpleHeliMPC');
    
    %% setup the differential equation
    f = acado.DifferentialEquation();
    
    DifferentialState x y z qw qx qy qz xd yd zd p q r;
    
    Control col roll pitch yaw;
    
    tau = 0.05;
    K = 720/tau * pi/180;
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
    endTime = 10;
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
    Q(11,11) = 1/(720*pi/180);
    Q(12,12) = 1/(720*pi/180);
    Q(13,13) = 1/(720*pi/180);
    
    ref = zeros(1,13);
    ref(4) = 1;
    ref(1) = -5;
    ref(2) = 0;
    ref(3) = -5;
    
    ocp.minimizeLSQ(Q, h, ref);
    
    ocp.subjectTo(f);
    
    ocp.subjectTo('AT_START', x == 0);
    ocp.subjectTo('AT_START', y == 0);
    ocp.subjectTo('AT_START', z == 0);
    ocp.subjectTo('AT_START', qw == 1);
    ocp.subjectTo('AT_START', qx == 0);
    ocp.subjectTo('AT_START', qy == 0);
    ocp.subjectTo('AT_START', qz == 0);
    ocp.subjectTo('AT_START', xd == 0);
    ocp.subjectTo('AT_START', yd == 0);
    ocp.subjectTo('AT_START', zd == 0);
    ocp.subjectTo('AT_START', p == 0);
    ocp.subjectTo('AT_START', q == 0);
    ocp.subjectTo('AT_START', r == 0);
    
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
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-6);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-2);
    
    algo.set('MAX_NUM_ITERATIONS', 100);

    algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );  % Example setting hessian approximation
    %algo.set( 'HESSIAN_APPROXIMATION', 'CONSTANT_HESSIAN' );  % Other possible settings
    %algo.set( 'HESSIAN_APPROXIMATION', 'FULL_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'BLOCK_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON_WITH_BLOCK_BFGS' );
END_ACADO;

%% run the optimizer
clearvars;
close all;
clc

tic;
output = simpleHeliMPC_RUN();
toc;

vis.time = output.STATES(:,1);
vis.signals.values = output.STATES(:,2:end);

%% plotting
figure('name', 'position');
subplot(3,1,1);
plot(output.STATES(:,1), output.STATES(:,2));
ylabel('x pos (m)');

subplot(3,1,2);
plot(output.STATES(:,1), output.STATES(:,3));
ylabel('y pos (m)');

subplot(3,1,3);
plot(output.STATES(:,1), output.STATES(:,4));
ylabel('z pos (m)');
xlabel('time (sec)');

euler = quat2eul(output.STATES(:,5:8), 'XYZ');
figure('name', 'angles');
subplot(3,1,1);
plot(output.STATES(:,1), euler(:,1)*180/pi);
ylabel('rol (deg)');

subplot(3,1,2);
plot(output.STATES(:,1), euler(:,2)*180/pi);
ylabel('pitch (deg)');

subplot(3,1,3);
plot(output.STATES(:,1), euler(:,3)*180/pi);
ylabel('yaw (deg)');
xlabel('time (sec)');


figure('name', 'rates')
subplot(3,1,1);
plot(output.STATES(:,1), output.STATES(:,12) * 180/pi);
ylabel('p (m)');

subplot(3,1,2);
plot(output.STATES(:,1), output.STATES(:,13) * 180/pi);
ylabel('q (m)');

subplot(3,1,3);
plot(output.STATES(:,1), output.STATES(:,14) * 180/pi);
ylabel('r (m)');
xlabel('time (sec)');


figure('name', 'inputs');
subplot(4,1,1)
plot(output.CONTROLS(:,1), output.CONTROLS(:,2));
ylabel('collective');

subplot(4,1,2)
plot(output.CONTROLS(:,1), output.CONTROLS(:,3));
ylabel('roll');

subplot(4,1,3)
plot(output.CONTROLS(:,1), output.CONTROLS(:,4));
ylabel('pitch');

subplot(4,1,4)
plot(output.CONTROLS(:,1), output.CONTROLS(:,5));
ylabel('yaw');
xlabel('time (sec)');











