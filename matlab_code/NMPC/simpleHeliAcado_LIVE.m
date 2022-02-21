
close all;
clc

BEGIN_ACADO;
    %% Setup the problem
    acadoSet('problemname','simpleHeliMPC_LIVE');
    
    x0 = acado.MexInputVector;
    inputStartTime = acado.MexInput;
    refTraj = acado.MexInputMatrix;
    
    %% setup the differential equation
    diffEQ = acado.DifferentialEquation();
    
    DifferentialState x y z qw qx qy qz xd yd zd p q r;
    
    Control col roll pitch yaw;
    
    tau = 0.05;
    K = 1000/tau * pi/180;
    mass = 1;
    Kcol = 5*mass/tau;
    
    Cbn = [(qw^2 + qx^2 - qy^2 - qz^2), 2*(qx*qy+qw*qz), 2*(qx*qz-qw*qy);
           2*(qx*qy-qw*qz), (qw^2 - qx^2 + qy^2 - qz^2), 2*(qy*qz+qw*qx);
           2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), (qw^2 - qx^2 - qy^2 + qz^2)]';
    
    diffEQ.add(dot(x) == xd);
    diffEQ.add(dot(y) == yd);
    diffEQ.add(dot(z) == zd);
    diffEQ.add(dot(qw) == 0.5 * (-qx*p - qy*q -qz*r));
    diffEQ.add(dot(qx) == 0.5 * (qw*p + qy*r - qz*q));
    diffEQ.add(dot(qy) == 0.5 * (qw*q - qx*r + qz*p));
    diffEQ.add(dot(qz) == 0.5 * (qw*r + qx*q - qy*p));
    diffEQ.add(dot(xd) == 1/mass * Cbn(1,3) * (-1/tau + Kcol * col));
    diffEQ.add(dot(yd) == 1/mass * Cbn(2,3) * (-1/tau + Kcol * col));
    diffEQ.add(dot(zd) == 1/mass * Cbn(3,3) * (-1/tau + Kcol * col) + 9.81);
    diffEQ.add(dot(p) == -1/tau * p + K*roll);
    diffEQ.add(dot(q) == -1/tau * q + K*pitch);
    diffEQ.add(dot(r) == -1/tau * r + K*yaw);
    
    %% OCP
    startTime = 0;
    endTime = 1;
    freq = 10;
    
    ocp = acado.OCP(startTime, endTime, endTime*freq);
    
    h = {x, y, z, qw, qx, qy, qz, xd, yd, zd, p, q, r};
    
    Q = eye(13);
        
    Q(1,1) = 10;
    Q(2,2) = 10;
    Q(3,3) = 10;
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
    
    ocp.subjectTo(diffEQ);
    
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
    algo = acado.RealTimeAlgorithm(ocp, 0.02); % Set up the optimization algorithm
    
    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-6);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-4 );
    
    algo.set('MAX_NUM_ITERATIONS', 2);

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
load('result.mat');
load('Melon1.mat');
load('lemniscate.mat');
data = Melon1; % choose reference trajectory here

trajectory = zeros(length(data(:,1)), 14);
trajectory(:,1) = data(:,1);
trajectory(:,2) = data(:,18);
trajectory(:,3) = -data(:,19);
trajectory(:,4) = -data(:,20);
trajectory(:,5) = data(:,11);
trajectory(:,6) = data(:,8);
trajectory(:,7) = -data(:,9);
trajectory(:,8) = -data(:,10);
trajectory(:,9) = data(:,15);
trajectory(:,10) = -data(:,16);
trajectory(:,11) = -data(:,17);
trajectory(:,12) = data(:,5);
trajectory(:,13) = -data(:,6);
trajectory(:,14) = -data(:,7);


dt=0.02; % time step
endTime = floor(trajectory(end,1));
t=0:dt:endTime; % have space for 300 seconds (5 minutes of simulation)

f_dyn = 'heliDynamics';
params.mass = 1;
params.tau = 0.05;
params.Kcol = 5*params.mass/params.tau;
params.K = 1000/params.tau * pi/180;

%************************************
%THIS GOES PRIOR TO SIMULATION 
%************************************
% adaptive element parameters
w_co = 15; %cut off frequency
A_s = [1,1,1,0,0,0;
       1,1,1,0,0,0;
       0,1,1,1,0,0;
       0,1,1,1,0,0;
       0,0,1,1,1,0;
       0,0,1,1,1,1]; %adaption gains (Hurwitz)
tau_c = 0.05; %first order z thrust response 
tau_p = 0.05; %first order p response 
tau_q = 0.05; %first order q response 
tau_r = 0.05; %first order r response 
m = 1; 
K_col = (5*m)/tau_c; 
K_phi = (1000/tau_p)*(pi/180);
K_theta = (1000/tau_q)*(pi/180);
K_psi = (1000/tau_r)*(pi/180);

%initialize 
u_L1 = 0; %initialize adaptive element input 
z_hat = zeros(6,1); 
G = zeros(6,1); 

% assign vectors
x = nan(length(t), 13);
xdot = x;
u = nan(length(t), 4);

x0 = trajectory(1,2:end);
x(1,:) = x0;

k = 1;
while t(k)<endTime

    out = simpleHeliMPC_LIVE_RUN(x(k,:), t(k), trajectory);
    
    u_mpc = out.U;
    
    %{
    %************************************
    %THIS GOES IN THE SIMULATION LOOP 
    %************************************

    %deconstruct state vector 
    p_n = x(k,1); p_e = x(k,2); p_d = x(k,3);
    q0 = x(k,4); q1 = x(k,5); q2 = x(k,6); q3 = x(k,7);
    v_n = x(k,8); v_e = x(k,9); v_d = x(k,10);
    p = x(k,11); q = x(k,12); r = x(k,13);
    %L1-Adaptive Augmentation 
    e_xb = [1-2*(q2^2+q3^2);2*(q1*q2+q0*q3);2*(q1*q3-q0*q2)]; 
    e_yb = [2*(q1*q2-q0*q3);1-2*(q1^2+q3^2);2*(q2*q3-q0*q1)]; 
    e_zb = [2*(q1*q3+q0*q2);2*(q2*q3-q0*q1);1-2*(q1^2+q2^2)]; 

    R_bi = [e_xb, e_yb, e_zb]; %DCM from body to inertial 

    z = [v_n v_e v_d p q r]'; %state vector 


    T_mpc = [0;0;(-1/tau_c)+K_col*u_mpc(1)];
    M_mpc = [(-1/tau_p)*p+K_phi*u_mpc(2);(-1/tau_q)*q+K_theta*u_mpc(3);(-1/tau_r)*r+K_psi*u_mpc(4)];

    f = [T_mpc.*e_zb;M_mpc]; %desired dynamics 
    g = [e_zb e_zb e_zb e_zb;zeros(3,1) eye(3)]; %uncertainty in matched component 
    g_T = [e_xb e_yb; zeros(3,2)]; %uncertainty in unmatched dynamics 

    PHI = A_s\((exp(A_s*k)) - eye(6)); 
    G = [g g_T]; 
    mu = (exp(A_s*k))*(z_hat - z); 

    sigma = -eye(6)*inv(G)*inv(PHI)*mu; %piecewise-constant adaptation law 
    sigma_m = sigma(1:4); 
    sigma_um = sigma(5:6); 

    u_L1 = u_L1*exp(-w_co*k) - sigma_m*(1 - exp(-w_co*k)); 
    z_hat = z_hat + (f + g*(u_L1 + sigma_m) + g_T*sigma_um + A_s*(z_hat - z))*k; 
    %}

    u(k,:) = u_mpc;
    
    [x(k+1,:),xdot(k,:)]= RK4_zoh(f_dyn, x(k,:), u(k,:), t, params, dt);
    
    disp(t(k));
    k = k + 1;
end

%% plotting
close all;
vis.time = t;
vis.signals.values = x;

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

figure('name', '3d')
hold on;
plot3(x(:,1), x(:,2), x(:,3));
plot3(x(1,1), x(1,2), x(1,3),'o')
plot3(x(end,1), x(end,2), x(end,3), 'x');
ylabel('E pos (m)');
xlabel('N pos(m)');
zlabel('D pos (m)');

legend('trajectory', 'start', 'end');

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


figure('name', 'velocity')
subplot(4,1,1)
plot(t, x(:,8), trajectory(:,1), trajectory(:,9))
ylabel('x vel (m/s)')

subplot(4,1,2)
plot(t, x(:,9), trajectory(:,1),trajectory(:,10))
ylabel('y vel (m/s)');

subplot(4,1,3)
plot(t, x(:,10), trajectory(:,1), trajectory(:,11));
ylabel('z vel (m/s)');

subplot(4,1,4)
plot(t, sqrt(x(:,8).* x(:,8) + x(:,9).* x(:,9) + x(:,10).* x(:,10)));
ylabel('total velocity (m/s)')
xlabel('time (sec)');


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










