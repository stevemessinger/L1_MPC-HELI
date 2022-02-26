
close all;
clearvars;
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
    Kcol = 5*mass*9.81;
    
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
    diffEQ.add(dot(xd) == 1/mass * Cbn(1,3) * (Kcol * col));
    diffEQ.add(dot(yd) == 1/mass * Cbn(2,3) * (Kcol * col));
    diffEQ.add(dot(zd) == 1/mass * Cbn(3,3) * (Kcol * col) + 9.81);
    diffEQ.add(dot(p) == -1/tau * p + K*roll);
    diffEQ.add(dot(q) == -1/tau * q + K*pitch);
    diffEQ.add(dot(r) == -1/tau * r + K*yaw);
    
    %% OCP
    startTime = 0;
    endTime = 1;
    freq = 5;
    
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
    algo = acado.RealTimeAlgorithm(ocp, 0.01); % Set up the optimization algorithm
    
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
load('result.mat');
load('Melon1.mat');
load('lemniscate.mat');
load('circle.mat');
circle(:,1) = circle(:,1) - circle(1,1);
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


dt=0.002; % time step
endTime = floor(trajectory(end,1));
t=0:dt:endTime; % have space for 300 seconds (5 minutes of simulation)

f_dyn = 'heliDynamics';
params.mass = 1; 
params.tau = 0.05;
params.Kcol = 5*params.mass*9.81;
params.K = 1000/params.tau * pi/180;

%************************************
%THIS GOES PRIOR TO SIMULATION 
%************************************
% adaptive element parameters
w_co = 15; %cut off frequency
A_s = -5*[1,0,0,0,0,0;
       0,1,0,0,0,0;
       0,0,1,0,0,0;
       0,0,0,1,0,0;
       0,0,0,0,1,0;
       0,0,0,0,0,1]; %adaption gains (Hurwitz)

tau_c = 0.05; %first order z thrust response 
tau_p = 0.05; %first order p response 
tau_q = 0.05; %first order q response 
tau_r = 0.05; %first order r response 
m = 1; 
K_col = (5*m)*9.81; 
K_phi = (1000/tau_p)*(pi/180);
K_theta = (1000/tau_q)*(pi/180);
K_psi = (1000/tau_r)*(pi/180);

%initialize 
u_L1 = zeros(4,length(t),2); %initialize adaptive element input 
u_mpc = u_L1; 
z_hat = zeros(6,length(t),2); 
z = z_hat;
G = zeros(6,1); 

sigm_temp = zeros(6,length(t),2); 

% assign vectors
x = nan(length(t), 13, 2);
xdot = x;
u = nan(length(t), 4,2);

for i = 1:2
    x0 = trajectory(1,2:end);
    x(1,:,i) = x0;
    z_hat(:,1,i) = [x0(8),x0(9),x0(10),x0(11),x0(12),x0(13)]'; 

    k = 1;
    count = 1;
    out = simpleHeliMPC_LIVE_RUN(x(k,:,i), t(k), trajectory);
    u_mpc(:,k,i) = out.U';
    while t(k)<endTime-1*dt
    
        if count == 5
            out = simpleHeliMPC_LIVE_RUN(x(k,:,i), t(k), trajectory);
            count = 1;
        end
        u_mpc(:,k,i) = out.U';
        %************************************
        %THIS GOES IN THE SIMULATION LOOP 
        %************************************
    
        %deconstruct state vector 
        p_n = x(k,1,i); p_e = x(k,2,i); p_d = x(k,3,i);
        q0 = x(k,4,i); q1 = x(k,5,i); q2 = x(k,6,i); q3 = x(k,7,i);
        v_n = x(k,8,i); v_e = x(k,9,i); v_d = x(k,10,i);
        p = x(k,11,i); q = x(k,12,i); r = x(k,13,i);
        %L1-Adaptive Augmentation 
        R_bi = [(q0^2 + q1^2 - q2^2 - q3^2), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2);
                 2*(q1*q2-q0*q3), (q0^2 - q1^2 + q2^2 - q3^2), 2*(q2*q3+q0*q1);
                 2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), (q0^2 - q1^2 - q2^2 + q3^2)]';
    
        e_xb = R_bi(:,1); 
        e_yb = R_bi(:,2);
        e_zb = R_bi(:,3);
    
        z(:,k,i) = [v_n v_e v_d p q r]'; %state vector 
    
        T_mpc = K_col*u_mpc(1,k,i)/m;
        M_mpc = [(-1/tau_p)*p+K_phi*u_mpc(2,k,i);(-1/tau_q)*q+K_theta*u_mpc(3,k,i);(-1/tau_r)*r+K_psi*u_mpc(4,k,i)];
    
        f = [[0;0;9.81]+T_mpc.*e_zb;M_mpc]; %desired dynamics 
        
        g = [e_zb zeros(3,3);zeros(3,1) eye(3)]; %uncertainty in matched component 
        g_T = [e_xb e_yb; zeros(3,2)]; %uncertainty in unmatched dynamics 
    
        PHI = inv(A_s)*(expm(A_s*dt) - eye(6)); 
        G = [g g_T];
        mu = expm(A_s*dt)*(z_hat(:,k,i) - z(:,k,i)); 
    
        sigma = -eye(6)*inv(G)*inv(PHI)*mu; %piecewise-constant adaptation law 
        sigma_m = sigma(1:4); 
        sigma_um = sigma(5:6); 
    
        temp = [sigma_um(1:2);sigma_m(1)]; 
        sigm_temp(:,k,i) = [temp;sigma_m(2:4)]; 
        
        u_L1(:,k+1,i) = u_L1(:,k,i)*exp(-w_co*dt) - sigma_m*(1 - exp(-w_co*dt)); 
        z_hat(:,k+1,i) = z_hat(:,k,i) + (f + g*(u_L1(:,k+1,i) + sigma_m) + g_T*sigma_um + A_s*(z_hat(:,k,i) - z(:,k,i)))*dt; 
    
        if i == 1
            u(k,:,i) = u_mpc(:,k,i);
        else 
            u(k,:,i) = u_mpc(:,k,i) + u_L1(:,k+1,i)./[K_col;K_phi;K_theta;K_psi];
        end
        
        [x(k+1,:,i), xdot(k,:,i)] = RK4_zoh(f_dyn, x(k,:,i), u(k,:,i), t(k), params, dt);
        
        disp(t(k));
        k = k + 1;
        count = count + 1;
    end
end
%% plotting
% close all;
vis.time = t(1:end-1);
vis.signals.values = x(1:end-1,:,1);

figure('name', 'position');
subplot(3,1,1);
plot(t, x(:,1,1), t, x(:,1,2), trajectory(:,1), trajectory(:,2));
ylabel('x pos (m)');
legend('MPC', 'L1+MPC', 'Reference');

subplot(3,1,2); 
plot(t, x(:,2,1), t, x(:,2,2),trajectory(:,1), trajectory(:,3));
ylabel('y pos (m)');
legend('MPC', 'L1+MPC', 'Reference');

subplot(3,1,3);
plot(t, x(:,3,1), t, x(:,3,2),trajectory(:,1), trajectory(:,4));
ylim([-5,5])
ylabel('z pos (m)');
xlabel('time (sec)');
legend('MPC', 'L1+MPC', 'Reference');

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
figure('name', 'angle');
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
hold on;
plot(t, x(:,4,1), t, x(:,4,2), trajectory(:,1), trajectory(:,5));
ylabel('qw')

subplot(4,1,2)
plot(t, x(:,5,1), t, x(:,5,2), trajectory(:,1), trajectory(:,6));
ylabel('qx')

subplot(4,1,3)
plot(t, x(:,6,1), t, x(:,6,2),trajectory(:,1), trajectory(:,7));
ylabel('qy')

subplot(4,1,4)
plot(t, x(:,7,1), t, x(:,7,2),trajectory(:,1), trajectory(:,8));
ylabel('qz')
xlabel('time (sec)')
legend('MPC', 'L1+MPC', 'Reference');


figure('name', 'velocity')
subplot(4,1,1)
plot(t, x(:,8,1), t, x(:,8,2),trajectory(:,1), trajectory(:,9))
ylabel('x vel (m/s)')

subplot(4,1,2)
plot(t, x(:,9,1), t, x(:,9,2),trajectory(:,1),trajectory(:,10))
ylabel('y vel (m/s)');

subplot(4,1,3)
plot(t, x(:,10,1), t, x(:,10,2),trajectory(:,1), trajectory(:,11));
ylabel('z vel (m/s)');
legend('MPC', 'L1+MPC', 'Reference');

subplot(4,1,4)
plot(t, sqrt(x(:,8).* x(:,8) + x(:,9).* x(:,9) + x(:,10).* x(:,10)));
ylabel('total velocity (m/s)')
xlabel('time (sec)');


figure('name', 'rates')
subplot(3,1,1);
plot(t, x(:,11,1) * 180/pi, t, x(:,11,2) * 180/pi,trajectory(:,1), trajectory(:,12)*180/pi);
ylabel('p (m)');

subplot(3,1,2);
plot(t, x(:,12,1) * 180/pi, t, x(:,12,2) * 180/pi,trajectory(:,1), trajectory(:,13)*180/pi);
ylabel('q (m)');

subplot(3,1,3);
plot(t, x(:,13,1) * 180/pi, t, x(:,13,2) * 180/pi,trajectory(:,1), trajectory(:,14)*180/pi);
ylabel('r (m)');
xlabel('time (sec)');
legend('MPC', 'L1+MPC', 'Reference');


figure('name', 'inputs');
subplot(4,1,1)
plot(t, u(:,1,1), t, u(:,1,2));
ylabel('collective');

subplot(4,1,2)
plot(t, u(:,2,1), t, u(:,2,2));
ylabel('roll');

subplot(4,1,3)
plot(t, u(:,3,1),t, u(:,3,1));
ylabel('pitch');

subplot(4,1,4)
plot(t, u(:,4,1), t, u(:,4,2));
ylabel('yaw');
xlabel('time (sec)');
legend('MPC', 'L1+MPC');

figure('name', 'sigma')
subplot(3,2,1)
plot(t,sigm_temp(1,:,2))
ylabel('x')
subplot(3,2,2)
plot(t,sigm_temp(2,:,2))
ylabel('y')
subplot(3,2,3)
plot(t,sigm_temp(3,:,2))
ylabel('z')
subplot(3,2,4)
plot(t,sigm_temp(4,:,2))
ylabel('\phi')
subplot(3,2,5)
plot(t,sigm_temp(5,:,2))
ylabel('\theta')
subplot(3,2,6)
plot(t,sigm_temp(6,:,2))
ylabel('\psi')

figure('name', 'observer')
subplot(3,2,1)
hold on 
plot(t,z(1,:,2))
plot(t,z_hat(1,:,2))
legend('actual','estimate')
ylabel('N_{vel}')
subplot(3,2,2)
hold on 
plot(t,z(2,:,2))
plot(t,z_hat(2,:,2))
legend('actual','estimate')
ylabel('E_{vel}')
subplot(3,2,3)
hold on 
plot(t,z(3,:,2))
plot(t,z_hat(3,:,2))
legend('actual','estimate')
ylabel('D_{vel}')
subplot(3,2,4)
hold on 
plot(t,z(4,:,2))
plot(t,z_hat(4,:,2))
legend('actual','estimate')
ylabel('p')
subplot(3,2,5)
hold on 
plot(t,z(5,:,2))
plot(t,z_hat(5,:,2))
legend('actual','estimate')
ylabel('q')
subplot(3,2,6)
hold on 
plot(t,z(6,:,2))
plot(t,z_hat(6,:,2))
legend('actual','estimate')
ylabel('r')








