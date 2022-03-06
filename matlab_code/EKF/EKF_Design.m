%{
Extended Kalman Filter design using Vicon motion capture and IMU data

Steve Messinger 
Sources: Optimal State Estimation -Dan Simon 
%}

clear all 
close all 
clc

% IMU data organized as follows 
%time	field.ax	field.ay	field.az	field.gx	field.gy	field.gz
%(1)       (2)         (3)         (4)         (5)         (6)         (7)

% MIT data organization 
%time	field.header.seq	field.header.stamp	field.header.frame_id	field.pose.position.x	field.pose.position.y	field.pose.position.z	
%field.pose.orientation.x	field.pose.orientation.y	field.pose.orientation.z	field.pose.orientation.w

load('mit_imu.mat') %load MIT imu data
load('mit_vicon.mat') %load MIT imu data
% load('result.mat');
% load('Melon1.mat');
% load('lemniscate.mat');
% load('circle.mat');
% circle(:,1) = circle(:,1) - circle(1,1);
% data = Melon1; % choose reference trajectory here
% 
% trajectory = zeros(length(data(:,1)), 14);
% trajectory(:,1) = data(:,1);
% trajectory(:,2) = data(:,18);
% trajectory(:,3) = -data(:,19);
% trajectory(:,4) = -data(:,20);
% trajectory(:,5) = data(:,11);
% trajectory(:,6) = data(:,8);
% trajectory(:,7) = -data(:,9);
% trajectory(:,8) = -data(:,10);
% trajectory(:,9) = data(:,15);
% trajectory(:,10) = -data(:,16);
% trajectory(:,11) = -data(:,17);
% trajectory(:,12) = data(:,5);
% trajectory(:,13) = -data(:,6);
% trajectory(:,14) = -data(:,7);

%% SETUP 

% choose which data to use 
run_case = 'mit'; % 'mit' runs mit blackbird test data
num_steps = 1000; % number of simulation steps to run 

%assign data 
switch run_case
    case 'mit'
        vicon_data = [mitviconquad(1:num_steps,1)*(1e-9) mitviconquad(1:num_steps,5:11)]; % [time xyz_pos q1 q2 q3 q0]
        imu_data = [mitimuquad1(1:num_steps,1)*(1e-9) mitimuquad1(1:num_steps,5:7) mitimuquad1(1:num_steps,2:4)]; % [time xyz_accel pqr]
end

%EKF filter operating specifications
ekf_freq = 100; 

% hardware specifications 
imu_freq = 1/(mean(imu_data(2:end,1) - imu_data(1:end-1,1)));  %100; % update frequency of imu (Hz)
vicon_freq = 1/(mean(vicon_data(2:end,1) - vicon_data(1:end-1,1))); %180; % update frequency of vicon pose data (Hz)

% NEED TO GIT RID OF THIS
vicon_data(1:num_steps,1) = vicon_data(:,1) - vicon_data(1,1); %initialize time to zero 
imu_data(1:num_steps,1) = imu_data(:,1) - imu_data(1,1); 

% setup simulation
ekf_time = vicon_data(1,1):1/ekf_freq:length(vicon_data)*(1/ekf_freq);
[max_freq,I] = max([ekf_freq,imu_freq,vicon_freq]);
switch I
    case 1
        dt = 0:1/max_freq:max(length(vicon_data),length(imu_data))*(1/max_freq); 
    case 2
        dt = imu_data(2:end,1) - imu_data(1:end-1,1); 
    case 3
        dt = vicon_data(2:end,1) - vicon_data(1:end-1,1); 
end 

%% SIMULATION
% allocate space for arrays
x_hat = nan(max(length(vicon_data),length(imu_data)),16); % states [1:p_x 2:p_y 3:p_z 4:v_x 5:v_y 6:v_z 7:q0 8:q1 9:q2 10:q3 11:b_ax 12:b_ay 13:b_az 14:b_wx 15:b_wy 16:b_wz] 
y = nan(max(length(vicon_data),length(imu_data)),7);
imu = nan(max(length(vicon_data),length(imu_data)),6); % [ax ay az gx gy gz]
vicon = nan(max(length(vicon_data),length(imu_data)),7); % [pos_x pos_y pos_z q0 q1 q2 q3]
P = zeros(16,16,length(vicon_data)); %EKF covariance matrix 
cr = nan(7,7,length(vicon_data));
r = nan(max(length(vicon_data),length(imu_data)),7); 
time = zeros(1,length(dt)+1);
%assign initial conditions
time(1) = vicon_data(1,1); 
vicon(1,:) = vicon_data(1,2:8);
x_hat(1,:) = [vicon_data(1,2:4) 1  1.5 0 vicon_data(1,5:8) 0.00001 0.00001 0.00001 0.00001 0.00001 0.00001 ]; 

%EKF filter setup
P(:,:,1) = eye(16); %initial covariance matrix
Q = diag([   0.1^2      0.1^2      0.1^2   0.04^2  0.04^2  0.04^2   0.02^2 0.02^2 0.02^2 0.02^2   0.000^2     0.000^2     0.000^2    0.000^2    0.000^2     0.000^2]);
          %pos_x  pos_y  pos_z  vel_x  vel_y  vel_z  q0    q1    q2    q3  %b_ax  b_ay  b_az  b_wx  b_wy  b_wz
R = diag([  0.0001^2    0.0001^2    0.0001^2    0.00001^2     0.00001^2     0.00001^2    0.00001^2]); 
         %pos_x  pos_y  pos_z  q0    q1    q2    q3
         
%initialize counters
vicon_counter = 1; 
imu_counter = 1; 
ekf_counter = 1; 

for k = 1:max(length(vicon_data),length(imu_data))-1
    % Vicon measurment occurs 
    if time(k) >= vicon_data(vicon_counter,1)
        vicon(k+1,:) = vicon_data(vicon_counter,2:8); 
        vicon(k+1,:) = [vicon_data(vicon_counter,2),-vicon_data(vicon_counter,3:4),vicon_data(vicon_counter,5:6),-vicon_data(vicon_counter,7:8)];
        vicon_counter = vicon_counter + 1; 
    else 
        vicon(k+1,:) = vicon(k,:); 
    end
    
    %IMU measurement occurs
    if time(k) >= imu_data(imu_counter,1)
        imu(k+1,:) = imu_data(imu_counter,2:7);
        imu_counter = imu_counter + 1; 
    else 
        imu(k+1,:) = imu(k,:); 
    end
    
    y(k+1,:) = y(k,:); 
    
    %EKF update occurs -> here we assume that the Vicon and IMU data
    %are coming at a rate faster than the filter is being ran so an "update
    %step" occurs each time the EKF is called
    if time(k) >= ekf_time(ekf_counter)
        %EKF measurment model
        y(k+1,:) = vicon(k+1,:);
        % EKF Process Model 
        % integrate EKF equations using RK4
        xdot1 = derivative(x_hat(k,:), imu(k+1,:), time(k));
        xdot2 = derivative(x_hat(k,:) + xdot1 * (1/ekf_freq) / 2, imu(k+1,:), time(k) + (1/ekf_freq) / 2);
        xdot3 = derivative(x_hat(k,:) + xdot2 * (1/ekf_freq) / 2, imu(k+1,:), time(k) + (1/ekf_freq) / 2);
        xdot4 = derivative(x_hat(k,:) + xdot3 * (1/ekf_freq), imu(k+1,:), time(k) + (1/ekf_freq));
        totalxdot = (xdot1 + 2 * xdot2 + 2 * xdot3 + xdot4) / 6;
        x_hat(k+1,:) = x_hat(k,:) + totalxdot * (1/ekf_freq);
        % asign temp variables 
        q0 = x_hat(k+1,7); q1 = x_hat(k+1,8); q2 = x_hat(k+1,9); q3 = x_hat(k+1,10); 
        vx = x_hat(k+1,4); vy = x_hat(k+1,5); vz = x_hat(k+1,6); 
        b_x = x_hat(k+1,11); b_y = x_hat(k+1,12); b_z = x_hat(k+1,13);
        b_wx = x_hat(k+1,14); b_wy = x_hat(k+1,15); b_wz = x_hat(k+1,16);
        %imu estimates [xyz_accel pqr]
        wx = imu(k+1,4); wy = imu(k+1,5); wz = imu(k+1,6); 
        ax = imu(k+1,1); ay = imu(k+1,2); az = imu(k+1,3); 
        % calculate A matrix 
        A = [   0   0   0   1   0   0   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   1   0   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   0   1   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   0   0   -2*q3*(ay-b_y)+2*q2*(az-b_z)    2*q2*(ay-b_y)+2*q3*(az-b_z)   -4*q2*(ax-b_x)+2*q1*(ay-b_y)+2*q0*(az-b_z)   -4*q3*(ax-b_x)-2*q0*(ay-b_y)+2*q1*(az-b_z)  -1+2*(q2^2 + q3^2)   -2*(q1*q2 - q0*q3)   -2*(q1*q3 + q0*q2)    0    0    0;
                0   0   0   0   0   0    2*q3*(ax-b_x)-2*q1*(az-b_z)    2*q2*(ax-b_x)-4*q1*(ay-b_y)-2*q0*(az-b_z)    2*q1*(ax-b_x)+2*q3*(az-b_z)    2*q0*(ax-b_x)-4*q3*(ay-b_y)+2*q2*(az-b_z)  -2*(q1*q2 + q0*q3)   -1+2*(q1^2 + q3^2)   -2*(q2*q3 - q0*q1)    0    0    0; 
                0   0   0   0   0   0   -2*q2*(ax-b_x)-2*q1*(ay-b_y)    2*q3*(ax-b_x)-2*q0*(ay-b_y)-4*q1*(az-b_z)   -2*q0*(ax-b_x)+2*q3*(ay-b_y)-4*q2*(az-b_z)     2*q1*(ax-b_x)+2*q2*(ay-b_y) -2*(q1*q3 - q0*q2)   -2*(q2*q3 - q0*q1)   -1+2*(q1^2 + q2^2)    0    0    0; 
                0   0   0   0   0   0   0   0.5*(-wx+b_wx)    0.5*(-wy+b_wy)    0.5*(-wz+b_wz)   0   0   0   0.5*q1   0.5*q2   0.5*q3; 
                0   0   0   0   0   0   0.5*(wx-b_wx)   0   0.5*(wz-b_wz)    0.5*(-wy+b_wy)      0   0   0  -0.5*q0   0.5*q3  -0.5*q2; 
                0   0   0   0   0   0   0.5*(wy-b_wy)   0.5*(-wz+b_wz)   0   0.5*(wx-b_wx)       0   0   0  -0.5*q3  -0.5*q0   0.5*q1; 
                0   0   0   0   0   0   0.5*(wz-b_wz)   0.5*(wy-b_wy)    0.5*(-wx+b_wx)     0    0   0   0   0.5*q2  -0.5*q1  -0.5*q0; 
                0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0; 
                0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0]; 
 
        % integrate covariance matrix P using RK4
        Pdot1 = A*P(:,:,k) + P(:,:,k)*A'  + Q;
        Pdot2 = A*(P(:,:,k) + Pdot1 * (1/ekf_freq) / 2) + (P(:,:,k) + Pdot1 * (1/ekf_freq) / 2)*A'  + Q;
        Pdot3 = A*(P(:,:,k) + Pdot2 * (1/ekf_freq) / 2) + (P(:,:,k) + Pdot2 * (1/ekf_freq) / 2)*A'  + Q;
        Pdot4 = A*(P(:,:,k) + Pdot3 * (1/ekf_freq)) + (P(:,:,k) + Pdot3 * (1/ekf_freq))*A'  + Q;
        totalPdot = (Pdot1 + 2 * Pdot2 + 2 * Pdot3 + Pdot4) / 6;
        P(:,:,k+1) = P(:, :, k) + totalPdot * (1/ekf_freq);
        % EKF measurment model 
        % if we get a sensor measurement update the EKF
        % calculate C matrix 
        C = [eye(3) zeros(3,13)
             zeros(4,6) eye(4) zeros(4,6)]; 
        %make sensor estimate of measurement using x_hat 
        h_hat = C*x_hat(k+1,:)'; 
        cr(:,:,k+1) = C*P(:,:,k+1)*C' + R;
        r(k+1,:) = (y(k+1,:)' - h_hat)'; 
        % update hybrid EKF
        K = P(:,:,k+1)*C'*inv(C*P(:,:,k+1)*C' + R); 
        x_hat(k+1,:) = x_hat(k+1,:)' + K*(y(k+1,:)' - h_hat);
        P(:,:,k+1) = (eye(16)-K*C)*P(:,:,k+1);
        
        ekf_counter = ekf_counter + 1; 
    else 
        % no EKF update
        x_hat(k+1,:) = x_hat(k,:);  
        P(:,:,k+1) = P(:,:,k); 
        cr(:,:,k+1) = cr(:,:,k);
        r(k+1,:) = r(k,:); 
    end

    time(k+1) = time(k) + dt(k); % update time 
end

%% PLOT

figure(1) 
subplot(3,2,1)
plot(time, vicon(:,1), time, x_hat(:,1))
ylabel('$N_{pos}$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(3,2,3)
plot(time, vicon(:,2), time, x_hat(:,2))
ylabel('$E_{pos}$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(3,2,5)
plot(time, vicon(:,3), time, x_hat(:,3))
ylabel('$D_{pos}$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(3,2,2)
plot(time, x_hat(:,4)); 
ylabel('$N_{vel}$','interpreter','latex')
xlabel('time (s)')
legend('$\hat{x}$','interpreter','latex')
grid
subplot(3,2,4)
plot(time, x_hat(:,5)); 
ylabel('$E_{vel}$','interpreter','latex')
xlabel('time (s)')
legend('$\hat{x}$','interpreter','latex')
grid
subplot(3,2,6)
plot(time, x_hat(:,6)); 
ylabel('$D_{vel}$','interpreter','latex')
xlabel('time (s)')
legend('$\hat{x}$','interpreter','latex')
grid 

figure(2) 
subplot(4,1,1)
plot(time, vicon(:,4), time, x_hat(:,7)); 
ylabel('$q_0$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(4,1,2)
plot(time, vicon(:,5), time, x_hat(:,8))
ylabel('$q_1$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(4,1,3)
plot(time, vicon(:,6),time, x_hat(:,9)); 
ylabel('$q_2$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(4,1,4)
plot(time, vicon(:,7), time, x_hat(:,10)); 
ylabel('$q_3$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid

figure(3) 
plot(time, x_hat(:,11:16))
legend('b_{a_x}','b_{a_y}','b_{a_z}','b_{w_x}','b_{w_y}','b_{w_z}')
xlabel('time (s)')
grid

figure(4) 
subplot(3,1,1)
plot(time,r(:,1),'k',time,2*sqrt(squeeze(cr(1,1,:))),'k--',time,-2*sqrt(squeeze(cr(1,1,:))),'k--');
xlabel('time (s)');
ylabel('residual $N_{pos}$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.05,0.05])
grid
subplot(3,1,2)
plot(time,r(:,2),'k',time,2*sqrt(squeeze(cr(2,2,:))),'k--',time,-2*sqrt(squeeze(cr(2,2,:))),'k--');
xlabel('time (s)');
ylabel('residual $E_{pos}$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.05,0.05])
grid
subplot(3,1,3)
plot(time,r(:,3),'k',time,2*sqrt(squeeze(cr(3,3,:))),'k--',time,-2*sqrt(squeeze(cr(3,3,:))),'k--');
xlabel('time (s)');
ylabel('residual $D_{pos}$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.05,0.05])
grid

figure(5)
subplot(2,2,1)
plot(time,r(:,4),'k',time,2*sqrt(squeeze(cr(4,4,:))),'k--',time,-2*sqrt(squeeze(cr(4,4,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_0$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.02,0.02])
grid
subplot(2,2,2)
plot(time,r(:,5),'k',time,2*sqrt(squeeze(cr(5,5,:))),'k--',time,-2*sqrt(squeeze(cr(5,5,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_1$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.02,0.02])
grid
subplot(2,2,3)
plot(time,r(:,6),'k',time,2*sqrt(squeeze(cr(6,6,:))),'k--',time,-2*sqrt(squeeze(cr(6,6,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_2$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.02,0.02])
grid
subplot(2,2,4)
plot(time,r(:,7),'k',time,2*sqrt(squeeze(cr(7,7,:))),'k--',time,-2*sqrt(squeeze(cr(7,7,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_3$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.02,0.02])
grid



