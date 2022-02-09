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

%% SETUP 

% choose which data to use 
run_case = 'mit'; % 'mit' runs mit blackbird test data

%assign data 
switch run_case
    case 'mit'
        vicon_data = [mitviconquad(:,1)*(1e-9) mitviconquad(:,5:7)]; % [time xyz_pos q1 q2 q3 q0]
        imu_data = [mitimuquad1(:,1)*(1e-9) mitimuquad1(:,5:7) mitimuquad1(:,2:4)]; % [time xyz_accel pqr]
end

%filter operating specifications
ekf_freq = 100; 

% hardware specifications 
imu_freq = 1/(imu_data(1001,1) - imu_data(1000,1));  %100; % update frequency of imu (Hz)
vicon_freq = 1/(vicon_data(1070,1) - vicon_data(1069,1)); %180; % update frequency of vicon pose data (Hz)

% NEED TO GIT RID OF THIS
vicon_data(:,1) = vicon_data(:,1) - vicon_data(1,1); %initialize time to zero 
imu_data(:,1) = imu_data(:,1) - imu_data(1,1); 

% setup simulation
[max_freq,I] = max([ekf_freq,imu_freq,vicon_freq]);
switch I
    case 1
        dt = 0:1/max_freq:max(length(vicon_data),length(imu_data))*(1/max_freq); 
    case 2
        dt = imu_data(2:end,1) - imu_data(1:end-1,1); 
    case 3
        dt = vicon_data(2:end,1) - vicon_data(1:end-1,1); 
end 
% allocate space for arrays
x_hat = nan(max(length(vicon_data),length(imu_data)),19); % states [p_x p_y p_z v_x v_y v_z q0 q1 q2 q3 w_x w_y w_z b_ax b_ay b_az b_wx b_wy b_wz] 
time = zeros(1,length(dt)+1); 


for k = 1:max(length(vicon_data),length(imu_data))-1

    
    if mod(time, mInterval) < dt
        % integrate EKF equations using RK4
        xdot1 = derivative(x_hat(:, k), y(:, mCntr - 1), u, time, gains, model);
        xdot2 = derivative(x_hat(:, k) + xdot1 * dt / 2, y(:, mCntr - 1), u, time + dt / 2, gains, model);
        xdot3 = derivative(x_hat(:, k) + xdot2 * dt / 2, y(:, mCntr - 1), u, time + dt / 2, gains, model);
        xdot4 = derivative(x_hat(:, k) + xdot3 * dt, y(:, mCntr - 1), u, time + dt, gains, model);
        totalxdot(:, k) = (xdot1 + 2 * xdot2 + 2 * xdot3 + xdot4) / 6;
        x_hat(:, k + 1) = x_hat(:, k) + totalxdot(:, k) * dt;

        % integrate covariance matrix P using RK4
        Pdot1 = A * P(:, :, k) * A' + G * Q * G';
        Pdot2 = A * (P(:, :, k) + Pdot1 * dt / 2) * A' + G * Q * G';
        Pdot3 = A * (P(:, :, k) + Pdot2 * dt / 2) * A' + G * Q * G';
        Pdot4 = A * (P(:, :, k) + Pdot3 * dt) * A' + G * Q * G';
        totalPdot = (Pdot1 + 2 * Pdot2 + 2 * Pdot3 + Pdot4) / 6;
        P(:, :, k + 1) = P(:, :, k) + totalPdot * dt;
    end

    time(k+1) = time(k) + dt; % update time 
end
