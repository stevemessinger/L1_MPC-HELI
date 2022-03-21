%{
C++ version for codegen 

Extended Kalman Filter design using Vicon motion capture and IMU data

Steve Messinger 
Sources: Optimal State Estimation -Dan Simon 

Important Note: 
    1. This assumes measurment model and process model are running at the
    same frequency (this can be changed fairly easily) 
    2. THE IMU DATA OFF THE BNO55 IS IN DEG/S!!! (this is accounted for in
    'derivative.m')

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
load('circle.mat') %load our circle vicon and IMU data set

%% SETUP 

% choose which data to use 
run_case = 'circle'; % 'mit' runs mit blackbird test data, 'circle' runs our custom data

%assign data 
switch run_case
    case 'mit'
        vicon_data = [mitviconquad(:,1)*(1e-9) mitviconquad(:,5:11)]; % [time xyz_pos q1 q2 q3 q0]
        imu_data = [mitimuquad1(:,1)*(1e-9) mitimuquad1(:,5:7) mitimuquad1(:,2:4)]; % [time xyz_accel pqr]
    case 'circle'
        vicon_data = circlevicon; % [time xyz_pos q1 q2 q3 q0]
        imu_data = circleIMU; % [time xyz_accel pqr]
        imu_data(:,1) = imu_data(:,1).*(1e-9); 
        vicon_data(:,1) = vicon_data(:,1).*(1e-9);
end

%EKF filter operating specifications
ekf_freq = 200; 

% hardware specifications 
imu_freq = 1/(mean(imu_data(2:end,1) - imu_data(1:end-1,1))); % update frequency of imu (Hz)
vicon_freq = 1/(mean(vicon_data(2:end,1) - vicon_data(1:end-1,1))); % update frequency of vicon pose data (Hz)

% initialize time to zero
vicon_data(:,1) = vicon_data(:,1) - vicon_data(1,1);  
imu_data(:,1) = imu_data(:,1) - imu_data(1,1); 

% setup simulation
ekf_time = vicon_data(1,1):1/ekf_freq:length(vicon_data)*(1/ekf_freq);
[max_freq,I] = max([ekf_freq,imu_freq,vicon_freq]);
switch I
    case 1
        dt = (1/max_freq)*ones(max(length(vicon_data),length(imu_data)),1); 
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
time = zeros(1,max(length(vicon_data),length(imu_data)));
%assign initial conditions
time(1) = vicon_data(1,1); 
vicon(1,:) = vicon_data(1,2:8);
x_hat(1,:) = [vicon_data(1,2:4) 1  -1.5 0 vicon_data(1,8) vicon_data(1,5:7) 0.00001 0.00001 0.00001 0.00001 0.00001 0.00001 ]; 

%initialize counters
vicon_counter = 1; 
imu_counter = 1; 
ekf_counter = 1; 

%initialize EKF
EKF_C = EKF(x_hat(1,:)'); 

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
        y(k+1,:) = [vicon(k+1,1:3) vicon(k+1,7) vicon(k+1,4:6)]; % switch quaterion from VICON in measurment
        %calculate state estimate 
        EKF_C.calc_estimate(y(k+1,:)', imu(k+1,:)', 1/ekf_freq); 
        %save some variables for plotting
        cr(:,:,k+1) = EKF_C.cr;
        r(k+1,:) = EKF_C.r'; 
        x_hat(k+1,:) = EKF_C.x_hat';
        P(:,:,k+1) = EKF_C.P;
        
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

figure(111) 
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

figure(211) 
subplot(4,1,1)
plot(time, vicon(:,7), time, x_hat(:,7)); 
ylabel('$q_0$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(4,1,2)
plot(time, vicon(:,4), time, x_hat(:,8))
ylabel('$q_1$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(4,1,3)
plot(time, vicon(:,5),time, x_hat(:,9)); 
ylabel('$q_2$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid
subplot(4,1,4)
plot(time, vicon(:,6), time, x_hat(:,10)); 
ylabel('$q_3$','interpreter','latex')
xlabel('time (s)')
legend('vicon','$\hat{x}$','interpreter','latex')
grid

figure(311) 
plot(time, x_hat(:,11:16))
legend('b_{a_x}','b_{a_y}','b_{a_z}','b_{w_x}','b_{w_y}','b_{w_z}')
xlabel('time (s)')
grid

figure(411) 
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

figure(511)
subplot(2,2,1)
plot(time,r(:,4),'k',time,2*sqrt(squeeze(cr(4,4,:))),'k--',time,-2*sqrt(squeeze(cr(4,4,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_0$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.1,0.1])
grid
subplot(2,2,2)
plot(time,r(:,5),'k',time,2*sqrt(squeeze(cr(5,5,:))),'k--',time,-2*sqrt(squeeze(cr(5,5,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_1$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.1,0.1])
grid
subplot(2,2,3)
plot(time,r(:,6),'k',time,2*sqrt(squeeze(cr(6,6,:))),'k--',time,-2*sqrt(squeeze(cr(6,6,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_2$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.1,0.1])
grid
subplot(2,2,4)
plot(time,r(:,7),'k',time,2*sqrt(squeeze(cr(7,7,:))),'k--',time,-2*sqrt(squeeze(cr(7,7,:))),'k--');
xlabel('time (s)');
ylabel('residual $q_3$', 'interpreter','latex');
legend('residual','+/-2sqrt(HPHt+R)');
ylim([-0.1,0.1])
grid



