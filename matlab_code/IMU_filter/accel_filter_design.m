%{
Design of acceleration filter from BNO55 IMU

%}
close all
clear all 
clc

% IMU data organized as follows 
%time	field.ax	field.ay	field.az	field.gx	field.gy	field.gz
%(1)       (2)         (3)         (4)         (5)         (6)         (7)

% MIT data organization 
%time	field.header.seq	field.header.stamp	field.header.frame_id	field.pose.position.x	field.pose.position.y	field.pose.position.z	
%field.pose.orientation.x	field.pose.orientation.y	field.pose.orientation.z	field.pose.orientation.w
load('data.mat')
load('imuTest.mat') %load imu test data 
load('mit_imu.mat') %load MIT imu data
load('mit_vicon.mat') %load MIT imu data

%% SETUP

run_case = 'test'; %change runcase to 'test', 'mit', 'data_#"
freq_analysis = 1; % perform fft of data (1) or not (0)
plot_filtered_data = 1; % plot filtered versus actual data (1) or not (0)

%% FILTER DESIGN 

if freq_analysis
    switch run_case
        case 'test'
            data = data1(50/0.01:end,:); 
        case 'mit'
            data = [mitimuquad1(:,1) mitimuquad1(:,5:7) mitimuquad1(:,2:4)];
    end
    data_avg = [movmean(data(:,1:4),50),movmean(data(:,5:7),50)];
    noise = data - data_avg;
    
    
    % PERFORM FFT
    Fs = 100; % sampling frequency in Hz
    T = 1/Fs; % sampling period
    L = length(noise); % length of signal 
    t = (0:L-1)*T; %time vector
    % Compute the two-sided spectrum P2
    Y = fft(noise(:,2:7)); 
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1,:);
    P1(2:end-1,:) = 2*P1(2:end-1,:);

    figure(1)
    hold on 
    f = Fs*(0:(L/2))/L;
    plot(f,P1(:,1:3)) 
    title('Single-Sided Amplitude Spectrum of accelerometer Data')
    xlabel('f (Hz)')
    ylabel('|P1(f)|')
    legend('$\ddot{x}$','$\ddot{y}$','$\ddot{z}$','interpreter','latex')

    figure(2)
    hold on 
    f = Fs*(0:(L/2))/L;
    plot(f,P1(:,4:6)) 
    title('Single-Sided Amplitude Spectrum of gyro Data')
    xlabel('f (Hz)')
    ylabel('|P1(f)|')
    legend('$p$','$q$','$r$','interpreter','latex')
    
    figure(3) 
    subplot(3,1,1)
    plot(t, data(:,2))
    hold on 
    plot(t, data_avg(:,2))
    ylabel('$\ddot{x}$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,2)
    plot(t, data(:,3))
    hold on 
    plot(t, data_avg(:,3))
    ylabel('$\ddot{y}$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,3)
    plot(t, data(:,4))
    hold on 
    plot(t, data_avg(:,4))
    ylabel('$\ddot{z}$','interpreter','latex')
    xlabel('time (s)')
    
    figure(4) 
    subplot(3,1,1)
    plot(t, data(:,5))
    hold on 
    plot(t, data_avg(:,5))
    ylabel('$p$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,2)
    plot(t, data(:,6))
    hold on 
    plot(t, data_avg(:,6))
    ylabel('$q$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,3)
    plot(t, data(:,7))
    hold on 
    plot(t, data_avg(:,7))
    ylabel('$r$','interpreter','latex')
    xlabel('time (s)')
end

%allocate space
filt_imu = zeros(size(data)); 

% Butterworth Filter Design 
% acceleromter filter
Rp = 3;
Rs = 60; 
Wp = [2]/(Fs/2); %pass band 0.075 10
Ws = [4]/(Fs/2);% [0.025, 15]
[n1,Wn1] = buttord(Wp,Ws,Rp,Rs);

[b,a] = butter(n1,Wn1);   % Get filter coeff of butter filter
filt_imu(:,1:4) = filtfilt(b,a, data(:,1:4)); % zero phase delay low-pass filtering


% gyro filter 
Rp = 1; 
Rs = 10; 
Wp = 20/(Fs/2); 
Ws = 25/(Fs/2);
[n2,Wn2] = buttord(Wp,Ws,Rp,Rs);

[b,a] = butter(n2,Wn2);   % Get filter coeff of butter filter
filt_imu(:,5:7) = filtfilt(b,a, data(:,5:7)); % zero phase delay low-pass filtering

%% Double integrate filtered IMU to position
dt = (data(2:end,1) - data(1:end-1,1))*10.^-9 ;

 x = nan(6, length(filt_imu));
 xdot = nan(6, length(filt_imu));
 nedIMU = nan(3, length(filt_imu));
 u = [filt_imu(:,2), filt_imu(:,3), filt_imu(:,4)] ;
 t = 0;
 dcm = quat2dcm([mitviconquad(:,11), mitviconquad(:,8:10)]);
 
 temp1 = dcm(:,:,1) * [mitviconquad(1,5); mitviconquad(1,6); mitviconquad(1,7)];
 temp2 = dcm(:,:,1) * [(mitviconquad(2,5)-mitviconquad(1,5))/(1/181); (mitviconquad(2,6)-mitviconquad(1,6))/(1/181); (mitviconquad(2,7)-mitviconquad(1,7))/(1/181)];  
 x(:,1) = [temp1(1);temp2(1); temp1(2); temp2(2); temp1(3); temp2(3)] ;

 nedIMU(1,1) = mitviconquad(1,5);
 i = 1;
for k = 1:length(filt_imu)-500
    
    [x(:,k+1), xdot(:,k)] = RK4_zoh('IMUint', x(:,k), u(k,:)', 0, 0, dt(k)); 
    
    nedIMU(:,k+1) = dcm(:,:,i)' * [x(1,k+1); x(3,k+1); x(5,k+1)];   
    if (mitviconquad(k,1)-mitviconquad(1,1))*10.^-9 <= t
       i = i+2;
    end
    t = t+dt(k);
end



figure('name', 'Prop filtered IMU data')
plot((data(1:end,1)-data(1,1))*10.^-9, nedIMU(1,:), (mitviconquad(:,1)-mitviconquad(1,1))*10.^-9, mitviconquad(:,5));
xlabel('time (sec)')
ylabel('position (m)')
legend('integrated', 'actual')

%% PLOT FILTER RESULTS
if plot_filtered_data
    figure(19) 
    subplot(3,1,1)
    plot(t, data(:,2))
    hold on 
    plot(t, filt_imu(:,2),'LineWidth',2)
    ylabel('$\ddot{x}$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,2)
    plot(t, data(:,3))
    hold on 
    plot(t, filt_imu(:,3),'LineWidth',2)
    ylabel('$\ddot{y}$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,3)
    plot(t, data(:,4))
    hold on 
    plot(t, filt_imu(:,4),'LineWidth',2)
    ylabel('$\ddot{z}$','interpreter','latex')
    xlabel('time (s)')

    figure(20) 
    subplot(3,1,1)
    plot(t, data(:,5))
    hold on 
    plot(t, filt_imu(:,5),'LineWidth',2)
    ylabel('$p$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,2)
    plot(t, data(:,6))
    hold on 
    plot(t, filt_imu(:,6),'LineWidth',2)
    ylabel('$q$','interpreter','latex')
    xlabel('time (s)')
    subplot(3,1,3)
    plot(t, data(:,7))
    hold on 
    plot(t, filt_imu(:,7),'LineWidth',2)
    ylabel('$r$','interpreter','latex')
    xlabel('time (s)')
end