%{
Proccess flight test data to model control system of MT OMP Hobby
Helicopter 

Parameter extraction from FT2
Model Validation on flipsvicon 

Data: 
    FT2IMU - step tests in all 6 DOF IMU data collected from BNO055
    FT2tar - taranis stick input data for step tests in all 6 DOF

    flipsvicon - VICON data for flips indoors 
%}
clear all
close all
clc 

%% IMPORT DATA 
% IMU data: [time(1) a_x(2) a_y(3) a_z(4) g_x(5) g_y(6) g_z(7)]   
% VICON data: [time(1) x_pos(2) y_pos(3) z_pos(4) q_x(5) q_y(6) q_z(7)
% q_w(8)] 
load('FT2.mat')
load('flips_vicon.mat')
load('angular_tf.mat')

%% ANGULAR System ID 
%*********************
%interpolate values to be used in system ID toolbox
% rt1 = interp1(FT2IMU{2:end,13}, FT2IMU{2:end,5}, FT2tar{2:end,2}); 
% pt1 = interp1(FT2IMU{2:end,13}, FT2IMU{2:end,6}, FT2tar{2:end,2});
% yt1 = interp1(FT2IMU{2:end,13}, FT2IMU{2:end,7}, FT2tar{2:end,2});

%print parameters
print_p = ['The roll parameters are: tau = ',num2str(1/roll_tf.Denominator(2)),', K_p = ', num2str(roll_tf.Numerator/roll_tf.Denominator(2))];
print_q = ['The pitch parameters are: tau = ',num2str(1/pitch_tf.Denominator(2)),', K_q = ', num2str(pitch_tf.Numerator/pitch_tf.Denominator(2))];
print_r = ['The yaw parameters are: tau = ',num2str(1/yaw_tf.Denominator(2)),', K_r = ', num2str(yaw_tf.Numerator/yaw_tf.Denominator(2))];
disp(print_p);
disp(print_q);
disp(print_r);

%% LINEAR SYSTEM ID
%********************
%interpolate values to be used in system ID toolbox
zt1 = interp1(FT2IMU{2:end,13}, FT2IMU{2:end,4}, FT2tar{2:end,2});

%% PLOTTING 
a = -1; 
i=1; 
input_z = zeros(1,length(FT2tar{2:end,2}));
while i < length(FT2tar{2:end,2})
    if FT2tar{1+i,5} > -0.74
        input_z(i) = 0; 
    else
        if abs(FT2tar{1+i,5}) > 0.99
            while abs(FT2tar{1+i,5}) > 0.99
                input_z(i) = a*(FT2tar{1+i,5}+0.75);
                i = i+1; 
            end
            a = -1*a; 
        end
        input_z(i) = a*(FT2tar{1+i,5}+0.75); 
    end
    i=i+1;
end

figure('name','z accleration')
subplot(2,1,1) 
plot(FT2tar{2:end,2},zt1,FT2tar{2:end,2},input_z*100)
subplot(2,1,2)
plot(FT2tar{2:end,2},FT2tar{2:end,5})