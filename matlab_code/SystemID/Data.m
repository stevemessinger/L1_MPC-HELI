
close all
clc 

load('FT2.mat')

figure('name', 'angular response')
subplot(3,1,1)
plot(FT2IMU{:,13},FT2IMU{:,5})
hold on 
plot(FT2tar{:,2},FT2tar{:,3}*348)
% xlim([120,180])
ylabel('\phi')
subplot(3,1,2)
plot(FT2IMU{:,13},FT2IMU{:,6})
hold on 
plot(FT2tar{:,2},FT2tar{:,4}*-348)
% xlim([120,180])
ylabel('\theta')
subplot(3,1,3)
plot(FT2IMU{:,13},FT2IMU{:,7})
hold on 
plot(FT2tar{:,2},FT2tar{:,6}*-600)
% xlim([120,180])
ylabel('\psi')

for i = 1:length(FT2tar{:,5})
    if 116.4 < FT2tar{i,2} && FT2tar{i,2} < 117.3 || 120.3 < FT2tar{i,2} && FT2tar{i,2} < 121.5 || 125.76 < FT2tar{i,2} && FT2tar{i,2} < 126.6 || 130.3 < FT2tar{i,2} && FT2tar{i,2} < 131.08
        u(i) = -(FT2tar{i,5} + 0.75);
    else 
        u(i) = (FT2tar{i,5} + 0.75);
    end
end

figure('name','linear z accel')
plot(FT2IMU{800:end-1,13},cumtrapz(diff(FT2IMU{800:end,13})',FT2IMU{800:end-1,4}'))
hold on 
% plot(FT2tar{:,2},(0.75 + FT2tar{:,5})*-80)
% plot(FT2tar{1800:end,2},u(1800:end)*-80)
ylabel('z')
% xlim([110,140])

