%{
Extended Kalman Filter design using Vicon motion capture and IMU data

Steve Messinger 
Sources: Optimal State Estimation -Dan Simon 
%}

clear all 
close all 
clc

load('imuTest.mat') %load imu test data 
load('mit_imu.mat') %load MIT imu data
load('mit_vicon.mat') %load MIT imu data