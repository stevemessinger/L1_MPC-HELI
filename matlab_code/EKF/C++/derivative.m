function xdot = derivative(x, imu, t)

    %EKF process model from IMU data 
    % state: [1:p_x 2:p_y 3:p_z 4:v_x 5:v_y 6:v_z 7:q0 8:q1 9:q2 10:q3 11:b_ax 12:b_ay 13:b_az 14:b_wx 15:b_wy 16:b_wz]
    % imu: [ax ay az wx wy wz]
    xdot = nan(16,1); %initialize derivative 
    
    %assign temp variables
    q0 = x(7); q1 = x(8); q2 = x(9); q3 = x(10); %quaternions 
    vx = x(4); vy = x(5); vz = x(6); %linear velocities
    b_x = x(11); b_y = x(12); b_z = x(13); %linear bias terms 
    b_wx = x(14); b_wy = x(15); b_wz = x(16); %angular bias terms 
    %imu estimates [xyz_accel pqr]
    wx = imu(4)*(pi/180); wy = imu(5)*(pi/180); wz = imu(6)*(pi/180); 
    ax = imu(1); ay = imu(2); az = imu(3); 
    
    %DCM body to inertial frame 
    T_bi = [ 1-2*(q2^2 + q3^2) 2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2);
             2*(q1*q2 + q0*q3) 1-2*(q1^2 + q3^2) 2*(q2*q3 - q0*q1); 
             2*(q1*q3 - q0*q2) 2*(q2*q3 - q0*q1) 1-2*(q1^2 + q2^2)];
    
    xdot(1:3) = [vx;vy;vz]; 
    xdot(4:6) = T_bi*([ax;ay;az] - [b_x;b_y;b_z]);% - [0;0;9.81]; % linear accelerations
    
    q_dot_mat = [    0           -(wx-b_wx)   -(wy-b_wy)    -(wz-b_wz);
                     (wx-b_wx)        0        (wz-b_wz)    -(wy-b_wy);
                     (wy-b_wy)   -(wz-b_wz)       0          (wx-b_wx);
                     (wz-b_wz)    (wy-b_wy)   -(wx-b_wx)         0];
    xdot(7:10) = 0.5*q_dot_mat*[q0;q1;q2;q3]; % quaternion dot equations 
    
    xdot(11:16) = 0; %bias terms -> derivative = 0 
end

