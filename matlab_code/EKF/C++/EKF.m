classdef EKF < handle 
    %EKF -> extended kalman filter class for codegen
    
    % public variables
    properties
        P = zeros(16); 
        Q = zeros(16); 
        R = zeros(7); 
        x_hat = zeros(16,1); 
        cr = zeros(7); 
        r = zeros(7,1);
    end
    % private variables 
    properties(Access = private)

    end
    
    methods
        function EKF = EKF(x0)
            %EKF Construct an instance of the EKF class
            EKF.P = eye(16); 
            EKF.Q = diag([   0.0^2      0.0^2      0.0^2   0.00036296  0.00036296  0.00036296   0.0141*(pi/180) 0.0141*(pi/180) 0.0141*(pi/180) 0.0141*(pi/180)   0.000^2     0.000^2     0.000^2    0.000^2    0.000^2     0.000^2]);
            %pos_x  pos_y  pos_z  vel_x  vel_y  vel_z  q0    q1    q2    q3  %b_ax  b_ay  b_az  b_wx  b_wy  b_wz
            EKF.R = diag([  0.0001^2    0.0001^2    0.0001^2    0.001^2     0.001^2     0.001^2    0.001^2]); 
            %pos_x  pos_y  pos_z  q0    q1    q2    q3
            EKF.x_hat = x0;
        end
        
        function calc_estimate(EKF,y,imu,dt)
            %calc_estimate -> updates EKF state estimate given y(sensor
            %measurement) and dt(EKF frequency)
            % EKF Process Model 
            % integrate EKF equations using RK4
            xdot1 = EKF.derivative(EKF.x_hat(:,1), imu(:,1));
            xdot2 = EKF.derivative(EKF.x_hat(:,1) + xdot1 * dt / 2, imu(:,1));
            xdot3 = EKF.derivative(EKF.x_hat(:,1) + xdot2 * dt / 2, imu(:,1));
            xdot4 = EKF.derivative(EKF.x_hat(:,1) + xdot3 * dt, imu(:,1));
            totalxdot = (xdot1 + 2 * xdot2 + 2 * xdot3 + xdot4) / 6;
            EKF.x_hat(:,1) = EKF.x_hat(:,1) + totalxdot * dt;
            % asign temp variables 
            q0 = EKF.x_hat(7,1); q1 = EKF.x_hat(8,1); q2 = EKF.x_hat(9,1); q3 = EKF.x_hat(10,1);
            b_x = EKF.x_hat(11,1); b_y = EKF.x_hat(12,1); b_z = EKF.x_hat(13,1);
            b_wx = EKF.x_hat(14,1); b_wy = EKF.x_hat(15,1); b_wz = EKF.x_hat(16,1);
            %imu estimates [xyz_accel pqr]
            wx = imu(4)*(pi/180); wy = -imu(5)*(pi/180); wz = -imu(6)*(pi/180); 
            ax = imu(1); ay = -imu(2); az = -imu(3); %% this depends how the IMU is mounted!
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
            Pdot1 = A*EKF.P + EKF.P*A'  + EKF.Q;
            Pdot2 = A*(EKF.P + Pdot1 * dt / 2) + (EKF.P + Pdot1 * dt / 2)*A'  + EKF.Q;
            Pdot3 = A*(EKF.P + Pdot2 * dt / 2) + (EKF.P + Pdot2 * dt / 2)*A'  + EKF.Q;
            Pdot4 = A*(EKF.P + Pdot3 * dt) + (EKF.P + Pdot3 * dt)*A'  + EKF.Q;
            totalPdot = (Pdot1 + 2 * Pdot2 + 2 * Pdot3 + Pdot4) / 6;
            EKF.P = EKF.P + totalPdot * dt;
            % EKF measurment model 
            % if we get a sensor measurement update the EKF
            % calculate C matrix 
            C = [eye(3) zeros(3,13)
                zeros(4,6) eye(4) zeros(4,6)]; 
            %make sensor estimate of measurement using x_hat 
            h_hat = C*EKF.x_hat(:,1); 
            EKF.cr = C*EKF.P*C' + EKF.R;
            EKF.r = (y(:,1) - h_hat); 
            % update hybrid EKF
            K = EKF.P*C'*inv(C*EKF.P*C' + EKF.R); 
            EKF.x_hat(:,1) = EKF.x_hat(:,1) + K*(y(:,1) - h_hat);
            EKF.P = (eye(16)-K*C)*EKF.P;
        end
        
        function xdot = derivative(EKF, x, imu)

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
            wx = imu(4)*(pi/180); wy = -imu(5)*(pi/180); wz = -imu(6)*(pi/180); 
            ax = imu(1); ay = -imu(2); az = -imu(3); %% this depends how the IMU is mounted!

            %DCM body to inertial frame 
            T_bi = [ 1-2*(q2^2 + q3^2) 2*(q1*q2 - q0*q3) 2*(q1*q3 + q0*q2);
                 2*(q1*q2 + q0*q3) 1-2*(q1^2 + q3^2) 2*(q2*q3 - q0*q1); 
                 2*(q1*q3 - q0*q2) 2*(q2*q3 - q0*q1) 1-2*(q1^2 + q2^2)];

            xdot(1:3) = [vx;vy;vz]; 
            xdot(4:6) = T_bi*([ax;ay;az] - [b_x;b_y;b_z]);% + [0;0;9.81]; % linear accelerations

            q_dot_mat = [    0           -(wx-b_wx)   -(wy-b_wy)    -(wz-b_wz);
                         (wx-b_wx)        0        (wz-b_wz)    -(wy-b_wy);
                         (wy-b_wy)   -(wz-b_wz)       0          (wx-b_wx);
                         (wz-b_wz)    (wy-b_wy)   -(wx-b_wx)         0];
            xdot(7:10) = 0.5*q_dot_mat*[q0;q1;q2;q3]; % quaternion dot equations 

            xdot(11:16) = 0; %bias terms -> derivative = 0 
        end
    end
end

