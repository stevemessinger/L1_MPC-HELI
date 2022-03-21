function [xhat, P] = foo(x0,dt,y,imu)

    %initialize EKF
    EKF_C = EKF(x0);
    
    EKF_C.calc_estimate(y, imu, dt); 
    %save some variables for plotting
    cr = EKF_C.cr;
    r = EKF_C.r; 
    xhat = EKF_C.x_hat;
    P = EKF_C.P;

end

