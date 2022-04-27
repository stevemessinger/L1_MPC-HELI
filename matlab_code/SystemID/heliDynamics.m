function xdot = heliDynamics(state, u, t, params, dt)
    xdot = nan(1,13);
    
    x = state(1);
    y = state(2);
    z = state(3);
    qw = state(4);
    qx = state(5);
    qy = state(6);
    qz = state(7);
    xd = state(8);
    yd = state(9);
    zd = state(10);
    p = state(11);
    q = state(12);
    r = state(13);
    
    col = min(max(u(1),-1),1);
    roll = min(max(u(2),-1),1);
    pitch = min(max(u(3),-1),1);
    yaw = min(max(u(4),-1),1);
    
    

    
    Cbn = [(qw^2 + qx^2 - qy^2 - qz^2), 2*(qx*qy+qw*qz), 2*(qx*qz-qw*qy);
           2*(qx*qy-qw*qz), (qw^2 - qx^2 + qy^2 - qz^2), 2*(qy*qz+qw*qx);
           2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), (qw^2 - qx^2 - qy^2 + qz^2)]';
    
    xdot(1) = xd;
    xdot(2) = yd;
    xdot(3) = zd;
    xdot(4) = 0.5 * (-qx*p - qy*q -qz*r);
    xdot(5) = 0.5 * (qw*p + qy*r - qz*q);
    xdot(6) = 0.5 * (qw*q - qx*r + qz*p);
    xdot(7) = 0.5 * (qw*r + qx*q - qy*p);
    xdot(8) = Cbn(1,3) * params.Kcol * col;
    xdot(9) = Cbn(2,3) * params.Kcol * col;
    xdot(10) = Cbn(3,3) * params.Kcol * col + 9.80665;
    xdot(11) = -1/params.taup * p + params.Kp*roll;
    xdot(12) = -1/params.tauq * q + params.Kq*pitch;
    xdot(13) = -1/params.taur * r + params.Kr*yaw;

end