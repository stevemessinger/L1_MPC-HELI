function xdot = IMUint(x, u, t, params)
A = [0, 1;
     0, 0];
B = [0; 1];

xdot = A*x + B*u;
end