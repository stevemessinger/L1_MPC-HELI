function [u_L1,z_hat] = L1_Adaption(x, u_mpc, k)
%{
    L1 Adaption element for compensation of matched and unmatched
    uncertainties of vehicle 

    x - state vector 
    u_mpc - NMPC input 
    k - time step 
%}

g = [0;0;-9.81];
%************************************
%THIS GOES PRIOR TO SIMULATION 
%************************************
% adaptive element parameters
w_co = 15; %cut off frequency
A_s = eye(6); %adaption gains (Hurwitz)
tau_c = 0.05; %first order z thrust response 
tau_p = 0.05; %first order p response 
tau_q = 0.05; %first order q response 
tau_r = 0.05; %first order r response 
m = 1; 
K_col = (5*m)/tau_c; 
K_phi = (1000/tau_p)*(pi/180);
K_theta = (1000/tau_q)*(pi/180);
K_psi = (1000/tau_r)*(pi/180);

%initialize 
u_L1 = 0; %initialize adaptive element input 
z_hat = zeros(6,1); 
G = zeros(6,1); 

%************************************
%THIS GOES IN THE SIMULATION LOOP 
%************************************

%deconstruct state vector 
p_n = x(1); p_e = x(2); p_d = x(3);
q0 = x(4); q1 = x(5); q2 = x(6); q3 = x(7);
v_n = x(8); v_e = x(9); v_d = x(10);
p = x(11); q = x(12); r = x(13);
%L1-Adaptive Augmentation 
e_xb = [1-2*(q2^2+q3^2);2*(q1*q2+q0*q3);2*(q1*q3-q0*q2)]; 
e_yb = [2*(q1*q2-q0*q3);1-2*(q1^2+q3^2);2*(q2*q3-q0*q1)]; 
e_zb = [2*(q1*q3+q0*q2);2*(q2*q3-q0*q1);1-2*(q1^2+q2^2)]; 

R_bi = [e_xb, e_yb, e_zb]; %DCM from body to inertial 

eul = quat2eul([q0;q1;q2;q3]); 

z = [v_n v_e v_d p q r]; %state vector 


T_mpc = [0;0;(-1/tau_c)+K_col*u_mpc(1)];
M_mpc = [(-1/tau_p)*p+K_phi*eul(3);(-1/tau_q)*q+K_theta*eul(2);(-1/tau_r)*r+K_psi*eul(1)];

f = [T_mpc*e_zb;M_mpc]; %desired dynamics 
g = [e_zb e_zb e_zb e_zb;zeros(3,1) eye(3)]; %uncertainty in matched component 
g_T = [e_xb e_yb; zeros(3,2)]; %uncertainty in unmatched dynamics 

PHI = A_s\((e^(A_s*k)) - eye(6)); 
G = [g g_T]; 
mu = (e^(A_s*k))*(z_hat - z); 

sigma = -eye(6)*inv(G)*inv(PHI)*mu; %piecewise-constant adaptation law 
sigma_m = sigma(1:4)'; 
sigma_um = sigma(5:6)'; 

u_L1 = u_L1*e^(-w_co*k) - sigma_m*(1 - e^(-w_co*k)); 
z_hat = z_hat + (f + g*(u_L1 + sigma_m) + g_T*sigma_um + A_s*(z_hat - z))*k; 
end