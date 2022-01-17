function xdot=helicopter(x,u,t,heli)
%{
dynamics of a micro helicopter 

inputs:
x=[pos_n pos_e pos_d u   v   w   q0   q1   q2   q3   p    q    r    b_1  
   (1)   (2)   (3)   (4) (5) (6) (7)  (8)  (9) (10) (11) (12) (13) (14)
   a_1  omega_mr  psi_mr]'
  (15)  (16)      (17)


Output
x=[pos_n_dot pos_e_dot pos_d_dot vel_n_dot vel_e_dot vel_d_dot]'

Here t is unused.

note that the helicopter blades rotate clockwise when viewed from above

%} 
xdot=nan(6,1); % allocate an array for xdot

u_wind = 0; 
v_wind = 0; 
w_wind = 0; 

% organize needed parameters 
g = 9.80665; % gravitational constant  
rho = 1.225; % air density @ SL (kg/m^3)
m = heli.m; % mass(kg) 
I_xx = heli.I_xx; % rolling moment of inertia (kg*m^2)
I_yy = heli.I_yy; % pitching moment of inertia (kg*m^2)
I_zz = heli.I_zz; % yawing moment of inertia (kg*m^2)
K_beta = heli.K_beta; % hub torsional stiffness (N*m/rad)
gamma_fb = heli.gamma_fb; % stabilizer bar lock number 
B_nom = heli.B_nom; % lateral cyclic to flap gain at nominal rpm (rad/rad)
A_nom = heli.A_nom; % longitudinal cyclic to flap gain at nominal rpm (rad/rad)
K_mu = heli.K_mu; % scaling of flap response to speed variation
omega_nom = heli.omega_nom; % nominal m.r. speed (rad/s)
R_mr = heli.R_mr; % m m.r. radius (m)
c_mr = heli.c_mr; % m.r. chord (m)
a_mr = heli.a_mr; % m.r. blade lift curve slope (rad^-1)
C_mr_D0 = heli.C_mr_D0; % m.r. blade zero lift drag coefficient
C_mr_T_max = heli.C_mr_T_max; % m.r. max thrust coefficient
I_beta_mr = heli.I_beta_mr; % m.r. blade flapping inertia (m^2)
R_tr = heli.R_tr; % t.r. radius (m)
c_tr = heli.c_tr; % t.r. chord (m)
a_tr = heli.a_tr; % t.r. blade lift curve slope (rad^-1)
C_tr_D0 = heli.C_tr_D0; % t.r. blade zero lift drag coefficient
C_tr_T_max = heli.C_tr_T_max; % t.r. max thrust coefficient
n_tr = heli.n_tr; % Gear ratio of t.r. to m. r.
n_es = heli.n_es; % Gear ratio of engine shaft to m. r.
delta_r_trim = heli.delta_r_trim; % t.r. pitch trim offset (rad)
S_vf = heli.S_vf; % Effective vertical fin area (m^2)
C_vf_La = heli.C_vf_La; % Vertical fin lift curve slope (rad^-1)
eps_vf_tr = heli.eps_vf_vf; % Fraction of vertical fin area exposed to t.r. induced velocity
S_ht = heli.S_ht; % Horizontal fin area (m^2)
C_ht_La = heli.C_ht_La; % Horizontal tail lift curve slope (rad^-1)
P_idle_eng = heli.P_idle_eng; % Engine idle power (W)
P_max_eng = heli.P_max_eng; % Engine max power (W)
K_p = heli.K_p; % Proportional governor gain (s/rad)
K_i = heli.K_i; % Integral governor gain (rad^-1)
S_fus_x = heli.S_fus_x; % Frontal fuselage drag area (m^2)
S_fus_y = heli.S_fus_y; % Side fuselage drag area (m^2)
h_mr = heli.h_mr; % m.r. hub height above c.g. (m) 
l_tr = heli.l_tr; % t.r. hub location behind c.g. (m)
h_tr = heli.h_tr; % t.r. height above c.g. (m)
l_ht = heli.l_ht; % Stabilizer location behind c.g. (m)
 
delta_col = u(1);
delta_lat = u(2); 
delta_lon = u(3); 
delta_r = u(4); 
delta_t = u(5); 
theta_0 = u(6);

a_1 = x(15); 
b_1 = x(14); 

% organize quaternions / euler rates
q0 = x(4); q1 = x(5); q2 = x(6); q3 = x(7);
p = x(11); q = x(12); r = x(13);
% organize linear velocities 
u = x(4); v = x(5); w = x(6); 
% organize main rotor angular position and velocity 
psi_mr = x(16); 
omega_mr = x(17); 

% DCM from body to NED
Cnb = [(q0^2+q1^2-q2^2-q3^2), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2);
        2*(q1*q2-q0*q3), (q0^2-q1^2+q2^2-q3^2), 2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), (q0^2-q1^2-q2^2+q3^2)];
    
% kinematics    
xdot(1:3) = Cnb'*x(8:10); % NED position 

% coriolis angular velocity term
omega = [0, -p, -q, -r;
         p, 0, r, -q;
         q, -r, 0, p;
         r, q, -p, 0];

euler = quat2eul([q0, q1, q2, q3]); %  [yaw, pitch, roll]

% nonlinear dynamics simulation
xdot(7:10) = 0.5*omega*x(4:7); % quaternion dot 


%Thrust 

% V_tip_mr = omega_mr*R_mr; % tip speed of the main rotor 
V_imr = sqrt((m*g)/(2*rho*pi*R_mr^2)); % m.r. induced velocity 
% lambda_i_mr = V_i_mr/V_tip_mr; % inflow ratio
% tau_lambda = 0.849/(4*lambda_trim*omega_mr); % time constant for settling of the inflow transients at hover
T_max = 20.5*m*g; %max thrust
n_w = 0.9; %account for nonideal wake contraction and the power lost due to the nonuniform velocity and pressure distribution in the wake
C_T_max = T_max/(rho*((omega_mr*R_mr)^2)*pi*R_mr^2); %thrust coefficient

% solve system of equations iteratively 
lambda_0 = 1; old_lambda = 0; f_j = 0.6; diff = 10; 
while diff > 0.001
    mu = sqrt(((u-u_wind)^2)+((v-v_wind)^2))/(omega_mr*R_mr); %advance ratio
    mu_z = (w-w_wind)/(omega_mr*R_mr); % z advance ratio
    sigma = (2*c_mr)/(pi*R_mr); 
    C_T_ideal = ((a_mr*sigma)/2)*(theta_0*((1/3)+((mu^2)/2))+((mu_z-lambda_0)/2));
    C_T = min(max(C_T_ideal,-C_T_max),C_T_max);
    % check for convergence 
    diff = lambda_0 - old_lambda;
    % store previous lambda value
    old_lambda = lambda_0; 
    % calculate new lambda value 
    G = (mu^2) + (lambda_0 - mu_z)^2; 
    h_j = -((2*n_w*lambda_0*sqrt(G) - C_T)*G)/(2*n_w*(G^(3/2)) + ((a_mr*sigma)/4)*G - C_T*(mu_z - lambda_0)); 
    lambda_0 = lambda_0 + f_j*h_j; 
end

Z_w = -((rho*omega_mr*R_mr*pi*(R_mr^2))/m)*((2*a_mr*sigma*lambda_0)/(16*lambda_0 + a_mr*sigma)); %vertical speed damping stability derivative
Z_col = -((rho*((omega_mr*R_mr)^2)*pi*(R_mr^2))/m)*(8/3)*((a_mr*sigma*lambda_0)/(16*lambda_0 + a_mr*sigma)); %collective pitch control derivative
a_z = Z_w*w + Z_col*delta_col; % z-accleration at hover 

% Torque 
C_Q = C_T*(lambda_0 - mu_z) + ((C_mr_D0*sigma)/(8))*(1 + ((7/3)*mu^2));%main rotor torque coefficient
Q_mr = C_Q*rho*((omega_mr*R_mr)^2)*pi*(R_mr^3); %yawing moment produced by the main rotor 

a_1_s = 0; 
b_1_s = 0; 

beta_mr = a_mr + a_1*cos(psi_mr) + b_1*sin(psi_mr); %main rotor flapping angle
beta_mr_s = a_1_s*sin(psi_mr) + b_1_s*cos(psi_mr); %flapping of the teetering stabilizer bar
% theta_mr = theta_0 + theta_lon*sin(psi_mr) + theta_lat*cos(psi_mr) + k_s*beta_s; %Stabilizer bar flapping contribution to the change of the main rotor blade pitch angle 

tau_e = 16/(gamma_fb*omega_mr); 
B_delta_lat = B_nom*((omega_mr/omega_nom)^2);
A_delta_lon = A_nom*((omega_mr/omega_nom)^2);
da_1_d_mu = 2*K_mu*(((4*delta_col)/3)-lambda_0); 
db_1_d_mu_v = -da_1_d_mu; 
da_1_d_mu_z = K_mu*((16*(mu^2))/((1-((mu^2)/2))*(8*abs(mu) + a_z*sigma))); 

%Main rotor moments and flapping dynamics 
xdot(14) = -p - (x(14)/tau_e) - (1/tau_e)*db_1_d_mu_v*((v-v_wind)/(omega_mr*R_mr)) + (B_delta_lat/tau_e)*delta_lat; 
xdot(15) = -q - (x(15)/tau_e) + (1/tau_e)*(da_1_d_mu*((u-u_wind)/(omega_mr*R_mr)) + da_1_d_mu_z*((w-w_wind)/(omega_mr*R_mr))) + (A_delta_lon/tau_e)*delta_lon;

T_mr = C_T*rho*((omega_mr*R_mr)^2)*pi*(R_mr^2); 

L_mr = (K_beta + T_mr*h_mr)*x(14); 
M_mr = (K_beta + T_mr*h_mr)*x(15); 

% rotor forces
X_mr = -T_mr*a_1; 
Y_mr = T_mr*b_1; 
Z_mr = -T_mr; 

%fuselage forces 
V_imr = 4.2; %this is only at hover -> needs to be solved for 
X_fus = S_fus_x*0.5*rho*(V_imr^2)*(u/V_imr);
Y_fus = S_fus_y*0.5*rho*(V_imr^2)*(v/V_imr);

% fuselage center of pressures wrt wind 
u_a = x(4) - u_wind;
v_a = x(5) - v_wind;
w_a = x(6) - w_wind;

% Tail rotor
f_t = 1 - (3/4)*(S_vf/(4*pi*(R_tr^2)));
omega_tr = n_tr*omega_mr; 
v_tr = v_a - l_tr*r + h_tr*p; 
mu_z_tr = v_tr/(omega_tr*R_tr); 
C_tr_T_del_r = 0.1; %%% THIS NEEDS TO CHANGE
C_tr_T_mu_z = 0.1; %%% THIS NEEDS TO CHANGE
Y_tr_del_r = -C_tr_T_del_r*((f_t*rho*((omega_tr*R_tr)^2)*pi*(R_tr^2))/(m)); 
Y_tr_v = -C_tr_T_mu_z*((f_t*rho*omega_tr*R_tr*pi*(R_tr^2))/(m)); 

Y_tr_max = f_t*C_tr_T_max*rho*((omega_tr*R_tr)^2)*pi*(R_tr^2); 
Y_tr = m*Y_tr_del_r*delta_r + m*Y_tr_v*mu_z_tr*omega_tr*R_tr; 
Y_tr = min(max(Y_tr,-Y_tr_max),Y_tr_max);
N_tr = -Y_tr*l_tr; 
L_tr = Y_tr*h_tr; 

g_i = (l_tr - R_mr - R_tr)/(h_tr); 
g_f = (l_tr - R_mr + R_tr)/(h_tr); 

if V_imr <= w_a || ((u_a)/(V_imr - w_a)) <= g_i
    K_lambda = 0; 
elseif ((u_a)/(V_imr - w_a)) >= g_f
    K_lambda = 1.5; 
else
    K_lambda = 1.5*((((u_a)/(V_imr - w_a))- g_i)/(g_f - g_i)); 
end 

% engine , govenor, rotorspeed model 
I_rot = 2.5*I_beta_mr; 
delta_t = min(max(delta_t,0),1);
P_e = P_max_eng*delta_t; 
Q_e = P_e/omega_mr; 
Q_mr = C_Q*rho*((omega_tr*R_tr)^2)*pi*(R_tr^3); %yawing moment produced by the tail
Q_tr = C_Q*rho*((omega_tr*R_tr)^2)*pi*(R_tr^3); %yawing moment produced by the tail
xdot(16) = xdot(13) + (1/I_rot)*(Q_e - Q_mr - n_tr*Q_tr); 

V_itr = sqrt((m*g)/(2*rho*pi*R_tr^2)); %this is only at hover -> needs to be solved for!!
%vertical fin forces and moments 
v_vf = v_a - eps_vf_tr*V_itr-l_tr*r; 
w_tr = w_a - l_tr*q - K_lambda*V_imr; 
V_inf_tr = sqrt(u_a*u_a + w_tr*w_tr); 
Y_vf = -0.5*rho*S_vf*(C_vf_La*V_inf_tr+norm(v_vf))*v_vf; 
N_vf = -Y_vf*l_tr; 
L_vf = Y_vf*h_tr; 

% Horizontal stabilizer forces and moments
w_ht = w_a + l_ht*q - K_lambda*V_imr; 
Z_ht = 0.5*rho*S_ht*(C_ht_La*norm(u_a)*w_ht + norm(w_ht)*w_ht); % NEED TO LIMIT THIS
Z_ht=min(max(Z_ht, -0.5*rho*S_ht*((u_a^2)+(w_ht^2))),0.5*rho*S_ht*((u_a^2)+(w_ht^2)));
M_ht = Z_ht*l_ht; 


% linear accelerations
xdot(4)= v*r - w*q - g*sin(euler(2)) + (X_mr + X_fus)/m; 
xdot(5)= w*p - u*r + g*sin(euler(3))*cos(euler(2)) + (Y_mr + Y_fus + Y_tr + Y_vf)/m; 
xdot(6)= u*q - v*p + g*cos(euler(3))*cos(euler(2)) + (Z_mr + Z_ht)/m;
% angular accelerations 
xdot(11)= q*r*(I_yy - I_zz)/I_xx + (L_vf + L_tr)/I_xx; 
xdot(12)= p*r*(I_zz - I_xx)/I_yy + (M_mr + M_ht)/I_yy; 
xdot(13)= p*q*(I_xx - I_yy)/I_zz + (-Q_e + N_vf*N_tr)/I_zz; 

xdot(16) = x(17); 
xdot(17) = x(16); 

end
