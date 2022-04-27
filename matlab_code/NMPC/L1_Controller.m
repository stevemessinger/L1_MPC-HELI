classdef L1_Controller < handle
    properties
        u_L1= zeros(4,1);
        u = zeros(4,1);
        z_hat = zeros(6,1);
        % adaptive element parameters
        w_co = 0;
        A_s = zeros(6,6);

        tau_p = 0.036889; %first order p response 
        tau_q = 0.06539; %first order q response 
        tau_r = 0.083315; %first order r response 
        m = 1; 
        K_col = 0; 
        K_phi = 0;
        K_theta = 0;
        K_psi = 0;
    end

    properties(Access = private)

    end

    methods
        function L1_Controller = L1_Controller(adaptiveGain, cutOffFrequency)
            % adaptive element parameters
            L1_Controller.w_co = cutOffFrequency; %cut off frequency
            L1_Controller.A_s = -adaptiveGain*[1,0,0,0,0,0;
                                 0,1,0,0,0,0;
                                 0,0,1,0,0,0;
                                 0,0,0,1,0,0;
                                 0,0,0,0,1,0;
                                 0,0,0,0,0,1]; %adaption gains (Hurwitz)


            L1_Controller.K_col = 3.8239*9.80665; 
            L1_Controller.K_phi = (295.9283/L1_Controller.tau_p)*(pi/180);
            L1_Controller.K_theta = (299.1935/L1_Controller.tau_q)*(pi/180);
            L1_Controller.K_psi = (667.8047/L1_Controller.tau_r)*(pi/180);
        end


        function updateController(L1_Controller, x, u_mpc, dt)
            %deconstruct state vector 
            q0 = x(4); q1 = x(5); q2 = x(6); q3 = x(7);
            v_n = x(8); v_e = x(9); v_d = x(10);
            p = x(11); q = x(12); r = x(13);

            %L1-Adaptive Augmentation 
            R_bi = [(q0^2 + q1^2 - q2^2 - q3^2), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2);
                     2*(q1*q2-q0*q3), (q0^2 - q1^2 + q2^2 - q3^2), 2*(q2*q3+q0*q1);
                     2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), (q0^2 - q1^2 - q2^2 + q3^2)]';
        
            e_xb = R_bi(:,1); 
            e_yb = R_bi(:,2);
            e_zb = R_bi(:,3);
        
            z = [v_n v_e v_d p q r]'; %state vector 
        
            T_mpc = L1_Controller.K_col*u_mpc(1);
            M_mpc = [(-1/L1_Controller.tau_p)*p+L1_Controller.K_phi*u_mpc(2);(-1/L1_Controller.tau_q)*q+L1_Controller.K_theta*u_mpc(3);(-1/L1_Controller.tau_r)*r+L1_Controller.K_psi*u_mpc(4)];
        
            f = [[0;0;9.80665]+T_mpc.*e_zb;M_mpc]; %desired dynamics 
            
            g = [e_zb zeros(3,3);zeros(3,1) eye(3)]; %uncertainty in matched component 
            g_T = [e_xb e_yb; zeros(3,2)]; %uncertainty in unmatched dynamics 
        
            PHI = inv(L1_Controller.A_s)*(expm(L1_Controller.A_s*dt) - eye(6)); 
            mu = expm(L1_Controller.A_s*dt)*(L1_Controller.z_hat - z); 
            G = [g g_T];
    
            sigma = -eye(6)*inv(G)*inv(PHI)*mu; %piecewise-constant adaptation law 
            sigma_m = sigma(1:4); 
            sigma_um = sigma(5:6); 
            
            % LPF on the adaption
            L1_Controller.u_L1 = L1_Controller.u_L1*exp(-L1_Controller.w_co*dt) - sigma_m*(1 - exp(-L1_Controller.w_co*dt)); 
    
            L1_Controller.z_hat = L1_Controller.z_hat + (f + g*(L1_Controller.u_L1 + sigma_m) + g_T*sigma_um + L1_Controller.A_s*(L1_Controller.z_hat - z))*dt;
            
            L1_Controller.u = u_mpc + L1_Controller.u_L1./[L1_Controller.K_col;L1_Controller.K_phi;L1_Controller.K_theta;L1_Controller.K_psi];
        end
    end
end