function [u, a, b, u2, a2, b2] = foo(x, u_mpc, dt, adaptiveGain, cutoffFrequency)
    L1_cont = L1_Controller(adaptiveGain, cutoffFrequency);
    
    L1_cont.updateController(x, u_mpc, dt);

    u = L1_cont.u;
    a = L1_cont.z_hat;
    b = L1_cont.w_co;
    
    L1_cont2 = L1_Controller(adaptiveGain, cutoffFrequency);

    L1_cont2.updateController(x, u_mpc, dt);
    
    u2 = L1_cont2.u;
    a2 = L1_cont2.z_hat;
    b2 = L1_cont2.w_co;

end