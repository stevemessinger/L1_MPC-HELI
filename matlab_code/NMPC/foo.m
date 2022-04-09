function u = foo(x, u_mpc, dt)
    L1_cont = L1_Controller(5, 15);

    L1_cont.updateController(x, u_mpc, dt);

    u = L1_cont.u;
end