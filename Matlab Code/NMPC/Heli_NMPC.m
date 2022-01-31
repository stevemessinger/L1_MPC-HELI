%{
ACADO Toolkit function for generating stand-alone NMPC for helictoper
control

%}

clear;

BEGIN_ACADO;
    
    
    %% set the problem name information
    acadoSet('problemname', 'Heli_NMPC'); 

    DifferentialState x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15 x16 x17 x18;
    Control u1 u2 u3 u4 u5 u6;
    Disturbance R;
    

    %% assign the ODE function for the optimizer
    f = acado.DifferentialEquation();
    f.linkMatlabODE('helicopter')

    %% OCP Setup
    startTime = 0;
    endTime = 1;
    frequency = 1000;
    
    ocp = acado.OCP(startTime, endTime, endTime*frequency);

    % we will use LSQ cost function
    h = {x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15, x16, x17, x18};

    % define the weighting matrix
    Q = eye(18);
    
    % reference to track
    r = zeros(1,18);
    r(1,7) = 1;

    % minimize wrt to LSQ
    ocp.minimizeLSQ(Q, h, r);

    % define constraints
    ocp.subjectTo(f); % optimize subject to the dynamics (at least)
    
    ocp.subjectTo(0 <= u1 <= 1); % input constraints
    ocp.subjectTo(0 <= u1 <= 1);
    ocp.subjectTo(0 <= u1 <= 1);
    ocp.subjectTo(0 <= u1 <= 1);
    ocp.subjectTo(0 <= u1 <= 1);
    ocp.subjectTo(0 <= u1 <= 1);
    
    ocp.subjectTo(R == 0.0); % disturbance constraint
    
    % state IC's
    ocp.subjectTo( 'AT_START', x1 == 1);
    ocp.subjectTo( 'AT_START', x2 == 1);
    ocp.subjectTo( 'AT_START', x3 == 1);
    ocp.subjectTo( 'AT_START', x4 == 0);
    ocp.subjectTo( 'AT_START', x5 == 0);
    ocp.subjectTo( 'AT_START', x6 == 0);
    ocp.subjectTo( 'AT_START', x7 == 1);
    ocp.subjectTo( 'AT_START', x8 == 0);
    ocp.subjectTo( 'AT_START', x9 == 0);
    ocp.subjectTo( 'AT_START', x10 == 0);
    ocp.subjectTo( 'AT_START', x11 == 0);
    ocp.subjectTo( 'AT_START', x12 == 0);
    ocp.subjectTo( 'AT_START', x13 == 0);
    ocp.subjectTo( 'AT_START', x14 == 0);
    ocp.subjectTo( 'AT_START', x15 == 0);
    ocp.subjectTo( 'AT_START', x16 == 167);
    ocp.subjectTo( 'AT_START', x17 == 0);
    ocp.subjectTo( 'AT_START', x18 == 0);
    

    
    %% Optimiziation Algorithm
    algo = acado.OptimizationAlgorithm(ocp);
    
    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-6);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-4 );

    algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );  % Example setting hessian approximation
    %algo.set( 'HESSIAN_APPROXIMATION', 'CONSTANT_HESSIAN' );  % Other possible settings
    %algo.set( 'HESSIAN_APPROXIMATION', 'FULL_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'BLOCK_BFGS_UPDATE' );
    %algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON_WITH_BLOCK_BFGS' );

END_ACADO;

%% We run the problem to check results
out = Heli_NMPC_RUN();