%{
ACADO Toolkit function for generating stand-alone NMPC for helictoper
control

%}

clear;

BEGIN ACADO;

acadoSet('problemname', 'Heli-NMPC'); 

% setup problem variables
DifferentialState ;
Control ;
Parameter T;

% setup inputs to MPC function
input1 = acado.MexInput; % initial Time
input2 = acado.MexInputMatrix; % initial State
input3 = acado.MexInputMatrix; % initial Control
input4 = acado.MexInput; % acado.OCP # of intervals

% setup Differential equations


% setup MPC problem
algo = acado.OptimizationAlgorithm(ocp);
algo.set('KKT_TOLERANCE', 1e-4); 
algo.initializeControls(input2);

END ACADO;