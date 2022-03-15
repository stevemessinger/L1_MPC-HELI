#include  "../include/mpc_Heli.h"

mpcController::mpcController(){
    verbose == false;

    poseSubscriber = nh.subscribe("pose", 500, &mpcController::poseCallback, this);
    inputPublisher = nh.advertise<mpc_node::Pose>("input", 500);

    ROS_INFO("mpc Controller Created!");
}

mpcController::mpcController(bool vrbs){
    verbose == vrbs;

    poseSubscriber = nh.subscribe("pose", 500, &mpcController::poseCallback, this);
    inputPublisher = nh.advertise<mpc_node::Pose>("input", 500);
    ROS_INFO("mpc Controller Created!");
}


mpcController::~mpcController(){

}

bool mpcController::init(){
    acado_initializeSolver();

    int    i, iter;
    acado_timer t;

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
    for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
    for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

    #if ACADO_INITIAL_STATE_FIXED
        for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.1;
    #endif

    if(verbose) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );
    
    ROS_INFO("Controller Initialized!");

    return true;
}

bool mpcController::loop(){

    // prepare the solver
    acado_preparationStep();

    // set the initial state
    acadoVariables.x0[0] = currentPose.x;
    acadoVariables.x0[1] = currentPose.y;
    acadoVariables.x0[2] = currentPose.z;
    acadoVariables.x0[3] = currentPose.qw;
    acadoVariables.x0[4] = currentPose.qx;
    acadoVariables.x0[5] = currentPose.qy;
    acadoVariables.x0[6] = currentPose.qz;
    acadoVariables.x0[7] = currentPose.vx;
    acadoVariables.x0[8] = currentPose.vy;
    acadoVariables.x0[9] = currentPose.vz;
    acadoVariables.x0[10] = currentPose.angx;
    acadoVariables.x0[11] = currentPose.angy;
    acadoVariables.x0[12] = currentPose.angz;


    // perform the feedback step
	status = acado_feedbackStep();

    if(verbose){
        std::cout << "Iteration #" << std::setw( 4 ) << 1
				 << ", KKT value: " << std::scientific << acado_getKKT()
				 << ", objective value: " << std::scientific << acado_getObjective()
				 << std::endl;
    }

    // get the inputs and publish them
    inputs.thrust = acadoVariables.u[0];
    inputs.roll = acadoVariables.u[1];
    inputs.pitch = acadoVariables.u[2];
    inputs.yaw = acadoVariables.u[3];
    inputPublisher.publish(inputs);

    return true;
}

void mpcController::poseCallback(const mpc_node::Pose::ConstPtr& poseMessage){
    currentPose = *poseMessage;
}