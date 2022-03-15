#include  "../include/mpc_Heli.h"

mpcController::mpcController(){
    verbose == false;

    poseSubscriber = nh.subscribe("pose", 500, &mpcController::poseCallback, this);
    trajSubscriber = nh.subscribe("trajectory", 500, &mpcController::trajectoryCallback, this);
    inputPublisher = nh.advertise<mpc_node::Pose>("input", 500);

    ROS_INFO("mpc Controller Created!");
}

mpcController::mpcController(bool vrbs){
    verbose == vrbs;

    poseSubscriber = nh.subscribe("pose", 500, &mpcController::poseCallback, this);
    trajSubscriber = nh.subscribe("trajectory", 500, &mpcController::trajectoryCallback, this);
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

    // perform the feedback step
	auto status = acado_feedbackStep();

    // get the inputs and publish them
    inputs.thrust = acadoVariables.u[0];
    inputs.roll = acadoVariables.u[1];
    inputs.pitch = acadoVariables.u[2];
    inputs.yaw = acadoVariables.u[3];
    inputPublisher.publish(inputs);

    return status;
}

void mpcController::poseCallback(const mpc_node::Pose::ConstPtr& poseMessage){
    // set the current pose after receiving pose update
    acadoVariables.x0[0] = poseMessage->x;
    acadoVariables.x0[1] = poseMessage->y;
    acadoVariables.x0[2] = poseMessage->z;
    acadoVariables.x0[3] = poseMessage->qw;
    acadoVariables.x0[4] = poseMessage->qx;
    acadoVariables.x0[5] = poseMessage->qy;
    acadoVariables.x0[6] = poseMessage->qz;
    acadoVariables.x0[7] = poseMessage->vx;
    acadoVariables.x0[8] = poseMessage->vy;
    acadoVariables.x0[9] = poseMessage->vz;
    acadoVariables.x0[10] = poseMessage->angx;
    acadoVariables.x0[11] = poseMessage->angy;
    acadoVariables.x0[12] = poseMessage->angz;
}

void mpcController::trajectoryCallback(const mpc_node::Trajectory::ConstPtr& trajectoryMessage){
    //set the current trajectory upon receiving new trajectory
    for(int i = 0; i < N; i++){
        acadoVariables.y[i*NY + 0] = trajectoryMessage->x[i];
        acadoVariables.y[i*NY + 1] = trajectoryMessage->y[i];
        acadoVariables.y[i*NY + 2] = trajectoryMessage->z[i];
        acadoVariables.y[i*NY + 3] = trajectoryMessage->qw[i];
        acadoVariables.y[i*NY + 4] = trajectoryMessage->qx[i];
        acadoVariables.y[i*NY + 5] = trajectoryMessage->qy[i];
        acadoVariables.y[i*NY + 6] = trajectoryMessage->qz[i];
        acadoVariables.y[i*NY + 7] = trajectoryMessage->vx[i];
        acadoVariables.y[i*NY + 8] = trajectoryMessage->vy[i];
        acadoVariables.y[i*NY + 9] = trajectoryMessage->vz[i];
        acadoVariables.y[i*NY + 10] = trajectoryMessage->angx[i];
        acadoVariables.y[i*NY + 11] = trajectoryMessage->angy[i];
        acadoVariables.y[i*NY + 12] = trajectoryMessage->angz[i];
    }
}
