#include  "../include/mpc_Heli.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

mpcController::mpcController(){
    verbose == false;

    poseSubscriber = nh.subscribe("pose", 500, &mpcController::poseCallback, this);
    trajSubscriber = nh.subscribe("trajectory", 500, &mpcController::trajectoryCallback, this);
    inputPublisher = nh.advertise<heli_messages::Inputs>("mpc_input", 500);


    if(!this->init()){
        ROS_ERROR("mpc Controller Failed to Initialize!");
    }
    else{
        ROS_INFO("mpc Controller Created!");
    }
}


mpcController::mpcController(bool vrbs){
    verbose = vrbs;

    poseSubscriber = nh.subscribe("pose", 500, &mpcController::poseCallback, this);
    trajSubscriber = nh.subscribe("trajectory", 500, &mpcController::trajectoryCallback, this);
    inputPublisher = nh.advertise<heli_messages::Inputs>("mpc_input", 500);


    if(!this->init()){
        ROS_ERROR("mpc Controller Failed to Initialize!");
    }
    else{
        ROS_INFO("mpc Controller Created!");
    }
}

// Destructor
mpcController::~mpcController(){

}


// Initialize the controller
bool mpcController::init(){


    if(acado_initializeSolver()){
        ROS_ERROR("mpcController: Failed to initialize solver!");
        return false;
    };

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
    for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0; 
    for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

    #if ACADO_INITIAL_STATE_FIXED
        for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.0;
        acadoVariables.x0[ 3 ] = 1.0;
        acadoVariables.x0[ 4 ] = 0;
        acadoVariables.x0[ 5 ] = 0;
        acadoVariables.x0[ 6 ] = 0;
    #endif

    acado_printHeader();

	/* Prepare first step */
	if(acado_preparationStep()){
        ROS_ERROR("mpcController: Failed to prepare first step!");
        return false;
    }
    
    return true;
}


int mpcController::loop(){

    if(nav){
        /* Perform the feedback step. */
        //for (i = 0; i < NX; i++) acadoVariables.x0[i] = acadoVariables.x[NX + i]; //assume it moved that state
        status = acado_feedbackStep();
        
        //std::cout << acadoVariables.x0[ 0 ] << ":" << acadoVariables.y[0] <<std::endl;
        
        /* Apply the new control immediately to the process, first NU components. */
        
        //get the inputs and publish them
        inputs.col = acadoVariables.u[0];
        inputs.roll = acadoVariables.u[1];
        inputs.pitch = acadoVariables.u[2];
        inputs.yaw = acadoVariables.u[3];
        inputs.nav = nav;

        // need to add throttle curve and set kill switch
        // Shift states and control and prepare for the next iteration
        //acado_shiftStates(2, 0, 0);
        //acado_shiftControls( 0 );
        inputPublisher.publish(inputs);
        acado_preparationStep();
    }else{

        inputs.col = 0;
        inputs.roll = 0;
        inputs.pitch = 0;
        inputs.yaw = 0;
        inputs.nav = nav;
        inputPublisher.publish(inputs);
    }

    
    return status;
}

void mpcController::poseCallback(const heli_messages::Pose::ConstPtr& poseMessage){
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

void mpcController::trajectoryCallback(const heli_messages::Trajectory::ConstPtr& trajectoryMessage){
    //set the current trajectory upon receiving new trajectory
    int i;
    for(i=0; i < N; i++){
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

    acadoVariables.yN[0] = trajectoryMessage->x[i-1];
    acadoVariables.yN[1] = trajectoryMessage->y[i-1];
    acadoVariables.yN[2] = trajectoryMessage->z[i-1];
    acadoVariables.yN[3] = trajectoryMessage->qw[i-1];
    acadoVariables.yN[4] = trajectoryMessage->qx[i-1];
    acadoVariables.yN[5] = trajectoryMessage->qy[i-1];
    acadoVariables.yN[6] = trajectoryMessage->qz[i-1];
    acadoVariables.yN[7] = trajectoryMessage->vx[i-1];
    acadoVariables.yN[8] = trajectoryMessage->vy[i-1];
    acadoVariables.yN[9] = trajectoryMessage->vz[i-1];
    acadoVariables.yN[10] = trajectoryMessage->angx[i-1];
    acadoVariables.yN[11] = trajectoryMessage->angy[i-1];
    acadoVariables.yN[12] = trajectoryMessage->angz[i-1];

    nav = trajectoryMessage->nav;
}

