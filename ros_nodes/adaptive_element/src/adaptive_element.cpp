#include "../include/adaptive_element.h"

adaptiveElement::adaptiveElement(){

    poseSubscriber = nh.subscribe("pose",  500, &adaptiveElement::poseSubscriberCallback, this);
    MPCSubscriber = nh.subscribe("mpc_input", 500, &adaptiveElement::MPCSubscriberCallback, this);
    inputsPublisher = nh.advertise<mpc_node::Inputs>("L1_inputs", 500);

    z_State = Eigen::VectorXd(6);
    g = {0, 0, 9.81};
    g_matched = Eigen::MatrixXd(6,4);
    g_matched.setZero();
    g_matched(3,1) = 1;
    g_matched(4,2) = 1;
    g_matched(5,3) = 1;
    g_unmatched = Eigen::MatrixXd(6,2);
    g_unmatched.setZero();
    A_s = Eigen::MatrixXd(6,6);
    A_s.setIdentity();
    A_s = -5 * A_s;
    PHI = Eigen::MatrixXd(6,6);
    w_co = 15;
    previousTime = ros::Time::now().toSec();
}

adaptiveElement::~adaptiveElement(){

}

void adaptiveElement::loop(){

    // collect variables
    float x = currentPose.x;
    float y = currentPose.y;
    float z = currentPose.z;
    float q0 = currentPose.qw;
    float q1 = currentPose.qx;
    float q2 = currentPose.qy;
    float q3 = currentPose.qz;
    float vx = currentPose.vx;
    float vy = currentPose.vy;
    float vz = currentPose.vz;
    float angx = currentPose.angx;
    float angy = currentPose.angy;
    float angz = currentPose.angz;
    float dt = ros::Time::now().toSec() - previousTime;
    previousTime = ros::Time::now().toSec();

    // Calculate DCM
    Rbi(0,0) = (q0*q0 + q1*q1 - q2*q2 - q3*q3);
    Rbi(0,1) = 2*(q1*q2+q0*q3);
    Rbi(0,2) = 2*(q1*q3-q0*q2);
    Rbi(1,0) = 2*(q1*q2-q0*q3);
    Rbi(1,1) = (q0*q0 - q1*q1 + q2*q2 - q3*q3);
    Rbi(1,2) = 2*(q2*q3+q0*q1);
    Rbi(2,0) = 2*(q1*q3+q0*q2);
    Rbi(2,1) = 2*(q2*q3-q0*q1);
    Rbi(2,2) = (q0*q0 - q1*q1 - q2*q2 + q3*q3);
    Rbi.transposeInPlace();

    // Split DCM into important components
    e_xb = {Rbi(0,0), Rbi(1,0), Rbi(2,0)};
    e_yb = {Rbi(0,1), Rbi(1,1), Rbi(2,1)};
    e_zb = {Rbi(0,2), Rbi(1,2), Rbi(2,2)};

    // build state vector
    z_State(0) = vx;
    z_State(1) = vy;
    z_State(2) = vz;
    z_State(3) = angx;
    z_State(4) = angy;
    z_State(5) = angz;

    // Calculate T_MPC
    float T_mpc = K_col*inputs(0)/m;

    // Calculate M_MPC
    Eigen::Vector3d M_mpc((-1/tau_p)*angx+K_roll*inputs(1), (-1/tau_q)*angy+K_pitch*inputs(2), (-1/tau_r)*angz+K_yaw*inputs(3));

    //assign the desired dynamics
    Eigen::Vector3d temp1 = g + T_mpc*e_zb;
    f = {g + T_mpc*e_zb, M_mpc};
    
    //calculate the matched Uncertainty
    g_matched(0,0) = e_zb(0);
    g_matched(1,0) = e_zb(1);
    g_matched(2,0) = e_zb(2);

    //calculate the unmatched uncertainty
    g_unmatched(0,0) = e_xb(0);
    g_unmatched(1,0) = e_xb(1);
    g_unmatched(2,0) = e_xb(2);
    g_unmatched(0,1) = e_yb(0);
    g_unmatched(1,1) = e_yb(1);
    g_unmatched(2,1) = e_yb(2);
    
    //PHI, G, Mu, sigma
    PHI = A_s.inverse() * ((A_s*dt).exp() - Eigen::MatrixXd(6,6).setIdentity());

    Eigen::MatrixXd G(g_matched.rows(), g_matched.cols() + g_unmatched.cols());
    G << g_matched, g_unmatched;

    Eigen::VectorXd mu(6);
    mu = (A_s*dt).exp() * (zHat - z_State);
    
    Eigen::VectorXd sigma = -Eigen::MatrixXd(6,6).setIdentity() * G.inverse() * PHI.inverse() * mu;
    Eigen::Vector4d sigma_m(sigma(0), sigma(1), sigma(2), sigma(3)); // matched
    Eigen::Vector2d sigma_um(sigma(4), sigma(5)); // unmatched

    //calculate L1 inputs (according to matched uncertainties)
    inputs = inputs * exp(-w_co*dt) - sigma_m*(1-exp(-w_co*dt));

    // propogate dynamics
    zHat = zHat + (f + g_matched*(inputs + sigma_m) + g_unmatched * sigma_um + A_s*(zHat - z_State))*dt;

    //publish input + MPC input
    mpc_node::Inputs msg;
    msg.col = inputs(0) + mpcInputs.col;
    msg.roll = inputs(1) + mpcInputs.roll;
    msg.pitch = inputs(2) + mpcInputs.pitch;
    msg.yaw = inputs(3) + mpcInputs.yaw;
    inputsPublisher.publish(msg);
}

void adaptiveElement::poseSubscriberCallback(const mpc_node::PoseConstPtr& msg){
    currentPose = *msg;
}

void adaptiveElement::MPCSubscriberCallback(const mpc_node::InputsConstPtr& msg){
    mpcInputs = *msg;
}