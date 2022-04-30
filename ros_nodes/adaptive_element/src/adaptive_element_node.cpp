// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "heli_messages/Pose.h"
#include "heli_messages/Inputs.h"

// Custom includes
#include "../include/matlab/L1_Controller.h"

/// Globals
double pose[13] = {0,0,0,1,0,0,0,0,0,0,0,0,0};
double mpcInputs[4] = {0,0,0,0};
int nav = 0;

/// Callbacks
void poseSubscriberCallback(const heli_messages::Pose::ConstPtr& msg){
    pose[0] = msg->x;
    pose[1] = msg->y;
    pose[2] = msg->z;
    pose[3] = msg->qw;
    pose[4] = msg->qx;
    pose[5] = msg->qy;
    pose[6] = msg->qz;
    pose[7] = msg->vx;
    pose[8] = msg->vy;
    pose[9] = msg->vz;
    pose[10] = msg->angx;
    pose[11] = msg->angy;
    pose[12] = msg->angz;
}

void MPCSubscriberCallback(const heli_messages::Inputs::ConstPtr& msg){
    mpcInputs[0] = msg->col;
    mpcInputs[1] = msg->roll;
    mpcInputs[2] = msg->pitch;
    mpcInputs[3] = msg->yaw; 
    nav = msg->nav;
}


/// Main
int main(int argc, char** argv){
    
    // ROS initialization
    ros::init(argc, argv, "adaptive_element_node");
    ros::NodeHandle nh;
    ros::Subscriber poseSubscriber;
    ros::Subscriber MPCSubscriber;
    ros::Publisher inputsPublisher;
    double startTime = ros::Time::now().toSec();

    poseSubscriber = nh.subscribe("pose",  1000, &poseSubscriberCallback);
    MPCSubscriber = nh.subscribe("mpc_input", 1000, &MPCSubscriberCallback);
    inputsPublisher = nh.advertise<heli_messages::Inputs>("L1_inputs", 1000);
    heli_messages::Inputs L1Inputsmsg;

    // L1 initialization
    L1_Controller controller;
    controller.init(5, 15);

    ros::Rate loopRate(500);
    

    while(ros::ok()){
        double now = ros::Time::now().toSec();
        double dt = now - startTime;

        // if nav = 1 then execute L1 controller
        if(nav){
            controller.updateController(pose, mpcInputs, dt);
            L1Inputsmsg.col = controller.u[0];
            L1Inputsmsg.roll = controller.u[1];
            L1Inputsmsg.pitch = controller.u[2];
            L1Inputsmsg.yaw = controller.u[3];

            inputsPublisher.publish(L1Inputsmsg);
            //std::cout<<"executing L1 controller"<<std::endl;           
        }
        else{ // else reinitialize controller
            //std::cout << "reinitializing controller" << std::endl;
            controller.init(5,15);
        }

        startTime = now;
        loopRate.sleep();
        ros::spinOnce();
    }

    return 0;
}