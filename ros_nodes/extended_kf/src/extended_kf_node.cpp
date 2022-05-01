#include <ros/ros.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "heli_messages/Pose.h"
#include "heli_messages/BNO.h"
#include "geometry_msgs/TransformStamped.h"
#include "../include/matlab/EKF.h"

double imu[6]; 
double vicon[7]; 

void imuCallback(const heli_messages::BNO::ConstPtr& imuMessage)
{
    imu[0] = imuMessage->ax; 
    imu[1] = imuMessage->ay;
    imu[2] = imuMessage->az;
    imu[3] = imuMessage->gx;
    imu[4] = imuMessage->gy;
    imu[5] = imuMessage->gz;
}


void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& viconMessage)
{
    vicon[0] = viconMessage->transform.translation.x; 
    vicon[1] = -viconMessage->transform.translation.y;
    vicon[2] = -viconMessage->transform.translation.z;
    vicon[3] = viconMessage->transform.rotation.w; 
    vicon[4] = viconMessage->transform.rotation.x; 
    vicon[5] = -viconMessage->transform.rotation.y; 
    vicon[6] = -viconMessage->transform.rotation.z; 
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "EKF_node");
    ros::NodeHandle nh;

    ROS_INFO("EKF Node Initialized!");

    ros::Subscriber imuSubscriber = nh.subscribe("BNOData", 1000, imuCallback);
    ros::Subscriber viconSubscriber = nh.subscribe("/vicon/Explorer_M2/Explorer_M2", 1000, viconCallback);
    ros::Publisher posePublisher;

    posePublisher = nh.advertise<heli_messages::Pose>("pose", 1000);

    double x0[16]; 
    x0[0]=0; x0[1]=0; x0[2]=0; x0[3]=0; x0[4]=0; x0[5]=0; x0[6]=1; x0[7]=0;
    x0[8]=0; x0[9]=0; x0[10]=0; x0[11]=0; x0[12]=0; x0[13]=0; x0[14]=0; x0[15]=0; 
    // Initialize EKF
    EKF EKF_ros;
    EKF_ros.init(x0);

    double currentTime; 
    ros::Duration d;
    double dt; 

    double pastTime = ros::Time::now().toSec(); 

    ros::Rate loop_rate(500);

    while(ros::ok())
    {
        currentTime = ros::Time::now().toSec(); 
        dt = currentTime - pastTime; 
        //calculate state estimation 
        heli_messages::Pose poseMessage;

        EKF_ros.calc_estimate(vicon, imu, dt); 

        poseMessage.x = EKF_ros.x_hat[0];
        poseMessage.y = EKF_ros.x_hat[1];
        poseMessage.z = EKF_ros.x_hat[2];
        poseMessage.vx = EKF_ros.x_hat[3];
        poseMessage.vy = EKF_ros.x_hat[4];
        poseMessage.vz = EKF_ros.x_hat[5];
        poseMessage.qw = EKF_ros.x_hat[6];
        poseMessage.qx = EKF_ros.x_hat[7];
        poseMessage.qy = EKF_ros.x_hat[8];
        poseMessage.qz = EKF_ros.x_hat[9]; 
        poseMessage.angx = imu[3];
        poseMessage.angy = -imu[4];
        poseMessage.angz = -imu[5]; 

        posePublisher.publish(poseMessage);

        pastTime = currentTime; 

        ros::spinOnce(); 
        loop_rate.sleep();               
    }
}