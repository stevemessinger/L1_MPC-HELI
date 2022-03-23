#include <ros/ros.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "extended_kf/Pose.h"
#include "extended_kf/Imu.h"
#include "extended_kf/Vicon.h"
#include "extended_kf/include/matlab_files/EKF.h"

float[6] imu; 
float[7] vicon; 

void imuCallback(const extended_kf::Imu::ConstPtr& imuMessage)
{
    imu[0] = imuMessage->ax; 
    imu[1] = imuMessage->ay;
    imu[2] = imuMessage->az;
    imu[3] = imuMessage->wx;
    imu[4] = imuMessage->wy;
    imu[5] = imuMessage->wz;
}

void viconCallback(const extended_kf::Vicon::ConstPtr& viconMessage)
{
    vicon[0] = viconMessage->x; 
    vicon[1] = viconMessage->y; 
    vicon[2] = viconMessage->z; 
    vicon[3] = viconMessage->qw; 
    vicon[4] = viconMessage->qx; 
    vicon[5] = viconMessage->qy; 
    vicon[6] = viconMessage->qz; 
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "EKF_node");
    ros::NodeHandle nh;

    ros::Subscriber imuSubscriber = nh.subscribe("/imu_data", 10, imuCallback);
    ros::Subscriber viconSubscriber = nh.subscribe("/vicon_data", 10, viconCallback);
    ros::Publisher posePublisher;

    posePublisher = nh.advertise<extended_kf::Pose>("/ekf_pose", 10);

    float[16] x0; 
    x0[0]=0; x0[1]=0; x0[2]=0; x0[3]=0; x0[4]=0; x0[5]=0; x0[6]=1; x0[7]=0;
    x0[8]=0; x0[9]=0; x0[10]=0; x0[11]=0; x0[12]=0; x0[13]=0; x0[14]=0; x0[15]=0; 
    // Initialize EKF
    EKF_ros = EKF(x0); 

    double currentTime; 
    ros::Duration d;
    double dt; 

    double pastTime = ros::Time::now().toSec(); 

    ros::Rate rate(200);

    while(ros::ok())
    {
        currentTime = ros::Time::now().toSec(); 
        dt = currentTime - pastTime; 
        //calculate state estimation 
        extended_kf::Pose poseMessage;

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
        poseMessage.angy = imu[4];
        poseMessage.angz = imu[5]; 

        posePublisher.publish(poseMessage);

        pastTime = currentTime; 

        ros::spinOnce(); 
        loop_rate.sleep();               
    }
}