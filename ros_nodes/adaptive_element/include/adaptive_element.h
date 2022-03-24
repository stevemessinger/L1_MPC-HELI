
// Ros includes
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "heli_messages/Pose.h"
#include "heli_messages/Inputs.h"

//3rd party includes
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

class adaptiveElement{
public:
    adaptiveElement();

    ~adaptiveElement();

    void loop();

private:

    void poseSubscriberCallback(const heli_messages::Pose::ConstPtr& msg);

    void MPCSubscriberCallback(const heli_messages::Inputs::ConstPtr& msg);

    // node variables
    ros::NodeHandle nh;
    ros::Subscriber poseSubscriber;
    ros::Subscriber MPCSubscriber;
    ros::Publisher inputsPublisher;

    // messaging variables
    heli_messages::Pose currentPose;
    heli_messages::Inputs mpcInputs;

    // L1 Variables
    Eigen::Matrix3d Rbi;
    Eigen::Vector3d e_xb;
    Eigen::Vector3d e_yb;
    Eigen::Vector3d e_zb;
    Eigen::VectorXd z_State;
    Eigen::VectorXd zHat;
    Eigen::Vector4d inputs;
    Eigen::Vector3d g;
    Eigen::VectorXd f;
    Eigen::MatrixXd g_matched;
    Eigen::MatrixXd g_unmatched;
    Eigen::MatrixXd A_s;
    Eigen::MatrixXd PHI;
    Eigen::MatrixXd G;

    float m;
    float K_col, K_roll, K_pitch, K_yaw;
    float tau_p, tau_q, tau_r;
    float w_co;
    float previousTime;
};