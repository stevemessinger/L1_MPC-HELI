
// Ros includes
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "mpc_node/Pose.h"
#include "mpc_node/Inputs.h"

//3rd party includes
#include <Eigen/Dense>

class adaptiveElement{
public:
    adaptiveElement();

    ~adaptiveElement();

    void loop();


private:

    void poseSubscriberCallback(const mpc_node::PoseConstPtr& msg);

    void MPCSubscriberCallback(const mpc_node::InputsConstPtr& msg);

    // node variables
    ros::NodeHandle nh;
    ros::Subscriber poseSubscriber;
    ros::Subscriber MPCSubscriber;
    ros::Publisher inputsPublisher;

    // messaging variables
    mpc_node::Pose currentPose;
    mpc_node::Inputs mpcInputs;

    // L1 Variables
    Eigen::Matrix3d Rbi;
    Eigen::Vector3d e_xb;
    Eigen::Vector3d e_yb;
    Eigen::Vector3d e_zb;
    Eigen::VectorXd z_State;
    Eigen::VectorXd zHat;
    Eigen::Vector4d inputs;
    Eigen::Vector3d g;
    Eigen::Vector3d f;
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