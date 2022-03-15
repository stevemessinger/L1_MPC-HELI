#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "mpc_node/Pose.h"

class adaptiveElement{
public:
    adaptiveElement();

    ~adaptiveElement();

    void loop();


private:

    void poseSubscriberCallback(const mpc_node::PoseConstPtr& poseMessage);

    ros::NodeHandle nh;
    ros::Subscriber poseSubscriber;


};