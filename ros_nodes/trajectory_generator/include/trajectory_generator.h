#include <fstream>
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "mpc_node/Trajectory.h"

class trajectoryGenerator{
    public:
    trajectoryGenerator();

    ~trajectoryGenerator();

    void init();

    void loop();

    private:

    ros::NodeHandle nh;
    ros::Publisher trajectoryPublisher;

    


};