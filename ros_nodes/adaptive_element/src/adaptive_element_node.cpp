#include "../include/adaptive_element.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "adaptive_element_node");
    ros::NodeHandle nh;
    adaptiveElement L1;

    ros::Rate loopRate(500);

    while(ros::ok()){
        L1.loop();
        loopRate.sleep();
        ros::spinOnce();
    }

    return 0;
}