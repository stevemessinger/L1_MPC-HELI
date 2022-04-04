#include "../include/mpc_Heli.h"
#define ever ;; //lol

int main(int argc, char** argv){
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    mpcController controller; // create the controller object
    
    for(ever){

        controller.loop(); //call the solver

        ros::spinOnce(); 
        loop_rate.sleep(); // wait for next loop
    }
    
    return 0;
}