#include "../include/trajectory_generator.h"

trajectoryGenerator::trajectoryGenerator(){
    trajectoryPublisher = nh.advertise<mpc_node::Trajectory>("trajectory", 500);

    //here we need to use the file defined in a config file to load in the trajectory data

}

trajectoryGenerator::~trajectoryGenerator(){

}


void trajectoryGenerator::loop(){
    //check for user input(takeoff, run trajectory, ect)

    // depending on the trajectory, calculate desired trajectory

    // check the timestamps on each pose in the trajectory and publish data
}