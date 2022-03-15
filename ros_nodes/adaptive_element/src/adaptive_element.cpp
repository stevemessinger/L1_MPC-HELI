#include "../include/adaptive_element.h"

adaptiveElement::adaptiveElement(){

    poseSubscriber = nh.subscribe("pose",  500, &adaptiveElement::poseSubscriberCallback, this);
}

adaptiveElement::~adaptiveElement(){

}

void adaptiveElement::loop(){

}

void adaptiveElement::poseSubscriberCallback(const mpc_node::PoseConstPtr& poseMessage){

}