#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pygame


class get_data:

    def __init__(self):
        self.pub = rospy.Publisher("/taranis_state", String, queue_size=10)
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def callback_data(self):
        #grab taranis values 
        pygame.event.pump()

        throttle = self.joystick.get_axis(0) #throttle 
        roll = self.joystick.get_axis(1) #roll
        pitch = self.joystick.get_axis(2) #pitch
        yaw = self.joystick.get_axis(3) #yaw
        arm = self.joystick.get_axis(4) # arm switch 

        input_str = "time:%s, thrl:%s, r:%s, p:%s, y:%s, a:%s" % (rospy.get_time(), throttle, roll, pitch, yaw, arm)
        #rospy.loginfo(input_str)
        self.pub.publish(input_str)

if __name__ == '__main__':
    rospy.init_node('get_data')
    taranis_data = get_data()
    rate = rospy.Rate(50) # 50hz

    while not rospy.is_shutdown():
            taranis_data.callback_data()
            rate.sleep()