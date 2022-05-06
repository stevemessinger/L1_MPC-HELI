
# This python node records inputs of the Taranis controller
# and publishes them

import rospy
from std_msgs.msg import String
import pygame

def taranis_talk():
    pub = rospy.Publisher('data_rec', String, queue_size=1)
    rospy.init_node('taranis_talk', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()


    while not rospy.is_shutdown():

        #grab taranis values 
        pygame.event.pump()

        throttle = joystick.get_axis(0) #throttle 
        roll = joystick.get_axis(1) #roll
        pitch = joystick.get_axis(2) #pitch
        yaw = joystick.get_axis(3) #yaw
        arm = joystick.get_axis(4) # arm switch 

        input_str = "time:%s, thrl:%s, r:%s, p:%s, y:%s, a:%s" % (rospy.get_time(), throttle, roll, pitch, yaw, arm)
        rospy.loginfo(input_str)
        pub.publish(input_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        taranis_talk()
    except rospy.ROSInterruptException:
        pass

