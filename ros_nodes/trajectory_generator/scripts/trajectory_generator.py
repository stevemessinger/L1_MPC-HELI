from threading import Thread
import rospy
import numpy as np

from heli_messages.msg import Inputs
from heli_messages.msg import Trajectory


class Trajectory_Gen: 

    def __init__(self):
        self.vehicle_state = 0 #manual mode by default
        self.stick_cmd = np.full((4, 1), 0, dtype=float) #initialize stick commands to zero
        self.vehicle_state = 0 #initialize to manual (0) not controller (1) 
        #threading initialization 
        self.t1 = Thread(target=self.console_input)
        self.t2 = Thread(target=self.publish_topics)
        #ROS initializations
        rospy.init_node('traj_gen', anonymous=True)
        self.rate = rospy.Rate(50)
        self.sticksPublisher = rospy.Publisher('L1_inputs', Inputs, queue_size=10)
        self.trajPublisher = rospy.Publisher("/traj_cmd", Trajectory, queue_size=10)
        # make dictionary of possible user inputs and set the entries to the respective functions
        self.inputDict = {
            "exit": self.initialize_input,
            "manual": self.manual_mode
        }
    
    def generate_takeoff_traj(self, dt, height): 
        # generates a trajectory a from (0,0,0) to height with time steps dt at a velocity of 1 m/s
        # dt - how often to generate commands
        # height - desired final takeoff height 
        # state vector: [time pos_x pos_y pos_z q0 q1 q2 q3 vx vy vz wx wy wz]

        time_steps = int(height/dt)
        self.takeoff_traj = np.full((14, time_steps+1), 0, dtype=float)

        pos_z = 0 
        time = 0

        for i in range(time_steps+1): 
            self.takeoff_traj[0,i] = time
            self.takeoff_traj[3, i] = pos_z
            pos_z += dt
            time += dt 

    def console_input(self): 
        while not rospy.is_shutdown():
            # ask and get user command 
            print("input command: ")
            user_input = input()
            #perform action based on command 
            self.inputDict.get(user_input, lambda: print("invalid input"))()

    def publish_topics(self): 
        while not rospy.is_shutdown():
            # hello_str = "hello world %s" % rospy.get_time()
            # self.sticksPublisher.publish(hello_str)
            # self.trajPublisher.publish(hello_str)
            print("thread 2 is running")
            self.rate.sleep()

    def main(self): 
        self.t1.start(); 
        self.t2.start(); 


if __name__ == "__main__":

    traj_gen = Trajectory_Gen()
    traj_gen.main()