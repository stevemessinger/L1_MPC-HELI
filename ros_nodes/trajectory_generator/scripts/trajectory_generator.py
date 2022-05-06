from threading import Thread
from tracemalloc import start
import rospy
import numpy as np
import time
import math
from pynput.keyboard import Key, Listener

from heli_messages.msg import Inputs
from heli_messages.msg import Trajectory
from heli_messages.msg import KillSwitch


class Trajectory_Gen: 

    def __init__(self):
        self.vehicle_state = 0 #manual mode by default
        self.stick_cmd = Inputs() #initialize stick commands to zero
        self.vehicle_state = 0 #initialize to manual (0) not controller (1) 
        self.trajectory = Trajectory() #initialize trajectory to zero
        self.trajectory.nav = 0
        self.kill = KillSwitch() #initialize kill switch to zero
        self.kill.killSwitch = 1 
        #threading initialization 
        self.t1 = Thread(target=self.console_input)
        self.t2 = Thread(target=self.publish_topics)
        self.t3 = Thread(target=self.killSwitchChecker)
        #ROS initializations
        rospy.init_node('traj_gen', anonymous=True)
        self.rate = rospy.Rate(500)
        self.sticksPublisher = rospy.Publisher('L1_inputs', Inputs, queue_size=1)
        self.trajPublisher = rospy.Publisher("trajectory", Trajectory, queue_size=1)
        self.killSwitchPublisher = rospy.Publisher("killSwitch", KillSwitch, queue_size=1)
        # make dictionary of possible user inputs and set the entries to the respective functions
        self.inputDict = {
            "exit": self.initialize_input,
            "manual": self.manual_mode,
            "takeoff": self.takeoff,
            "land": self.land,
            "check surfaces" : self.check_surfaces,
            "lemniscate" : self.lemniscate,
            "circle" : self.circle,
            "melon" : self.melon,
            "start" : self.start,
            "stop" : self.stop
        }

        # load in all the files
        self.land_traj = np.loadtxt('/home/ros/catkin_ws/src/L1_MPC-HELI/ros_nodes/trajectory_generator/src/Land.txt', delimiter='\t', skiprows=1)
        self.takeoff_traj = np.loadtxt('/home/ros/catkin_ws/src/L1_MPC-HELI/ros_nodes/trajectory_generator/src/Takeoff.txt', delimiter='\t', skiprows=1)
        self.lemniscate_traj = np.loadtxt('/home/ros/catkin_ws/src/L1_MPC-HELI/ros_nodes/trajectory_generator/src/lemniscate.txt', delimiter='\t', skiprows=1)
        #self.circle_traj = np.loadtxt('../include/circle.csv', delimiter=',', skiprows=1)
        #self.melon_traj = np.loadtxt('../include/melon.csv', delimiter=',', skiprows=1)
    
    def killSwitchChecker(self):
        def on_press(key):
            if key == Key.space:
                print("Killing the vehicle!")
                self.kill.killSwitch = 1
                self.trajectory.nav = 0
                return False
        
        def on_release(key):
            return False

        while not rospy.is_shutdown():
            with Listener(on_press=on_press, on_release=on_release) as listener:
                listener.join()
            self.rate.sleep()

    def takeoff(self):
        print("executing takeoff!")
        
        endTime = self.takeoff_traj[-1,0]

        startTime = time.time()
        i = 0
        self.trajectory.nav = 1
        while(time.time() - startTime < endTime):
            if(time.time() - startTime > self.takeoff_traj[i,0]): 
                if(i+10 >= self.takeoff_traj.shape[0]): 
                    continue
                else:
                    i +=1
                
            self.trajectory.x = self.takeoff_traj[i:i+10, 1]
            self.trajectory.y = self.takeoff_traj[i:i+10, 2]
            self.trajectory.z = self.takeoff_traj[i:i+10, 3]
            self.trajectory.qw = self.takeoff_traj[i:i+10, 4]
            self.trajectory.qx = self.takeoff_traj[i:i+10, 5]
            self.trajectory.qy = self.takeoff_traj[i:i+10, 6]
            self.trajectory.qz = self.takeoff_traj[i:i+10, 7]
            self.trajectory.vx = self.takeoff_traj[i:i+10, 8]
            self.trajectory.vy = self.takeoff_traj[i:i+10, 9]
            self.trajectory.vz = self.takeoff_traj[i:i+10, 10]
            self.trajectory.angx = self.takeoff_traj[i:i+10, 11]
            self.trajectory.angy = self.takeoff_traj[i:i+10, 12]
            self.trajectory.angz = self.takeoff_traj[i:i+10, 13]
            
            print(i)
        
        print("End of takeoff!")
        

    def land(self):
        print("executing landing!")
        
        endTime = self.land_traj[-1,0]

        startTime = time.time()
        i = 0
        self.trajectory.nav = 1
        while(time.time() - startTime < endTime):
            if(time.time() - startTime > self.land_traj[i,0]): 
                if(i+10 >= self.land_traj.shape[0]): 
                    continue
                else:
                    i +=1
                
            self.trajectory.x = self.land_traj[i:i+10, 1]
            self.trajectory.y = self.land_traj[i:i+10, 2]
            self.trajectory.z = self.land_traj[i:i+10, 3]
            self.trajectory.qw = self.land_traj[i:i+10, 4]
            self.trajectory.qx = self.land_traj[i:i+10, 5]
            self.trajectory.qy = self.land_traj[i:i+10, 6]
            self.trajectory.qz = self.land_traj[i:i+10, 7]
            self.trajectory.vx = self.land_traj[i:i+10, 8]
            self.trajectory.vy = self.land_traj[i:i+10, 9]
            self.trajectory.vz = self.land_traj[i:i+10, 10]
            self.trajectory.angx = self.land_traj[i:i+10, 11]
            self.trajectory.angy = self.land_traj[i:i+10, 12]
            self.trajectory.angz = self.land_traj[i:i+10, 13]
            
            print(i)
        
        self.trajectory.nav = 0
        
        print("End of landing!")
 

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
            self.sticksPublisher.publish(self.stick_cmd)
            self.trajPublisher.publish(self.trajectory)
            self.killSwitchPublisher.publish(self.kill)
            self.rate.sleep()

    def initialize_input(self):
        print("initializing inputs")
        self.stick_cmd = np.full((6, 1), 0, dtype=float)
        self.vehicle_state = 0

    def manual_mode(self):
        self.vehicle_state = 0
        print("manual mode")

    def check_surfaces(self): 
        print("Checking control surfaces!")
        startTime = time.time()

        while(time.time() - startTime < 10): 
            self.stick_cmd.roll = math.cos(0.5*time.time())
            self.stick_cmd.pitch = math.cos(0.5*time.time())
            self.stick_cmd.throttle = 0
            self.stick_cmd.yaw = math.cos(0.5*time.time())
            self.stick_cmd.col = math.cos(0.5*time.time())
            self.stick_cmd.killSwitch = 1
        print("Done checking control surfaces!")
    
    def lemniscate(self):
        print("executing lemniscate!")
        
        endTime = self.lemniscate_traj[-1,0]

        startTime = time.time()
        i = 0
        self.trajectory.nav = 1
        while(time.time() - startTime < endTime):
            if(time.time() - startTime > self.lemniscate_traj[i,0]): 
                if(i+10 >= self.lemniscate_traj.shape[0]): 
                    continue
                else:
                    i +=1
                
            self.trajectory.x = self.lemniscate_traj[i:i+10, 1]
            self.trajectory.y = self.lemniscate_traj[i:i+10, 2]
            self.trajectory.z = self.lemniscate_traj[i:i+10, 3]
            self.trajectory.qw = self.lemniscate_traj[i:i+10, 4]
            self.trajectory.qx = self.lemniscate_traj[i:i+10, 5]
            self.trajectory.qy = self.lemniscate_traj[i:i+10, 6]
            self.trajectory.qz = self.lemniscate_traj[i:i+10, 7]
            self.trajectory.vx = self.lemniscate_traj[i:i+10, 8]
            self.trajectory.vy = self.lemniscate_traj[i:i+10, 9]
            self.trajectory.vz = self.lemniscate_traj[i:i+10, 10]
            self.trajectory.angx = self.lemniscate_traj[i:i+10, 11]
            self.trajectory.angy = self.lemniscate_traj[i:i+10, 12]
            self.trajectory.angz = self.lemniscate_traj[i:i+10, 13]
            
            print(i)
        
        print("End of Lemniscate!")

    def startTrajectory(self, ):
        print("Starting trajectory!")
        

        self.trajectory.nav = 1


    def circle(self):
        print("executing circle!")

        #load in circle trajectory from a csv file
        circle_traj = np.loadtxt('circle_traj.txt', delimiter=',')
        endTime = circle_traj[-1,0]
        
    def melon(self):
        print("executing melon!")

        #load in melon trajectory from a csv file
        melon_traj = np.loadtxt('melon_traj.txt', delimiter=',')
        endTime = melon_traj[-1,0]

    def start(self):
        print("starting!")
        self.kill.killSwitch = 0
        
        
    def stop(self):
        print("stopping!")
        self.kill.killSwitch = 1
        

    def main(self): 
        self.t1.start()
        self.t2.start() 
        self.t3.start()


if __name__ == "__main__":

    traj_gen = Trajectory_Gen()
    traj_gen.main()