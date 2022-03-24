from threading import Thread

from sympy import Float
import rospy
import numpy as np


def console_input(filename, substr, new_substr):
    print(f'Processing the file {filename}')
    # get the contents of the file
    with open(filename, 'r') as f:
        content = f.read()

    # replace the substr by new_substr
    content = content.replace(substr, new_substr)

    # write data into the file
    with open(filename, 'w') as f:
        f.write(content)


def main():
    filenames = [
        'c:/temp/test1.txt',
        'c:/temp/test2.txt',
        'c:/temp/test3.txt',
        'c:/temp/test4.txt',
        'c:/temp/test5.txt',
        'c:/temp/test6.txt',
        'c:/temp/test7.txt',
        'c:/temp/test8.txt',
        'c:/temp/test9.txt',
        'c:/temp/test10.txt',
    ]

    # create threads
    threads = [Thread(target=replace, args=(filename, 'id', 'ids'))
            for filename in filenames]

    # start the threads
    for thread in threads:
        thread.start()

    # wait for the threads to complete
    for thread in threads:
        thread.join()


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
        self.sticksPublisher = rospy.Publisher("/stick_cmd", Sticks, queue_size=10)
        self.trajPublisher = rospy.Publisher("/traj_cmd", Trajectory, queue_size=10)
    
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
            something = input()
            print("thread 1 is running" + something)

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