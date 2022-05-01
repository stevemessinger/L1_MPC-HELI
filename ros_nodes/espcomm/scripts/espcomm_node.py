from ast import Global
from email import message
from ipaddress import collapse_addresses
import string
import rospy
import websocket
import types
import numpy
from heli_messages.msg import Inputs
from heli_messages.msg import BNO
from heli_messages.msg import KillSwitch

command = "0:0:0:0:0"
throttle = int(1811)

def inputCallback(msg):
    global command
    global throttle
    roll = int(numpy.interp(max(-1, min(msg.roll, 1)), [-1, 1], [337, 1646]))
    pitch = int(numpy.interp(max(-1, min(msg.pitch,1)), [-1, 1], [337, 1646]))
    yaw = int(numpy.interp(max(-1, min(msg.yaw, 1)), [-1, 1], [172, 1811]))
    col = int(numpy.interp(max(-1, min(msg.col,1)), [-1, 1], [172, 1811]))

    command = str(roll) + ":" + str(pitch) + ":" + str(throttle) + ":" + str(yaw) + ":" + str(col)
    #print(command)
    return

def publishBNOData():
    rospy.init_node('espcomm', anonymous=True)
    pub = rospy.Publisher('BNOData', BNO, queue_size=100)
    rospy.Subscriber('mpc_input', Inputs, inputCallback)
    #rospy.Subscriber('L1_inputs', Inputs, inputCallback)
    rospy.Subscriber('killSwitch', KillSwitch, killSwitchCallback)
    rate = rospy.Rate(500) # 100hz
    msg = BNO()

    connected = False
    while not connected:
        try:
            serverIP = "192.168.4.1"
            ws = websocket.create_connection("ws://" + serverIP)
            connected = True
        except Exception as e:
            print(e)

    print("Connected!")

    while not rospy.is_shutdown():
        ws.send(command)
        message = ws.recv()
        data = message.split(':')
        
        try:
            msg.ax = float(data[0])
            msg.ay = float(data[1])
            msg.az = float(data[2])
            msg.gx = float(data[3])
            msg.gy = float(data[4])
            msg.gz = float(data[5])
            msg.sysCal = float(data[6])
            msg.accelCal = float(data[7])
            msg.gyroCal = float(data[8])
            msg.magCal = float(data[9])
            msg.dt = float(data[10])
        except Exception as e:
            print(e)

        pub.publish(msg)
        rate.sleep()
    return

def killSwitchCallback(msg):
    global throttle
    if msg.killSwitch:
        throttle = int(1811)
    else:
        throttle = int(337)
    return

if __name__ == '__main__':
    try:
        publishBNOData()
    except rospy.ROSInterruptException:
        pass
