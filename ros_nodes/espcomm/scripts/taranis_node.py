import rospy
import types
import numpy
from heli_messages.msg import Inputs
import serial
import numpy as np

roll = 0
pitch = 0
yaw = 0 
col = 0

def inputCallback(msg):
    global roll 
    global pitch 
    global yaw 
    global col 
    roll = int(numpy.interp(max(-1, min(msg.roll, 1)), [-1, 1], [-500, 500]))
    pitch = int(numpy.interp(max(-1, min(msg.pitch, 1)), [-1, 1], [500, -500]))
    yaw = int(numpy.interp(max(-1, min(msg.yaw, 1)), [-1, 1], [-500, 500]))
    col = int(numpy.interp(max(-1, min(msg.col, 1)), [-1, 1], [500, -500]))

    return

def sendTaranis():
    rospy.init_node('taranis_cmd', anonymous=True)
    rospy.Subscriber('mpc_input', Inputs, inputCallback)
    #rospy.Subscriber('L1_inputs', Inputs, inputCallback)
    rate = rospy.Rate(100) # 100hz

    ser = serial.Serial('/dev/ttyACM0')
    print(ser.name)
    ser.write(b'\r')

    while not rospy.is_shutdown():
        value0 = roll
        value1 = col
        value2 = yaw
        value3 = pitch
        
        message0 = f'set trainer 0 {int(value0)} \r'
        message1 = f'set trainer 1 {int(value1)} \r'
        message2 = f'set trainer 2 {int(value2)} \r'
        message3 = f'set trainer 3 {int(value3)} \r'

        ser.write(message0.encode())
        ser.write(message1.encode())
        ser.write(message2.encode())
        ser.write(message3.encode())
        rate.sleep()
    return

if __name__ == '__main__':
    try:
        sendTaranis()
    except rospy.ROSInterruptException:
        pass
