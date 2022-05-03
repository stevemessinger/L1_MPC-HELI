from ast import Global
from email import message
from ipaddress import collapse_addresses
import string
import rospy
import websocket

import numpy

from heli_messages.msg import BNO



def publishBNOData():
    rospy.init_node('espcomm', anonymous=True)
    pub = rospy.Publisher('BNOData', BNO, queue_size=100)
    rate = rospy.Rate(100) # 100hz
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
        ws.send('x')
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


if __name__ == '__main__':
    try:
        publishBNOData()
    except rospy.ROSInterruptException:
        pass
