import rospy
from espcomm.msg import BNO
import websocket


def publishBNOData():
    pub = rospy.Publisher('BNOData', BNO, queue_size=10)
    rospy.init_node('espcomm', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    msg = BNO()

    connected = False
    while not connected:
        try:
            serverIP = "192.168.0.164"
            ws = websocket.create_connection("ws://" + serverIP)
            connected = True
        except Exception as e:
            print(e)

    print("Connected!")

    while not rospy.is_shutdown():
        message = ws.recv()
        data = message.split(':')

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

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publishBNOData()
    except rospy.ROSInterruptException:
        pass