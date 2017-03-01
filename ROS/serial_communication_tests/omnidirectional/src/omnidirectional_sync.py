#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32

class SerialTest(object):
    def __init__(self):
        self.name = "omnidirectional_sync"
        self.value = 0

        self.rcv = 0
        self.sent = 0

        rospy.init_node(self.name)

        self.pub = rospy.Publisher("/comTestPub", Int32, queue_size=5)
        self.sub = rospy.Subscriber("/comTestSub", Int32, self.subFunc)


    def subFunc(self, msg):
        # increase the counter for received messages
        self.rcv += 1

        # Map value from 0-1024 to analog Output Range of 0-255
        self.value = msg.data / 4

        # increment counter for sent messages
        self.sent +=1

        # publish the value
        self.pub.publish(self.value)

        # print every 100 publishings
        if(self.sent % 100 == 0):
            print("Sent: %d Received: %d" % (self.sent, self.rcv))


if __name__ == "__main__":
    print("Initialization")
    serial_test = SerialTest()

    print("Starting 'loop'")
    while not rospy.is_shutdown():
        rospy.spin()
