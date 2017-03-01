#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32

class SerialTest(object):
    def __init__(self):
        self.name = "ardu_subscriber"
        self.value = 0

        # direction of the value
        self.valueD = 1

        # create the publisher for the value
        self.pub = rospy.Publisher("/comTestPub", Int32, queue_size=5)

        # initialize node
        rospy.init_node(self.name)


    def loop(self):
        # increase or decrease value
        self.value += self.valueD

        # reverse direction
        if(self.value <= 0 or self.value >= 255):
            self.valueD *= -1

        # publish the message
        self.pub.publish(self.value)

        time.sleep(0.01)

if __name__ == "__main__":
    print("Initializing")
    serial_test = SerialTest()
    rospy.spin()

    # repeat the loop
    print("Starting Loop")
    while not rospy.is_shutdown():
        self.loop()

    rospy.spin()
