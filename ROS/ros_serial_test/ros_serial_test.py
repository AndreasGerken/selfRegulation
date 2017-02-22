#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32

class SerialTest(object):
    def __init__(self):
        print("Initializing")
        self.name = "serial_test"
        self.value = 0
        self.valueD = 1

        self.pub = rospy.Publisher("/comTestPub", Int32, queue_size=5)

        rospy.init_node(self.name)

        print("Starting Loop")
        while not rospy.is_shutdown():
            self.loop()

    def loop(self):
        self.value += self.valueD

        #reverse direction
        if(self.value <= 0 or self.value >= 255):
            self.valueD *= -1

        self.pub.publish(self.value)
        time.sleep(0.01)

if __name__ == "__main__":
    serial_test = SerialTest()
    rospy.spin()
