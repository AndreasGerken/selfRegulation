#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty

class SerialTest(object):
    def __init__(self):
        self.name = "serial_test"

        self.pub = rospy.Publisher("/comTest", Empty, queue_size=5)

        rospy.init_node(self.name)
        while not rospy.is_shutdown():
            self.loop()

    def loop(self):
        print("Message Published")
        self.pub.publish()
        time.sleep(1)

if __name__ == "__main__":
    serial_test = SerialTest()
    rospy.spin()
