import sys, time, datetime, argparse
sys.path.insert(0, '../../../../python/')

from classes.homeostasis_c import Homeostasis
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

def printWithTime(msg):
    print datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3] + ": " + msg

class Homeostasis_ROS(object):

    def __init__(self, args):
        self.name = "Homeostasis_ROS"

        # create motor message
        self.motorMessage = Float32MultiArray()

        # initialize ros node
        rospy.init_node(self.name)

        # initialize publishers and subscribers
        self.pub = rospy.Publisher("/homeostasis_motor", Float32MultiArray, queue_size=5)
        self.sub = rospy.Subscriber("/homeostasis_sensor", Float32MultiArray, self.receiveSensorValues)

        # initialize homeokinesis
        self.homeostasis = Homeostasis(dim_m=1, dim_s = 2)

        if args.verbose:
            printWithTime("initialization finished")


    def receiveSensorValues(self, msg):
        sensorData = np.array(msg.data).reshape((2,1))

        if args.verbose:
            printWithTime("SensorData received")
            print(sensorData)

        self.homeostasis.x = sensorData

        self.homeostasis.learningStep()

        self.motorMessage.data = [self.homeostasis.y[0,0]]

        self.pub.publish(self.motorMessage)

        if args.verbose:
            printWithTime("MotorData sent")
            print(self.motorMessage.data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Ros wrapper for homeostasis class")
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    args = parser.parse_args()

    Homeostasis_ROS(args)
    rospy.spin()
