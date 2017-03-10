import sys, time, argparse
sys.path.insert(0, '../')

from classes.homeostasis_c import Homeostasis
import rospy
from std_msgs.msg import Float32MultiArray
from tiny_msgs.msg import tinyIMU
import numpy as np

class Homeostasis_ROS(object):

    def __init__(self, args):
        self.name = "Homeostasis_ROS"

        # create motor message
        self.motorMessage = Float32MultiArray()

        # initialize ros node
        rospy.init_node(self.name)

        # initialize publishers and subscribers
        self.pub = rospy.Publisher("/homeostasis_motor", Float32MultiArray, queue_size=5)
        self.sub = rospy.Subscriber("/tinyImu", tinyIMU, self.receiveSensorValues)

        # initialize homeokinesis
        self.homeostasis = Homeostasis(dim_m=4, dim_s = 6)
        self.homeostasis.setLearningRates(0.15,0.1)

        if args.verbose:
            print("initialization finished")

        self.minX = np.ones(6) * 100000
        self.maxX = np.ones(6) * -100000
        self.avgX = np.zeros(6)


    def receiveSensorValues(self, msg):
        #sensorData = np.array(msg.data).reshape((2,1))

        if args.verbose:
            print("SensorData received: " + time.strftime('%X'))
            print(msg)


        self.homeostasis.x[1] = (msg.accel.x + 32768.0) / (2.0 * 32768.0) * 2.0
        self.homeostasis.x[0] = (msg.accel.y + 32768.0) / (2.0 * 32768.0) * 2.0
        self.homeostasis.x[2] = (msg.accel.z + 32768.0) / (2.0 * 32768.0) * 2.0
        self.homeostasis.x[3] = (msg.gyro.x + 32768.0) / (2.0 * 32768.0) * 2.0
        self.homeostasis.x[4] = (msg.gyro.y + 32768.0) / (2.0 * 32768.0) * 2.0
        self.homeostasis.x[5] = (msg.gyro.z + 32768.0) / (2.0 * 32768.0) * 2.0

        for i in range(0,6):
            if(self.homeostasis.x[i] > self.maxX[i]):
                self.maxX[i] = self.homeostasis.x[i]
            if(self.homeostasis.x[i] < self.minX[i]):
                self.minX[i] = self.homeostasis.x[i]

            self.avgX[i] = self.avgX[i] * 0.99 + self.homeostasis.x[i] * 0.01

            print("x " + str(i) + " min: " + str(self.minX[i]) + " max " + str(self.maxX[i]) + " avg " + str(self.avgX[i]))

        self.homeostasis.learningStep()

        motor = self.homeostasis.y.flatten().tolist()

        for m in range(len(motor)):
            if(motor[m] > 1):
                motor[m] = 1
            if(motor[m] < -1):
                motor[m] = -1
            motor[m] *= 32768.0

        # bring in range - 32768 to 32768
        #motor = motor * 32768.0

        self.motorMessage.data = motor

        self.pub.publish(self.motorMessage)

        if args.verbose:
            print("MotorData sent: " + time.strftime('%X'))
            print(self.motorMessage.data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Ros wrapper for homeostasis class")
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    args = parser.parse_args()

    Homeostasis_ROS(args)
    rospy.spin()
