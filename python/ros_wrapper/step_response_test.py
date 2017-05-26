import sys, time, argparse
sys.path.insert(0, '../')

from classes.homeostasis_c import Homeostasis
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from tiny_msgs.msg import tinyIMU
import numpy as np
import matplotlib.pyplot as plt
from drawnow import drawnow

class Homeostasis_ROS(object):

    def __init__(self, args):
        self.name = "Homeostasis_ROS"

        # create motor message
        self.motorMessage = Float32MultiArray()

        # create predError message
        self.predErrorMessage = Float32()

        # initialize ros node
        rospy.init_node(self.name)

        # initialize publishers and subscribers
        self.pub = rospy.Publisher("/homeostasis_motor", Float32MultiArray, queue_size=5)
        self.predErrorPub = rospy.Publisher("/predictionError", Float32, queue_size=5)
        self.motorPub = rospy.Publisher("/motor", Float32, queue_size=5)

        self.sub = rospy.Subscriber("/tinyImu", tinyIMU, self.receiveSensorValues)

        # initialize homeokinesis
        self.homeostasis = Homeostasis(dim_m=1, dim_s = 120)
        self.homeostasis.setLearningRates(0.3,0.1)

        if args.verbose:
            print("initialization finished")

        self.minX = np.ones(6) * 100000
        self.maxX = np.ones(6) * -100000
        self.avgX = np.zeros(6)

        self.phase1 = 0
        self.phase2 = 0
        self.motor1 = 0
        self.motor2 = 0

        self.phaseIncrement = 0

        self.seq = 0

    def receiveSensorValues(self, msg):

        self.seq += 1

        #sensorData = np.array(msg.data).reshape((2,1))

        if args.verbose:
            print("SensorData received: " + time.strftime('%X'))
            print(msg)

        self.homeostasis.x = np.roll(self.homeostasis.x, 6)
        self.homeostasis.x[0] = (msg.accel.x + 32768.0) / (2.0 * 32768.0)
        self.homeostasis.x[1] = (msg.accel.y + 32768.0) / (2.0 * 32768.0)
        self.homeostasis.x[2] = (msg.accel.z + 32768.0) / (2.0 * 32768.0)
        self.homeostasis.x[3] = (msg.gyro.x + 32768.0) / (2.0 * 32768.0)
        self.homeostasis.x[4] = (msg.gyro.y + 32768.0) / (2.0 * 32768.0)
        self.homeostasis.x[5] = (msg.gyro.z + 32768.0) / (2.0 * 32768.0)


        print("x: " + str(self.homeostasis.x))

        # recurrent
        #self.homeostasis.x[6] = self.motor1
        #self.homeostasis.x[7] = self.motor2
        #self.homeostasis.x[8] = self.phase1
        #self.homeostasis.x[9] = self.phase2
        #self.homeostasis.x[10] = self.phaseIncrement
        #self.homeostasis.x[11] = self.phaseOffset

        #for i in range(0,6):
        #    if(self.homeostasis.x[i] > self.maxX[i]):
        #        self.maxX[i] = self.homeostasis.x[i]
        #    if(self.homeostasis.x[i] < self.minX[i]):
        #        self.minX[i] = self.homeostasis.x[i]
#
#            self.avgX[i] = self.avgX[i] * 0.99 + self.homeostasis.x[i] * 0.01
#
#            print("x " + str(i) + " min: " + str(self.minX[i]) + " max " + str(self.maxX[i]) + " avg " + str(self.avgX[i]))

        self.homeostasis.learningStep()

        #print("C: " + str(self.homeostasis.controller.C))
        #print("A: %s" % self.homeostasis.model.A)

        # min 0.5 Hz, max 3 Hz:
        # min 1 Pi Rad/s bei 100Hz pi/20 Increments per step
        # max 6 Pi Rad/s bei 100Hz 4pi/20 Increments per step
        # phase Increment is between -1 and 1
        # +1 /2 [0; 1]
        # * 5 pi [0; 5pi]
        # + 1 pi [1pi; 6pi]
        # /100 [1pi/100; 4pi/100]
        # ()((incr + 1)/ 2) * 5 pi + 1 pi) / 100)

        #self.phaseOffset1 +=  self.homeostasis.y[1,0] * np.pi * 2.0
        #print("offset1: " + str(self.phaseOffset1))

        #self.phaseOffset2 +=  self.homeostasis.y[2,0] * np.pi * 2.0
        #print("offset2: " + str(self.phaseOffset2))

        #self.phaseOffset3 +=  self.homeostasis.y[3,0] * np.pi * 2.0
        #print("offset3: " + str(self.phaseOffset3))

        #print("prediction: %s" % self.homeostasis.xPred)

        #print("PredictionError: " + str(self.homeostasis.calculatePredictionError()[0,0]))
        #self.predErrorArray.append(self.homeostasis.calculatePredictionError()[0,0])

        #plt.scatter(self.homeostasis.y.flatten().tolist())

        self.motor1 = -1.0
        if(self.seq > 100 and self.seq < 110):
            self.motor1 = (self.seq - 105.0) / 5.0

        if self.seq >= 110:
            self.motor1 = 1.0

        motor = [self.motor1, self.motor1, self.motor1, self.motor1]

        for m in range(len(motor)):
            motor[m] *= 32768.0 # same coding as imu

        self.motorMessage.data = motor

        self.pub.publish(self.motorMessage)


        # publish prediction error
        self.predErrorMessage.data = self.homeostasis.calculatePredictionError()[0,0]
        self.predErrorPub.publish(self.predErrorMessage)

        # publish phaseIncrement

        if args.verbose:
            print("MotorData sent: " + time.strftime('%X'))
            print(self.motorMessage.data)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = "Ros wrapper for homeostasis class")
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    args = parser.parse_args()

    homeostasis_ros = Homeostasis_ROS(args)

    rospy.spin()
