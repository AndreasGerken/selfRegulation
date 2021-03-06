#!/usr/bin/env python
"""homekinesis with python, compare playfulmachines
$ python hk.py -h"""

''' THIS CODE COMES FROM https://github.com/koro/lpzrobots/blob/master/ode_robots/simulations/ros_connection/hk.py '''

# FIXME: put the learner / control structure into class to easily load
#        der/martius or reservoir model

import time, argparse, sys
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

# TLE = False
# TLE = True

def dtanh(x):
    return 1 - np.tanh(x)**2

def idtanh(x):
    return 1./dtanh(x) # hm?

class LPZRos(object):
    modes = {"hs": 0, "hk": 1, "eh_pi_d": 2}
    def __init__(self, mode="hs"):
        self.name = "lpzros"
        self.mode = LPZRos.modes[mode]
        self.cnt = 0
        ############################################################
        # ros stuff
        rospy.init_node(self.name)
        # pub=rospy.Publisher("/motors", Float64MultiArray, queue_size=1)
        self.pub_motors  = rospy.Publisher("/motors", Float64MultiArray)
        self.sub_sensor = rospy.Subscriber("/sensors", Float64MultiArray, self.cb_sensors)
        # pub=rospy.Publisher("/chatter", Float64MultiArray)
        self.msg=Float64MultiArray()

        ############################################################
        # model + meta params
        self.numsen = 2
        self.nummot = 2
        self.bufsize = 2
        self.creativity = 0.5
        # self.epsA = 0.1
        self.epsA = 0.02
        # self.epsA = 0.001
        # self.epsC = 0.001
        # self.epsC = 0.1
        self.epsC = 0.5

        ############################################################
        # forward model
        self.A = np.eye(self.numsen) * 1.
        self.b = np.zeros((self.numsen,1))
        # controller
        self.C  = np.eye(self.nummot) * 0.4
        self.h  = np.zeros((self.nummot,1))
        self.g  = np.tanh # sigmoidal activation function
        self.g_ = dtanh # derivative of sigmoidal activation function
        # state
        self.x = np.ones ((self.numsen, self.bufsize))
        self.y = np.zeros((self.numsen, self.bufsize))
        self.z = np.zeros((self.numsen, 1))
        # auxiliary variables
        self.L     = np.zeros((self.numsen, self.nummot))
        self.v_avg = np.zeros((self.numsen, 1))
        self.xsi   = np.zeros((self.numsen, 1))

    def cb_sensors(self, msg):
        """lpz sensors callback: receive sensor values, sos algorithm attached"""
        # self.msg.data = []
        self.x = np.roll(self.x, 1, axis=1) # push back past
        self.y = np.roll(self.y, 1, axis=1) # push back past
        # update with new sensor data
        self.x[:,0] = msg.data
        # compute new motor values
        x_tmp = np.atleast_2d(self.x[:,0]).T + self.v_avg * self.creativity
        # print self.g(np.dot(self.C, x_tmp) + self.h)
        self.y[:,0] = self.g(np.dot(self.C, x_tmp) + self.h).reshape((self.nummot,))

        self.cnt += 1
        if self.cnt <= 2: return

        # print "x", self.x
        # print "y", self.y

        # local variables
        x = np.atleast_2d(self.x[:,1]).T
        y = np.atleast_2d(self.y[:,1]).T
        x_fut = np.atleast_2d(self.x[:,0]).T

        # print "x", x.shape, x, x_fut.shape, x_fut
        z = np.dot(self.C, x + self.v_avg * self.creativity) + self.h
        # z = np.dot(self.C, x)
        # print z.shape, x.shape
        # print z - x

        g_prime = dtanh(z) # derivative of g
        g_prime_inv = idtanh(z) # inverse derivative of g

        print "g_prime", self.cnt, g_prime
        print "g_prime_inv", self.cnt, g_prime_inv

        xsi = x_fut - (np.dot(self.A, y) + self.b)
        print "xsi =", xsi

        # forward model learning
        self.A += self.epsA * np.dot(xsi, y.T) + (self.A * -0.003) * 0.1
        # self.A += self.epsA * np.dot(xsi, np.atleast_2d(self.y[:,0])) + (self.A * -0.003) * 0.1
        self.b += self.epsA * xsi              + (self.b * -0.001) * 0.1

        print "A", self.cnt, self.A
        # print "b", self.b

        if self.mode == 1: # TLE / homekinesis
            eta = np.dot(np.linalg.pinv(self.A), xsi)
            zeta = np.clip(eta * g_prime_inv, -1., 1.)
            print "eta", self.cnt, eta
            print "zeta", self.cnt, zeta
            # print "C C^T", np.dot(self.C, self.C.T)
            # mue = np.dot(np.linalg.pinv(np.dot(self.C, self.C.T)), zeta)
            lambda_ = np.eye(2) * np.random.uniform(-0.01, 0.01, 2)
            mue = np.dot(np.linalg.pinv(np.dot(self.C, self.C.T) + lambda_), zeta)
            v = np.clip(np.dot(self.C.T, mue), -1., 1.)
            self.v_avg += (v - self.v_avg) * 0.1
            print "v", self.cnt, v
            print "v_avg", self.cnt, self.v_avg
            EE = 1.0

            # print EE, v
            if True: # logarithmic error
                # EE = .1 / (np.sqrt(np.linalg.norm(v)) + 0.001)
                EE = .1 / (np.square(np.linalg.norm(v)) + 0.001)
            # print EE
            # print "eta", eta
            # print "zeta", zeta
            # print "mue", mue

            dC = (np.dot(mue, v.T) + (np.dot((mue * y * zeta), -2 * x.T))) * EE * self.epsC
            dh = mue * y * zeta * -2 * EE * self.epsC

            # pass
            # dC = np.zeros_like(self.C)
            # dh = np.zeros_like(self.h)

        elif self.mode == 0: # homestastic learning
            eta = np.dot(self.A.T, xsi)
            print "eta", self.cnt, eta.shape, eta
            dC = np.dot(eta * g_prime, x.T) * self.epsC
            dh = eta * g_prime * self.epsC
            # print dC, dh
            # self.C +=

        self.h += np.clip(dh, -.1, .1)
        self.C += np.clip(dC, -.1, .1)

        # print "C", self.C
        # print "h", self.h
        # print "y", self.y

        # self.msg.data.append(m[0])
        # self.msg.data.append(m[1])
        self.msg.data = self.y[:,0].tolist()
        # print("sending msg", msg)
        self.pub_motors.publish(self.msg)
        # time.sleep(0.1)
        # if self.cnt > 20:
        #     rospy.signal_shutdown("stop")
        #     sys.exit(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="lpzrobots ROS controller: test homeostatic/kinetic learning")
    parser.add_argument("-m", "--mode", type=str, help="select mode: " + str(LPZRos.modes))
    # parser.add_argument("-m", "--mode", type=int, help="select mode: ")
    args = parser.parse_args()

    # sanity check
    if not args.mode in LPZRos.modes:
        print "invalid mode string, use one of " + str(LPZRos.modes)
        sys.exit(0)

    lpzros = LPZRos(args.mode)
    rospy.spin()
    # while not rospy.shutdown():
