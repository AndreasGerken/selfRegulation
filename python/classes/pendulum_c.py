#!/usr/bin/env/python

# This is a controlable pendulum as a testsystem for self learning algorithms
# The pendulum has one motor degree from -1 to 1 and two sensor degrees from -1 to 1

import numpy as np

class Pendulum():

    def __init__(self):
        # motor and sensor dimensions for outside use
        self.dim_m = 1
        self.dim_s = 2

        # start ~ 90 deg to the right
        self.angle = np.random.uniform(- np.pi/8.0, + np.pi/8.0, size=(self.dim_m,1))
        self.angleSpeed = np.zeros_like(self.angle)

        # length of the pendulum
        self.l = 1.5

        # gravitational force
        self.g = -0.01
        self.friction = 0.95
        self.motorTorque = 0.01

    # return measurement with two values (sin and cos of angle)
    def getMeasurement(self):
        x1 = np.cos(self.angle[0,0])
        x2 = np.sin(self.angle[0,0])

        x = np.reshape(np.array([x1, x2]), (self.dim_s,1))
        return x

    # calculate next timestep.
    # Argument: y vector of motor torque percentage (1 max, 0 min)
    def calculateStep(self, y):

        # accelerate with given torque percentage
        self.angleSpeed += self.motorTorque * np.reshape(y[0,0], (1,1))

        # friction
        self.angleSpeed *= self.friction

        # gravity
        self.angleSpeed += np.cos(self.angle) * self.g

        # move pendulum with calculated speed
        self.angle += self.angleSpeed

        # reset angle after full rotation
        if(self.angle > 2.0 * np.pi):
            self.angle -= 2.0 * np.pi
        if(self.angle < 0.0):
            self.angle += 2.0 * np.pi
