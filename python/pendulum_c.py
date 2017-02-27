#!/usr/bin/env/python
# TODO: NOT TESTED!!!!!!!

class Pendulum():

    def __init__(self):

        # start ~ 90 deg to the right
        self.angle = np.random.uniform(- np.pi/8.0, + np.pi/8.0, size=(1,1))
        self.angleSpeed = np.zeros_like(angle)

        # length of the pendulum
        self.l = 1.5

        # gravitational force
        self.g = -0.01
        self.friction = 0.99
        self.motorTorque = 0.01

    # return measurement with two values (sin and cos of angle)
    def getMeasurement(self):
        x1 = np.sin(angle[0,0])
        x2 = np.cos(angle[0,0])

        x = np.reshape(np.array(x1, x2), (2,-1))

        return x

    # calculate next timestep.
    # Argument: y vector of motor torque percentage (1 max, 0 min)
    def calculateStep(self, y):

        # accelerate with given torque percentage
        self.angleSpeed = self.motorTorque * np.reshape(y[0,0], (1,1))

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

    def getAngle(self):
        return self.angle
