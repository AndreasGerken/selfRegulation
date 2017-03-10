#!/usr/bin/env/python
# TODO: NOT TESTED!!!!!!!

import numpy as np

class Homeostasis():

    # enum class for controller variables
    class Controller():
        pass

    # enum class for model variables
    class Model():
        pass


    def __init__(self, dim_m, dim_s):

        # initialize dimensions
        self.dim_m = dim_m
        self.dim_s = dim_s

        # initialize "brain" variables
        self.x = np.zeros((self.dim_s, 1))   # sensor vector
        self.xPred = np.zeros_like(self.x)    # predicted sensor vector
        self.y = np.zeros((self.dim_m, 1))   # motor vector

        # initialize model variables
        self.model = self.Model()
        self.model.A = np.zeros([self.dim_s, self.dim_m])
        self.model.b = np.zeros_like(self.x)

        # initialize controller variables
        self.controller = self.Controller()
        self.controller.C = np.random.uniform(-1e-1, 1e-1, size=(self.dim_m, self.dim_s))  # random
        self.controller.h = np.random.uniform(-1e-2, 1e-2, size=self.y.shape)

        # initialize learning variables
        # TODO: should this be done here?
        self.setLearningRates(0.05, 0.15)

    # Setter for learning rates of the model and the controller
    def setLearningRates(self, epsA, epsC):
        self.model.eps = epsA    # learning rate for model
        self.controller.eps = epsC    # learning rate for controller

    # calculates and returns the prediction error (difference between predicted)
    # and real sensor values
    def calculatePredictionError(self):
        return self.x - self.xPred

    # trains the model (in this case A and b) from the prediction error
    def trainModel(self, _xError):
        _dA = self.controller.eps * _xError * self.y.T
        _db = self.controller.eps * _xError

        self.model.A += _dA
        self.model.b += _db

    # trains the controller (in this case C and h) from the prediction error
    def trainController(self, _xError):
        _z = np.dot(self.controller.C, self.x) + self.controller.h
        _g_z = 1 - np.power(np.tanh(_z), 2)
        _eta = np.dot(self.model.A.T, _xError) * _g_z

        _dC = self.controller.eps * np.dot(_eta, self.x.T)
        _dh = self.controller.eps * _eta

        self.controller.C += _dC
        self.controller.h += _dh

    # calculate new motor command and return it
    def calculateMotorCommand(self):
        return np.tanh(np.dot(self.controller.C, self.x) + self.controller.h)

    # calculate new prediction (about the sensors) and return it
    def calculatePrediction(self):
        return np.dot(self.model.A, self.y) + self.model.b

    # unifies all necessary steps to learn from new data.
    # before, new sensor values shall be read,
    # afterwards the motor commands should be sent.
    def learningStep(self):

        # calculate prediction error
        _xError = self.calculatePredictionError()

        # train model
        self.trainModel(_xError)

        # train controller
        self.trainController(_xError)

        # calculate next motor command
        self.y = self.calculateMotorCommand()

        # calculate new prediction from model
        self.xPred = self.calculatePrediction()

    # getter for controller
    def getController(self): return self.controller

    # getter for model
    def getModel(self):
        return self.model
