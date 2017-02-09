from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig = plt.figure()
ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))
line, = ax.plot([], [], lw=2)

angle = 10
angleSpeed = 0.5
l = 1
g = -0.01
friction = 0.97
motorTorque = 0.009

A = np.zeros([2, 1])
b = np.zeros([2, 1])

C = np.ones([1, 2])
h = 0.1

y = 0

epsA = 0.0002
epsC = 0.002


def init():
    line.set_data([], [])
    return line,


def animate(i):
    global angle, angleSpeed, A, C, h, b, y
    x = np.reshape([np.sin(angle), np.cos(angle)],[2,1])
    print("x:", x)

    # Feed forward model

    if (i != 0):
        xPred = np.dot(A, y) + b
        xError = x - xPred
        print("xError: ", xError)

        # xError = np.dot(xError.T, xError)

        # print("xError: ", xError)

        # Train Model
        A += epsA * xError * y
        b += epsA * xError

        # Train Controller
        z = np.dot(C, x) + h
        g_z = 1 - np.power(np.tanh(z),2)
        print("g_z:",g_z)

        # TODO: hacky?
        n = (np.dot(A, g_z) * xError).T
        print("n:", n)

        C += epsC * n *

        C += epsC * A * (1 - np.power(np.tanh(C * x + h), 2)) * xError * x
        h += epsC * A * (1 - np.power(np.tanh(C * x + h), 2)) * xError

    #    print("A b C h:", A, b, C, h)

    # Control ##

    # K(x) = tanh(Cx + h)
    # print(C.shape, x.shape, h.shape)
    y = np.tanh(np.dot(C, x) + h)
    print("y:", y)

    # Dynamics model ##

    # gravity
    angleSpeed += np.cos(angle) * g

    # motor Torque
    #if (int(i / 200) % 2 == 0):
    #    angleSpeed += 0.1
    # else:
    #	y = 0

    angleSpeed += motorTorque * y[0][0]
    # friction
    angleSpeed *= friction

    # calculate new position
    angle += angleSpeed

    line_x = np.zeros(2)
    line_y = np.zeros(2)

    print("angle:",angle)
    line_x[1] = np.cos(angle) * l
    line_y[1] = np.sin(angle) * l

    line.set_data(line_x, line_y)
    return line,


anim = animation.FuncAnimation(fig, animate, init_func=init, frames=400, interval=20, blit=True)

plt.show()
