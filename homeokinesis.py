"""
homeostasis example according to playfulmachines, ca. pgs. 65-67. The numbers of the formulas in the comments correspond to the formulas in the book.
control timesteps (i.e. 2000) with --numsteps 2000
control mode with --mode animate|none to have a animation of the pendulum
control disturbance at 1000 timesteps with --disturbance true|false
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# global variable 
x_ = 0
line = 0


def animate(i):
    global x_
    line_x = np.zeros(2)
    line_y = np.zeros(2)
    numsteps = x_.shape[0]
    line_x[1] = x_.reshape(numsteps, -1)[i][0]
    line_y[1] = x_.reshape(numsteps, -1)[i][1]
    line.set_data(line_x, line_y)
    return line,


def init():
    line.set_data([], [])
    return line,


def main(args):
    global x_, line
    ndim_s = 2
    ndim_m = 1

    numsteps = args.numsteps
    mode = args.mode
    disturbance = args.disturbance

    # system
    angle = np.random.uniform(- np.pi / 8.0, + np.pi / 8.0, size=(1, 1))  # start ~ 90 deg to the right
    angleSpeed = np.ones_like(angle) * 0.0
    l = 1.5
    g = -0.01
    friction = 0.99
    motorTorque = 0.02

    # brain
    x = np.zeros((ndim_s, 1))
    xPred = np.zeros_like(x)
    xError = np.zeros_like(x)
    y = np.zeros((ndim_m, 1))

    A = np.zeros([ndim_s, ndim_m])
    b = np.zeros_like(x)

    C = np.random.uniform(-1e-1, 1e-1, size=(ndim_m, ndim_s))
    h = np.random.uniform(-1e-3, 1e-3, size=y.shape)  # ones_like(y) * 0.1

    epsA = 0.03
    epsC = 0.15
    # global angle, angleSpeed, A, C, h, b, y

    # initialize logging variables
    x_ = np.zeros((numsteps,) + x.shape)
    xPred_ = np.zeros((numsteps,) + xPred.shape)
    xError_ = np.zeros((numsteps,) + xError.shape)
    y_ = np.zeros((numsteps,) + y.shape)
    Anorm_ = np.zeros((numsteps, 1))
    Cnorm_ = np.zeros((numsteps, 1))
    angle_ = np.zeros((numsteps,) + angle.shape)
    angleSpeed_ = np.zeros((numsteps,) + angleSpeed.shape)

    for i in range(numsteps):

        # new measurement
        x = np.array([[np.sin(angle[0, 0])], [np.cos(angle[0, 0])]])  # ,[2,1])
        # print("x:", x)

        # calculate prediction error
        xError = x - xPred
        # print("xError: ", xError)

        # Train Model
        dA = epsA * xError * y  # formula 4.5
        A += dA
        db = epsA * xError  # formula 4.6
        b += db

        # calculate norm from A for logging
        Anorm = np.linalg.norm(A, 2)

        # print("|A| = %f, |dA| = %f" % (Anorm, np.linalg.norm(dA, 2)))
        # print("|b| = %f, |db| = %f" % (np.linalg.norm(b, 2), np.linalg.norm(db, 2)))

        # Train Controller
        z = np.dot(C, x) + h  # formula 4.9+ this formula has some strange notion in the book which is a bit confusing. On page 81, this easy form of the formula is presented again.
        # print("z:", z, z.shape)

        # TODO: Finalize homeokinesis
        # preperations for homeokinesis


        g_z = 1 - np.power(np.tanh(z), 2)  # formula 4.9+

        # print("g_z:", g_z, g_z.shape)

        eta = np.dot(A.T, xError) * g_z  # formula 4.9

        # print("eta.shape", eta.shape)

        dC = epsC * np.dot(eta, x.T)  # formula 4.7
        dh = epsC * eta  # formula 4.8

        # print("dC.shape", dC.shape)
        # dC = np.zeros_like(C)
        # dh = np.zeros_like(h)

        C += dC
        h += dh

        Cnorm = np.linalg.norm(C, 2)

        # # TODO: hacky?
        # n = (np.dot(A, g_z) * xError).T
        # print("n:", n)

        # C += epsC * n * 1.0

        # C += epsC * A * (1 - np.power(np.tanh(C * x + h), 2)) * xError * x
        # h += epsC * A * (1 - np.power(np.tanh(C * x + h), 2)) * xError


        # #    print("A b C h:", A, b, C, h)

        # Controler
        y = np.tanh(np.dot(C, x) + h)  # formula 4.3
        # print("y:", y)

        # Feed forward model
        # predict next sensor state
        xPred = np.dot(A, y) + b

        # TODO: In this example, the controler will control the angle directly. Elsewise it would not really be possible, to predict the sensor output from the input (M = Ay + b) since when y is just torque, there is no information about the system. The System would have to be more complex (M = Ax + By + c)

        # angleSpeed += motorTorque * y[0][0]
        angleSpeed = motorTorque * y

        # friction
        angleSpeed *= friction

        # # gravity
        angleSpeed += np.cos(angle) * g

        # add disturbance after 1000 timesteps
        if (i > 1000 and i < 1050 and disturbance):
            angleSpeed += 0.1

        # calculate new position
        angle += angleSpeed

        if (angle > 2.0 * np.pi):
            angle -= 2.0 * np.pi
        if (angle < 0.0):
            angle += 2.0 * np.pi
        # angle = angleSpeed


        # logging
        x_[i] = x
        xPred_[i] = xPred
        xError_[i] = xError
        Anorm_[i] = Anorm
        Cnorm_[i] = Cnorm
        y_[i] = y
        angle_[i] = angle
        angleSpeed_[i] = angleSpeed

    # print("x_.shape", x_.shape)


    plt.figure()
    plt.subplot(511)
    plt.plot([4] * 2000, "k--", alpha=0.5, label="xP0")
    plt.plot(x_.reshape((numsteps, -1)), "k-", alpha=0.5, label="x")
    plt.plot(xPred_.reshape((numsteps, -1)) + 2, "b-", alpha=0.5, label="xP")
    plt.plot(xError_.reshape((numsteps, -1)) + 4, "r-", alpha=0.5, label="xE")
    plt.legend()
    plt.subplot(512)
    plt.plot(y_.reshape((numsteps, -1)), "k-", label="y")
    plt.legend()
    plt.subplot(513)
    plt.plot(angle_.reshape((numsteps, -1)), "k-", label="angle")
    plt.legend()
    plt.subplot(514)
    plt.plot(angleSpeed_.reshape((numsteps, -1)), "k-", label="angledot")
    plt.legend()
    plt.subplot(515)
    plt.plot(Anorm_.reshape((numsteps, -1)), "k-", label="|A|")
    plt.plot(Cnorm_.reshape((numsteps, -1)), "k-", label="|C|")
    plt.legend()

    if (mode == "animate"):
        # animate the pendulum
        fig = plt.figure(figsize=(8, 8))
        ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))
        line, = ax.plot([], [], lw=2)
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=numsteps, interval=20, blit=True)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-ns", "--numsteps", type=int, default=2000)
    parser.add_argument("-m", "--mode", type=str, default="none")
    parser.add_argument("-d", "--disturbance", type=bool, default="false")
    args = parser.parse_args()
    main(args)
