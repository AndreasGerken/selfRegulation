from classes.pendulum_c import Pendulum
from classes.homeostasis_c import Homeostasis
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# initialize variables
dim_m = 1
dim_s = 2

# global variable
pendulum = Pendulum()
homeostasis = Homeostasis(dim_m, dim_s)
controller = homeostasis.getController()
model = homeostasis.getModel()
line = 0

def animate(i):
    global pendulum
    global homeostasis
    homeostasis.x = pendulum.getMeasurement().reshape(2,1)
    homeostasis.learningStep()

    line_x = np.zeros(2)
    line_y = np.zeros(2)
    line_x[1] = homeostasis.x[0]
    line_y[1] = homeostasis.x[1]
    line.set_data(line_x, line_y)
    if(i > 200 and i < 400):
        pendulum.calculateStep(np.ones((1,1)))
    else:
        pendulum.calculateStep(homeostasis.y)
    return line,

def init():
    line.set_data([], [])
    return line,



numsteps = 2000



# animate the pendulum
fig = plt.figure(figsize=(8, 8))
ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))
line, = ax.plot([], [], lw=2)
anim = animation.FuncAnimation(fig, animate, init_func=init, frames=numsteps, interval=20, blit=True)
plt.show()
