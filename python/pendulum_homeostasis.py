from pendulum_c import Pendulum
from homeostasis_c import Homeostasis
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# initialize variables
dim_m = 1
dim_s = 2

pendulum = Pendulum()
homeostasis = Homeostasis(dim_m, dim_s)
controller = homeostasis.getController()
model = homeostasis.getModel()

# animate the pendulum
fig = plt.figure(figsize=(8, 8))
ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))
line, = ax.plot([], [], lw=2)
anim = animation.FuncAnimation(fig, animate, init_func=init, frames=numsteps, interval=20, blit=True)

def animate(i):
    global x_
    line_x = np.zeros(2)
    line_y = np.zeros(2)
    numsteps = x_.shape[0]
    line_x[1] = x_.reshape(numsteps, -1)[i][0]
    line_y[1] = x_.reshape(numsteps, -1)[i][1]
    line.set_data(line_x, line_y)
    return line,
