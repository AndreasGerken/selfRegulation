import sys
sys.path.insert(0, '../')

from classes.pendulum_c import Pendulum
import numpy as np
import matplotlib.pyplot as plt

pendulum = Pendulum()
steps = 1000
angles = [0]*steps

# start and stop time of the motor
start = 3 * steps / 8
end = 5 * steps / 8

for i in range(0,steps):
    # save the pendulum angle
    angles[i] = pendulum.angle[0,0]

    if(i > start and i < end):
        # activate the motor and calculate next step
        pendulum.calculateStep(np.ones((1,1))*0.987)
    else:
        pendulum.calculateStep(np.ones((1,1))*0)

plt.figure()

# plot the angles
plt.plot(angles)

# plot the area where the motor was active
plt.plot([start ] * 2,[0,2*np.pi], 'r-')
plt.plot([end] * 2, [0, 2 * np.pi], 'r-')

plt.show()
