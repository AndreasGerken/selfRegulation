import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

fig = plt.figure()
ax = plt.axes(xlim=(-2,2), ylim=(-2, 2))
line, = ax.plot([], [], lw =2)

angle = 10
angleSpeed =0.1
l = 1
g = -0.01
friction = 0.97
motorTorque = 0.009

A = 0
C = 0
h = 0.1 
b = 0
y = 0

epsA = 0.0002
epsC = 0.002

def init():
	line.set_data([], [])
	angle = 20
	angleSpeed = 0.1
	l = 1.5
	return line,

def animate(i):
	global angle, angleSpeed, A, C, h, b, y

	x[0] = np.sin(angle)
	x[1] = np.cos(angle)

	## Feed forward model

	if(i != 0):
		xPred = A * y + b
		xError = np.power(x - xPred,2)
		print(xError)
		
		## Train Model
		A += epsA * xError * y
		b += epsA * xError

		## Train Controller
		C += epsC * A * (1 - np.power(np.tanh(C * x + h),2)) * xError * x
		h += epsC * A * (1 - np.power(np.tanh(C * x + h),2)) * xError

		print("A b C h:",A,b,C,h)
	
	## Control ##

	# K(x) = tanh(Cx + h)
	y = np.tanh(C * x + h)
	print(y)


	## Dynamics model ##
	
	# gravity
	angleSpeed += np.cos(angle) * g

	# motor Torque
	if(int(i/200)%2==0):
		y = 1
	else:
		y = 0
	angleSpeed += motorTorque * y

	# friction
	angleSpeed *= friction

	# calculate new position
	angle += angleSpeed

	line_x = np.zeros(2)
	line_y = np.zeros(2)
	line_x[1] = np.cos(angle) * l
	line_y[1] = np.sin(angle) * l

	line.set_data(line_x,line_y)
	return line,

anim = animation.FuncAnimation(fig, animate, init_func = init, frames = 400, interval = 20, blit = True)


plt.show()
