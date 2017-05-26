import numpy as np
import sklearn
import matplotlib.pyplot as plt

mean = [0,0]
cov = [[1, 0], [0, 1]]
x, y = np.random.multivariate_normal(mean, cov, 5000).T

plt.plot(x,y, "b.")
plt.show()
