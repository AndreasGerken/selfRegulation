import warnings
import matplotlib.pyplot as plt

import numpy as np
from numpy.random import randn

from matplotlib.lines import Line2D
from numpy import array, eye, pi, cos, linspace


from explauto.models.gmminf import GMM


x = linspace(-2, 2, 100)
y = cos(x) + 0.1 * randn(*x.shape)
data = array([x, y]).T

# Fit a GMM P(XY). Our GMM class is a subclasse of sklearn.mixture.GMM, so you can use all the methods from this latter
gmm = GMM(n_components=3, covariance_type='full')
gmm.fit(data)

def predict_x(gmm, y):
    # Infer the gmm P(X | Y=value_y) (the infered gmm is therefore 1D)
    gmm_inf = gmm.inference([1], [0], array(y))
    (gmm_inf)

print(predict_x(gmm,0))
