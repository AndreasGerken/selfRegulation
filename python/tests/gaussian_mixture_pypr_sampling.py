from numpy import *
from matplotlib.pylab import *
import pypr.clustering.gmm as gmm


# create data

seed(1)
mc = [0.4, 0.4, 0.2] # Mixing coefficients
centroids = [ array([0,0]), array([3,3]), array([0,4]) ]
ccov = [ array([[1,0.4],[0.4,1]]), diag((1,2)), diag((0.4,0.1)) ]

# Generate samples from the gaussian mixture model
X = gmm.sample_gaussian_mixture(centroids, ccov, mc, samples=1000)
fig1 = figure()
plot(X[:,0], X[:,1], '.', alpha = 0.2)

cen_lst, cov_lst, p_k, logL = gmm.em_gm(X, K = 3, max_iter = 400, \
verbose = True, iter_call = None)

x1plt = np.linspace(axis()[0], axis()[1], 200)

for i in range(len(cen_lst)):
    text(cen_lst[i][0], cen_lst[i][1], str(i+1), horizontalalignment='center',
        verticalalignment='center', size=32, color=(0.2,0,0))
    ex,ey = gmm.gauss_ellipse_2d(cen_lst[i], cov_lst[i])
    plot(ex, ey, 'k', linewidth=0.5)

# get conditional distribution
y = 0.3
(con_cen, con_cov, new_p_k) = gmm.cond_dist(np.array([y,np.nan]), \
        cen_lst, cov_lst, p_k)

x2plt = gmm.gmm_pdf(c_[x1plt], con_cen, con_cov, new_p_k)

x2plt/=np.sum(x2plt)

newSamples = np.random.choice(x1plt.tolist(), 20, p = x2plt.tolist())

plot( np.ones_like(newSamples) * y, newSamples,'.r')

ax2 =twiny()

plot(x2plt, x1plt, '-g')
show()
