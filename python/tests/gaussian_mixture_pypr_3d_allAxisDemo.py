# An example of using the Expectation Maximization (EM) algorithm to find the
# parameters for a mixture of gaussian given a set of samples.

from numpy import *
from matplotlib.pylab import *
import pypr.clustering.gmm as gmm
from mpl_toolkits.mplot3d import Axes3D

def iter_plot(cen_lst, cov_lst, itr):
    # For plotting EM progress
    if itr % 2 == 0:
        for i in range(len(cen_lst)):
            x,y = gmm.gauss_ellipse_2d(cen_lst[i], cov_lst[i])
            plot(x, y, 'k', linewidth=0.5)

seed(1)
mc = [0.4, 0.4, 0.2] # Mixing coefficients
centroids = [ array([0,0,0]), array([3,3,2]), array([0,4,3]) ]
ccov = [ array([[1,0.4,0.4],[0.4,1, 0.4],[0.4,0.4,1]]), diag((1,2,0.4)), diag((0.4,0.1,1)) ]

# Generate samples from the gaussian mixture model
X = gmm.sample_gaussian_mixture(centroids, ccov, mc, samples=5000)
fig = figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X[:500,0].tolist(), X[:500,1].tolist(), X[:500,2].tolist(), c='k', alpha=0.1, edgecolors='none')

# Expectation-Maximization of Mixture of Gaussians
cen_lst, cov_lst, p_k, logL = gmm.em_gm(X, K = 3, max_iter = 400, verbose = True, iter_call = None)
print "Log likelihood (how well the data fits the model) = ", logL


#ax.scatter(centroids[:][0] ,centroids[:][1] ,centroids[:][2], c = 'y', s=60)

# Plot the cluster ellipses
for i in range(len(centroids)):
    ax.scatter(centroids[i][0] ,centroids[i][1] ,centroids[i][2], c = 'y', s=60, alpha= 0.5, edgecolors='none')

for i in range(len(cen_lst)):
    ax.scatter(cen_lst[i][0] ,cen_lst[i][1] ,cen_lst[i][2], c = 'g', s=60, alpha = 0.5, edgecolors='none')


# Now we will find the conditional distribution of x given y
#fig2 = figure()
#ax1 = subplot(111)
#plot(X[:,0], X[:,1], ',')
x = -1.0
# axhline(y)
#x1plt = np.linspace(axis()[0], axis()[1], 200)
# for i in range(len(cen_lst)):
#     text(cen_lst[i][0], cen_lst[i][1], str(i+1), horizontalalignment='center',
#         verticalalignment='center', size=32, color=(0.2,0,0))
#     ex,ey = gmm.gauss_ellipse_2d(cen_lst[i], cov_lst[i])
#     plot(ex, ey, 'k', linewidth=0.5)
# ax2 = twinx()

for x in range(-4,8):
    (con_cen, con_cov, new_p_k) = gmm.cond_dist(np.array([x, np.nan, np.nan]), \
            cen_lst, cov_lst, p_k)
    #samples = gmm.sample_gaussian_mixture(con_cen, con_cov, new_p_k, samples = 20)

    # add fixed values
    #samples = np.hstack((np.ones((samples.shape[0], 1)) * x, samples))

    #new_p_k += np.max(new_p_k) / 5
    #new_p_k /= np.max(new_p_k)

    #for i in range(len(samples)):
    #    ax.scatter(samples[i][0] ,samples[i][1] ,samples[i][2], c = 'r', alpha = 0.1, edgecolors='none')

    for i in range(len(con_cen)):
        ex,ey = gmm.gauss_ellipse_2d(con_cen[i], con_cov[i])
        ax.plot(np.ones_like(ex) * x, ex, ey, 'b', linewidth=1, alpha = new_p_k[i] *  0.8)
for y in range(-4,8):
    (con_cen, con_cov, new_p_k) = gmm.cond_dist(np.array([np.nan, y, np.nan]), \
            cen_lst, cov_lst, p_k)
    #samples = gmm.sample_gaussian_mixture(con_cen, con_cov, new_p_k, samples = 20)

    # add fixed values
    #samples = np.hstack((np.ones((samples.shape[0], 1)) * x, samples))

    #new_p_k += np.max(new_p_k) / 5
    #new_p_k /= np.max(new_p_k)

    #for i in range(len(samples)):
    #    ax.scatter(samples[i][0] ,samples[i][1] ,samples[i][2], c = 'r', alpha = 0.1, edgecolors='none')

    for i in range(len(con_cen)):
        ex,ey = gmm.gauss_ellipse_2d(con_cen[i], con_cov[i])
        ax.plot( ex,np.ones_like(ex) * y, ey, 'r', linewidth=1, alpha = new_p_k[i] * 0.8)

for z in range(-4,8):
    (con_cen, con_cov, new_p_k) = gmm.cond_dist(np.array([np.nan, np.nan, z]), \
            cen_lst, cov_lst, p_k)
    #samples = gmm.sample_gaussian_mixture(con_cen, con_cov, new_p_k, samples = 20)

    # add fixed values
    #samples = np.hstack((np.ones((samples.shape[0], 1)) * x, samples))

    #new_p_k += np.max(new_p_k) / 5
    #new_p_k /= np.max(new_p_k)

    #for i in range(len(samples)):
    #    ax.scatter(samples[i][0] ,samples[i][1] ,samples[i][2], c = 'r', alpha = 0.1, edgecolors='none')

    for i in range(len(con_cen)):
        ex,ey = gmm.gauss_ellipse_2d(con_cen[i], con_cov[i])
        ax.plot( ex,ey,np.ones_like(ex) * z,  'y', linewidth=1, alpha = new_p_k[i]  *0.8)



#x2plt = gmm.gmm_pdf(c_[x1plt], con_cen, con_cov, new_p_k)
#ax.plot(x1plt, x2plt,'r', linewidth=2)


ax.legend()

show()
