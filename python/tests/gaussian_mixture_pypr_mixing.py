from numpy import *
from matplotlib.pylab import *
import pypr.clustering.gmm as gmm




def generateData(n):
    mc = [0.4, 0.4, 0.2] # Mixing coefficients
    centroids = [ array([0,0]), array([3,3]), array([0,4]) ]
    ccov = [ array([[1,0.4],[0.4,1]]), diag((1,2)), diag((0.4,0.1)) ]

    # Generate samples from the gaussian mixture model
    X = gmm.sample_gaussian_mixture(centroids, ccov, mc, samples=n)
    return X

def plotCenters(cen_lst, cov_lst, c, offset):
    for i in range(len(cen_lst)):
        text(cen_lst[i][0], cen_lst[i][1], str(i+1 + offset), horizontalalignment='center',
            verticalalignment='center', size=32, color=(0.2,0,0))
        ex,ey = gmm.gauss_ellipse_2d(cen_lst[i], cov_lst[i])
        plot(ex, ey, c, linewidth=0.5)

def plotData(X, c):

    plot(X[:,0], X[:,1],'.', c = c, alpha = 0.2)




# create data

#seed(1)

fig1 = figure()

X = generateData(70)
plotData(X, 'b')
cen_lst_1, cov_lst_1, p_k_1, logL_1 = gmm.em_gm(X, K = 3, max_iter = 400, \
verbose = True, iter_call = None)
plotCenters(cen_lst_1, cov_lst_1, 'b', 0)


X = generateData(70)
plotData(X, 'y')
cen_lst_2, cov_lst_2, p_k_2, logL_2 = gmm.em_gm(X, K = 3, max_iter = 400, \
verbose = True, iter_call = None)
plotCenters(cen_lst_2, cov_lst_2, 'y', 3)



# get conditional distribution
#y = 0.3
#(con_cen, con_cov, new_p_k) = gmm.cond_dist(np.array([y,np.nan]), cen_lst, cov_lst, p_k)

#samples = gmm.sample_gaussian_mixture(con_cen, con_cov, new_p_k, samples = 20)

#plot( np.ones_like(samples) * y, samples,'.r')

show()
