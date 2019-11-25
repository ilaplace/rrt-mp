from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

# #data = numpy.loadtxt('./resources/result.csv', delimiter=",")
# data = numpy.loadtxt('./resources/tree.csv',delimiter=",")
# fig = plt.figure()
# #ax = fig.gca(projection='3d')
# #ax.plot(data[:,1],data[:,2],'.-','.-')
# plt.scatter(data[:,0],data[:,1])
# plt.show()


# Plot cost/itetaion
data = numpy.loadtxt('./resources/costs.txt')
fig = plt.figure()
plt.grid()
plt.xlabel('Number of iterations [x10]')
plt.ylabel('Cost')
#ax = fig.gca(projection='3d')
#ax.plot(data[:,1],data[:,2],'.-','.-')
plt.plot(data[:,0],data[:,1])
plt.show()