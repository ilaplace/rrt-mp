from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

#rrtmp = numpy.loadtxt('./resources/costs.txt')
# rrt = numpy.loadtxt('../omplControl/resources/costs.txt')

# fig = plt.figure()

# plt.plot(rrtmp[:,0],rrtmp[:,2], label='RRT*mp')
# plt.plot(rrt[:,0],rrt[:,1], label='RRT', linestyle='dotted')
# plt.xlabel(r'Number of iterations $[x 10^4]$')
# plt.ylabel('Computation Time [s]')
# plt.grid()
# plt.legend()
# plt.show()

## Increasing the weight of rotation in distance
fig = plt.figure()
zero = numpy.loadtxt('./resources/costswithoutrot.txt')
ofive = numpy.loadtxt('./resources/costswithrot05.txt')
sevfive = numpy.loadtxt('./resources/costwithrot075.txt')
one = numpy.loadtxt('./resources/costswithrot.txt')
plt.plot(zero[:,0],zero[:,2], label='0.0')
plt.plot(ofive[:,0],ofive[:,2], label='0.5', linestyle='dotted')
plt.plot(sevfive[:,0],sevfive[:,2], label='0.75', linestyle='dashed')
plt.plot(one[:,0],one[:,2], label='1.0', linestyle='dashdot')
plt.xlabel(r'Number of iterations $[x 10]$')
plt.ylabel('Time [s]')
plt.grid()
plt.legend()
plt.show()