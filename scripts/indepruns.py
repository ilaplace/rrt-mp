from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

## Increasing the weight of rotation in distance
fig = plt.figure()
real = numpy.loadtxt('./resources/realcostsindep.txt')
ofive = numpy.loadtxt('./resources/costsallweightsindepruns.txt')
plt.plot(real[:,0],real[:,2], label='Real cost')
plt.plot(ofive[:,0],ofive[:,2], label='Distance metric', linestyle='dotted')
plt.xlabel(r'Number of iterations $[x 10]$')
plt.ylabel('Time')
plt.grid()
plt.legend()
plt.show()