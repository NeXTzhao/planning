import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

plt.rcParams['axes.unicode_minus'] = False

fig = plt.figure()
ax = Axes3D(fig)

#  x,y value
x = np.arange(-4, 4, 0.25)
y = np.arange(-4, 4, 0.25)
x, y = np.meshgrid(x, y)
R = np.sqrt(x ** 2 + y ** 2)
z = np.sin(R)

ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
ax.contourf(x, y, z, zdir='z', offset=-2, cmap='rainbow')
ax.set_zlim(-2, 2)
plt.show()
