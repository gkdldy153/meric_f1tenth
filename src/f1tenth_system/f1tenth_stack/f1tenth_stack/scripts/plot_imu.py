import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the unit axes
ax.quiver(0, 0, 0, 1, 0, 0, color='r', length=1.0, normalize=True, label='X axis')
ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=1.0, normalize=True, label='Y axis')
ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=1.0, normalize=True, label='Z axis')

ax.set_xlim([0, 1.1])
ax.set_ylim([0, 1.1])
ax.set_zlim([0, 1.1])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Right-Handed Coordinate Frame')

ax.legend()
plt.show()
