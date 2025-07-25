import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load trajectory
data = np.loadtxt('/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/traj/simple.txt')

x = data[:,0]
y = data[:,1]
z = data[:,2]

# 2D plot (x-y)
plt.figure(figsize=(8,3))
plt.plot(x, y, label='Tank Trajectory (x-y)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Tank Trajectory: Top View (x-y)')
plt.xlim(-8.5, 8.5)
plt.ylim(-1.0, 1.0)
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# 3D plot (x-y-z)
fig = plt.figure(figsize=(8,4))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='Tank Trajectory (x-y-z)')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Tank Trajectory: 3D View')
ax.set_xlim(-8.5, 8.5)
ax.set_ylim(-1.0, 1.0)
ax.set_zlim(-0.9, -0.1)
ax.legend()
plt.tight_layout()
plt.show() 