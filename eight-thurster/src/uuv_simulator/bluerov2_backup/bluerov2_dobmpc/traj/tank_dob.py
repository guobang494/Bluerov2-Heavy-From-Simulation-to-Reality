#--------------------------------------
# Generate an extra-smooth trajectory for tank environment
# Tank dimensions: 18.15m x 2.5m x 1m
#--------------------------------------

import numpy as np

# Parameters
sample_time = 0.05                 # seconds
duration = 120                     # seconds
amp_x = 6.0                        # smaller amplitude for x direction (gentler)
amp_y = 0.6                        # smaller amplitude for y direction (gentler)
frq_x = 0.008                      # lower frequency for x direction (slower, smoother)
frq_y = 0.012                      # lower frequency for y direction

# Starting position (tank edge, middle width, middle height)
x0 = -8                            # near south wall
y0 = 0                             # middle of tank width
z0 = -0.5                          # middle of tank height

# Trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

# Very gentle, slow oscillation
traj[:,0] = x0 + amp_x * np.sin(2 * np.pi * frq_x * t)     # x: very gentle oscillation
traj[:,1] = y0 + amp_y * np.sin(2 * np.pi * frq_y * t)     # y: very gentle oscillation
traj[:,2] = z0                                             # z: constant depth
traj[:,3] = 0                                              # phi
traj[:,4] = 0                                              # theta
traj[:,5] = 0                                              # psi
traj[:,6] = amp_x * 2 * np.pi * frq_x * np.cos(2 * np.pi * frq_x * t)  # u: x velocity
traj[:,7] = amp_y * 2 * np.pi * frq_y * np.cos(2 * np.pi * frq_y * t)  # v: y velocity
traj[:,8] = 0                                              # w
traj[:,9] = 0                                              # p
traj[:,10] = 0                                             # q
traj[:,11] = 0                                             # r
traj[:,12] = 0                                             # u1
traj[:,13] = 0                                             # u2
traj[:,14] = 0                                             # u3
traj[:,15] = 0                                             # u4

# Ensure trajectory stays within tank bounds
for i in range(len(traj)):
    traj[i,0] = np.clip(traj[i,0], -8.5, 8.5)
    traj[i,1] = np.clip(traj[i,1], -1.0, 1.0)
    traj[i,2] = np.clip(traj[i,2], -0.9, -0.1)

# write to txt
np.savetxt('tank_dob.txt', traj, fmt='%f')

print("Tank trajectory generated:")
print(f"X range: {np.min(traj[:,0]):.2f} to {np.max(traj[:,0]):.2f}")
print(f"Y range: {np.min(traj[:,1]):.2f} to {np.max(traj[:,1]):.2f}")
print(f"Z range: {np.min(traj[:,2]):.2f} to {np.max(traj[:,2]):.2f}")
print(f"Duration: {duration} seconds")
print(f"Total points: {len(traj)}")
