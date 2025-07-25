#--------------------------------------
# Generate a simple, very smooth reference trajectory for NMPC
#--------------------------------------

import numpy as np

# Parameters
sample_time = 0.05                 # seconds
duration = 60                      # seconds
amp = 3                            # smaller amplitude for gentler curve
frq = 0.03                         # lower frequency for longer, smoother wave

x0 = -50
y0 = 0
z0 = -20

# Trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

# Very gentle S-curve: x increases linearly, y is a low-frequency, low-amplitude sine wave
traj[:,0] = x0 + t                 # x: moves forward from -50
traj[:,1] = y0 + amp * np.sin(2 * np.pi * frq * t) # y: very gentle sine wave
traj[:,2] = z0                     # z: constant depth
traj[:,3] = 0                      # phi
traj[:,4] = 0                      # theta
traj[:,5] = 0                      # psi
traj[:,6] = 1                      # u: constant forward speed
traj[:,7] = amp * 2 * np.pi * frq * np.cos(2 * np.pi * frq * t) # v: y-direction velocity
traj[:,8] = 0                      # w
traj[:,9] = 0                      # p
traj[:,10] = 0                     # q
traj[:,11] = 0                     # r
traj[:,12] = 0                     # u1
traj[:,13] = 0                     # u2
traj[:,14] = 0                     # u3
traj[:,15] = 0                     # u4

# write to txt
np.savetxt('simple.txt', traj, fmt='%f')
