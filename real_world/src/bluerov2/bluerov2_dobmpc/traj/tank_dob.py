# -*- coding: utf-8 -*-
# --------------------------------------
# Figure-8 reference trajectory (vertical lemniscate) for NMPC in Tank
# Tank: 2.5m x 18.15m x 1m   (x=short, y=long)
# Requirements satisfied:
#  - Centered at (0,0,z0)
#  - x range: [-0.45, +0.45]
#  - y range: [-6.5, +6.5]
#  - z fixed at 0.23
#  - Start/End at (x0, y0, z0) via smooth connect-in/out
#  - Vertical figure-8 (上半环 / 下半环)，不是左右两半
# --------------------------------------

import numpy as np
import matplotlib.pyplot as plt

# -------- Parameters --------
sample_time = 0.05
duration_total = 300.0                  # 从180s增加到300s - 更长总时间
T_in  = 20.0                            # 从10s增加到20s - 更慢的起始阶段
T_out = 20.0                            # 从10s增加到20s - 更慢的结束阶段
T_loop = 120.0                          # 从60s增加到120s - 更慢的八字循环
loops = int((duration_total - T_in - T_out) // T_loop)
T_mid = loops * T_loop                  # total time on the figure-8

# amplitudes (STRICT)
Ax = 0.5                                # x amplitude => x in [-0.45, +0.45]
Ay = 3                                 # y amplitude => y in [-6.5, +6.5]

# Initial pose
x0 = -0.44
y0 = -2.80
z0 = 0.23



# Tank bounds (for plotting reference)
tank_x_min, tank_x_max = -1.25, 1.25
tank_y_min, tank_y_max = -9.075, 9.075
tank_z_min, tank_z_max = -1.0, 0.0

# --------------------------------------
# Helper: quintic (min-jerk) blend s(τ)=6τ^5-15τ^4+10τ^3, τ in [0,1]
def sblend(t, T):
    tau = np.clip(t / max(T, 1e-9), 0.0, 1.0)
    return 6*tau**5 - 15*tau**4 + 10*tau**3

# Vertical figure-8 (Gerono) centered at (0,0):
# y = Ay * sin(θ)         -> 上半（y>0）与下半（y<0）
# x = Ax * sin(θ)*cos(θ)  = (Ax/2) * sin(2θ)  -> x在 ±Ax 内摆动
def fig8_xy(theta):
    y = Ay * np.sin(theta)
    x = Ax * np.sin(theta) * np.cos(theta)
    return x, y

# Build time arrays
N_in  = int(np.round(T_in / sample_time))
N_mid = int(np.round(T_mid / sample_time))
N_out = int(np.round(T_out / sample_time))
N_tot = N_in + N_mid + N_out + 1

t = np.arange(0.0, N_tot) * sample_time
traj = np.zeros((N_tot, 16))  # x y z phi theta psi u v w p q r u1 u2 u3 u4

# --------------------------------------
# Phase A: smooth connect-in (x0,y0) -> (0,0)
for i in range(N_in):
    ti = i * sample_time
    s = sblend(ti, T_in)
    traj[i, 0] = (1 - s) * x0
    traj[i, 1] = (1 - s) * y0
    traj[i, 2] = z0
# ensure final sample hits exactly (0,0) before loop
traj[max(N_in-1, 0), 0] = 0.0
traj[max(N_in-1, 0), 1] = 0.0
traj[max(N_in-1, 0), 2] = z0

# --------------------------------------
# Phase B: figure-8 loops (vertical)
# Start at θ=0 => (x,y)=(0,0), then run 'loops' complete 8s
if N_mid > 0:
    theta = np.linspace(0.0, 2*np.pi*loops, N_mid, endpoint=False)
    x_mid, y_mid = fig8_xy(theta)
    traj[N_in:N_in+N_mid, 0] = x_mid
    traj[N_in:N_in+N_mid, 1] = y_mid
    traj[N_in:N_in+N_mid, 2] = z0

# --------------------------------------
# Phase C: smooth return to (x0,y0)
x_end = traj[N_in+N_mid-1, 0] if N_mid > 0 else traj[N_in-1, 0]
y_end = traj[N_in+N_mid-1, 1] if N_mid > 0 else traj[N_in-1, 1]
for k in range(N_out+1):
    ti = k * sample_time
    s = sblend(ti, T_out)
    idx = N_in + N_mid + k
    if idx >= N_tot: break
    traj[idx, 0] = (1 - s) * x_end + s * x0
    traj[idx, 1] = (1 - s) * y_end + s * y0
    traj[idx, 2] = z0

# orientation (keep zero) & angular rates/thrusts
# phi theta psi p q r u1..u4 already 0

# --------------------------------------
# Velocities via numerical differentiation (central difference)
dt = sample_time
traj[:, 6] = np.gradient(traj[:, 0], dt)  # u (x-dot)
traj[:, 7] = np.gradient(traj[:, 1], dt)  # v (y-dot)
traj[:, 8] = np.gradient(traj[:, 2], dt)  # w (z-dot) -> zero

# Extend tail for NMPC horizon padding (20 points)
pad = 20
last = traj[-1, :].copy()
traj = np.vstack([traj, np.tile(last, (pad, 1))])

# --------------------------------------
# Save
np.savetxt('tank_dob_real.txt', traj, fmt='%.6f')
print("Trajectory saved to tank_fig8_vertical.txt")
print(f"Total points (incl. pad): {traj.shape[0]}")
print(f"Figure-8 loops: {loops}, loop period: {T_loop}s")
print(f"X range: [{traj[:,0].min():.3f}, {traj[:,0].max():.3f}] (target ±{Ax})")
print(f"Y range: [{traj[:,1].min():.3f}, {traj[:,1].max():.3f}] (target ±{Ay})")
print(f"Start: ({traj[0,0]:.3f}, {traj[0,1]:.3f}, {traj[0,2]:.3f})")
print(f"End:   ({traj[-(pad+1),0]:.3f}, {traj[-(pad+1),1]:.3f}, {traj[-(pad+1),2]:.3f})")

# --------------------------------------
# Visualization (保留你的风格)
plt.figure(figsize=(15,10))

# 3D view
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
ax1 = plt.subplot(2,2,1, projection='3d')
ax1.plot(traj[:,0], traj[:,1], traj[:,2], linewidth=2, label='Vertical Figure-8 Trajectory')
ax1.scatter(traj[0,0], traj[0,1], traj[0,2], s=80, label='Start')
ax1.scatter(traj[-(pad+1),0], traj[-(pad+1),1], traj[-(pad+1),2], s=80, label='End')
xx, yy = np.meshgrid([tank_x_min, tank_x_max], [tank_y_min, tank_y_max])
zz = np.full_like(xx, tank_z_min)
ax1.plot_surface(xx, yy, zz, alpha=0.25)
ax1.set_xlim(tank_x_min, tank_x_max); ax1.set_ylim(tank_y_min, tank_y_max); ax1.set_zlim(tank_z_min, tank_z_max)
ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.set_zlabel('Z (m)')
ax1.set_title('3D Vertical Figure-8 (上半/下半)')
ax1.legend()

# XY top view
ax2 = plt.subplot(2,2,2)
ax2.plot(traj[:,0], traj[:,1], linewidth=2, label='Vertical Figure-8')
ax2.scatter(traj[0,0], traj[0,1], s=60, label='Start/End')
ax2.add_patch(plt.Rectangle((tank_x_min, tank_y_min),
                            tank_x_max-tank_x_min, tank_y_max-tank_y_min,
                            fill=False, linewidth=2, label='Tank Boundary'))
ax2.set_aspect('equal', adjustable='box')
ax2.grid(True)
ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
ax2.set_title('Top View (垂直八字：上半环 / 下半环)')
ax2.legend()
ax2.set_xlim(tank_x_min-0.5, tank_x_max+0.5)
ax2.set_ylim(tank_y_min-0.5, tank_y_max+0.5)

# X vs t
ax3 = plt.subplot(2,2,3)
time_main = np.arange(N_tot) * sample_time
ax3.plot(time_main, traj[:N_tot,0], linewidth=2, label='X')
ax3.axhline(tank_x_min, linestyle='--'); ax3.axhline(tank_x_max, linestyle='--')
ax3.set_xlabel('Time (s)'); ax3.set_ylabel('X (m)')
ax3.set_title('X vs Time'); ax3.grid(True); ax3.legend()

# Y vs t
ax4 = plt.subplot(2,2,4)
ax4.plot(time_main, traj[:N_tot,1], linewidth=2, label='Y')
ax4.axhline(tank_y_min, linestyle='--'); ax4.axhline(tank_y_max, linestyle='--')
ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Y (m)')
ax4.set_title('Y vs Time'); ax4.grid(True); ax4.legend()

plt.tight_layout()
plt.savefig('tank_dob_real.png', dpi=300, bbox_inches='tight')
plt.show()
print("Figure saved to tank_fig8_vertical.png")
