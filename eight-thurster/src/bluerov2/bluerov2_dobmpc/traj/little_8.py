import numpy as np
import matplotlib.pyplot as plt

# 参数设定
sample_time = 0.05         # 秒
duration = 60              # 总时长（秒）
ramp_up_time = 2           # 起步静止时间（秒）
amp_x = 8                  # x方向振幅
amp_y = 0.8                # y方向振幅
frq = 0.05                 # 频率

x0 = 0
y0 = 0
z0 = -20

# 时间轴
t = np.arange(0, duration + sample_time, sample_time)
traj = np.zeros((len(t), 16))  # x y z φ θ ψ u v w p q r u1 u2 u3 u4

# 主体路径
t_shift = t - ramp_up_time
t_shift[t_shift < 0] = 0  # 0~ramp时间内设为0（静止）

traj[:, 0] = amp_x * np.cos(t_shift * frq) + x0
traj[:, 1] = amp_y * np.sin(t_shift * frq) * np.cos(t_shift * frq) + y0
traj[:, 2] = z0

# 速度
traj[:, 6] = -amp_x * frq * np.sin(t_shift * frq)          # u
traj[:, 7] = amp_y * frq * np.cos(2 * t_shift * frq)       # v

# 保存
np.savetxt("little_8.txt", traj, fmt="%.6f")
print("✅ Trajectory saved as little_8.txt")


