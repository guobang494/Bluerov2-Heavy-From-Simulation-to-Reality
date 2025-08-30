import numpy as np
import pandas as pd

# 1. 读取数据（Blue Robotics T200 Excel 14V）
df = pd.read_excel("T200-Public-Performance.xlsx", sheet_name="14 V")

# 2. 提取 PWM (us) 和推力 (N)，必要时把 kgf 转成 N
pwm = df[" PWM (µs)"].values
thrust = df[" Force (Kg f)"].values * 9.80665  # kgf → N

# 3. 设定中点和死区
neutral, deadband = 1500, 30

# 4. 分段数据
x_rev = pwm[pwm < neutral - deadband] - neutral
y_rev = thrust[pwm < neutral - deadband]
x_fwd = pwm[pwm > neutral + deadband] - neutral
y_fwd = thrust[pwm > neutral + deadband]

# 5. 三阶多项式拟合
crev = np.polyfit(x_rev, y_rev, 3)  # 反向
cfwd = np.polyfit(x_fwd, y_fwd, 3)  # 正向

print("Reverse:", crev)  # a3, a2, a1, a0
print("Forward:", cfwd)  # a3, a2, a1, a0
