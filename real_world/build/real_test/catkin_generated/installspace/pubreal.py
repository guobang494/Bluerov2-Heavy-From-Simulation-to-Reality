#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist
# set limit
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# map to pwm
def map_to_pwm_x(n, pwm_min_x=1440, pwm_mid_x=1500, pwm_max_x=1560, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_x
    if n >= 0:
        pwm = pwm_mid_x + n * (pwm_max_x - pwm_mid_x)
    else:
        pwm = pwm_mid_x + n * (pwm_mid_x - pwm_min_x)
    return int(clamp(int(round(pwm)), pwm_min_x, pwm_max_x))

def map_to_pwm_y(n, pwm_min_y=1464, pwm_mid_y=1500, pwm_max_y=1536, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_y
    if n >= 0:
        pwm = pwm_mid_y + n * (pwm_max_y - pwm_mid_y)
    else:
        pwm = pwm_mid_y + n * (pwm_mid_y - pwm_min_y)
    return int(clamp(int(round(pwm)), pwm_min_y, pwm_max_y))

def map_to_pwm_z(n, pwm_min_z=1500, pwm_mid_z=1500, pwm_max_z=1500, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_z
    if n >= 0:
        pwm = pwm_mid_z + n * (pwm_max_z - pwm_mid_z)
    else:
        pwm = pwm_mid_z + n * (pwm_mid_z - pwm_min_z)
    return int(clamp(int(round(pwm)), pwm_min_z, pwm_max_z))


def map_to_pwm_yaw(n, pwm_min_yaw=1464, pwm_mid_yaw=1500, pwm_max_yaw=1536, deadband=0.023):   # need to find a better parameter and rang
    n = clamp(n, -1.0, 1.0)
    if abs(n) < deadband:
        return pwm_mid_yaw
    if n >= 0:
        pwm = pwm_mid_yaw + n * (pwm_max_yaw - pwm_mid_yaw)
    else:
        pwm = pwm_mid_yaw + n * (pwm_mid_yaw - pwm_min_yaw)
    return int(clamp(int(round(pwm)), pwm_min_yaw, pwm_max_yaw))



# define publish function
def publish_pwm_dict(pub, ch_pwm):
    msg = OverrideRCIn()
    msg.channels = [0] * 18
    for ch, pwm in ch_pwm.items():
        idx = int(ch) - 1
        if 0 <= idx < 18:
            msg.channels[idx] = int(pwm)
    pub.publish(msg)
# 全局变量存储当前速度
current_velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw_rate': 0.0}
velocity_ready = False

def velocity_callback(msg):
    """从DOBMPC速度命令话题直接获取速度信息"""
    global current_velocity, velocity_ready
    
    # 直接从Twist消息获取速度命令 (体坐标系)
    current_velocity['vx'] = msg.linear.x
    current_velocity['vy'] = msg.linear.y  
    current_velocity['vz'] = msg.linear.z
    current_velocity['yaw_rate'] = msg.angular.z
    velocity_ready = True

# define node
def main():
    rospy.init_node("velfile_to_rc_override")

    # ROS话题配置
    velocity_topic = rospy.get_param('~velocity_topic', '/bluerov2/cmd_vel')

    # parameters
    pwm_mid = 1500  # 中性PWM值，用于关闭时安全停止
    deadband = 0.023
    vmax_x = 0.424593  # m/s need to test
    vmax_y = 0.147  # m/s need to test
    vmax_z = 0.2  # m/s need to test
    yaw_max = 0.334  # rad/s need to test
    ch_forward, ch_lateral, ch_vertical, ch_yaw = 5, 6, 3, 4    # channel define need test
    play_rate_hz = 20.0

    pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    
    # 订阅DOBMPC速度命令话题
    velocity_sub = rospy.Subscriber(velocity_topic, Twist, velocity_callback, queue_size=10)
    
    
    
    # 等待ROS速度数据
    rospy.loginfo("等待DOBMPC速度命令数据...")
    while not rospy.is_shutdown() and not velocity_ready:
        rospy.sleep(0.1)
        
    if rospy.is_shutdown():
        return
        
    rospy.loginfo("开始实时速度控制...")
    rate = rospy.Rate(play_rate_hz)
    
    # 实时处理ROS速度数据
    while not rospy.is_shutdown():
        # 获取当前速度命令
        vx = current_velocity['vx']
        vy = current_velocity['vy']
        vz = current_velocity['vz']
        yaw_rate = current_velocity['yaw_rate']

        # 归一化 (保持原有逻辑)
        nx = clamp(vx / vmax_x, -1.0, 1.0)
        ny = clamp(vy / vmax_y, -1.0, 1.0)
        nz = clamp(vz / vmax_z, -1.0, 1.0)
        nr = clamp(yaw_rate / yaw_max, -1.0, 1.0)

        # 映射为 PWM (保持原有逻辑)
        pwm_forward = map_to_pwm_x(nx)
        pwm_lateral = map_to_pwm_y(ny)
        pwm_vertical = map_to_pwm_z(nz)
        pwm_yaw = map_to_pwm_yaw(nr)

        # 发布PWM (保持原有逻辑)
        publish_pwm_dict(pub, {
            ch_forward: pwm_forward,
            ch_lateral: pwm_lateral,
            ch_vertical: pwm_vertical,
            ch_yaw: pwm_yaw
        })
        
        # 定期打印调试信息
        if rospy.get_time() % 2.0 < 0.1:  # 每2秒打印一次
            rospy.loginfo_throttle(2.0, 
                f"速度: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, yaw_rate={yaw_rate:.3f}")
            rospy.loginfo_throttle(2.0,
                f"PWM: forward={pwm_forward}, lateral={pwm_lateral}, vertical={pwm_vertical}, yaw={pwm_yaw}")

        rate.sleep()

    # 关闭时发送中性PWM (保持原有逻辑)
    rospy.loginfo("发送中性PWM信号...")
    for _ in range(5):
        publish_pwm_dict(pub, {
            ch_forward: pwm_mid,
            ch_lateral: pwm_mid,
            ch_vertical: pwm_mid,
            ch_yaw: pwm_mid
        })
        rospy.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
