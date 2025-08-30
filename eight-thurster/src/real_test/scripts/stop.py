#!/usr/bin/env python
import rospy
from mavros_msgs.msg import OverrideRCIn

rospy.init_node("estop_simple")
pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

pwm_mid = 1500
channels = [0]*18
# 四个方向通道号（默认 Heavy 配置：前进5、横移6、垂直3、偏航4）
for ch in [5, 6, 3, 4]:
    channels[ch-1] = pwm_mid

rate = rospy.Rate(10)  # 10Hz 保持
while not rospy.is_shutdown():
    msg = OverrideRCIn()
    msg.channels = channels
    pub.publish(msg)
    rate.sleep()
