#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from nav_msgs.msg import Odometry        # 用于接收 /bluerov2/pose_gt
from geometry_msgs.msg import Twist      # 用于发布给 MAVROS 的速度指令

def odom_callback(msg):
    """
    回调函数：每当接收到新的 Odometry 数据时触发
    从中提取 linear 和 angular.z 速度，并转发到 MAVROS
    """
    twist_msg = Twist()

    # 提取线速度（单位 m/s）
    twist_msg.linear.x = msg.twist.twist.linear.x
    twist_msg.linear.y = msg.twist.twist.linear.y
    twist_msg.linear.z = msg.twist.twist.linear.z

    # 提取偏航角速度（单位 rad/s），通常控制 angular.z 即可
    twist_msg.angular.z = msg.twist.twist.angular.z

    # 发布到 MAVROS 接口
    mavros_pub.publish(twist_msg)

if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('pose_gt_to_mavros_node')

    # 创建发布器，将 Twist 消息发送给 MAVROS 接口
    mavros_pub = rospy.Publisher(
        '/mavros/setpoint_velocity/cmd_vel_unstamped',
        Twist,
        queue_size=10
    )

    # 创建订阅器，接收地面真值的速度信息
    rospy.Subscriber(
        '/bluerov2/pose_gt',
        Odometry,
        odom_callback
    )

    # 保持节点运行
    rospy.spin()
