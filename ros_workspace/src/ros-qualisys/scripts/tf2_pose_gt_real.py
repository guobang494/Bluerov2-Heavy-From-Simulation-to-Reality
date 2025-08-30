#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_msgs.msg import TFMessage
import numpy as np

class TF2PoseGroundTruth:
    def __init__(self):
        rospy.init_node('tf2_pose_gt_real', anonymous=True)
        
        # Parameters
        self.target_frame = rospy.get_param('~child_frame_id', 'BlueROV2H')  # 要跟踪的刚体名称
        self.reference_frame = rospy.get_param('~frame_id', 'mocap')     # 参考坐标系
        self.publish_rate = rospy.get_param('~publish_rate', 20.0 )            # 发布频率 Hz
        self.pose_topic = rospy.get_param('~pose_topic', '/bluerov2_heavy')           # 发布话题名
        
        # Subscribe to /tf topic from ros_qualisys
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        
        # Publisher for ground truth pose (Odometry format)
        self.pose_pub = rospy.Publisher(self.pose_topic, Odometry, queue_size=10)
        
        # Variables for velocity calculation
        self.last_pose = None
        self.last_time = None
        self.current_transform = None
        
        # Rate control
        self.rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo(f"TF2 Pose Ground Truth Node started")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        rospy.loginfo(f"Reference frame: {self.reference_frame}")
        rospy.loginfo(f"Subscribing to: /tf")
        rospy.loginfo(f"Publishing to: {self.pose_topic}")
        rospy.loginfo(f"Publish rate: {self.publish_rate} Hz")
    
    def tf_callback(self, tf_msg):
        """Callback for the /tf topic subscriber."""
        rospy.loginfo_throttle(5.0, f"Received tf message with {len(tf_msg.transforms)} transforms")
        
        # Find the transform from the reference frame to the target frame
        for transform in tf_msg.transforms:
            rospy.loginfo_throttle(5.0, f"Transform: frame_id={transform.header.frame_id}, child_frame_id={transform.child_frame_id}")
            
            if transform.header.frame_id == self.reference_frame and transform.child_frame_id == self.target_frame:
                self.current_transform = transform
                rospy.loginfo_throttle(5.0, f"Found matching transform for {self.target_frame}!")
                break
        else:
            rospy.logwarn_throttle(5.0, f"No matching transform found. Looking for: frame_id={self.reference_frame}, child_frame_id={self.target_frame}")
    
    def calculate_velocity(self, current_pose, current_time):
        """计算线速度和角速度"""
        linear_vel = [0.0, 0.0, 0.0]
        angular_vel = [0.0, 0.0, 0.0]
        
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                # 计算线速度
                linear_vel[0] = (current_pose.position.x - self.last_pose.position.x) / dt
                linear_vel[1] = (current_pose.position.y - self.last_pose.position.y) / dt
                linear_vel[2] = (current_pose.position.z - self.last_pose.position.z) / dt
                
                # 简化的角速度计算 (这里可以根据需要改进)
                # 实际应用中可能需要更复杂的四元数差分计算
                angular_vel[0] = (current_pose.orientation.x - self.last_pose.orientation.x) / dt
                angular_vel[1] = (current_pose.orientation.y - self.last_pose.orientation.y) / dt
                angular_vel[2] = (current_pose.orientation.z - self.last_pose.orientation.z) / dt
        
        return linear_vel, angular_vel
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                # Check if we have received tf data
                if self.current_transform is None:
                    rospy.logwarn_throttle(1.0, f"Waiting for tf data for frame: {self.target_frame}")
                    self.rate.sleep()
                    continue
                
                # 创建 Odometry 消息
                odom_msg = Odometry()
                
                # 填充消息头
                odom_msg.header.stamp = self.current_transform.header.stamp
                odom_msg.header.frame_id = self.reference_frame
                odom_msg.child_frame_id = self.target_frame
                
                # 填充位置信息
                odom_msg.pose.pose.position.x = self.current_transform.transform.translation.x
                odom_msg.pose.pose.position.y = self.current_transform.transform.translation.y
                odom_msg.pose.pose.position.z = self.current_transform.transform.translation.z
                
                # 填充姿态信息
                odom_msg.pose.pose.orientation.x = self.current_transform.transform.rotation.x
                odom_msg.pose.pose.orientation.y = self.current_transform.transform.rotation.y
                odom_msg.pose.pose.orientation.z = self.current_transform.transform.rotation.z
                odom_msg.pose.pose.orientation.w = self.current_transform.transform.rotation.w
                
                # 计算速度
                current_time = self.current_transform.header.stamp
                linear_vel, angular_vel = self.calculate_velocity(
                    odom_msg.pose.pose, current_time
                )
                
                # 填充速度信息
                odom_msg.twist.twist.linear.x = linear_vel[0]
                odom_msg.twist.twist.linear.y = linear_vel[1]
                odom_msg.twist.twist.linear.z = linear_vel[2]
                odom_msg.twist.twist.angular.x = angular_vel[0]
                odom_msg.twist.twist.angular.y = angular_vel[1]
                odom_msg.twist.twist.angular.z = angular_vel[2]
                
                # 设置协方差矩阵 (可以根据实际情况调整)
                # 位置协方差 (6x6 矩阵，展开为36个元素)
                odom_msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,  # x
                                          0, 0.01, 0, 0, 0, 0,  # y
                                          0, 0, 0.01, 0, 0, 0,  # z
                                          0, 0, 0, 0.01, 0, 0,  # roll
                                          0, 0, 0, 0, 0.01, 0,  # pitch
                                          0, 0, 0, 0, 0, 0.01]  # yaw
                
                # 速度协方差 (6x6 矩阵，展开为36个元素)
                odom_msg.twist.covariance = [0.1, 0, 0, 0, 0, 0,   # vx
                                           0, 0.1, 0, 0, 0, 0,   # vy
                                           0, 0, 0.1, 0, 0, 0,   # vz
                                           0, 0, 0, 0.1, 0, 0,   # wx
                                           0, 0, 0, 0, 0.1, 0,   # wy
                                           0, 0, 0, 0, 0, 0.1]   # wz
                
                # 发布消息到 pose_pub
                self.pose_pub.publish(odom_msg)
                
                # 更新历史数据用于速度计算
                self.last_pose = odom_msg.pose.pose
                self.last_time = current_time
                
                # 日志输出 (降低频率)
                if rospy.get_time() % 1.0 < 0.01:  # 每秒输出一次
                    rospy.loginfo_throttle(1.0, 
                        f"Published pose to {self.pose_topic}: [{odom_msg.pose.pose.position.x:.3f}, "
                        f"{odom_msg.pose.pose.position.y:.3f}, "
                        f"{odom_msg.pose.pose.position.z:.3f}]")
                
            except Exception as e:
                rospy.logerr(f"Unexpected error: {str(e)}")
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TF2PoseGroundTruth()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TF2 Pose Ground Truth node terminated.")
    except Exception as e:
        rospy.logerr(f"Failed to start TF2 Pose Ground Truth node: {str(e)}")
