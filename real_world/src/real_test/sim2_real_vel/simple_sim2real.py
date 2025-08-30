#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosbag
import time
import numpy as np
from mavros_msgs.msg import OverrideRCIn

class SimpleSim2RealController:
    """
    简化版本的Sim2Real控制器
    从rosbag中读取速度数据并通过RC override控制BlueROV2
    """
    
    def __init__(self, bag_file_path):
        rospy.init_node('simple_sim2real_controller')
        
        # 参数设置
        self.bag_file_path = bag_file_path
        self.pose_topic = '/bluerov2/pose_gt'
        
        # 速度限制和缩放因子
        self.max_linear_vel = 0.5  # 最大线性速度 m/s
        self.max_angular_vel = 0.3  # 最大角速度 rad/s
        self.velocity_scale = 0.3  # 速度缩放因子，降低速度以保证安全
        
        # 发布器
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        
        # 速度数据存储
        self.velocity_data = []
        
        
    
    def load_velocity_data_from_bag(self):
        """从rosbag文件中加载速度数据"""
        rospy.loginfo("开始从rosbag加载速度数据...")
        
        try:
            bag = rosbag.Bag(self.bag_file_path)
            
            for topic, msg, t in bag.read_messages(topics=[self.pose_topic]):
                # 提取时间戳
                timestamp = msg.header.stamp.to_sec()
                
                # 提取速度数据
                self.velocity_data.append({
                    'timestamp': timestamp,
                    'linear_x': msg.twist.twist.linear.x,
                    'linear_y': msg.twist.twist.linear.y,
                    'linear_z': msg.twist.twist.linear.z,
                    'angular_z': msg.twist.twist.angular.z
                })
            
            bag.close()
            
            rospy.loginfo(f"✅ 成功加载 {len(self.velocity_data)} 条速度数据")
            
            if self.velocity_data:
                timestamps = [d['timestamp'] for d in self.velocity_data]
                duration = timestamps[-1] - timestamps[0]
                rospy.loginfo(f"数据持续时间: {duration:.2f}s")
        
        except Exception as e:
            rospy.logerr(f"加载rosbag数据失败: {e}")
            return False
        
        return len(self.velocity_data) > 0
    
    def velocity_to_rc_pwm(self, vel_x, vel_y, vel_z, vel_yaw):
        """
        将速度转换为RC PWM值
        BlueROV2 RC通道映射: [roll, pitch, throttle, yaw, forward, lateral, camera_tilt, lights]
        PWM范围: 1100-1900, 中性值: 1500
        """
        # 应用速度缩放和限制
        vel_x = np.clip(vel_x * self.velocity_scale, -self.max_linear_vel, self.max_linear_vel)
        vel_y = np.clip(vel_y * self.velocity_scale, -self.max_linear_vel, self.max_linear_vel)
        vel_z = np.clip(vel_z * self.velocity_scale, -self.max_linear_vel, self.max_linear_vel)
        vel_yaw = np.clip(vel_yaw * self.velocity_scale, -self.max_angular_vel, self.max_angular_vel)
        
        # PWM转换参数
        pwm_range = 400  # (1900-1100)/2
        neutral_pwm = 1500
        
        # 初始化通道为中性值
        channels = [1500] * 8
        
        # 映射速度到PWM值
        channels[0] = neutral_pwm  # roll (保持中性)
        channels[1] = neutral_pwm  # pitch (保持中性)
        channels[2] = int(neutral_pwm - (vel_z / self.max_linear_vel) * pwm_range)  # throttle (上下运动)
        channels[3] = int(neutral_pwm + (vel_yaw / self.max_angular_vel) * pwm_range)  # yaw (偏航)
        channels[4] = int(neutral_pwm + (vel_x / self.max_linear_vel) * pwm_range)   # forward/backward
        channels[5] = int(neutral_pwm + (vel_y / self.max_linear_vel) * pwm_range)   # lateral (左右)
        channels[6] = neutral_pwm  # camera_tilt (保持中性)
        channels[7] = 1100         # lights (关闭)
        
        # 确保PWM值在有效范围内
        channels = [max(1100, min(1900, pwm)) for pwm in channels]
        
        return channels
    
    def publish_rc_override(self, channels):
        """发布RC override指令"""
        msg = OverrideRCIn()
        msg.channels = channels + [0] * (18 - len(channels))  # 填充到18个通道
        self.rc_override_pub.publish(msg)
    
    def stop_robot(self):
        """停止机器人运动"""
        rospy.loginfo("停止机器人...")
        neutral_channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1100]
        self.publish_rc_override(neutral_channels)
        time.sleep(1)
    
    def run_velocity_replay(self):
        """回放速度数据"""
        if not self.velocity_data:
            rospy.logerr("没有速度数据可回放!")
            return
        
        rospy.loginfo("🚀 开始速度数据回放...")
        
        # 获取起始时间
        start_time = rospy.Time.now().to_sec()
        data_start_time = self.velocity_data[0]['timestamp']
        
        # 设置回放速度（可以调整这个值来改变回放速度）
        playback_speed = 1.0  # 1.0 = 正常速度，0.5 = 半速，2.0 = 2倍速
        
        for i, vel_data in enumerate(self.velocity_data):
            if rospy.is_shutdown():
                break
            
            # 计算应该执行的时间
            elapsed_data_time = (vel_data['timestamp'] - data_start_time) / playback_speed
            target_time = start_time + elapsed_data_time
            current_time = rospy.Time.now().to_sec()
            
            # 等待到正确的时间
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # 转换速度为RC PWM
            channels = self.velocity_to_rc_pwm(
                vel_data['linear_x'],
                vel_data['linear_y'],
                vel_data['linear_z'],
                vel_data['angular_z']
            )
            
            # 发布RC指令
            self.publish_rc_override(channels)
            
            # 打印进度（每2秒打印一次）
            if i % 40 == 0:  # 假设20Hz，每40个数据点约2秒
                progress = (i / len(self.velocity_data)) * 100
                rospy.loginfo(f"回放进度: {progress:.1f}% - 速度: x={vel_data['linear_x']:.3f}, y={vel_data['linear_y']:.3f}, z={vel_data['linear_z']:.3f}, yaw={vel_data['angular_z']:.3f}")
                rospy.loginfo(f"PWM: {channels[:6]}")  # 显示前6个通道的PWM值
        
        # 回放完成，停止机器人
        self.stop_robot()
        rospy.loginfo("✅ 速度数据回放完成!")
    
    def run(self):
        """运行主程序"""
        try:
            # 1. 加载rosbag数据
            if not self.load_velocity_data_from_bag():
                rospy.logerr("无法加载数据，退出程序")
                return
            
            rospy.loginfo("准备开始控制...")
            rospy.loginfo("注意：请确保BlueROV2已连接并处于适当的模式")
            rospy.loginfo("按 Ctrl+C 可随时停止程序")
            
            # 等待用户确认
            input("按 Enter 开始速度回放...")
            
            # 2. 开始速度回放
            self.run_velocity_replay()
            
        except KeyboardInterrupt:
            rospy.loginfo("程序被用户中断")
        except Exception as e:
            rospy.logerr(f"运行出错: {e}")
        finally:
            # 确保停止机器人
            self.stop_robot()
            rospy.loginfo("程序结束")

if __name__ == '__main__':
    try:
        # rosbag文件路径
        bag_file_path = '/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/bluerov2_dobmpc_20250731_083732.bag'
        
        # 创建控制器
        controller = SimpleSim2RealController(bag_file_path)
        
        # 运行控制器
        controller.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被ROS中断")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}") 