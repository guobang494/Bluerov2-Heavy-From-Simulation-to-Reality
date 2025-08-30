#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosbag
import time
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import Twist
import tf.transformations as tf_trans

class Sim2RealController:
    """
    从rosbag中读取Pose_gt数据，提取xyz和yaw速度信息，
    通过mavros RC override话题控制BlueROV2
    """
    
    def __init__(self, bag_file_path):
        rospy.init_node('sim2real_controller')
        
        # 参数设置
        self.bag_file_path = bag_file_path
        self.pose_topic = '/bluerov2/pose_gt'
        
        # 速度限制 (m/s 和 rad/s)
        self.max_linear_vel = 0.5  # 最大线性速度
        self.max_angular_vel = 0.3  # 最大角速度
        
        # RC通道映射 (BlueROV2标准映射)
        # channels: [roll, pitch, throttle, yaw, forward, lateral, camera_tilt, lights]
        self.rc_channels = [1500] * 8  # 初始化为中性值
        
        # 发布器和订阅器
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # 状态变量
        self.current_state = State()
        self.rate = rospy.Rate(20)  # 20Hz
        self.is_armed = False
        
        # 存储速度数据
        self.velocity_data = []
        self.timestamps = []
        
        rospy.loginfo("Sim2Real控制器初始化完成")
        rospy.loginfo(f"Rosbag文件: {self.bag_file_path}")
    
    def state_callback(self, msg):
        """MAVROS状态回调"""
        self.current_state = msg
        self.is_armed = msg.armed
    
    def wait_for_connection(self):
        """等待与MAVROS连接"""
        rospy.loginfo("等待与MAVROS连接...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()
        rospy.loginfo("✅ 已连接到MAVROS!")
    
    def arm_robot(self):
        """解锁机器人"""
        rospy.loginfo("尝试解锁机器人...")
        try:
            response = self.arming_client(True)
            if response.success:
                rospy.loginfo("✅ 机器人已解锁!")
                return True
            else:
                rospy.logwarn("❌ 解锁失败!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"解锁服务调用失败: {e}")
            return False
    
    def disarm_robot(self):
        """上锁机器人"""
        rospy.loginfo("上锁机器人...")
        try:
            response = self.arming_client(False)
            if response.success:
                rospy.loginfo("✅ 机器人已上锁!")
                return True
            else:
                rospy.logwarn("❌ 上锁失败!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"上锁服务调用失败: {e}")
            return False
    
    def set_manual_mode(self):
        """设置为手动模式"""
        rospy.loginfo("设置为手动模式...")
        try:
            response = self.set_mode_client(custom_mode="MANUAL")
            if response.mode_sent:
                rospy.loginfo("✅ 已设置为手动模式!")
                return True
            else:
                rospy.logwarn("❌ 模式设置失败!")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"模式设置服务调用失败: {e}")
            return False
    
    def extract_yaw_from_quaternion(self, quaternion):
        """从四元数提取yaw角"""
        # 四元数: (x, y, z, w)
        euler = tf_trans.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return euler[2]  # yaw角
    
    def load_velocity_data_from_bag(self):
        """从rosbag文件中加载速度数据"""
        rospy.loginfo("开始从rosbag加载速度数据...")
        
        try:
            bag = rosbag.Bag(self.bag_file_path)
            
            for topic, msg, t in bag.read_messages(topics=[self.pose_topic]):
                # 提取时间戳
                timestamp = msg.header.stamp.to_sec()
                
                # 提取线性速度 (xyz)
                linear_vel = np.array([
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z
                ])
                
                # 提取角速度 (yaw)
                angular_vel_z = msg.twist.twist.angular.z
                
                # 存储数据
                self.velocity_data.append({
                    'timestamp': timestamp,
                    'linear_x': linear_vel[0],
                    'linear_y': linear_vel[1], 
                    'linear_z': linear_vel[2],
                    'angular_z': angular_vel_z
                })
                
                self.timestamps.append(timestamp)
            
            bag.close()
            
            rospy.loginfo(f"✅ 成功加载 {len(self.velocity_data)} 条速度数据")
            
            if self.velocity_data:
                rospy.loginfo(f"时间范围: {self.timestamps[0]:.2f}s - {self.timestamps[-1]:.2f}s")
                rospy.loginfo(f"数据持续时间: {self.timestamps[-1] - self.timestamps[0]:.2f}s")
        
        except Exception as e:
            rospy.logerr(f"加载rosbag数据失败: {e}")
            return False
        
        return len(self.velocity_data) > 0
    
    def velocity_to_rc_pwm(self, vel_x, vel_y, vel_z, vel_yaw):
        """
        将速度转换为RC PWM值
        PWM范围: 1100-1900, 中性值: 1500
        """
        # 限制速度范围
        vel_x = np.clip(vel_x, -self.max_linear_vel, self.max_linear_vel)
        vel_y = np.clip(vel_y, -self.max_linear_vel, self.max_linear_vel)
        vel_z = np.clip(vel_z, -self.max_linear_vel, self.max_linear_vel)
        vel_yaw = np.clip(vel_yaw, -self.max_angular_vel, self.max_angular_vel)
        
        # 转换为PWM值 (1100-1900范围)
        # 速度范围 [-max_vel, max_vel] -> PWM范围 [1100, 1900]
        pwm_range = 400  # (1900-1100)/2
        neutral_pwm = 1500
        
        # BlueROV2通道映射: [roll, pitch, throttle, yaw, forward, lateral, camera_tilt, lights]
        channels = [1500] * 8  # 初始化
        
        channels[0] = neutral_pwm  # roll (保持中性)
        channels[1] = neutral_pwm  # pitch (保持中性)
        channels[2] = int(neutral_pwm - (vel_z / self.max_linear_vel) * pwm_range)  # throttle (上下)
        channels[3] = int(neutral_pwm + (vel_yaw / self.max_angular_vel) * pwm_range)  # yaw
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
        
        for i, vel_data in enumerate(self.velocity_data):
            if rospy.is_shutdown():
                break
            
            # 计算应该执行的时间
            target_time = start_time + (vel_data['timestamp'] - data_start_time)
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
            
            # 打印进度
            if i % 20 == 0:  # 每20个数据点打印一次
                progress = (i / len(self.velocity_data)) * 100
                rospy.loginfo(f"回放进度: {progress:.1f}% - 速度: x={vel_data['linear_x']:.3f}, y={vel_data['linear_y']:.3f}, z={vel_data['linear_z']:.3f}, yaw={vel_data['angular_z']:.3f}")
        
        # 回放完成，停止机器人
        self.stop_robot()
        rospy.loginfo("✅ 速度数据回放完成!")
    
    def run(self):
        """运行主程序"""
        try:
            # 1. 加载rosbag数据
            if not self.load_velocity_data_from_bag():
                return
            
            # 2. 等待MAVROS连接
            self.wait_for_connection()
            
            # 3. 设置手动模式
            if not self.set_manual_mode():
                return
            
            # 4. 解锁机器人
            if not self.arm_robot():
                return
            
            # 等待一下确保系统稳定
            rospy.sleep(2)
            
            # 5. 开始速度回放
            self.run_velocity_replay()
            
            # 6. 上锁机器人
            self.disarm_robot()
            
        except Exception as e:
            rospy.logerr(f"运行出错: {e}")
        finally:
            # 确保停止机器人
            self.stop_robot()

if __name__ == '__main__':
    try:
        # rosbag文件路径
        bag_file_path = '/home/zeb/test-8/eight-thurster/src/bluerov2/bluerov2_dobmpc/rosbag/bluerov2_dobmpc_20250731_083732.bag'
        
        # 创建控制器
        controller = Sim2RealController(bag_file_path)
        
        rospy.loginfo("按 Enter 开始sim2real控制，或 Ctrl+C 退出...")
        input()
        
        # 运行控制器
        controller.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被ROS中断")
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
