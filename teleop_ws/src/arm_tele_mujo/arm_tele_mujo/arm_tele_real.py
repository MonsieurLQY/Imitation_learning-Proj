#! /usr/bin/env python3
import rclpy
from std_msgs.msg import Float32MultiArray
# from rclpy.executors import MultiThreadedExecutor
from cust_msgs.msg import Stampfloat32array

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import rclpy.duration

import numpy as np

from .drake_model import Quatnumpy_to_Rotation
from .arm_tele import ArmTele

# ArmTele 的真机数据流派生类。
#
# 输入：/quat_data             手套姿态，约 50 Hz
# 处理：四元数 -> 旋转矩阵 -> 逆运动学
# 输出：/rokae_control_joints  7 个低频目标关节角
#
# 这里仍然只是在 ROS 2 中“生成目标”，不会直接调用 Rokae SDK。真正进入物理
# 机械臂的是 rokae_rt_node；两者之间还要经过 mpc_interpolation。

class ArmTele_real(ArmTele):
    def __init__(self):

        # 父类构造函数已经创建 /quat_data 订阅、机器人模型和 IK 求解器。
        super().__init__()
        self.get_logger().info('ArmTele_real Node Started')
        # 该 QoS 与下游 MPC 的订阅设置对应：只保留最新一条、可靠传输、
        # 不为后来加入的订阅者保留旧数据，并期望约 1 ms 的 deadline。
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0, nanoseconds=1000000)
        )

        # 在第一条手套姿态到达前，每 50 ms 发布一次全零关节目标。
        # 第一条消息到达后会取消此定时器，改为发布 IK 结果。
        self.received_first_msg = False
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.zero_joints = np.zeros(7, dtype=np.float32)

        # 教学主链路的第二个关键 Topic：MPC 节点订阅它。
        # 每条 Float32MultiArray 正常应包含 7 个目标关节角。
        self.control_pub_ = self.create_publisher(
            Float32MultiArray, '/rokae_control_joints', qos_profile)

        # 记录实际参与 IK 的前三组四元数及 ROS 时间戳，属于观测/记录支线。
        self.origin_data_pub_ = self.create_publisher(
            Stampfloat32array, 'origin_quat_data_used', 10)


        # if self._q is not None:
        #     self.real_q = self._q.copy()

    def timer_callback(self):
        # 尚未收到 /quat_data 时提供零目标。注意：这个 Topic 若已接到完整的
        # 真机链路，下游如何处理零目标仍需结合 MPC 和驱动状态判断。
        if not self.received_first_msg:
            goal_msg = Float32MultiArray()
            goal_msg.data = self.zero_joints.tolist()
            self.control_pub_.publish(goal_msg)


    def quat_callback(self, msg):

        # 第一条姿态意味着手套数据已经接通，停止周期性零目标。
        if not self.received_first_msg:
                self.received_first_msg = True
                self.timer.cancel()
                self.timer.destroy()
                self.get_logger().info('Received first valid message, stopped zero-joint timer.')

        # 以下处理覆盖父类同名回调，并增加“发布关节目标”和“失败回退”逻辑。
        self.quats = np.array(msg.data)
        rots = Quatnumpy_to_Rotation(self.quats) 
        # print(f"last value:{self.last_valid_q}")

        # 上臂、肘部、腕部的旋转矩阵进入 IK。q_last 优先使用上次求解值，
        # 以帮助求解器获得时间上连续的关节解。
        self._q = self._ik.ori_inv(
            up_ori=rots[2].matrix(),
            elbow_ori=rots[1].matrix(),
            wrist_ori=rots[0].matrix(),
            q_last=self._q if self._q is not None else self.last_valid_q,
        )
        if self._q is not None:
            # IK 成功：保存 7 维结果并发布到 /rokae_control_joints。
            self.last_valid_q = self._q
            goal_msg = Float32MultiArray()
            goal_msg.data = self._q.tolist()
            self.control_pub_.publish(goal_msg)
            # 同时记录本次真正用于求解的前三组（12 个数）四元数。
            ori_data_msg = Stampfloat32array()
            ori_data_msg.header.stamp = self.get_clock().now().to_msg()
            ori_data_msg.data = self.quats[0:12].tolist()
            self.origin_data_pub_.publish(ori_data_msg)
        else:
            # IK 失败：绝不发布 None 或无效维数，而是重发最近一次有效关节角。
            self.get_logger().warn('No valid joint angles found,sent last valid joint angles')
            goal_msg = Float32MultiArray()
            goal_msg.data = self.last_valid_q.tolist()
            self.control_pub_.publish(goal_msg)
            return
        
def main(args=None):

    rclpy.init(args=args)
    arm_tele = ArmTele_real()
    try:
        rclpy.spin(arm_tele)
    except KeyboardInterrupt:
        arm_tele.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        # 清理节点和ROS 2环境
        arm_tele.destroy_node()
        rclpy.shutdown()
