#! /usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Int32MultiArray


import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
from pynput import keyboard

from .mjmodel import mj_armHandSys
from .drake_model import arm_ik,Quatnumpy_to_Rotation

# 机械臂遥操作的公共姿态处理节点。
#
# 教学主链路：
#   /quat_data（手套的 4 组四元数）
#       -> quat_callback()
#       -> NumPy 数组
#       -> 四元数转旋转矩阵
#       -> 逆运动学
#       -> self._q（机械臂目标关节角）
#
# 本基类只完成“姿态 -> 关节角”。真机使用的 ArmTele_real 会继承本类，
# 并把 self._q 发布到 /rokae_control_joints。

class ArmTele(Node):
    def __init__(self,):
        super().__init__('arm_tele')
        self.get_logger().info('Arm Teleop Node Started')

        # 订阅 glove_data_pub 发布的手套姿态。相对名称 quat_data 在根命名空间
        # 下解析为 /quat_data；队列深度为 10。
        self.quat_sub = self.create_subscription(
            Float32MultiArray,
            'quat_data', self.quat_callback, 10)

        self.package_name = 'arm_hand_description'

        # model_name 同时决定所加载的机器人模型和目标关节向量的维数。
        self.declare_parameter('model_name', 'roake_7dof')
        self.model_name = self.get_parameter('model_name').value

        # MuJoCo 模型用于描述机器人系统；Drake arm_ik 对象负责逆运动学。
        self.robot = mj_armHandSys(self.model_name,self.package_name)
        self.get_logger().info(f'Loading MuJoCo model: {self.robot.mjcf_model_path}')

        self._ik = arm_ik(f'{self.robot.sdf_model}',self.model_name)
        # self._q 既保存本次求解结果，也会作为下一次 IK 的 q_last。
        # 以相邻时刻的关节角作为初值，有助于选择连续、接近当前姿态的解。
        if  '_7dof' in self.model_name:
            self._q = np.zeros(7)
        elif '_6dof' in self.model_name:
            self._q = np.zeros(6)
        elif 'sup_arm' in self.model_name:
            self._q = np.zeros(44)
        # 真机派生类会在 IK 失败时重发这个最近一次有效结果。
        self.last_valid_q = self._q

    
    def quat_callback(self, msg):
        # now = int(time.time())
        # glove_data_pub 的设计发布频率约为 50 Hz。msg.data 含 16 个浮点数，
        # 可理解为连续排列的 4 组四元数。
        self.quats = np.array(msg.data) # 50HZ

        # 将四元数数组转换为可供 IK 使用的旋转对象。
        rots = Quatnumpy_to_Rotation(self.quats)

        # 手套各 IMU 与人体部位的对应关系：
        #   rots[2] -> 上臂，rots[1] -> 肘部，rots[0] -> 腕部。
        # ori_inv() 输出机器人各关节的目标角；本基类暂不发布该结果。
        self._q = self._ik.ori_inv(up_ori=rots[2].matrix(),elbow_ori=rots[1].matrix(),wrist_ori=rots[0].matrix(),q_last=self._q)



def main(args=None):

    # 初始化 rclpy、创建节点并进入 ROS 2 回调事件循环。
    rclpy.init(args=args)
    arm_tele = ArmTele()

    try:
        rclpy.spin(arm_tele)  # 运行ROS事件循环
    except KeyboardInterrupt:
        pass

    arm_tele.destroy_node()
    rclpy.shutdown()
