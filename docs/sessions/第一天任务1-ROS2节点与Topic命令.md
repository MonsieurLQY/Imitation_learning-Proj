# 第一天任务 1：ROS 2 节点与 Topic 命令

## 1. 学习目标

完成本节后，你应当能够在不修改源码、不驱动真机的情况下回答：

- 当前有哪些 ROS 2 节点正在运行？
- 某个节点订阅和发布了什么 Topic？
- 某个 Topic 的消息类型、发布者和订阅者是什么？
- Topic 中的数据是什么，发送频率是否正常？

本项目中的节点可以理解为独立运行的程序，Topic 是节点之间传递数据的通道。

## 2. 使用前准备

以下操作应在安装了 ROS 2 Humble 的 Ubuntu 环境执行。每打开一个新终端，都要加载环境：

```bash
source /opt/ros/humble/setup.bash
cd ~/Imitation_learning-Proj/teleop_ws
source install/setup.bash
```

如果还没有编译工作空间：

```bash
source /opt/ros/humble/setup.bash
cd ~/Imitation_learning-Proj/teleop_ws
colcon build --symlink-install
source install/setup.bash
```

仅运行查询命令不会让机械臂运动，但不要为了查看节点而启动 `rokae_rt_node`。它会连接、上电并命令机器人回零。

## 3. 查看节点

### 查看所有正在运行的节点

```bash
ros2 node list
```

可能看到的节点包括：

| 节点 | 作用 |
| --- | --- |
| `/glove_data_pub` | 读取 WiseGlove，发布姿态和手指数据 |
| `/arm_tele` | 将人体姿态转换成机械臂关节角 |
| `/interpolation_node` | 平滑关节目标，生成高频控制序列 |
| `/rt_robot_control_node` | 通过 Rokae SDK 控制真机 |
| `/six_dim_force` | 读取六维力传感器 |
| `/camera` | 读取腕部相机 |

实际节点名以 `ros2 node list` 输出为准。

### 查看一个节点的接口

```bash
ros2 node info /arm_tele
```

重点看：

- `Subscribers`：节点接收什么。
- `Publishers`：节点发送什么。
- `Services` 和 `Action Servers`：本项目入门阶段可以暂时忽略。

例如，`/arm_tele` 应订阅 `/quat_data`；真机版本还会发布 `/rokae_control_joints`。

## 4. 查看 Topic

### 列出所有 Topic

```bash
ros2 topic list
```

同时显示消息类型：

```bash
ros2 topic list -t
```

本项目关键 Topic 是：

| Topic | 消息类型 | 含义 |
| --- | --- | --- |
| `/quat_data` | `std_msgs/msg/Float32MultiArray` | 手套 IMU 姿态数据 |
| `/rokae_control_joints` | `std_msgs/msg/Float32MultiArray` | IK 输出的 7 个目标关节角 |
| `/sent_joints` | `std_msgs/msg/Float32MultiArray` | 插值后送往驱动的 7 个关节角 |

### 查看 Topic 的连接关系

```bash
ros2 topic info /quat_data --verbose
```

它会显示消息类型、发布者、订阅者以及 QoS。用同样方法检查：

```bash
ros2 topic info /rokae_control_joints --verbose
ros2 topic info /sent_joints --verbose
```

如果 Topic 存在但没有发布者，`echo` 会一直等待；这不代表命令坏了，而是上游节点没有运行或没有发数据。

## 5. 观察消息内容

```bash
ros2 topic echo /quat_data
```

你会看到类似：

```yaml
layout:
  dim: []
  data_offset: 0
data: [ ... ]
```

本项目手套节点一次向 `/quat_data` 放入 16 个浮点数。它们对应 4 组四元数数据。`arm_tele_real` 随后将这些数据转换成旋转矩阵。

观察 IK 输出：

```bash
ros2 topic echo /rokae_control_joints
```

正常情况下 `data` 应有 7 个数，分别对应七个关节角，代码内部使用弧度。

观察插值输出：

```bash
ros2 topic echo /sent_joints
```

它也应包含 7 个关节角，但更新频率通常高于 `/rokae_control_joints`。

输出太快时只看一条：

```bash
ros2 topic echo --once /rokae_control_joints
```

## 6. 检查频率

```bash
ros2 topic hz /quat_data
ros2 topic hz /rokae_control_joints
ros2 topic hz /sent_joints
```

按代码设计：

- 手套定时器周期是 20 ms，因此 `/quat_data` 目标约为 50 Hz。
- `arm_tele_real` 收到一条姿态消息后计算一次 IK，所以 `/rokae_control_joints` 通常跟随输入频率。
- `/sent_joints` 用于实时驱动，设计频率明显更高；应以实际测量为准。

判断频率时不要只看瞬时值，至少观察几秒。如果频率大幅波动，应检查串口、计算耗时、QoS 和系统负载。

## 7. 消息类型查询

查看一个消息的结构：

```bash
ros2 interface show std_msgs/msg/Float32MultiArray
```

该消息没有“关节名称”和“单位”字段，只保存浮点数组。因此数组长度、顺序和单位完全依赖项目代码约定。这也是阅读源码时必须确认的内容。

## 8. 推荐练习

在只启动手套和仿真的情况下完成以下记录：

```text
节点名称：
发布 /quat_data 的节点：
订阅 /quat_data 的节点：
/quat_data 消息长度：
/quat_data 平均频率：
/rokae_control_joints 消息长度：
/sent_joints 消息长度：
```

完成标准：你能够仅凭 `node list`、`node info`、`topic info`、`topic echo` 和 `topic hz`，定位一条控制链路在哪一段没有数据。

