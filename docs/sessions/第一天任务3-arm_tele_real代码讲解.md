# 第一天任务 3：`arm_tele_real.py` 代码讲解

## 1. 文件职责

文件位置：

```text
teleop_ws/src/arm_tele_mujo/arm_tele_mujo/arm_tele_real.py
```

它不是底层机器人驱动。它负责：

1. 继承公共的机械臂遥操作节点。
2. 接收 `/quat_data` 姿态消息。
3. 调用逆运动学求出 7 个关节角。
4. 将结果发布到 `/rokae_control_joints`。

真正与 Rokae 机器人通信的是 `rokae_rt/src/main.cpp`。

## 2. 继承关系

```python
class ArmTele_real(ArmTele):
```

父类 `ArmTele` 的构造函数已经完成：

- 创建名为 `arm_tele` 的 ROS 2 节点。
- 订阅 `/quat_data`。
- 读取 `model_name` 参数，默认是 `roake_7dof`。
- 加载 MuJoCo/SDF 机器人模型。
- 创建 `arm_ik` 逆运动学求解器。
- 为七自由度机器人初始化 7 维零关节向量。

所以子类调用：

```python
super().__init__()
```

之后，就已经具备订阅姿态和计算 IK 的基础条件。

## 3. QoS 配置

代码为关节控制 Topic 配置了：

```python
QoSProfile(
    depth=1,
    reliability=RELIABLE,
    durability=VOLATILE,
    deadline=1ms
)
```

含义是：

- `depth=1`：只保留最新的一条消息，避免控制命令积压。
- `RELIABLE`：尽力保证消息可靠交付。
- `VOLATILE`：新订阅者不会收到很久以前保存的历史消息。
- `deadline=1ms`：声明期望较短的通信周期。

入门时无需修改，但上下游 QoS 不兼容时，可能出现 Topic 存在却收不到消息的情况。

## 4. 启动后的全零定时器

构造函数创建：

```python
self.timer = self.create_timer(0.05, self.timer_callback)
self.zero_joints = np.zeros(7, dtype=np.float32)
```

只要还没收到第一条手套消息，`timer_callback()` 就会每 50 ms 发布：

```text
[0, 0, 0, 0, 0, 0, 0]
```

到 `/rokae_control_joints`。

这段逻辑的意图可能是给下游提供初始目标，但在真机上具有明显风险：全零关节角是一个真实运动目标，不等于“保持当前姿态”。因此不能把启动 `arm_tele_real` 理解成完全无动作的操作。

## 5. `quat_callback()` 逐步讲解

### 5.1 收到第一条消息后停止零位定时器

```python
if not self.received_first_msg:
    self.received_first_msg = True
    self.timer.cancel()
    self.timer.destroy()
```

之后目标将由手套姿态驱动，不再周期发布全零数组。

### 5.2 解析姿态数据

```python
self.quats = np.array(msg.data)
rots = Quatnumpy_to_Rotation(self.quats)
```

`msg.data` 来自 `/quat_data`。手套节点发布 16 个浮点数，转换函数将其解释成多组四元数，再转成旋转对象。

这里缺少显式的消息长度检查。若数组长度或排列不符合预期，可能抛出异常或得到错误姿态。因此仿真和真机前都应先用：

```bash
ros2 topic echo --once /quat_data
```

确认长度和数据变化正常。

### 5.3 调用逆运动学

```python
self._q = self._ik.ori_inv(
    up_ori=rots[2].matrix(),
    elbow_ori=rots[1].matrix(),
    wrist_ori=rots[0].matrix(),
    q_last=self._q if self._q is not None else self.last_valid_q,
)
```

输入是上臂、肘部和腕部的目标朝向。输出 `self._q` 是 7 维关节角。

`q_last` 很重要：七自由度机械臂对同一个末端或肢体姿态通常存在多组解。上一时刻关节角为求解器提供初值，有助于获得连续的解，减少突然跳到另一种构型的风险。

### 5.4 求解成功

```python
if self._q is not None:
    self.last_valid_q = self._q
```

随后将 NumPy 数组转换为列表，通过 `Float32MultiArray` 发布到：

```text
/rokae_control_joints
```

同时把使用过的原始姿态发布到 `origin_quat_data_used`，并附加 ROS 时间戳。这个 Topic 主要用于数据记录和后续对齐。

### 5.5 求解失败

当 `ori_inv()` 返回 `None` 时：

```python
goal_msg.data = self.last_valid_q.tolist()
self.control_pub_.publish(goal_msg)
```

也就是说，程序保持并重发最后一次有效目标，而不是发布无效数据。这是基本的降级策略，但仍有两个注意点：

- 没有连续失败次数上限。
- 没有超时后主动停止真机的逻辑。

因此操作者仍需观察日志和机械臂状态。

## 6. `main()` 生命周期

```python
rclpy.init()
arm_tele = ArmTele_real()
rclpy.spin(arm_tele)
```

`spin()` 让节点持续等待 Topic 和定时器事件。按 `Ctrl+C` 后进入清理逻辑，销毁节点并关闭 ROS 2。

注意：退出这个 IK 节点不等于一定会停止底层机器人驱动。`rokae_rt_node` 有自己的进程和控制逻辑，真机停止应按照已经验证过的完整流程执行。

## 7. 阅读后应能回答的问题

1. 输入 Topic 是什么？——`/quat_data`。
2. 输入数据如何变成关节角？——四元数转旋转矩阵，再调用 `ori_inv()`。
3. 输出 Topic 是什么？——`/rokae_control_joints`。
4. 输出数组多长？——Rokae 模型下为 7。
5. IK 失败怎么办？——发布最后一次有效关节角。
6. 第一条姿态消息到达前发布什么？——每 50 ms 发布一次全零关节目标。
7. 这个文件会直接连接机器人吗？——不会，但其输出可能经下游节点最终控制真机。

## 8. 仿真观察建议

同时打开三个终端观察：

```bash
ros2 topic echo /quat_data
ros2 topic echo /rokae_control_joints
ros2 topic hz /rokae_control_joints
```

缓慢移动手套，重点观察关节角是否连续。如果某一帧出现很大的跃迁，即使 IK 返回了有效数组，也不应直接进入真机测试。

