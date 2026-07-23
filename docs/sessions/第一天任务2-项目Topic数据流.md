# 第一天任务 2：项目 Topic 数据流

## 1. 总体数据流

这个项目的机械臂遥操作主链路是：

```text
人的手臂动作
    │
    ▼
WiseGlove IMU
    │ /quat_data：16个浮点数，约50 Hz
    ▼
arm_tele_real
    │ 四元数 → 旋转矩阵 → 逆运动学
    │ /rokae_control_joints：7个目标关节角
    ▼
mpc_interpolation
    │ 平滑轨迹并约束速度、加速度等
    │ /sent_joints：7个高频关节命令
    ▼
rokae_rt_node
    │ Rokae 实时控制 SDK
    ▼
Rokae xMateEr Pro 真机
```

前三段是“生成目标”，最后的 `rokae_rt_node` 才会真正控制物理机械臂。

## 2. 第一段：手套发布姿态

源文件：

```text
teleop_ws/src/wiseglove18/src/glove_data_pub.cpp
```

手套节点创建了 `/quat_data` 发布者，并使用 20 ms 定时器读取一次姿态，即设计频率约为 50 Hz。

程序默认连接：

```text
/dev/teleop/glove
```

刚启动时 `sent_quat_flag` 为 `false`，因此不会立即发布姿态。终端中按 `a` 后会：

1. 调用 `ResetQuat()`，将当前姿态作为新的参考。
2. 翻转 `sent_quat_flag`。
3. 开始或停止发布 `/quat_data`。

每条消息放入 16 个浮点数，可以理解为 4 组四元数。后续 IK 主要使用其中代表腕部、肘部和上臂的三组姿态。

## 3. 第二段：姿态转换成关节角

公共逻辑位于：

```text
teleop_ws/src/arm_tele_mujo/arm_tele_mujo/arm_tele.py
```

真机派生类位于：

```text
teleop_ws/src/arm_tele_mujo/arm_tele_mujo/arm_tele_real.py
```

处理过程为：

```text
msg.data
  → NumPy 数组
  → Quatnumpy_to_Rotation()
  → 三个旋转矩阵
  → arm_ik.ori_inv()
  → 7维关节角 q
```

逆运动学输入的对应关系是：

```python
up_ori    = rots[2].matrix()
elbow_ori = rots[1].matrix()
wrist_ori = rots[0].matrix()
```

`q_last` 是上一次关节角，用来帮助求解器选择连续、接近当前姿态的解。求解成功后，节点将 7 个关节角发布到：

```text
/rokae_control_joints
```

如果求解失败，程序不会发布失败结果，而是重新发布 `last_valid_q`。

## 4. 第三段：MPC 插值

源文件：

```text
teleop_ws/src/mpc_interpolation/src/interpolation.cpp
```

该节点订阅：

```text
/rokae_control_joints
```

并发布：

```text
/sent_joints
```

为什么不能直接把约 50 Hz 的 IK 输出送给实时机械臂？因为相邻目标可能不连续，通信周期也不足以直接满足实时控制。插值节点的作用是生成更密集、更平滑的关节轨迹。

权重配置位于：

```text
teleop_ws/src/mpc_interpolation/config/mpc_weight.yaml
```

其中包括位置、速度、加速度和控制量的代价权重。入门阶段先理解用途，不要直接调参。

## 5. 第四段：真机执行

源文件：

```text
teleop_ws/src/rokae_rt/src/main.cpp
```

`rokae_rt_node` 订阅 `/sent_joints`，要求每条消息恰好包含 7 个数。合法消息会进入一个有限长度队列，实时回调再逐条取出并交给 Rokae SDK。

需要特别区分两个动作：

1. 启动节点：连接机器人、进入自动及实时模式、上电，并执行全零位 `MoveJ`。
2. 按下 `c`：启动关节位置实时跟随循环。

因此“还没有按 `c`”不等于“机器人不会运动”。节点构造阶段已经存在回零动作。

## 6. Launch 文件没有覆盖整条链路

运行：

```bash
ros2 launch start_up start.launch.py
```

只会启动：

- `arm_tele_real`
- `mpc_interpolation_node`
- 力传感器节点
- 腕部相机节点

它不会启动：

- `wiseglove18/glove_data_pub`
- `rokae_rt/rokae_rt_node`

这两个节点需要单独启动。理解这一点可以避免看到 `/quat_data` 或 `/sent_joints` 没有数据时误判程序故障。

## 7. 故障定位方法

从上游向下游逐段检查：

```bash
ros2 topic hz /quat_data
ros2 topic hz /rokae_control_joints
ros2 topic hz /sent_joints
```

| 现象 | 优先检查 |
| --- | --- |
| 没有 `/quat_data` | 手套串口、手套节点、是否按过 `a` |
| 有姿态但没有关节目标 | `arm_tele_real` 是否运行、IK 是否报错 |
| 有关节目标但没有 `/sent_joints` | MPC 节点、配置文件、进程是否退出 |
| `/sent_joints` 正常但真机不跟随 | 驱动连接、是否按 `c`、SDK/机器人状态 |
| 真机突然跳动 | 初始目标、回零过程、姿态标定、IK 解连续性 |

## 8. 本节结论

你需要记住三个关键 Topic：

```text
/quat_data              人体姿态
/rokae_control_joints   IK 得到的低频关节目标
/sent_joints            送给真机驱动的高频关节命令
```

以及一个安全边界：`rokae_rt_node` 是从软件数据进入物理世界的最后一关。

