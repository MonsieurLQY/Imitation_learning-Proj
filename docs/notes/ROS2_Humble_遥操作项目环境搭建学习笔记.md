# ROS 2 Humble 与遥操作项目环境搭建学习笔记

## 1. 本次学习目标

本次工作的目标是：在 Ubuntu 22.04 上完成 ROS 2 Humble 环境配置，下载并编译 `Imitation_learning-Proj` 项目，解决编译过程中的依赖和 CMake 配置问题，并为后续学习 ROS 2 节点、Topic、仿真链路和真机遥操作做好准备。

项目工作空间路径为：

```bash
~/Projects/Imitation_learning-Proj/teleop_ws
```

当前已经完成：

- Ubuntu 22.04 环境确认
- ROS 2 Humble 安装
- ROS 2 基础通信测试
- `rosdep`、`colcon` 等工具安装
- 项目依赖检查
- 项目整体编译成功
- WiseGlove、qpOASES、Qt6 等依赖问题排查
- `mpc_interpolation` 单独编译成功
- 整个 ROS 2 工作空间编译成功

---

## 2. ROS 2 Humble 基础环境

### 2.1 系统版本

ROS 2 Humble 推荐运行在：

```text
Ubuntu 22.04 Jammy
```

检查系统版本：

```bash
lsb_release -a
```

或：

```bash
cat /etc/os-release
```

---

### 2.2 Locale 配置

执行：

```bash
locale
```

本机输出中：

```text
LANG=en_US.UTF-8
LANGUAGE=zh_CN:en_US:en
LC_NUMERIC=zh_CN.UTF-8
LC_TIME=zh_CN.UTF-8
LC_ALL=
```

这是正常的。

关键点：

- ROS 2 要求 UTF-8 环境。
- `en_US.UTF-8` 与 `zh_CN.UTF-8` 都满足要求。
- `LC_ALL` 为空并不是错误，表示没有用一个总变量覆盖所有分类设置。
- 无需为了 ROS 2 强制设置永久 `LC_ALL`。

---

### 2.3 ROS 2 Humble 环境加载

每个新终端中都需要加载 ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

检查：

```bash
echo $ROS_DISTRO
```

预期：

```text
humble
```

为了自动加载，可以将其加入 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 2.4 工作空间环境加载

项目编译成功后，每个终端还需要加载工作空间：

```bash
cd ~/Projects/Imitation_learning-Proj/teleop_ws
source install/setup.bash
```

完整顺序：

```bash
source /opt/ros/humble/setup.bash
cd ~/Projects/Imitation_learning-Proj/teleop_ws
source install/setup.bash
```

检查工作空间是否生效：

```bash
echo $AMENT_PREFIX_PATH
```

输出中应包含：

```text
/home/lqy/Projects/Imitation_learning-Proj/teleop_ws/install
```

---

## 3. ROS 2 基础测试

安装 ROS 2 后，可通过官方 talker/listener 测试通信。

终端 1：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

若终端 2 持续收到：

```text
I heard: [Hello World: ...]
```

说明 ROS 2 基础通信正常。

停止节点：

```text
Ctrl+C
```

---

## 4. 项目下载与工作空间检查

项目建议放在 Ubuntu 自己的 Linux 文件系统中，而不是 Windows 的 NTFS 分区中编译。

克隆项目：

```bash
cd ~/Projects
git clone https://github.com/MengMengMengH/Imitation_learning-Proj.git
```

进入工作空间：

```bash
cd ~/Projects/Imitation_learning-Proj/teleop_ws
```

查看 ROS 2 包：

```bash
colcon list
```

本项目包含：

```text
arm_hand_description
arm_tele_mujo
cam_srv
cust_msgs
data_collection
force_srv
inspire_hand
mpc_interpolation
qt_mainwin_control
rokae_rt
sixd_interfaces
start_up
wiseglove18
```

查看所有 `package.xml` 中的包名：

```bash
find src -name package.xml -exec grep -H "<name>" {} \;
```

---

## 5. rosdep 依赖检查

标准命令：

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --rosdistro humble
```

参数说明：

- `--from-paths src`：扫描 `src` 目录中的 ROS 包
- `--ignore-src`：忽略当前工作空间中已有的源码包
- `-r`：遇到部分错误时继续处理其他依赖
- `-y`：自动确认安装
- `--rosdistro humble`：指定 ROS 2 Humble

---

## 6. package.xml 中 Eigen3 依赖名错误

最初出现：

```text
Cannot locate rosdep definition for [Eigen3]
```

原因：

- CMake 中通常使用：

```cmake
find_package(Eigen3 REQUIRED)
```

- 但 `package.xml` 中应使用 rosdep key：

```xml
<depend>eigen</depend>
```

而不能写：

```xml
<depend>Eigen3</depend>
```

系统依赖安装：

```bash
sudo apt install -y libeigen3-dev
```

正确区分：

### package.xml

```xml
<depend>eigen</depend>
```

### CMakeLists.txt

```cmake
find_package(Eigen3 REQUIRED)
```

```cmake
target_link_libraries(target_name Eigen3::Eigen)
```

---

## 7. 无效依赖 roake_control_interfaces

`mpc_interpolation/package.xml` 中原本存在：

```xml
<depend>roake_control_interfaces</depend>
```

但检查后发现：

```bash
grep -Rni "roake_control_interfaces" src
```

只在 `package.xml` 中出现。

工作空间中既没有：

```text
roake_control_interfaces
```

也没有：

```text
rokae_control_interfaces
```

而 `mpc_interpolation` 源码实际只使用：

```cpp
std_msgs::msg::Float32MultiArray
sensor_msgs::msg::JointState
```

`CMakeLists.txt` 也没有：

```cmake
find_package(roake_control_interfaces REQUIRED)
```

因此可判断该依赖是遗留或无效依赖，删除：

```xml
<depend>roake_control_interfaces</depend>
```

删除后重新运行 `rosdep`。

---

## 8. WiseGlove 库问题

### 8.1 原始错误

编译 `wiseglove18` 时出现：

```text
wiseglove library not found
```

原始 CMake：

```cmake
find_library(WISEGLOVE_LIB wiseglove)

if (NOT WISEGLOVE_LIB)
  message(FATAL_ERROR "wiseglove library not found")
endif()
```

### 8.2 实际项目已经自带库

检查：

```bash
find src/wiseglove18 -maxdepth 4 -type f | sort
```

发现：

```text
src/wiseglove18/include/wiseglove18/libwiseglove.so
src/wiseglove18/include/wiseglove18/Wiseglove.h
```

因此问题不是库不存在，而是 CMake 只在系统默认路径寻找，没有搜索项目内部目录。

### 8.3 修改思路

在 `CMakeLists.txt` 中定义：

```cmake
set(WISEGLOVE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/include/wiseglove18"
)
```

查找项目内库：

```cmake
find_library(
  WISEGLOVE_LIB
  NAMES wiseglove
  PATHS "${WISEGLOVE_DIR}"
  NO_DEFAULT_PATH
)
```

添加头文件路径：

```cmake
target_include_directories(
  glove_data_pub
  PRIVATE
  "${WISEGLOVE_DIR}"
)
```

链接库：

```cmake
target_link_libraries(
  glove_data_pub
  "${WISEGLOVE_LIB}"
)
```

安装动态库：

```cmake
install(
  FILES "${WISEGLOVE_DIR}/libwiseglove.so"
  DESTINATION lib/${PROJECT_NAME}
)
```

为运行时配置 RPATH：

```cmake
set_target_properties(
  glove_data_pub
  PROPERTIES
  BUILD_RPATH "${WISEGLOVE_DIR}"
  INSTALL_RPATH "$ORIGIN"
)
```

清理旧缓存：

```bash
rm -rf build/wiseglove18
rm -rf install/wiseglove18
```

单独编译：

```bash
colcon build \
  --symlink-install \
  --packages-select wiseglove18 \
  --event-handlers console_direct+
```

---

## 9. qpOASES 依赖问题

### 9.1 qpOASES 是什么

qpOASES 是一个二次规划求解器，常用于模型预测控制 MPC 中的 QP 求解。

项目中：

```cmake
find_package(qpOASES REQUIRED)
```

说明它被当作外部依赖，而不是项目内部源码。

---

### 9.2 如何检查是否安装

检查 apt/dpkg：

```bash
dpkg -l | grep -i qpoases
```

检查动态库：

```bash
ldconfig -p | grep -i qpoases
```

搜索系统文件：

```bash
find /usr /usr/local \
  \( -iname 'libqpOASES.so*' -o -iname 'libqpOASES.a' \) \
  2>/dev/null
```

检查头文件：

```bash
find /usr /usr/local \
  -iname 'qpOASES.hpp' \
  2>/dev/null
```

检查 CMake 配置：

```bash
find /usr /usr/local \
  \( -iname 'qpOASESConfig.cmake' -o -iname 'qpoases-config.cmake' \) \
  2>/dev/null
```

检查项目内部：

```bash
cd ~/Projects/Imitation_learning-Proj
find . -iname '*qpoases*'
```

当以上均为空时，可确认系统和项目均未提供 qpOASES。

---

### 9.3 安装到用户目录

不一定安装到系统目录 `/usr/local`，可以安装到：

```text
~/.local/qpoases
```

下载：

```bash
cd ~/Projects
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
```

编译：

```bash
mkdir -p build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local/qpoases"
```

```bash
cmake --build . -j"$(nproc)"
cmake --install .
```

配置环境变量：

```bash
export CMAKE_PREFIX_PATH="$HOME/.local/qpoases:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$HOME/.local/qpoases/lib:$LD_LIBRARY_PATH"
```

永久写入：

```bash
echo 'export CMAKE_PREFIX_PATH="$HOME/.local/qpoases:$CMAKE_PREFIX_PATH"' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="$HOME/.local/qpoases/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc
source ~/.bashrc
```

---

### 9.4 CMake 找到 qpOASES，但头文件找不到

CMake 输出：

```text
-- Found qpOASES: /home/lqy/.local/qpoases/include
```

但编译时报：

```text
fatal error: qpOASES.hpp: No such file or directory
```

原因：

CMake 找到了包，但没有将 qpOASES 头文件目录传给具体编译目标。

检查实际位置：

```bash
find ~/.local/qpoases/include -name qpOASES.hpp
```

本机属于：

```text
/home/lqy/.local/qpoases/include/qpOASES.hpp
```

因此源码：

```cpp
#include <qpOASES.hpp>
```

无需修改。

只需在两个目标中加入：

```cmake
"$ENV{HOME}/.local/qpoases/include"
```

例如：

```cmake
target_include_directories(mpc_interpolation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "$ENV{HOME}/.local/qpoases/include"
)
```

```cmake
target_include_directories(cubic_spline_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "$ENV{HOME}/.local/qpoases/include"
)
```

---

### 9.5 头文件找到，但链接器找不到库

错误：

```text
/usr/bin/ld: cannot find -lqpOASES
```

或中文：

```text
/usr/bin/ld: 找不到 -lqpOASES
```

含义：

- 头文件已经找到
- 源码已经成功编译
- 但链接器不知道 `libqpOASES.so` 在哪里

检查库文件：

```bash
find ~/.local/qpoases \
  \( -name 'libqpOASES.so*' -o -name 'libqpOASES.a' \) \
  -print
```

推荐在 CMake 中显式寻找：

```cmake
find_library(
  QPOASES_LIBRARY
  NAMES qpOASES
  PATHS
    "$ENV{HOME}/.local/qpoases/lib"
    "$ENV{HOME}/.local/qpoases/lib64"
  REQUIRED
)
```

然后：

```cmake
target_link_libraries(mpc_interpolation_node
  ${QPOASES_LIBRARY}
  Eigen3::Eigen
  Boost::system
  Boost::thread
  Boost::atomic
)
```

```cmake
target_link_libraries(cubic_spline_node
  ${QPOASES_LIBRARY}
  Eigen3::Eigen
)
```

最终 `mpc_interpolation` 编译成功。

---

## 10. Qt6 依赖问题

本次遇到了两种不同情况。

---

### 10.1 mpc_interpolation 中的 Qt6 是无效遗留依赖

原 CMake 中：

```cmake
find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)
```

但实际链接项全部被注释：

```cmake
# Qt6::Widgets
# Qt6::Charts
```

搜索源码：

```bash
grep -RniE \
"#include.*Qt|QWidget|QChart|QApplication|Qt6::" \
src/mpc_interpolation
```

结果只有 CMake 注释，没有源码使用 Qt。

因此可安全删除或注释：

```cmake
# find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)
```

这说明：

> 不能看到 `find_package` 报错就直接安装依赖，应先确认源码是否真的使用该依赖。

---

### 10.2 qt_mainwin_control 中 Qt6 是真实依赖

该包明确使用：

```cmake
find_package(Qt6 REQUIRED COMPONENTS Widgets Core Gui Charts)
```

链接：

```cmake
Qt6::Widgets
Qt6::Core
Qt6::Gui
Qt6::Charts
```

源码包含：

```cpp
#include <QMainWindow>
#include <QTimer>
#include <QApplication>
```

因此 Qt6 是真实依赖，不能删除。

安装：

```bash
sudo apt update
sudo apt install -y \
  qt6-base-dev \
  qt6-base-dev-tools \
  libqt6charts6-dev
```

说明：

- `qt6-base-dev`：提供 Core、Gui、Widgets
- `qt6-base-dev-tools`：提供 Qt6 构建工具
- `libqt6charts6-dev`：提供 Charts

安装后检查：

```bash
find /usr/lib/x86_64-linux-gnu/cmake \
  -name Qt6Config.cmake \
  -o -name Qt6ChartsConfig.cmake
```

清理并单包构建：

```bash
rm -rf build/qt_mainwin_control
rm -rf install/qt_mainwin_control
```

```bash
colcon build \
  --symlink-install \
  --packages-select qt_mainwin_control \
  --event-handlers console_direct+
```

---

## 11. 编译警告与编译错误的区别

qpOASES 编译时出现：

```text
warning: unused parameter
```

例如：

```text
unused parameter ‘nu’
unused parameter ‘toCopy’
```

这是警告，不会导致构建失败。

判断是否成功，应看最后：

```text
Finished <<< mpc_interpolation
Summary: 1 package finished
```

即使出现：

```text
1 package had stderr output
```

也不一定失败。因为编译器通常会把 warning 输出到 stderr。

真正失败通常会出现：

```text
Failed <<<
Aborted <<<
fatal error
CMake Error
undefined reference
cannot find -l...
```

---

## 12. colcon 构建方法

### 12.1 编译整个工作空间

```bash
cd ~/Projects/Imitation_learning-Proj/teleop_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

---

### 12.2 单独编译一个包

```bash
colcon build \
  --symlink-install \
  --packages-select mpc_interpolation \
  --event-handlers console_direct+
```

---

### 12.3 顺序编译，便于定位错误

```bash
colcon build \
  --symlink-install \
  --executor sequential \
  --event-handlers console_direct+
```

顺序构建的优点：

- 一个包一个包地编译
- 容易定位第一个真实错误
- 避免多个包并行输出混杂

---

### 12.4 清理单包缓存

修改 `CMakeLists.txt` 后建议：

```bash
rm -rf build/<package_name>
rm -rf install/<package_name>
```

例如：

```bash
rm -rf build/mpc_interpolation
rm -rf install/mpc_interpolation
```

然后重新编译。

---

## 13. 编译完成后的检查

加载环境：

```bash
source /opt/ros/humble/setup.bash
cd ~/Projects/Imitation_learning-Proj/teleop_ws
source install/setup.bash
```

查看包：

```bash
ros2 pkg list | grep mpc_interpolation
```

查看包中可执行程序：

```bash
ros2 pkg executables mpc_interpolation
```

检查整个项目：

```bash
ros2 pkg executables | grep -E \
"wiseglove18|mpc_interpolation|rokae_rt|arm_tele_mujo|qt_mainwin_control"
```

---

## 14. ROS 2 节点与 Topic 基础

### 14.1 查看节点

```bash
ros2 node list
```

查看节点信息：

```bash
ros2 node info /arm_tele
```

重点：

- `Subscribers`
- `Publishers`

---

### 14.2 查看 Topic

```bash
ros2 topic list
```

带类型：

```bash
ros2 topic list -t
```

项目关键 Topic：

```text
/quat_data
/rokae_control_joints
/sent_joints
```

可能的控制链：

```text
WiseGlove
    ↓
/quat_data
    ↓
arm_tele
    ↓
/rokae_control_joints
    ↓
mpc_interpolation
    ↓
/sent_joints
    ↓
rokae_rt
```

---

### 14.3 查看 Topic 连接关系

```bash
ros2 topic info /quat_data --verbose
```

```bash
ros2 topic info /rokae_control_joints --verbose
```

```bash
ros2 topic info /sent_joints --verbose
```

---

### 14.4 查看消息

```bash
ros2 topic echo --once /quat_data
```

```bash
ros2 topic echo --once /rokae_control_joints
```

```bash
ros2 topic echo --once /sent_joints
```

预期：

- `/quat_data`：16 个浮点数，4 组四元数
- `/rokae_control_joints`：7 个关节角
- `/sent_joints`：7 个插值后的关节角

---

### 14.5 查看频率

```bash
ros2 topic hz /quat_data
```

```bash
ros2 topic hz /rokae_control_joints
```

```bash
ros2 topic hz /sent_joints
```

预期关系：

```text
/quat_data                约 50 Hz
/rokae_control_joints     大致跟随输入频率
/sent_joints              通常更高
```

---

## 15. 安全注意事项

当前阶段不要直接运行：

```text
rokae_rt
rokae_rt_node
rt_robot_control_node
```

也不要直接运行整个 `start_up` 启动系统。

原因：

- 真机节点可能连接 Rokae SDK
- 可能上电
- 可能发送回零命令
- 可能直接发送关节控制指令

在运行任何节点之前，应先检查：

```bash
grep -RniE \
"create_publisher|create_subscription|rokae|power|home|move" \
src/rokae_rt
```

并确认：

- 节点连接什么设备
- 启动后是否立即控制机械臂
- 是否有安全开关
- 是否要求急停和工作空间清空
- 是否需要实验室导师在场

---

## 16. 下一阶段建议

### 阶段 1：只查询，不运行真机

```bash
ros2 pkg executables arm_tele_mujo
ros2 pkg executables mpc_interpolation
ros2 pkg executables wiseglove18
ros2 pkg executables rokae_rt
```

查看 launch 文件：

```bash
find src -type f \
  \( -name "*.launch.py" -o -name "*.launch.xml" -o -name "*.launch.yaml" \) \
  -print
```

查看 Topic：

```bash
grep -RniE \
'create_publisher|create_subscription' \
src/arm_tele_mujo \
src/mpc_interpolation \
src/wiseglove18 \
src/rokae_rt
```

---

### 阶段 2：优先使用仿真节点

优先研究：

```text
arm_tele_mujo
```

确认不会连接真机后，再启动仿真链路。

---

### 阶段 3：手工发布测试数据

例如向关节目标 Topic 发布 7 个零关节角：

```bash
ros2 topic pub --once \
/rokae_control_joints \
std_msgs/msg/Float32MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

再观察：

```bash
ros2 topic echo --once /sent_joints
```

但必须先确认 `mpc_interpolation_node` 不会直接调用真机 SDK。

---

## 17. 本次最重要的经验

### 经验 1：先判断依赖是否真实使用

遇到：

```cmake
find_package(SomeLibrary REQUIRED)
```

不要立即安装。

先搜索：

```bash
grep -Rni "SomeLibrary" src/<package>
```

判断：

- 源码是否 include
- 是否链接库
- 是否只是遗留配置
- 是否工作空间已有同名包

---

### 经验 2：区分 rosdep key、CMake 包名和系统包名

例如 Eigen：

```text
package.xml: eigen
CMake: Eigen3
apt: libeigen3-dev
```

三者名称可能不同。

---

### 经验 3：CMake 找到包，不等于目标可以使用

例如 qpOASES：

```text
find_package 成功
```

仍可能缺：

- 头文件 include path
- 库文件 link path
- 运行时动态库路径

完整流程：

```text
配置阶段
→ 编译阶段
→ 链接阶段
→ 运行阶段
```

每一阶段都可能单独失败。

---

### 经验 4：警告不等于失败

判断最终结果应看：

```text
Finished <<<
Summary: N packages finished
```

而不是只看终端中是否出现 `warning` 或 `stderr output`。

---

### 经验 5：修改 CMake 后清理缓存

```bash
rm -rf build/<package>
rm -rf install/<package>
```

可以避免旧配置影响新构建。

---

## 18. 常用命令速查

### 加载环境

```bash
source /opt/ros/humble/setup.bash
cd ~/Projects/Imitation_learning-Proj/teleop_ws
source install/setup.bash
```

### 编译整个工作空间

```bash
colcon build --symlink-install
```

### 顺序编译

```bash
colcon build \
  --symlink-install \
  --executor sequential \
  --event-handlers console_direct+
```

### 单包编译

```bash
colcon build \
  --symlink-install \
  --packages-select <package_name> \
  --event-handlers console_direct+
```

### 安装 rosdep 依赖

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y \
  --rosdistro humble
```

### 查看包

```bash
colcon list
ros2 pkg list
```

### 查看可执行程序

```bash
ros2 pkg executables <package_name>
```

### 查看节点

```bash
ros2 node list
ros2 node info /node_name
```

### 查看 Topic

```bash
ros2 topic list -t
ros2 topic info /topic_name --verbose
ros2 topic echo --once /topic_name
ros2 topic hz /topic_name
```

---

## 19. 当前状态总结

当前环境状态：

```text
Ubuntu 22.04
ROS 2 Humble
项目工作空间编译成功
Eigen 配置完成
WiseGlove 项目内动态库配置完成
qpOASES 用户目录安装完成
Qt6 开发依赖安装完成
mpc_interpolation 编译成功
qt_mainwin_control 编译成功
整个工作空间编译成功
```

下一步应当从：

```text
查询节点
→ 查询 Topic
→ 阅读 launch 文件
→ 仿真节点测试
→ 手工发布虚拟消息
→ 最后才考虑真机
```

逐步推进，而不是直接启动真机控制系统。
