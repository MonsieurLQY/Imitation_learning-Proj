from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# 本 launch 只覆盖主链路中间部分：
#
#   arm_tele_real：/quat_data -> /rokae_control_joints
#   interpolation_node：/rokae_control_joints -> /sent_joints
#
# 它还会启动力传感器和腕部相机，但明确不会启动：
#   1. wiseglove18/glove_data_pub（/quat_data 的来源）
#   2. rokae_rt/rokae_rt_node（/sent_joints 的真机执行端）
#
# 因此运行本 launch 后看不到 /quat_data，不一定是程序故障；应先确认手套节点
# 是否另行启动并在其终端按过 'a'。同样，仅运行本 launch 不会启动 Rokae 驱动。
def generate_launch_description():  
    ld = LaunchDescription()

    # 找到已安装的 MPC 包，并加载位置、速度、加速度和控制量四类权重。
    config_path = os.path.join(
            get_package_share_directory('mpc_interpolation'),
            'config',
            'mpc_weight.yaml'
        )

    # 主链路第三段：把低频 IK 目标平滑成高频 /sent_joints。
    interpolation_node = Node(
        package='mpc_interpolation',
        executable='mpc_interpolation_node',
        output='screen',
        name='interpolation_node',
        parameters=[config_path]
    )

    # 主链路第二段：把 /quat_data 转换成 7 维 /rokae_control_joints。
    IK_node = Node(
        package='arm_tele_mujo',
        executable='arm_tele_real',
        output='screen',
        name="arm_tele_real"
    )

    # 以下两个节点是传感器支线，不参与“手套 -> IK -> MPC -> 真机”控制主链路。
    force_node = Node(
        package='force_srv',
        executable='force_srv',
        output='screen',
        name='six_dim_force'
    )

    camera_node = Node(
        package='cam_srv',
        executable='wrist_cam_capture',
        output='screen',
        name='camera'
    )

    # LaunchDescription 中只添加这四个进程，手套节点和真机驱动需单独运行。
    ld.add_action(IK_node)
    ld.add_action(interpolation_node)
    ld.add_action(force_node)
    ld.add_action(camera_node)

    return ld
