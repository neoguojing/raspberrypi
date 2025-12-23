import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取配置文件路径
    # 假设你的包名叫 'robot'，且配置文件在 config/ekf.yaml
    pkg_share = get_package_share_directory('robot')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 2. 定义 EKF 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}] # 仿真环境设为 True
    )

    return LaunchDescription([
        ekf_node
    ])