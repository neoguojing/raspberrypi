import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('robot')

    # 包含基础机器人节点 (Camera + Car)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'robot.launch.py'))
    )

    # 包含 SLAM 节点
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'slam.launch.py'))
    )

    return LaunchDescription([
        base_launch,
        slam_launch
    ])