import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('robot')

    # 包含 SLAM 节点
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'slam.launch.py'))
    )

    # 包含 nv2 节点
    nv2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'nv2.launch.py'))
    )

    # 包含 efk 节点
    efk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'efk.launch.py'))
    )

    return LaunchDescription([
        slam_launch,
        nv2_launch,
        efk_launch
    ])