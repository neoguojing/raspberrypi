from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import json
import os
def generate_launch_description():
    """
    ROS 2 启动文件，用于同时启动 CarDriverNode 和 CameraPublisherNode。
    """
    
    # 替换 'my_robot_pkg' 为您的 ROS 2 包名
    package_name = 'robot' 
    pkg_share = get_package_share_directory(package_name)


    maunual_ctl_node = Node(
        package=package_name,
        executable='manual_nav_commander', # 确保在 setup.py 中定义了此入口点
        name='manual_nav_commander',
        output='screen',
        parameters=[

        ]
    )

    explore_node = Node(
        package=package_name,
        executable='explore_node', # 确保在 setup.py 中定义了此入口点
        name='explore_node',
        output='screen',
        parameters=[

        ]
    )
    
    # 3. 返回启动描述
    return LaunchDescription([
        explore_node,
    ])