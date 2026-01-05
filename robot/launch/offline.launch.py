import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 路径与参数定义 ---
    package_name = "robot"
    pkg_path = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    camera_publisher_node = Node(
        package=package_name,
        executable='camera_publisher_node', # 确保在 setup.py 中定义了此入口点
        name='camera_publisher_node',
        output='screen',
        parameters=[
            {'camera_frequency': 15.0},
            {'is_camera': False},
            {'source': '../asset/fpv-demo-50-2.mp4'},
        ]
    )
    
    algo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'algo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        camera_publisher_node,
        algo_launch
    ])