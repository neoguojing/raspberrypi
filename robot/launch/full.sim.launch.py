import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- 1. 路径与参数定义 ---
    pkg_path = get_package_share_directory("robot")
    
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'sim.launch.py')),
    )
    
    algo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'algo.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'compressed': 'false'
        }.items()
    )

    return LaunchDescription([
        sim_launch,
        algo_launch
    ])