import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # --- 1. 路径与参数定义 ---
    pkg_path = get_package_share_directory("robot")
    
    sensor_mode = LaunchConfiguration('sensor_mode')
    slam_backend = LaunchConfiguration('slam_backend')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'sim.launch.py')),
        condition=IfCondition(
            PythonExpression([
                "'", sensor_mode, "' in ['laser', 'mono']"
            ])
        )
    )

    sim_stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'sim.stereo.launch.py')),
        condition=IfCondition(
            PythonExpression([
                "'", sensor_mode, "' == 'stereo'"
            ])
        )
    )
    
    algo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'algo.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'compressed': 'false',
            'sensor_mode': sensor_mode,
            'slam_backend': slam_backend
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_backend',
            default_value='rtabmap',
            description='orbslam3 | rtabmap | slam_toolbox'
        ),
        DeclareLaunchArgument(
            'sensor_mode',
            default_value='stereo',
            description='mono | stereo | laser'
        ),
        sim_launch,
        sim_stereo_launch,
        algo_launch
    ])