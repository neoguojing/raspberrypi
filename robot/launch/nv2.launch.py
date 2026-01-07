import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 定义必要的参数和文件路径 ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    # 获取 Nav2 包的共享目录
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 设置您的包名和配置文件路径
    pkg_name = 'robot' # 替换为您的实际包名
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    
    # 您的 Nav2 配置 YAML 文件路径
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(config_dir, 'nav2_params.yaml')

    # --- 2. 声明 Launch 参数 ---

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # --- 3. 启动 Nav2 的核心组件 ---
    print("Using Nav2 parameters file: " + str(default_params_file))
    # Nav2 Bringup 的主要启动文件，它会启动 Map Server, AMCL, Planner, Controller, Behavior Tree 等
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'map': '',
            # 定位与地图控制
            'use_localization': 'False',         # 关闭 Nav2 自带的 SLAM，因为你有 ORB-SLAM3
            'slam': 'False',
            # 性能优化
            'use_composition': 'True', # 建议开启，提升性能
            'use_respawn': 'True',     # 如果某个节点崩溃，尝试自动重启
            'autostart': 'true',

            # 命名空间控制（多机运行才需要修改）
            'namespace': '',
            'use_namespace': 'false',
            'log_level': 'debug'
            
        }.items(),
    )

    # nav2_bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'params_file': params_file,
    #         'autostart': 'True',
    #         'use_composition': 'True',
    #         'use_respawn': 'True',
    #         'namespace': '',
    #         'use_namespace': 'False',
    #         'log_level': 'debug'
    #     }.items(),
    # )

    # --- 4. 返回 LaunchDescription ---
    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file_cmd,
        nav2_bringup_launch
    ])