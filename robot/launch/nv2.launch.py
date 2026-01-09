import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace

def generate_launch_description():
    # --- 1. 路径与变量定义 ---
    pkg_name = 'robot'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    default_params_file_path = os.path.join(config_dir, 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # --- 2. 声明参数 ---
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=default_params_file_path)
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true')

    # --- 3. 关键补丁：启动组件容器 ---
    # 如果 use_composition 为 True，必须有一个容器
    container_node = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # --- 4. 包含导航实现 ---
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_composition': 'True',
            'container_name': 'nav2_container', # 显式指定容器名
            'autostart': autostart,
        }.items(),
    )

    # --- 5. 地图保存相关 ---
    map_saver_nodes = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_saver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_saver_server']
            }]
        )
    ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file_cmd,
        declare_autostart_cmd,
        container_node,       # 必须先有容器
        nav2_navigation_launch,
        map_saver_nodes
    ])