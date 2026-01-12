import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    default_params_file = os.path.join(config_dir, 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # ---------- launch args ----------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file)
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')

    # ---------- Nav2 容器 ----------
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ---------- Navigation（仅加载组件，不负责生命周期） ----------
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'use_composition': 'True',
            'container_name': 'nav2_container',
            'autostart': 'False',   # ❗关键：我们自己管 lifecycle
        }.items(),
    )

    # ---------- Nav2 Lifecycle Manager（核心修复） ----------
    lifecycle_nav2 = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'behavior_server',
                'smoother_server',
                'velocity_smoother',
                'collision_monitor',
                'global_costmap',
                'local_costmap',
                'waypoint_follower',
                'route_server',
                'docking_server',
            ],
        }],
    )

    # ---------- Map Saver（你原来是对的） ----------
    map_saver = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_saver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_saver_server'],
            }],
        ),
    ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        nav2_container,
        navigation,
        lifecycle_nav2,   # ⭐⭐⭐ 关键
        map_saver,
    ])
