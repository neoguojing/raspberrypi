import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    pkg_name = 'robot' # 替换为你的包名
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    my_params_file = os.path.join(config_dir, 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. 对应你 YAML 中定义的每一个服务节点
    nav2_nodes = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'collision_monitor'),
        ('nav2_map_server', 'map_saver_server'),
    ]

    nodes_to_start = []
    # 用于生命周期管理的节点列表
    lc_node_names = [node[1] for node in nav2_nodes]

    # 2. 批量构建 Node 对象
    for pkg, exec_name in nav2_nodes:
        nodes_to_start.append(
            Node(
                package=pkg,
                executable=exec_name,
                name=exec_name,
                output='screen',
                parameters=[my_params_file, {'use_sim_time': use_sim_time}],
                # 如果你的雷达或速度话题需要重映射，在这里统一处理
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
            )
        )

    # 3. 生命周期管理器 (至关重要：负责激活上述所有节点)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lc_node_names
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', use_sim_time),
        
        *nodes_to_start,
        lifecycle_manager
    ])