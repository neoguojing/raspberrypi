import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot'
    config_dir = os.path.join(
        get_package_share_directory(pkg_name),
        'config'
    )

    # ===============================
    # Launch args
    # ===============================
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map_file')
    scan_topic = LaunchConfiguration('scan_topic')

    # ===============================
    # slam_toolbox parameters
    # ===============================
    slam_toolbox_params = {
        "use_sim_time": use_sim_time,

        # 模式
        "mode": mode,  # mapping / localization

        # Frame
        "map_frame": LaunchConfiguration('map_frame'),
        "odom_frame": LaunchConfiguration('odom_frame'),
        "base_frame": LaunchConfiguration('base_frame'),

        # scan
        "scan_topic": scan_topic,

        # TF
        "publish_tf": True,

        # 性能 & 稳定性
        "throttle_scans": 1,
        "transform_publish_period": 0.05,
        "map_update_interval": 3.0,
        "resolution": 0.05,

        # 匹配参数
        "max_laser_range": 5.0,
        "minimum_travel_distance": 0.2,
        "minimum_travel_heading": 0.1,
        "scan_buffer_size": 10,
        "scan_buffer_maximum_scan_distance": 10.0,

        # 闭环检测
        "loop_search_maximum_distance": 3.0,
        "loop_match_minimum_chain_size": 10,
        "loop_match_maximum_variance_coarse": 3.0,
        "loop_match_minimum_response_coarse": 0.35,

        # solver
        "solver_plugin": "solver_plugins::CeresSolver",
        "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY",
        "ceres_preconditioner": "SCHUR_JACOBI",
        "ceres_trust_strategy": "LEVENBERG_MARQUARDT",

        # localization only
        "map_file_name": map_file,
    }

    # ===============================
    # slam_toolbox node
    # ===============================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[slam_toolbox_params],
        arguments=[
            '--ros-args',
            '--log-level', 'slam_toolbox:=info'
        ]
    )

    return LaunchDescription([
        # ===============================
        # Declare args
        # ===============================
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            description='mapping | localization'
        ),

        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Map yaml for localization mode'
        ),

        DeclareLaunchArgument(
            'scan_topic',
            default_value='/seg/scan'
        ),

        # TF frames
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),

        slam_toolbox_node
    ])
