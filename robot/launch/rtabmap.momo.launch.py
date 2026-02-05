import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition
from launch_ros.actions import Node
from typing import Text, List, Tuple


def generate_launch_description():
    pkg_name = 'robot'  # 替换为你的包名
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    # -------------------
    # LaunchConfigurations
    # -------------------
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    compressed = LaunchConfiguration('compressed')

    rgb_topic = LaunchConfiguration('rgb_topic')
    
    # ===============================
    # 1. 图像解压节点（仅 compressed=true 时启动）
    # ===============================
    republish_rgb = Node(
        package='image_transport',
        executable='republish',
        name='republish_rgb',
        namespace=namespace,
        condition=IfCondition(compressed),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', [rgb_topic, '/compressed']),
            ('out', [rgb_topic, '_relay']),
        ],
        output='screen'
    )

        # ===============================
        # 2. RTAB-Map 核心节点
        # ===============================
    slam_parameters = {
        # 通用
        "use_sim_time": use_sim_time,
        "frame_id": LaunchConfiguration('frame_id'),
        "odom_frame_id": LaunchConfiguration('odom_frame_id'),
        "map_frame_id": LaunchConfiguration('map_frame_id'),
        "publish_tf": LaunchConfiguration('publish_tf_map'),
        "approx_sync": True,
        "sync_queue_size": 30,
        "topic_queue_size": 30,

        # 传感器订阅
        "subscribe_rgb": True,
        "subscribe_depth": LaunchConfiguration('depth'),
        "subscribe_stereo": False,
        "subscribe_scan": LaunchConfiguration('subscribe_scan'),
        "subscribe_odom_info": False,  # EKF / wheel odom
        "subscribe_imu": LaunchConfiguration('subscribe_imu'),

        # 地图参数
        "Grid/Sensor": "0",  # 0=激光
        "Grid/FromDepth": "false",
        "Grid/3D": "false",
        "Grid/RangeMax": "4.0",
        "Grid/RayTracing": "true",
        "Grid/CellSize": "0.05",
        "Grid/OctoMap": "false",

        # 视觉特征参数
        "Kp/DetectorStrategy": "2",
        "Kp/MaxFeatures": "1000",
        "Vis/EstimationType": "2",
        "Vis/FeatureType": "2",
        "Vis/EpipolarGeometryVar": "0.5",
        "Vis/Iterations": "300",
        "Vis/MinInliers": "8",
        "Vis/InlierDistance": "0.1",

        # 闭环策略
        "Reg/Strategy": "0",  # 1=ICP (激光), 0=Visual
        "Reg/Force3DoF": "true",
        "RGBD/OptimizeMaxError": "5.0",
        "RGBD/NeighborLinkRefining": "true",

        # 单目尺度恢复
        "Mem/StereoFromMotion": "true",
        "Mem/UseOdomFeatures": "true",

        "Odom/Strategy": "1", # 强制使用外部里程计（如果你已经有EKF了）
        "Odom/ResetCountdown": "0",  # 禁止 odom reset

        # 地图稳定性
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        "Mem/STMSize": "30",

        # 回环后不整体平移地图
        "RGBD/OptimizeFromGraphEnd": "true",
        "RGBD/LinearUpdate": "0.1",
        "RGBD/AngularUpdate": "0.1",

        # 限制 map 更新频率
        "Rtabmap/DetectionRate": "1",  # 1Hz 就够
    }
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap_momo',
        name='rtabmap',
        namespace=namespace,
        condition=UnlessCondition(compressed),
        output='screen',
        arguments=[
            '--delete_db_on_start',
            '--ros-args',
            '--log-level', 'warn'
        ],
        parameters=[slam_parameters],
        remappings=[
            ('rgb/image', rgb_topic),
            ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
            ("odom", LaunchConfiguration('odom_topic')),
            ("map", LaunchConfiguration('map_topic')),
            ("scan", LaunchConfiguration('scan_topic'))
        ],
    )

    rtabmap_slam_compressed = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_momo_compressed',
        namespace=namespace,
        condition=IfCondition(compressed),
        output='screen',
        arguments=[
            '--delete_db_on_start',
            '--ros-args',
            '--log-level', 'warn'
        ],
        parameters=[slam_parameters],
        remappings=[
            ('rgb/image', [rgb_topic, '_relay']),
            ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
            ("odom", LaunchConfiguration('odom_topic')),
            ("map", LaunchConfiguration('map_topic')),
            ("scan", LaunchConfiguration('scan_topic'))
        ],
    )

    return LaunchDescription([
        # -------------------
        # 基础参数
        # -------------------
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('subscribe_scan', default_value='true'),
        DeclareLaunchArgument('subscribe_imu', default_value='false'),
        DeclareLaunchArgument('depth', default_value='false'),
        DeclareLaunchArgument('compressed', default_value='false'),

        # -------------------
        # TF / Frame IDs
        # -------------------
        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('publish_tf_map', default_value='true'),

        # -------------------
        # Topic 设置
        # -------------------
        DeclareLaunchArgument('rgb_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/ekf/odom'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('scan_topic', default_value='/seg/scan'),

        republish_rgb,
        rtabmap_slam,
        rtabmap_slam_compressed,
    ])
