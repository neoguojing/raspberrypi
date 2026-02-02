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
        "subscribe_rgb": False,
        "subscribe_depth": False,
        "subscribe_stereo": False,
        "subscribe_scan": True,
        "subscribe_odom_info": False,  # EKF / wheel odom
        "subscribe_imu": False,
        "subscribe_scan_cloud": False,

        "Mem/UseOdomFeatures": "false",
        "Vis/FeatureType": "0",
        "Kp/DetectorStrategy": "0",
        # 核心：ICP 与 2D 设定
        "Reg/Strategy": "1",          # 0=Visual, 1=ICP, 2=Visual+ICP
        "Reg/Force3DoF": "true",      # 2D 模式
        "Optimizer/Slam2D": "true",
        
        # ICP 参数微调
        "Icp/CorrespondenceRatio": "0.05",
        "Icp/VoxelSize": "0.0",
        "Icp/MaxCorrespondenceDistance": "0.15",
        "Icp/Strategy": "0",          # 0=Point-to-Point, 1=Point-to-Plane
        "Icp/Iterations": "50",
        "Icp/PointToPlane": "false",
        
        # 地图生成
        "Grid/Sensor": "0",           # 0=Laser Scan, 1=Depth, 2=Both
        "Grid/FromDepth": "false",
        "Grid/RayTracing": "true",    # 开启射线追踪以清理地图
        "Grid/RangeMax": "4.0",      # 激光有效最远距离
        "Grid/CellSize": "0.05",      # 地图分辨率 (5cm)
        "Grid/3D": "false",  
        
        # 内存与回环检测 (基于激光的扫描匹配回环)
        "RGBD/ProximityBySpace": "false", # 允许在靠近先前位置时通过 ICP 闭环
        "RGBD/AngularUpdate": "0.0",    # 旋转 0.05 rad 更新一次
        "RGBD/LinearUpdate": "0.0",      # 移动 0.1 m 更新一次
        "Mem/IncrementalMemory": "true", # False 则为纯定位模式
        "Mem/GenerateCloud": "false",      # 不生成点云以节省资源

        "Odom/Strategy": "1", # 强制使用外部里程计（如果你已经有EKF了）
        "Odom/ResetCountdown": "0",  # 禁止 odom reset
    }
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_laser',
        namespace=namespace,
        condition=UnlessCondition(compressed),
        output='screen',
        arguments=[
            '--delete_db_on_start',
            '--ros-args',
            '--log-level', 'rtabmap:=warn'
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
        name='rtabmap_laser_compressed',
        namespace=namespace,
        condition=IfCondition(compressed),
        output='screen',
        arguments=[
            '--delete_db_on_start',
            '--ros-args',
            '--log-level', 'rtabmap:=warn'
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
