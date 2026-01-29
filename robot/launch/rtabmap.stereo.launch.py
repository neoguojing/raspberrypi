import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot' 
    
    # --- LaunchConfigurations ---
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    compressed = LaunchConfiguration('compressed')
    
    # 双目特有话题参数
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    left_camera_info = LaunchConfiguration('left_camera_info_topic')
    right_camera_info = LaunchConfiguration('right_camera_info_topic')

    # ===============================
    # 1. 图像解压节点 (如果是双目压缩，需要解压左右两路)
    # ===============================
    # 注意：为了简洁，这里演示 raw 模式，如需压缩，需为 left 和 right 各开一个 republish 节点
    # ===============================
    # 1. 左目图像解压
    # ===============================
    republish_left = Node(
        package='image_transport',
        executable='republish',
        name='republish_left',
        namespace=namespace,
        condition=IfCondition(compressed),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', [left_image_topic, '/compressed']),
            ('out', [left_image_topic, '_relay']),
        ],
        output='screen'
    )

    # ===============================
    # 2. 右目图像解压
    # ===============================
    republish_right = Node(
        package='image_transport',
        executable='republish',
        name='republish_right',
        namespace=namespace,
        condition=IfCondition(compressed),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', [right_image_topic, '/compressed']),
            ('out', [right_image_topic, '_relay']),
        ],
        output='screen'
    )

    # ===============================
    # 2. RTAB-Map 核心参数 (双目版)
    # ===============================
    slam_parameters = {
        "use_sim_time": use_sim_time,
        "frame_id": LaunchConfiguration('frame_id'),
        "odom_frame_id": LaunchConfiguration('odom_frame_id'),
        "map_frame_id": LaunchConfiguration('map_frame_id'),
        "publish_tf": LaunchConfiguration('publish_tf_map'),
        "approx_sync": True,  # 双目通常需要近似同步
        "queue_size": 30,

        # --- 核心切换：开启双目订阅 ---
        "subscribe_rgb": False,        # 双目模式下关闭普通 RGB 订阅
        "subscribe_depth": False,      # 深度图设为 false
        "subscribe_stereo": True,      # 激活双目订阅
        "subscribe_scan": LaunchConfiguration('subscribe_scan'),
        "subscribe_imu": LaunchConfiguration('subscribe_imu'),

        # 地图参数
        # "Grid/Sensor": "0", 
        "Grid/FromDepth": "true", # 如果有激光雷达，设为 false；若想用双目点云建图，设为 true
        "Grid/MinDepth": "0.8",  # 过滤掉 0.3 米以内的所有数据，直接无视盲区噪点
        "Grid/MaxDepth": "4.0",  # 远距离太虚的数据也不要
        "Grid/RangeMin": "0.3",
        "Grid/RangeMax": "4.0",  # 不要看太远，减少点云密度
        "Grid/CellSize": "0.1",
        "Grid/MinGroundHeight": "-0.3",
        "Grid/MaxGroundHeight": "0.2", # 调整地面高度阈值，适应不同机器人底盘高度
        "Grid/MaxObstacleSlope": "60",
        "Grid/NormalK": "20",
        "Grid/NormalRadius": "0.15",
        "Grid/MaxGroundAngle": "35.0",
        "Grid/NormalsSegmentation": "false", # 关闭法线分割，节省计算
        "Grid/ClusterRadius": "0.1",   # 较小的聚类半径
        "Grid/MinClusterSize": "30",    # 忽略掉孤立的小簇点（降噪的同时提速）
        "Grid/FlatObstacleDetected": "true", # 针对平整地面障碍的特殊检测方案
        "Grid/GroundIsObstacle": "false",
        # 4. 清理无效障碍（超重要）
        "Grid/NoiseFilteringRadius": "0.2",
        "Grid/NoiseFilteringMinNeighbors": "3",

        


        # 视觉特征与闭环
        "Vis/EstimationType": "1",    # 1=Stereo (双目特征估计)
        "Vis/FeatureType": "2",       # ORB
        "Kp/DetectorStrategy": "2",
        "Reg/Strategy": "0",          # 0=Visual, 1=ICP, 2=Both
        
        # 双目匹配参数
        "Stereo/MinDisparity": "1",
        "Stereo/MaxDisparity": "128",
        "Stereo/OpticalFlow": "false", # false 则使用特征匹配，true 则使用光流

        "Odom/Strategy": "1", # 强制使用外部里程计（如果你已经有EKF了）
        "Odom/ResetCountdown": "0",  # 禁止 odom reset

        "Vis/MaxFeatures": "800",  # 双目可以适当增加特征数量
        
        # 地图稳定性
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",

        # 回环后不整体平移地图
        "RGBD/OptimizeMaxError": "5.0",
        "RGBD/OptimizeFromGraphEnd": "true",
        "RGBD/LinearUpdate": "0.0",
        "RGBD/AngularUpdate": "0.0",

        # 限制 map 更新频率
        "Rtabmap/DetectionRate": "1",
        "Rtabmap/TimeThr": "0",
    }

    rtabmap_stereo = Node(
        package='rtabmap_slam',
        executable='rtabmap', # 或者使用 'stereo_odometry' 如果你需要 RTAB-Map 计算里程计
        name='rtabmap_stereo',
        namespace=namespace,
        condition=UnlessCondition(compressed),
        output='screen',
        parameters=[slam_parameters],
        remappings=[
            # 双目映射关系
            ("left/image_rect", left_image_topic),
            ("right/image_rect", right_image_topic),
            ("left/camera_info", left_camera_info),
            ("right/camera_info", right_camera_info),
            
            ("odom", LaunchConfiguration('odom_topic')),
            ("map", LaunchConfiguration('map_topic')),
            ("scan", LaunchConfiguration('scan_topic'))
        ],
        arguments=['--delete_db_on_start']
    )

    rtabmap_stereo_compressed = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_stereo_compressed',
        namespace=namespace,
        condition=IfCondition(compressed),
        output='screen',
        parameters=[slam_parameters],
        remappings=[
            ("left/image_rect", [left_image_topic, '_relay']),
            ("right/image_rect", [right_image_topic, '_relay']),
            ("left/camera_info", left_camera_info),
            ("right/camera_info", right_camera_info),

            ("odom", LaunchConfiguration('odom_topic')),
            ("map", LaunchConfiguration('map_topic')),
            ("scan", LaunchConfiguration('scan_topic'))
        ],
        arguments=['--delete_db_on_start']
    )


    return LaunchDescription([
        # 基础参数
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('subscribe_scan', default_value='false'),
        DeclareLaunchArgument('subscribe_imu', default_value='false'),
        DeclareLaunchArgument('compressed', default_value='false'),

        # TF
        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('publish_tf_map', default_value='true'),

        # 双目 Topic 设置 (根据你的相机驱动修改)
        DeclareLaunchArgument('left_image_topic', default_value='/camera/left/image_raw'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/right/image_raw'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/camera/left/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/camera/right/camera_info'),
        
        DeclareLaunchArgument('odom_topic', default_value='/ekf/odom'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),

        # -------------------
        # 解压
        # -------------------
        republish_left,
        republish_right,

        # -------------------
        # RTAB-Map
        # -------------------
        rtabmap_stereo,
        rtabmap_stereo_compressed,

    ])