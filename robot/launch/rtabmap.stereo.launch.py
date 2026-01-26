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

        # 视觉特征与闭环
        "Vis/EstimationType": "1",    # 1=Stereo (双目特征估计)
        "Vis/FeatureType": "2",       # ORB
        "Kp/DetectorStrategy": "2",
        "Reg/Strategy": "0",          # 0=Visual, 1=ICP, 2=Both
        
        # 双目匹配参数
        "Stereo/MinDisparity": "1",
        "Stereo/MaxDisparity": "256",
        "Stereo/OpticalFlow": "false", # false 则使用特征匹配，true 则使用光流

        "Odom/Strategy": "1", # 强制使用外部里程计（如果你已经有EKF了）
        "Vis/MaxFeatures": "600",
        "RGBD/OptimizeFromGraphEnd": "false",
    }

    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap', # 或者使用 'stereo_odometry' 如果你需要 RTAB-Map 计算里程计
        name='rtabmap',
        namespace=namespace,
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

    return LaunchDescription([
        # 基础参数
        DeclareLaunchArgument('namespace', default_value='rtabmap'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('subscribe_scan', default_value='false'),
        DeclareLaunchArgument('subscribe_imu', default_value='false'),
        DeclareLaunchArgument('compressed', default_value='false'),

        # TF
        DeclareLaunchArgument('frame_id', default_value='base_link'),
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

        rtabmap_slam,
    ])