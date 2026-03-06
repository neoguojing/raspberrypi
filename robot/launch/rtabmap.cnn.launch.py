import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

# https://github.com/magicleap/SuperGluePretrainedNetwork/tree/master/models/weights

def generate_launch_description():
    # --- 环境变量设置：确保 Python 能找到 RTAB-Map 的 CNN 脚本 ---
    # 通常脚本位于 rtabmap 源码或安装路径的特定目录下
    
    # --- LaunchConfigurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_stereo = LaunchConfiguration('subscribe_stereo') # 关键：是否为双目模式
    
    # Topic 映射定义
    left_img = LaunchConfiguration('left_image_topic')
    right_img = LaunchConfiguration('right_image_topic')
    left_info = LaunchConfiguration('left_camera_info_topic')
    right_info = LaunchConfiguration('right_camera_info_topic')
    rgb_img = LaunchConfiguration('rgb_topic')
    depth_img = LaunchConfiguration('depth_topic')
    camera_info = LaunchConfiguration('camera_info_topic')

    # ===============================
    # 1. 核心 SLAM 参数 (CNN + 高性能 PC 优化版)
    # ===============================
    slam_parameters = {
        "use_sim_time": use_sim_time,
        "frame_id": LaunchConfiguration('frame_id'),
        "odom_frame_id": LaunchConfiguration('odom_frame_id'),
        "map_frame_id": LaunchConfiguration('map_frame_id'),
        "publish_tf": LaunchConfiguration('publish_tf_map'),
        "approx_sync": True,
        "queue_size": 100, # 高性能 PC 可以支撑更大的队列缓存

        # 模式订阅控制
        "subscribe_stereo": is_stereo,
        "subscribe_rgb": UnlessCondition(is_stereo),
        "subscribe_depth": UnlessCondition(is_stereo),
        "subscribe_scan": LaunchConfiguration('subscribe_scan'),
        "subscribe_imu": LaunchConfiguration('subscribe_imu'),

        # --- CNN 前端：SuperPoint (特征提取) ---
        "Vis/FeatureType": "11",              # 11=SuperPoint
        "SuperPoint/ModelPath": LaunchConfiguration('superpoint_model'),
        "SuperPoint/Threshold": "0.01",
        "SuperPoint/NMS": "true",
        "SuperPoint/NMSRadius": "4",
        "SuperPoint/Cuda": "true",            # 强制开启 GPU

        # --- CNN 后端：SuperGlue (特征匹配) ---
        "Vis/CorNNType": "6",                 # 6=SuperGlue
        "PyMatcher/Path": LaunchConfiguration('superglue_script'),
        "PyMatcher/Model": "indoor",          # 针对室内环境
        "PyMatcher/Iterations": "20",
        "PyMatcher/Threshold": "0.2",
        "PyMatcher/Cuda": "true",

        # --- 视觉质量与回环校验 ---
        "Vis/MaxFeatures": "1000",            # SP 特征质量极高，1000点足以
        "Vis/MinInliers": "35",               # CNN 下 35+ 的匹配非常可靠
        "Vis/BundleAdjustment": "1",          # 开启平差优化
        "Vis/EstimationType": "0",            # 0=3D->2D (PnP)
        
        # --- 地图生成 (LiDAR + 视觉融合) ---
        "Grid/Sensor": "2",                   # 2=Both (LiDAR用于平面，视觉用于垂直)
        "Grid/FromDepth": "false",            # 既然有雷达，建图底图以雷达为准
        "Grid/RayTracing": "true",            # 清理动态障碍
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "5.0",               # 感知范围扩展到 5 米
        "Grid/MaxDepth": "5.0",               # 双目点云也看 5 米
        
        # --- 后端优化与稳定性 (解决大地图错误) ---
        "Reg/Strategy": "2",                  # 0=Vis, 1=ICP, 2=Vis+ICP
        "Reg/Force3DoF": "true",              # Ackermann 强制 2D
        "Optimizer/Strategy": "1",            # 1=g2o (PC 推荐)
        "Optimizer/Robust": "true",           # 开启鲁棒核函数
        "RGBD/OptimizeMaxError": "1.0",       # 收紧误差限制
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityMaxGraphDepth": "0",   # 全局搜索
        "RGBD/OptimizeFromGraphEnd": "true",
        "Rtabmap/DetectionRate": "2.0",       # PC 性能支持 2Hz 更新
    }

    # ===============================
    # 2. RTAB-Map 节点重构 (统一入口)
    # ===============================
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_cnn',
        output='screen',
        parameters=[slam_parameters],
        remappings=[
            # 双目模式映射
            ("left/image_rect", left_img),
            ("right/image_rect", right_img),
            ("left/camera_info", left_info),
            ("right/camera_info", right_info),
            # 单目/RGBD模式映射
            ("rgb/image_rect", rgb_img),
            ("depth/image_rect", depth_img),
            ("rgb/camera_info", camera_info),
            # 通用话题
            ("odom", LaunchConfiguration('odom_topic')),
            ("scan", LaunchConfiguration('scan_topic')),
            ("imu", "/imu/data_raw")
        ],
        arguments=['--delete_db_on_start']
    )

    return LaunchDescription([
        # 基础运行参数
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('subscribe_stereo', default_value='true', description='Switch between Stereo and RGBD'),
        DeclareLaunchArgument('subscribe_scan', default_value='true'),
        DeclareLaunchArgument('subscribe_imu', default_value='true'),

        # 权重文件路径 (务必修改为真实路径)
        DeclareLaunchArgument('superpoint_model', default_value='/home/user/models/superpoint_v1.pth'),
        DeclareLaunchArgument('superglue_script', default_value='/home/user/rtabmap/corelib/src/python/SuperGlue.py rtabmap_superglue.py'),

        # Topic 设置
        DeclareLaunchArgument('left_image_topic', default_value='/camera/left/image_rect'),
        DeclareLaunchArgument('right_image_topic', default_value='/camera/right/image_rect'),
        DeclareLaunchArgument('left_camera_info_topic', default_value='/camera/left/camera_info'),
        DeclareLaunchArgument('right_camera_info_topic', default_value='/camera/right/camera_info'),
        
        DeclareLaunchArgument('rgb_topic', default_value='/camera/rgb/image_rect'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/rgb/camera_info'),

        DeclareLaunchArgument('odom_topic', default_value='/ekf/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/seg/scan'),

        # 框架定义
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('publish_tf_map', default_value='true'),

        # 启动节点
        rtabmap_node,
    ])