import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取 rtabmap_ros 包的共享目录
    rtabmap_pkg_dir = get_package_share_directory('rtabmap_ros')

    # 2. 声明 Launch 文件参数
    # 是否使用 GUI (Rtabmap viz)
    use_rtabmap_viz = LaunchConfiguration('rtabmap_viz', default='true')
    
    # 传感器数据话题
    rgb_topic = LaunchConfiguration('rgb_topic', default='/camera/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera/color/camera_info')
    
    # 里程计话题 (通常由独立节点提供，如 camera_link_tf_odom)
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')

    # 3. RTAB-Map SLAM 节点配置
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        output='screen',
        parameters=[{
            # 基本配置
            'frame_id': 'base_link',  # 机器人基座坐标系
            'subscribe_depth': True,  # 订阅深度图像
            'subscribe_rgb': True,    # 订阅 RGB 图像
            'subscribe_odom': True,   # 订阅里程计
            'approx_sync': True,      # 近似时间同步

            # SLAM 和建图参数
            'Mem/IncrementalMemory': 'true',  # 启用增量建图
            'Mem/InitWMWithZeroCfg': 'true',  # 避免首次回环检测时出现错误的匹配
            'Rtabmap/DetectionRate': '5',     # 检测频率（Hz）
            'RGBD/ProximityBySpace': 'true',  # 允许空间上的回环检测
            'Reg/Force3DoF': 'true',          # 仅优化 X/Y/Yaw (如果运行2D SLAM)
            
            # 地图输出：发布 2D 占用栅格地图
            'Grid/FromKnownArea': 'true',     
            'Grid/CellSize': '0.05',          # 栅格分辨率 (米)
            
            # 数据库路径 (可选)
            # 'database_path': 'rtabmap.db' 
        }],
        remappings=[
            # 话题重映射
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('odom', odom_topic)
        ],
    )
    
    # 4. 可视化节点配置 (可选)
    rtabmap_viz_node = Node(
        package='rtabmap_ros',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{'frame_id': 'base_link'}],
        condition=launch.conditions.IfCondition(use_rtabmap_viz)
    )

    return LaunchDescription([
        # 声明参数供用户在命令行修改
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='是否启动 rtabmap 可视化界面'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw', description='RGB 图像话题'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw', description='深度图像话题'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info', description='相机信息话题'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='里程计话题'),

        rtabmap_node,
        rtabmap_viz_node,
        
        # 另外启动 RViz2 观察地图 (可选)
        # IncludeLaunchDescription(...) 
    ])