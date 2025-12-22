import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # RTAB-Map 核心节点
    rtabmap_node = Node(
        package='rtabmap_slam', # 注意：ROS 2 Humble 及之后版本包名为 rtabmap_slam
        executable='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,    # 如果有雷达可以设为 True 增强地图
            'approx_sync': True,
            'queue_size': 30,           # 树莓派建议增加队列缓存

            # --- 地图增强参数 ---
            'RGBD/NeighborLinkRefining': 'true', # 闭环检测后细化邻居连接
            'RGBD/ProximityBySpace': 'true',     # 空间近接检测
            'RGBD/AngularUpdate': '0.01',        # 即使旋转很小也更新地图
            'RGBD/LinearUpdate': '0.01',         # 即使移动很小也更新地图
            
            # --- 2.D 占据栅格地图增强 (供 Nav2 使用) ---
            'Grid/FromKnownArea': 'false',       # 设为 false 以实时动态发现环境
            'Grid/RayTracing': 'true',           # 启用射线追踪，清除动态障碍物
            'Grid/3D': 'false',                  # 强制输出 2D 地图
            'Grid/CellSize': '0.05',             # 分辨率需与 Nav2 YAML 保持一致
            'Grid/MaxObstacleHeight': '1.8',     # 过滤天花板
            'Grid/MinGroundHeight': '0.05',      # 过滤地面噪点
            
            # --- 内存与性能优化 (针对树莓派) ---
            'Mem/IncrementalMemory': 'true', 
            'Mem/ReduceGraph': 'true',           # 减少位姿图节点以节省内存
            'DbSqlite3/CacheSize': '10000',      # 增加数据库缓存提高读写速度
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom', '/slam3/odom'),                   # 此处需指向 ORB-SLAM3 发布的里程计
            ('grid_map', '/map')                 # 将 RTAB-Map 输出的地图映射给 Nav2
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rtabmap_node,
    ])