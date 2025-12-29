import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # ======================
            # 1. 订阅与高频同步
            # ======================
            'frame_id': 'base_link',
            'subscribe_rgb': True,
            'subscribe_depth': False,
            'subscribe_imu': True,
            'approx_sync': True,
            'queue_size': 50,               # 内存充足，增大队列缓冲
            'wait_imu_to_init': True,

            # ======================
            # 2. 视觉增强（PC级性能）
            # ======================
            'Mem/StereoFromMotion': 'true',  # 单目核心
            'Vis/EstimationType': '0',       # 2D->2D 极线几何（单目首选）
            'Vis/MinInliers': '20',          # 提高门槛，增加位姿鲁棒性
            'Vis/FeatureType': '8',          # 0=SURF 6=GFTT 8=ORB (PC端可改用 0-SURF 精度更高)
            'Vis/MaxFeatures': '1000',       # 增加特征点数量（默认500）
            'Vis/BundleAdjustment': '1',     # 开启全局/局部束调整优化
            'Vis/CorType': '0',              # 0=Features Matching, 1=Optical Flow
            'Vis/PnPFlags': '0',

            # ======================
            # 3. 栅格地图 (Grid Map) 精度提升
            # ======================
            'Grid/3D': 'false',               # PC端可以处理3D点云投影
            'Grid/FromDepth': 'true',
            'Grid/RangeMax': '3.0',          # 性能好可以看得更远
            'Grid/CellSize': '0.05',         # 恢复 5cm 分辨率
            'Grid/RayTracing': 'true',       # 必须开启，清除单目深度浮影
            'Grid/ClusterRadius': '0.05',    # 噪点过滤阈值
            'Grid/NormalSegmentation': 'false', # 利用法线分割地面与障碍物，更精准

            # ======================
            # 4. 回环检测与优化
            # ======================
            'Kp/MaxFeatures': '800',         # 回环检测特征点翻倍，提高识别率
            'RGBD/OptimizeFromGraphEnd': 'true', # 设置为false以优化整个全局地图
            'RGBD/PlanarScan': 'false',
            'Optimizer/Strategy': '1',       # 0=g2o, 1=gtsam (PC推荐用1-gtsam，精度更高)

            # ======================
            # 5. 存储与内存管理 (关闭树莓派那种激进限制)
            # ======================
            'Rtabmap/DetectionRate': '2',    # 提高到 2Hz 更新频率
            'Mem/IncrementalMemory': 'true',
            'Mem/NotLinkedNodesKept': 'true', # 保留更多未连接节点用于回环
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', '/imu/data_raw'),
            ('odom', '/ekf/odom'),
            ('grid_map', '/map'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        rtabmap_node,
    ])
