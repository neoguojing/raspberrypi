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
            # 1. 订阅与 TF / 时间同步
            # ======================
            'frame_id': 'base_footprint',

            'subscribe_rgb': True,
            'subscribe_depth': False,          # 单目
            'subscribe_rgbd': False,
            'subscribe_stereo': False,
            'subscribe_imu': True,

            'approx_sync': True,
            'queue_size': 50,
            'wait_imu_to_init': True,

            # EKF 里程计
            'subscribe_odom': True,
            'odom_topic': '/ekf/odom',

            # TF 发布策略（⚠️关键）
            'publish_tf': True,                # 只发布 map->odom
            'publish_odom': False,             # ❌ 不发布 odom->base
            
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # 点云（供 Nav2 / 调试）
            'Publish/Clouds': 'True',             # /cloud_map, /cloud_obstacles

            # 禁止内部视觉里程计（必须）
            'visual_odometry': False,


            # ======================
            # 2. 单目视觉前端（PC 级性能）
            # ======================
            'Mem/StereoFromMotion': 'True',       # 单目核心机制
            'Stereo/MinDisparity': '1.0',    # 最小视差，影响三角化精度
            'Stereo/MaxDisparity': '100.0',

            # 1. 改变估计类型（最关键！）
            # 默认是 0 (3D->3D)，要求两帧都有深度。
            # 改为 1 (3D->2D)，只要有一帧有深度就能回环。
            'Vis/EstimationType': '1',            # 2D-2D 极线几何（单目最稳）
            'Vis/FeatureType': '8',               # ORB
            'Kp/DetectorStrategy': '8',
            'Vis/MaxFeatures': '600',
            'Vis/MinInliers': '10',               # 单目地面场景更稳（防 TF 跳变）
            'Vis/InlierDistance': '0.1',    # 增加容错，单目尺度不准，距离阈值稍微大一点点

            'Vis/BundleAdjustment': '1',          # 开启 BA
            'Vis/CorType': '0',                   # 特征匹配（非光流）
            'Vis/PnPFlags': '0',

            # 1. 强制单目生成 3D 点云的关键（极其重要）
            'Vis/EstimationType': '1',      # 0=3D->2D (需深度), 1=2D->2D (单目专用)
            'Vis/ForwardEstOnly': 'True',   # 单目通常只支持前向估计
            'Vis/CorGuessWinSize': '0',      # 设置为 0 表示不限制搜索窗口，让它在全图找匹配
            # ======================
            # 3. Grid Map（⚠️单目专用配置）
            # ======================
            'Grid/3D': 'False',                   # 2D 地图
            'Grid/FromDepth': 'False',            # ❌ 单目无真实深度
            'Grid/FromLaserScan': 'False',

            'Grid/RangeMax': '3.0',
            'Grid/CellSize': '0.05',              # 5cm
            'Grid/RayTracing': 'True',             # 清除假障碍（非常重要）
            'Grid/ClusterRadius': '0.05',
            'Grid/NormalSegmentation': 'False',


            # ======================
            # 4. 回环检测与约束优化（单目跑车关键）
            # ======================
            'Kp/MaxFeatures': '400',

            # 使用 EKF odom 精细化闭环（防误匹配）
            'RGBD/NeighborLinkRefining': 'True',

            # 回到空间相近位置时主动尝试闭环
            'RGBD/ProximityBySpace': 'True',

            'RGBD/OptimizeFromGraphEnd': 'True',
            'RGBD/PlanarScan': 'False',

            # 强制视觉注册（无雷达 / 无真实深度）
            'Reg/Strategy': '0',

            # 只做 2D SLAM（⚠️轮式机器人必开）
            'Optimizer/Slam2d': 'True',

            # 图优化器
            'Optimizer/Strategy': '1',             # gtsam（PC 更稳）


            # ======================
            # 5. 内存 / 运行策略（长期运行）
            # ======================
            'Rtabmap/DetectionRate': '2.0',         # 2 Hz（稳定优先）

            'Mem/IncrementalMemory': 'True',
            'Mem/NotLinkedNodesKept': 'True',
            'Mem/IncrementalDictionary': 'True', # 确保词典是增量构建的
            'Mem/InitWMWithAllNodes': 'False',   # 不要尝试加载历史节点
            'Mem/ImagePostExtraction': 'True', # 确保回环时能重新提取特征
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', '/imu/data_raw'),
            ('odom', '/ekf/odom'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        rtabmap_node,
    ])
