import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # -------------------
    # Launch args
    # -------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_mode = LaunchConfiguration('odom_mode')  # stereo | rgbd | icp

    frame_id = LaunchConfiguration('frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')

    # image / depth / scan
    left_image = LaunchConfiguration('left_image')
    right_image = LaunchConfiguration('right_image')
    left_info = LaunchConfiguration('left_info')
    right_info = LaunchConfiguration('right_info')

    rgb_image = LaunchConfiguration('rgb_image')
    depth_image = LaunchConfiguration('depth_image')
    camera_info = LaunchConfiguration('camera_info')

    scan_topic = LaunchConfiguration('scan')

    # ===============================
    # Stereo Odometry
    # ===============================
    stereo_odom = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        condition=IfCondition(PythonExpression([odom_mode, " == 'stereo'"])),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'publish_tf': False,
            'approx_sync': True,
            'queue_size': 30,
        }],
        remappings=[
            ('left/image_rect', left_image),
            ('right/image_rect', right_image),
            ('left/camera_info', left_info),
            ('right/camera_info', right_info),
            ('odom', '/visual_odom'),
        ],
    )

    # ===============================
    # RGB-D Odometry
    # ===============================
    rgbd_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        condition=IfCondition(PythonExpression([odom_mode, " == 'rgbd'"])),

        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'publish_tf': False,
            'approx_sync': True,
        }],
        remappings=[
            ('rgb/image', rgb_image),
            ('depth/image', depth_image),
            ('rgb/camera_info', camera_info),
            ('odom', '/visual_odom'),
        ],
    )

    # ===============================
    # ICP Odometry
    # ===============================
    icp_odom = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='rtabmap_odometry',
        condition=IfCondition(PythonExpression([odom_mode, " == 'icp'"])),

        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'odom_frame_id': odom_frame_id,
            'publish_tf': 'false',
            # 策略与算法
            "Icp/Strategy": "1",               # 0=点到点, 1=点到面 (推荐)
            "Icp/PointToPlane": "true",        # 配合 Strategy 1
            "Icp/Iterations": "100",           # 高算力支持多次迭代以求极致精度
            "Icp/VoxelSize": "0.05",           # 下采样去噪，5cm 足够
            "Icp/MaxCorrespondenceDistance": "0.15", # 匹配搜索半径
            
            # 运动补偿
            "Odom/GuessMotion": "true",        # 开启运动预测，显著降低丢帧概率
            "Odom/Strategy": "0",              # 0=Frame-to-Map (通常更稳), 1=Frame-to-Frame
            
            # 鲁棒性
            "Icp/CorrespondenceRatio": "0.15", # 至少 15% 的点匹配上才算成功
            "Icp/OutlierRatio": "0.85",        # 剔除离群点的比例
            
            # 硬件优化
            "Odom/ScanKeyFrameThr": "0.9",     # 当新帧与关键帧重叠度低于 90% 时更新参考帧
            "Reg/Force3DoF": "true",           # Ackermann 机器人强制 2D 匹配 (锁定 Z/Roll/Pitch)
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', '/visual_odom'),
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_mode', default_value='icp'),

        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),

        # stereo
        DeclareLaunchArgument('left_image', default_value='/camera/image_rect'),
        DeclareLaunchArgument('right_image', default_value='/camera/right/image_rect'),
        DeclareLaunchArgument('left_info', default_value='/camera/camera_info'),
        DeclareLaunchArgument('right_info', default_value='/camera/right/camera_info'),

        # rgbd
        DeclareLaunchArgument('rgb_image', default_value='/camera/image_rect'),
        DeclareLaunchArgument('depth_image', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info', default_value='/camera/camera_info'),

        # icp
        DeclareLaunchArgument('scan', default_value='/seg/scan'),

        stereo_odom,
        rgbd_odom,
        icp_odom,
    ])
