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
            'publish_tf': False,
            'Reg/Force3DoF': True,
            'Icp/MaxCorrespondenceDistance': 0.15,
        }],
        remappings=[
            ('scan', scan_topic),
            ('odom', '/visual_odom'),
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_mode', default_value='stereo'),

        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),

        # stereo
        DeclareLaunchArgument('left_image', default_value='/camera/left/image_rect'),
        DeclareLaunchArgument('right_image', default_value='/camera/right/image_rect'),
        DeclareLaunchArgument('left_info', default_value='/camera/left/camera_info'),
        DeclareLaunchArgument('right_info', default_value='/camera/right/camera_info'),

        # rgbd
        DeclareLaunchArgument('rgb_image', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_image', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info', default_value='/camera/color/camera_info'),

        # icp
        DeclareLaunchArgument('scan', default_value='/scan'),

        stereo_odom,
        rgbd_odom,
        icp_odom,
    ])
