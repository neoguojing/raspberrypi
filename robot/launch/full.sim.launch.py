import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    # --- 1. 路径与参数定义 ---
    pkg_path = get_package_share_directory("robot")

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 配置文件路径
    xacro_file = os.path.join(pkg_path, 'config', '4wd_car.urdf.xacro') # 确保文件名正确

    # --- 2. 机器人模型解析 (包含新加入的摄像头和传感器) ---
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 发布机器人状态 (TF: base_link -> camera_link 等)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # --- 3. 仿真环境与生成 ---
    # 启动 Gazebo 基础服务
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 在 Gazebo 中生成小车
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'n20_sg90_robot', '-string', robot_desc],
        output='screen'
    )

    # --- 4. GZ Bridge (通信桥梁：确保摄像头和IMU数据进入ROS) ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 控制与反馈
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # 传感器数据
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )
    
    algo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'algo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        rsp_node,
        gazebo_sim,
        spawn_robot,
        gz_bridge,
        algo_launch
    ])