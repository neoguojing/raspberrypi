import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # --- 1. 路径与参数定义 ---
    pkg_path = get_package_share_directory("robot")

    # 配置文件路径
    xacro_file = os.path.join(pkg_path, 'config', '4wd_car_stereo.urdf.xacro') # 确保文件名正确

    # --- 2. 机器人模型解析 (包含新加入的摄像头和传感器) ---
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 发布机器人状态 (TF: base_link -> camera_link 等)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # --- 3. 仿真环境与生成 --- 在宿主机
    # 启动 Gazebo 基础服务
    # gazebo_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
    #     launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    # )

    # 在 Gazebo 中生成小车
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'n20_sg90_robot_stereo',
            '-string', robot_desc,
            '-world', 'default',      # 明确指定要在名为 warehouse 的世界中生成
            '-x', '2.0',                # X 坐标偏移
            '-y', '0.0',                # Y 坐标
            '-z', '0.2',                # Z 轴抬高，避免压到地板或碰撞盒重叠
            '-R', '0.0',                # 欧拉角：滚转
            '-P', '0.0',                # 欧拉角：俯仰
            '-Y', '1.57'                # 欧拉角：偏航 (1.57弧度约为90度，改变朝向)
        ],
        # arguments=[
        #     '-name', 'n20_sg90_robot',
        #     '-string', robot_desc,
        #     '-world', 'default',      # 明确指定要在名为 bedroom 的世界中生成
        #     '-x', '-1.0',
        #     '-y', '0.0',
        #     '-z', '0.1',
        #     '-Y', '0.0' # 这里的 Y 是 Yaw 偏航角
        # ],
        output='screen'
    )

    # --- 4. GZ Bridge (通信桥梁：确保摄像头和IMU数据进入ROS) ---
    # --- 4. GZ Bridge (修改为双目适配版) ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. 基础控制与时间同步
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # 2. 双目流 (必须与你的 <topic>/stereo/image_raw</topic> 标签一致)
            # Gazebo 会自动在 Topic 后面追加 /left/image_raw 和 /right/image_raw
            
            # --- 左目 ---
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # --- 右目 ---
            '/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        rsp_node,
        spawn_robot,
        gz_bridge,
    ])