import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 路径设置 ---
    
    # 默认路径（可以被启动参数覆盖）
    default_voc_path = "/home/ros_user/orbslam3/vocabulary/ORBvoc.txt"
    default_yaml_path = "/home/ros_user/orbslam3/config/monocular/EuRoC.yaml"

    # --- 2. 声明启动参数 (可在命令行修改) ---
    voc_path_arg = DeclareLaunchArgument(
        'voc_path',
        default_value=default_voc_path,
        description='Path to ORB-SLAM3 vocabulary file'
    )

    yaml_path_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=default_yaml_path,
        description='Path to ORB-SLAM3 camera config file'
    )

    # --- 3. 定义节点 ---
    orb_slam3_node = Node(
        package='orbslam3',          # 你的 colcon 包名
        executable='mono',           # 你的 C++ 可执行文件名
        name='orbslam3_mono',
        output='screen',
        # ORB-SLAM3 的 C++ 节点通常通过 main(argc, argv) 接收参数
        arguments=[
            LaunchConfiguration('voc_path'),
            LaunchConfiguration('yaml_path')
        ],
        parameters=[{
            'use_sim_time': True,    # 模拟环境下必须开启
        }],
        # 映射到你小车发布的摄像头话题
        remappings=[
            ('/camera/image_raw', '/camera/image_raw')
        ]
    )

    return LaunchDescription([
        voc_path_arg,
        yaml_path_arg,
        orb_slam3_node
    ])