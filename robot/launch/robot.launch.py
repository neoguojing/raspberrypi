from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import json
import os
def generate_launch_description():
    """
    ROS 2 启动文件，用于同时启动 CarDriverNode 和 CameraPublisherNode。
    """
    
    # 替换 'my_robot_pkg' 为您的 ROS 2 包名
    package_name = 'robot' 
    pkg_share = get_package_share_directory(package_name)

    # 1. 获取 JSON 文件路径 (假设放在你功能包的 config 目录下)
    robot_config_path = os.path.join(
        pkg_share,
        'config',
        'robot_config.json'
    )

    camera_config_path = os.path.join(
        pkg_share,
        'config',
        'imx219.json'
    )

    # 2. 读取并解析 JSON 
    with open(robot_config_path, 'r') as f:
        robot_config = json.load(f)

    # 1. 配置 CarDriverNode
    static_tf_pub_node = Node(
        package=package_name,
        executable='static_tf_pub_node', # 确保在 setup.py 中定义了此入口点
        name='static_tf_pub_node',
        output='screen',
        parameters=[{
            'tf_data': json.dumps(robot_config['tf_frames']) # 序列化为字符串传入
        }]
    )
    
    # 1. 配置 CarDriverNode
    car_driver_node = Node(
        package=package_name,
        executable='car_driver_node', # 确保在 setup.py 中定义了此入口点
        name='car_driver_node',
        output='screen'
    )
    

    # 2. 配置 CameraPublisherNode
    camera_publisher_node = Node(
        package=package_name,
        executable='camera_publisher_node', # 确保在 setup.py 中定义了此入口点
        name='camera_publisher_node',
        output='screen',
        parameters=[
            {'camera_frequency': 15.0},
            {'is_camera': True},
            {'source': ''},
            {'compressed': True},
            {'camera_config': camera_config_path},
        ]
    )

    icm20948_spi_node = Node(
        package=package_name,
        executable='icm20948_spi_node', # 确保在 setup.py 中定义了此入口点
        name='icm20948_spi_node',
        output='screen',
        parameters=[
            {'imu_frequency': 100}
        ]
    )
    

    # 3. 返回启动描述
    return LaunchDescription([
        # static_tf_pub_node,
        car_driver_node,
        camera_publisher_node,
        # icm20948_spi_node
    ])