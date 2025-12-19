from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    ROS 2 启动文件，用于同时启动 CarDriverNode 和 CameraPublisherNode。
    """
    
    # 替换 'my_robot_pkg' 为您的 ROS 2 包名
    package_name = 'robot' 

    # 1. 配置 CarDriverNode
    car_driver_node = Node(
        package=package_name,
        executable='car_driver_node', # 确保在 setup.py 中定义了此入口点
        name='car_driver_node',
        output='screen',
        parameters=[
            {'odom_frequency': 30.0}
        ]
    )
    

    # 2. 配置 CameraPublisherNode
    camera_publisher_node = Node(
        package=package_name,
        executable='camera_publisher_node', # 确保在 setup.py 中定义了此入口点
        name='camera_publisher_node',
        output='screen',
        parameters=[
            {'camera_frequency': 15.0}
        ]
    )
    

    # 3. 返回启动描述
    return LaunchDescription([
        car_driver_node,
        camera_publisher_node
    ])