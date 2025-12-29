from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import json
import os
def generate_launch_description():
    """
    ROS 2 启动文件
    """
    
    # 替换 'my_robot_pkg' 为您的 ROS 2 包名
    package_name = 'robot' 
    pkg_share = get_package_share_directory(package_name)

    # 1. 获取 JSON 文件路径 (假设放在你功能包的 config 目录下)
    config_path = os.path.join(
        pkg_share,
        'config',
        'robot_config.json'
    )

    # 2. 读取并解析 JSON 
    with open(config_path, 'r') as f:
        robot_config = json.load(f)
    

    # 2. 配置 CameraPublisherNode
    camera_config_path = os.path.join(pkg_share, 'config', 'imx219.json')
    camera_height = robot_config['tf_frames']['camera_link']['offset']['z'] + robot_config['tf_frames']['base_link']['offset']['z']
    seg_node = Node(
        package=package_name,
        executable='seg_to_scan_node', # 确保在 setup.py 中定义了此入口点
        name='seg_to_scan_node',
        output='screen',
        parameters=[
            {'camera_x_offset': robot_config['tf_frames']['camera_link']['offset']['x']},
            {'camera_height': camera_height},
            {'camera_pitch': robot_config['tf_frames']['camera_link']['offset']['pitch']},
            {'max_detection_range': 5.0},
            {'config_path' : camera_config_path},
        ]
    )
    

    # 3. 返回启动描述
    return LaunchDescription([
        seg_node
    ])