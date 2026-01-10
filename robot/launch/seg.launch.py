from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    """
    ROS 2 启动文件
    """
    
    # 替换 'my_robot_pkg' 为您的 ROS 2 包名
    package_name = 'robot' 
    pkg_share = get_package_share_directory(package_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 2. 配置 CameraPublisherNode
    seg_node = Node(
        package=package_name,
        executable='seg_to_scan_node', # 确保在 setup.py 中定义了此入口点
        name='seg_to_scan_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'zenoh_topic': 'rt/scan',
            'ros_topic': '/seg/scan',
            'frame_id': 'base_footprint'
        }]
    )
    

    # 3. 返回启动描述
    return LaunchDescription([
        seg_node
    ])