from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动单目 SLAM 节点
        Node(
            package='orb_slam3_ros2_wrapper',
            executable='mono_node',
            name='orb_slam3_mono',
            output='screen',
            parameters=[
                # 传入配置文件路径
                {'voc_file': '/path/to/ORBvoc.txt'}, 
                {'settings_file': '/path/to/camera_settings.yaml'},
                # 重新映射话题以匹配机器人配置
                {'image_topic': '/robot/camera/image_raw'} 
            ]
        ),
        # ... 可以添加 Nav2 启动文件 ...
    ])