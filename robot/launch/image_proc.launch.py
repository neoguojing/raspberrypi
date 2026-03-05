from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. 声明与获取参数
    sensor_mode = LaunchConfiguration('sensor_mode')
    queue_size = LaunchConfiguration('queue_size')
    
    is_stereo = PythonExpression(["'", sensor_mode, "' == 'stereo'"])

    return LaunchDescription([
        # 声明 Launch 参数，方便从命令行修改
        DeclareLaunchArgument(
            'sensor_mode', 
            default_value='stereo',
            description='mono | stereo | laser'
        ),
        DeclareLaunchArgument(
            'queue_size', 
            default_value='5',
            description='Queue size for image message synchronization'
        ),

        ComposableNodeContainer(
            name='image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # --- 左目去畸变 & 矫正 ---
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_left',
                    parameters=[{
                        'queue_size': queue_size,
                        'approx_sync': True,             # 开启近似同步，容忍左右目微小时间差        
                    }],
                    remappings=[
                        ('image_raw', '/camera/image_raw'),
                        ('camera_info', '/camera/camera_info'),
                        ('image_rect', '/camera/image_rect')
                    ],
                ),
                # --- 右目去畸变 & 矫正 (仅 Stereo 模式) ---
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_right',
                    condition=IfCondition(is_stereo),
                    parameters=[{
                        'queue_size': queue_size,
                        'approx_sync': True,             # 开启近似同步，容忍左右目微小时间差        
                    }],
                    remappings=[
                        ('image_raw', '/camera/right/image_raw'),
                        ('camera_info', '/camera/right/camera_info'),
                        ('image_rect', '/camera/right/image_rect')
                    ],
                ),
                # --- 视差图生成 (双目匹配) ---
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name='disparity_node',
                    condition=IfCondition(is_stereo),
                    # DisparityNode 通常也需要 queue_size 来同步左右目图像
                    parameters=[{
                        'queue_size': queue_size,
                        'approx_sync': True,             # 开启近似同步，容忍左右目微小时间差        
                    }],
                    remappings=[
                        ('/camera/image_rect', '/camera/image_rect'),
                        ('/camera/right/image_rect', '/camera/right/image_rect'),
                        ('/camera/camera_info', '/camera/camera_info'),
                        ('/camera/right/camera_info', '/camera/right/camera_info'),
                        ('disparity', 'disparity')
                    ],
                )
            ],
            output='screen',
        )
    ])