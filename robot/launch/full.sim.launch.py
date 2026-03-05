from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. 声明参数
    sensor_mode_arg = DeclareLaunchArgument(
        'sensor_mode',
        default_value='stereo',
        description='mono | stereo | laser'
    )

    sensor_mode = LaunchConfiguration('sensor_mode')

    # 2. 定义条件判断：是否为双目模式
    # 当 sensor_mode == 'stereo' 时，这个表达式返回 'True' (字符串)
    is_stereo = PythonExpression(["'", sensor_mode, "' == 'stereo'"])

    container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 第一个节点：单目或双目左目 (始终启动)
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node_left',
                # 假设单目使用 'camera/image_raw'，双目左目也用同样的或自定义
                remappings=[
                    ('image_raw', 'left/image_raw'),
                    ('camera_info', 'left/camera_info'),
                    ('image_rect', 'left/image_rect')
                ],
            ),
            
            # 第二个节点：仅在 stereo 模式下启动
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node_right',
                condition=IfCondition(is_stereo), # 核心控制逻辑
                remappings=[
                    ('image_raw', 'right/image_raw'),
                    ('camera_info', 'right/camera_info'),
                    ('image_rect', 'right/image_rect')
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        sensor_mode_arg,
        container
    ])