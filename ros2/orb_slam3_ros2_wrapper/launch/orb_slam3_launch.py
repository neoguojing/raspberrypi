import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义包名和配置目录路径
    pkg_name = 'orb_slam3_ros2_wrapper'
    pkg_share_dir = get_package_share_directory(pkg_name)
    config_dir = os.path.join(pkg_share_dir, 'config')

    # 2. 声明 Launch 参数，允许用户在启动时覆盖默认值

    # 词典文件 (ORB-SLAM3 核心文件)
    voc_file_arg = DeclareLaunchArgument(
        'voc_file',
        default_value=os.path.join(config_dir, 'ORBvoc.txt'),
        description='Path to the ORB Vocabulary file (ORBvoc.txt)'
    )

    # 相机配置文件 (包含内参、IMU参数等)
    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value=os.path.join(config_dir, 'camera_and_imu_settings.yaml'),
        description='Path to the configuration YAML file for camera and sensors'
    )
    
    # 图像话题名称
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/robot/camera/image_raw',
        description='Input image topic'
    )

    # IMU话题名称 (仅 mono_imu_node 需要)
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='Input IMU data topic (for mono_imu_node)'
    )

    # 选择启动哪个节点 (可选参数，方便调试)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mono',
        description='SLAM mode to launch: "mono" or "mono_imu"'
    )

    # 3. 定义两个节点，并根据 mode 参数选择性启动
    
    # --- 单目 SLAM 节点 (mono_node) ---
    orb_slam3_mono_node = Node(
        package=pkg_name,
        executable='mono_node',
        name='orb_slam3_mono',
        output='screen',
        # condition=IfEqualsSubstitution(LaunchConfiguration('mode'), 'mono'), # ROS 2 暂无 IfEqualsSubstitution, 使用 Python 逻辑处理或在外部选择
        parameters=[
            {'voc_file': LaunchConfiguration('voc_file')},
            {'settings_file': LaunchConfiguration('settings_file')},
            {'image_topic': LaunchConfiguration('image_topic')},
        ]
    )

    # --- 单目惯导 SLAM 节点 (mono_imu_node) ---
    orb_slam3_mono_imu_node = Node(
        package=pkg_name,
        executable='mono_imu_node',
        name='orb_slam3_mono_imu',
        output='screen',
        # condition=IfEqualsSubstitution(LaunchConfiguration('mode'), 'mono_imu'),
        parameters=[
            {'voc_file': LaunchConfiguration('voc_file')},
            {'settings_file': LaunchConfiguration('settings_file')},
            {'image_topic': LaunchConfiguration('image_topic')},
            {'imu_topic': LaunchConfiguration('imu_topic')}, # 惯导节点需要IMU话题
        ]
    )
    
    # 4. 返回 LaunchDescription
    # 注意：由于 ROS 2 launch 没有内置的 IfEqualsSubstitution，我们在这里简单地将两个节点都包含进去。
    # 
    # 在实际使用中，推荐使用 launch.conditions.IfCondition 或 launch.conditions.UnlessCondition 
    # 结合 LaunchConfiguration('mode') 来实现条件启动。
    # 为简洁起见，我们暂时将它们放在列表中，但请注意，如果同时运行，它们将互相冲突。
    # 
    # 如果您需要严格的条件控制，您应该使用 ros2 launch 的内置条件判断：
    # from launch.conditions import IfCondition, UnlessCondition
    
    return LaunchDescription([
        # 声明参数
        voc_file_arg,
        settings_file_arg,
        image_topic_arg,
        imu_topic_arg,
        mode_arg, # 即使暂时不用 IfCondition，也保留 mode 参数以便将来升级

        # 启动节点
        # 【重要】用户需要通过命令行覆盖 'mode' 参数，并使用 IfCondition 来控制节点的启动。
        # 
        # 为了避免同时启动，您可以手动创建两个单独的 launch 文件，或者像这样添加条件：
        
        Node(
            package=pkg_name,
            executable='mono_node',
            name='orb_slam3_mono',
            output='screen',
            condition=IfCondition(
                '"{}" == "mono"'.format(LaunchConfiguration('mode'))
            ),
            parameters=[
                {'voc_file': LaunchConfiguration('voc_file')},
                {'settings_file': LaunchConfiguration('settings_file')},
                {'image_topic': LaunchConfiguration('image_topic')},
            ]
        ),

        Node(
            package=pkg_name,
            executable='mono_imu_node',
            name='orb_slam3_mono_imu',
            output='screen',
            condition=IfCondition(
                '"{}" == "mono_imu"'.format(LaunchConfiguration('mode'))
            ),
            parameters=[
                {'voc_file': LaunchConfiguration('voc_file')},
                {'settings_file': LaunchConfiguration('settings_file')},
                {'image_topic': LaunchConfiguration('image_topic')},
                {'imu_topic': LaunchConfiguration('imu_topic')},
            ]
        ),
    ])