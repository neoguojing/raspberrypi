import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_path = get_package_share_directory('robot')

    # 1. 定义统一的参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false', # 模拟环境默认为 true
        description='Use simulation (Gazebo) clock if true'
    )

    image_compressed = LaunchConfiguration('compressed', default='true')
    declare_image_compressed = DeclareLaunchArgument(
        'compressed',
        default_value='true', # 模拟环境默认为 false
        description='是否压缩图片'
    )

    # 2. 统一定义需要传递给子 Launch 的参数字典
    common_args = {'use_sim_time': use_sim_time}.items()

    slam3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'orbslam3_mono.launch.py')),
        launch_arguments=common_args
    )

    # 包含 efk 节点
    efk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'ekf.launch.py')),
        launch_arguments=common_args
    )

    # 包含 seg 节点 安装包不兼容,需要从外部zenoh 写入
    seg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'seg.launch.py')),
        launch_arguments=common_args
    )

    # 包含 rtabmap 节点
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'rtabmap.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                   'compressed': image_compressed
                }.items()
    )

    # 包含 nv2 节点
    nv2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'nv2.launch.py')),
        launch_arguments=common_args
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'ctl.launch.py')),
        launch_arguments=common_args
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_image_compressed,
        # slam3_launch,
        efk_launch,
        seg_launch,
        nv2_launch,
        explore_launch,
        map_launch, #必须在nv2_launch后面，一个bug
    ])