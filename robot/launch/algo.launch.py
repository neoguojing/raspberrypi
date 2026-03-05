import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition,UnlessCondition

def generate_launch_description():
    pkg_path = get_package_share_directory('robot')
    config_dir = os.path.join(pkg_path, 'config')
    stereo_params_file = os.path.join(config_dir, 'nav2_params_stereo.yaml')
    # ===============================
    # 参数
    # ===============================
    use_sim_time = LaunchConfiguration('use_sim_time')
    compressed = LaunchConfiguration('compressed')
    slam_backend = LaunchConfiguration('slam_backend')
    sensor_mode = LaunchConfiguration('sensor_mode')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('compressed', default_value='true'),
        DeclareLaunchArgument(
            'slam_backend',
            default_value='rtabmap',
            description='orbslam3 | rtabmap | slam_toolbox'
        ),
        DeclareLaunchArgument(
            'sensor_mode',
            default_value='stereo',
            description='mono | stereo | laser'
        ),
    ]

    common_args = {'use_sim_time': use_sim_time}.items()
    # 图像去畸变
    image_proc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'image_proc.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'sensor_mode': sensor_mode
        }.items()
    )

    # 包含 efk 节点
    efk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'ekf.launch.py')),
        launch_arguments=common_args
    )

    # slam3 momo
    orbslam3_mono = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'orbslam3_mono.launch.py')
        ),
        launch_arguments=common_args,
        condition=IfCondition(
            PythonExpression([
                "'", slam_backend, "' == 'orbslam3' and '",
                sensor_mode, "' == 'mono'"
            ])
        )
    )

    orbslam3_stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'orbslam3_stereo.launch.py')
        ),
        launch_arguments=common_args,
        condition=IfCondition(
            PythonExpression([
                "'", slam_backend, "' == 'orbslam3' and '",
                sensor_mode, "' == 'stereo'"
            ])
        )
    )


    # 视觉模拟激光
    seg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'seg.launch.py')
        ),
        launch_arguments=common_args,
        condition=IfCondition(
            PythonExpression([
                "'", sensor_mode, "' != 'stereo'"
            ])
        )
    )


    # rtabmap 激光
    rtabmap_laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'compressed': compressed
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", slam_backend, "' == 'rtabmap' and '",
                sensor_mode, "' == 'laser'"
            ])
        )
    )

    # rtabmap 单目
    rtabmap_mono = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rtabmap.momo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'compressed': compressed
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", slam_backend, "' == 'rtabmap' and '",
                sensor_mode, "' == 'mono'"
            ])
        )
    )
    # rtabmap 双目
    rtabmap_stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rtabmap.stereo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'compressed': compressed
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", slam_backend, "' == 'rtabmap' and '",
                sensor_mode, "' == 'stereo'"
            ])
        )
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'slam_toolbox.launch.py')
        ),
        launch_arguments=common_args,
        condition=IfCondition(
            PythonExpression([
                "'", slam_backend, "' == 'slam_toolbox'"
            ])
        )
    )

    # 包含 nv2 节点
    nv2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'nv2.launch.py')),
        launch_arguments=common_args,
        condition=UnlessCondition(
            PythonExpression(["'", sensor_mode, "' == 'stereo'"])
        )
    )

    nv2_stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'nv2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': stereo_params_file
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", sensor_mode, "' == 'stereo'"])
        )
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'ctl.launch.py')),
        launch_arguments=common_args
    )

    return LaunchDescription([
        *declare_args,
        efk_launch,
        orbslam3_mono,
        orbslam3_stereo,
        seg_launch,
        rtabmap_laser,
        rtabmap_mono,
        rtabmap_stereo,
        slam_toolbox,
        nv2_launch,
        nv2_stereo_launch,
        explore_launch,
        image_proc_launch,
    ])