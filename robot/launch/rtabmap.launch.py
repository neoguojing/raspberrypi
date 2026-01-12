import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from typing import Text, List, Tuple


# 条件转换辅助类（可选）
class ConditionalText:
    def __init__(self, text_if: str, text_else: str, condition: LaunchConfiguration):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: LaunchContext) -> Text:
        cond_val = self.condition.perform(context).lower()
        return self.text_if if cond_val in ['true', '1'] else self.text_else


def remap_rgb_image(context) -> List[Tuple[str, str]]:
    """根据压缩参数返回 rgb/image remap
    压缩模式订阅 republish 输出的 relay topic
    非压缩模式直接订阅原始 topic
    """
    rgb_topic = LaunchConfiguration('rgb_topic').perform(context)
    compressed = LaunchConfiguration('compressed').perform(context).lower() in ['true', '1']
    # 压缩模式订阅 republish 输出
    return [("rgb/image", f"{rgb_topic}_relay" if compressed else rgb_topic)]


def get_republish_remap(context) -> List[Tuple[str, str]]:
    """
    配置 republish 节点的话题映射关系。
    
    该节点的作用是：[压缩话题] -> [解压] -> [原始话题(relay)]
    
    映射逻辑解释：
    1. "in/{transport}" : image_transport 节点的标准输入接口。
       例如：当 transport 为 'compressed' 时，映射 'in/compressed' 
       对应到实际话题 '/camera/image_raw/compressed'。
       
    2. "out" : image_transport 节点的标准输出接口（发布 raw 格式）。
       映射到 '/camera/image_raw_relay'，供下游 RTAB-Map 节点订阅。
    """
    
    # 获取基础话题名，例如: /camera/image_raw
    rgb_topic = LaunchConfiguration('rgb_topic').perform(context)

    return [
        # 输入：将虚拟的 in/compressed 绑定到真实的 /camera/image_raw/compressed
        (f"in/compressed", f"{rgb_topic}/compressed"),
        
        # 输出：将解压后的 raw 图像发布到后缀为 _relay 的新话题上
        ("out", f"{rgb_topic}_relay")
    ]


def print_params(context: LaunchContext, *args, **kwargs):
    rgb_topic = LaunchConfiguration('rgb_topic').perform(context)
    compressed = LaunchConfiguration('compressed').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    print(f"[Launch Debug] rgb_topic={rgb_topic}, compressed={compressed}, use_sim_time={use_sim_time}")
    return []

def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace')
    use_sim_time_val = LaunchConfiguration('use_sim_time')
    
    print_params(context)
    return [
        # ===============================
        # 1. 图像解压节点 (针对单目 RGB 压缩)
        # ===============================
        Node(
            package='image_transport',
            executable='republish',
            name='republish_rgb',
            condition=IfCondition(LaunchConfiguration('compressed')),
            parameters=[{'use_sim_time': use_sim_time_val}],
            remappings=get_republish_remap(context),
            arguments=['compressed', 'raw'],
            namespace=ns
        ),

        # ===============================
        # 2. RTAB-Map 核心节点
        # ===============================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=[
                '--delete_db_on_start',
                '--ros-args',
                '--log-level', 'rtabmap:=warn'
            ],
            parameters=[{
                # 通用
                "use_sim_time": use_sim_time_val,
                "frame_id": LaunchConfiguration('frame_id'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id'),
                "map_frame_id": LaunchConfiguration('map_frame_id'),
                "publish_tf": LaunchConfiguration('publish_tf_map'),
                "approx_sync": True,
                "sync_queue_size": 30,
                "topic_queue_size": 30,

                # 传感器订阅
                "subscribe_rgb": True,
                "subscribe_depth": LaunchConfiguration('depth'),
                "subscribe_stereo": False,
                "subscribe_scan": LaunchConfiguration('subscribe_scan'),
                "subscribe_odom_info": False,  # EKF / wheel odom
                "subscribe_imu": LaunchConfiguration('subscribe_imu'),

                # 地图参数
                "Grid/Sensor": "0",  # 0=激光
                "Grid/FromDepth": "false",
                "Grid/3D": "false",
                "Grid/RangeMax": "4.0",
                "Grid/RayTracing": "true",
                "Grid/CellSize": "0.05",
                "Grid/OctoMap": "false",

                # 视觉特征参数
                "Kp/DetectorStrategy": "2",
                "Kp/MaxFeatures": "1000",
                "Vis/EstimationType": "2",
                "Vis/FeatureType": "2",
                "Vis/EpipolarGeometryVar": "0.5",
                "Vis/Iterations": "300",
                "Vis/MinInliers": "8",
                "Vis/InlierDistance": "0.1",

                # 闭环策略
                "Reg/Strategy": "0",
                "Reg/Force3DoF": "true",
                "RGBD/OptimizeMaxError": "5.0",
                "RGBD/NeighborLinkRefining": "true",

                # 内存管理
                "RGBD/LinearUpdate": "0.1",
                "RGBD/AngularUpdate": "0.1",
                "Mem/IncrementalMemory": "true",
                "Mem/STMSize": "30",

                # 单目尺度恢复
                "Mem/StereoFromMotion": "true",
                "Mem/UseOdomFeatures": "true"
            }],
            remappings=[
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("map", LaunchConfiguration('map_topic')),
                ("scan", LaunchConfiguration('scan_topic'))
            ] + remap_rgb_image(context),
            namespace=ns
        ),
    ]


def generate_launch_description():
    pkg_name = 'robot'  # 替换为你的包名
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')

    return LaunchDescription([
        # -------------------
        # 基础参数
        # -------------------
        DeclareLaunchArgument('namespace', default_value='rtabmap'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('subscribe_scan', default_value='true'),
        DeclareLaunchArgument('subscribe_imu', default_value='false'),
        DeclareLaunchArgument('depth', default_value='false'),
        DeclareLaunchArgument('compressed', default_value='false'),

        # -------------------
        # TF / Frame IDs
        # -------------------
        DeclareLaunchArgument('frame_id', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('publish_tf_map', default_value='true'),

        # -------------------
        # Topic 设置
        # -------------------
        DeclareLaunchArgument('rgb_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/ekf/odom'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('scan_topic', default_value='/seg/scan'),

        OpaqueFunction(function=launch_setup)
    ])
