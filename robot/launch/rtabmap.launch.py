import os
from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from typing import Text

# 条件转换辅助类
class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if, self.text_else, self.condition = text_if, text_else, condition
    def perform(self, context: 'LaunchContext') -> Text:
        return self.text_if if str(self.condition).lower() in ['true', '1'] else self.text_else

def launch_setup(context, *args, **kwargs):
    # 获取命名空间
    ns = LaunchConfiguration('namespace')

    return [
        # 1. 图像解压节点 (针对单目 RGB)
        Node(
            package='image_transport', executable='republish', name='republish_rgb',
            condition=IfCondition(LaunchConfiguration('compressed')),
            remappings=[
                (['in/', LaunchConfiguration('rgb_image_transport')], [LaunchConfiguration('rgb_topic'), '/', LaunchConfiguration('rgb_image_transport')]),
                ('out', [LaunchConfiguration('rgb_topic'), '_relay'])],
            arguments=[LaunchConfiguration('rgb_image_transport'), 'raw'],
            namespace=ns),

        # 2. RTAB-Map SLAM 核心节点
        Node(
            package='rtabmap_slam', executable='rtabmap', name="rtabmap", output="screen",
            arguments=[
                '--delete_db_on_start',
                '--ros-args', 
                '--log-level', 'rtabmap:=error'  # 设置 RTAB-Map 日志级别为 error
            ],
            parameters=[{
                "subscribe_rgb": 'true',
                "subscribe_depth": LaunchConfiguration('depth'), # 默认设为 false
                "subscribe_stereo": 'false',
                "frame_id": LaunchConfiguration('frame_id'),
                "map_frame_id": LaunchConfiguration('map_frame_id'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id'), # 对应 EKF 输出的 odom frame
                "publish_tf": LaunchConfiguration('publish_tf_map'),   # 发布 map -> odom
                "use_sim_time": LaunchConfiguration('use_sim_time'),
                "approx_sync": 'true',
                "queue_size": 10,
                "Mem/IncrementalMemory": ConditionalText("true", "false", LaunchConfiguration('localization')).perform(context),
            }],
            remappings=[
                ("rgb/image", ConditionalText([LaunchConfiguration('rgb_topic'), '_relay'], LaunchConfiguration('rgb_topic'), LaunchConfiguration('compressed')).perform(context)),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                ("odom", LaunchConfiguration('odom_topic')), # 订阅 EKF 发布的里程计
                ("map", LaunchConfiguration('map_topic'))],
            namespace=ns),

        # 3. 可选：视觉里程计 (如果 EKF 已经提供了可靠里程计，此部分可设 visual_odometry 为 false)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', name="rgbd_odometry",
            condition=IfCondition(LaunchConfiguration('visual_odometry')),
            arguments=[
                '--ros-args', 
                '--log-level', 'rtabmap:=error'  # 设置 RTAB-Map 日志级别为 error
            ],
            parameters=[{
                "frame_id": LaunchConfiguration('frame_id'),
                "publish_tf": 'false', # 强制设为 False，由 EKF 发布 odom -> base
                "approx_sync": 'true',
            }],
            remappings=[
                ("rgb/image", ConditionalText([LaunchConfiguration('rgb_topic'), '_relay'], LaunchConfiguration('rgb_topic'), LaunchConfiguration('compressed')).perform(context)),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic'))],
            namespace=ns),
    ]

def generate_launch_description():
    return LaunchDescription([
        # 基础参数
        DeclareLaunchArgument('namespace',      default_value='rtabmap'),
        DeclareLaunchArgument('use_sim_time',   default_value='false'),
        DeclareLaunchArgument('localization',   default_value='false'),
        
        # TF 与 帧 ID
        DeclareLaunchArgument('frame_id',       default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame_id',  default_value='odom'),
        DeclareLaunchArgument('map_frame_id',   default_value='map'),
        DeclareLaunchArgument('publish_tf_map', default_value='true'), # 发布 map->odom
        
        # 话题设置
        DeclareLaunchArgument('rgb_topic',      default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('odom_topic',     default_value='/ekf/odom'), # EKF 输出的里程计话题
        DeclareLaunchArgument('map_topic',      default_value='/map'),
        
        # 功能开关
        DeclareLaunchArgument('depth',           default_value='false', description='单目模式通常关闭深度'),
        DeclareLaunchArgument('compressed',      default_value='false',  description='支持压缩图像输入'),
        DeclareLaunchArgument('rgb_image_transport', default_value='compressed'),
        DeclareLaunchArgument('visual_odometry', default_value='false', description='若由外部EKF提供里程计，则设为false'),

        OpaqueFunction(function=launch_setup)
    ])