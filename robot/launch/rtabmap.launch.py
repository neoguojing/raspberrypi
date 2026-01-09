import os
from ament_index_python.packages import get_package_share_directory
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
                '--log-level', 'rtabmap:=warn'  # 设置 RTAB-Map 日志级别为 error
            ],
            parameters=[{
                "subscribe_rgb": True,
                "subscribe_depth": LaunchConfiguration('depth'), # 默认设为 false
                "subscribe_stereo": False,
                "subscribe_scan": LaunchConfiguration('subscribe_scan'),
                "frame_id": LaunchConfiguration('frame_id'),
                "map_frame_id": LaunchConfiguration('map_frame_id'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id'), # 对应 EKF 输出的 odom frame
                "publish_tf": LaunchConfiguration('publish_tf_map'),   # 发布 map -> odom
                "use_sim_time": LaunchConfiguration('use_sim_time'),
                "approx_sync": True,
                "sync_queue_size": 50,

                # 2. [地图生成控制 - 解决您的警告]
                "Grid/Sensor": "0",               # 0=从激光雷达创建地图 (消除警告的关键)
                "Grid/FromDepth": "false",        # 明确禁止从深度图生成地图
                "Grid/RangeMax": "5.0",           # 激光雷达的最大有效探测距离
                "Grid/RayTracing": "true",        # 开启射线追踪以清理空旷区域的障碍物
                "Grid/CellSize": "0.05",          # 地图分辨率 5cm

                # 3. [视觉特征提取 - 针对单目增强]
                "Kp/DetectorStrategy": "2",       # 使用 ORB 特征点，对单目环境更鲁棒 (0=SURF, 2=ORB)
                "Kp/MaxFeatures": "1000",         # 增加特征点数量以提高闭环匹配成功率
                "Vis/EstimationType": "2",        # 2=2D->2D (对极几何)，单目运动估计的首选
                "Vis/FeatureType": "2",           # 视觉特征类型保持与 Kp 一致

                # 4. [闭环检测策略]
                "Reg/Strategy": "0",              # 0=仅视觉匹配，1=ICP，2=视觉+ICP
                # 如果您希望闭环时用激光雷达精修位姿，建议设为 "2"
                "Reg/Force3DoF": "true",          # 如果是地面小车，强制 3 自由度 (x, y, yaw) 增加稳定性
                "RGBD/OptimizeMaxError": "3.0",   # 拒绝优化后误差过大的闭环链接

                # 5. [内存与更新管理]
                "RGBD/LinearUpdate": "0.1",       # 机器人移动 0.1m 更新一次地图
                "RGBD/AngularUpdate": "0.1",      # 机器人旋转约 5.7 度更新一次地图
                "Mem/IncrementalMemory": "true",  # true=建图模式，false=纯定位模式
                "Mem/STMSize": "30",              # 短期记忆大小，单目模式下建议略微调大

                # 6. [单目尺度恢复补丁]
                "Mem/StereoFromMotion": "true",   # 核心参数：利用 EKF 里程计产生的位移来推算单目特征点的深度
            }],
            remappings=[
                ("rgb/image", ConditionalText([LaunchConfiguration('rgb_topic'), '_relay'], LaunchConfiguration('rgb_topic'), LaunchConfiguration('compressed')).perform(context)),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                ("odom", LaunchConfiguration('odom_topic')), # 订阅 EKF 发布的里程计
                ("map", LaunchConfiguration('map_topic')),
                ("scan", LaunchConfiguration('scan_topic')),
            ],
                
            namespace=ns),

        # 3. 可选：视觉里程计 (如果 EKF 已经提供了可靠里程计，此部分可设 visual_odometry 为 false)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', name="rgbd_odometry",
            condition=IfCondition(LaunchConfiguration('visual_odometry')),
            arguments=[
                '--ros-args', 
                '--log-level', 'rtabmap:=warn'  # 设置 RTAB-Map 日志级别为 error
            ],
            parameters=[{
                "frame_id": LaunchConfiguration('frame_id'),
                "publish_tf": False, # 强制设为 False，由 EKF 发布 odom -> base
                "approx_sync": True,
            }],
            remappings=[
                ("rgb/image", ConditionalText([LaunchConfiguration('rgb_topic'), '_relay'], LaunchConfiguration('rgb_topic'), LaunchConfiguration('compressed')).perform(context)),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic'))],
            namespace=ns),
    ]

def generate_launch_description():
        # 设置您的包名和配置文件路径
    pkg_name = 'robot' # 替换为您的实际包名
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')

    return LaunchDescription([
        # 基础参数
        DeclareLaunchArgument('namespace',      default_value='rtabmap'),
        DeclareLaunchArgument('use_sim_time',   default_value='false'),
        DeclareLaunchArgument('localization',   default_value='false'),
        DeclareLaunchArgument('subscribe_scan', default_value='true',       description=''),
        DeclareLaunchArgument('scan_topic',     default_value='/seg/scan',       description=''),
        
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
        DeclareLaunchArgument('scan_topic',      default_value='/seg/scan',       description=''),
        
        # 功能开关
        DeclareLaunchArgument('depth',           default_value='false', description='单目模式通常关闭深度'),
        DeclareLaunchArgument('compressed',      default_value='false',  description='支持压缩图像输入'),
        DeclareLaunchArgument('rgb_image_transport', default_value='compressed'),
        DeclareLaunchArgument('visual_odometry', default_value='false', description='若由外部EKF提供里程计，则设为false'),

        OpaqueFunction(function=launch_setup)
    ])