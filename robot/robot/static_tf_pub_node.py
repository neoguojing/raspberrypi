import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math

class JsonTfPublisher(Node):
    def __init__(self):
        super().__init__('json_static_tf_publisher')
        self.tf_publisher = StaticTransformBroadcaster(self)
        
        # 声明参数，默认值为空字符串
        self.declare_parameter('tf_data', '')
        json_str = self.get_parameter('tf_data').get_parameter_value().string_value

        if not json_str:
            self.get_logger().error("未接收到 tf_data 参数！")
            return

        try:
            frames = json.loads(json_str)
            self.publish_transforms(frames)
        except Exception as e:
            self.get_logger().error(f"解析参数失败: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """将弧度制欧拉角转换为四元数 (ROS 2 要求的格式)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return {
            'x': sr * cp * cy - cr * sp * sy,
            'y': cr * sp * cy + sr * cp * sy,
            'z': cr * cp * sy - sr * sp * cy,
            'w': cr * cp * cy + sr * sp * sy
        }

    def publish_transforms(self, frames):
        static_transforms = []

        for frame_id, info in frames.items():
            if info['parent'] in ['odom', 'map']:
                self.get_logger().warn(f"跳过动态坐标系 {frame_id} 的静态发布，这应由 EKF 处理。")
                continue
            
            t = TransformStamped()
            
            # 时间戳和坐标系名称
            t.header.stamp.sec = 0
            t.header.stamp.nanosec = 0
            t.header.frame_id = info['parent']
            t.child_frame_id = frame_id

            # 设置平移 (Translation)
            t.transform.translation.x = float(info['offset']['x'])
            t.transform.translation.y = float(info['offset']['y'])
            t.transform.translation.z = float(info['offset']['z'])

            # 设置旋转 (Rotation) - 内部转换为四元数
            q = self.euler_to_quaternion(
                float(info['offset']['roll']),
                float(info['offset']['pitch']),
                float(info['offset']['yaw'])
            )
            t.transform.rotation.x = q['x']
            t.transform.rotation.y = q['y']
            t.transform.rotation.z = q['z']
            t.transform.rotation.w = q['w']

            static_transforms.append(t)
            self.get_logger().info(f"已加载 TF: {info['parent']} -> {frame_id}")

        # 一次性发布所有静态变换
        self.tf_publisher.sendTransform(static_transforms)

def main():
    rclpy.init()
    node = JsonTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()