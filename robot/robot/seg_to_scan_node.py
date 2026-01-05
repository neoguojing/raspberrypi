import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import zenoh
import json
import time

class ZenohToLaserScan(Node):
    def __init__(self):
        super().__init__('zenoh_to_laserscan_bridge')
        
        # 1. 声明 ROS 2 参数
        self.declare_parameter('zenoh_topic', 'rt/scan')
        self.declare_parameter('ros_topic', '/seg/scan')
        self.declare_parameter('frame_id', 'base_link')

        zenoh_topic = self.get_parameter('zenoh_topic').get_parameter_value().string_value
        ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('frame_id').get_parameter_value().string_value

        # 2. 创建 ROS 2 发布者
        self.publisher_ = self.create_publisher(LaserScan, ros_topic, 10)

        # 3. 初始化 Zenoh 会话
        self.get_logger().info(f'🔗 正在连接 Zenoh 并订阅: {zenoh_topic}')
        config = zenoh.Config()
        config.insert_json5(
            "connect/endpoints",
            '["tcp/127.0.0.1:7447"]'
        )
        self.session = zenoh.open(config)
        
        # 4. 订阅 Zenoh 话题
        # 使用 lambda 保证回调能访问 self
        self.sub = self.session.declare_subscriber(
            zenoh_topic, 
            self.zenoh_callback
        )
        
        self.get_logger().info('✅ Zenoh -> ROS2 LaserScan 桥接节点已启动')

    def zenoh_callback(self, sample):
        try:
            # 解析 JSON 负载
            payload_bytes = sample.payload.to_bytes()
            data = json.loads(payload_bytes.decode("utf-8"))
            self.get_logger().info(
                f'ZenohToLaserScan received: {data}'
            )
            # 构造 LaserScan 消息
            scan_msg = LaserScan()
            
            # 使用当前 ROS 时间或 JSON 中的时间戳
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "base_link"
            
            # 填充雷达几何参数
            scan_msg.angle_min = float(data['angle_min'])
            scan_msg.angle_max = float(data['angle_max'])
            scan_msg.angle_increment = float(data['angle_increment'])
            
            # 这里的 range 限制应与推理端一致
            scan_msg.range_min = float(data['range_min'])
            scan_msg.range_max = float(data['range_max'])
            
            # 转换 ranges 列表 (处理 JSON 序列化后的数值)
            scan_msg.ranges = [float(r) for r in data['ranges']]
            
            # 发布到 ROS 2
            self.publisher_.publish(scan_msg)
            
        except Exception as e:
            self.get_logger().error(f'解析 Zenoh 数据失败: {e}')

    def destroy_node(self):
        self.session.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZenohToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()