import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import zenoh
import json
import time
import copy

class ZenohToLaserScan(Node):
    def __init__(self):
        super().__init__('zenoh_to_laserscan_bridge')
        
        # 1. 声明 ROS 2 参数
        self.declare_parameter('zenoh_topic', 'rt/scan')
        self.declare_parameter('ros_topic', '/seg/scan')
        self.declare_parameter('ros_topic_local', '/seg/scan/local')

        self.declare_parameter('frame_id', 'base_footprint')

        zenoh_topic = self.get_parameter('zenoh_topic').get_parameter_value().string_value
        ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        ros_topic_local = self.get_parameter('ros_topic_local').get_parameter_value().string_value

        self.target_frame = self.get_parameter('frame_id').get_parameter_value().string_value

        # 2. 创建 ROS 2 发布者
        self.publisher_ = self.create_publisher(LaserScan, ros_topic, 10)
        self.publisher_local = self.create_publisher(LaserScan, ros_topic_local, 10)

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

        self.latest_scan = None
        self.latest_scan_local = None

        # 10Hz
        self.timer = self.create_timer(1.0 / 10.0, self.publish_latest_scan)

        self.get_logger().info('✅ Zenoh -> ROS2 LaserScan 桥接节点已启动')

    def publish_latest_scan(self):
        if self.latest_scan :
                # 用当前 ROS 时间刷新时间戳（很重要）
            # self.latest_scan.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.latest_scan)

        if self.latest_scan_local :
                # 用当前 ROS 时间刷新时间戳（很重要）
            # self.latest_scan_local.header.stamp = self.get_clock().now().to_msg()
            self.publisher_local.publish(self.latest_scan_local)
        
        

    def zenoh_callback(self, sample):
        try:
            # 解析 JSON 负载
            payload_bytes = sample.payload.to_bytes()
            data = json.loads(payload_bytes.decode("utf-8"))
            self.get_logger().debug(
                f'ZenohToLaserScan received: {data}'
            )
            # 构造 LaserScan 消息
            scan_msg = LaserScan()
            
            # 使用当前 ROS 时间或 JSON 中的时间戳
            # 核心修复：获取当前节点的时钟类型
            node_clock = self.get_clock()
            t = Time(seconds=data['stamp'],clock_type=node_clock.clock_type)
            scan_msg.header.stamp = t.to_msg()
            scan_msg.header.frame_id = self.target_frame
            
            # 填充雷达几何参数
            scan_msg.angle_min = float(data['angle_min'])
            scan_msg.angle_max = float(data['angle_max'])
            scan_msg.angle_increment = float(data['angle_increment'])
            
            # 这里的 range 限制应与推理端一致
            scan_msg.range_min = float(data['range_min'])
            scan_msg.range_max = float(data['range_max'])
            
            scan_local = copy.deepcopy(scan_msg)

            # 转换 ranges 列表 (处理 JSON 序列化后的数值)
            # scan_msg.ranges = [float(r) for r in data['ranges']]
            processed_ranges = []
            for r in data['ranges']:
                if r is None:  # 处理 JSON 中的 null
                    processed_ranges.append(float('inf'))
                elif r > scan_msg.range_max: # 显式处理超量程
                    processed_ranges.append(float('inf'))
                elif r < scan_msg.range_min: # 处理过近的无效数据
                    processed_ranges.append(float('nan')) # 或者设为 inf 取决于你想不想清空近处
                else:
                    processed_ranges.append(float(r))

            scan_msg.ranges = processed_ranges
            # scan_msg.ranges = [float(r) if scan_msg.range_max > r else scan_msg.range_max+1.0 for r in data['ranges']]
            
            # 发布到 ROS 2
            # self.publisher_.publish(scan_msg)
            self.latest_scan = scan_msg
            # self.latest_scan_local = scan_local

            now = node_clock.now()
            diff = now - t
            latency_sec = diff.nanoseconds / 1e9

            if latency_sec > 2.0: # 超过 1s 打印警告
                self.get_logger().warn(f"数据积压！延迟高达: {latency_sec:.3f} s, {self.latest_scan}")
            
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