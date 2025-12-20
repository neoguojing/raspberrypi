import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot.car.car import FourWheelCar # 确保这个路径正确
from tf_transformations import quaternion_from_euler

# 

class CarDriverNode(Node):
    def __init__(self):
        super().__init__('car_driver_node')
        self.get_logger().info('🚗 小车驱动节点启动...')

        # 实例化硬件控制类
        self.car_controller = FourWheelCar()

        # 参数设置
        self.declare_parameter('odom_frequency', 30.0)
        self.odom_frequency = self.get_parameter('odom_frequency').get_parameter_value().double_value
        
        # 订阅器：接收运动指令 (来自 /cmd_vel)
        self.create_subscription(
            Twist,
            'cmd_vel', 
            self.twist_callback,
            10)

        # 发布器：发布里程计 (Odometry)
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom', 
            10)
        
        # 定时器：周期性发布里程计
        self.timer = self.create_timer(1.0 / self.odom_frequency, self.odom_timer_callback)
        self.get_logger().info(f'Odometry 定时器启动，频率: {self.odom_frequency} Hz')


    def twist_callback(self, msg: Twist):
        """
        接收 geometry_msgs/Twist 消息，并控制小车运动。
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # a. 处理线速度
        if linear_x > 0.01:
            self.car_controller.forward(linear_x)
        elif linear_x < -0.01:
            self.car_controller.backward(abs(linear_x))
        # 允许线速度接近0时，只处理转向
        
        # b. 处理角速度 (转向)
        if abs(angular_z) > 0.01:
            self.car_controller.set_angle(angular_z)
        
        # c. 停止逻辑
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            self.car_controller.stop()

    
    def odom_timer_callback(self):
        """
        定时器触发，用于周期性地发布 Odometry 数据。
        """
        current_time = self.get_clock().now().to_msg()
        self.publish_odometry(current_time)


    def publish_odometry(self, timestamp):
        """
        从 FourWheelCar 获取里程计数据并发布 ROS 2 Odometry 消息。
        """
        # 假设 FourWheelCar.get_odometry_data() 返回 x, y, theta
        x, y, theta = self.car_controller.get_odometry_data()
        
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 位置
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # TODO: 速度信息 (可选)
        # odom.twist.twist.linear.x = self.car_controller.get_linear_speed()
        # odom.twist.twist.angular.z = self.car_controller.get_angular_speed()

        self.odom_publisher.publish(odom)

    def destroy_node(self):
        # 确保小车停止
        self.car_controller.stop() 
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('小车驱动节点被中断...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()