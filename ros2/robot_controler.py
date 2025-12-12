import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # 用于接收速度指令
from nav_msgs.msg import Odometry     # 用于发布里程计数据
from sensor_msgs.msg import Image     # 用于发布图像数据
from std_msgs.msg import Header       # 用于 Odometry 和 Image 消息的时间戳/帧ID
from car.car import FourWheelCar
from camera.camera import RpiCamera
# --- ROS 2 控制节点 ---
class SLAMRobotControllerNode(Node):
    def __init__(self):
        # 1. 初始化ROS 2节点
        super().__init__('slam_robot_controller_node')
        self.get_logger().info('SLAM Robot Controller 节点启动...')

        # 2. 实例化硬件控制类
        self.car_controller = FourWheelCar()
        self.camera_driver = RpiCamera()

        # 3. 参数设置 (可选，可以通过yaml文件或命令行传入)
        self.declare_parameter('update_frequency', 30.0)
        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        
        # 4. 订阅器：接收运动指令 (通常来自 /cmd_vel)
        # 收到 geometry_msgs/Twist 消息时，调用 twist_callback
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel', # 标准的速度指令话题
            self.twist_callback,
            10) # 队列大小

        # 5. 发布器：发布里程计 (Odometry)
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom', # 标准的里程计话题
            10)

        # 6. 发布器：发布图像帧 (Image)
        self.image_publisher = self.create_publisher(
            Image,
            'image_raw', # 标准的原始图像话题
            10)
        
        # 7. 定时器：周期性发布里程计和图像帧
        # 以指定的频率运行 timer_callback
        self.timer = self.create_timer(1.0 / self.update_frequency, self.timer_callback)
        self.get_logger().info(f'定时器启动，频率: {self.update_frequency} Hz')


    # --- 回调函数 ---

    def twist_callback(self, msg: Twist):
        """
        接收 geometry_msgs/Twist 消息，并控制小车运动。
        msg.linear.x 是线速度 (m/s)
        msg.angular.z 是角速度 (rad/s)
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 记录接收到的速度指令
        # self.get_logger().info(f'接收到速度: 线性={linear_x:.2f}, 角度={angular_z:.2f}')

        # a. 处理线速度
        if linear_x > 0.01:
            self.car_controller.forward(linear_x)
        elif linear_x < -0.01:
            self.car_controller.backward(abs(linear_x))
        else:
            # 当线速度接近0时，只处理转向或停止
            pass
        
        # b. 处理角速度 (转向)
        if abs(angular_z) > 0.01:
            self.car_controller.set_angle(angular_z)
        
        # c. 停止逻辑 (如果速度都非常小)
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            self.car_controller.stop()


    def timer_callback(self):
        """
        定时器触发，用于周期性地发布 Odometry 和 Image 数据。
        """
        current_time = self.get_clock().now().to_msg()
        
        # 1. 发布 Odometry 数据 
        self.publish_odometry(current_time)

        # 2. 发布 Image 数据 
        self.publish_image(current_time)


    def publish_odometry(self, timestamp):
        """
        从 FourWheelCar 获取里程计数据并发布 ROS 2 Odometry 消息。
        """
        # TODO: 集成您的 FourWheelCar.get_odometry_data()
        x, y, theta = self.car_controller.get_odometry_data()
        
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom' # 全局坐标系
        odom.child_frame_id = 'base_link' # 机器人本体坐标系

        # a. 位置 (假设小车提供的是 x, y, theta)
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        # 将 theta 转换为四元数 (Quaternions)，这是 ROS 2 的标准
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0.0, 0.0, theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # TODO: 速度信息 (可选，但推荐)
        # odom.twist.twist.linear.x = current_linear_speed 
        # odom.twist.twist.angular.z = current_angular_speed

        self.odom_publisher.publish(odom)


    def publish_image(self, timestamp):
        """
        从 RpiCamera 获取图像帧并发布 ROS 2 Image 消息。
        """
        # 1. 从 RpiCamera 获取图像数据
        frame_data = self.camera_driver.get_frame()

        # 2. 转换为 ROS 2 Image 消息 (这通常涉及到OpenCV/cv_bridge)
        # 由于我们没有 cv_bridge 的完整代码，这里仅创建骨架：
        image_msg = Image()
        image_msg.header.stamp = timestamp
        image_msg.header.frame_id = 'camera_link' # 摄像头的坐标系ID

        # TODO: 将 frame_data 转换为 Image 消息的格式
        # image_msg.height = ...
        # image_msg.width = ...
        # image_msg.encoding = 'rgb8' or 'bgr8'
        # image_msg.is_bigendian = 0
        # image_msg.step = ...
        # image_msg.data = frame_data # 图像的实际字节数据

        # self.image_publisher.publish(image_msg) # 取消注释以发布


# --- 主函数 ---
def main(args=None):
    rclpy.init(args=args) # 初始化 ROS 2 上下文
    node = SLAMRobotControllerNode() # 创建节点实例

    try:
        rclpy.spin(node) # 保持节点运行，直到被关闭或中断
    except KeyboardInterrupt:
        node.get_logger().info('节点被中断...')
    finally:
        # 清理资源
        node.car_controller.stop() # 确保小车停止
        node.destroy_node()
        rclpy.shutdown() # 关闭 ROS 2 上下文

if __name__ == '__main__':
    main()