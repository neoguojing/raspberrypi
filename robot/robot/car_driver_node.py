import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot.car.car import FourWheelCar # 确保这个路径正确

class CarDriverNode(Node):
    def __init__(self):
        super().__init__('car_driver_node')
        self.get_logger().info('🚗 小车驱动节点启动...')

        # 实例化硬件控制类
        self.car_controller = FourWheelCar()
        
        # 订阅器：接收运动指令 (来自 /cmd_vel)
        self.create_subscription(
            Twist,
            'cmd_vel', 
            self.twist_callback,
            10)


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