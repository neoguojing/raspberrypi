import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import spidev
import time
import numpy as np

class ICM20948Node(Node):
    def __init__(self):
        super().__init__('icm20948_node')
        
        # 1. SPI 初始化
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # 对应总线 0, 片选 0
        self.spi.max_speed_hz = 7000000
        self.spi.mode = 0b11
        
        # 2. 传感器初始化
        self.init_icm20948()
        
        # 3. ROS 2 发布者
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.01, self.timer_callback) # 100Hz
        self.get_logger().info('ICM-20948 SPI Node Started')

    def write_reg(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value])

    def read_regs(self, reg, length):
        data = self.spi.xfer2([reg | 0x80] + [0x00] * length)
        return data[1:]

    def select_bank(self, bank):
        self.write_reg(0x7f, bank << 4)

    def init_icm20948(self):
        self.select_bank(0)
        self.write_reg(0x06, 0x01) # 解除休眠，自动选择时钟源
        self.write_reg(0x07, 0x00) # 启用所有轴
        
        self.select_bank(2)
        self.write_reg(0x01, 0x01) # 陀螺仪量程: ±250dps
        self.write_reg(0x14, 0x01) # 加速度量程: ±2g
        self.select_bank(0)

    def timer_callback(self):
        # 一次性读取 12 字节（Acc X/Y/Z + Gyro X/Y/Z）
        raw_data = self.read_regs(0x2D, 12)
        
        # 解析 16 位补码
        def to_signed(high, low):
            val = (high << 8) | low
            return val - 65536 if val > 32767 else val

        ax, ay, az = to_signed(raw_data[0], raw_data[1]), to_signed(raw_data[2], raw_data[3]), to_signed(raw_data[4], raw_data[5])
        gx, gy, gz = to_signed(raw_data[6], raw_data[7]), to_signed(raw_data[8], raw_data[9]), to_signed(raw_data[10], raw_data[11])

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # 换算系数：
        # Acc ±2g: 16384 LSB/g
        # Gyro ±250dps: 131 LSB/(deg/s)
        g_to_ms2 = 9.80665
        dps_to_rads = np.pi / 180.0

        msg.linear_acceleration.x = (ax / 16384.0) * g_to_ms2
        msg.linear_acceleration.y = (ay / 16384.0) * g_to_ms2
        msg.linear_acceleration.z = (az / 16384.0) * g_to_ms2

        msg.angular_velocity.x = (gx / 131.0) * dps_to_rads
        msg.angular_velocity.y = (gy / 131.0) * dps_to_rads
        msg.angular_velocity.z = (gz / 131.0) * dps_to_rads

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ICM20948Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.spi.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()