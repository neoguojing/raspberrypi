import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import spidev
import time
import numpy as np

# 模块引脚,树莓派 5 引脚,备注
# VCC,Pin 1 (3.3V),电源
# GND,Pin 6 (GND),地
# SCL,Pin 23 (GPIO 11),SPI SCK (时钟)
# SDA,Pin 19 (GPIO 10),SPI MOSI (数据输出)
# ADO,Pin 21 (GPIO 9),SPI MISO (数据输入)
# NCS,Pin 24 (GPIO 8),SPI CE0 (片选/nCS)
# INT,Pin 7 (GPIO 4),可选：中断引脚，用于数据准备就绪提醒
# FSYNC,悬空 / 不连,帧同步，一般用于相机同步
# ACL,悬空 / 不连,辅助 I2C 时钟 (Auxiliary SCL)
# ADA,悬空 / 不连,辅助 I2C 数据 (Auxiliary SDA)

class ICM20948Node(Node):
    def __init__(self):
        super().__init__('icm20948_node')
        
        # 1. SPI 初始化
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # 对应总线 0, 片选 0
        self.spi.max_speed_hz = 1000000
        # self.spi.mode = 0b11
        self.spi.mode = 0b00

        # 参数设置
        self.declare_parameter('imu_frequency', 100)
        self.camera_frequency = self.get_parameter('imu_frequency').get_parameter_value().double_value
        
        # 2. 传感器初始化
        self.init_icm20948()
        
        # 3. ROS 2 发布者
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(1/self.camera_frequency, self.timer_callback) # 100Hz
        self.get_logger().info('ICM-20948 SPI Node Started')

    def write_reg(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value])

    def read_regs(self, reg, length):
        data = self.spi.xfer2([reg | 0x80] + [0x00] * length)
        return data[1:]

    def select_bank(self, bank):
        self.write_reg(0x7f, bank << 4)

    def init_icm20948(self):
        # --- 1. 硬件存在性检查 ---
        self.select_bank(0)
        whoami = self.read_regs(0x00, 1)[0]
        if whoami != 0xEA:
            self.get_logger().error(f"ICM20948 未找到! 读到: 0x{whoami:02x}, 应为: 0xEA")
            # 实际开发中可以抛出异常
            return False
        else:
            self.get_logger().info("ICM20948 SPI 通讯正常 (WHO_AM_I: 0xEA)")

        # --- 2. 软复位与解除休眠 ---
        self.write_reg(0x06, 0x80) # 复位
        time.sleep(0.1)
        self.write_reg(0x06, 0x01) # 唤醒 + 自动选择时钟源
        self.write_reg(0x10, 0x00) # 禁用所有中断引脚输出

        # --- 3. Bank 2: 配置传感器特性 ---
        self.select_bank(2)
        # 配置加速度: ±2g + DLPF(111Hz) -> 0x31 (0b00110001)
        self.write_reg(0x14, 0x31) 
        # 配置陀螺仪: ±250dps + DLPF(119Hz) -> 0x31 (0b00110001)
        self.write_reg(0x01, 0x31) 
        # 设置采样频率分频: 1125/(1+10) ≈ 102Hz
        self.write_reg(0x00, 10) 
        
        # --- 4. 回到 Bank 0 准备数据读取 ---
        self.select_bank(0)
        self.get_logger().info("传感器配置完成，进入读取模式")
        return True

    def timer_callback(self):
        # 一次性读取 12 字节（Acc X/Y/Z + Gyro X/Y/Z）
        raw_data = self.read_regs(0x2D, 12)
        if all(d == 0 or d == 255 for d in raw_data):
            # self.get_logger().warn("SPI Read suspect - all zeros/ones")
            return
        
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
        
        # 协方差填充 (根据电机环境适当放大)
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # -1表示不提供姿态

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