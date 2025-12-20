from robot.car.steer import Steering
from robot.car.motor import Motor
import time

steering_pin = 18   # 示例：BCM18（原 BOARD 12）

motor_pins = [
    (6,5),  # 前左 BOARD 29 31
    (13, 19),   # 前右 BOARD 33 35
    (16,26),    # 后左 BOARD 37 36
    (20, 21)   # 后右 BOARD 38 40
]
# ---------------------------
# 四轮小车控制类
# ---------------------------
class FourWheelCar:
    def __init__(self, steering_pin=18, motor_pins_list=motor_pins):
        self.steering = Steering(steering_pin)
        self.motors = [Motor(*pins) for pins in motor_pins_list]

    def forward(self, speed=85):
        for m in self.motors:
            m.forward_run(speed)

    def backward(self, speed=70):
        for m in self.motors:
            m.backward_run(speed)

    def stop(self):
        for m in self.motors:
            m.soft_stop()

    def turn_left(self, angle=30):
        self.steering.set_angle(-abs(angle))

    def turn_right(self, angle=30):
        self.steering.set_angle(abs(angle))

    def set_angle(self, angle=30):
        self.steering.set_angle(angle)

    def center_steering(self):
        if self.steering:
            self.steering.center()

    def cleanup(self):
        self.stop()
        if self.steering:
            self.steering.cleanup()
        for m in self.motors:
            m.cleanup()
    
    def get_odometry_data(self):
        """模拟获取里程计数据 (位置 x,y, 角度 theta)"""
        # 实际应从编码器或其他传感器读取
        return 0.0, 0.0, 0.0 # x, y, theta
    
    def drive(self):
        try:
            while True:
                cmd = input("输入命令 w/s/a/d/q: ")
                if cmd == 'w':
                    self.forward()
                elif cmd == 's':
                    self.backward()
                elif cmd == 'a':
                    self.turn_left(10)
                elif cmd == 'd':
                    self.turn_right(10)
                elif cmd == 'q':
                    self.stop()
                    self.center_steering()
                else:
                    print("无效命令")
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()


# ---------------------------
# 测试代码（你需要将 BOARD 改为 BCM）
# ---------------------------
if __name__ == "__main__":
        
    car = FourWheelCar()

    try:
        print("▶ 自动测试开始…")

        # 1. 前进
        print("➡ 前进 2 秒")
        car.forward(85)
        time.sleep(5)

        # 2. 后退
        print("⬅ 后退 2 秒")
        car.backward(70)
        time.sleep(5)

        # 3. 左转
        print("↙ 左转 1 秒")
        car.turn_left(30)
        time.sleep(3)
        car.center_steering()

        # 4. 右转
        print("↘ 右转 1 秒")
        car.turn_right(30)
        time.sleep(5)
        car.center_steering()

        # 5. 前进 + 小转向
        print("➡ 前进并右偏 2 秒")
        car.turn_right(15)
        car.forward(80)
        time.sleep(5)
        car.center_steering()

        # 停止
        print("■ 停止")
        car.stop()
        time.sleep(1)

        print("✅ 自动测试结束")

    except KeyboardInterrupt:
        print("\n手动中断")

    finally:
        car.cleanup()
        print("GPIO 已清理")

