from car.steer import Steering
from car.motor import Motor


# ---------------------------
# 四轮小车控制类
# ---------------------------
class FourWheelCar:
    def __init__(self, steering_pin, motor_pins_list, pwm=True):
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

    def center_steering(self):
        if self.steering:
            self.steering.center()

    def cleanup(self):
        self.stop()
        if self.steering:
            self.steering.cleanup()
        for m in self.motors:
            m.cleanup()


# ---------------------------
# 测试代码（你需要将 BOARD 改为 BCM）
# ---------------------------
if __name__ == "__main__":
    steering_pin = 18   # 示例：BCM18（原 BOARD 12）

    motor_pins = [
        (6,5),  # 前左 BOARD 29 31
        (13, 19),   # 前右 BOARD 33 35
        (16,26),    # 后左 BOARD 37 36
        (20, 21)   # 后右 BOARD 38 40
    ]

    car = FourWheelCar(steering_pin, motor_pins, pwm=True)

    try:
        while True:
            cmd = input("输入命令 w/s/a/d/q: ")
            if cmd == 'w':
                car.forward()
            elif cmd == 's':
                car.backward()
            elif cmd == 'a':
                car.turn_left(10)
            elif cmd == 'd':
                car.turn_right(10)
            elif cmd == 'q':
                car.stop()
                car.center_steering()
            else:
                print("无效命令")
    except KeyboardInterrupt:
        pass
    finally:
        car.cleanup()

