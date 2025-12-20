import RPi.GPIO as GPIO
import time
# GPIO 初始化
# ---------------------------
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# ---------------------------
# 舵机控制类
# ---------------------------
class Steering:
    def __init__(self, pin, min_angle=-45, max_angle=45):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle

        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)  # 50Hz
        self.pwm.start(7.5)  # 中位
        time.sleep(0.2)

    def set_angle(self, angle):
        angle = max(min(angle, self.max_angle), self.min_angle)
        duty = 7.5 + (angle / 18.0)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.1)
        self.pwm.ChangeDutyCycle(0)

    def center(self):
        self.set_angle(0)

    def cleanup(self):
        self.pwm.stop()


# ---------------------------
# 电机控制类 (支持 PWM)
# ---------------------------
class Motor:
    def __init__(self, forward_pin, backward_pin, pwm=False, freq=100):
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        self.use_pwm = pwm

        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.backward_pin, GPIO.OUT)

        if pwm:
            self.pwm_forward = GPIO.PWM(self.forward_pin, freq)
            self.pwm_backward = GPIO.PWM(self.backward_pin, freq)
            self.pwm_forward.start(0)
            self.pwm_backward.start(0)
        else:
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.LOW)

    def forward(self, speed=100):
        """speed: 0-100"""
        if self.use_pwm:
            self.pwm_forward.ChangeDutyCycle(speed)
            self.pwm_backward.ChangeDutyCycle(0)
        else:
            GPIO.output(self.forward_pin, GPIO.HIGH)
            GPIO.output(self.backward_pin, GPIO.LOW)

    def backward(self, speed=100):
        if self.use_pwm:
            self.pwm_forward.ChangeDutyCycle(0)
            self.pwm_backward.ChangeDutyCycle(speed)
        else:
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.HIGH)

    def stop(self):
        if self.use_pwm:
            self.pwm_forward.ChangeDutyCycle(0)
            self.pwm_backward.ChangeDutyCycle(0)
        else:
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.LOW)

    def cleanup(self):
        if self.use_pwm:
            self.pwm_forward.stop()
            self.pwm_backward.stop()
        self.stop()


# ---------------------------
# 四轮小车控制类
# ---------------------------
class FourWheelCar:
    def __init__(self, steering_pin, motor_pins_list, pwm=False):
        """
        motor_pins_list: [(前左F,前左B), (前右F,前右B), (后左F,后左B), (后右F,后右B)]
        """
        self.steering = Steering(steering_pin)
        self.motors = [Motor(*pins, pwm=pwm) for pins in motor_pins_list]

    def forward(self, speed=100):
        for m in self.motors:
            m.forward(speed)

    def backward(self, speed=100):
        for m in self.motors:
            m.backward(speed)

    def stop(self):
        for m in self.motors:
            m.stop()

    def turn_left(self, angle=30):
        self.steering.set_angle(-abs(angle))

    def turn_right(self, angle=30):
        self.steering.set_angle(abs(angle))

    def center_steering(self):
        self.steering.center()

    def cleanup(self):
        self.stop()
        self.steering.cleanup()
        for m in self.motors:
            m.cleanup()
        GPIO.cleanup()


# ---------------------------
# 测试示例
# ---------------------------
if __name__ == "__main__":
    # 舵机引脚
    steering_pin = 12

    # 四电机引脚 BOARD 排针: [(前左F,前左B), (前右F,前右B), (后左F,后左B), (后右F,后右B)]
    motor_pins = [
        (16, 18),  # 前左
        (22, 24),  # 前右
        (32, 36),  # 后左
        (33, 35)   # 后右
    ]

    car = FourWheelCar(steering_pin, motor_pins, pwm=True)

    try:
        print("▶ 自动测试开始…")

        # 1. 前进
        print("➡ 前进 2 秒")
        car.forward(60)
        time.sleep(2)

        # 2. 后退
        print("⬅ 后退 2 秒")
        car.backward(60)
        time.sleep(2)

        # 3. 左转
        print("↙ 左转 1 秒")
        car.turn_left(30)
        time.sleep(1)
        car.center_steering()

        # 4. 右转
        print("↘ 右转 1 秒")
        car.turn_right(30)
        time.sleep(1)
        car.center_steering()

        # 5. 前进 + 小转向
        print("➡ 前进并右偏 2 秒")
        car.turn_right(15)
        car.forward(60)
        time.sleep(2)
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

